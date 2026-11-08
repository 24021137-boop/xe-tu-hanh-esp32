# Xe điều khiển qua web (WebSocket): ESP32-S3 + L298N + 5 mắt TCRT5000
# Bản v5 (dùng chung index.html của bản v4):
#   - LEFT_REVERSE / RIGHT_REVERSE: đảo chiều từng bánh bằng code, không cần đổi dây
#   - KICK: "đá" khởi động, cấp PWM cao trong thời gian ngắn khi bánh bắt đầu quay,
#           giúp xe phản ứng ngay khi nhấn (không bị rung chờ ở tốc độ thấp)
#   - Thanh tốc độ tính theo % (10..100), quy đổi sang khoảng PWM MIN_PWM..MAX_PWM
#   - LEFT_TRIM / RIGHT_TRIM bù lệch giữa hai bánh
#   - Dò line an toàn: đứng yên tới khi thấy line, mất line quá lâu thì tự dừng
# Cần 2 file trên board: main.py (file này) và index.html (bản v4)
# Cần firmware MicroPython cho ESP32-S3 bản 1.20 trở lên

import network
import json
import hashlib
import binascii
from machine import Pin, PWM
from time import ticks_ms, ticks_diff, ticks_add, sleep

try:
    import asyncio
except ImportError:
    import uasyncio as asyncio

# ================= CẤU HÌNH =================

# ---- WiFi ----
WIFI_MODE = "AP"             # "AP": ESP32 tự phát WiFi | "STA": vào WiFi nhà bạn
AP_SSID = "XeESP32"
AP_PASS = "12345678"         # tối thiểu 8 ký tự
STA_SSID = "TEN_WIFI_NHA"    # chỉ dùng khi WIFI_MODE = "STA"
STA_PASS = "MAT_KHAU_WIFI"

# ---- Driver L298N ----
ENA = PWM(Pin(4), freq=1000, duty_u16=0)    # PWM động cơ trái (đã tháo jumper ENA)
IN1 = Pin(5, Pin.OUT)                       # động cơ trái
IN2 = Pin(6, Pin.OUT)
IN3 = Pin(7, Pin.OUT)                       # động cơ phải
IN4 = Pin(15, Pin.OUT)
ENB = PWM(Pin(16), freq=1000, duty_u16=0)   # PWM động cơ phải (đã tháo jumper ENB)

# ---- 5 mắt dò line, xếp từ TRÁI sang PHẢI (OUT1 -> OUT5) ----
SENSOR_PINS = [46, 3, 8, 18, 17]
WEIGHTS = [-2, -1, 0, 1, 2]
sensors = [Pin(p, Pin.IN) for p in SENSOR_PINS]
LINE_LEVEL = 0               # module của bạn: vạch đen cho mức 0, nền trắng cho mức 1

# ---- Chiều quay động cơ ----
LEFT_REVERSE = True          # True nếu nhấn tiến mà bánh trái quay lùi (đặt False nếu đã đổi dây)
RIGHT_REVERSE = True         # True nếu nhấn tiến mà bánh phải quay lùi (đặt False nếu đã đổi dây)

# ---- Tốc độ ----
MIN_PWM = 50                 # PWM nhỏ nhất để động cơ bắt đầu quay (đo bằng test_dong_co.py,
                             # lấy giá trị của bánh khó quay hơn rồi cộng thêm khoảng 10)
MAX_PWM = 120                # PWM lớn nhất cho phép: hạ xuống nếu xe vẫn nhanh
LEFT_TRIM = 1.0              # hệ số bù bánh trái: giảm (0.9, 0.85...) nếu xe lệch phải
RIGHT_TRIM = 1.0             # hệ số bù bánh phải: giảm nếu xe lệch trái

# ---- Cú "đá" khởi động ----
KICK_MS = 80                 # thời gian đá (ms). Đặt 0 để tắt
KICK_PWM = 200               # PWM trong lúc đá (chỉ áp dụng nếu cao hơn PWM đang chạy)

# ---- Thông số khác ----
SEARCH_SPEED = 120           # tốc độ quay tìm line khi vừa mất line
LOST_TIMEOUT_MS = 1500       # mất line (hoặc cả 5 mắt cùng thấy line) quá lâu thì tự dừng
FAILSAFE_MS = 400            # chế độ tay: quá thời gian này không có lệnh mới thì xe tự dừng

AUTO_START = False           # True: bật nguồn là tự vào chế độ dò line (xe vẫn đứng yên tới khi thấy line)
AUTO_START_DELAY_S = 3

# ================= TRẠNG THÁI =================

state = {
    "mode": "stop",          # "stop" | "manual" | "line"  (khởi động ở chế độ dừng cho an toàn)
    "left": 0,
    "right": 0,
    "speed": 40,             # tốc độ theo % (10..100), dùng cho chế độ tay và làm tốc độ nền khi dò line
    "kp": 30.0,
    "kd": 20.0,
    "err": 0.0,
    "last_error": 0.0,
    "armed": False,          # đã từng thấy line kể từ lúc vào chế độ dò line chưa
    "abnormal_since": None,  # thời điểm bắt đầu mất line / cả 5 mắt cùng thấy line
    "last_cmd": ticks_ms(),
}
server = None
current_ws = None
WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

_last_pwm = [0, 0]           # giá trị PWM lần trước của bánh trái / phải (để nhận biết lúc bắt đầu quay)
_kick_end = [0, 0]           # thời điểm kết thúc cú đá của từng bánh


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ================= ĐIỀU KHIỂN ĐỘNG CƠ =================

def speed_to_pwm(pct):
    """Đổi tốc độ 0..100% thành PWM trong khoảng MIN_PWM..MAX_PWM (0% = dừng)."""
    if pct <= 0:
        return 0
    return MIN_PWM + (MAX_PWM - MIN_PWM) * pct // 100


def kick(i, v):
    """Cú đá khởi động cho bánh i (0 = trái, 1 = phải).
    Khi bánh bắt đầu quay từ đứng yên (hoặc đổi chiều), cấp PWM cao trong KICK_MS."""
    if KICK_MS <= 0:
        return v
    now = ticks_ms()
    last = _last_pwm[i]
    if v != 0 and (last == 0 or (v > 0) != (last > 0)):
        _kick_end[i] = ticks_add(now, KICK_MS)
    _last_pwm[i] = v
    if v != 0 and ticks_diff(_kick_end[i], now) > 0:
        boosted = max(abs(v), KICK_PWM)
        return boosted if v > 0 else -boosted
    return v


def drive(left, right):
    """left, right là PWM trong khoảng -MAX_PWM..MAX_PWM (âm = quay lùi)."""
    left = clamp(int(left * LEFT_TRIM), -MAX_PWM, MAX_PWM)
    right = clamp(int(right * RIGHT_TRIM), -MAX_PWM, MAX_PWM)

    if LEFT_REVERSE:
        left = -left
    if RIGHT_REVERSE:
        right = -right

    left = kick(0, left)
    right = kick(1, right)

    IN1.value(1 if left > 0 else 0)
    IN2.value(1 if left < 0 else 0)
    ENA.duty_u16(abs(left) * 257)       # 255 * 257 = 65535

    IN3.value(1 if right > 0 else 0)
    IN4.value(1 if right < 0 else 0)
    ENB.duty_u16(abs(right) * 257)


def manual_speeds(d, s):
    if d == "F":
        return s, s
    if d == "B":
        return -s, -s
    if d == "L":
        return -s, s        # quay trái tại chỗ
    if d == "R":
        return s, -s        # quay phải tại chỗ
    return 0, 0


def read_line():
    """Trả về (error, số mắt thấy line). error = None nếu không mắt nào thấy line.
    error: -2 (line lệch hẳn trái) ... 0 (giữa) ... +2 (lệch hẳn phải)."""
    total = 0
    count = 0
    for s, w in zip(sensors, WEIGHTS):
        if s.value() == LINE_LEVEL:
            total += w
            count += 1
    if count == 0:
        return None, 0
    return total / count, count


async def control_loop():
    """Vòng điều khiển chính, chạy song song với web server."""
    while True:
        mode = state["mode"]

        if mode == "line":
            error, count = read_line()
            base = speed_to_pwm(state["speed"])
            now = ticks_ms()

            if 1 <= count <= 4:
                # Bình thường: có 1 đến 4 mắt thấy line -> bám line
                state["armed"] = True
                state["abnormal_since"] = None
                correction = state["kp"] * error + state["kd"] * (error - state["last_error"])
                drive(base + correction, base - correction)
                state["last_error"] = error
                state["err"] = error

            elif not state["armed"]:
                # Chưa thấy line lần nào: đứng yên, chờ đặt xe lên vạch
                drive(0, 0)

            else:
                # Đã từng bám line nhưng giờ mất line (0 mắt) hoặc cả 5 mắt cùng thấy line
                if state["abnormal_since"] is None:
                    state["abnormal_since"] = now
                if ticks_diff(now, state["abnormal_since"]) > LOST_TIMEOUT_MS:
                    apply_mode("stop")                      # mất line quá lâu: tự dừng
                elif count == 5:
                    drive(base, base)                       # ngã tư: đi thẳng
                elif state["last_error"] > 0:
                    drive(SEARCH_SPEED, -SEARCH_SPEED)      # quay phải tìm line
                else:
                    drive(-SEARCH_SPEED, SEARCH_SPEED)      # quay trái tìm line

        elif mode == "manual":
            # Failsafe: mất kết nối / thôi nhận lệnh thì dừng
            if ticks_diff(ticks_ms(), state["last_cmd"]) > FAILSAFE_MS:
                drive(0, 0)
            else:
                drive(state["left"], state["right"])

        else:
            drive(0, 0)

        await asyncio.sleep_ms(10)


# ================= LỆNH TỪ WEB =================

def apply_cmd(d, pct):
    if state["mode"] == "manual":
        pct = clamp(pct, 0, 100)
        state["speed"] = pct
        state["left"], state["right"] = manual_speeds(d, speed_to_pwm(pct))
        state["last_cmd"] = ticks_ms()


def apply_mode(m):
    if m in ("manual", "line", "stop"):
        state["mode"] = m
        state["left"] = 0
        state["right"] = 0
        state["last_error"] = 0.0
        state["armed"] = False
        state["abnormal_since"] = None
        state["last_cmd"] = ticks_ms()
        if m != "line":
            drive(0, 0)


def apply_params(spd, kp, kd):
    state["speed"] = clamp(int(float(spd)), 0, 100)
    state["kp"] = clamp(float(kp), 0, 200)
    state["kd"] = clamp(float(kd), 0, 200)


def handle_ws_msg(msg):
    """Định dạng tin nhắn:  D,<F|B|L|R|S>,<tốc độ %>  |  M,<mode>  |  P,<tốc độ %>,<kp>,<kd>"""
    p = msg.split(",")
    t = p[0]
    if t == "D" and len(p) >= 3:
        apply_cmd(p[1], int(p[2]))
    elif t == "M" and len(p) >= 2:
        apply_mode(p[1])
    elif t == "P" and len(p) >= 4:
        apply_params(p[1], p[2], p[3])


def status():
    return {
        "mode": state["mode"],
        "line": [1 if s.value() == LINE_LEVEL else 0 for s in sensors],
        "err": state["err"],
        "speed": state["speed"],
        "kp": state["kp"],
        "kd": state["kd"],
    }


# ================= WIFI =================

def start_ap():
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASS, authmode=network.AUTH_WPA2_PSK)
    while not ap.active():
        sleep(0.1)
    return ap.ifconfig()[0]


def start_sta():
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    try:
        sta.config(pm=network.WLAN.PM_NONE)     # tắt tiết kiệm điện WiFi để giảm trễ
    except Exception:
        pass
    sta.connect(STA_SSID, STA_PASS)
    for _ in range(20):
        if sta.isconnected():
            return sta.ifconfig()[0]
        sleep(1)
    return None


def start_wifi():
    if WIFI_MODE == "STA":
        ip = start_sta()
        if ip:
            return ip
        print("Khong ket noi duoc WiFi nha, chuyen sang che do AP")
    return start_ap()


# ================= WEBSOCKET =================

def ws_accept(key):
    return binascii.b2a_base64(hashlib.sha1(key + WS_GUID).digest()).strip()


def ws_frame(opcode, payload):
    if isinstance(payload, str):
        payload = payload.encode()
    n = len(payload)
    if n < 126:
        head = bytes([0x80 | opcode, n])
    else:
        head = bytes([0x80 | opcode, 126, n >> 8, n & 0xFF])
    return head + payload


async def ws_read_frame(reader):
    hdr = await reader.readexactly(2)
    opcode = hdr[0] & 0x0F
    masked = hdr[1] & 0x80
    length = hdr[1] & 0x7F
    if length == 126:
        length = int.from_bytes(await reader.readexactly(2), "big")
    elif length == 127:
        length = int.from_bytes(await reader.readexactly(8), "big")
    mask = await reader.readexactly(4) if masked else None
    data = await reader.readexactly(length) if length else b""
    if mask:
        data = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
    return opcode, data


async def ws_pusher(writer):
    """Đẩy trạng thái (5 mắt dò line, chế độ...) về trang web mỗi 200 ms."""
    try:
        while True:
            writer.write(ws_frame(1, json.dumps(status())))
            await writer.drain()
            await asyncio.sleep_ms(200)
    except Exception:
        pass


async def ws_session(reader, writer):
    global current_ws
    current_ws = writer
    pusher_task = asyncio.create_task(ws_pusher(writer))
    try:
        while True:
            opcode, data = await ws_read_frame(reader)
            if opcode == 8:       # close frame
                break
            if opcode == 9:       # ping -> pong
                writer.write(ws_frame(10, data))
                await writer.drain()
            elif opcode == 1:     # text frame
                handle_ws_msg(data.decode())
    except Exception:
        pass
    finally:
        pusher_task.cancel()
        if current_ws is writer:
            current_ws = None


# ================= HTTP SERVER =================

INDEX_HTML = None

def get_index_html():
    global INDEX_HTML
    if INDEX_HTML is None:
        try:
            with open("index.html", "r") as f:
                INDEX_HTML = f.read().encode("utf-8")
        except Exception as e:
            INDEX_HTML = b"<h1>Loi doc file index.html</h1>"
    return INDEX_HTML


async def send(writer, body, ctype=b"text/html; charset=utf-8", status_code=b"200 OK"):
    if isinstance(body, str):
        body = body.encode("utf-8")
    writer.write(b"HTTP/1.1 " + status_code + b"\r\n"
                 b"Content-Type: " + ctype + b"\r\n"
                 b"Content-Length: " + str(len(body)).encode() + b"\r\n"
                 b"Connection: close\r\n\r\n" + body)
    await writer.drain()


async def handle(reader, writer):
    try:
        line = await reader.readline()
        if not line:
            return
        parts = line.decode().split()
        if len(parts) < 2:
            return
        method, path = parts[0], parts[1]

        headers = {}
        while True:
            hline = await reader.readline()
            if not hline or hline == b"\r\n":
                break
            hp = hline.decode().split(":", 1)
            if len(hp) == 2:
                headers[hp[0].strip().lower()] = hp[1].strip()

        route = path.split("?")[0]

        if route == "/" or route == "/index.html":
            await send(writer, get_index_html())

        elif route == "/ws":
            ws_key = headers.get("sec-websocket-key", "").encode()
            writer.write(b"HTTP/1.1 101 Switching Protocols\r\n"
                         b"Upgrade: websocket\r\n"
                         b"Connection: Upgrade\r\n"
                         b"Sec-WebSocket-Accept: " + ws_accept(ws_key) + b"\r\n\r\n")
            await writer.drain()
            await ws_session(reader, writer)

        elif route == "/status":
            await send(writer, json.dumps(status()), b"application/json")

        else:
            await send(writer, "Not found", b"text/plain", b"404 Not Found")

    except Exception as e:
        print("Loi:", e)
    finally:
        try:
            writer.close()
            await writer.wait_closed()
        except Exception:
            pass


async def main():
    global server
    ip = start_wifi()
    print("Mo trinh duyet: http://%s/" % ip)
    asyncio.create_task(control_loop())
    server = await asyncio.start_server(handle, "0.0.0.0", 80)
    if AUTO_START:
        await asyncio.sleep(AUTO_START_DELAY_S)
        apply_mode("line")
    while True:
        await asyncio.sleep(1)


try:
    asyncio.run(main())
finally:
    drive(0, 0)                 # dừng động cơ khi bấm Stop / Ctrl+C
    if server:
        server.close()
    asyncio.new_event_loop()
