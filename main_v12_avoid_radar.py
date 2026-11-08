# Xe điều khiển qua web (WebSocket): ESP32-S3 + L298N + 5 mắt TCRT5000 + Servo SG90 + HC-SR04
# Bản v12 (Hướng A: Radar quét sườn thông minh + Khôi phục cua rẽ mượt mà nguyên bản):
#   - Khôi phục thuật toán dò line và rẽ cua mượt mà, bén ngọt như ban đầu (loại bỏ block giật góc vuông)
#   - HƯỚNG A: Tận dụng Servo xoay sang sườn xe (150° hoặc 30°) làm radar quét liên tục khi tiến
#     -> Xe biết chính xác 100% khi nào đã vượt qua khỏi vật cản để bẻ lái về line (không đoán mò theo delay!)
#   - Đầy đủ tính năng đảo chiều và test bánh xe trực tiếp trên giao diện web

import network
import json
import hashlib
import binascii
from machine import Pin, PWM, time_pulse_us
from time import ticks_ms, ticks_diff, ticks_add, sleep, sleep_us

try:
    import asyncio
except ImportError:
    import uasyncio as asyncio

# ================= CẤU HÌNH =================

# ---- WiFi ----
WIFI_MODE = "AP"             # "AP": ESP32 tự phát WiFi | "STA": vào WiFi nhà bạn
AP_SSID = "XeESP32"
AP_PASS = "12345678"         # tối thiểu 8 ký tự
STA_SSID = "TEN_WIFI_NHA"
STA_PASS = "MAT_KHAU_WIFI"

# ---- Driver L298N ----
ENA = PWM(Pin(4), freq=1000, duty_u16=0)    # PWM động cơ trái
IN1 = Pin(5, Pin.OUT)                       # động cơ trái
IN2 = Pin(6, Pin.OUT)
IN3 = Pin(7, Pin.OUT)                       # động cơ phải
IN4 = Pin(15, Pin.OUT)
ENB = PWM(Pin(16), freq=1000, duty_u16=0)   # PWM động cơ phải

# ---- 5 mắt dò line TCRT5000 (OUT1 -> OUT5) ----
SENSOR_PINS = [46, 3, 8, 18, 17]
WEIGHTS = [-2.0, -1.0, 0.0, 1.0, 2.0]
sensors = [Pin(p, Pin.IN) for p in SENSOR_PINS]
LINE_LEVEL = 0               # vạch đen = 0, nền trắng = 1

# ---- Servo SG90 & Cảm biến siêu âm HC-SR04 ----
SERVO_PIN = 10               # Chân cam Servo (PWM 50Hz)
TRIG_PIN = 11                # Chân TRIG của cảm biến siêu âm
ECHO_PIN = 12                # Chân ECHO của cảm biến siêu âm

servo_pwm = PWM(Pin(SERVO_PIN), freq=50, duty_u16=0)
trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)
trig.value(0)

AVOID_DIST = 20              # Khoảng cách phát hiện vật cản phía trước (cm)

# ---- Chiều quay động cơ ----
LEFT_REVERSE = True          # True theo chuẩn đấu dây của xe
RIGHT_REVERSE = True         # True theo chuẩn đấu dây của xe

# ---- Tốc độ ----
MIN_PWM = 56
MAX_PWM = 145
LEFT_TRIM = 1.0
RIGHT_TRIM = 1.05

# ---- Cú đá khởi động nhạy bén (thắng ma sát bánh xe trên sàn) ----
KICK_MS = 75
KICK_PWM = 190

# ---- Tốc độ né vật cản & tìm line ----
SEARCH_SPEED = 115           # Tốc độ quay tìm line bén ngọt như bản gốc
BYPASS_TURN_SPEED = 85       # Tốc độ quay bẻ lái khi né
BYPASS_DRIVE_SPEED = 72      # Tốc độ tiến sườn khi né
LINE_SEARCH_SPEED = 66       # Tốc độ tiến tìm line đều tay

LOST_TIMEOUT_MS = 1500
FAILSAFE_MS = 400

AUTO_START = False
AUTO_START_DELAY_S = 3

# ================= TRẠNG THÁI =================

state = {
    "mode": "stop",          # "stop" | "manual" | "line" | "avoid"
    "sub_state": "",         # "", "bypassing", "clearing_obs", "finding_line", "line_recovered"
    "left": 0,
    "right": 0,
    "speed": 40,             # % (10..100)
    "kp": 30.0,
    "kd": 20.0,
    "err": 0.0,
    "last_error": 0.0,
    "armed": False,
    "abnormal_since": None,
    "last_cmd": ticks_ms(),
    "dist": 99.0,            # cm
    "servo_deg": 90,         # độ
}
server = None
current_ws = None
WS_GUID = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11"

_last_pwm = [0, 0]
_kick_end = [0, 0]


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ================= ĐIỀU KHIỂN SERVO & SIÊU ÂM =================

def set_servo_angle(deg):
    deg = clamp(deg, 0, 180)
    state["servo_deg"] = deg
    us = 500 + (deg / 180.0) * 2000
    duty = int((us / 20000.0) * 65535)
    servo_pwm.duty_u16(duty)


def read_distance():
    trig.value(0)
    sleep_us(2)
    trig.value(1)
    sleep_us(10)
    trig.value(0)
    try:
        pulse = time_pulse_us(echo, 1, 30000)
        if pulse > 0:
            d = pulse / 58.2
            if 2.0 <= d <= 400.0:
                return round(d, 1)
    except Exception:
        pass
    return None


# ================= ĐIỀU KHIỂN ĐỘNG CƠ =================

def speed_to_pwm(pct):
    if pct <= 0:
        return 0
    return MIN_PWM + (MAX_PWM - MIN_PWM) * pct // 100


def kick(i, v):
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
    ENA.duty_u16(abs(left) * 257)

    IN3.value(1 if right > 0 else 0)
    IN4.value(1 if right < 0 else 0)
    ENB.duty_u16(abs(right) * 257)


def manual_speeds(d, pct):
    """
    Điều khiển tay cho xe 4 bánh 4WD:
    - Rẽ / Xoay tại chỗ (Tank Turn):
      + Rẽ trái (L, SL): Bánh TRÁI LÙI (-turn_s), Bánh PHẢI TIẾN (+turn_s) -> xe xoay vòng trái ngay tại chỗ.
      + Rẽ phải (R, SR): Bánh TRÁI TIẾN (+turn_s), Bánh PHẢI LÙI (-turn_s) -> xe xoay vòng phải ngay tại chỗ.
      + turn_s đảm bảo tối thiểu 90 PWM để thắng ma sát trượt của 4 bánh trên mặt sàn.
    - Cua lướt góc chéo (FL, FR, BL, BR): Vi sai bánh trong chậm, bánh ngoài nhanh để bo cua mượt khi đang chạy.
    """
    s = speed_to_pwm(pct)
    turn_s = max(s, 90)
    s_arc = speed_to_pwm(int(pct * 0.35))

    if d == "F":
        return s, s
    if d == "B":
        return -s, -s

    # Rẽ / Xoay tại chỗ (Tank Turn) đúng nguyên lý: 1 bên lùi, 1 bên tiến
    if d in ("L", "SL"):
        return -turn_s, turn_s
    if d in ("R", "SR"):
        return turn_s, -turn_s

    # Cua lướt tiến vi sai góc chéo
    if d == "FL":
        return s_arc, s
    if d == "FR":
        return s, s_arc

    # Cua lùi vi sai góc chéo
    if d == "BL":
        return -s_arc, -s
    if d == "BR":
        return -s, -s_arc

    return 0, 0


def read_raw_sensors():
    return [1 if s.value() == LINE_LEVEL else 0 for s in sensors]


def read_line():
    total = 0
    count = 0
    for s, w in zip(sensors, WEIGHTS):
        if s.value() == LINE_LEVEL:
            total += w
            count += 1
    if count == 0:
        return None, 0
    return total / count, count


# ================= HƯỚNG A: NÉ VẬT CẢN DÙNG SERVO QUÉT SƯỜN THÔNG MINH =================

async def bypass_obstacle_and_find_line(turn_dir):
    """
    HƯỚNG A: Dùng Servo làm radar quét sườn xe liên tục khi tiến.
    Xe biết chính xác 100% khi nào đã vượt qua khỏi vật cản để bẻ lái về line!
    """
    state["sub_state"] = "bypassing"

    turn_spd = BYPASS_TURN_SPEED      # 85
    fwd_spd = BYPASS_DRIVE_SPEED      # 72
    search_spd = LINE_SEARCH_SPEED    # 66

    # 1. Phanh dừng xe trước vật cản
    drive(0, 0)
    await asyncio.sleep_ms(150)

    # 2. Lùi nhẹ một khoảng ngắn tạo không gian bẻ lái
    drive(-fwd_spd, -fwd_spd)
    await asyncio.sleep_ms(200)
    drive(0, 0)
    await asyncio.sleep_ms(100)

    # 3. Bẻ lái né ra khỏi vạch (~40 độ)
    if turn_dir == "R":
        drive(turn_spd, -turn_spd)      # rẽ phải ~40 độ
    else:
        drive(-turn_spd, turn_spd)      # rẽ trái ~40 độ
    await asyncio.sleep_ms(230)
    drive(0, 0)
    await asyncio.sleep_ms(100)

    # 4. HƯỚNG A: QUAY SERVO SANG SƯỜN ĐỂ "CANH" VẬT CẢN (RADAR SƯỜN)
    # Né sang phải -> vật cản nằm bên TRÁI xe -> Servo quay 150°
    # Né sang trái -> vật cản nằm bên PHẢI xe -> Servo quay 30°
    side_angle = 150 if turn_dir == "R" else 30
    set_servo_angle(side_angle)
    await asyncio.sleep_ms(220)

    # 5. TIẾN SƯỜN VÀ ĐỌC KHOẢNG CÁCH SƯỜN LIÊN TỤC:
    # Xe vừa tiến vừa kiểm tra vật cản bên cạnh. Khi khoảng cách hổng ra (>32cm)
    # tức là xe ĐÃ CHẠY QUA HẾT VẬT CẢN!
    state["sub_state"] = "clearing_obs"
    drive(fwd_spd, fwd_spd)
    start_t = ticks_ms()
    clear_count = 0

    while ticks_diff(ticks_ms(), start_t) < 1800:
        d = read_distance()
        if d is not None:
            state["dist"] = d
            if d > 32.0:
                clear_count += 1
                if clear_count >= 2:    # xác nhận đã hết vật cản
                    break
            else:
                clear_count = 0
        else:
            clear_count += 1
            if clear_count >= 3:
                break
        await asyncio.sleep_ms(25)

    # Nhích thêm một nhịp ngắn (120ms) để bánh sau cũng vượt qua hẳn góc vật cản
    await asyncio.sleep_ms(120)
    drive(0, 0)
    await asyncio.sleep_ms(100)

    # 6. Đưa Servo về nhìn thẳng (90°)
    set_servo_angle(90)
    await asyncio.sleep_ms(180)

    # 7. Bẻ ngoặt lái hướng đầu xe xiên về lại trục line
    if turn_dir == "R":
        drive(-turn_spd, turn_spd)      # bẻ ngoặt trái
    else:
        drive(turn_spd, -turn_spd)      # bẻ ngoặt phải
    await asyncio.sleep_ms(300)
    drive(0, 0)
    await asyncio.sleep_ms(120)

    # 8. Bò chậm rãi tìm line (tối đa 750ms, chạm vạch là phanh ngay)
    state["sub_state"] = "finding_line"
    found = False
    start_t = ticks_ms()

    drive(search_spd, search_spd)
    while ticks_diff(ticks_ms(), start_t) < 750:
        err, count = read_line()
        if count > 0:
            drive(0, 0)                  # PHANH DỪNG NGAY KHI CHẠM VẠCH
            found = True
            break
        await asyncio.sleep_ms(10)

    drive(0, 0)
    await asyncio.sleep_ms(100)

    # 9. Nếu sau 750ms chưa chạm, nhích xoay tại chỗ từng nhịp để quét vạch
    if not found:
        start_t = ticks_ms()
        while ticks_diff(ticks_ms(), start_t) < 1200:
            if turn_dir == "R":
                drive(-turn_spd, turn_spd)
            else:
                drive(turn_spd, -turn_spd)
            await asyncio.sleep_ms(40)
            drive(0, 0)
            await asyncio.sleep_ms(25)

            err, count = read_line()
            if count > 0:
                found = True
                break
        drive(0, 0)

    # 10. CÂN CHỈNH ĐẦU XE THẲNG THEO CHIỀU ĐƯỜNG LINE BAN ĐẦU:
    if found:
        state["sub_state"] = "line_recovered"
        if turn_dir == "R":
            drive(turn_spd, -turn_spd)   # rẽ nhẹ phải để thẳng hàng vạch
        else:
            drive(-turn_spd, turn_spd)   # rẽ nhẹ trái để thẳng hàng vạch
        await asyncio.sleep_ms(140)
        drive(0, 0)
        await asyncio.sleep_ms(120)

        state["armed"] = True
        state["abnormal_since"] = None
        err, count = read_line()
        if err is not None:
            state["last_error"] = err
    else:
        state["sub_state"] = "line_lost"

    set_servo_angle(90)
    await asyncio.sleep_ms(150)
    return found


# ================= VÒNG LẶP ĐIỀU KHIỂN CHÍNH (KHÔI PHỤC CUA RẼ MƯỢT MÀ) =================

async def control_loop():
    set_servo_angle(90)
    dist_tick = 0

    while True:
        mode = state["mode"]
        dist_tick += 1

        # Cập nhật định kỳ khoảng cách mỗi ~100ms
        if dist_tick % 10 == 0:
            d = read_distance()
            if d is not None:
                state["dist"] = d

        if mode == "line":
            # --- 1. KIỂM TRA NÉ VẬT CẢN TRÊN ĐƯỜNG ĐUA ---
            if state["dist"] <= AVOID_DIST:
                drive(0, 0)
                await asyncio.sleep_ms(150)

                # Quét servo trái / phải tìm hướng thoáng
                set_servo_angle(150)
                await asyncio.sleep_ms(300)
                d_left = read_distance() or 0

                set_servo_angle(30)
                await asyncio.sleep_ms(350)
                d_right = read_distance() or 0

                set_servo_angle(90)
                await asyncio.sleep_ms(200)

                turn_dir = "R" if d_right >= d_left else "L"
                await bypass_obstacle_and_find_line(turn_dir)
                continue

            # --- 2. THUẬT TOÁN BÁM LINE PID MƯỢT MÀ NGUYÊN BẢN (KHÔNG BỊ GIẬT KHỰNG) ---
            error, count = read_line()
            base = speed_to_pwm(state["speed"])
            now = ticks_ms()

            if 1 <= count <= 4:
                state["armed"] = True
                state["abnormal_since"] = None
                state["sub_state"] = ""
                # Thuật toán PID dò line mượt mà nguyên bản của xe:
                correction = state["kp"] * error + state["kd"] * (error - state["last_error"])
                drive(base + correction, base - correction)
                state["last_error"] = error
                state["err"] = error

            elif not state["armed"]:
                drive(0, 0)

            else:
                if state["abnormal_since"] is None:
                    state["abnormal_since"] = now
                if ticks_diff(now, state["abnormal_since"]) > LOST_TIMEOUT_MS:
                    apply_mode("stop")
                elif count == 5:
                    drive(base, base)                       # Ngã tư: tiến thẳng dứt khoát
                elif state["last_error"] > 0:
                    drive(SEARCH_SPEED, -SEARCH_SPEED)      # Rẽ phải tìm line nhanh gọn
                else:
                    drive(-SEARCH_SPEED, SEARCH_SPEED)      # Rẽ trái tìm line nhanh gọn

        elif mode == "avoid":
            # --- CHẾ ĐỘ NÉ VẬT CẢN TỰ HÀNH ---
            if state["servo_deg"] != 90:
                set_servo_angle(90)
                await asyncio.sleep_ms(200)

            d = read_distance()
            if d is not None:
                state["dist"] = d
            else:
                d = state["dist"]

            if d > AVOID_DIST:
                base = speed_to_pwm(state["speed"])
                drive(base, base)
                await asyncio.sleep_ms(40)
            else:
                drive(0, 0)
                await asyncio.sleep_ms(150)

                drive(-BYPASS_DRIVE_SPEED, -BYPASS_DRIVE_SPEED)
                await asyncio.sleep_ms(220)
                drive(0, 0)
                await asyncio.sleep_ms(150)

                set_servo_angle(150)
                await asyncio.sleep_ms(300)
                d_left = read_distance() or 0

                set_servo_angle(30)
                await asyncio.sleep_ms(350)
                d_right = read_distance() or 0

                set_servo_angle(90)
                await asyncio.sleep_ms(200)

                turn_spd = BYPASS_TURN_SPEED
                if d_left > d_right and d_left > AVOID_DIST:
                    drive(-turn_spd, turn_spd)
                    await asyncio.sleep_ms(280)
                elif d_right >= d_left and d_right > AVOID_DIST:
                    drive(turn_spd, -turn_spd)
                    await asyncio.sleep_ms(280)
                else:
                    drive(turn_spd, -turn_spd)
                    await asyncio.sleep_ms(550)

                drive(0, 0)
                await asyncio.sleep_ms(80)

        elif mode == "manual":
            # --- ĐIỀU KHIỂN TAY ---
            if ticks_diff(ticks_ms(), state["last_cmd"]) > FAILSAFE_MS:
                drive(0, 0)
            else:
                drive(state["left"], state["right"])

        else:
            drive(0, 0)

        await asyncio.sleep_ms(10)


# ================= LỆNH TỪ WEB =================

async def run_test_motor(side):
    spd = max(speed_to_pwm(state["speed"]), 130)
    if side == "L":
        drive(spd, 0)
    elif side == "R":
        drive(0, spd)
    await asyncio.sleep_ms(1500)
    drive(0, 0)


def apply_cmd(d, pct):
    if state["mode"] == "manual":
        pct = clamp(pct, 0, 100)
        state["speed"] = pct
        state["left"], state["right"] = manual_speeds(d, pct)
        state["last_cmd"] = ticks_ms()


def apply_mode(m):
    if m in ("manual", "line", "avoid", "stop"):
        state["mode"] = m
        state["sub_state"] = ""
        state["left"] = 0
        state["right"] = 0
        state["last_error"] = 0.0
        state["armed"] = False
        state["abnormal_since"] = None
        state["last_cmd"] = ticks_ms()
        if m in ("stop", "manual"):
            drive(0, 0)
        set_servo_angle(90)


def apply_params(spd, kp, kd):
    state["speed"] = clamp(int(float(spd)), 0, 100)
    state["kp"] = clamp(float(kp), 0, 200)
    state["kd"] = clamp(float(kd), 0, 200)


def handle_ws_msg(msg):
    global LEFT_REVERSE, RIGHT_REVERSE
    p = msg.split(",")
    t = p[0]
    if t == "D" and len(p) >= 3:
        apply_cmd(p[1], int(p[2]))
    elif t == "M" and len(p) >= 2:
        apply_mode(p[1])
    elif t == "P" and len(p) >= 4:
        apply_params(p[1], p[2], p[3])
    elif t == "SV" and len(p) >= 2:
        set_servo_angle(int(p[1]))
    elif t == "REV" and len(p) >= 3:
        if p[1] == "L":
            LEFT_REVERSE = (p[2] == "1")
        elif p[1] == "R":
            RIGHT_REVERSE = (p[2] == "1")
    elif t == "TEST" and len(p) >= 2:
        asyncio.create_task(run_test_motor(p[1]))


def status():
    return {
        "mode": state["mode"],
        "sub": state.get("sub_state", ""),
        "line": read_raw_sensors(),
        "err": state["err"],
        "speed": state["speed"],
        "kp": state["kp"],
        "kd": state["kd"],
        "dist": state["dist"],
        "servo": state["servo_deg"],
        "rev_l": 1 if LEFT_REVERSE else 0,
        "rev_r": 1 if RIGHT_REVERSE else 0,
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
        sta.config(pm=network.WLAN.PM_NONE)
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
    try:
        while True:
            writer.write(ws_frame(1, json.dumps(status())))
            await writer.drain()
            await asyncio.sleep_ms(150)
    except Exception:
        pass


async def ws_session(reader, writer):
    global current_ws
    current_ws = writer
    pusher_task = asyncio.create_task(ws_pusher(writer))
    try:
        while True:
            opcode, data = await ws_read_frame(reader)
            if opcode == 8:
                break
            if opcode == 9:
                writer.write(ws_frame(10, data))
                await writer.drain()
            elif opcode == 1:
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
        except Exception:
            INDEX_HTML = b"<h1>Loi doc index.html</h1>"
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
    drive(0, 0)
    if server:
        server.close()
    asyncio.new_event_loop()
