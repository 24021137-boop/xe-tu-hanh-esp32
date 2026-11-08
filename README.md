# Xe Tự Hành ESP32-S3 (ESP32-S3 Self-Driving Robot Car)

Dự án xe tự hành 4WD sử dụng vi điều khiển **ESP32-S3**, điều khiển qua mạng WiFi nội bộ bằng giao diện Web (WebSocket) và tích hợp chế độ tự động **Dò Line (Line Following)** sử dụng thanh 5 cảm biến hồng ngoại TCRT5000 với thuật toán PID.

---

## 📌 Tính năng chính

- **Phát WiFi (Access Point - AP):** ESP32 tự phát mạng WiFi riêng (`XeESP32` / mật khẩu `12345678`), không phụ thuộc router ngoài.
- **Web UI điều khiển thời gian thực:** Truy cập trực tiếp qua trình duyệt tại địa chỉ `http://192.168.4.1/`:
  - Bàn phím điều khiển thủ công: Tiến, Lùi, Rẽ Trái tại chỗ (Tank Turn), Rẽ Phải tại chỗ.
  - Phím tắt bàn phím: `W A S D` hoặc phím mũi tên `▲ ▼ ◄ ►`, phím `Space` để dừng khẩn cấp.
  - Thanh trượt chỉnh tốc độ động cơ theo % (10% - 100%).
  - Chỉnh tham số thuật toán PID trực tiếp từ web (`KP`, `KD`).
  - Hiển thị trực quan trạng thái 5 mắt cảm biến hồng ngoại theo thời gian thực.
- **Thuật toán dò line PID 5 mắt:**
  - Tự động bám vạch kẻ đen trên nền trắng.
  - Xử lý khi lệch vạch: Tự động xoay tìm lại đường line nếu bị trật vạch (`SEARCH_SPEED`).
  - Tính năng Failsafe: Tự động dừng xe nếu mất line hoặc gặp sự cố quá thời gian quy định.
- **Cơ chế KICK khởi động:** Cấp xung PWM cao trong thời gian ngắn (80ms) khi động cơ bắt đầu quay để triệt tiêu ma sát tĩnh, giúp xe chuyển động nhạy bén ngay cả ở tốc độ thấp.

---

## 🛠 Sơ đồ kết nối phần cứng (Pinout)

### 1. Mạch điều khiển động cơ L298N
> *Lưu ý: Tháo 2 jumper ENA và ENB trên mạch L298N để điều khiển tốc độ bằng PWM từ ESP32.*

| L298N Pin | Chân ESP32-S3 | Chức năng |
| :--- | :--- | :--- |
| **ENA** | **GPIO 4** | Điều khiển tốc độ bánh Trái (PWM 1kHz) |
| **IN1** | **GPIO 5** | Chiều quay bánh Trái |
| **IN2** | **GPIO 6** | Chiều quay bánh Trái |
| **IN3** | **GPIO 7** | Chiều quay bánh Phải |
| **IN4** | **GPIO 15** | Chiều quay bánh Phải |
| **ENB** | **GPIO 16** | Điều khiển tốc độ bánh Phải (PWM 1kHz) |
| **GND** | **GND** | Nối chung mass với ESP32 và Nguồn Pin |

### 2. Cụm 5 cảm biến dò line TCRT5000 (Từ Trái sang Phải)

| Cảm biến | Chân ESP32-S3 | Trọng số PID |
| :--- | :--- | :--- |
| Mắt 1 (Ngoài cùng bên Trái) | **GPIO 46** | -2 |
| Mắt 2 (Trái) | **GPIO 3** | -1 |
| Mắt 3 (Chính giữa) | **GPIO 8** | 0 |
| Mắt 4 (Phải) | **GPIO 18** | +1 |
| Mắt 5 (Ngoài cùng bên Phải) | **GPIO 17** | +2 |

---

## 📂 Cấu trúc thư mục

```text
├── main.py                     # Chương trình chính MicroPython trên ESP32
├── index.html                  # Giao diện điều khiển Web (HTML/CSS/JS)
├── test_toc_do_thap.py         # Script kiểm tra tốc độ tối thiểu của động cơ
├── firmware.bin                # Firmware MicroPython cho ESP32-S3
└── README.md                   # Tài liệu hướng dẫn dự án
```

---

## 🚀 Hướng dẫn nạp code vào ESP32-S3

### Cách 1: Dùng Thonny IDE
1. Mở phần mềm **Thonny IDE**.
2. Chọn Interpreter: **MicroPython (ESP32)** và cổng COM tương ứng của board.
3. Mở file [main.py](main.py) và [index.html](index.html).
4. Lưu cả 2 file trực tiếp vào bộ nhớ của ESP32 (`File -> Save as... -> MicroPython device`).
5. Bấm nút **RST / EN** trên board để khởi động lại.

### Cách 2: Dùng lệnh `mpremote` (Command Line)
```bash
# Nạp file main.py và index.html
python -m mpremote connect <CỔNG_COM> fs cp main.py :main.py
python -m mpremote connect <CỔNG_COM> fs cp index.html :index.html

# Khởi động lại ESP32
python -m mpremote connect <CỔNG_COM> reset
```

---

## 📱 Hướng dẫn sử dụng

1. Bật nguồn xe.
2. Dùng điện thoại hoặc máy tính kết nối vào mạng WiFi:
   - **Tên WiFi (SSID):** `XeESP32`
   - **Mật khẩu:** `12345678`
3. Mở trình duyệt web (Chrome, Safari, Cốc Cốc...) và truy cập: **`http://192.168.4.1/`**
4. Chọn chế độ:
   - **Điều khiển tay:** Nhấn giữ các phím mũi tên hoặc nút bấm trên màn hình để lái xe.
   - **Dò line:** Đặt xe lên vạch kẻ đen và bấm nút "Dò line", xe sẽ tự động bám theo đường kẻ.
   - **Dừng:** Phanh dừng khẩn cấp xe.
