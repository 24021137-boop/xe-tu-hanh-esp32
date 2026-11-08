# Tìm PWM thấp nhất mà bánh còn QUAY LIÊN TỤC sau khi đã "đá" khởi động
# CÁCH DÙNG: kê xe lên cao cho bánh lơ lửng, mở file trong Thonny rồi bấm F5.
# Không cần lưu vào board.
# Với mỗi bánh: đá khởi động, rồi giảm dần PWM và in giá trị ra Shell.
# Hãy ghi lại giá trị lúc bánh DỪNG QUAY (hoặc chỉ còn rung).

from machine import Pin, PWM
from time import sleep, sleep_ms

ENA = PWM(Pin(4), freq=1000, duty_u16=0)    # PWM bánh trái
IN1 = Pin(5, Pin.OUT)
IN2 = Pin(6, Pin.OUT)
IN3 = Pin(7, Pin.OUT)
IN4 = Pin(15, Pin.OUT)
ENB = PWM(Pin(16), freq=1000, duty_u16=0)   # PWM bánh phải

LEFT = (ENA, IN1, IN2)
RIGHT = (ENB, IN3, IN4)

KICK_PWM = 200      # giống KICK_PWM trong main.py
KICK_MS = 80        # giống KICK_MS trong main.py
START = 140         # bắt đầu giảm từ PWM này
STOP = 40           # giảm tới PWM này
STEP = 5
HOLD_S = 1.5        # mỗi mức giữ bao lâu để bạn quan sát


def set_wheel(w, pwm):
    """Cho 1 bánh quay với PWM 0..255 (chiều quay không quan trọng khi đo)."""
    en, a, b = w
    a.value(1)
    b.value(0)
    en.duty_u16(pwm * 257)


def stop_all():
    for en, a, b in (LEFT, RIGHT):
        en.duty_u16(0)
        a.value(0)
        b.value(0)


def sweep(name, w):
    print("--- Banh", name, ": da khoi dong, giam dan PWM ---")
    set_wheel(w, KICK_PWM)
    sleep_ms(KICK_MS)
    for pwm in range(START, STOP - 1, -STEP):
        print(name, "PWM =", pwm)
        set_wheel(w, pwm)
        sleep(HOLD_S)
    stop_all()
    sleep(1.5)


try:
    sweep("TRAI", LEFT)
    sweep("PHAI", RIGHT)
    print("Xong. Ghi lai PWM luc moi banh DUNG QUAY (hoac chi con rung).")
finally:
    stop_all()
