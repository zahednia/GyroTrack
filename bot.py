import requests
import time
import numpy as np
import matplotlib.pyplot as plt
from pyquaternion import Quaternion

# URL برای دریافت داده‌ها از ESP32
url = "http://localhost:3000/0"

# زمان گام (ببین تقریبا با نرخ ارسال ESP32 یکی باشه)
dt = 0.02

# ثابت گرانش (اگر خواستی از Z کم کنی)
g = 9.81

# ---------------- وضعیت اولیه ----------------
# موقعیت و سرعت در دستگاه "زمین"
x_pos, y_pos, z_pos = 0.0, 0.0, 0.0
vx, vy, vz = 0.0, 0.0, 0.0

# مسیر برای رسم
history_x, history_y, history_z = [x_pos], [y_pos], [z_pos]

# وضعیت چرخشی سنسور (کواترنیون)، اول بدون چرخش
orientation = Quaternion()


# تابع برای دریافت داده‌ها از ESP32
def fetch_data_from_esp32():
    try:
        response = requests.get(url, timeout=0.5)
        if response.status_code == 200:
            return response.json()
        else:
            print("خطا در دریافت داده‌ها از ESP32:", response.status_code)
            return None
    except Exception as e:
        print(f"خطا: {e}")
        return None


# تابع برای اصلاح داده‌های شتاب خطی (فعلاً فقط اگر ماژول برعکس شد)
def correct_acceleration(acc_x, acc_y, acc_z, reverse=False):
    if reverse:
        acc_x = -acc_x
        acc_y = -acc_y
        acc_z = -acc_z
    return acc_x, acc_y, acc_z


# به‌روزرسانی وضعیت چرخشی با ژیروسکوپ (انتگرال‌گیری از سرعت زاویه‌ای)
def calculate_orientation(q_current, gyro_x, gyro_y, gyro_z, dt):
    # gyro ها را به بردار تبدیل می‌کنیم (فرض: rad/s یا واحد نسبی ثابت)
    omega = np.array([gyro_x, gyro_y, gyro_z], dtype=float)
    omega_norm = np.linalg.norm(omega)
    if omega_norm == 0:
        return q_current  # اگر چرخشی نداریم، همان قبلی

    angle = omega_norm * dt           # زاویه چرخش در این بازه زمان
    axis = omega / omega_norm         # محور چرخش
    dq = Quaternion(axis=axis, angle=angle)  # چرخش کوچک
    q_new = q_current * dq            # چرخش کلی = قبلی * اضافه شده
    return q_new.normalised


# ---------------- حلقه اصلی ----------------
plt.ion()

while True:
    data = fetch_data_from_esp32()

    if data:
        # استخراج داده‌های شتاب‌سنج و ژیروسکوپ از JSON
        acc_x = float(data["acc_x"])
        acc_y = float(data["acc_y"])
        acc_z = float(data["acc_z"])
        gyro_x = float(data["gyro_x"])
        gyro_y = float(data["gyro_y"])
        gyro_z = float(data["gyro_z"])

        # اگر خواستی برحسب شرایط، معکوس کنی
        reverse_module = False
        acc_x, acc_y, acc_z = correct_acceleration(acc_x, acc_y, acc_z, reverse_module)

        # 1) به‌روزرسانی وضعیت چرخشی سنسور
        orientation = calculate_orientation(orientation, gyro_x, gyro_y, gyro_z, dt)

        # 2) شتاب در دستگاه سنسور
        acc_body = np.array([acc_x, acc_y, acc_z], dtype=float)

        # 3) تبدیل شتاب به دستگاه زمین (اثرات چرخش سنسور حذف می‌شود)
        acc_world = orientation.rotate(acc_body)

        # اگر دوست داری گرانش را حذف کنی (بسته به جهت محور Z)
        # مثلا اگر وقتی سنسور ساکن است تقریباً acc_world[2] ≈ +g است:
        # acc_world[2] -= g

        # 4) انتگرال‌گیری: شتاب -> سرعت -> موقعیت (در دستگاه زمین)
        vx += acc_world[0] * dt
        vy += acc_world[1] * dt
        vz += acc_world[2] * dt

        x_pos += vx * dt
        y_pos += vy * dt
        z_pos += vz * dt

        # ذخیره مسیر برای رسم
        history_x.append(x_pos)
        history_y.append(y_pos)
        history_z.append(z_pos)

        # فقط برای دیباگ
        yaw, pitch, roll = orientation.yaw_pitch_roll
        print(f"Quat: {orientation}")
        print(f"Roll: {roll:.3f}, Pitch: {pitch:.3f}, Yaw: {yaw:.3f}")
        print(f"acc_world: {acc_world}\n")

    # ----------- رسم دو نمودار دقیقاً مثل قبل -----------
    plt.clf()
    ax1 = plt.subplot(121)  # XY
    ax2 = plt.subplot(122)  # ارتفاع Z در زمان

    # نمودار چهار جهته XY
    ax1.axhline(0, color='black', linewidth=1)
    ax1.axvline(0, color='black', linewidth=1)
    ax1.plot(history_x, history_y, label='Path in XY plane', color='blue')
    ax1.plot(0, 0, 'ko')  # مبدا
    ax1.set_aspect('equal', 'box')
    ax1.set_xlabel("X")
    ax1.set_ylabel("Y")
    ax1.set_title("Movement in XY plane")
    ax1.legend()

    # نمودار ارتفاع Z بر حسب زمان (نمونه‌ها)
    ax2.plot(history_z, label='Height (Z)', color='green')
    ax2.set_xlabel("Sample (time)")
    ax2.set_ylabel("Height (Z)")
    ax2.set_title("Height over Time")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()
    plt.pause(0.01)

    time.sleep(dt)
