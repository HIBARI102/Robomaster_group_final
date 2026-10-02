import time
import math
import sys
import json
import os
from robomaster import robot

def normalize_angle(deg):
    return (deg + 180.0) % 360.0 - 180.0

# ตัวแปรเก็บข้อมูลจาก Callback
latest_pos = [0.0, 0.0]
latest_yaw = 0.0

def chassis_pos_handler(sub_info):
    global latest_pos
    if sub_info and len(sub_info) >= 2 and sub_info[0] is not None and sub_info[1] is not None:
        latest_pos = [float(sub_info[0]), float(sub_info[1])]

def chassis_att_handler(sub_info):
    global latest_yaw
    if sub_info and len(sub_info) >= 2 and sub_info[1] is not None:
        latest_yaw = float(sub_info[1])

def move_forward_60cm_test(ep_chassis, target_speed_mps=0.24, calib_factor=1.0):
    """ฟังก์ชันเดินหน้า 60 cm พร้อม Ramp-down 85% (ถอดแบบมาจาก SLAM หลัก)"""
    target_step_m = 0.60
    calibrated_target_m = target_step_m * calib_factor

    start_pos_x = latest_pos[0]
    start_pos_y = latest_pos[1]
    target_yaw = latest_yaw

    Kp, Ki, Kd = 1.2, 0.05, 0.1
    prev_error, integral = 0.0, 0.0
    distance_traveled = 0.0
    max_timeout = (calibrated_target_m / target_speed_mps) + 2.0
    start_time = time.time()

    while distance_traveled < calibrated_target_m and (time.time() - start_time < max_timeout):
        curr_x = latest_pos[0]
        curr_y = latest_pos[1]

        distance_traveled = math.sqrt((curr_x - start_pos_x)**2 + (curr_y - start_pos_y)**2)
        curr_yaw = latest_yaw
        error = normalize_angle(target_yaw - curr_yaw)

        integral += error * 0.01
        derivative = (error - prev_error) / 0.01
        prev_error = error

        # 🌟 Ramp-down Profile: ชะลอความเร็วช่วง 85% สุดท้าย
        if distance_traveled >= (calibrated_target_m * 0.85):
            current_exec_speed = 0.10
        else:
            current_exec_speed = target_speed_mps

        z_speed = max(min((Kp * error) + (Ki * integral) + (Kd * derivative), 30.0), -30.0)
        ep_chassis.drive_speed(x=current_exec_speed, y=0, z=-z_speed)
        time.sleep(0.01)

    # Active Brake สั่งหยุดนิ่งไม่ให้สไลด์
    ep_chassis.drive_speed(x=0, y=0, z=0)
    time.sleep(0.10)
    ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)

if __name__ == '__main__':
    print("=== ROBOMASTER EP - DISTANCE CALIBRATION TOOL ===")
    speed_input = float(input("กำหนดความเร็วที่ต้องการ Calibration (m/s) [แนะนำ 0.24 - 0.30]: "))
    speed_key = f"{speed_input:.2f}"

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    
    ep_chassis = ep_robot.chassis
    ep_chassis.sub_position(freq=20, callback=chassis_pos_handler)
    ep_chassis.sub_attitude(freq=20, callback=chassis_att_handler)
    
    time.sleep(1.0) # รอเซนเซอร์เริ่มทำงาน

    print("\nเตรียมพร้อม: วางหุ่นยนต์ตรงจุดเริ่มต้นข้างตลับเมตร")
    input("กด Enter เพื่อเริ่มทดสอบเดิน 3 รอบ...")

    measured_distances = []

    for run_idx in range(1, 4):
        print(f"\n---> การทดสอบรอบที่ {run_idx}/3 (สั่งเดิน 60 cm ที่สปีด {speed_key} m/s)...")
        time.sleep(1.0)
        
        move_forward_60cm_test(ep_chassis, target_speed_mps=speed_input, calib_factor=1.0)
        
        real_cm = float(input(f"วัดระยะทางจริงที่เดินได้ในรอบที่ {run_idx} (หน่วย cm): "))
        measured_distances.append(real_cm)

    # คำนวณค่าเฉลี่ยระยะจริงที่ได้
    avg_measured_cm = sum(measured_distances) / len(measured_distances)
    
    # คำนวณ Calibration Factor (สั่ง / ได้จริง)
    calculated_factor = 60.0 / avg_measured_cm
    
    print("\n" + "="*50)
    print(f"ผลการทดสอบ 3 รอบ: {measured_distances} cm")
    print(f"ระยะทางเฉลี่ยที่วัดได้จริง : {avg_measured_cm:.2f} cm")
    print(f"Distance Calib Factor สรุปสำหรับสปีด {speed_key} m/s: {calculated_factor:.4f}")
    print("="*50)

    # บันทึกลงไฟล์ JSON
    json_filename = "distance_calib_params.json"
    calib_db = {}
    
    if os.path.exists(json_filename):
        try:
            with open(json_filename, "r", encoding="utf-8") as f:
                calib_db = json.load(f)
        except Exception:
            calib_db = {}

    calib_db[speed_key] = calculated_factor

    with open(json_filename, "w", encoding="utf-8") as f:
        json.dump(calib_db, f, indent=4, ensure_ascii=False)

    print(f"[SUCCESS] บันทึกค่า Calibration ลงไฟล์ '{json_filename}' เรียบร้อยแล้ว!")

    ep_chassis.unsub_position()
    ep_chassis.unsub_attitude()
    ep_robot.close()