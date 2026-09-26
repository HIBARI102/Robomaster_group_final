import os
import json
import cv2
import numpy as np
from robomaster import robot

# ชื่อไฟล์ JSON สำหรับเซฟและโหลดค่า
CONFIG_FILE = "hsv_config.json"

# ค่า Default HSV เริ่มต้น (ใช้โครงสร้างแบบ List of Ranges รองรับสีแดง 2 ช่วง)
DEFAULT_CONFIG = {
    "red": [
        {"lower": [0, 100, 80], "upper": [10, 255, 255]},
        {"lower": [165, 100, 80], "upper": [180, 255, 255]}
    ],
    "yellow": [
        {"lower": [15, 100, 100], "upper": [35, 255, 255]}
    ],
    "green": [
        {"lower": [38, 80, 50], "upper": [75, 255, 255]}
    ],
    "blue": [
        {"lower": [95, 90, 50], "upper": [135, 255, 255]}
    ]
}

def load_config():
    """โหลดไฟล์ hsv_config.json ถ้ามี ถ้าไม่มีให้ใช้ DEFAULT_CONFIG"""
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                print(f"✅ โหลดค่าคอนฟิกจาก '{CONFIG_FILE}' สำเร็จ!")
                return data
        except Exception as e:
            print(f"⚠️ อ่านไฟล์คอนฟิกไม่สำเร็จ: {e} | ใช้ค่า Default แทน")
    return DEFAULT_CONFIG.copy()

def save_config(config_data):
    """บันทึกค่าคอนฟิกลงไฟล์ hsv_config.json"""
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(config_data, f, indent=4)
        print(f"🎉 [SUCCESS] บันทึกค่าสีลงไฟล์ '{CONFIG_FILE}' เรียบร้อยแล้ว!")
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาดในการบันทึกไฟล์: {e}")

def nothing(x):
    pass

def setup_trackbars():
    """สร้างหน้าต่างควบคุม Trackbars"""
    cv2.namedWindow("HSV Control", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Control", 500, 350)

    cv2.createTrackbar("H_min", "HSV Control", 0, 180, nothing)
    cv2.createTrackbar("H_max", "HSV Control", 180, 180, nothing)
    cv2.createTrackbar("S_min", "HSV Control", 0, 255, nothing)
    cv2.createTrackbar("S_max", "HSV Control", 255, 255, nothing)
    cv2.createTrackbar("V_min", "HSV Control", 0, 255, nothing)
    cv2.createTrackbar("V_max", "HSV Control", 255, 255, nothing)

def update_trackbars(range_data):
    """อัปเดตตำแหน่ง Trackbar ตามค่าสีปัจจุบัน"""
    lower = range_data["lower"]
    upper = range_data["upper"]
    cv2.setTrackbarPos("H_min", "HSV Control", lower[0])
    cv2.setTrackbarPos("H_max", "HSV Control", upper[0])
    cv2.setTrackbarPos("S_min", "HSV Control", lower[1])
    cv2.setTrackbarPos("S_max", "HSV Control", upper[1])
    cv2.setTrackbarPos("V_min", "HSV Control", lower[2])
    cv2.setTrackbarPos("V_max", "HSV Control", upper[2])

def get_trackbar_values():
    """ดึงค่าปัจจุบันจาก Trackbars"""
    h_min = cv2.getTrackbarPos("H_min", "HSV Control")
    h_max = cv2.getTrackbarPos("H_max", "HSV Control")
    s_min = cv2.getTrackbarPos("S_min", "HSV Control")
    s_max = cv2.getTrackbarPos("S_max", "HSV Control")
    v_min = cv2.getTrackbarPos("V_min", "HSV Control")
    v_max = cv2.getTrackbarPos("V_max", "HSV Control")
    return [h_min, s_min, v_min], [h_max, s_max, v_max]

def print_instructions():
    print("\n==================================================")
    print("🎨 ROBOMASTER EP - HSV COLOR CALIBRATION TOOL")
    print("==================================================")
    print("กดปุ่มเพื่อเลือกสีและปรับแต่งค่า HSV:")
    print("  [1] : สีแดง (Red - Main Range)")
    print("  [2] : สีเหลือง (Yellow)")
    print("  [3] : สีเขียว (Green)")
    print("  [4] : สีน้ำเงิน (Blue)")
    print("  [5] : สีแดงช่วงที่ 2 (Red - High Range 165-180)")
    print("--------------------------------------------------")
    print("  [S] : บันทึกค่าสีลงไฟล์ 'hsv_config.json'")
    print("  [Q] : ปิดโปรแกรม")
    print("==================================================\n")

def main():
    config_data = load_config()
    current_color = "red"
    range_idx = 0
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_camera.start_video_stream(display=False)
    
    ep_gimbal.recenter().wait_for_completed()

    setup_trackbars()
    update_trackbars(config_data[current_color][range_idx])
    print_instructions()

    try:
        # สั่ง Gimbal หมุนกลับมาหน้าตรง Pitch = 0, Yaw = 0
        while True:
            img = ep_camera.read_cv2_image(strategy="newest")
            if img is None:
                continue

            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # ดึงค่าจาก Trackbar และอัปเดตลงโครงสร้างข้อมูลปัจจุบัน
            lower_val, upper_val = get_trackbar_values()
            config_data[current_color][range_idx]["lower"] = lower_val
            config_data[current_color][range_idx]["upper"] = upper_val

            # สร้าง Mask เพื่อแสดงผลการกรองสี
            lower_np = np.array(lower_val)
            upper_np = np.array(upper_val)
            mask = cv2.inRange(hsv, lower_np, upper_np)
            result = cv2.bitwise_and(img, img, mask=mask)

            # แสดง OSD สถานะการเลือกสีบนหน้าจอหลัก
            status_text = f"Color: {current_color.upper()} (Range {range_idx+1})"
            cv2.putText(img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            cv2.imshow("Original Camera", img)
            cv2.imshow("Mask (Binary)", mask)
            cv2.imshow("Result Filtered", result)

            key = cv2.waitKey(1) & 0xFF

            # # การควบคุมผ่านคีย์บอร์ด
            if key == ord('1'):
                current_color = "red"
                range_idx = 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 สลับไปปรับแต่ง: RED Range 1")
            elif key == ord('2'):
                current_color = "yellow"
                range_idx = 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 สลับไปปรับแต่ง: YELLOW")
            elif key == ord('3'):
                current_color = "green"
                range_idx = 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 สลับไปปรับแต่ง: GREEN")
            elif key == ord('4'):
                current_color = "blue"
                range_idx = 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 สลับไปปรับแต่ง: BLUE")
            elif key == ord('5'):
                current_color = "red"
                range_idx = 1
                update_trackbars(config_data[current_color][range_idx])
                print("👉 สลับไปปรับแต่ง: RED Range 2 (High Range)")
            elif key in (ord('s'), ord('S')):
                save_config(config_data)
            elif key in (ord('q'), ord('Q')):
                print("[INFO] กำลังปิดระบบ Calibration...")
                break

    finally:
        cv2.destroyAllWindows()
        ep_camera.stop_video_stream()
        ep_robot.close()

if __name__ == '__main__':
    main()