import os
import json
import cv2
import numpy as np
import time
import queue
from robomaster import robot

CONFIG_FILE = "hsv_config.json"
ROI_MASK_FILE = "roi_mask.png"
GIMBAL_FAST_SPEED = 250

DEFAULT_CONFIG = {
    "red": [
        {"lower": [0, 100, 80], "upper": [10, 255, 255]},
        {"lower": [165, 100, 80], "upper": [180, 255, 255]}
    ],
    "yellow": [{"lower": [15, 100, 100], "upper": [35, 255, 255]}],
    "green": [{"lower": [38, 80, 50], "upper": [75, 255, 255]}],
    "blue": [{"lower": [95, 90, 50], "upper": [135, 255, 255]}]
}

# ==========================================
# ⚡ FAST RECTANGLE DRAG-TO-SELECT MASK GLOBALS
# ==========================================
drawing = False
roi_mode = "PAINT"  # โหมด: 'PAINT' (เปิดพื้นที่สี่เหลี่ยม *1) หรือ 'ERASE' (ลบสี่เหลี่ยมเป็นดำ *0)
start_point = (-1, -1)
current_point = (-1, -1)
roi_mask = None

def draw_rect_callback(event, x, y, flags, param):
    global drawing, start_point, current_point, roi_mask, roi_mode
    if roi_mask is None: return

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        start_point = (x, y)
        current_point = (x, y)

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            current_point = (x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        if drawing:
            drawing = False
            end_point = (x, y)
            
            # คำนวณพิกัดมุมสี่เหลี่ยม (x1, y1) ถึง (x2, y2)
            x1, x2 = min(start_point[0], end_point[0]), max(start_point[0], end_point[0])
            y1, y2 = min(start_point[1], end_point[1]), max(start_point[1], end_point[1])

            draw_val = 255 if roi_mode == "PAINT" else 0
            
            # เติมพิกเซลในกรอบสี่เหลี่ยมรวดเร็วทันที (Fast Array Slicing)
            roi_mask[y1:y2, x1:x2] = draw_val
            
            start_point = (-1, -1)
            current_point = (-1, -1)

def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
                if "red" in data and len(data["red"]) < 2:
                    data["red"].append({"lower": [165, 100, 80], "upper": [180, 255, 255]})
                print(f"✅ โหลดค่าคอนฟิกจาก '{CONFIG_FILE}' สำเร็จ!")
                return data
        except Exception as e:
            print(f"⚠️ อ่านไฟล์คอนฟิกไม่สำเร็จ: {e} | ใช้ค่า Default แทน")
    return DEFAULT_CONFIG.copy()

def save_config(config_data):
    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(config_data, f, indent=4)
        print(f"🎉 [SUCCESS] บันทึกค่าสีลงไฟล์ '{CONFIG_FILE}' เรียบร้อยแล้ว!")
    except Exception as e:
        print(f"❌ เกิดข้อผิดพลาดในการบันทึกไฟล์: {e}")

def save_roi_mask():
    global roi_mask
    if roi_mask is not None:
        cv2.imwrite(ROI_MASK_FILE, roi_mask)
        print(f"💾 [SAVED] บันทึกไฟล์ '{ROI_MASK_FILE}' เรียบร้อยแล้ว!")

def nothing(x): pass

def setup_trackbars():
    cv2.namedWindow("HSV Control", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("HSV Control", 500, 320)

    cv2.createTrackbar("H_min", "HSV Control", 0, 180, nothing)
    cv2.createTrackbar("H_max", "HSV Control", 180, 180, nothing)
    cv2.createTrackbar("S_min", "HSV Control", 0, 255, nothing)
    cv2.createTrackbar("S_max", "HSV Control", 255, 255, nothing)
    cv2.createTrackbar("V_min", "HSV Control", 0, 255, nothing)
    cv2.createTrackbar("V_max", "HSV Control", 255, 255, nothing)

def update_trackbars(range_data):
    lower = range_data["lower"]
    upper = range_data["upper"]
    cv2.setTrackbarPos("H_min", "HSV Control", lower[0])
    cv2.setTrackbarPos("H_max", "HSV Control", upper[0])
    cv2.setTrackbarPos("S_min", "HSV Control", lower[1])
    cv2.setTrackbarPos("S_max", "HSV Control", upper[1])
    cv2.setTrackbarPos("V_min", "HSV Control", lower[2])
    cv2.setTrackbarPos("V_max", "HSV Control", upper[2])

def get_trackbar_values():
    h_min = cv2.getTrackbarPos("H_min", "HSV Control")
    h_max = cv2.getTrackbarPos("H_max", "HSV Control")
    s_min = cv2.getTrackbarPos("S_min", "HSV Control")
    s_max = cv2.getTrackbarPos("S_max", "HSV Control")
    v_min = cv2.getTrackbarPos("V_min", "HSV Control")
    v_max = cv2.getTrackbarPos("V_max", "HSV Control")
    return [h_min, s_min, v_min], [h_max, s_max, v_max]

# ==========================================
# 📐 GEOMETRIC SHAPE DETECTION
# ==========================================
def detect_geometric_shapes(img, mask):
    display_img = img.copy()
    
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

    roi = cv2.bitwise_and(img, img, mask=mask_clean)
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    sobelx = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=3)
    sobely = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=3)
    sobel_edges = np.uint8(np.clip(cv2.magnitude(sobelx, sobely), 0, 255))
    canny_edges = cv2.Canny(blur, 50, 150)
    combined_edges = cv2.bitwise_or(sobel_edges, canny_edges)

    contours, _ = cv2.findContours(combined_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 150: continue

        x, y, bw, bh = cv2.boundingRect(cnt)
        if bw == 0 or bh == 0: continue

        bbox_mask = mask_clean[y:y+bh, x:x+bw]
        color_pixel_count = cv2.countNonZero(bbox_mask)
        fill_ratio = (color_pixel_count / float(bw * bh)) * 100.0

        if fill_ratio < 40.0: continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0: continue

        approx = cv2.approxPolyDP(cnt, 0.03 * perimeter, True)
        num_vertices = len(approx)

        extent = float(area) / (bw * bh)
        aspect_ratio = float(bw) / float(bh)

        subtype = "SQUARE"
        if num_vertices >= 5 and extent < 0.83:
            subtype = "CIRCLE"
        elif num_vertices == 4 or extent >= 0.80:
            if 0.85 <= aspect_ratio <= 1.18:
                subtype = "SQUARE"
            elif aspect_ratio > 1.18:
                subtype = "RECT_H"
            elif aspect_ratio < 0.85:
                subtype = "RECT_V"

        color_box = (0, 255, 0) if subtype != "CIRCLE" else (255, 0, 255)
        cv2.rectangle(display_img, (x, y), (x + bw, y + bh), color_box, 2)
        lbl = f"{subtype} ({fill_ratio:.0f}%)"
        cv2.putText(display_img, lbl, (x, max(15, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_box, 2)

    return display_img, mask_clean, sobel_edges, canny_edges

def safe_gimbal_move(ep_gimbal, yaw_deg):
    try:
        action = ep_gimbal.moveto(pitch=0, yaw=yaw_deg, pitch_speed=GIMBAL_FAST_SPEED, yaw_speed=GIMBAL_FAST_SPEED)
        if action is not None:
            action.wait_for_completed(timeout=1.5)
    except Exception as e:
        print(f"⚠️ Gimbal Move Notice: {e}")

def print_instructions():
    print("\n==================================================")
    print("🎨 ROBOMASTER EP - FAST DRAG-RECTANGLE ROI & HSV CALIBRATION")
    print("==================================================")
    print("การเลือก ROI (คลิกเมาส์ค้างลากเป็นกรอบสี่เหลี่ยม):")
    print("  [R] : โหมด Drag Paint (*1 - เปิดภาพในกรอบสี่เหลี่ยม)")
    print("  [E] : โหมด Drag Eraser (*0 - ลบพื้นที่สี่เหลี่ยมเป็นดำ)")
    print("  [I] : เลือกทั้งภาพ (*1 ทั้งหมด แล้วค่อยเอา [E] มาลบสี่เหลี่ยมออก)")
    print("  [C] : ลบทั้งหมดให้กลายเป็นสีดำ (*0 ทั้งหมด)")
    print("--------------------------------------------------")
    print("เลือกปรับสี HSV:")
    print("  [1]: แดง Range 1 (Low)  | [5]: แดง Range 2 (High)")
    print("  [2]: เหลือง             | [3]: เขียว")
    print("  [4]: น้ำเงิน")
    print("--------------------------------------------------")
    print("ฟังก์ชันอื่นๆ:")
    print("  [M] : สลับ Filter View (COLOR_RESULT -> MASK -> SOBEL -> CANNY -> RAW)")
    print("  [S] : บันทึก 'hsv_config.json' และ 'roi_mask.png'")
    print("  [Q] : ออกจากโปรแกรม")
    print("==================================================\n")

def main():
    global roi_mask, roi_mode, drawing, start_point, current_point
    config_data = load_config()
    current_color = "red"
    range_idx = 0
    view_mode = "COLOR_RESULT"
    filter_modes = ["COLOR_RESULT", "MASK", "SOBEL", "CANNY", "RAW"]
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    
    stream_started = False
    for attempt in range(3):
        try:
            print(f"📹 กำลังเปิดสตรีมกล้อง... (พยายามครั้งที่ {attempt+1})")
            ep_camera.start_video_stream(display=False)
            stream_started = True
            time.sleep(1.0)
            break
        except Exception as e:
            print(f"⚠️ เปิดสตรีมไม่สำเร็จ: {e}")
            time.sleep(1.0)

    if not stream_started:
        print("❌ ไม่สามารถเปิดสตรีมวิดีโอจาก RoboMaster ได้")
        ep_robot.close()
        return

    safe_gimbal_move(ep_gimbal, 0)

    setup_trackbars()
    update_trackbars(config_data[current_color][range_idx])

    cv2.namedWindow("Original (Evaluated Target)")
    cv2.setMouseCallback("Original (Evaluated Target)", draw_rect_callback)

    print_instructions()

    try:
        while True:
            try:
                raw_img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            except (queue.Empty, Exception):
                time.sleep(0.02)
                continue

            if raw_img is None or not isinstance(raw_img, np.ndarray) or raw_img.size == 0:
                time.sleep(0.01)
                continue

            h, w, _ = raw_img.shape

            if roi_mask is None:
                if os.path.exists(ROI_MASK_FILE):
                    loaded = cv2.imread(ROI_MASK_FILE, cv2.IMREAD_GRAYSCALE)
                    if loaded is not None and loaded.shape == (h, w):
                        roi_mask = loaded
                    else:
                        roi_mask = np.zeros((h, w), dtype=np.uint8)
                else:
                    roi_mask = np.zeros((h, w), dtype=np.uint8)

            # คูณภาพด้วย roi_mask ทันที (*1 = ภาพจริง, *0 = สีดำ)
            img = cv2.bitwise_and(raw_img, raw_img, mask=roi_mask)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            lower_val, upper_val = get_trackbar_values()
            config_data[current_color][range_idx]["lower"] = lower_val
            config_data[current_color][range_idx]["upper"] = upper_val

            if current_color == "red" and len(config_data["red"]) >= 2:
                r1_low = np.array(config_data["red"][0]["lower"])
                r1_up  = np.array(config_data["red"][0]["upper"])
                r2_low = np.array(config_data["red"][1]["lower"])
                r2_up  = np.array(config_data["red"][1]["upper"])

                mask1 = cv2.inRange(hsv, r1_low, r1_up)
                mask2 = cv2.inRange(hsv, r2_low, r2_up)
                mask_combined = cv2.bitwise_or(mask1, mask2)
            else:
                mask_combined = cv2.inRange(hsv, np.array(lower_val), np.array(upper_val))

            evaluated_img, clean_mask, sobel_img, canny_img = detect_geometric_shapes(img, mask_combined)

            osd_orig = evaluated_img.copy()

            # วาดเส้นกรอบพรีวิวขณะที่กำลังลากเมาส์สี่เหลี่ยมสดๆ
            if drawing and start_point != (-1, -1) and current_point != (-1, -1):
                box_color = (0, 255, 0) if roi_mode == "PAINT" else (0, 0, 255)
                cv2.rectangle(osd_orig, start_point, current_point, box_color, 2)

            status_text = f"COLOR: {current_color.upper()} (R{range_idx+1})"
            roi_text = f"MODE: DRAG RECT ({roi_mode})"
            cv2.putText(osd_orig, status_text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(osd_orig, roi_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if roi_mode == "PAINT" else (0, 0, 255), 2)

            if view_mode == "COLOR_RESULT":
                filter_img = cv2.bitwise_and(img, img, mask=clean_mask)
            elif view_mode == "MASK":
                filter_img = cv2.cvtColor(clean_mask, cv2.COLOR_GRAY2BGR)
            elif view_mode == "SOBEL":
                filter_img = cv2.cvtColor(sobel_img, cv2.COLOR_GRAY2BGR)
            elif view_mode == "CANNY":
                filter_img = cv2.cvtColor(canny_img, cv2.COLOR_GRAY2BGR)
            else:
                filter_img = raw_img.copy()

            cv2.putText(filter_img, f"FILTER VIEW: {view_mode} (Press 'M' to switch)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow("Original (Evaluated Target)", osd_orig)
            cv2.imshow("Filtered Process View", filter_img)

            key = cv2.waitKey(1) & 0xFF

            # --- ปุ่มจัดการ ROI ---
            if key in (ord('r'), ord('R')):
                roi_mode = "PAINT"
                print("🖌️ โหมด: DRAG PAINT (คลิกค้างลากเป็นกรอบเพื่อแสดงภาพ)")
            elif key in (ord('e'), ord('E')):
                roi_mode = "ERASE"
                print("🧹 โหมด: DRAG ERASER (คลิกค้างลากเป็นกรอบเพื่อลบเป็นดำ)")
            elif key in (ord('i'), ord('I')):
                roi_mask = np.ones((h, w), dtype=np.uint8) * 255
                print("🖼️ เลือกทั้งภาพ (*1 ทั้งหมด)")
            elif key in (ord('c'), ord('C')):
                roi_mask = np.zeros((h, w), dtype=np.uint8)
                print("🗑️ ล้างเป็นสีดำทั้งหมด (*0 ทั้งหมด)")

            # --- เลือกสี HSV ---
            elif key == ord('1'):
                current_color, range_idx = "red", 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 RED Range 1 (Low)")
            elif key == ord('5'):
                current_color, range_idx = "red", 1
                update_trackbars(config_data[current_color][range_idx])
                print("👉 RED Range 2 (High)")
            elif key == ord('2'):
                current_color, range_idx = "yellow", 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 YELLOW")
            elif key == ord('3'):
                current_color, range_idx = "green", 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 GREEN")
            elif key == ord('4'):
                current_color, range_idx = "blue", 0
                update_trackbars(config_data[current_color][range_idx])
                print("👉 BLUE")

            # --- Gimbal Control ---
            elif key in (ord('w'), ord('W')):
                print("🔄 หมุน Gimbal ไปหน้า (0°)")
                safe_gimbal_move(ep_gimbal, 0)
            elif key in (ord('a'), ord('A')):
                print("🔄 หมุน Gimbal ไปซ้าย (-90°)")
                safe_gimbal_move(ep_gimbal, -90)
            elif key in (ord('d'), ord('D')):
                print("🔄 หมุน Gimbal ไปขวา (+90°)")
                safe_gimbal_move(ep_gimbal, 90)
            elif key in (ord('x'), ord('X')):
                print("🔄 หมุน Gimbal ไปหลัง (180°)")
                safe_gimbal_move(ep_gimbal, 180)

            # --- View Mode & Save/Quit ---
            elif key in (ord('m'), ord('M')):
                view_mode = filter_modes[(filter_modes.index(view_mode) + 1) % len(filter_modes)]
                print(f"🔄 สลับ Filter View เป็น: {view_mode}")
            elif key in (ord('s'), ord('S')):
                save_config(config_data)
                save_roi_mask()
            elif key in (ord('q'), ord('Q')):
                print("[INFO] กำลังปิดระบบ Calibration...")
                break

    finally:
        try:
            cv2.destroyAllWindows()
            ep_camera.stop_video_stream()
            ep_robot.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()