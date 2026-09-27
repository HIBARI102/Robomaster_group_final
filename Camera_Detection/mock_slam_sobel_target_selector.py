import os
import json
import time
import math
import cv2
import numpy as np
import threading
import queue
import traceback
from collections import Counter
from robomaster import robot

# ==========================================
# 1. SETUP & CONFIGURATION
# ==========================================
GRID_W = 5
GRID_H = 5
CELL_SIZE_CM = 60.0
CAMERA_FOCAL_LENGTH_PX = 650.0
GIMBAL_FAST_SPEED = 320    
GIMBAL_SLOW_SPEED = 90     

TOF_A, TOF_B, TOF_C = 0.0, 1.0, 0.0
if os.path.exists("tof_calib_params.json"):
    try:
        with open("tof_calib_params.json", "r", encoding="utf-8") as f:
            calib_data = json.load(f)
            TOF_A = calib_data.get("a", 0.0)
            TOF_B = calib_data.get("b", calib_data.get("slope", 1.0))
            TOF_C = calib_data.get("c", calib_data.get("intercept", 0.0))
            print("✅ [TOF CALIB LOADED] โหลดค่า Calibration ToF สำเร็จ")
    except Exception as e:
        print(f"⚠️ Load ToF Calib Error: {e}")

DEFAULT_HSV_RANGES = {
    "red": [{"lower": [0, 100, 80], "upper": [10, 255, 255]}, {"lower": [165, 100, 80], "upper": [180, 255, 255]}],
    "yellow": [{"lower": [15, 100, 100], "upper": [35, 255, 255]}],
    "green": [{"lower": [38, 80, 50], "upper": [75, 255, 255]}],
    "blue": [{"lower": [95, 90, 50], "upper": [135, 255, 255]}]
}

def load_hsv_ranges(json_path="hsv_config.json"):
    data = DEFAULT_HSV_RANGES
    if os.path.exists(json_path):
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            pass
    return {c: [(np.array(r["lower"]), np.array(r["upper"])) for r in ranges] for c, ranges in data.items()}

COLOR_RANGES = load_hsv_ranges("hsv_config.json")

def normalize_angle(deg):
    return (deg + 180.0) % 360.0 - 180.0

# ==========================================
# 2. SAFE GIMBAL CONTROL HELPER (แก้จุดค้างหลุด)
# ==========================================
def safe_gimbal_move(ep_gimbal, pitch, yaw, speed=GIMBAL_FAST_SPEED, wait_sec=0.15):
    """
    สั่ง Gimbal หมุนและรอจนกว่าจะถึงตำแหน่งจริงแบบเดียวกับ SLAM (ป้องกัน Action Overlap)
    """
    try:
        # 1. ส่งคำสั่งและเก็บ Action Object ไว้
        action = ep_gimbal.moveto(pitch=pitch, yaw=yaw, pitch_speed=speed, yaw_speed=speed)
        
        # 2. รอให้ Gimbal หมุนเสร็จสิ้นจริง (ตั้ง Timeout ไว้ 2.0 วินาทีสำหรับมุมกวาดกว้าง)
        if action is not None:
            action.wait_for_completed(timeout=2.0)
            
    except Exception as e:
        print(f"⚠️ Gimbal Action Notice: {e}")
    
    # 3. หน่วงเวลาสั้นๆ เพิ่มเติมเพื่อให้เซนเซอร์/ภาพนิ่งสนิท
    time.sleep(wait_sec)

# ==========================================
# 3. SHARED THREAD HUB + SPATIAL MEMORY
# ==========================================
class ThreadHub:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.is_scanning = False
        self.status_msg = "IDLE"
        self.debug_mode = "NORMAL"
        
        self.tof_raw_cm = 999.0
        self.tof_calibrated_cm = 999.0
        self.grid_x = 2
        self.grid_y = 2
        
        self.latest_frame = None
        self.targets_found = []
        self.target_id_counter = 1

    def get_robot_world_pos(self):
        x_cm = (self.grid_x + 0.5) * CELL_SIZE_CM
        y_cm = (self.grid_y + 0.5) * CELL_SIZE_CM
        return x_cm, y_cm

    def update_tof(self, raw_mm):
        with self.lock:
            if raw_mm and raw_mm[0] is not None:
                try:
                    raw_cm = float(raw_mm[0]) / 10.0
                    if raw_cm <= 0:
                        calib_cm = 999.0
                    else:
                        calib_cm = (TOF_A * (raw_cm ** 2)) + (TOF_B * raw_cm) + TOF_C
                        calib_cm = max(0.0, calib_cm)
                    self.tof_raw_cm = raw_cm
                    self.tof_calibrated_cm = calib_cm
                except Exception:
                    pass

    def register_target_spatial(self, abs_yaw_deg, tof_dist_cm, color, shape):
        with self.lock:
            rx_cm, ry_cm = self.get_robot_world_pos()
            rad = math.radians(abs_yaw_deg)
            
            target_world_x = rx_cm + (tof_dist_cm * math.sin(rad))
            target_world_y = ry_cm + (tof_dist_cm * math.cos(rad))

            for t in self.targets_found:
                prev_x = t["world_pos_cm"]["x"]
                prev_y = t["world_pos_cm"]["y"]
                dist_between = math.sqrt((target_world_x - prev_x)**2 + (target_world_y - prev_y)**2)
                
                if dist_between <= 25.0 and t["color"] == color:
                    print(f"⚠️ [DUPLICATE DETECTED] เป้าหมายนี้เคยถูกบันทึกแล้ว! (ระยะห่าง {dist_between:.1f} cm)")
                    return False

            norm_dir = normalize_angle(abs_yaw_deg)
            cardinal_dir = "N"
            target_gx, target_gy = self.grid_x, self.grid_y

            if -45.0 <= norm_dir <= 45.0: target_gy += 1; cardinal_dir = "N"
            elif 45.0 < norm_dir <= 135.0: target_gx += 1; cardinal_dir = "E"
            elif norm_dir > 135.0 or norm_dir < -135.0: target_gy -= 1; cardinal_dir = "S"
            elif -135.0 <= norm_dir < -45.0: target_gx -= 1; cardinal_dir = "W"

            record = {
                "target_id": self.target_id_counter,
                "color": color,
                "shape": shape,
                "target_grid": {"x": target_gx, "y": target_gy},
                "standing_grid": {"x": self.grid_x, "y": self.grid_y},
                "world_pos_cm": {"x": round(target_world_x, 1), "y": round(target_world_y, 1)},
                "firing_direction": cardinal_dir,
                "firing_yaw_deg": round(norm_dir, 1),
                "is_knocked_down": False
            }
            self.targets_found.append(record)
            self.target_id_counter += 1
            print(f"\n🎉 [CONFIRMED & SAVED] บันทึกพิกัดเป้าหมายลง Map สำเร็จ!")
            print(f"   ├─ เป้าหมาย : {color.upper()} ({shape})")
            print(f"   ├─ พิกัดโลก  : World Pos ({target_world_x:.1f}, {target_world_y:.1f}) cm")
            print(f"   ├─ จุดยืนยิง : Grid({self.grid_x}, {self.grid_y})")
            print(f"   └─ ทิศหันยิง : ทิศ {cardinal_dir} ({norm_dir:.1f} องศา)\n")
            return True

hub = ThreadHub()

# ==========================================
# 4. VISION PIPELINE
# ==========================================
def get_color_mask(hsv_img, target_color):
    ranges = COLOR_RANGES.get(target_color, [])
    mask = None
    for (lower, upper) in ranges:
        m = cv2.inRange(hsv_img, lower, upper)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask

def process_frame_sobel_pipeline(img, distance_cm, target_color="red", target_shape_type="SQUARE"):
    if img is None: return None, {}

    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = get_color_mask(hsv, target_color)
        if mask is None:
            return None, {"mask": np.zeros_like(img[:,:,0]), "sobel": np.zeros_like(img[:,:,0])}

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

        debug_views = {"mask": mask_clean, "sobel": sobel_edges, "canny": canny_edges, "combined": combined_edges}

        # 1. Circle Detection
        if target_shape_type in ["CIRCLE", "ANY"]:
            circles = cv2.HoughCircles(blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30, param1=50, param2=30, minRadius=10, maxRadius=120)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                for c in circles[0, :]:
                    cx, cy, r = c[0], c[1], c[2]
                    real_d_cm = ((r * 2) * distance_cm) / CAMERA_FOCAL_LENGTH_PX
                    if abs(real_d_cm - 7.0) <= 2.5:
                        return {
                            "color": target_color, "shape": "CIRCLE",
                            "bbox": (max(0, cx - r), max(0, cy - r), r * 2, r * 2),
                            "cx": cx, "cy": cy, "real_w_cm": real_d_cm, "real_h_cm": real_d_cm
                        }, debug_views

        # 2. Rectangle Detection
        contours, _ = cv2.findContours(combined_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 120: continue

            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw == 0 or bh == 0: continue

            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue

            circularity = (4 * np.pi * area) / (perimeter * perimeter)
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)
            solidity = float(area) / hull_area if hull_area > 0 else 0

            approx = cv2.approxPolyDP(cnt, 0.035 * perimeter, True)
            vertices = len(approx)

            real_w_cm = (bw * distance_cm) / CAMERA_FOCAL_LENGTH_PX
            real_h_cm = (bh * distance_cm) / CAMERA_FOCAL_LENGTH_PX
            rect_area = bw * bh
            extent = float(area) / rect_area if rect_area > 0 else 0

            if circularity >= 0.78 and solidity > 0.88:
                continue

            if (vertices == 4 or extent > 0.68) and circularity < 0.78:
                margin = 2.2
                is_sq = (abs(real_w_cm - 7.0) <= margin) and (abs(real_h_cm - 7.0) <= margin)
                is_rect_h = (abs(real_w_cm - 9.0) <= margin) and (abs(real_h_cm - 6.0) <= margin)
                is_rect_v = (abs(real_w_cm - 6.0) <= margin) and (abs(real_h_cm - 9.0) <= margin)

                subtype = None
                if is_sq: subtype = "SQUARE"
                elif is_rect_h: subtype = "RECT_H"
                elif is_rect_v: subtype = "RECT_V"

                if subtype and (target_shape_type == "ANY" or target_shape_type == subtype):
                    return {
                        "color": target_color, "shape": subtype,
                        "bbox": (x, y, bw, bh),
                        "cx": x + (bw // 2), "cy": y + (bh // 2),
                        "real_w_cm": real_w_cm, "real_h_cm": real_h_cm
                    }, debug_views

    except Exception:
        pass

    return None, {}

# ==========================================
# 5. SAFE WORKER THREAD (SAFE EXCEPTION HANDLING)
# ==========================================
def is_angle_duplicate(new_angle, saved_angles, threshold_deg=15.0):
    for ang in saved_angles:
        if abs(normalize_angle(new_angle - ang)) <= threshold_deg:
            return True
    return False

def slam_scan_worker(ep_gimbal, ep_camera, target_color, target_shape):
    """ครอบ Exception ทั้งระบบไม่ให้หลุดกลับหน้า Terminal เด็ดขาด"""
    try:
        with hub.lock:
            hub.is_scanning = True
            hub.status_msg = "PHASE 1: RAPID SWEEP..."

        print(f"\n==================================================")
        print(f"🔄 [PHASE 1] หมุนกวาดเร็ว 4 ทิศทาง (เป้าหมาย: {target_color.upper()} | {target_shape})")
        print("==================================================")
        
        scan_seq = [('AHEAD', 0), ('RIGHT', 90), ('BACK', 180), ('LEFT', -90)]
        scan_queue = queue.Queue()
        logged_angles = []

        # 1. RAPID SWEEP
        for dir_name, rel_y in scan_seq:
            if not hub.running: break
            print(f"👉 [SWEEP] หมุนไปทิศทาง {dir_name} ({rel_y} deg)...")

            safe_gimbal_move(ep_gimbal, pitch=-3.5, yaw=rel_y, speed=GIMBAL_FAST_SPEED, wait_sec=0.8)

            with hub.lock:
                img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                tof_dist = hub.tof_calibrated_cm

            if img is not None:
                res, _ = process_frame_sobel_pipeline(img, tof_dist, target_color, target_shape)

                if res:
                    h, w, _ = img.shape
                    offset_x_px = res["cx"] - (w // 2)
                    deg_offset = (offset_x_px / CAMERA_FOCAL_LENGTH_PX) * (180.0 / math.pi)
                    target_yaw = rel_y + deg_offset

                    if not is_angle_duplicate(target_yaw, logged_angles):
                        logged_angles.append(target_yaw)
                        scan_queue.put({"dir_name": dir_name, "yaw": target_yaw})
                        print(f"📝 [TRIGGER QUEUED] สะกิดเจอ! ดันมุม {target_yaw:.1f} deg ใส่ Queue")

        # 2. QUEUE EXECUTION
        if scan_queue.empty():
            print("\nℹ️ [SWEEP COMPLETE] หมุนกวาดรอบแรกไม่พบเป้าหมายน่าสงสัย")
        else:
            print(f"\n==================================================")
            print(f"🎯 [PHASE 2 & 3] ทยอยดึงคิวเป้าหมายออกมาเล็งช้าๆ (ใน Queue มี {scan_queue.qsize()} จุด)")
            print("==================================================")

            while not scan_queue.empty() and hub.running:
                item = scan_queue.get()
                target_yaw = item["yaw"]
                dir_name = item["dir_name"]

                print(f"\n🔍 [EXECUTE QUEUE] หมุน Gimbal ช้าๆ ไปมุม {target_yaw:.1f} deg...")
                safe_gimbal_move(ep_gimbal, pitch=-3.5, yaw=target_yaw, speed=GIMBAL_SLOW_SPEED, wait_sec=1.0)

                # รอนิ่งสนิท 0.4 วินาที ให้ภาพและ ToF นิ่ง
                time.sleep(0.40)

                with hub.lock:
                    img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                    tof_dist = hub.tof_calibrated_cm

                if img is not None:
                    res, _ = process_frame_sobel_pipeline(img, tof_dist, target_color, target_shape)

                    # Center Lock Alignment
                    if res:
                        h, w, _ = img.shape
                        offset_x_px = res["cx"] - (w // 2)
                        if abs(offset_x_px) > 15:
                            deg_offset = (offset_x_px / CAMERA_FOCAL_LENGTH_PX) * (180.0 / math.pi)
                            target_yaw = target_yaw + deg_offset
                            print(f"🎯 Center Re-Align: ขยับ Gimbal เล็งตรงกลางไปที่มุม {target_yaw:.1f} deg...")
                            safe_gimbal_move(ep_gimbal, pitch=-3.5, yaw=target_yaw, speed=50, wait_sec=0.6)

                # WARM-UP DETECT (3 Hits หรือ Timeout 0.5s)
                print("⏳ [WARM-UP] รอกล้องจับเจอเป้าหมายติดกัน 3 เฟรม (Timeout 0.5s)...")
                warmup_start = time.time()
                consecutive_hits = 0

                while time.time() - warmup_start < 0.50:
                    with hub.lock:
                        w_img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                        w_tof = hub.tof_calibrated_cm
                    
                    if w_img is not None:
                        w_res, _ = process_frame_sobel_pipeline(w_img, w_tof, target_color, target_shape)
                        if w_res:
                            consecutive_hits += 1
                            if consecutive_hits >= 3:
                                print("✅ [WARM-UP SUCCESS] จับภาพนิ่งเจอติดกัน 3 เฟรมแล้ว!")
                                break
                        else:
                            consecutive_hits = 0
                    time.sleep(0.03)

                # 10-FRAME SAMPLING
                print("📷 [10-FRAME SAMPLING] จ้องนิ่งเพื่อเก็บภาพ 10 เฟรมประมวลผล...")
                valid_samples = []
                REQUIRED_MATCH_FRAMES = 7

                for frame_idx in range(1, 11):
                    time.sleep(0.04)
                    with hub.lock:
                        sample_img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                        sample_tof = hub.tof_calibrated_cm

                    if sample_img is not None:
                        s_res, _ = process_frame_sobel_pipeline(sample_img, sample_tof, target_color, target_shape)
                        if s_res:
                            valid_samples.append((s_res["color"], s_res["shape"]))
                            print(f"   ├─ Frame {frame_idx:02d}/10: ✅ PASS -> {s_res['color'].upper()} {s_res['shape']} (ToF: {sample_tof:.1f}cm)")
                        else:
                            print(f"   ├─ Frame {frame_idx:02d}/10: ❌ FAIL / NO MATCH")
                    else:
                        print(f"   ├─ Frame {frame_idx:02d}/10: ⚠️ EMPTY FRAME")

                success_count = len(valid_samples)
                print(f"📊 สรุปผลสแกน 10 เฟรม: ผ่าน {success_count}/10 เฟรม (เกณฑ์: >={REQUIRED_MATCH_FRAMES})")

                if success_count >= REQUIRED_MATCH_FRAMES:
                    most_common = Counter(valid_samples).most_common(1)[0][0]
                    conf_color, conf_shape = most_common
                    with hub.lock:
                        tof_final = hub.tof_calibrated_cm
                    hub.register_target_spatial(abs_yaw_deg=target_yaw, tof_dist_cm=tof_final, color=conf_color, shape=conf_shape)
                else:
                    print("❌ [FALSE ALARM REJECTED] ตรวจจับผ่านน้อยกว่า 7 เฟรม ยกเลิกการบันทึก!")

                scan_queue.task_done()

        safe_gimbal_move(ep_gimbal, pitch=0, yaw=0, speed=GIMBAL_FAST_SPEED, wait_sec=0.5)

    except Exception as err:
        print(f"\n❌ [CRITICAL SCANNER THREAD ERROR]: {err}")
        traceback.print_exc()

    finally:
        with hub.lock:
            hub.is_scanning = False
            hub.status_msg = "SCAN COMPLETE"

# ==========================================
# 6. USER MENU & MAIN ENTRY
# ==========================================
def select_target_menu():
    print("\n==================================================")
    print("🎯 ROBOMASTER TARGET SELECTION MENU")
    print("==================================================")
    print("เลือกสีเป้าหมาย (Color):")
    print("  1. แดง (Red) | 2. เหลือง (Yellow) | 3. เขียว (Green) | 4. น้ำเงิน (Blue)")
    c_choice = input("👉 เลือกสี [1-4] (Default=3 Green): ").strip()
    color_map = {"1": "red", "2": "yellow", "3": "green", "4": "blue"}
    target_color = color_map.get(c_choice, "green")

    print("\nเลือกรูปแบบทรงเป้าหมาย (Shape):")
    print("  1. สี่เหลี่ยมผืนผ้าแนวตั้ง (6x9 cm - RECT_V)")
    print("  2. สี่เหลี่ยมผืนผ้าแนวนอน (9x6 cm - RECT_H)")
    print("  3. สี่เหลี่ยมจัตุรัส (7x7 cm - SQUARE)")
    print("  4. วงกลม (7 cm - CIRCLE)")
    print("  5. ไม่จำกัดทรง (ANY)")
    s_choice = input("👉 เลือกทรง [1-5] (Default=3 SQUARE): ").strip()
    shape_map = {"1": "RECT_V", "2": "RECT_H", "3": "SQUARE", "4": "CIRCLE", "5": "ANY"}
    target_shape = shape_map.get(s_choice, "SQUARE")

    print(f"\n✅ เลือกเป้าหมาย: สี [{target_color.upper()}] | ทรง [{target_shape}]")
    print("==================================================\n")
    return target_color, target_shape

if __name__ == '__main__':
    target_color, target_shape = select_target_menu()

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor

    ep_sensor.sub_distance(freq=20, callback=lambda info: hub.update_tof(info))
    ep_camera.start_video_stream(display=False)

    print("--------------------------------------------------")
    print(" [ M ] : สลับโหมดภาพ Debug (NORMAL -> SOBEL -> CANNY -> MASK)")
    print(" [ R ] : สั่งเริ่ม Two-Pass Queue Scan (มีระบบปลอดภัย ป้องกันสคริปต์ดับ)")
    print(" [ Q ] : บันทึก Map_Data_Round1.json และปิดระบบ")
    print("--------------------------------------------------\n")

    try:
        while hub.running:
            try:
                img = ep_camera.read_cv2_image(strategy="newest")
                if img is None: 
                    time.sleep(0.01)
                    continue

                with hub.lock:
                    hub.latest_frame = img
                    current_tof = hub.tof_calibrated_cm
                    debug_m = hub.debug_mode
                    is_scan = hub.is_scanning

                detected_target, debug_views = process_frame_sobel_pipeline(
                    img, current_tof, target_color=target_color, target_shape_type=target_shape
                )

                display_img = img.copy()
                if debug_m == "SOBEL" and "sobel" in debug_views:
                    display_img = cv2.cvtColor(debug_views["sobel"], cv2.COLOR_GRAY2BGR)
                elif debug_m == "CANNY" and "canny" in debug_views:
                    display_img = cv2.cvtColor(debug_views["canny"], cv2.COLOR_GRAY2BGR)
                elif debug_m == "MASK" and "mask" in debug_views:
                    display_img = cv2.cvtColor(debug_views["mask"], cv2.COLOR_GRAY2BGR)

                if detected_target:
                    x, y, bw, bh = detected_target["bbox"]
                    cv2.rectangle(display_img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    lbl = f"MATCH: {detected_target['color'].upper()} {detected_target['shape']} ({detected_target['real_w_cm']:.1f}cm)"
                    cv2.putText(display_img, lbl, (x, max(15, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                scan_str = "SCANNING & ALIGNING..." if is_scan else "IDLE"
                cv2.putText(display_img, f"STATUS: {scan_str}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255) if is_scan else (0, 255, 0), 2)
                cv2.putText(display_img, f"TARGET: {target_color.upper()} | {target_shape}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (241, 196, 15), 1)
                cv2.putText(display_img, f"VIEW MODE: {debug_m} (Press 'M')", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                cv2.putText(display_img, f"ToF Calibrated: {current_tof:.1f} cm", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

                cv2.imshow("Threaded Two-Pass SLAM Scanner", display_img)

                key = cv2.waitKey(20) & 0xFF
                if key in (ord('q'), ord('Q')):
                    break
                elif key in (ord('m'), ord('M')):
                    modes = ["NORMAL", "SOBEL", "CANNY", "MASK"]
                    with hub.lock:
                        hub.debug_mode = modes[(modes.index(hub.debug_mode) + 1) % len(modes)]
                        print(f"🔄 สลับโหมดภาพเป็น: {hub.debug_mode}")
                elif key in (ord('r'), ord('R')):
                    if not hub.is_scanning:
                        scan_thread = threading.Thread(
                            target=slam_scan_worker,
                            args=(ep_gimbal, ep_camera, target_color, target_shape),
                            daemon=True
                        )
                        scan_thread.start()
                    else:
                        print("⚠️ หุ่นยนต์กำลังสแกนอยู่ โปรดรอสักครู่...")
            except Exception as e:
                time.sleep(0.01)

    finally:
        hub.running = False
        data = {"grid_size": {"w": GRID_W, "h": GRID_H}, "targets_found": hub.targets_found}
        with open("Map_Data_Round1.json", "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        print("\n💾 [EXPORT] บันทึกไฟล์ Map_Data_Round1.json เรียบร้อยแล้ว!")

        try:
            ep_sensor.unsub_distance()
            cv2.destroyAllWindows()
            ep_camera.stop_video_stream()
            ep_robot.close()
        except Exception:
            pass
        print("[INFO] ปิดระบบเรียบร้อยแล้ว")