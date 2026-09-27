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
# 1. CONFIGURATION & PARAMETERS
# ==========================================
GRID_W = 5
GRID_H = 5
CELL_SIZE_CM = 60.0
CAMERA_FOCAL_LENGTH_PX = 650.0
GIMBAL_FAST_SPEED = 300    
GIMBAL_SLOW_SPEED = 80     

MIN_PIXEL_FILL_RATIO = 40.0  # ปรับเกณฑ์ Fill Ratio ป้องกันเงามั่ว

TOF_A, TOF_B, TOF_C = 0.0, 1.0, 0.0
if os.path.exists("tof_calib_params.json"):
    try:
        with open("tof_calib_params.json", "r", encoding="utf-8") as f:
            calib_data = json.load(f)
            TOF_A = float(calib_data.get("a", 0.0))
            TOF_B = float(calib_data.get("b", calib_data.get("slope", 1.0)))
            TOF_C = float(calib_data.get("c", calib_data.get("intercept", 0.0)))
    except Exception:
        pass

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

def safe_gimbal_move(ep_gimbal, pitch, yaw, speed=GIMBAL_FAST_SPEED, wait_sec=0.25):
    try:
        action = ep_gimbal.moveto(pitch=pitch, yaw=yaw, pitch_speed=speed, yaw_speed=speed)
        if action is not None:
            action.wait_for_completed(timeout=2.0)
    except Exception:
        pass
    time.sleep(wait_sec)

# ==========================================
# 2. SHARED HUB & SPATIAL MEMORY
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
            if raw_mm and len(raw_mm) > 0 and raw_mm[0] is not None:
                try:
                    raw_cm = float(raw_mm[0]) / 10.0
                    if raw_cm <= 0:
                        calib_cm = 999.0
                    else:
                        calib_cm = (TOF_A * (raw_cm ** 2)) + (TOF_B * raw_cm) + TOF_C
                        if calib_cm <= 0 or calib_cm > 400.0:
                            calib_cm = raw_cm
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
                
                if dist_between <= 20.0 and t["color"] == color:
                    print(f"⚠️ [DUPLICATE DETECTED] เป้าหมายนี้คือชิ้นเดิมที่เคยบันทึกแล้ว (ระยะห่าง {dist_between:.1f} cm)")
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
            print(f"\n🎉 [CONFIRMED & SAVED] บันทึกพิกัดเป้าหมายใหม่สำเร็จ! (เป้าหมายที่ {self.target_id_counter})")
            print(f"   ├─ เป้าหมาย : {color.upper()} ({shape})")
            print(f"   ├─ พิกัดโลก  : World Pos ({target_world_x:.1f}, {target_world_y:.1f}) cm")
            print(f"   ├─ ระยะ ToF  : {tof_dist_cm:.1f} cm")
            print(f"   ├─ จุดยืนยิง : Grid({self.grid_x}, {self.grid_y})")
            print(f"   └─ ทิศหันยิง : ทิศ {cardinal_dir} ({norm_dir:.1f} องศา)\n")
            self.target_id_counter += 1
            return True

hub = ThreadHub()

# ==========================================
# 3. ACCURATE SHAPE DETECTION PIPELINE
# ==========================================
def get_color_mask(hsv_img, target_color):
    ranges = COLOR_RANGES.get(target_color, [])
    mask = None
    for (lower, upper) in ranges:
        m = cv2.inRange(hsv_img, lower, upper)
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask
def process_frame_multi_targets(img, distance_cm, target_color="red", target_shape_type="SQUARE", strict_mode=False):
    if img is None or not isinstance(img, np.ndarray) or img.size == 0:
        return [], {}

    detected_list = []
    try:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = get_color_mask(hsv, target_color)
        if mask is None:
            return [], {}

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
            if area < 150: continue # ตัดขอบเล็กๆ ทิ้ง

            x, y, bw, bh = cv2.boundingRect(cnt)
            if bw == 0 or bh == 0: continue

            # % Pixel Fill Ratio ป้องกันเงามั่ว
            bbox_mask = mask_clean[y:y+bh, x:x+bw]
            color_pixel_count = cv2.countNonZero(bbox_mask)
            fill_ratio = (color_pixel_count / float(bw * bh)) * 100.0

            if fill_ratio < MIN_PIXEL_FILL_RATIO:
                continue 

            # ----------------------------------------------------
            # 📐 GEOMETRIC SHAPE CLASSIFICATION (คณิตศาสตร์เรขาคณิต)
            # ----------------------------------------------------
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0: continue

            # 1. ประมาณจุดยอดมุม (Polygon Approximation)
            approx = cv2.approxPolyDP(cnt, 0.03 * perimeter, True)
            num_vertices = len(approx)

            # 2. คำนวณ Extent Ratio (พื้นที่จริง / พื้นที่ Bounding Box)
            extent = float(area) / (bw * bh)

            # 3. คำนวณ Aspect Ratio (ความกว้าง / ความสูง)
            aspect_ratio = float(bw) / float(bh)

            subtype = None

            # 🛑 แยกประเภทเรขาคณิตเด็ดขาด:
            # - ถ้ามีจุดยอด 4 มุม และ Extent สูง (> 0.80) -> เป็น "สี่เหลี่ยม" แน่นอน
            # - ถ้าจุดยอดเยอะ (> 5 มุม) และ Extent อยู่ช่วง (~0.65-0.82) -> เป็น "วงกลม"
            if num_vertices >= 5 and extent < 0.83:
                subtype = "CIRCLE"
            elif num_vertices == 4 or extent >= 0.80:
                # 📐 แยกประเภทสี่เหลี่ยมด้วยอัตราส่วน Aspect Ratio (w / h)
                if 0.85 <= aspect_ratio <= 1.18:
                    subtype = "SQUARE"
                elif aspect_ratio > 1.18:
                    subtype = "RECT_H"
                elif aspect_ratio < 0.85:
                    subtype = "RECT_V"
            else:
                # Fallback สี่เหลี่ยมกรณีขอบเบลอ
                subtype = "SQUARE"

            real_w_cm = (bw * distance_cm) / CAMERA_FOCAL_LENGTH_PX if distance_cm < 300 else 7.0
            real_h_cm = (bh * distance_cm) / CAMERA_FOCAL_LENGTH_PX if distance_cm < 300 else 7.0

            # ----------------------------------------------------
            # 🌟 OUTPUT RESULTS
            # ----------------------------------------------------
            if not strict_mode:
                detected_list.append({
                    "color": target_color, "shape": subtype,
                    "bbox": (x, y, bw, bh),
                    "cx": x + (bw // 2), "cy": y + (bh // 2),
                    "fill_ratio": fill_ratio,
                    "real_w_cm": real_w_cm, "real_h_cm": real_h_cm
                })
            else:
                if target_shape_type == "ANY" or subtype == target_shape_type:
                    detected_list.append({
                        "color": target_color, "shape": subtype,
                        "bbox": (x, y, bw, bh),
                        "cx": x + (bw // 2), "cy": y + (bh // 2),
                        "fill_ratio": fill_ratio,
                        "real_w_cm": real_w_cm, "real_h_cm": real_h_cm
                    })

        return detected_list, {"mask": mask_clean, "sobel": sobel_edges}
    except Exception:
        pass
    return [], {}

# ==========================================
# 4. WORKER THREAD (FAST TARGET HEAD -> SLOW CENTER LOCK)
# ==========================================
def slam_scan_worker(ep_gimbal, ep_camera, target_color, target_shape):
    try:
        with hub.lock:
            hub.is_scanning = True
            hub.status_msg = "PHASE 1: MULTI-TARGET SWEEP..."

        print(f"\n==================================================")
        print(f"🔄 [PHASE 1] หมุนกวาดเร็ว 4 ทิศทาง (เป้าหมาย: {target_color.upper()} | {target_shape})")
        print("==================================================")
        
        scan_seq = [('AHEAD', 0), ('RIGHT', 90), ('BACK', 180), ('LEFT', -90)]
        scan_queue = queue.Queue()

        for dir_name, rel_y in scan_seq:
            if not hub.running: break
            print(f"👉 [SWEEP] หมุนเร็วไปทิศทาง {dir_name} ({rel_y} deg)...")

            safe_gimbal_move(ep_gimbal, pitch=-3.5, yaw=rel_y, speed=GIMBAL_FAST_SPEED, wait_sec=0.20)

            with hub.lock:
                img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                tof_dist = hub.tof_calibrated_cm

            if img is not None:
                targets_in_frame, _ = process_frame_multi_targets(img, tof_dist, target_color, target_shape, strict_mode=False)

                if targets_in_frame:
                    h, w, _ = img.shape
                    print(f"📸 พบเป้าหมายน่าสงสัยในทิศ {dir_name} จำนวน {len(targets_in_frame)} ชิ้น!")

                    for idx, item in enumerate(targets_in_frame, 1):
                        offset_x_px = item["cx"] - (w // 2)
                        deg_offset = (offset_x_px / CAMERA_FOCAL_LENGTH_PX) * (180.0 / math.pi)
                        item_yaw = rel_y + deg_offset

                        scan_queue.put({"dir_name": dir_name, "yaw": item_yaw})
                        print(f"   ├─ ชิ้นที่ {idx}: cx={item['cx']}px (Fill: {item['fill_ratio']:.1f}%) -> ดันมุม {item_yaw:.1f} deg เข้า Queue")

        if scan_queue.empty():
            print("\nℹ️ [SWEEP COMPLETE] หมุนกวาดรอบแรกไม่พบเป้าหมายน่าสงสัย")
        else:
            print(f"\n==================================================")
            print(f"🎯 [PHASE 2 & 3] ทยอยดึงคิวสแกนทีละชิ้น (มีทั้งหมด {scan_queue.qsize()} จุดใน Q)")
            print("==================================================")

            target_index = 0
            while not scan_queue.empty() and hub.running:
                target_index += 1
                item = scan_queue.get()
                target_yaw = item["yaw"]
                current_pitch = -3.5 
                dir_name = item["dir_name"]

                print(f"\n🔍 [QUEUE Item {target_index}] ⚡ หมุนเร็วไปที่มุม Yaw: {target_yaw:.1f} deg...")
                safe_gimbal_move(ep_gimbal, pitch=current_pitch, yaw=target_yaw, speed=GIMBAL_FAST_SPEED, wait_sec=0.20)

                with hub.lock:
                    img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                    tof_dist = hub.tof_calibrated_cm

                if img is not None:
                    targets_now, _ = process_frame_multi_targets(img, tof_dist, target_color, target_shape, strict_mode=True)
                    if targets_now:
                        h, w, _ = img.shape
                        closest_item = min(targets_now, key=lambda t: math.hypot(t["cx"] - (w // 2), t["cy"] - (h // 2)))
                        
                        offset_x_px = closest_item["cx"] - (w // 2)
                        offset_y_px = closest_item["cy"] - (h // 2)
                        
                        deg_offset_yaw = (offset_x_px / CAMERA_FOCAL_LENGTH_PX) * (180.0 / math.pi)
                        deg_offset_pitch = -(offset_y_px / CAMERA_FOCAL_LENGTH_PX) * (180.0 / math.pi)

                        target_yaw = target_yaw + deg_offset_yaw
                        current_pitch = np.clip(current_pitch + deg_offset_pitch, -15.0, 15.0)

                        print(f"🎯 [CENTER LOCK] 🐢 ขยับ Gimbal ช้าๆ ปรับเข้ากลางวัตถุ -> Yaw: {target_yaw:.1f}° | Pitch: {current_pitch:.1f}°")
                        safe_gimbal_move(ep_gimbal, pitch=current_pitch, yaw=target_yaw, speed=40, wait_sec=0.25)

                print("⏳ [WARM-UP] รอกล้องและ ToF เล็งนิ่งกลางวัตถุ 3 เฟรม...")
                warmup_start = time.time()
                consecutive_hits = 0

                while time.time() - warmup_start < 0.50:
                    with hub.lock:
                        w_img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                        w_tof = hub.tof_calibrated_cm
                    
                    if w_img is not None:
                        w_targets, _ = process_frame_multi_targets(w_img, w_tof, target_color, target_shape, strict_mode=True)
                        if w_targets:
                            consecutive_hits += 1
                            if consecutive_hits >= 3:
                                print("✅ [WARM-UP SUCCESS] นิ่งสนิทกลางเป้าแล้ว!")
                                break
                        else:
                            consecutive_hits = 0
                    time.sleep(0.03)

                print("📷 [10-FRAME SAMPLING] จ้องนิ่งเพื่อเก็บค่าระยะ ToF...")
                valid_samples = []
                tof_samples = []
                REQUIRED_MATCH_FRAMES = 7

                for frame_idx in range(1, 11):
                    time.sleep(0.04)
                    with hub.lock:
                        sample_img = hub.latest_frame.copy() if hub.latest_frame is not None else None
                        sample_tof = hub.tof_calibrated_cm

                    if sample_img is not None:
                        s_targets, _ = process_frame_multi_targets(sample_img, sample_tof, target_color, target_shape, strict_mode=True)
                        if s_targets:
                            s_res = s_targets[0]
                            valid_samples.append((s_res["color"], s_res["shape"]))
                            tof_samples.append(sample_tof)
                            print(f"   ├─ Frame {frame_idx:02d}/10: ✅ PASS -> {s_res['color'].upper()} {s_res['shape']} (ToF Target: {sample_tof:.1f}cm)")
                        else:
                            print(f"   ├─ Frame {frame_idx:02d}/10: ❌ FAIL")

                success_count = len(valid_samples)
                print(f"📊 สรุปผลสแกน 10 เฟรม: ผ่าน {success_count}/10 เฟรม")

                if success_count >= REQUIRED_MATCH_FRAMES:
                    most_common = Counter(valid_samples).most_common(1)[0][0]
                    conf_color, conf_shape = most_common
                    tof_final = float(np.median(tof_samples)) if tof_samples else hub.tof_calibrated_cm
                    hub.register_target_spatial(abs_yaw_deg=target_yaw, tof_dist_cm=tof_final, color=conf_color, shape=conf_shape)
                else:
                    print("❌ [FALSE ALARM REJECTED] ตรวจจับผ่านน้อยกว่า 7 เฟรม!")

                scan_queue.task_done()

        safe_gimbal_move(ep_gimbal, pitch=0, yaw=0, speed=GIMBAL_FAST_SPEED, wait_sec=0.2)

    except Exception as err:
        print(f"\n❌ [SCANNER ERROR NOTICE]: {err}")

    finally:
        with hub.lock:
            hub.is_scanning = False
            hub.status_msg = "IDLE (Ready for next R)"
            print("\n🟢 [READY] สแกนรอบนี้เสร็จสิ้น! สามารถกดปุ่ม 'R' เพื่อเริ่มสแกนรอบใหม่ได้ทันที")

# ==========================================
# 5. USER MENU & SAFE MAIN ENTRY
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
    s_choice = input("👉 เลือกทรง [1-5] (Default=4 CIRCLE): ").strip()
    shape_map = {"1": "RECT_V", "2": "RECT_H", "3": "SQUARE", "4": "CIRCLE", "5": "ANY"}
    target_shape = shape_map.get(s_choice, "CIRCLE")

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
    print(" [ M ] : สลับโหมดภาพ Debug (NORMAL -> SOBEL -> MASK)")
    print(" [ R ] : สั่งเริ่ม Multi-Target Queue Scan (กดได้เรื่อยๆ ไม่จบโปรแกรม)")
    print(" [ Q ] : บันทึก Map_Data_Round1.json และปิดระบบ")
    print("--------------------------------------------------\n")

    try:
        while hub.running:
            try:
                img = ep_camera.read_cv2_image(strategy="newest")
                if img is None or not isinstance(img, np.ndarray) or img.size == 0: 
                    time.sleep(0.01)
                    continue

                with hub.lock:
                    hub.latest_frame = img.copy()
                    current_tof = hub.tof_calibrated_cm
                    debug_m = hub.debug_mode
                    is_scan = hub.is_scanning

                detected_targets, debug_views = process_frame_multi_targets(
                    img, current_tof, target_color=target_color, target_shape_type=target_shape, strict_mode=True
                )

                display_img = img.copy()
                
                # 🌟 เซฟตีป้องกัน OpenCV GUI Crash ตอนสลับ View
                if debug_m == "SOBEL" and "sobel" in debug_views and debug_views["sobel"] is not None:
                    display_img = cv2.cvtColor(debug_views["sobel"], cv2.COLOR_GRAY2BGR)
                elif debug_m == "MASK" and "mask" in debug_views and debug_views["mask"] is not None:
                    display_img = cv2.cvtColor(debug_views["mask"], cv2.COLOR_GRAY2BGR)

                for item in detected_targets:
                    x, y, bw, bh = item["bbox"]
                    cv2.rectangle(display_img, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                    lbl = f"MATCH: {item['color'].upper()} {item['shape']} (Fill: {item['fill_ratio']:.0f}%)"
                    cv2.putText(display_img, lbl, (x, max(15, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                scan_str = "SCANNING & ALIGNING..." if is_scan else "IDLE (Press 'R')"
                cv2.putText(display_img, f"STATUS: {scan_str}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255) if is_scan else (0, 255, 0), 2)
                cv2.putText(display_img, f"TARGET: {target_color.upper()} | {target_shape}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (241, 196, 15), 1)
                cv2.putText(display_img, f"ToF Calibrated: {current_tof:.1f} cm", (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

                cv2.imshow("Multi-Target SLAM Scanner", display_img)

                key = cv2.waitKey(20) & 0xFF
                if key in (ord('q'), ord('Q')):
                    break
                elif key in (ord('m'), ord('M')):
                    modes = ["NORMAL", "SOBEL", "MASK"]
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