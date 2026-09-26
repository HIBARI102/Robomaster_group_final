import threading
import time
import math
import logging
import sys
import csv
import heapq
from collections import deque
import numpy as np
import pygame
from robomaster import robot
import json
import os

# ==========================================
# 1. SETUP UTF-8 LOGGING & CSV TELEMETRY
# ==========================================
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

file_handler = logging.FileHandler('exploration_log.txt', mode='w', encoding='utf-8')
stream_handler = logging.StreamHandler(sys.stdout)

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S',
    handlers=[stream_handler, file_handler]
)

recent_logs = deque(maxlen=5)

def log_event(msg):
    logging.info(msg)
    recent_logs.append(msg)
    file_handler.flush()

TOF_A = 0.0
TOF_B = 1.0
TOF_C = 0.0

if os.path.exists("tof_calib_params.json"):
    try:
        with open("tof_calib_params.json", "r", encoding="utf-8") as f:
            calib_data = json.load(f)
            TOF_A = calib_data.get("a", 0.0)
            TOF_B = calib_data.get("b", calib_data.get("slope", 1.0))
            TOF_C = calib_data.get("c", calib_data.get("intercept", 0.0))
            log_event(f"[TOF CALIB LOADED] Degree 2: Real = ({TOF_A:.6f}*Raw^2) + ({TOF_B:.4f}*Raw) + ({TOF_C:.4f})")
    except Exception as e:
        log_event(f"Failed to load ToF calibration file: {e}")

def normalize_angle(deg):
    return (deg + 180.0) % 360.0 - 180.0

def snap_to_cardinal_yaw(raw_yaw):
    norm_yaw = normalize_angle(raw_yaw)
    snapped = round(norm_yaw / 90.0) * 90.0
    return normalize_angle(snapped)

CSV_FILE = 'exploration_telemetry.csv'
with open(CSV_FILE, mode='w', newline='', encoding='utf-8') as f:
    writer = csv.writer(f)
    writer.writerow([
        'Timestamp', 'Step', 'Grid_X', 'Grid_Y', 'Heading_Deg',
        'ToF_Ahead_cm', 'ToF_Right_cm', 'ToF_Back_cm', 'ToF_Left_cm',
        'Wall_N', 'Wall_E', 'Wall_S', 'Wall_W'
    ])

def log_telemetry_csv(step, gx, gy, heading, tof_dict, wall_dict):
    timestamp = time.strftime('%H:%M:%S')
    with open(CSV_FILE, mode='a', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerow([
            timestamp, step, gx, gy, f"{heading:.1f}",
            f"{tof_dict.get('AHEAD', -1):.1f}",
            f"{tof_dict.get('RIGHT', -1):.1f}",
            f"{tof_dict.get('BACK', -1):.1f}",
            f"{tof_dict.get('LEFT', -1):.1f}",
            wall_dict.get('N', False),
            wall_dict.get('E', False),
            wall_dict.get('S', False),
            wall_dict.get('W', False)
        ])

# ==========================================
# 2. CONFIG & PHYSICAL PARAMETERS
# ==========================================
GRID_W = int(input("ความกว้างสนาม (จำนวนช่อง X): "))
GRID_H = int(input("ความยาวสนาม (จำนวนช่อง Y): "))
START_X = int(input("จุดเริ่มต้น X (grid index 0-based): "))
START_Y = int(input("จุดเริ่มต้น Y (grid index 0-based): "))
INIT_DIR = input("ทิศหันหน้าเริ่มต้น (N, E, S, W): ").upper()

# 🌟 GIMBAL CONFIG SPEED (320 deg/s หมุนนุ่มนวล)
GIMBAL_FAST_SPEED = 320    

DIR_MAP = {'N': 0.0, 'E': 90.0, 'S': -180.0, 'W': -90.0}
INITIAL_YAW_DEG = DIR_MAP.get(INIT_DIR, 0.0)

CELL_SIZE_CM = 60.0
WALL_THRESHOLD_CM = 52.0

ROBOT_LENGTH_CM = 33.0
ROBOT_WIDTH_CM = 25.0
TOF_OFFSET_X_CM = 7.5
ROBOT_FRONT_LIMIT_CM = (ROBOT_LENGTH_CM / 2.0) - TOF_OFFSET_X_CM 

# 🌟 เกณฑ์ Safety Brake
SAFE_MIN_TOF = 16.0 + ROBOT_FRONT_LIMIT_CM     
BRAKE_TRIGGER_TOF = 10.0 + ROBOT_FRONT_LIMIT_CM

move_target_step_m = 0.60
target_speed_mps = 0.24  # กำหนดสปีดเดินหลักตรงนี้

# 🌟 ระบบโหลด DISTANCE CALIBRATION PARAMS ตามความเร็วที่ใช้งาน
DISTANCE_CALIB_FACTOR = 1.0
speed_key = f"{target_speed_mps:.2f}"

if os.path.exists("distance_calib_params.json"):
    try:
        with open("distance_calib_params.json", "r", encoding="utf-8") as f:
            dist_calib_db = json.load(f)
            if speed_key in dist_calib_db:
                DISTANCE_CALIB_FACTOR = float(dist_calib_db[speed_key])
                log_event(f"[DIST CALIB LOADED] Loaded Factor = {DISTANCE_CALIB_FACTOR:.4f} for speed {speed_key} m/s")
            else:
                log_event(f"[DIST CALIB WARNING] Speed {speed_key} m/s not found in calib file. Using default factor = 1.0")
    except Exception as e:
        log_event(f"[DIST CALIB ERROR] Failed to load file: {e}. Using default factor = 1.0")
else:
    log_event("[DIST CALIB NOT FOUND] 'distance_calib_params.json' not found. Using theoretical calculation (factor = 1.0)")

# ==========================================
# 3. SHARED HUB & BAYESIAN MAPPING
# ==========================================
class SharedHub:
    def __init__(self, w, h, sx, sy):
        self.lock = threading.Lock()
        self.w = w
        self.h = h
        
        self.visited = np.zeros((h, w), dtype=int)
        self.N_wall_logodds = np.full((h, w), -2.0)
        self.E_wall_logodds = np.full((h, w), -2.0)
        
        self.S_outer_wall_logodds = np.full(w, 4.0)
        self.W_outer_wall_logodds = np.full(h, 4.0)
        self.N_wall_logodds[h-1, :] = 4.0
        self.E_wall_logodds[:, w-1] = 4.0
        
        self.N_wall_locked = np.zeros((h, w), dtype=bool)
        self.E_wall_locked = np.zeros((h, w), dtype=bool)
        
        self.grid_x = sx
        self.grid_y = sy
        
        self.pose = {
            'x_cm': (sx + 0.5) * CELL_SIZE_CM,
            'y_cm': (sy + 0.5) * CELL_SIZE_CM,
            'chassis_yaw': INITIAL_YAW_DEG
        }
        
        self.raw_init_x_m = None
        self.raw_init_y_m = None
        self.yaw_offset_deg = 0.0
        self.is_calibrated = False
        
        self.last_turn_dir = 1 
        self.latest_raw_pos = [None, None]
        self.latest_raw_yaw = None
        
        self.tof_raw = 0.0
        self.tof_filtered = 0.0
        self.tof_last_update_time = 0.0
        self.gimbal_yaw = 0.0
        self.is_aligning = False
        self.is_scanning = False
        self.is_rotating = False
        self.emergency_stop = False
        self.running = True
        self.status_msg = "Initializing..."
        
        self.visited[sy][sx] = 1

    def set_scanning(self, state: bool):
        with self.lock:
            self.is_scanning = state

    def set_aligning(self, state: bool):
        with self.lock:
            self.is_aligning = state

    def lock_passage(self, cell1, cell2):
        x1, y1 = cell1
        x2, y2 = cell2
        with self.lock:
            if x2 == x1 and y2 == y1 + 1:
                self.N_wall_logodds[y1, x1] = -4.0
                self.N_wall_locked[y1, x1] = True
            elif x2 == x1 + 1 and y2 == y1:
                self.E_wall_logodds[y1, x1] = -4.0
                self.E_wall_locked[y1, x1] = True
            elif x2 == x1 and y2 == y1 - 1 and y2 >= 0:
                self.N_wall_logodds[y2, x2] = -4.0
                self.N_wall_locked[y2, x2] = True
            elif x2 == x1 - 1 and y2 == y1 and x2 >= 0:
                self.E_wall_logodds[y2, x2] = -4.0
                self.E_wall_locked[y2, x2] = True

    def update_wall_prob(self, gx, gy, direction, is_wall):
        with self.lock:
            if not (0 <= gx < self.w and 0 <= gy < self.h):
                return
            L_OCC = 3.0
            L_FREE = -1.5  
            
            if direction == 'N':
                if self.N_wall_locked[gy, gx]: return
                if gy == self.h - 1 and not is_wall:
                    self.N_wall_logodds[gy, gx] = -4.0
                else:
                    self.N_wall_logodds[gy, gx] = np.clip(self.N_wall_logodds[gy, gx] + (L_OCC if is_wall else L_FREE), -4.0, 4.0)
            elif direction == 'E':
                if self.E_wall_locked[gy, gx]: return
                if gx == self.w - 1 and not is_wall:
                    self.E_wall_logodds[gy, gx] = -4.0
                else:
                    self.E_wall_logodds[gy, gx] = np.clip(self.E_wall_logodds[gy, gx] + (L_OCC if is_wall else L_FREE), -4.0, 4.0)
            elif direction == 'S':
                if gy > 0:
                    if self.N_wall_locked[gy - 1, gx]: return
                    self.N_wall_logodds[gy - 1, gx] = np.clip(self.N_wall_logodds[gy - 1, gx] + (L_OCC if is_wall else L_FREE), -4.0, 4.0)
                else:
                    if not is_wall:
                        self.S_outer_wall_logodds[gx] = -4.0
                    else:
                        self.S_outer_wall_logodds[gx] = np.clip(self.S_outer_wall_logodds[gx] + L_OCC, -4.0, 4.0)
            elif direction == 'W':
                if gx > 0:
                    if self.E_wall_locked[gy, gx - 1]: return
                    self.E_wall_logodds[gy, gx - 1] = np.clip(self.E_wall_logodds[gy, gx - 1] + (L_OCC if is_wall else L_FREE), -4.0, 4.0)
                else:
                    if not is_wall:
                        self.W_outer_wall_logodds[gy] = -4.0
                    else:
                        self.W_outer_wall_logodds[gy] = np.clip(self.W_outer_wall_logodds[gy] + L_OCC, -4.0, 4.0)

    def clean_outer_predicted_walls(self):
        with self.lock:
            for x in range(self.w):
                if self.N_wall_logodds[self.h - 1, x] <= 2.0:
                    self.N_wall_logodds[self.h - 1, x] = -4.0
            for y in range(self.h):
                if self.E_wall_logodds[y, self.w - 1] <= 2.0:
                    self.E_wall_logodds[y, self.w - 1] = -4.0
            for x in range(self.w):
                if self.S_outer_wall_logodds[x] <= 2.0:
                    self.S_outer_wall_logodds[x] = -4.0
            for y in range(self.h):
                if self.W_outer_wall_logodds[y] <= 2.0:
                    self.W_outer_wall_logodds[y] = -4.0
        log_event("[MAP CLEAN] Cleared outer unconfirmed walls.")

    def has_wall_between(self, cell1, cell2):
        x1, y1 = cell1
        x2, y2 = cell2
        if x2 == x1 and y2 == y1 + 1:    return self.N_wall_logodds[y1, x1] > 0.0 if y1 < self.h else True
        elif x2 == x1 + 1 and y2 == y1:  return self.E_wall_logodds[y1, x1] > 0.0 if x1 < self.w else True
        elif x2 == x1 and y2 == y1 - 1:  return self.N_wall_logodds[y2, x2] > 0.0 if y2 >= 0 else True
        elif x2 == x1 - 1 and y2 == y1:  return self.E_wall_logodds[y2, x2] > 0.0 if x2 >= 0 else True
        return True

    def set_grid_pos(self, gx, gy):
        with self.lock:
            self.grid_x = gx
            self.grid_y = gy
            self.pose['x_cm'] = (gx + 0.5) * CELL_SIZE_CM
            self.pose['y_cm'] = (gy + 0.5) * CELL_SIZE_CM
            if 0 <= gy < self.h and 0 <= gx < self.w:
                self.visited[gy][gx] = 1

    def get_state(self):
        with self.lock:
            return {
                'visited': self.visited.copy(),
                'N_confirmed': self.N_wall_logodds > 2.0,
                'N_predicted': (self.N_wall_logodds > 0.0) & (self.N_wall_logodds <= 2.0),
                'E_confirmed': self.E_wall_logodds > 2.0,
                'E_predicted': (self.E_wall_logodds > 0.0) & (self.E_wall_logodds <= 2.0),
                'S_outer_confirmed': self.S_outer_wall_logodds > 2.0,
                'S_outer_predicted': (self.S_outer_wall_logodds > 0.0) & (self.S_outer_wall_logodds <= 2.0),
                'W_outer_confirmed': self.W_outer_wall_logodds > 2.0,
                'W_outer_predicted': (self.W_outer_wall_logodds > 0.0) & (self.W_outer_wall_logodds <= 2.0),
                'pose': self.pose.copy(),
                'grid_x': self.grid_x,
                'grid_y': self.grid_y,
                'tof_raw': self.tof_raw,
                'filtered_tof': self.tof_filtered,
                'gimbal_yaw': self.gimbal_yaw,
                'is_aligning': self.is_aligning,
                'is_scanning': self.is_scanning,
                'is_rotating': self.is_rotating,
                'is_calibrated': self.is_calibrated,
                'emergency_stop': self.emergency_stop,
                'running': self.running,
                'status_msg': self.status_msg
            }

    def set_status_msg(self, msg):
        with self.lock:
            self.status_msg = msg

    def set_raw_data(self, pos_xy, yaw_deg):
        with self.lock:
            if pos_xy[0] is not None: self.latest_raw_pos[0] = pos_xy[0]
            if pos_xy[1] is not None: self.latest_raw_pos[1] = pos_xy[1]
            if yaw_deg is not None: self.latest_raw_yaw = yaw_deg

    def calibrate_origin(self):
        timeout = 5.0
        start_t = time.time()
        while time.time() - start_t < timeout:
            with self.lock:
                if self.latest_raw_pos[0] is not None and self.latest_raw_yaw is not None:
                    self.raw_init_x_m = self.latest_raw_pos[0]
                    self.raw_init_y_m = self.latest_raw_pos[1]
                    self.pose['chassis_yaw'] = INITIAL_YAW_DEG
                    raw_yaw_deg = self.latest_raw_yaw
                    self.yaw_offset_deg = (INITIAL_YAW_DEG - raw_yaw_deg) % 360.0
                    self.is_calibrated = True
                    return True
            time.sleep(0.02)
        return False

    def stop_workers(self):
        with self.lock:
            self.running = False

hub = SharedHub(GRID_W, GRID_H, START_X, START_Y)
tof_buffer = deque(maxlen=5)

# ==========================================
# 🌟 HELPER FUNCTION: ACTIVE BRAKING
# ==========================================
def stop_chassis_active_brake(ep_chassis, brake_duration=0.10):
    """สั่ง Active Brake ล็อกล้อและตัดแรง inertia ป้องกันการไถล"""
    try:
        ep_chassis.drive_speed(x=0, y=0, z=0)
        time.sleep(brake_duration)
        ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
    except Exception:
        pass

# ==========================================
# 4. SENSOR CALLBACKS
# ==========================================
def tof_data_handler(sub_info):
    try:
        if sub_info and len(sub_info) > 0 and sub_info[0] is not None:
            raw_dist_cm = float(sub_info[0]) / 10.0
            if raw_dist_cm <= 0:
                calibrated_dist_cm = 999.0
            else:
                calibrated_dist_cm = (TOF_A * (raw_dist_cm ** 2)) + (TOF_B * raw_dist_cm) + TOF_C
                calibrated_dist_cm = max(0.0, calibrated_dist_cm)
        else:
            calibrated_dist_cm = 999.0
    except (ValueError, TypeError):
        calibrated_dist_cm = 999.0

    with hub.lock:
        hub.tof_raw = calibrated_dist_cm
        hub.tof_last_update_time = time.time()
        tof_buffer.append(calibrated_dist_cm)
        hub.tof_filtered = float(np.median(list(tof_buffer))) if tof_buffer else calibrated_dist_cm
        
        if hub.is_aligning or hub.is_rotating or hub.is_scanning:
            hub.emergency_stop = False
        else:
            if 0.5 < calibrated_dist_cm < BRAKE_TRIGGER_TOF:
                hub.emergency_stop = True
            elif calibrated_dist_cm >= SAFE_MIN_TOF:
                hub.emergency_stop = False

def gimbal_data_handler(sub_info):
    if sub_info and len(sub_info) > 1 and sub_info[1] is not None:
        try:
            with hub.lock:
                hub.gimbal_yaw = float(sub_info[1])
        except (ValueError, TypeError):
            pass

def chassis_pos_handler(sub_info):
    if sub_info and len(sub_info) >= 2 and sub_info[0] is not None and sub_info[1] is not None:
        try:
            current_yaw = hub.latest_raw_yaw if hub.latest_raw_yaw is not None else 0.0
            hub.set_raw_data([float(sub_info[0]), float(sub_info[1])], current_yaw)
        except (ValueError, TypeError):
            pass

def chassis_att_handler(sub_info):
    if sub_info and len(sub_info) >= 2 and sub_info[1] is not None:
        try:
            pos = hub.latest_raw_pos if hub.latest_raw_pos[0] is not None else [0.0, 0.0]
            hub.set_raw_data(pos, float(sub_info[1]))
        except (ValueError, TypeError):
            pass

# ==========================================
# 5. GIMBAL SCANNING & RECENTER LOGIC
# ==========================================
def reset_gimbal_to_center_fast(ep_gimbal):
    """ฟังก์ชันสั่ง Gimbal กลับหน้าตรงด้วยสปีด 320 deg/s"""
    try:
        ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=GIMBAL_FAST_SPEED, yaw_speed=GIMBAL_FAST_SPEED).wait_for_completed(timeout=0.8)
    except Exception as e:
        log_event(f"[GIMBAL RESET ERROR] {e}")

def deg_to_cardinal_exact(abs_deg):
    norm_deg = normalize_angle(abs_deg)
    if -45.0 <= norm_deg <= 45.0:
        return 'N'
    elif 45.0 < norm_deg <= 135.0:
        return 'E'
    elif norm_deg > 135.0 or norm_deg < -135.0:
        return 'S'
    elif -135.0 <= norm_deg < -45.0:
        return 'W'
    return 'N'

def get_fresh_tof_distance(ep_gimbal, target_yaw, pitch=0):
    """หมุน Gimbal -> หน่วงนิ่งสนิท -> ล้าง Buffer -> ดึงค่าสด 5 ค่า"""
    move_start_t = time.time()
    try:
        ep_gimbal.moveto(pitch=pitch, yaw=target_yaw, pitch_speed=GIMBAL_FAST_SPEED, yaw_speed=GIMBAL_FAST_SPEED).wait_for_completed(timeout=1.0)
    except Exception as e:
        log_event(f"[GIMBAL MOVE TIMEOUT] {e}")
        
    time.sleep(0.10)
    
    with hub.lock:
        tof_buffer.clear()
    
    fresh_samples = []
    start_t = time.time()
    
    while len(fresh_samples) < 5 and (time.time() - start_t < 0.22):
        with hub.lock:
            if hub.tof_last_update_time > move_start_t:
                val = hub.tof_raw
                if 2.0 <= val <= 300.0:
                    fresh_samples.append(val)
        time.sleep(0.015)
        
    if fresh_samples:
        return float(np.median(fresh_samples))
        
    return 999.0

def scan_4_directions(ep_gimbal, current_gx, current_gy, snap_yaw):
    """สแกน 4 ทิศทางด้วยความเร็วที่แม่นยำ"""
    hub.set_scanning(True)
    hub.set_status_msg("Scanning 4 Directions...")
    
    scan_seq = [('AHEAD', 0), ('RIGHT', 90), ('BACK', 180), ('LEFT', -90)]
    tof_results = {}
    
    for name, rel_y in scan_seq:
        dist_cm = get_fresh_tof_distance(ep_gimbal, target_yaw=rel_y, pitch=0)
        
        if (WALL_THRESHOLD_CM - 6.0) < dist_cm < (WALL_THRESHOLD_CM + 6.0):
            dist_down = get_fresh_tof_distance(ep_gimbal, target_yaw=rel_y, pitch=-5)
            dist_cm = min(dist_cm, dist_down)
            
        tof_results[name] = dist_cm
        
        abs_dir_deg = normalize_angle(snap_yaw + rel_y)
        w_dir = deg_to_cardinal_exact(abs_dir_deg)
        
        is_wall_detected = (dist_cm < WALL_THRESHOLD_CM)
        hub.update_wall_prob(current_gx, current_gy, w_dir, is_wall=is_wall_detected)

    reset_gimbal_to_center_fast(ep_gimbal)
    time.sleep(0.05)
    hub.set_scanning(False)
    return tof_results

def check_and_recenter(ep_chassis, ep_gimbal, tof_dict):
    """จัดตำแหน่งกึ่งกลางด้วย Deadband 3.0 cm และสั่ง Active Brake เมื่อขยับเสร็จ"""
    hub.set_aligning(True)
    ideal_dist = 20.0  
    max_detect_dist = 45.0  
    
    dist_ahead = tof_dict.get('AHEAD', 999.0)
    dist_right = tof_dict.get('RIGHT', 999.0)
    dist_back = tof_dict.get('BACK', 999.0)
    dist_left = tof_dict.get('LEFT', 999.0)
    
    shift_x = 0.0
    shift_y = 0.0
    
    if dist_right < max_detect_dist and dist_left < max_detect_dist:
        center_err_y = (dist_right - dist_left) / 2.0
        if abs(center_err_y) > 3.0:
            shift_y = np.clip(center_err_y / 100.0, -0.05, 0.05)
    elif dist_right < max_detect_dist:
        err_y = dist_right - ideal_dist
        if abs(err_y) > 3.0:
            shift_y = np.clip(err_y / 100.0, -0.05, 0.05)
    elif dist_left < max_detect_dist:
        err_y = dist_left - ideal_dist
        if abs(err_y) > 3.0:
            shift_y = np.clip(-err_y / 100.0, -0.05, 0.05)

    if dist_ahead < max_detect_dist and dist_back < max_detect_dist:
        center_err_x = (dist_ahead - dist_back) / 2.0
        if abs(center_err_x) > 3.0:
            shift_x = np.clip(center_err_x / 100.0, -0.05, 0.05)
    elif dist_ahead < max_detect_dist:
        err_x = dist_ahead - ideal_dist
        if abs(err_x) > 3.0:
            shift_x = np.clip(err_x / 100.0, -0.05, 0.05)
    elif dist_back < max_detect_dist:
        err_x = dist_back - ideal_dist
        if abs(err_x) > 3.0:
            shift_x = np.clip(-err_x / 100.0, -0.05, 0.05)
            
    has_moved = False
    if abs(shift_x) > 0.015 or abs(shift_y) > 0.015:
        log_event(f"[CENTERING] Adjusting Position: x={shift_x:.2f}m, y={shift_y:.2f}m")
        try:
            ep_chassis.move(x=shift_x, y=shift_y, z=0, xy_speed=0.15).wait_for_completed(timeout=2.0)
            stop_chassis_active_brake(ep_chassis, brake_duration=0.08)
            has_moved = True
        except Exception as e:
            log_event(f"[CENTERING TIMEOUT] {e}")
            
        time.sleep(0.08)
        reset_gimbal_to_center_fast(ep_gimbal)
        time.sleep(0.08)

    hub.set_aligning(False)
    return has_moved

# ==========================================
# 6. ODOMETRY + NAVIGATION CONTROL
# ==========================================
def turn_to_orientation_imu(ep_chassis, ep_gimbal, target_yaw_deg):
    with hub.lock:
        hub.is_rotating = True
        hub.emergency_stop = False
    
    target_yaw_deg = normalize_angle(target_yaw_deg)
    
    with hub.lock:
        curr_yaw = hub.pose['chassis_yaw']
        
    diff = normalize_angle(target_yaw_deg - curr_yaw)
    
    if abs(abs(diff) - 180.0) < 5.0:
        with hub.lock:
            hub.last_turn_dir *= -1
            turn_dir = hub.last_turn_dir
        diff = 180.0 * turn_dir

    if abs(diff) <= 2.5:
        with hub.lock:
            hub.is_rotating = False
        return True

    log_event(f"[TURN START] Rotating {diff:.1f} deg to Target Yaw: {target_yaw_deg:.0f} deg")
    hub.set_status_msg(f"Rotating relative {diff:.0f} deg")
    
    reset_gimbal_to_center_fast(ep_gimbal)
    
    try:
        ep_chassis.move(x=0, y=0, z=-diff, z_speed=130).wait_for_completed(timeout=3.0)
        time.sleep(0.08)
    except Exception as e:
        log_event(f"[TURN ERROR TIMEOUT] {e}")
    
    stop_chassis_active_brake(ep_chassis, brake_duration=0.08)
    reset_gimbal_to_center_fast(ep_gimbal)
    time.sleep(0.08)
    
    with hub.lock:
        snapped_yaw = snap_to_cardinal_yaw(target_yaw_deg)
        hub.pose['chassis_yaw'] = snapped_yaw
        if hub.latest_raw_yaw is not None:
            hub.yaw_offset_deg = (snapped_yaw - hub.latest_raw_yaw) % 360.0
        hub.is_rotating = False

    log_event(f"[TURN COMPLETED] Settled Yaw: {snapped_yaw:.1f} deg")
    return True

def align_wall_heading_from_tof(ep_chassis, start_tof_side_cm, end_tof_side_cm, side_name):
    if start_tof_side_cm < 45.0 and end_tof_side_cm < 45.0:
        delta_d = end_tof_side_cm - start_tof_side_cm
        travel_dist_cm = 60.0

        angle_error_rad = math.atan2(delta_d, travel_dist_cm)
        angle_error_deg = math.degrees(angle_error_rad)
        
        correct_z_deg = -angle_error_deg if side_name == 'RIGHT' else angle_error_deg

        if 1.0 < abs(correct_z_deg) <= 8.0:
            log_event(f"[WALL ALIGN] Correcting Parallel Drift: {correct_z_deg:.2f} deg using '{side_name}' wall (Delta: {delta_d:.1f}cm)")
            try:
                ep_chassis.move(x=0, y=0, z=correct_z_deg, z_speed=50).wait_for_completed(timeout=2.0)
                stop_chassis_active_brake(ep_chassis, brake_duration=0.08)
                time.sleep(0.08)
            except Exception as e:
                log_event(f"[WALL ALIGN TIMEOUT] {e}")

def a_star_search(start, target, grid_w, grid_h):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, target)}
    
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == target:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)

            if 0 <= nx < grid_w and 0 <= ny < grid_h:
                if not hub.has_wall_between(current, neighbor):
                    tentative_g = g_score[current] + 1
                    if tentative_g < g_score.get(neighbor, float('inf')):
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f_score[neighbor] = tentative_g + heuristic(neighbor, target)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

def find_reachable_frontier(start_x, start_y, grid_w, grid_h):
    state = hub.get_state()
    visited_map = state['visited']
    
    visited = np.zeros_like(visited_map, dtype=bool)
    queue = deque([(start_x, start_y)])
    visited[start_y][start_x] = True
    neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    while queue:
        cx, cy = queue.popleft()

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < grid_w and 0 <= ny < grid_h and not visited[ny][nx]:
                if not hub.has_wall_between((cx, cy), (nx, ny)):
                    visited[ny][nx] = True
                    if visited_map[ny][nx] == 0:
                        target_frontier = (nx, ny)
                        path = a_star_search((start_x, start_y), target_frontier, grid_w, grid_h)
                        if path:
                            return target_frontier, path
                    elif visited_map[ny][nx] == 1:
                        queue.append((nx, ny))
    return None, None

def move_forward_60cm_straight(ep_chassis, target_step_m=move_target_step_m, target_speed=target_speed_mps):
    """
    เดินหน้า 60 cm พร้อม Calibration Factor, Ramp-down 85% และ Active Braking
    """
    # 🌟 คูณ Calibration Factor ปรับระยะเป้าหมายให้ตรงตามโลกจริง
    calibrated_target_m = target_step_m * DISTANCE_CALIB_FACTOR

    with hub.lock:
        start_pos_x = hub.latest_raw_pos[0] if hub.latest_raw_pos[0] is not None else 0.0
        start_pos_y = hub.latest_raw_pos[1] if hub.latest_raw_pos[1] is not None else 0.0
        target_yaw = hub.pose['chassis_yaw']

    Kp, Ki, Kd = 1.2, 0.05, 0.1
    prev_error, integral = 0.0, 0.0
    distance_traveled = 0.0
    max_timeout = (calibrated_target_m / target_speed) + 1.5
    start_time = time.time()
    
    move_successful = True

    while distance_traveled < calibrated_target_m and (time.time() - start_time < max_timeout):
        state = hub.get_state()
        
        if state['emergency_stop']:
            curr_tof = state['tof_raw']
            log_event(f"[SAFETY BRAKE] Obstacle detected ({curr_tof:.1f}cm)! Stopping.")
            move_successful = (distance_traveled >= (calibrated_target_m * 0.70))
            break

        with hub.lock:
            curr_x = hub.latest_raw_pos[0] if hub.latest_raw_pos[0] is not None else start_pos_x
            curr_y = hub.latest_raw_pos[1] if hub.latest_raw_pos[1] is not None else start_pos_y

        distance_traveled = math.sqrt((curr_x - start_pos_x)**2 + (curr_y - start_pos_y)**2)
        curr_yaw = state['pose']['chassis_yaw']
        error = normalize_angle(target_yaw - curr_yaw)

        integral += error * 0.01
        derivative = (error - prev_error) / 0.01
        prev_error = error

        # 🌟 Ramp-down Profile: พอเดินใกล้ถึง 85% ให้ผ่อนความเร็วลงเพื่อตัดแรงไถล
        if distance_traveled >= (calibrated_target_m * 0.85):
            current_exec_speed = 0.10
        else:
            current_exec_speed = target_speed

        z_speed = np.clip((Kp * error) + (Ki * integral) + (Kd * derivative), -30.0, 30.0)
        ep_chassis.drive_speed(x=current_exec_speed, y=0, z=-z_speed)
        time.sleep(0.01)

    # 🌟 Active Brake หยุดล้อล็อกนิ่งทันทีหลังเดินครบรอบ
    stop_chassis_active_brake(ep_chassis, brake_duration=0.10)

    with hub.lock:
        current_yaw = hub.pose['chassis_yaw']
        snapped_yaw = snap_to_cardinal_yaw(current_yaw)
        hub.pose['chassis_yaw'] = snapped_yaw
        if hub.latest_raw_yaw is not None:
            hub.yaw_offset_deg = (snapped_yaw - hub.latest_raw_yaw) % 360.0

    return move_successful

def return_to_origin(ep_chassis, ep_gimbal, start_x, start_y):
    log_event("Returning to Origin...")
    dir_offsets = {0: (0, 1), 90: (1, 0), -180: (0, -1), 180: (0, -1), -90: (-1, 0)}

    while hub.get_state()['running']:
        state = hub.get_state()
        curr_gx = state['grid_x']
        curr_gy = state['grid_y']

        if curr_gx == start_x and curr_gy == start_y:
            log_event("Arrived back at Origin successfully!")
            hub.set_status_msg("Returned to Origin. Exploration Complete!")
            hub.clean_outer_predicted_walls()
            save_map_image("final_map.png")
            break

        path = a_star_search((curr_gx, curr_gy), (start_x, start_y), GRID_W, GRID_H)
        if path and path[0] == (curr_gx, curr_gy):
            path.pop(0)

        if not path or len(path) == 0:
            log_event("Cannot find valid return path to Origin!")
            break

        next_cell = path[0]
        dx = next_cell[0] - curr_gx
        dy = next_cell[1] - curr_gy

        target_yaw = None
        for angle, offset in dir_offsets.items():
            if offset == (dx, dy):
                target_yaw = angle
                break

        if target_yaw is not None:
            turn_success = turn_to_orientation_imu(ep_chassis, ep_gimbal, target_yaw)
            if not turn_success:
                continue

            if hub.has_wall_between((curr_gx, curr_gy), next_cell):
                continue

            move_success = move_forward_60cm_straight(ep_chassis, target_step_m=move_target_step_m, target_speed=target_speed_mps)
            if move_success:
                hub.lock_passage((curr_gx, curr_gy), next_cell)
                hub.set_grid_pos(next_cell[0], next_cell[1])

# ==========================================
# 7. WORKERS & SCREENSHOT SAVER
# ==========================================
def save_map_image(filename="map_snapshot.png"):
    try:
        pygame.event.pump()
        surface = pygame.display.get_surface()
        if surface is not None:
            pygame.display.flip()
            pygame.image.save(surface, filename)
            log_event(f"[MAP SAVED] Screenshot saved successfully as '{filename}'")
    except Exception as e:
        log_event(f"[MAP SAVE FAILED] {e}")

def mapping_worker(ep_sensor, ep_gimbal, ep_chassis):
    ep_sensor.sub_distance(freq=50, callback=tof_data_handler)
    ep_gimbal.sub_angle(freq=20, callback=gimbal_data_handler)
    ep_chassis.sub_position(freq=20, callback=chassis_pos_handler)
    ep_chassis.sub_attitude(freq=20, callback=chassis_att_handler)
    
    while hub.get_state()['running']:
        state = hub.get_state()
        if state['emergency_stop'] and not (state['is_aligning'] or state['is_rotating'] or state['is_scanning']):
            ep_chassis.drive_wheels(w1=0, w2=0, w3=0, w4=0)
            time.sleep(0.01)
            continue
        time.sleep(0.01)

def exploration_worker(ep_chassis, ep_gimbal):
    while not hub.get_state()['is_calibrated']:
        time.sleep(0.05)

    dir_offsets = {0: (0, 1), 90: (1, 0), -180: (0, -1), 180: (0, -1), -90: (-1, 0)}
    dir_names = {0: 'North (N)', 90: 'East (E)', -180: 'South (S)', 180: 'South (S)', -90: 'West (W)'}
    wall_dir_map = {0: 'N', 90: 'E', -180: 'S', 180: 'S', -90: 'W'}
    
    step_count = 0
    previous_tof_side = None

    while hub.get_state()['running']:
        state = hub.get_state()
        current_gx = state['grid_x']
        current_gy = state['grid_y']
        p = state['pose']
        
        if state['emergency_stop']:
            log_event("[UNSTUCK] Obstacle detected! Backing up slightly...")
            hub.set_aligning(True)
            ep_chassis.drive_speed(x=-0.14, y=0, z=0)
            backup_start = time.time()
            while time.time() - backup_start < 0.50:
                if hub.get_state()['tof_raw'] >= SAFE_MIN_TOF:
                    break
                time.sleep(0.01)
            stop_chassis_active_brake(ep_chassis, brake_duration=0.08)
            hub.set_aligning(False)
                
            raw_yaw = p['chassis_yaw']
            snap_dir = int(round(raw_yaw / 90.0) * 90)
            snap_dir = int(normalize_angle(snap_dir))
            continue

        raw_yaw = p['chassis_yaw']
        snap_yaw = int(round(raw_yaw / 90.0) * 90)
        snap_yaw = int(normalize_angle(snap_yaw))

        step_count += 1
        log_event(f"--- STEP {step_count} ---")
        log_event(f"Grid({current_gx}, {current_gy}) | Heading: {dir_names.get(snap_yaw, 'Unknown')} ({raw_yaw:.1f} deg)")

        # -----------------------------------------------------------------
        # 🌟 LOOP SCAN 4 ทิศทาง <-> RECENTER
        # -----------------------------------------------------------------
        recenter_count = 0
        max_recenter_limit = 3

        while True:
            tof_dict = scan_4_directions(ep_gimbal, current_gx, current_gy, snap_yaw)
            has_moved = check_and_recenter(ep_chassis, ep_gimbal, tof_dict)
            
            if has_moved and recenter_count < max_recenter_limit:
                recenter_count += 1
                log_event(f"[RECENTER LOOP {recenter_count}] Position adjusted -> Re-scanning cell...")
                time.sleep(0.08)
                continue
            else:
                break

        # -----------------------------------------------------------------
        # 🌟 คำนวณปรับขนาน (WALL ALIGNMENT)
        # -----------------------------------------------------------------
        if previous_tof_side is not None:
            side_name, start_dist = previous_tof_side
            end_dist = tof_dict.get(side_name, 999.0)
            align_wall_heading_from_tof(ep_chassis, start_dist, end_dist, side_name)
            previous_tof_side = None

        wall_results = {
            'N': state['N_confirmed'][current_gy, current_gx] or state['N_predicted'][current_gy, current_gx],
            'E': state['E_confirmed'][current_gy, current_gx] or state['E_predicted'][current_gy, current_gx],
            'S': (hub.N_wall_logodds[current_gy - 1, current_gx] > 0.0) if current_gy > 0 else (hub.S_outer_wall_logodds[current_gx] > 0.0),
            'W': (hub.E_wall_logodds[current_gy, current_gx - 1] > 0.0) if current_gx > 0 else (hub.W_outer_wall_logodds[current_gy] > 0.0)
        }
        log_telemetry_csv(step_count, current_gx, current_gy, raw_yaw, tof_dict, wall_results)

        # -----------------------------------------------------------------
        # 🌟 วางแผนเดินทางไปเซลล์ถัดไป (A* SEARCH)
        # -----------------------------------------------------------------
        target_frontier, path = find_reachable_frontier(current_gx, current_gy, GRID_W, GRID_H)
        if path and path[0] == (current_gx, current_gy):
            path.pop(0)

        if not target_frontier or not path or len(path) == 0:
            log_event("All reachable frontiers mapped! Initiating return to origin.")
            return_to_origin(ep_chassis, ep_gimbal, START_X, START_Y)
            break

        next_cell = path[0]
        dx = next_cell[0] - current_gx
        dy = next_cell[1] - current_gy

        target_yaw = None
        for angle, offset in dir_offsets.items():
            if offset == (dx, dy):
                target_yaw = angle
                break

        if target_yaw is not None:
            turn_success = turn_to_orientation_imu(ep_chassis, ep_gimbal, target_yaw)
            if not turn_success:
                continue

            new_snap_yaw = int(round(target_yaw / 90.0) * 90)
            new_snap_yaw = int(normalize_angle(new_snap_yaw))
            tof_dict_after_turn = scan_4_directions(ep_gimbal, current_gx, current_gy, new_snap_yaw)

            if tof_dict_after_turn.get('RIGHT', 999.0) < 45.0:
                previous_tof_side = ('RIGHT', tof_dict_after_turn['RIGHT'])
            elif tof_dict_after_turn.get('LEFT', 999.0) < 45.0:
                previous_tof_side = ('LEFT', tof_dict_after_turn['LEFT'])

            move_success = move_forward_60cm_straight(ep_chassis, target_step_m=move_target_step_m, target_speed=target_speed_mps)
            
            if move_success:
                hub.lock_passage((current_gx, current_gy), next_cell)
                hub.set_grid_pos(next_cell[0], next_cell[1])
                log_event(f"[MAP UPDATED] Grid({next_cell[0]}, {next_cell[1]})")
            else:
                log_event(f"[MOVE BLOCKED] Path to Grid({next_cell[0]}, {next_cell[1]}) blocked.")
                if target_yaw in wall_dir_map:
                    w_dir = wall_dir_map[target_yaw]
                    hub.update_wall_prob(current_gx, current_gy, w_dir, is_wall=True)

        time.sleep(0.01)

# ==========================================
# 8. PYGAME GUI SYSTEM
# ==========================================
COLOR_BG = (25, 28, 36)
COLOR_GRID_BG = (40, 44, 52)
COLOR_GRID_LINE = (60, 64, 72)
COLOR_UNVISITED = (70, 75, 85)
COLOR_VISITED = (46, 204, 113)
COLOR_WALL_CONFIRMED = (231, 76, 60)
COLOR_WALL_PREDICTED = (52, 152, 219)
COLOR_ROBOT = (241, 196, 15)
COLOR_TEXT = (236, 240, 241)
COLOR_PANEL = (33, 37, 43)

def run_pygame_gui():
    pygame.init()
    pygame.font.init()
    
    font_large = pygame.font.SysFont("Tahoma", 20, bold=True)
    font_small = pygame.font.SysFont("Tahoma", 14)
    
    CELL_PIXELS = 60
    MARGIN = 20
    PANEL_WIDTH = 320
    WALL_THICKNESS = 4
    
    map_px_w = GRID_W * CELL_PIXELS
    map_px_h = GRID_H * CELL_PIXELS
    win_w = map_px_w + (MARGIN * 3) + PANEL_WIDTH
    win_h = max(map_px_h + (MARGIN * 2), 550)
    
    screen = pygame.display.set_mode((win_w, win_h))
    pygame.display.set_caption("RoboMaster EP - SLAM Exploration")
    clock = pygame.time.Clock()

    while hub.get_state()['running']:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                hub.clean_outer_predicted_walls()
                save_map_image("interrupted_map.png")
                hub.stop_workers()
                return

        state = hub.get_state()
        visited = state['visited']
        N_conf = state['N_confirmed']
        N_pred = state['N_predicted']
        E_conf = state['E_confirmed']
        E_pred = state['E_predicted']
        S_out_conf = state['S_outer_confirmed']
        S_out_pred = state['S_outer_predicted']
        W_out_conf = state['W_outer_confirmed']
        W_out_pred = state['W_outer_predicted']
        p = state['pose']
        
        screen.fill(COLOR_BG)
        map_origin_x = MARGIN
        map_origin_y = MARGIN + map_px_h

        pygame.draw.rect(screen, COLOR_GRID_BG, (MARGIN, MARGIN, map_px_w, map_px_h))

        for gy in range(GRID_H):
            for gx in range(GRID_W):
                px = map_origin_x + (gx * CELL_PIXELS)
                py = map_origin_y - ((gy + 1) * CELL_PIXELS)
                cell_rect = (px + 1, py + 1, CELL_PIXELS - 2, CELL_PIXELS - 2)
                color = COLOR_VISITED if visited[gy][gx] == 1 else COLOR_UNVISITED
                pygame.draw.rect(screen, color, cell_rect)
                lbl = font_small.render(f"{gx},{gy}", True, (150, 150, 150))
                screen.blit(lbl, (px + 4, py + 4))

        for gx in range(GRID_W + 1):
            x = map_origin_x + (gx * CELL_PIXELS)
            pygame.draw.line(screen, COLOR_GRID_LINE, (x, MARGIN), (x, MARGIN + map_px_h))
        for gy in range(GRID_H + 1):
            y = map_origin_y - (gy * CELL_PIXELS)
            pygame.draw.line(screen, COLOR_GRID_LINE, (MARGIN, y), (MARGIN + map_px_w, y))

        for gy in range(GRID_H):
            for gx in range(GRID_W):
                x_left = map_origin_x + (gx * CELL_PIXELS)
                x_right = x_left + CELL_PIXELS
                y_bottom = map_origin_y - (gy * CELL_PIXELS)
                y_top = y_bottom - CELL_PIXELS

                if N_conf[gy, gx]:
                    pygame.draw.line(screen, COLOR_WALL_CONFIRMED, (x_left, y_top), (x_right, y_top), WALL_THICKNESS)
                elif N_pred[gy, gx]:
                    pygame.draw.line(screen, COLOR_WALL_PREDICTED, (x_left, y_top), (x_right, y_top), WALL_THICKNESS)

                if E_conf[gy, gx]:
                    pygame.draw.line(screen, COLOR_WALL_CONFIRMED, (x_right, y_top), (x_right, y_bottom), WALL_THICKNESS)
                elif E_pred[gy, gx]:
                    pygame.draw.line(screen, COLOR_WALL_PREDICTED, (x_right, y_top), (x_right, y_bottom), WALL_THICKNESS)

        for gx in range(GRID_W):
            x_left = map_origin_x + (gx * CELL_PIXELS)
            x_right = x_left + CELL_PIXELS
            if S_out_conf[gx]:
                pygame.draw.line(screen, COLOR_WALL_CONFIRMED, (x_left, map_origin_y), (x_right, map_origin_y), WALL_THICKNESS)
            elif S_out_pred[gx]:
                pygame.draw.line(screen, COLOR_WALL_PREDICTED, (x_left, map_origin_y), (x_right, map_origin_y), WALL_THICKNESS)

        for gy in range(GRID_H):
            y_bottom = map_origin_y - (gy * CELL_PIXELS)
            y_top = y_bottom - CELL_PIXELS
            if W_out_conf[gy]:
                pygame.draw.line(screen, COLOR_WALL_CONFIRMED, (map_origin_x, y_top), (map_origin_x, y_bottom), WALL_THICKNESS)
            elif W_out_pred[gy]:
                pygame.draw.line(screen, COLOR_WALL_PREDICTED, (map_origin_x, y_top), (map_origin_x, y_bottom), WALL_THICKNESS)

        r_gx = state['grid_x'] + 0.5
        r_gy = state['grid_y'] + 0.5
        rx_px = map_origin_x + (r_gx * CELL_PIXELS)
        ry_px = map_origin_y - (r_gy * CELL_PIXELS)
        pygame.draw.circle(screen, COLOR_ROBOT, (int(rx_px), int(ry_px)), 12)

        total_yaw = normalize_angle(p['chassis_yaw'] + state['gimbal_yaw'])
        yaw_rad = math.radians(total_yaw)
        dir_x = rx_px + 22 * math.sin(yaw_rad)
        dir_y = ry_px - 22 * math.cos(yaw_rad)
        pygame.draw.line(screen, (255, 255, 255), (rx_px, ry_px), (dir_x, dir_y), 3)

        panel_x = MARGIN + map_px_w + MARGIN
        pygame.draw.rect(screen, COLOR_PANEL, (panel_x, MARGIN, PANEL_WIDTH, win_h - (MARGIN * 2)), border_radius=8)

        py_y = MARGIN + 15
        screen.blit(font_large.render("SYSTEM STATUS", True, (241, 196, 15)), (panel_x + 15, py_y))
        py_y += 35

        info_lines = [
            f"Status: {state['status_msg']}",
            f"Grid  : ({state['grid_x']}, {state['grid_y']})",
            f"Heading: {p['chassis_yaw']:.1f} deg",
            f"ToF   : {state['filtered_tof']:.1f} cm",
            f"Calibrated: {'Yes' if state['is_calibrated'] else 'No'}",
            f"EMG Brake : {'ACTIVE' if state['emergency_stop'] else 'Normal'}"
        ]

        for line in info_lines:
            color = (231, 76, 60) if "ACTIVE" in line else COLOR_TEXT
            screen.blit(font_small.render(line, True, color), (panel_x + 15, py_y))
            py_y += 22

        py_y += 10
        pygame.draw.line(screen, COLOR_GRID_LINE, (panel_x + 15, py_y), (panel_x + PANEL_WIDTH - 15, py_y))
        py_y += 15
        screen.blit(font_large.render("RECENT LOGS", True, (241, 196, 15)), (panel_x + 15, py_y))
        py_y += 30

        for log_msg in list(recent_logs)[::-1]:
            screen.blit(font_small.render(log_msg[-32:], True, (180, 180, 180)), (panel_x + 15, py_y))
            py_y += 18

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()

# ==========================================
# 9. MAIN ENTRY POINT
# ==========================================
if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    
    ep_sensor = ep_robot.sensor
    ep_gimbal = ep_robot.gimbal
    ep_chassis = ep_robot.chassis
    
    log_event("Setting Gimbal to Front Position...")
    reset_gimbal_to_center_fast(ep_gimbal)
    
    t1 = threading.Thread(target=mapping_worker, args=(ep_sensor, ep_gimbal, ep_chassis), daemon=True)
    t2 = threading.Thread(target=exploration_worker, args=(ep_chassis, ep_gimbal), daemon=True)
    
    t1.start()
    t2.start()
    
    log_event("Calibrating Local Frame...")
    if hub.calibrate_origin():
        log_event("Calibration Complete! Starting...")
    else:
        log_event("Calibration timeout, proceeding fallback.")
    
    try:
        run_pygame_gui()
    except KeyboardInterrupt:
        log_event("[KEYBOARD INTERRUPT] Stopping Robot safely & saving map...")
        hub.clean_outer_predicted_walls()
        save_map_image("interrupted_map.png")
    finally:
        hub.stop_workers()
        time.sleep(0.2)
            
        try:
            ep_sensor.unsub_distance()
            ep_gimbal.unsub_angle()
            ep_chassis.unsub_position()
            ep_chassis.unsub_attitude()
            ep_robot.close()
        except Exception:
            pass
        log_event("Robot connection closed cleanly.")