import json
import os
import cv2

class ROIManager:
    def __init__(self, config_path="roi_config.json"):
        self.config_path = config_path
        self.top_ratio = 0.20
        self.bottom_ratio = 0.80
        self.left_ratio = 0.00
        self.right_ratio = 1.00
        self.load_config()

    def load_config(self):
        """ โหลดค่าตั้งต้น ROI จาก JSON หากไม่มีไฟล์จะทำการสร้างให้ใหม่ """
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                    self.top_ratio = float(data.get("roi_top_ratio", 0.20))
                    self.bottom_ratio = float(data.get("roi_bottom_ratio", 0.80))
                    self.left_ratio = float(data.get("roi_left_ratio", 0.00))
                    self.right_ratio = float(data.get("roi_right_ratio", 1.00))
            except Exception as e:
                print(f"[ROI] Error loading config: {e}")
        else:
            self.save_config()

    def save_config(self):
        """ บันทึกค่า Config ปัจจุบันลงไฟล์ JSON """
        data = {
            "roi_top_ratio": self.top_ratio,
            "roi_bottom_ratio": self.bottom_ratio,
            "roi_left_ratio": self.left_ratio,
            "roi_right_ratio": self.right_ratio
        }
        try:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            print(f"[ROI] Created default {self.config_path}")
        except Exception as e:
            print(f"[ROI] Error saving config: {e}")

    def get_crop_bounds(self, frame_height, frame_width):
        """ คำนวณพิกัด Pixel (y1, y2, x1, x2) จากสัดส่วน Ratio """
        y1 = int(frame_height * self.top_ratio)
        y2 = int(frame_height * self.bottom_ratio)
        x1 = int(frame_width * self.left_ratio)
        x2 = int(frame_width * self.right_ratio)
        return y1, y2, x1, x2

    def crop_frame(self, frame):
        """ ตัดภาพเฉพาะพื้นที่ ROI และคืนค่า Offset (x1, y1) """
        if frame is None or frame.size == 0:
            return None, (0, 0)

        h, w, _ = frame.shape
        y1, y2, x1, x2 = self.get_crop_bounds(h, w)
        
        # ป้องกัน Index หลุดขอบ
        y1, y2 = max(0, y1), min(h, y2)
        x1, x2 = max(0, x1), min(w, x2)

        cropped = frame[y1:y2, x1:x2]
        return cropped, (x1, y1)

    def draw_roi_box(self, frame, color=(255, 0, 255), thickness=2):
        """ วาดกรอบสี่เหลี่ยม ROI บนภาพหลักเพื่อดูตอน Calibrate หน้างาน """
        if frame is None or frame.size == 0:
            return frame

        h, w, _ = frame.shape
        y1, y2, x1, x2 = self.get_crop_bounds(h, w)
        
        output_frame = frame.copy()
        cv2.rectangle(output_frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(output_frame, "ROI AREA", (x1 + 10, max(y1 + 25, 25)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return output_frame

    def map_to_full_frame(self, cx_roi, cy_roi, offset_tuple):
        """ แปลงพิกัดศูนย์กลาง (cx, cy) จากภาพ ROI กลับไปยังพิกัดภาพใหญ่ """
        offset_x, offset_y = offset_tuple
        return cx_roi + offset_x, cy_roi + offset_y