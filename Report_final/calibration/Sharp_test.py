import json
import os
import time
import numpy as np
from robomaster import robot

class SharpPolyReader:
    """ คลาสสำหรับโหลดไฟล์ JSON สมการกำลังสอง และแปลงค่า ADC เป็นระยะทาง (cm) """
    def __init__(self, json_file="calibration_sharp_poly.json"):
        if not os.path.exists(json_file):
            raise FileNotFoundError(f"ไม่พบไฟล์: {json_file}")
            
        with open(json_file, "r", encoding="utf-8") as f:
            data = json.load(f)
            
        self.left = data["LEFT_SENSOR"]
        self.right = data["RIGHT_SENSOR"]

    def _calc_dist(self, adc_val, cfg):
        # 1. ตรวจสอบค่าที่ไม่ถูกต้อง หรือไม่มีสัญญาณ
        if adc_val is None or adc_val <= 0:
            return 30.0

        # 2. ตรวจสอบ Noise Threshold และ min_adc 
        # หากค่า ADC ต่ำกว่า noise_threshold หรือ min_adc แสดงว่าวัตถุอยู่ไกลเกินระยะตรวจจับ หรือเป็น Noise
        min_cutoff = max(cfg.get("min_adc", 0), cfg.get("noise_threshold", 0))
        if adc_val < min_cutoff:
            return 30.0  # ไกลกว่า 30 cm หรือเป็น Noise

        # 3. ตรวจสอบ max_adc (สะท้อนกลับแรงเกินไป/ใกล้เกินระยะ)
        if adc_val > cfg["max_adc"]:
            return 5.0   # ใกล้กว่า 5 cm

        # 4. คำนวณตามสมการกำลังสอง: y = a*x^2 + b*x + c
        dist = (cfg["a"] * (adc_val ** 2)) + (cfg["b"] * adc_val) + cfg["c"]
        
        # จำกัดช่วงผลลัพธ์ให้อยู่ในช่วง 5.0 - 30.0 cm
        return round(float(np.clip(dist, 5.0, 30.0)), 1)

    def get_left_cm(self, adc_val):
        return self._calc_dist(adc_val, self.left)

    def get_right_cm(self, adc_val):
        return self._calc_dist(adc_val, self.right)


if __name__ == '__main__':
    # 1. โหลดตัวแปลงระยะทางจากไฟล์ JSON
    sharp = SharpPolyReader("calibration_sharp_poly.json")

    # 2. เริ่มเชื่อมต่อหุ่นยนต์
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")  # ปรับเป็น "sta" หรือ "rndis" ตามการเชื่อมต่อ
    sensor_adaptor = ep_robot.sensor_adaptor

    # ID และ Port ของ Sensor Adaptor
    LEFT_ID, LEFT_PORT = 2, 1
    RIGHT_ID, RIGHT_PORT = 1, 1

    print("=== เริ่มดึงค่าการทำงาน Sharp IR Sensor ===")
    print("กด Ctrl+C เพื่อหยุดการทำงาน\n")

    try:
        while True:
            # ดึงค่า ADC ดิบจาก Sensor Adaptor
            adc_l = sensor_adaptor.get_adc(id=LEFT_ID, port=LEFT_PORT)
            adc_r = sensor_adaptor.get_adc(id=RIGHT_ID, port=RIGHT_PORT)

            # คำนวณเป็นระยะทางจริง (cm)
            dist_l = sharp.get_left_cm(adc_l)
            dist_r = sharp.get_right_cm(adc_r)

            # แสดงผลการทำงาน
            print(f"ซ้าย (L): {dist_l:4.1f} cm (ADC: {adc_l}) | ขวา (R): {dist_r:4.1f} cm (ADC: {adc_r})")

            time.sleep(0.05)  # อ่านข้อมูลความถี่ 20 Hz

    except KeyboardInterrupt:
        print("\nหยุดการทำงาน")

    finally:
        ep_robot.close()