import time
import json
import numpy as np
from robomaster import robot

def main():
    print("==================================================")
    print("  RoboMaster EP - ToF Calibration (Quadratic Degree 2)")
    print("==================================================")
    
    # เพิ่มจุดวัดระยะจริงให้ถี่ขึ้นเพื่อความแม่นยำของกราฟกำลังสอง (หน่วย cm)
    test_distances_cm = [10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 50.0, 60.0]
    
    raw_samples_list = []
    real_distances_list = []
    
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_sensor = ep_robot.sensor
    ep_gimbal = ep_robot.gimbal
    
    try:
        ep_gimbal.recenter().wait_for_completed(timeout=2.0)
    except Exception:
        pass
    
    latest_tof_cm = 999.0
    
    def tof_handler(sub_info):
        nonlocal latest_tof_cm
        if sub_info and len(sub_info) > 0 and sub_info[0] is not None:
            latest_tof_cm = float(sub_info[0]) / 10.0

    ep_sensor.sub_distance(freq=50, callback=tof_handler)
    time.sleep(1.0)
    
    print("\nเตรียมหุ่นยนต์ให้อยู่ในแนวตรงกับกำแพง หรือแผ่นฉากตั้งตรง")
    
    try:
        for d_real in test_distances_cm:
            input(f"\n👉 วางหุ่นยนต์ห่างจากกำแพงระยะจริง {d_real:.1f} cm แล้วกด Enter...")
            
            print("กำลังเก็บค่าจาก ToF Sensor (100 samples)...")
            samples = []
            start_t = time.time()
            
            while len(samples) < 100 and (time.time() - start_t < 5.0):
                if 2.0 <= latest_tof_cm <= 300.0:
                    samples.append(latest_tof_cm)
                time.sleep(0.02)
                
            if len(samples) < 20:
                print("⚠️ อ่านค่าเซนเซอร์ไม่สำเร็จ ข้ามจุดนี้...")
                continue
                
            raw_median = float(np.median(samples))
            raw_std = float(np.std(samples))
            print(f"✅ ระยะจริง: {d_real:.1f} cm | ค่าดัก ToF: {raw_median:.2f} cm (Std: {raw_std:.2f})")
            
            raw_samples_list.append(raw_median)
            real_distances_list.append(d_real)

        if len(raw_samples_list) < 4:
            print("\n❌ ข้อมูลไม่เพียงพอสำหรับสร้าง Quadratic Curve (ต้องการอย่างน้อย 4 จุด)!")
            return

        x_raw = np.array(raw_samples_list)
        y_real = np.array(real_distances_list)
        
        # 🌟 คำนวณ Polynomial Degree 2: Y = a*X^2 + b*X + c
        poly_coefs = np.polyfit(x_raw, y_real, 2)
        a, b, c = poly_coefs
        
        # คำนวณ R-squared (Coefficient of Determination)
        y_pred = np.polyval(poly_coefs, x_raw)
        ss_res = np.sum((y_real - y_pred) ** 2)
        ss_tot = np.sum((y_real - np.mean(y_real)) ** 2)
        r_squared = 1 - (ss_res / ss_tot)
        
        print("\n==========================================")
        print("     QUADRATIC CALIBRATION RESULT        ")
        print("==========================================")
        print(f"สมการปรับแก้: Real_Dist = ({a:.6f} * Raw^2) + ({b:.6f} * Raw) + ({c:.4f})")
        print(f"ความแม่นยำ R²: {r_squared:.4f}")
        
        calib_data = {
            "model_type": "quadratic",
            "a": float(a),
            "b": float(b),
            "c": float(c),
            "r_squared": float(r_squared),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        
        with open("tof_calib_params.json", "w", encoding="utf-8") as f:
            json.dump(calib_data, f, indent=4)
            
        print("💾 บันทึกไฟล์ 'tof_calib_params.json' เรียบร้อยพร้อมใช้งาน!")

    finally:
        try:
            ep_sensor.unsub_distance()
            ep_robot.close()
        except Exception:
            pass

if __name__ == '__main__':
    main()