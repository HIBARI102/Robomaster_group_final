import time
from robomaster import robot

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")
    ep_sensor_adaptor = ep_robot.sensor_adaptor

    print("--- กำลังสแกนหา Port ของ Sharp Sensor ---")
    print("ลองเอามือขยับเข้า-ออก หน้าเซนเซอร์ เพื่อดูว่าพอร์ตไหนค่า ADC เปลี่ยนแปลง\n")

    try:
        while True:
            # วนลูปเช็ค ID 1-2 และ Port 1-2
            for adapter_id in [1, 2]:
                for port_num in [1, 2]:
                    try:
                        adc = ep_sensor_adaptor.get_adc(id=adapter_id, port=port_num)
                        print(f"ID: {adapter_id} | Port: {port_num} => ADC: {adc}")
                    except Exception as e:
                        pass
            print("-" * 35)
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("หยุดการทดสอบ")

    ep_robot.close()