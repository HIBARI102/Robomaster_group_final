import cv2
import numpy as np

# Variables สำหรับเก็บสถานะ
drawing = False
ix, iy = -1, -1
img = None
current_color = (255, 255, 255) # เริ่มต้นด้วยสีขาว
color_name = "WHITE"

def draw_rectangle(event, x, y, flags, param):
    global ix, iy, drawing, img, temp_img, current_color

    # คลิกเมาส์ซ้าย = วาดด้วยสีที่เลือกไว้
    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    # คลิกเมาส์ขวา = วาดด้วยสีดำเสมอ (ใช้ลบส่วนเกินได้รวดเร็ว)
    elif event == cv2.EVENT_RBUTTONDOWN:
        drawing = True
        ix, iy = x, y
        param['active_color'] = (0, 0, 0) # กำหนดให้เป็นสีดำชั่วคราว

    # ขณะกำลังลากเมาส์ แสดงตัวอย่างสี่เหลี่ยม
    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            temp_img = img.copy()
            active_color = param.get('active_color', current_color)
            cv2.rectangle(temp_img, (ix, iy), (x, y), active_color, -1)

    # ปล่อยเมาส์ เพื่อวาดลงบนภาพจริง
    elif event == cv2.EVENT_LBUTTONUP or event == cv2.EVENT_RBUTTONUP:
        if drawing:
            drawing = False
            active_color = param.get('active_color', current_color)
            cv2.rectangle(img, (ix, iy), (x, y), active_color, -1)
            param['active_color'] = current_color # คืนค่าสีเดิม

def main():
    global img, temp_img, current_color, color_name
    
    # 1. โหลดภาพ Mask ของคุณ
    image_path = 'roi_mask.png'  # เช่น ไฟล์ภาพ mask
    img = cv2.imread(image_path)
    
    if img is None:
        print(f"ไม่พบไฟล์ภาพ: {image_path}")
        return

    mouse_params = {'active_color': current_color}
    temp_img = img.copy()
    cv2.namedWindow('Mask Editor')
    cv2.setMouseCallback('Mask Editor', draw_rectangle, mouse_params)

    print("=== วิธีการใช้งาน ===")
    print("- คลิกเมาส์ซ้ายค้างแล้วลาก: วาดตามสีที่เลือกไว้ปัจจุบัน")
    print("- คลิกเมาส์ขวาค้างแล้วลาก: วาดสีดำ (ใช้ลบ) ได้ทันที")
    print("- กด 'w': สลับเป็น โหมดสีขาว (WHITE)")
    print("- กด 'b': สลับเป็น โหมดสีดำ (BLACK)")
    print("- กด 's': บันทึกภาพเป็น 'mask_updated.png'")
    print("- กด 'r': Reset ภาพกลับไปเริ่มต้น")
    print("- กด 'q' หรือ ESC: ออกจากโปรแกรม")

    while True:
        # แสดงผลภาพพร้อมป้ายบอกสีปัจจุบัน
        display_img = temp_img.copy() if drawing else img.copy()
        
        # ใส่ข้อความบอกโหมดสีบนมุมซ้ายบนของหน้าต่าง
        cv2.putText(display_img, f"Current Color: {color_name}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if color_name == "WHITE" else (0, 0, 255), 2)

        cv2.imshow('Mask Editor', display_img)

        key = cv2.waitKey(1) & 0xFF

        # กด 'w' สลับเป็นสีขาว
        if key == ord('w'):
            current_color = (255, 255, 255)
            mouse_params['active_color'] = current_color
            color_name = "WHITE"
            print("สลับโหมดเป็น: สีขาว (WHITE)")

        # กด 'b' สลับเป็นสีดำ
        elif key == ord('b'):
            current_color = (0, 0, 0)
            mouse_params['active_color'] = current_color
            color_name = "BLACK"
            print("สลับโหมดเป็น: สีดำ (BLACK)")

        # กด 's' เพื่อบันทึกภาพ
        elif key == ord('s'):
            cv2.imwrite('mask_updated.png', img)
            print("บันทึกภาพเรียบร้อย: mask_updated.png")

        # กด 'r' เพื่อรีเซ็ตภาพ
        elif key == ord('r'):
            img = cv2.imread(image_path)
            print("รีเซ็ตภาพเรียบร้อย")

        # กด 'q' หรือ ESC เพื่อปิด
        elif key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()