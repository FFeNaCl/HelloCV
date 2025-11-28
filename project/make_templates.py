# save_as: make_templates.py
import cv2
import numpy as np

numbers = [1, 2, 3, 4, 5]
size_w, size_h = 40, 56
font = cv2.FONT_HERSHEY_SIMPLEX
font_scale = 2.2
thickness = 5
color = (255, 255, 255)  # 白色数字
bg_color = (0, 0, 0)      # 黑色背景

for num in numbers:
    img = np.zeros((size_h, size_w), dtype=np.uint8)
    
    # 计算文字大小和位置（自动居中）
    text = str(num)
    text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
    text_x = (size_w - text_size[0]) // 2
    text_y = (size_h + text_size[1]) // 2
    
    cv2.putText(img, text, (text_x, text_y), font, font_scale, color, thickness, cv2.LINE_AA)
    
    cv2.imwrite(f"/home/tomori/Vision_Arena_2025/templates/template_{num}.png", img)
    print(f"生成 template_{num}.png")