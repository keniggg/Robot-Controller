import cv2
import numpy as np

def show_hsv_value(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_val = hsv[y, x]
        print(f"Clicked at ({x}, {y}) - HSV: {hsv_val}")

# 读取你的图像
image = cv2.imread("./image.png")
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

cv2.namedWindow("image")
cv2.setMouseCallback("image", show_hsv_value)

while True:
    cv2.imshow("image", image)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cv2.destroyAllWindows()
