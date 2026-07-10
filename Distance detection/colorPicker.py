import cv2
import numpy as np

def hex_to_opencv_hsv_range(hex_color, h_tol=8, s_tol=70, v_tol=70):
    hex_color = hex_color.lstrip("#")

    r = int(hex_color[0:2], 16)
    g = int(hex_color[2:4], 16)
    b = int(hex_color[4:6], 16)

    bgr_pixel = np.uint8([[[b, g, r]]])
    hsv_pixel = cv2.cvtColor(bgr_pixel, cv2.COLOR_BGR2HSV)[0][0]

    h, s, v = map(int, hsv_pixel)

    lower = (
        max(0, h - h_tol),
        max(0, s - s_tol),
        max(0, v - v_tol),
    )

    upper = (
        min(180, h + h_tol),
        min(255, s + s_tol),
        min(255, v + v_tol),
    )


    return hsv_pixel.tolist(), lower, upper


center, lower, upper = hex_to_opencv_hsv_range("#779d54") # verde
center, lower, upper = hex_to_opencv_hsv_range("#2c5b7f") #azul de cartulina oscuro
center, lower, upper = hex_to_opencv_hsv_range("#5492d3") #azul de cartulina claro
center, lower, upper = hex_to_opencv_hsv_range("#4a708f") #azul de cartulina oscuro
# center, lower, upper = hex_to_opencv_hsv_range("#1b3768") # azul de sofas

print("center:", center)
print("lower:", lower)
print("upper:", upper)