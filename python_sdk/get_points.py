import cv2
import numpy as np

map_img = cv2.imread("./map2.jpg")

x_list = [1.35]
y_list = [0.25]
X = 4.8
Y = 4.0
x_size = map_img.shape[1]
y_size = map_img.shape[0]

from SC import calc_spline_course

draw_img = map_img.copy()


def update():
    global draw_img
    draw_img = map_img.copy()
    if len(x_list) < 2:
        return
    cx, cy, cyaw, ck, s = calc_spline_course(x_list, y_list, ds=0.01)
    for x, y in zip(cx, cy):
        cx = x / X * x_size
        cy = y_size - (y) / Y * y_size
        cv2.circle(draw_img, (int(cx), int(cy)), 1, (0, 0, 255), 1)


def mouse_callback(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        tx = x / x_size * X
        ty = (y_size - y) / y_size * Y
        x_list.append(tx)
        y_list.append(ty)
        update()
        print(f"x_list: {x_list}\ny_list: {y_list}")
    elif event == cv2.EVENT_RBUTTONDOWN:
        if len(x_list) > 1:
            x_list.pop()
            y_list.pop()
        update()
        print(f"x_list: {x_list}\ny_list: {y_list}")


while True:
    cv2.imshow("map", draw_img)
    cv2.setMouseCallback("map", mouse_callback)
    if (key := cv2.waitKey(1)) == ord("q"):
        break
    elif key == ord("r"):
        x_list = [1.35]
        y_list = [0.25]
        update()
        print(f"x_list: {x_list}\ny_list: {y_list}")
