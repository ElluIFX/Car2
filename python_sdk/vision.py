import math
import random
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
from FlightController import FC_Controller
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from loguru import logger
from simple_pid import PID

fc = FC_Controller()
fc.start_listen_serial("COM3", print_state=True, block_until_connected=True)
fc.wait_for_connection()
# while True:
#     pulse = int(input("Pulse: "))
#     logger.debug(f"Set pulse: {pulse}")
#     fc.set_two_servo_pulse(pulse,pulse)
camera, id = open_camera()

print(f"Camera ID: {id}")
get = change_cam_resolution(camera, 920, 480, 60)
print(f"Info: {get[0]}x{get[1]} @ {get[2]}fps")
vision_debug()
set_cam_autowb(camera, True)
set_manual_exporsure(camera, -6.5)
pid_x = PID(0.4, 0.02, 0.02, setpoint=0, output_limits=(-180, 180), auto_mode=False)
pid_y = PID(0.6, 0.01, 0.005, setpoint=10, output_limits=(-180, 180), auto_mode=False)
fc.set_two_servo_pulse(round(1180), round(1900))  # 1900
fc.set_laser(False)
x_pwm = 1180.0
y_pwm = 1900.0
pwm_x_limit = (500, 2500)
pwm_y_limit = (1500, 2500)


################ PROCESS ##############################

x_random_range = 10
y_random_range = 4
max_random_add = 30
found = False
found_time = 0.0
lost_time = 0.0
auto_mode = False
lost = False

y_base = 1900
y_sta = 10
y_k = 0.2
y_limit = (-20, 20)


def process(img: np.ndarray) -> np.ndarray:
    global x_pwm, y_pwm, found, found_time, auto_mode, lost_time, lost
    # fc.set_servo_pulse(fc.SERVO1, 2000)
    # fc.set_servo_degree(1, 135)
    f, dx, dy = find_red_area(img)
    if not f:
        dx = 0
        dy = 0
        if not lost:
            lost = True
            lost_time = time.perf_counter()
        if auto_mode:
            auto_mode = False
            pid_x.set_auto_mode(False)
            pid_y.set_auto_mode(False)
        if lost and time.perf_counter() - lost_time > 2:
            lost = False
            found = False
            auto_mode = False
            pid_x.set_auto_mode(False)
            pid_y.set_auto_mode(False)
            fc.set_two_servo_pulse(round(1180), round(1900))  # 1900
            fc.set_laser(False)
            pid_y.setpoint = y_sta
    else:
        lost = False
        if not auto_mode:
            pid_x.set_auto_mode(True, last_output=0)
            pid_y.set_auto_mode(True, last_output=0)
            auto_mode = True
        if found:
            random_add = round((time.perf_counter() - found_time) * 3)
            random_add = min(random_add, max_random_add)
            # dx += random.randint(-x_random_range - random_add, x_random_range + random_add)
            # dy += random.randint(-y_random_range, y_random_range)
            dx += math.sin(time.perf_counter() * 10) * (x_random_range + random_add)
            dy += math.sin(time.perf_counter() * 2) * (y_random_range + random_add / 5)
            print(random_add)
        if (not found) and abs(dx - pid_x.setpoint) < 5 and abs(dy - pid_y.setpoint) < 5:
            found = True
            found_time = time.perf_counter()
            sp = y_sta + dy * (y_pwm - y_base)
            pid_y.setpoint = max(y_limit[0], min(y_limit[1], sp))
            fc.set_laser(True)
        out_x = pid_x(dx)
        x_pwm += out_x  # type: ignore
        x_pwm = max(pwm_x_limit[0], min(pwm_x_limit[1], x_pwm))
        out_y = pid_y(dy)
        y_pwm -= out_y  # type: ignore
        y_pwm = max(pwm_y_limit[0], min(pwm_y_limit[1], y_pwm))
        fc.set_two_servo_pulse(round(x_pwm), round(y_pwm))  # 1900
        print(found, dx, dy, x_pwm, y_pwm)
    return img


#######################################################
while True:
    img = camera.read()[1]
    if img is None:
        continue
    cv2.imshow("Origin", img)
    ################# PROCESS ##########################
    try:
        img = process(img)
    except Exception as e:
        logger.error(e)
    cv2.imshow("Processed", img)
    ####################################################
    k = cv2.waitKey(1) & 0xFF
    if k == ord("q"):
        break
cv2.destroyAllWindows()
