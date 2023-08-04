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

class Detector:
    x_base_m = 1180.0
    x_base_l = 1850.0
    x_base_r = 500.0

    pwm_x_limit = (500, 2500)
    pwm_y_limit = (1500, 2500)

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
    y_k = 0.1
    y_limit = (-20, 20)

    x_base = x_base_m
    y_base = y_base

    def __init__(self, fc):
        self.fc = fc
        self.camera, id = open_camera()
        get = change_cam_resolution(self.camera, 920, 480, 60)
        print(f"Info: {get[0]}x{get[1]} @ {get[2]}fps")
        vision_debug(True)
        set_cam_autowb(self.camera, True)
        set_manual_exporsure(self.camera, 80)
        self.pid_x = PID(
            0.4, 0.02, 0.02, setpoint=0, output_limits=(-180, 180), auto_mode=False
        )
        self.pid_y = PID(
            0.6,
            0.01,
            0.005,
            setpoint=self.y_sta,
            output_limits=(-180, 180),
            auto_mode=False,
        )
        self.x_pwm = self.x_base
        self.y_pwm = self.y_base
        fc.set_two_servo_pulse(round(self.x_pwm), round(self.y_pwm))
        fc.set_laser(False)

    def go_to_base(self):
        self.fc.set_two_servo_pulse(round(self.x_base), round(self.y_base))

    def process(self, loop=False):
        fc = self.fc
        sucessed = False
        while True:
            img = self.camera.read()[1]
            if img is None:
                continue
            f, dx, dy = find_red_area(img)
            if not f:
                dx = 0
                dy = 0
                if not self.lost:
                    self.lost = True
                    self.lost_time = time.perf_counter()
                if self.auto_mode:
                    self.auto_mode = False
                    self.pid_x.set_auto_mode(False)
                    self.pid_y.set_auto_mode(False)
                if self.lost and time.perf_counter() - self.lost_time > 2:
                    self.lost = False
                    self.found = False
                    self.auto_mode = False
                    self.pid_x.set_auto_mode(False)
                    self.pid_y.set_auto_mode(False)
                    fc.set_two_servo_pulse(round(self.x_base), round(self.y_base))
                    fc.set_laser(False)
                    self.pid_y.setpoint = self.y_sta
                    if not loop and sucessed:
                        return
            else:
                self.lost = False
                if not self.auto_mode:
                    self.pid_x.set_auto_mode(True, last_output=0)
                    self.pid_y.set_auto_mode(True, last_output=0)
                    self.auto_mode = True
                if self.found:
                    random_add = round((time.perf_counter() - self.found_time) * 3)
                    random_add = min(random_add, self.max_random_add)
                    # dx += random.randint(-x_random_range - random_add, x_random_range + random_add)
                    # dy += random.randint(-y_random_range, y_random_range)
                    dx += math.sin(time.perf_counter() * 10) * (
                        self.x_random_range + random_add
                    )
                    dy += math.sin(time.perf_counter() * 2) * (
                        self.y_random_range + random_add / 5
                    )
                if (
                    (not self.found)
                    and abs(dx - self.pid_x.setpoint) < 5
                    and abs(dy - self.pid_y.setpoint) < 5
                ):
                    self.found = True
                    self.found_time = time.perf_counter()
                    # sp = y_sta + dy * (y_pwm - y_base)
                    # pid_y.setpoint = max(y_limit[0], min(y_limit[1], sp))
                    fc.set_laser(True)
                    sucessed = True
                out_x = self.pid_x(dx)
                self.x_pwm += out_x  # type: ignore
                self.x_pwm = max(
                    self.pwm_x_limit[0], min(self.pwm_x_limit[1], self.x_pwm)
                )
                out_y = self.pid_y(dy)
                self.y_pwm -= out_y  # type: ignore
                self.y_pwm = max(
                    self.pwm_y_limit[0], min(self.pwm_y_limit[1], self.y_pwm)
                )
                fc.set_two_servo_pulse(round(self.x_pwm), round(self.y_pwm))  # 1900


#######################################################
if __name__ == "__main__":

    fc = FC_Controller()
    fc.start_listen_serial("/dev/ttyS6", print_state=True, block_until_connected=True)
    fc.wait_for_connection()


    # while True:
    #     pulse = int(input("Pulse: "))
    #     logger.debug(f"Set pulse: {pulse}")
    #     fc.set_two_servo_pulse(pulse,1900)
    try:
        detector = Detector(fc)
        detector.x_base = detector.x_base_l
        detector.process()
        detector.x_base = detector.x_base_m
        detector.go_to_base()
    except KeyboardInterrupt:
        time.sleep(1)
