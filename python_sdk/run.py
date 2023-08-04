import os
import time
from turtle import st

os.chdir(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from loguru import logger
from MPST import Controller, get_map_course

fc = FC_Controller()
fc.start_listen_serial("COM24", print_state=True, block_until_connected=True)
fc.wait_for_connection()
radar = LD_Radar()
radar.start(subtask_skip=8)
radar.start_resolve_pose(1000, 0.8, 0.6, rotation_adapt=True)
dl = 0.02
target_speed = 0.10
cx, cy, cyaw, ck = get_map_course(dl)
mpst = Controller(cx, cy, cyaw, ck, target_speed, dl)
base_x = 0
base_y = 0


def update_steer_and_speed(steer_rad: float, speed_mps: float):
    WHEEL_PERIMETER = 0.5
    SERVO_MIDVALUE = 1400
    SERVO_FULLADD = 400
    MAX_STEER = np.deg2rad(27)
    rpm = speed_mps / WHEEL_PERIMETER * 60
    steer_deg = np.rad2deg(steer_rad)
    steer_deg = np.clip(steer_deg, -MAX_STEER, MAX_STEER)
    pulse = SERVO_MIDVALUE + steer_deg / MAX_STEER * SERVO_FULLADD
    fc.set_steer_and_speed(pulse, rpm)


def get_xy_yaw():
    x2_cm, y2_cm, yaw2 = radar.rt_pose
    return -y2_cm / 100, x2_cm / 100, np.deg2rad(-yaw2 + 90)


def calibrate():
    global base_x, base_y
    base_x, base_y, _ = get_xy_yaw()


def get_xyyaw_relative():
    x, y, yaw = get_xy_yaw()
    return x - base_x, y - base_y, yaw


def update_state():
    x, y, yaw = get_xyyaw_relative()
    v = (fc.state.motor_l_speed_mps.value + fc.state.motor_r_speed_mps.value) / 2
    logger.debug(f"x: {x}, y: {y}, yaw: {yaw}, v: {v}")
    mpst.update_state(x, y, yaw, v)


update_steer_and_speed(0, 0)
update_state()
for steer, acc in mpst.iter_output():
    update_steer_and_speed(steer, acc)
    logger.debug(f"steer: {steer}, acc: {acc}")
    time.sleep(0.05)
    update_state()
