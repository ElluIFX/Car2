import os
import time

os.chdir(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from fire_vision import Detector
from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from loguru import logger
from SC import Controller, State, get_map_course, get_map_course_2, get_map_course_3

speed = 0.0


def callback(state):
    global speed
    v = (state.motor_l_speed_mps.value + state.motor_r_speed_mps.value) / 2
    speed += (v - speed) * 0.1


fc = FC_Controller()
fc.start_listen_serial("/dev/ttyS6", print_state=False, block_until_connected=True, callback=callback)
fc.wait_for_connection()
fc.motor_reset(fc.MOTOR_L | fc.MOTOR_R)
fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
fc.set_two_servo_pulse(1500, 1500)
radar = LD_Radar()
radar.debug = False
radar.start(subtask_skip=4)
radar.start_resolve_pose(800, 0.7, 0.3, rotation_adapt=True)
detector = Detector(fc)
base_x = 0
base_y = 0

pulse_list = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300]
deg_list = [35, 31, 24, 16, 7, 0, -6, -9, -13, -19, -20.5, -22, -23.7, -24]
rad_list = np.deg2rad(deg_list)
max_angle_r = deg_list[0]
max_angle_l = deg_list[-1]
max_angle_r_rad = rad_list[0]
max_angle_l_rad = rad_list[-1]


def get_pulse_from_deg(deg):
    if deg >= deg_list[0]:
        return pulse_list[0]
    if deg <= deg_list[-1]:
        return pulse_list[-1]
    for i in range(len(deg_list) - 1):
        if deg_list[i + 1] <= deg <= deg_list[i]:
            return np.interp(deg, [deg_list[i + 1], deg_list[i]], [pulse_list[i + 1], pulse_list[i]])


def motor_lr(vx: float, vz: float):
    Axle_spacing = 0.146  # 小车前后轴轴距 单位：m
    Wheel_spacing = 0.163  # 主动轮轮距 单位：m

    # 对于阿克曼小车vz代表右前轮转向角度
    R = Axle_spacing / np.tan(vz) - 0.5 * Wheel_spacing
    # 转弯半径 单位：m
    # 前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
    if vz > max_angle_r_rad:
        vz = max_angle_r_rad
    elif vz < max_angle_l_rad:
        vz = max_angle_l_rad
    # 运动学逆解
    if vz != 0:
        vl = vx * (R - 0.5 * Wheel_spacing) / R
        vr = vx * (R + 0.5 * Wheel_spacing) / R
    else:
        vl = vx
        vr = vx
    return vl, vr


def update_steer_and_speed(steer_rad: float, speed_mps: float):
    WHEEL_PERIMETER = 0.21049
    rpm = lambda speed_mps: speed_mps / WHEEL_PERIMETER * 60
    steer_deg = np.rad2deg(-steer_rad)
    pulse = get_pulse_from_deg(steer_deg)
    fc.set_steer_and_speed(pulse, rpm(speed_mps))
    # speed_l, speed_r = motor_lr(speed_mps, steer_rad)
    # fc.set_steer_and_two_speed(pulse, rpm(speed_l), rpm(speed_r))


def get_xy_yaw():
    x2_cm, y2_cm, yaw2 = radar.rt_pose
    return -y2_cm / 100, x2_cm / 100, np.deg2rad(-yaw2 + 90)


def calibrate():
    global base_x, base_y
    x, y, _ = get_xy_yaw()
    base_x = x - 1.35
    base_y = y - 0.25


def get_xyyaw_relative():
    x, y, yaw = get_xy_yaw()
    return x - base_x, y - base_y, yaw


update_steer_and_speed(0, 0)
time.sleep(2)
calibrate()

dl = 0.1
target_speed = 0.22


def update_state(mpst, vel=None):
    x, y, yaw = get_xyyaw_relative()
    # v = (fc.state.motor_l_speed_mps.value + fc.state.motor_r_speed_mps.value) / 2 if vel is None else vel
    v = speed
    logger.debug(f"x: {x}, y: {y}, yaw: {yaw}, v: {v}")
    mpst.update_state(x, y, yaw, v)


DT = 0.1

from route import get_route

x = 1.1
y = 2.8
enter_p, leave_p, side = get_route(x, y, dl=dl)
try:
    last_update = time.perf_counter()
    vel = target_speed
    cx, cy, cyaw, ck = enter_p
    x, y, yaw = get_xyyaw_relative()
    initial_state = State(x, y, yaw, 0)
    mpst = Controller(cx, cy, cyaw, ck, target_speed, dl, initial_state)
    fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
    update_state(mpst)
    for steer, acc in mpst.iter_output():
        vel = mpst.get_speed()
        update_steer_and_speed(steer, vel)
        logger.debug(f"steer: {steer}, acc: {acc}, vel: {vel}")
        while time.perf_counter() - last_update < DT:
            time.sleep(0.02)
        last_update = time.perf_counter()
        update_state(mpst)

    update_steer_and_speed(0, 0)
    fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.BREAK)
    detector.x_base = getattr(detector, f"x_base_{side}")
    detector.go_to_base()
    time.sleep(1)
    detector.process()
    detector.x_base = detector.x_base_m
    detector.go_to_base()

    last_update = time.perf_counter()
    vel = target_speed
    cx, cy, cyaw, ck = leave_p
    x, y, yaw = get_xyyaw_relative()
    initial_state = State(x, y, yaw, 0)
    mpst = Controller(cx, cy, cyaw, ck, target_speed, dl, initial_state)
    fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
    update_state(mpst)
    for steer, acc in mpst.iter_output():
        vel = mpst.get_speed()
        update_steer_and_speed(steer, vel)
        logger.debug(f"steer: {steer}, acc: {acc}, vel: {vel}")
        while time.perf_counter() - last_update < DT:
            time.sleep(0.02)
        last_update = time.perf_counter()
        update_state(mpst)

    update_steer_and_speed(0, 0)
    fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.BREAK)

finally:
    update_steer_and_speed(0, 0)
    time.sleep(1)
