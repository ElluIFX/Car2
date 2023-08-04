import os
import time

os.chdir(os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from loguru import logger
from MPST import Controller, get_map_course,State,DT,MAX_STEER

fc = FC_Controller()
fc.start_listen_serial("/dev/ttyS6", print_state=False, block_until_connected=True)
fc.wait_for_connection()
fc.motor_reset(fc.MOTOR_L | fc.MOTOR_R)
fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
fc.set_two_servo_pulse(1500,1500)
radar = LD_Radar()
radar.debug = False
radar.start(subtask_skip=8)
radar.start_resolve_pose(1000, 0.8, 0.8, rotation_adapt=True)

base_x = 0
base_y = 0



def update_steer_and_speed(steer_rad: float, speed_mps: float):
    WHEEL_PERIMETER = 0.21049
    SERVO_MIDVALUE = 1450
    SERVO_FULLADD = 500
    rpm = speed_mps / WHEEL_PERIMETER * 60
    steer_rad = np.clip(steer_rad, -MAX_STEER, MAX_STEER)
    pulse = SERVO_MIDVALUE + steer_rad / MAX_STEER * SERVO_FULLADD
    fc.set_steer_and_speed(pulse, rpm)

while True:
    steer = input("pulse: ")
    fc.set_steer_and_speed(int(steer), 0)

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

dl = 0.02
target_speed = 0.2
cx, cy, cyaw, ck = get_map_course(dl)
x,y,yaw = get_xyyaw_relative()
initial_state = State(x,y,yaw,0)
mpst = Controller(cx, cy, cyaw, ck, target_speed, dl,initial_state)
def update_state(vel = None):
    x, y, yaw = get_xyyaw_relative()
    v = (fc.state.motor_l_speed_mps.value + fc.state.motor_r_speed_mps.value) / 2 if vel is None else vel
    logger.debug(f"x: {x}, y: {y}, yaw: {yaw}, v: {v}")
    mpst.update_state(x, y, yaw, v)
try:
    last_update = time.perf_counter()
    update_state()
    vel = 0
    for steer, acc in mpst.iter_output():
        vel = mpst.a_to_v(acc,vel)
        update_steer_and_speed(steer,vel)
        logger.debug(f"steer: {steer}, acc: {acc}, vel: {vel}")
        while time.perf_counter() - last_update < DT:
            time.sleep(0.01)
        last_update += DT
        update_state(vel=vel)
    update_steer_and_speed(0, 0)
    fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.BREAK)
except KeyboardInterrupt:
    update_steer_and_speed(0,0)
    time.sleep(1)
