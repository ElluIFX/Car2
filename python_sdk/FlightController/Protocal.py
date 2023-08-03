import struct
import time
from typing import Optional, Tuple, Union

from FlightController.Base import Byte_Var, FC_Base_Uart_Comunication
from loguru import logger


class FC_Protocol(FC_Base_Uart_Comunication):
    """
    协议层, 定义了实际的控制命令
    """

    # constants
    MOTOR_L = 0x01
    MOTOR_R = 0x02
    BREAK = 0x00
    GLIDE = 0x01
    SPD_CTRL = 0x02
    POS_CTRL = 0x03
    MANUAL = 0x04

    SERVO1 = 0x01
    SERVO2 = 0x02
    SERVO3 = 0x04
    SERVO4 = 0x08

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)  # type: ignore
        self._byte_temp1 = Byte_Var()
        self._byte_temp2 = Byte_Var()
        self._byte_temp3 = Byte_Var()
        self._byte_temp4 = Byte_Var()

    def _action_log(self, action: str, data_info: Optional[str] = None):
        if self.settings.action_log_output:
            string = f"[FC] [ACTION] {action.upper()}"
            if data_info is not None:
                string += f" -> {data_info}"
            logger.info(string)

    def send_to_wireless(self, data: bytes) -> None:
        """
        转发数据到无线串口
        """
        self.send_data_to_fc(data, 0x01, False)

    def set_laser(self, state: bool) -> None:
        """
        设置激光器状态
        """
        self.send_data_to_fc(b"\x01" if state else b"\x00", 0x02, False)

    def set_motor_mode(self, motor: int, mode: int) -> None:
        """
        设置电机模式 (0:刹车 1:滑行 2:速度环 3:位置环 4:手动)
        """
        self.send_data_to_fc(struct.pack("<BB", motor, mode), 0x03, True)

    def set_motor_pwm(self, motor: int, pwm: float):
        """
        设置电机PWM
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, pwm), 0x04)

    def set_motor_speed_rpm(self, motor: int, speed: float):
        """
        设置电机速度环速度 (rpm)
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, speed), 0x05)

    def set_motor_speed_mps(self, motor: int, speed: float):
        """
        设置电机速度环速度 (m/s)
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, speed), 0x06)

    def set_motor_pos_speed(self, motor: int, speed: float):
        """
        设置电机位置环速度 (rpm)
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, speed), 0x07)

    def set_motor_pos_speed_mps(self, motor: int, speed: float):
        """
        设置电机位置环速度 (m/s)
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, speed), 0x08)

    def set_motor_pos_degree(self, motor: int, degree: float):
        """
        设置电机位置环角度 (°)
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, degree), 0x09)

    def motor_pos_go_degree(self, motor: int, degree: float):
        """
        电机位置环旋转指定角度 (°)
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, degree), 0x0A)

    def motor_pos_go_meter(self, motor: int, meter: float):
        """
        电机前进指定距离 (m)
        """
        self.send_data_to_fc(struct.pack("<Bf", motor, meter), 0x0B)

    def motor_reset(self, motor: int):
        """
        重置电机状态
        """
        self.send_data_to_fc(struct.pack("<B", motor), 0x0C, True)

    def set_servo_pulse(self, servo: int, pulse: int):
        """
        设置舵机脉宽 (us) (500us - 2500us)
        """
        self.send_data_to_fc(struct.pack("<BI", servo, pulse), 0x0D)

    def set_two_servo_pulse(self, pulse1: int, pulse2: int):
        """
        设置两个舵机脉宽 (us) (500us - 2500us)
        """
        self.send_data_to_fc(struct.pack("<II", pulse1, pulse2), 0x0E)
