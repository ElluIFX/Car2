import math
import os
from typing import List

import numpy as np
from loguru import logger

CRC_TABLE = [
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C, # 0x00 - 0x0F
    0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5, # 0x10 - 0x1F
    0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43, # 0x20 - 0x2F
    0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA, # 0x30 - 0x3F
    0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62, # 0x40 - 0x4F
    0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB, # 0x50 - 0x5F
    0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D, # 0x60 - 0x6F
    0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4, # 0x70 - 0x7F
    0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20, # 0x80 - 0x8F
    0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89, # 0x90 - 0x9F
    0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F, # 0xA0 - 0xAF
    0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96, # 0xB0 - 0xBF
    0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E, # 0xC0 - 0xCF
    0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7, # 0xD0 - 0xDF
    0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01, # 0xE0 - 0xEF
    0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8, # 0xF0 - 0xFF
]  # fmt: skip
"""
Usage of CRC_TABLE:
    CRC_TABLE[(CRC_TABLE[(CRC_TABLE[byte1] ^ byte2) & 0xFF] ^ byte3) & 0xFF] ^ byte4 ...
"""


def calculate_crc8(data: bytes) -> int:
    """
    Calculate CRC 8 of data.
    data: bytes-like object
    """
    crc = 0x00
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


def decode_human_str(data: bytes) -> str:
    return "".join(
        chr(byte) if (32 <= byte <= 126 or byte == ord("\r") or byte == ord("\n")) else f"\\x{byte:02x}"
        for byte in data
    )


def decode_hex_str(data: bytes) -> str:
    return "".join(f"\\x{byte:02x}" for byte in data)


class Tmux:
    """
    Tmux warpper for python
    """

    def __init__(self, session_name: str = "", sudo: bool = False):
        self._session = session_name
        self._sudo = sudo

    def new_session(self):
        cmd = ""
        if self._sudo:
            cmd += f"sudo "
        cmd += f"tmux new-session -d -s {self._session}"
        os.system(cmd)
        return self

    def send_command(self, command: str, enter: bool = True):
        cmd = ""
        if self._sudo:
            cmd += f"sudo "
        cmd += f"tmux send-keys -t {self._session} '{command}'"
        if enter:
            cmd += " Enter"
        os.system(cmd)
        return self

    def send_key_interruption(self):
        cmd = ""
        if self._sudo:
            cmd += f"sudo "
        cmd += f"tmux send-keys -t {self._session} C-c"
        os.system(cmd)
        return self

    def kill_session(self):
        cmd = ""
        if self._sudo:
            cmd += f"sudo "
        cmd += f"tmux kill-session -t {self._session}"
        os.system(cmd)
        return self

    def capture_output(
        self, start_line=None, end_line=None, tail=None, connect_line=True, clear_control=False, include_escape=False
    ) -> str:
        cmd = ""
        if self._sudo:
            cmd += f"sudo "
        cmd += f"tmux capture-pane -p -t {self._session}:0"
        if start_line is not None:
            cmd += f" -S {start_line}"
        else:
            cmd += " -S-"
        if end_line is not None:
            cmd += f" -E {end_line}"
        else:
            cmd += " -E-"
        if connect_line:
            cmd += " -J"
        if clear_control:
            cmd += " -C"
        if include_escape:
            cmd += " -e"
        if tail is not None:
            cmd += f" | tail -n {tail}"
        return os.popen(cmd).read()

    @staticmethod
    def get_running_sessions() -> List[str]:
        list = os.popen("tmux ls").read().split("\n")
        return [l.strip().split(":")[0] for l in list if l.strip() != ""]

    @property
    def session_pid(self) -> int:
        cmd = ""
        if self._sudo:
            cmd += f"sudo "
        cmd += "tmux list-panes -F '#{pane_pid}' " + f"-t {self._session}"
        return int(os.popen(cmd).read().split("\n")[0].strip())

    @property
    def session_busy(self) -> bool:
        cmd = ""
        if self._sudo:
            cmd += f"sudo "
        cmd += f"ps --ppid {self.session_pid} | wc -l"
        return int(os.popen(cmd).read().split("\n")[0].strip()) > 1

    @property
    def session_running(self) -> bool:
        return self._session in Tmux.get_running_sessions()


def quaternions_to_euler(x, y, z, w):
    # mathod 1
    # r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # p = math.asin(2 * (w * y - z * x))
    # y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # mathod 2
    # r = math.atan2(2.0 * (w * x + y * z), w * w - x * x - y * y + z * z)
    # p = -math.asin(2.0 * (x * z - w * y))
    # y = math.atan2(2.0 * (w * z + x * y), w * w + x * x - y * y - z * z)

    # mathod 3
    # Resolve the gimbal lock problem
    sinp = 2.0 * (w * y - z * x)
    p = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    r = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    y = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # convert radians to degrees
    r, p, y = math.degrees(r), math.degrees(p), math.degrees(y)
    return r, p, y


def quaternions_to_rotation_matrix(x, y, z, w) -> np.ndarray:
    """
    将wxyz的四元数转换为3x3的旋转矩阵
    """
    # 构造旋转矩阵
    rotation_matrix = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )

    return rotation_matrix


def rotation_matrix_to_quaternions(rotation_matrix: np.ndarray) -> tuple:
    """
    将3x3的旋转矩阵转换为wxyz的四元数
    """
    # 计算四元数的w分量
    w = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
    # 计算四元数的x, y, z分量
    x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4 * w)
    y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4 * w)
    z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4 * w)

    return x, y, z, w
