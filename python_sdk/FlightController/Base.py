import re
import struct
import threading
import time
from threading import Event
from typing import Callable, Dict, List, Optional

import serial
from FlightController.Serial import (
    SerialReader,
    SerialReaderBuffered,
    SerialReaderLike,
    SerialReaderThreaded,
)
from loguru import logger
from serial.serialutil import SerialException
from serial.tools.list_ports import comports


def bytes_to_str(data):
    return " ".join([f"{b:02X}" for b in data])


class Byte_Var:
    """
    C-like byte类型变量与python泛型变量的转换类
    使用时直接操作成员bytes和value即可
    """

    name = ""

    def __set_name__(self, owner, name):
        self.name = name

    def __init__(self, ctype="u8", var_type=int, value_multiplier: float = 1.0):
        """Args:
        ctype (str): C-like类型(如u8, u16, u32, s8, s16, s32)
        py_var_type (_type_): python类型(如int, float)
        value_multiplier (float, optional): 值在从byte向python转换时的乘数.
        """
        self.reset(0, ctype, var_type, value_multiplier)

    def reset(self, init_value, ctype: str, py_var_type, value_multiplier: float = 1.0):
        """重置变量

        Args:
            init_value (_type_): 初始值(浮点值或整数值)
            ctype (str): C-like类型(如u8, u16, u32, s8, s16, s32)
            py_var_type (_type_): python类型(如int, float)
            value_multiplier (float, optional): 值在从byte向python转换时的乘数.
        """
        ctype_word_part = ctype[0]
        ctype_number_part = ctype[1:]
        if ctype_word_part.lower() == "u":
            self._byte_length = int(int(ctype_number_part) // 8)
            self._signed = False
            self._float = False
        elif ctype_word_part.lower() == "s":
            self._byte_length = int(int(ctype_number_part) // 8)
            self._signed = True
            self._float = False
        elif ctype == "float":
            self._byte_length = 4
            self._signed = True
            self._float = True
        else:
            raise ValueError(f"Invalid ctype: {ctype}")
        if not self._float and int(ctype_number_part) % 8 != 0:
            raise ValueError(f"Invalid ctype: {ctype}")
        if py_var_type not in [int, float, bool]:
            raise ValueError(f"Invalid var_type: {py_var_type}")
        self._var_type = py_var_type
        self._multiplier = value_multiplier
        self._value = self._var_type(init_value)
        return self

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value = self._var_type(value)

    def update_value_with_mul(self, value):
        self._value = self._var_type(value * self._multiplier)

    @property
    def bytes(self):
        if self._float:
            return struct.pack("<f", self._value / self._multiplier)
        if self._multiplier != 1:
            return int(round(self._value / self._multiplier)).to_bytes(self._byte_length, "little", signed=self._signed)
        else:
            return int(self._value).to_bytes(self._byte_length, "little", signed=self._signed)

    @bytes.setter
    def bytes(self, value):
        if self._float:
            self._value = self._var_type(struct.unpack("<f", value)[0] * self._multiplier)
        else:
            self._value = self._var_type(int.from_bytes(value, "little", signed=self._signed) * self._multiplier)

    @property
    def byte_length(self):
        return self._byte_length

    @byte_length.setter
    def byte_length(self, value):
        raise Exception("byte_length is read-only")

    @property
    def struct_fmt_type(self):
        if self._float:
            return "f"
        base_dict = {1: "b", 2: "h", 4: "i", 8: "q"}
        if self._signed:
            return base_dict[self._byte_length]
        else:
            return base_dict[self._byte_length].upper()


def decode(data: bytes) -> str:
    return "".join(chr(byte) if 32 <= byte <= 126 else f"\\x{byte:02x}" for byte in data)


class FC_State_Struct:
    motor_l_speed = Byte_Var("float", float)
    motor_l_speed_mps = Byte_Var("float", float)
    motor_l_pos = Byte_Var("s32", int)
    motor_l_degree = Byte_Var("float", float)
    motor_r_speed = Byte_Var("float", float)
    motor_r_speed_mps = Byte_Var("float", float)
    motor_r_pos = Byte_Var("s32", int)
    motor_r_degree = Byte_Var("float", float)

    RECV_ORDER = [  # 数据包顺序
        motor_l_speed,motor_l_speed_mps,motor_l_pos,motor_l_degree,
        motor_r_speed,motor_r_speed_mps,motor_r_pos,motor_r_degree,
    ]  # fmt: skip

    def __init__(self):
        self._fmt_string = "<" + "".join([i.struct_fmt_type for i in self.RECV_ORDER])
        self._fmt_length = struct.calcsize(self._fmt_string)
        self.update_event = Event()

    def update_from_bytes(self, bytes):
        if len(bytes) != self._fmt_length:
            raise ValueError(f"Invalid bytes length: {len(bytes)} != {self._fmt_length}")
        vals = struct.unpack(self._fmt_string, bytes)
        for i, val in enumerate(vals):
            self.RECV_ORDER[i].update_value_with_mul(val)
        self.update_event.set()

    @property
    def command_now(self):
        return (self.cid.value, self.cmd_0.value, self.cmd_1.value)

    def print(self, extra_info: Optional[List[str]] = None):
        RED = "\033[1;31m"
        GREEN = "\033[1;32m"
        YELLOW = "\033[1;33m"
        BLUE = "\033[1;34m"
        CYAN = "\033[1;36m"
        PURPLE = "\033[1;35m"
        RESET = "\033[0m"
        BACK = "\033[F"
        LINELIMIT = 100  # 每行最多显示的字符数
        LOG_SPACE = 3  # 为日志留出的空间
        BOXCOLOR = BLUE
        HEAD = f"{BOXCOLOR}| {RESET}"
        TAIL = f"{BOXCOLOR} |{RESET}"
        lines = [
            BOXCOLOR
            + "-" * ((LINELIMIT - 32) // 2)
            + PURPLE
            + f" ▲ System log / ▼ System status "
            + BOXCOLOR
            + "-" * ((LINELIMIT - 32) // 2)
            + RESET,
            HEAD,
        ]

        def remove_color(text):
            return re.sub(r"\033\[[0-9;]*m", "", text)

        def len_s(text):
            return len(remove_color(text))

        varlist = [
            f"{YELLOW}{var.name}: {f'{GREEN}√ ' if var.value else f'{RED}x {RESET}'}"
            if type(var.value) == bool
            else (
                f"{YELLOW}{var.name}:{CYAN}{var.value:^7.02f}{RESET}"
                if type(var.value) == float
                else f"{YELLOW}{var.name}:{CYAN}{var.value:^4d}{RESET}"
            )
            for var in self.RECV_ORDER
        ]
        if extra_info:
            varlist += extra_info
        for vartext in varlist:
            if len_s(lines[-1]) + len_s(vartext) > LINELIMIT - 2:
                lines[-1] += " " * (LINELIMIT - len_s(lines[-1]) - 2) + TAIL
                lines.append(HEAD)
            lines[-1] += vartext
        lines[-1] += " " * (LINELIMIT - len_s(lines[-1]) - 2) + TAIL
        lines.append(f"{BOXCOLOR}{'-' * LINELIMIT}{RESET}")
        for _ in range(LOG_SPACE):
            lines.insert(0, " " * LINELIMIT)
        text = "\n".join(lines) + BACK * (len(lines) - 1)
        print(text, end="")


class FC_Event:
    """飞控事件类"""

    name = ""

    def __set_name__(self, owner, name):
        self.name = name

    def __init__(self):
        self._status = False
        self._callback = None
        self._callback_trigger = True

    def __bool__(self):
        return self._status

    def set(self):
        self._status = True
        self._check_callback()

    def clear(self):
        self._status = False
        self._check_callback()

    def wait(self, timeout=None) -> bool:
        """
        等待事件置位
        Returns:
            bool: True if the event is set, False if the timeout occurred.
        """
        if timeout is None:
            while self._status == False:
                time.sleep(0.1)
        else:
            start_time = time.perf_counter()
            while self._status == False:
                time.sleep(0.1)
                if time.perf_counter() - start_time > timeout:
                    logger.warning("[FC] Wait for event timeout")
                    break
        return self._status

    def wait_clear(self, timeout=None) -> bool:
        ret = self.wait(timeout)
        if ret:
            self.clear()
        return ret

    def _check_callback(self):
        if callable(self._callback) and self._status == self._callback_trigger:
            self._callback()

    def set_callback(self, callback, trigger=True):
        """设置回调函数

        Args:
            callback (function): 目标函数
            trigger (bool, optional): 回调触发方式 (True为事件置位时触发). Defaults to True.
        """
        self._callback = callback
        self._callback_trigger = trigger

    def is_set(self) -> bool:
        return self._status


class FC_Event_Struct:
    key_short = FC_Event()
    key_long = FC_Event()
    key_double = FC_Event()

    EVENT_CODE = {
        0x01: key_short,
        0x02: key_long,
        0x03: key_double,
    }


class FC_Settings_Struct:
    wait_ack_timeout = 0.1  # 应答帧超时时间
    wait_sending_timeout = 0.2  # 发送等待超时时间
    ack_max_retry = 3  # 应答失败最大重发次数
    action_log_output = True  # 是否输出动作日志
    auto_change_mode = True  # 是否自动切换飞控模式以匹配目标动作
    raise_if_no_ack = True  # 当ACK帧校验失败时抛出异常
    raise_if_timeout = True  # 当发送等待超时时抛出异常


def get_fc_com() -> Optional[str]:
    VID_PID = "0483:5740"
    for port, desc, hwid in sorted(comports()):
        if VID_PID in hwid:
            logger.info(f"[FC] Found FC hwid on port {port}")
            return port
    return None


class FC_Base_Uart_Comunication(object):
    """
    通讯层, 实现了与飞控的直接串口通讯
    """

    def __init__(self) -> None:
        super().__init__()
        self.running = False
        self.connected = False
        self._thread_list: List[threading.Thread] = []
        self._state_update_callback: Optional[Callable[[FC_State_Struct], None]] = None
        self._print_state_flag = False
        self._reader: Optional[SerialReaderLike] = None
        self._send_lock = threading.Lock()
        self._recivied_ack_dict: Dict[int, Event] = {}
        self._radar_callback: Optional[Callable[[bytes], None]] = None
        self._uart_screen_callback: Optional[Callable[[bytes], None]] = None
        self._wireless_callback: Optional[Callable[[bytes], None]] = None
        self.state = FC_State_Struct()
        self.event = FC_Event_Struct()
        self.settings = FC_Settings_Struct()

    def start_listen_serial(
        self,
        serial_dev: Optional[str] = None,
        baudrate: int = 500000,
        print_state=False,
        callback: Optional[Callable[[FC_State_Struct], None]] = None,
        block_until_connected: bool = False,
    ):
        """连接飞控通信串口

        Args:
            serial_dev (str, optional): 串口设备名, 默认自动搜索
            baudrate (int, optional): 波特率
            print_state (bool, optional): 是否在终端打印飞控状态
            callback (Callable[[FC_State_Struct], None], optional): 状态更新回调函数

        """
        self._state_update_callback = callback
        self._print_state_flag = print_state
        if not block_until_connected:
            if serial_dev is None:
                serial_dev = get_fc_com()
                assert serial_dev, "FC comport not found"
            self._ser = serial.Serial(serial_dev, baudrate, timeout=0.5, write_timeout=0)
        else:
            while True:
                try:
                    if serial_dev is None:
                        assert (serial_dev := get_fc_com())
                    self._ser = serial.Serial(serial_dev, baudrate, timeout=0.5)
                    break
                except Exception as e:
                    logger.warning(f"[FC] Serial port open failed: {e}, retrying")
                    time.sleep(1)
        self._reader = SerialReaderBuffered(self._ser, [0xAA, 0x55])
        logger.info("[FC] Serial port opened")
        self.running = True
        _listen_thread = threading.Thread(target=self._listen_serial_task)
        _listen_thread.daemon = True
        _listen_thread.start()
        self._thread_list.append(_listen_thread)

    def close(self, joined=True) -> None:
        self.running = False
        self.connected = False
        if joined:
            for thread in self._thread_list:
                thread.join()
                self._thread_list.remove(thread)
        if self._reader:
            self._reader.close()
        logger.info("[FC] Threads closed, FC offline")

    def send_data_to_fc(
        self,
        data: bytes,
        option: int,
        need_ack: bool = False,
        no_lock: bool = False,
        _ack_retry_count: int = None,  # type: ignore
    ) -> int:
        """将数据向飞控发送, 并等待应答, 一切操作都将由该函数发送, 因此重构到
        其他通讯方式时只需重构该函数即可

        Args:
            data (bytes): bytes类型的数据
            option (int): 选项, 对应飞控代码
            need_ack (bool, optional): 是否需要应答验证
            no_lock (bool, optional): 是否不加锁发送
            _ack_retry_count (int, optional): 应答超时时最大重发次数, 此处由函数自动递归设置, 请修改settings中的选项.

        Returns:
            int: 实际发送的数据长度
        """
        assert self.running, "FC is closed"
        if need_ack:
            if _ack_retry_count is None:
                _ack_retry_count = self.settings.ack_max_retry
            check_ack = option
            for add_bit in data:
                check_ack = (check_ack + add_bit) & 0xFF
            self._recivied_ack_dict[check_ack] = Event()
            if _ack_retry_count <= 0:
                logger.error("Wait ACK reached max retry")
                if self.settings.raise_if_no_ack:
                    raise Exception("Wait ACK reached max retry")
                return 0
            send_option = option | 0x80
        else:
            send_option = option
        if isinstance(data, list):
            data = bytes(data)
        if not isinstance(data, bytes):
            raise TypeError("data must be bytes")
        send_data = b"\xaa\x22" + send_option.to_bytes(1, "little") + len(data).to_bytes(1, "little") + data
        send_data += (sum(send_data) & 0xFF).to_bytes(1, "little")  # checksum
        if not no_lock:
            if not self._send_lock.acquire(timeout=self.settings.wait_sending_timeout):
                logger.error("[FC] Wait sending data timeout")
                if self.settings.raise_if_timeout:
                    raise Exception("Wait sending data timeout")
                return 0
            try:
                cnt = self._ser.write(send_data)
            finally:
                self._send_lock.release()
        else:
            cnt = self._ser.write(send_data)
        if need_ack:
            if not self._recivied_ack_dict[check_ack].wait(self.settings.wait_ack_timeout):
                self._recivied_ack_dict.pop(check_ack)
                logger.warning(f"[FC] ACK timeout, retry - {_ack_retry_count}")
                return self.send_data_to_fc(data, option, need_ack, no_lock, _ack_retry_count - 1)  # type: ignore
            self._recivied_ack_dict.pop(check_ack)
        return cnt if cnt else 0

    def _listen_serial_task(self):
        logger.info("[FC] Listen serial thread started")
        last_heartbeat_time = time.perf_counter()
        last_receive_time = time.perf_counter()
        while self.running:
            try:
                while self._reader.read():
                    last_receive_time = time.perf_counter()
                    self._update_fc_data(self._reader.data)
                if time.perf_counter() - last_heartbeat_time > 0.25:  # 心跳包
                    self.send_data_to_fc(b"\x01", 0x00)
                    last_heartbeat_time = time.perf_counter()
                if time.perf_counter() - last_receive_time > 0.5:  # 断连检测
                    if self.connected:
                        self.connected = False
                        logger.warning("[FC] Disconnected")
                else:
                    time.sleep(0.001)  # 降低CPU占用
            except SerialException:
                logger.warning("[FC] Serialport is closed, try to reopen")
                self.connected = False
                self._ser.close()
                while self.running:
                    try:
                        self._ser.open()
                        logger.info("[FC] Serialport reopened")
                        break
                    except SerialException:
                        time.sleep(0.5)
            except Exception as e:
                logger.exception(f"[FC] Listen serial exception")
        logger.info("[FC] Listen serial thread closed")

    def _update_fc_data(self, _data: bytes):
        cmd = _data[0]
        data = _data[1:]
        if cmd == 0x01:  # 状态回传
            self.state.update_from_bytes(data)
            if not self.connected:
                self.connected = True
                logger.info("[FC] Connected")
            if callable(self._state_update_callback):
                self._state_update_callback(self.state)
            if self._print_state_flag:
                self.state.print()
        elif cmd == 0x02:  # ACK返回
            if (event := self._recivied_ack_dict.get(data[0])) is None:
                logger.warning(f"[FC] Unrecognized ACK: {data[0]}")
            else:
                event.set()
        elif cmd == 0x03:  # 事件通讯
            event_code = data[0]
            event_operator = data[1]
            if event_operator == 0x01:  # set
                self.event.EVENT_CODE[event_code].set()
                logger.debug(f"[FC] Event {self.event.EVENT_CODE[event_code].name} set")
            elif event_operator == 0x02:  # clear
                self.event.EVENT_CODE[event_code].clear()
                logger.debug(f"[FC] Event {self.event.EVENT_CODE[event_code].name} clear")
        elif cmd == 0x04:  # 雷达数据
            if self._radar_callback is not None:
                self._radar_callback(data)
        elif cmd == 0x05:  # 串口屏数据
            if self._uart_screen_callback is not None:
                self._uart_screen_callback(data)
        elif cmd == 0x06:  # 调试信息
            logger.debug(f"[FC] Message: {decode(data)}")
        elif cmd == 0x07:  # 无线通讯
            if self._wireless_callback is not None:
                self._wireless_callback(data)

    def register_radar_callback(self, func: Callable[[bytes], None], threaded=False):
        """注册雷达通讯函数, 应由雷达类调用, 用户不应调用此函数

        Args:
            func (Callable[bytes]): 参数为接收到的数据
            threaded (bool, optional): 使用线程进行异步处理
        """
        assert callable(func), "func must be callable"
        if not threaded:
            self._radar_callback = func
        else:
            self._radar_callback = lambda data: threading.Thread(target=func, args=(data,), daemon=True).start()

    def register_uart_screen_callback(self, func: Callable[[bytes], None], threaded=False):
        """注册串口屏通讯函数

        Args:
            func (Callable[bytes]): 参数为接收到的数据
            threaded (bool, optional): 使用线程进行异步处理
        """
        assert callable(func), "func must be callable"
        if not threaded:
            self._uart_screen_callback = func
        else:
            self._uart_screen_callback = lambda data: threading.Thread(target=func, args=(data,), daemon=True).start()

    def register_wireless_callback(self, func: Callable[[bytes], None], threaded=False):
        """注册无线通讯函数

        Args:
            func (Callable[bytes]): 参数为接收到的数据
            threaded (bool, optional): 使用线程进行异步处理
        """
        assert callable(func), "func must be callable"
        if not threaded:
            self._wireless_callback = func
        else:
            self._wireless_callback = lambda data: threading.Thread(target=func, args=(data,), daemon=True).start()
