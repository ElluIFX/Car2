import struct
from threading import Event
from typing import Callable, List, Literal, Optional, Union

from FlightController import FC_Controller, FC_Like
from loguru import logger

from .Utils import decode_hex_str, decode_human_str


class UARTScreen:
    """
    三易串口屏驱动
    """

    def __init__(self, fc: FC_Like) -> None:
        self._fc = fc
        self._data: Union[int, float, str, None] = None
        self._report_callback: Optional[Callable[[str], None]] = None
        self._data_update_event = Event()
        fc.register_uart_screen_callback(self._callback)

    def _callback(self, data: bytes):
        if data[0] == ord("\\"):
            data = data[1:-2]
            report = data.decode("utf-8")
            if self._report_callback is not None:
                self._report_callback(report)
            # logger.debug(f"[UartScr] Report: {report}")

        else:
            cmdtype = data[0]
            if cmdtype == 0x00:  # 指令结果
                result = data[1]
                if result == 0x00:
                    logger.debug("[UartScr] Command excuted")
                elif result == 0x01:
                    logger.error("[UartScr] Command invalid")
                elif 0x02 <= result <= 0x06:
                    logger.error(f"[UartScr] Param {result-1} invalid")
                elif result == 0x07:
                    logger.error("[UartScr] Param number invalid")
                elif result == 0x08:
                    logger.error(f"[UartScr] Uart overtime")
            elif cmdtype == 0x01:  # 四字节小端整数
                value = int.from_bytes(data[1:5], "little")
                self._data = value
                self._data_update_event.set()
                logger.debug(f"[UartScr] Int data: {value}")
            elif cmdtype == 0x02:  # 四字节小端浮点数
                value = struct.unpack("<f", data[1:5])[0]
                self._data = value
                self._data_update_event.set()
                logger.debug(f"[UartScr] Float data: {value}")
            elif cmdtype == 0x03:  # 字符串
                data = data[1:-2]
                self._data = data.decode("utf-8")
                self._data_update_event.set()
                logger.debug(f"[UartScr] String data: {self._data}")

    def send_command(self, cmd: str):
        """发送指令"""
        if not cmd.endswith("\r\n"):
            cmd += "\r\n"
        self._fc.send_to_uart_screen(cmd.encode())

    def send_string(self, string: str):
        """发送字符串"""
        self._fc.send_to_uart_screen(string.encode())

    def register_report_callback(self, callback: Callable[[str], None]):
        """注册报告(以\\开头的串口数据)回调函数"""
        self._report_callback = callback

    def get_system_value(self, param: str) -> Union[int, float, str, None]:
        """获取系统参数"""
        self._data_update_event.clear()
        self.send_command(f"sget {param}")
        if not self._data_update_event.wait(1):
            return None
        return self._data

    def set_system_value(self, param: str, *args: Union[int, float, str]):
        """设置系统参数"""
        cmd = f"sset {param}"
        for arg in args:
            cmd += f" {arg}"
        self.send_command(cmd)

    def get_widget_value(self, widget: str) -> Union[int, float, str, None]:
        """获取控件参数"""
        self._data_update_event.clear()
        self.send_command(f"wget {widget}")
        if not self._data_update_event.wait(1):
            return None
        return self._data

    def set_widget_value(self, widget: str, *args: Union[int, float, str]):
        """设置控件参数"""
        cmd = f"wset {widget}"
        for arg in args:
            cmd += f" {arg}"
        self.send_command(cmd)

    def page(self, name_id: Union[str, int]):
        """
        切换页面
        """
        self.send_command(f"page {name_id}")

    def event(self, name, value):
        """
        模拟触发控件事件
        """
        self.send_command(f"event {name} {value}")

    def click(self, name):
        """
        模拟点击控件
        """
        self.send_command(f"click {name}")

    def addt(
        self,
        page_id: int,
        widget_id: int,
        channel: Literal[0, 1, 2],
        data_type: Literal["int", "float", "char", "short"],
        data: List[Union[int, float]],
    ):
        """
        曲线数据透传
        """
        type_str = {"int": "i", "float": "f", "char": "B", "short": "h"}
        byte_data = struct.pack(f"<{len(data)}{type_str[data_type]}", *data)
        front_data = struct.pack("<BBBH", page_id, widget_id, channel, len(byte_data))
        send_data = "addt".encode() + front_data + b"\00\00\00" + byte_data
        self._fc.send_to_uart_screen(send_data)


if __name__ == "__main__":
    import random

    fc = FC_Controller()
    fc.start_listen_serial()
    fc.wait_for_connection()
    urs = UARTScreen(fc)
