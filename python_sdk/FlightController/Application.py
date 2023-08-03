import threading
import time
from typing import Optional

import numpy as np
from FlightController.Protocal import FC_Protocol
from loguru import logger


class FC_Application(FC_Protocol):
    """
    应用层, 基于协议层进行开发, 不触及底层通信
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

    def set_action_log(self, output: bool) -> None:
        """
        设置动作日志输出
        """
        self.settings.action_log_output = output


    def wait_for_connection(self, timeout_s=-1) -> bool:
        """
        等待飞控连接
        """
        t0 = time.perf_counter()
        while not self.connected:
            time.sleep(0.1)
            if timeout_s > 0 and time.perf_counter() - t0 > timeout_s:
                logger.warning("[FC] wait for fc connection timeout")
                return False
        self._action_log("wait ok", "fc connection")
        return True
