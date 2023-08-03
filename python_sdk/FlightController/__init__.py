import sys as _sys
from typing import Union as _Union

from loguru import logger

from .Application import FC_Application as __FC_without_remote_layer__
from .Remote import FC_Client, FC_Server


class FC_Controller(__FC_without_remote_layer__):
    """
    本地飞控
    """

    pass  # 只是个别名


FC_Like = _Union[FC_Controller, FC_Client, FC_Server]  # type annotations
__all__ = ["FC_Controller", "FC_Client", "FC_Server", "logger", "FC_Like"]

logger.remove()
logger.add(
    "fc_log/{time}.log",
    retention="1day",
    level="DEBUG",
    backtrace=True,
    diagnose=True,
    filter=lambda record: "debug" not in record["extra"],
)
logger.add(
    "fc_log/navigation_debug_{time}.log",
    retention="1day",
    filter=lambda record: "debug" in record["extra"],
    level="DEBUG",
)
logger.add(_sys.stdout, filter=lambda record: "debug" not in record["extra"], level="DEBUG")
