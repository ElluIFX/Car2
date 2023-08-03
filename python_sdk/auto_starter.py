"""
自启动管理器
"""

import os
import sys
from typing import Callable, Dict, Literal, Optional

PATH = os.path.dirname(os.path.abspath(__file__))
os.chdir(PATH)
print(f"Current path: {PATH}")


def system_type() -> Literal["Windows", "Linux", "Darwin"]:
    """
    获取系统类型
    :return: 系统类型
    """
    if sys.platform.startswith("win"):
        return "Windows"
    elif sys.platform.startswith("linux"):
        return "Linux"
    else:
        raise Exception("Unsupported system type")


print(f"System type: {system_type()}")


def run_py_script_in_background(pwd: str, path: str, *args, **kwargs):
    """
    在后台运行Python脚本
    :param path: 脚本路径
    :param args: 脚本参数
    :param kwargs: 脚本参数
    """
    pwd = os.path.abspath(pwd)
    path = os.path.abspath(path)
    if system_type() == "Windows":
        arg = f"cd {pwd} &"
        arg += f"start python {path} {' '.join(args)}"
        # subprocess.Popen(["python", path, *args])
    else:
        arg = f"cd {pwd} &&"
        arg += f"python3 {path} {' '.join(args)}"
        # subprocess.Popen(["python3", path, *args])
    os.system(arg)


try:
    run_py_script_in_background(PATH, "./server.py")
except:
    import traceback

    traceback.print_exc()
    input("Press any key to exit...")
