import os
import time
from typing import Dict, List

from loguru import logger

from .Utils import Tmux


class RosManager(object):
    """
    ROS 后台包管理器
    (单实例模式)
    """

    __instance = None
    _running_packages: List[str] = []
    _running_tmux: Dict[str, Tmux] = {}
    _running_commands: Dict[str, str] = {}

    def __new__(cls, *args, **kwargs):
        if not RosManager.__instance:
            RosManager.__instance = object.__new__(cls)
        return RosManager.__instance

    def __init__(self) -> None:
        pass

    def _bringup(self, action: str, package_name: str, launch_file: str, launch_args: str, sub_id, kill_exist):
        full_name = f"{package_name}_{sub_id}"
        if full_name in self._running_packages:
            if kill_exist:
                self.kill_package(package_name, sub_id)
            else:
                logger.error(f"[ROS Manager] Package {full_name} is already running, skip")
                return
        cmd = f"ros2 {action} {package_name} {launch_file} {launch_args}"
        tmux = Tmux(full_name)
        if full_name in Tmux.get_running_sessions():
            if kill_exist:
                logger.warning(f"[ROS Manager] Package {full_name} seems to be running, kill it")
                tmux.send_key_interruption()
                time.sleep(1)
                tmux.kill_session()
            else:
                logger.warning(f"[ROS Manager] Package {full_name} seems to be running, skip")
                self._running_packages.append(full_name)
                self._running_tmux[full_name] = tmux
                self._running_commands[full_name] = cmd
                return
        tmux.new_session()
        time.sleep(1)
        tmux.send_command(cmd)
        logger.info(f"[ROS Manager] Package {full_name} launched")
        self._running_packages.append(full_name)
        self._running_tmux[full_name] = tmux
        self._running_commands[full_name] = cmd

    def launch_package(self, package_name: str, launch_file: str, launch_args: str = "", sub_id=0, kill_exist=True):
        """Launch a package

        Args:
            package_name/launch_file/launch_args: Same as what you use in terminal
            sub_id (int, optional): Specify a sub id when you want to launch multiple packages with same name
            kill_exist (bool, optional): Kill and recreate session if it is already running

        Returns:
            bool: True if launch success, False if already running
        """
        return self._bringup("launch", package_name, launch_file, launch_args, sub_id, kill_exist)

    def run_package(self, package_name: str, launch_file: str, launch_args: str = "", sub_id=0, kill_exist=True):
        """Run a package

        Args:
            package_name/launch_file/launch_args: Same as what you use in terminal
            sub_id (int, optional): Specify a sub id when you want to launch multiple packages with same name
            kill_exist (bool, optional): Kill and recreate session if it is already running

        Returns:
            bool: True if launch success, False if already running
        """
        return self._bringup("run", package_name, launch_file, launch_args, sub_id, kill_exist)

    def kill_package(self, package_name: str, sub_id=0):
        full_name = f"{package_name}_{sub_id}"
        if full_name in self._running_packages:
            self._running_tmux[full_name].send_key_interruption()
            time.sleep(1)
            self._running_tmux[full_name].kill_session()
            self._running_packages.remove(full_name)
            self._running_tmux.pop(full_name)
            self._running_commands.pop(full_name)
            logger.info(f"[ROS Manager] Package {full_name} killed")
        else:
            logger.error(f"[ROS Manager] Package {full_name} is not running")

    def reboot_package(self, package_name: str, sub_id=0):
        full_name = f"{package_name}_{sub_id}"
        if full_name not in self._running_packages:
            raise Exception(f"[ROS Manager] Package {full_name} is not running")
        self._running_tmux[full_name].send_key_interruption()
        time.sleep(1)
        self._running_tmux[full_name].kill_session()
        self._running_tmux[full_name].new_session()
        time.sleep(1)
        self._running_tmux[full_name].send_command(self._running_commands[full_name])
        logger.info(f"[ROS Manager] Package {full_name} rebooted")

    def kill_all_packages(self):
        for package_name in self._running_packages[:]:
            self.kill_package(package_name)
        logger.info(f"[ROS Manager] All packages killed")

    def reboot_all_packages(self):
        for package_name in self._running_packages:
            self.reboot_package(package_name)
        logger.info(f"[ROS Manager] All packages rebooted")

    def send_command_to_package(self, package_name: str, command: str, enter: bool = True, sub_id=0):
        """
        Send command (key sequence) to a running package's tmux session
        """
        full_name = f"{package_name}_{sub_id}"
        if full_name not in self._running_packages:
            raise Exception(f"[ROS Manager] Package {full_name} is not running")
        self._running_tmux[full_name].send_command(command, enter)
        logger.debug(f"[ROS Manager] Send command '{command}' to {full_name}")

    def get_log(self, package_name: str, line_num, sub_id=0):
        """
        Get log output of a running package
        """
        full_name = f"{package_name}_{sub_id}"
        if full_name not in self._running_packages:
            raise Exception(f"[ROS Manager] Package {full_name} is not running")
        output = self._running_tmux[full_name].capture_output(tail=line_num + 20)
        return [line.strip() for line in output.split("\n") if line.strip() != ""][-line_num:]

    @staticmethod
    def get_running_nodes() -> List[str]:
        return os.popen("ros2 node list").read().split("\n")

    @staticmethod
    def get_running_topics() -> List[str]:
        return os.popen("ros2 topic list").read().split("\n")

    @staticmethod
    def get_running_services() -> List[str]:
        return os.popen("ros2 service list").read().split("\n")

    @staticmethod
    def chmod(path: str, permission: str = "777"):
        """
        Change permission of a file or folder using sudo
        Modify sudoers file to allow user to run sudo without password
        """
        os.system(f"sudo chmod {permission} {path}")
        logger.debug(f"[ROS Manager] Change permission of {path} to {permission}")

    def is_running(self, package_name: str, sub_id=0) -> bool:
        return f"{package_name}_{sub_id}" in self._running_packages

    def is_live(self, package_name: str, sub_id=0) -> bool:
        full_name = f"{package_name}_{sub_id}"
        if full_name not in self._running_packages:
            return False
        return self._running_tmux[full_name].session_busy
