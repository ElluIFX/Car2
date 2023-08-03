import time
from threading import Event
from typing import Optional, Tuple

import numpy as np
from cartographer_ros_msgs.msg import SensorTopics, TrajectoryOptions
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped, Twist
from loguru import logger
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from scipy.spatial.transform import Rotation

from .RosManager import RosManager
from .RosNode import MapClientNode, MapListenNode, PoseListenNode, Tf2ListenNode

"""
                               ^
    ---------------------------| x
    |                          |
    |              ^           |
    |              | car_x     |
    |              |           |
    |              |           |
    |     <--------*           |
    |   car_y     car_origin   |
    |                          |
 <-----------------------------|---
    y                   OccupacyGrid_origin

参数计算关系:
OccupacyGrid_origin.x = - car_origin_to_map_back;
OccupacyGrid_origin.y = - 0.5*height*resolution;
OccupacyGrid_origin.yaw = 0;
"""


class RosMapper(object):
    """
    ROS 地图导航模块 (Cartographer)
    """

    def __init__(self, get_trans=True, get_map=True) -> None:
        if get_trans:
            self._trans_node = Tf2ListenNode(
                self.trans_callback, source_frame="camera_pose_frame", target_frame="map", frequency=30
            )
        if get_map:
            self._map_node = MapListenNode(self.map_callback)
        self._map_client = MapClientNode()
        self.map = OccupancyGrid()
        self.trans = Transform()
        self._trans_update_count = 0
        self.trans_update_event = Event()
        self.trans_event_skip = 1
        self._map_update_count = 0
        self.map_update_event = Event()
        self.map_event_skip = 1
        logger.info("[MAPPER] Init success")

    def map_callback(self, msg: OccupancyGrid):
        self.map = msg
        self._map_update_count += 1
        if self._map_update_count >= self.map_event_skip:
            self.map_update_event.set()
            self._map_update_count = 0

    def trans_callback(self, msg: Transform):
        self.trans = msg
        self._trans_update_count += 1
        if self._trans_update_count >= self.trans_event_skip:
            self.trans_update_event.set()
            self._trans_update_count = 0

    def start_trajectory(self, options: TrajectoryOptions, topics: SensorTopics) -> int:
        """
        开始建图

        Args:
            topics: 传感器话题
            options: 建图参数

        Returns:
            trajectory_id: 建图ID
        """
        result = self._map_client.start_trajectory(options, topics)
        logger.debug(f"[MAPPER] Start trajectory: {result}")
        return result.trajectory_id  # type: ignore

    def finish_trajectory(self, trajectory_id: int) -> None:
        """
        结束建图

        Args:
            trajectory_id: 建图ID
        """
        self._map_client.finish_trajectory(trajectory_id)

    def write_state(self, filename: str, include_unfinished_submaps: bool = True) -> None:
        """
        保存地图

        Args:
            filename: 地图文件名(.pbstream)
            include_unfinished_submaps: 是否包含未完成的子图
        """
        self._map_client.write_state(filename, include_unfinished_submaps)

    def load_state(self, state_filename: str) -> None:
        """
        加载地图(重启Cartographer并加载地图)

        Args:
            state_filename: 地图文件名(.pbstream)
        """
        rm = RosManager()
        if rm.is_running("cartographer_ros"):
            rm.kill_package("cartographer_ros")
        rm.launch_package("cartographer_ros", "cartographer.launch", f":=load_state_filename={state_filename}")
        logger.debug(f"[MAPPER] Load state: {state_filename}, waiting for Cartographer restart")
        while "/map" not in rm.get_running_topics():
            time.sleep(1)
        logger.debug(f"[MAPPER] Cartographer restart success")

    def reboot_cartographer(self) -> None:
        """
        重启Cartographer
        """
        rm = RosManager()
        if not rm.is_running("cartographer_ros"):
            rm.launch_package("cartographer_ros", "cartographer.launch")
        else:
            rm.reboot_package("cartographer_ros")
        logger.debug(f"[MAPPER] Reboot Cartographer, waiting for Cartographer restart")
        while "/map" not in rm.get_running_topics():
            time.sleep(1)
        logger.debug(f"[MAPPER] Cartographer restart success")

    @property
    def position(self) -> np.ndarray:
        """
        获取当前位置(x, y, z)
        (单位: m, 匿名(ROS)坐标系)
        """
        return np.array([self.trans.translation.x, self.trans.translation.y, self.trans.translation.z])

    @property
    def orientation(self) -> np.ndarray:
        """
        获取当前姿态(x, y, z, w)
        """
        return np.array(
            [
                self.trans.rotation.x,
                self.trans.rotation.y,
                self.trans.rotation.z,
                self.trans.rotation.w,
            ]
        )

    @property
    def eular_rotation(self) -> np.ndarray:
        """
        获取欧拉角姿态
        返回值: roll, pitch, yaw / degree
        """
        return Rotation.from_quat(self.orientation).as_euler("yxz", degrees=True)

    def establish_secondary_origin(
        self,
        force_level: bool = True,
        x_offset: float = 0.0,
        y_offset: float = 0.0,
        z_offset: float = 0.0,
        yaw_offset: float = 0,
    ) -> Tuple[np.ndarray, Rotation, np.ndarray, np.ndarray, float]:
        """
        以当前位置和姿态建立副坐标系原点
        force_level: 强制副坐标系为水平面
        offset: 当前位置相对于副坐标系原点的偏移
        (yaw_offset: 仅当返回eular时有效)
        return: 变换参数
        """
        # 获取当前位置和朝向
        position = self.position
        orientation = self.orientation
        if force_level:
            orientation[0] = 0
            orientation[2] = 0
        _secondary_position = position
        _secondary_rotation = Rotation.from_quat(orientation)  # xyzw
        _secondary_rotation_matrix = _secondary_rotation.as_matrix()
        _offset_position = np.array([x_offset, y_offset, z_offset])  # type: ignore
        _offset_yaw = yaw_offset
        trans_args = (
            _secondary_position,
            _secondary_rotation,
            _secondary_rotation_matrix,
            _offset_position,
            _offset_yaw,
        )
        self._last_trans_args = trans_args
        return trans_args

    def get_pose_in_secondary_frame(
        self, trans_args: Tuple[np.ndarray, Rotation, np.ndarray, np.ndarray, float], as_eular=True
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取当前位置和姿态在副坐标系中的表示
        trans_args: 副坐标系变换参数(由establish_secondary_origin获得)
        as_eular: 是否返回欧拉角
        (单位: m, 匿名(ROS)坐标系)
        return: xyz位置, xyzw四元数/rpy欧拉角
        """
        (
            _secondary_position,
            _secondary_rotation,
            _secondary_rotation_matrix,
            _offset_position,
            _offset_yaw,
        ) = trans_args
        # 获取当前位置和朝向
        position = self.position
        orientation = self.orientation
        # 将当前位置和朝向转换到副坐标系中
        position -= _secondary_position
        # 反向应用副坐标系的旋转矩阵
        position = np.dot(position, _secondary_rotation_matrix.T)
        # 反向应用副坐标系的朝向
        rotation = Rotation.from_quat(orientation) * _secondary_rotation.inv()

        position += _offset_position
        if as_eular:
            euler = rotation.as_euler("zxy", degrees=True)
            if _offset_yaw != 0:
                euler[2] = (euler[2] + _offset_yaw + 180) % 360 - 180
            return position, euler
        return position, rotation.as_quat()
