import datetime
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, List, Literal, Tuple

import numpy as np
from loguru import logger
from scipy.spatial.transform import Rotation

from .Utils import (
    quaternions_to_euler,
    quaternions_to_rotation_matrix,
    rotation_matrix_to_quaternions,
)


@dataclass()
class T265_Pose_Frame(object):
    """
    T265 姿态数据帧
    (单位: m, T265坐标系)
    """

    @dataclass
    class _XYZ:  # 三维坐标
        x: float
        y: float
        z: float

    @dataclass
    class _WXYZ:  # 四元数
        w: float
        x: float
        y: float
        z: float

    translation: _XYZ  # 位移 / m
    rotation: _WXYZ  # 四元数姿态
    velocity: _XYZ  # 速度 / m/s
    acceleration: _XYZ  # 加速度 / m/s^2
    angular_velocity: _XYZ  # 角速度 / rad/s
    angular_acceleration: _XYZ  # 角加速度 / rad/s^2
    tracker_confidence: int  # 跟踪置信度 0: Failed, 1: Low, 2: Medium, 3: High
    mapper_confidence: int  # 建图置信度 0: Failed, 1: Low, 2: Medium, 3: High

    @staticmethod
    def get_zero() -> "T265_Pose_Frame":
        return T265_Pose_Frame(
            translation=T265_Pose_Frame._XYZ(0, 0, 0),
            rotation=T265_Pose_Frame._WXYZ(1, 0, 0, 0),
            velocity=T265_Pose_Frame._XYZ(0, 0, 0),
            acceleration=T265_Pose_Frame._XYZ(0, 0, 0),
            angular_velocity=T265_Pose_Frame._XYZ(0, 0, 0),
            angular_acceleration=T265_Pose_Frame._XYZ(0, 0, 0),
            tracker_confidence=0,
            mapper_confidence=0,
        )


"""
note:
T265 姿态坐标系
            y
         z  ^
          \ |
           \|
   x<---[ (O O)]
pitch-dx, yaw-dy, roll-dz
所有轴向均为右手系
"""


class T265(object):
    """
    Realsense T265 包装类
    """

    def __init__(self, connection: Literal["raw", "ros"] = "raw", **args) -> None:
        """
        初始化 T265

        connection: 连接方式(raw: python驱动, ros: ros订阅)
        """
        self.pose: T265_Pose_Frame = T265_Pose_Frame.get_zero()  # type: ignore
        self.frame_num: int = 0  # frame number
        self.frame_timestamp: float = 0.0  # timestamp
        self.running = False
        self.update_event = threading.Event()
        self._callbacks: List[List] = []
        self._update_count = 0
        self._last_trans_args: Any = None
        self._connection_type = connection
        if connection == "raw":
            self._connect_rs(**args)
        elif connection == "ros":
            from .RosNode import T265ListenNode

            # 解耦:按需导入ROS相关的模块,防止非ROS环境下无法运行
            self._ros_node = T265ListenNode(self._callback_ros)
        else:
            raise ValueError(f"Invalid connection type: {connection}")

    def _connect_rs(self, **args) -> None:
        try:
            import pyrealsense2 as rs

            rs.config()
        except:
            import pyrealsense2.pyrealsense2 as rs  # for linux
        log_level = args.get("log_to_file", "info")
        if args.get("log_to_file", False):
            rs.log_to_file(getattr(rs.log_severity, log_level), "rs_t265.log")
        if args.get("log_to_console", False):
            rs.log_to_console(getattr(rs.log_severity, log_level))
        self._connect_args = args
        self._pipe = rs.pipeline()
        self._cfg = rs.config()
        self._cfg.enable_stream(rs.stream.pose)
        self._device = self._cfg.resolve(self._pipe).get_device()
        logger.info(f"[T265] Connected to {self._device}")
        logger.debug(f"[T265] Device sensors: {self._device.query_sensors()}")
        pose_sensor = self._device.first_pose_sensor()
        logger.debug(f"[T265] Pose sensor: {pose_sensor}")
        pose_sensor.set_option(rs.option.enable_auto_exposure, args.get("enable_auto_exposure", 1))
        pose_sensor.set_option(rs.option.enable_mapping, args.get("enable_mapping", 1))
        pose_sensor.set_option(rs.option.enable_map_preservation, args.get("enable_map_preservation", 1))
        pose_sensor.set_option(rs.option.enable_relocalization, args.get("enable_relocalization", 1))
        pose_sensor.set_option(rs.option.enable_pose_jumping, args.get("enable_pose_jumping", 1))
        pose_sensor.set_option(rs.option.enable_dynamic_calibration, args.get("enable_dynamic_calibration", 1))
        # logger.debug(f"[T265] Pose sensor options:")
        # for opt in pose_sensor.get_supported_options():
        #     logger.debug(f"[T265]   {opt}: {pose_sensor.get_option(opt)}")

    def _updated(self):
        if self._print_update:
            self._print_pose()
        self._update_count += 1
        if self._callbacks:
            for item in self._callbacks:
                item[1] += 1
                if item[1] >= item[2]:
                    item[0](self.pose, self.frame_num, self.frame_timestamp)
                    item[1] = 0
        if self._update_count >= self.event_skip:
            self.update_event.set()
            self._update_count = 0

    def _callback_rs(self, frame) -> None:
        pose = frame.as_pose_frame()
        if not pose:
            return
        self.frame_num = pose.frame_number
        self.frame_timestamp = pose.timestamp
        self.pose = pose.get_pose_data()
        self._updated()

    def _callback_ros(self, data) -> None:
        self.frame_num += 1
        # y -> z, x -> -y, z -> -x
        self.pose.translation.x = -data.pose.pose.position.y
        self.pose.translation.y = data.pose.pose.position.z
        self.pose.translation.z = -data.pose.pose.position.x
        self.pose.rotation.w = data.pose.pose.orientation.w
        self.pose.rotation.x = -data.pose.pose.orientation.y
        self.pose.rotation.y = data.pose.pose.orientation.z
        self.pose.rotation.z = -data.pose.pose.orientation.x
        self.pose.velocity.x = -data.twist.twist.linear.y
        self.pose.velocity.y = data.twist.twist.linear.z
        self.pose.velocity.z = -data.twist.twist.linear.x
        self.pose.angular_velocity.x = -data.twist.twist.angular.y
        self.pose.angular_velocity.y = data.twist.twist.angular.z
        self.pose.angular_velocity.z = -data.twist.twist.angular.x
        self.pose.tracker_confidence = 3
        self.pose.mapper_confidence = 3
        self._updated()

    def _print_pose(self, refresh=True) -> None:
        BACK = "\033[F"
        r, p, y = self.eular_rotation
        text = (
            f"T265 Pose Frame #{self.frame_num} at {datetime.datetime.fromtimestamp(self.frame_timestamp / 1000)}\n"
            f"Translation    :{self.pose.translation.x:11.6f},{self.pose.translation.y:11.6f},{self.pose.translation.z:11.6f};\n"
            f"Velocity       :{self.pose.velocity.x:11.6f},{self.pose.velocity.y:11.6f},{self.pose.velocity.z:11.6f};\n"
            f"Acceleration   :{self.pose.acceleration.x:11.6f},{self.pose.acceleration.y:11.6f},{self.pose.acceleration.z:11.6f};\n"
            f"Angular vel    :{self.pose.angular_velocity.x:11.6f},{self.pose.angular_velocity.y:11.6f},{self.pose.angular_velocity.z:11.6f};\n"
            f"Angular accel  :{self.pose.angular_acceleration.x:11.6f},{self.pose.angular_acceleration.y:11.6f},{self.pose.angular_acceleration.z:11.6f};\n"
            f"Rotation       :{self.pose.rotation.w:11.6f},{self.pose.rotation.x:11.6f},{self.pose.rotation.y:11.6f},{self.pose.rotation.z:11.6f};\n"
            f"Roll/Pitch/Yaw :{r:11.6f},{p:11.6f},{y:11.6f};\n"
            f"Tracker conf: {self.pose.tracker_confidence}, Mapper conf: {self.pose.mapper_confidence}"
        )
        if self._last_trans_args is not None:
            position, eular = self.get_pose_in_secondary_frame(self._last_trans_args, as_eular=True)
            text += (
                f"\n2nd Translation   : {position[0]:11.6f},{position[1]:11.6f},{position[2]:11.6f};\n"
                f"2nd Roll/Pitch/Yaw: {eular[0]:11.6f},{eular[1]:11.6f},{eular[2]:11.6f}"
            )
            if refresh:
                text += BACK * 3
        if refresh:
            print(f"{text}{BACK* 8}", end="")

    def start(self, async_update: bool = True, print_update: bool = False, event_skip=0) -> None:
        """
        开始监听 T265

        async_update: 是否使用异步回调的方式监听, 若为 False, 则需要手动调用 update() 方法
        print_update: 是否在控制台打印更新
        event_skip: 事件触发间隔(每隔多少次更新触发一次事件)
        """
        self._print_update = print_update
        self.event_skip = event_skip
        self._update_count = 0
        self._start_time = time.perf_counter()
        self.running = True
        if self._connection_type == "ros":
            logger.info("[T265] Running in ROS mode, ready for topic update")
            return
        self._async = async_update
        if self._async:
            self._pipe.start(self._cfg, self._callback_rs)
        else:
            self._pipe.start(self._cfg)
        logger.info("[T265] Started")

    def update(self):
        """
        更新 T265 状态(阻塞直到有新的数据帧到来)
        """
        if not self.running:
            raise RuntimeError("T265 is not running")
        if self._async:
            raise RuntimeError("Async mode")
        frames = self._pipe.wait_for_frames()
        pose = frames.get_pose_frame()
        if not pose:
            return
        self.frame_num = pose.frame_number
        self.frame_timestamp = pose.timestamp
        self.pose = pose.get_pose_data()
        if self._print_update:
            self._print_pose()
        self._update_count += 1
        if self._callbacks:
            for item in self._callbacks:
                item[1] += 1
                if item[1] >= item[2]:
                    item[0](self.pose, self.frame_num, self.frame_timestamp)
                    item[1] = 0
        if self._update_count >= self.event_skip:
            self.update_event.set()
            self._update_count = 0

    def register_callback(
        self, callback: Callable[[T265_Pose_Frame, int, float], None], callback_skip: int = 0
    ) -> None:
        """
        注册 T265 更新回调函数
        回调参数: T265_Pose_Frame, 帧编号, 帧时间戳(ms)
        """
        self._callbacks.append([callback, 0, callback_skip])

    def unregister_callback(self, callback: Callable[[T265_Pose_Frame, int, float], None]) -> None:
        """
        注销 T265 更新回调函数
        """
        for item in self._callbacks:
            if item[0] == callback:
                self._callbacks.remove(item)

    def stop(self) -> None:
        """
        停止监听 T265
        """
        self.running = False
        if self._connection_type != "ros":
            self._pipe.stop()
        logger.info("[T265] Stopped")

    @property
    def fps(self) -> float:
        """
        获取 T265 的平均更新速率
        """
        fps = self._update_count / (time.perf_counter() - self._start_time)
        self._update_count = 0
        self._start_time = time.perf_counter()
        return fps

    def hardware_reset(self) -> None:
        """
        强制重置 T265 并重新连接
        """
        self._device.hardware_reset()
        logger.warning("[T265] Hardware reset, waiting for reconnection...")
        cnt = 0
        while True:
            try:
                self._connect_rs(**self._connect_args)
                break
            except RuntimeError:
                time.sleep(1)
                cnt += 1
                assert cnt < 8, "T265 reconnection failed"
        if self.running:
            if self._async:
                self._pipe.start(self._cfg, self._callback_rs)
            else:
                self._pipe.start(self._cfg)
            self._update_count = 0
            self._start_time = time.perf_counter()

    @property
    def eular_rotation(self) -> np.ndarray:
        """
        获取欧拉角姿态
        返回值: roll, pitch, yaw / deg
        """
        # in convert matrices: roll (x), pitch (y), yaw (z)
        # so we swap axis: x, y, z = r_z, r_x, r_y
        # return quaternions_to_euler(
        #     self.pose.rotation.z, self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.w
        # )

        return Rotation.from_quat(
            [self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w]
        ).as_euler("zxy", degrees=True)

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
        position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])  # type: ignore
        orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])  # type: ignore
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
        (单位: m, T265坐标系)
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
        position = np.array([self.pose.translation.x, self.pose.translation.y, self.pose.translation.z])  # type: ignore
        orientation = np.array([self.pose.rotation.x, self.pose.rotation.y, self.pose.rotation.z, self.pose.rotation.w])  # type: ignore
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


if __name__ == "__main__":
    t265 = T265()
    # t265.hardware_reset()
    t265.start(print_update=True)
    try:
        while True:
            time.sleep(0.1)
            get = input(">>> \n")
            if get == "e":
                t265.establish_secondary_origin()
    finally:
        t265.stop()
