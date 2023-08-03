import threading
import time
from typing import Any, List, Literal, Optional, Tuple, Union

import numpy as np
from attr import dataclass
from FlightController import FC_Like
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265, T265_Pose_Frame
from loguru import logger
from simple_pid import PID

from .PathPlanner import PFBPP, TrajectoryGenerator

logger_dbg = logger.bind(debug=True)


class PARAMS:
    ######## 解算参数 ########
    MAP_SIZE = 1000  # 雷达扫网定位图像大小
    POLYLINE = False  # 雷达扫网定位图像是否导出线框图
    SCALE_RATIO = 0.9  # 雷达扫网定位缩放比例
    LOW_PASS_RATIO = 0.6  # 雷达扫网定位低通滤波系数
    ######## 频率除数 影响PID更新频率 ########
    RADAR_SKIP = 1  # 雷达更新事件频率除数 python管理雷达:400/PACKLEN/RADAR_SKIP ROS管理雷达:1/RS_SKIP
    RS_SKIP = 3  # T265更新事件频率除数 200/RS_SKIP
    MAP_SKIP = 1  # ROS建图更新事件频率除数 5/MAP_SKIP
    FUSION_SKIP = 10  # 雷达融合T265频率除数 python管理雷达:400/PACKLEN/RADAR_SKIP/RS_SKIP/FUSION_SKIP ROS不使用该参数


class Navigation(object):
    """
    闭环导航, 使用realsense T265作为位置闭环, 使用雷达SLAM作为定位校准
    """

    def __init__(self, *args, **kwargs):
        """
        Args:
            fc: 飞控实例(必须) (FC_Controller, FC_Client, FC_Server)
            radar: 雷达实例(必须) (LD_Radar)
            rs: realsense实例(必须) (T265)
            mapper: ROS地图模块实例(可选) (RosMapper)
        """
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.rs: T265 = kwargs["rs"]
        if "mapper" in kwargs:
            from FlightController.Components.RosMapper import RosMapper

            # 解耦:按需导入ROS相关的模块,防止非ROS环境下无法运行
            self.mapper: Optional[RosMapper] = kwargs["mapper"]
        else:
            self.mapper = None
        ############### PID #################
        self.navi_speed = 40  # 导航速度 / cm/s
        self.pid_tunings = {  # PID参数 (仅导航XY使用)
            "default": (0.35, 0.0, 0.08),  # 默认
            "navi": (1.4, 0.0, 0.02),  # 导航
            "hover": (0.65, 0.0, 0.02),  # 悬停
            "land": (0.85, 0.0, 0.02),  # 降落
        }
        self.height_pid = PID(0.8, 0.0, 0.1, setpoint=0, output_limits=(-30, 30), auto_mode=False)
        self.navi_x_pid = PID(
            *self.pid_tunings["default"],
            setpoint=0,
            output_limits=(-self.navi_speed, self.navi_speed),
            auto_mode=False,
        )
        self.navi_y_pid = PID(
            *self.pid_tunings["default"],
            setpoint=0,
            output_limits=(-self.navi_speed, self.navi_speed),
            auto_mode=False,
        )
        self.yaw_pid = PID(0.7, 0.0, 0.05, setpoint=0, output_limits=(-30, 30), auto_mode=False)
        #####################################
        self.current_x = 0  # 当前位置X(相对于基地点) / cm
        self.current_y = 0  # 当前位置Y(相对于基地点) / cm
        self.current_yaw = 0  # 当前偏航角(顺时针为正) / deg
        self.current_height = 0  # 当前高度(激光高度) / cm
        self.current_height_rs = 0.0  # 当前高度(realsense高度) / cm
        self.basepoint: Any = np.array([0.0, 0.0])  # 基地点(雷达坐标系)(Note:仅用于雷达扫网定位,建图则不需要) / cm
        #####################################
        self.keep_height_flag = False  # 定高状态
        self.navigation_flag = False  # 导航状态
        self.keep_height_by_rs = False  # 使用realsense定高
        self.running = False
        self._thread_list: List[threading.Thread] = []
        self.traj_running_event = threading.Event()

    def calibrate_basepoint(self, wait=True) -> np.ndarray:
        """
        重置基地点到当前雷达位置 / cm
        """
        if wait and not self.radar.rt_pose_update_event.wait(1):
            logger.error("[NAVI] reset_basepoint(): Radar pose update timeout")
            raise RuntimeError("Radar pose update timeout")
        x, y, _ = self.radar.rt_pose
        self.basepoint = np.array([x, y])
        logger.info(f"[NAVI] Basepoint reset to {self.basepoint}")
        return self.basepoint

    def set_basepoint(self, point):
        """
        设置基地点(雷达坐标系) / cm
        """
        self.basepoint = np.asarray(point)
        logger.info(f"[NAVI] Basepoint set to {self.basepoint}")

    def set_navigation_state(self, state: bool):
        """
        设置导航状态
        """
        self.navigation_flag = state
        if state and self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            logger.debug("[NAVI] Auto set fc mode to HOLD_POS_MODE")

    def set_keep_height_state(self, state: bool):
        """
        设置定高状态
        """
        self.keep_height_flag = state

    def stop(self, join=False):
        """
        停止导航
        """
        self.running = False
        self.radar.stop_resolve_pose()
        if join:
            for thread in self._thread_list:
                thread.join()
        logger.info("[NAVI] Navigation stopped")

    def start(self, mode="fusion"):
        """
        启动导航
        mode: 导航模式, "radar"/"rs"/"fusion"/"fusion-ros"
        """
        if self.running:
            logger.warning("[NAVI] Navigation already running, restarting...")
            self.stop(join=True)
        self.running = True
        self.radar.subtask_skip = PARAMS.RADAR_SKIP
        self.rs.event_skip = PARAMS.RS_SKIP
        self._fusion_skip = PARAMS.FUSION_SKIP
        if self.mapper is not None:
            self.mapper.trans_event_skip = PARAMS.MAP_SKIP
        self._fusion_cnt = 0
        self._t265_trans_args = None
        self.switch_navigation_mode(mode)  # type: ignore
        self._realtime_control_data_in_xyzYaw = [0, 0, 0, 0]
        self.update_realtime_control(vel_x=0, vel_y=0, vel_z=0, yaw=0)
        logger.info("[NAVI] Realtime control started")
        self._thread_list.append(threading.Thread(target=self._keep_height_task, daemon=True))
        self._thread_list[-1].start()
        self._thread_list.append(threading.Thread(target=self._navigation_task, daemon=True))
        self._thread_list[-1].start()
        logger.info("[NAVI] Navigation started")

    def update_realtime_control(
        self,
        vel_x: Optional[int] = None,
        vel_y: Optional[int] = None,
        vel_z: Optional[int] = None,
        yaw: Optional[int] = None,
    ) -> None:
        """
        更新实时控制帧
        """
        if vel_x is not None:
            self._realtime_control_data_in_xyzYaw[0] = vel_x
        if vel_y is not None:
            self._realtime_control_data_in_xyzYaw[1] = vel_y
        if vel_z is not None:
            self._realtime_control_data_in_xyzYaw[2] = vel_z
        if yaw is not None:
            self._realtime_control_data_in_xyzYaw[3] = yaw
        self.fc.send_realtime_control_data(*self._realtime_control_data_in_xyzYaw)

    def switch_navigation_mode(self, mode: Literal["radar", "rs", "fusion", "fusion-ros"]):
        """
        切换导航模式
        radar: 仅雷达扫网定位
        rs: 仅T265定位
        fusion: 雷达扫网定位辅助T265定位
        fusion-ros: ROS建图辅助T265定位
        """
        assert mode in ("radar", "rs", "fusion", "fusion-ros"), "Invalid navigation mode"
        if mode == "radar" or mode == "fusion":
            assert self.radar.running, "Radar not running"
            self.radar.start_resolve_pose(
                size=PARAMS.MAP_SIZE,
                scale_ratio=PARAMS.SCALE_RATIO,
                low_pass_ratio=PARAMS.LOW_PASS_RATIO,
                polyline=PARAMS.POLYLINE,
            )
            logger.info("[NAVI] Radar resolve pose started")
        elif self.radar._rtpose_flag:
            self.radar.stop_resolve_pose()
            logger.info("[NAVI] Radar resolve pose stopped")
        if mode == "rs" or mode == "fusion" or mode == "fusion-ros":
            assert self.rs.running, "RealSense not running"
        if mode == "fusion-ros":
            assert self.mapper is not None, "Mapper not initialized"
        self._navigation_mode = mode
        logger.info(f"[NAVI] Navigation mode switched to {mode}")

    def _rs_speed_report_callback(self, pose: T265_Pose_Frame, _, __):
        vel_x = round(-pose.velocity.z * 100)
        vel_y = round(-pose.velocity.x * 100)
        vel_z = round(pose.velocity.x * 100)
        self.fc.send_general_speed(x=vel_x, y=vel_y, z=vel_z)
        # pos_x = round(-pose.position.z * 100)
        # pos_y = round(-pose.position.x * 100)
        # pos_z = round(pose.position.y * 100)
        # fc.send_general_position(x=pos_x, y=pos_y, z=pos_z)

    def set_rs_speed_report(self, state: bool, skip: int = 1):
        """
        设置RealSense速度上报状态
        skip: 速度上报间隔(freq = 200/skip)
        """
        if state:
            self.rs.register_callback(self._rs_speed_report_callback, skip)
        else:
            self.rs.unregister_callback(self._rs_speed_report_callback)

    def switch_pid(self, pid: Union[str, tuple]):
        """
        切换平面导航PID参数

        pid: str:在self.pid_tunings中的键值 / tuple:自定义PID参数
        """
        if isinstance(pid, str):
            tuning = self.pid_tunings.get(pid, self.pid_tunings["default"])
        else:
            tuning = pid  # type: ignore
        self.navi_x_pid.tunings = tuning
        self.navi_y_pid.tunings = tuning
        logger.debug(f"[NAVI] PID Tunings set to {pid}: {tuning}")

    def _keep_height_task(self):
        paused = False
        while self.running:
            try:
                if not self.keep_height_by_rs:
                    if not self.fc.state.update_event.wait(1):
                        logger.warning("[NAVI] FC state update timeout")
                        self.update_realtime_control(vel_z=0)
                        continue
                    self.fc.state.update_event.clear()
                    self.current_height = self.fc.state.alt_add.value
                    height = self.current_height
                else:
                    if not self.rs.update_event.wait(1):
                        logger.warning("[NAVI] RealSense height timeout")
                        self.update_realtime_control(vel_z=0)
                        continue
                    height = self.current_height_rs
                logger_dbg.debug(f"[NAVI] Current height: {height}")
                if not (
                    self.keep_height_flag
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
                    and self.fc.state.unlock.value
                ):
                    if not paused:
                        paused = True
                        self.height_pid.set_auto_mode(False)
                        self.update_realtime_control(vel_z=0)
                        logger.info("[NAVI] Keep height paused")
                    continue
                if paused:
                    paused = False
                    self.height_pid.set_auto_mode(True, last_output=0)
                    logger.info("[NAVI] Keep Height resumed")
                out_hei = round(self.height_pid(height))  # type: ignore
                self.update_realtime_control(vel_z=out_hei)
                logger_dbg.info(f"[NAVI] Height PID output: {out_hei}")
            except Exception as e:
                logger.exception("[NAVI] Keep height task error")
                self.update_realtime_control(vel_z=0)

    def _get_t265_pose(self, wait=True) -> Optional[Tuple[float, float, float, bool]]:
        if wait and not self.rs.update_event.wait(1):
            logger.warning("[NAVI] RealSense pose timeout")
            return None
        self.rs.update_event.clear()
        if self._t265_trans_args is None:
            current_x = -self.rs.pose.translation.z * 100
            current_y = -self.rs.pose.translation.x * 100
            self.current_height_rs = self.rs.pose.translation.y * 100
            current_yaw = -self.rs.eular_rotation[2]
        else:
            position, eular = self.rs.get_pose_in_secondary_frame(self._t265_trans_args, as_eular=True)
            current_x = -position[2] * 100  # type: ignore
            current_y = -position[0] * 100  # type: ignore
            self.current_height_rs = position[1] * 100  # type: ignore
            current_yaw = -eular[2]  # type: ignore
        available = self.rs.pose.tracker_confidence >= 2
        logger_dbg.debug(f"[NAVI] RealSense pose: {current_x}, {current_y}, {current_yaw}, {available}")
        return current_x, current_y, current_yaw, available  # type: ignore

    def _get_radar_pose(self, wait=True) -> Optional[Tuple[float, float, float, bool]]:
        if wait and not self.radar.rt_pose_update_event.wait(1):
            logger.warning("[NAVI] Radar pose timeout")
            return None
        self.radar.rt_pose_update_event.clear()
        current_x, current_y, current_yaw = self.radar.rt_pose
        current_x -= self.basepoint[0]
        current_y -= self.basepoint[1]
        logger_dbg.debug(f"[NAVI] Radar pose: {current_x}, {current_y}, {current_yaw}")
        return current_x, current_y, current_yaw, current_x + current_y != 0

    def _get_fusion_pose(self) -> Optional[Tuple[float, float, float, bool]]:
        if self.radar.rt_pose_update_event.is_set():
            self.radar.rt_pose_update_event.clear()
            self._fusion_cnt += 1
        if self._fusion_cnt >= self._fusion_skip:
            self._fusion_cnt = 0
            self.calibrate_realsense(wait=False)
        return self._get_t265_pose()

    def _get_fusion_ros_pose(self) -> Optional[Tuple[float, float, float, bool]]:
        if self.mapper.trans_update_event.is_set():  # type: ignore
            self.mapper.trans_update_event.clear()  # type: ignore
            self.calibrate_realsense_ros(wait=False)
        ret = self._get_t265_pose()
        if not ret:
            return None
        x, y, yaw, avai = ret
        return x, y, yaw, avai and self.mapper._trans_node.transform_established  # type: ignore

    def _navigation_task(self):
        paused = False
        while self.running:
            try:
                if self._navigation_mode == "radar":
                    pose = self._get_radar_pose()
                elif self._navigation_mode == "rs":
                    pose = self._get_t265_pose()
                elif self._navigation_mode == "fusion":
                    pose = self._get_fusion_pose()
                elif self._navigation_mode == "fusion-ros":
                    pose = self._get_fusion_ros_pose()
                else:
                    raise ValueError(f"Unknown navigation mode: {self._navigation_mode}")
                if pose is None:
                    self.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.warning("[NAVI] Navigation pose not available")
                    continue
                self.current_x, self.current_y, self.current_yaw, available = (
                    float(pose[0]),
                    float(pose[1]),
                    float(pose[2]),
                    bool(pose[3]),
                )
                logger_dbg.debug(f"[NAVI] Pose: {self.current_x}, {self.current_y}, {self.current_yaw}")
                if not (
                    self.navigation_flag
                    and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
                    and self.fc.state.unlock.value
                ):  # 导航需在解锁/定点模式下运行
                    if not paused:
                        paused = True
                        self.navi_x_pid.set_auto_mode(False)
                        self.navi_y_pid.set_auto_mode(False)
                        self.yaw_pid.set_auto_mode(False)
                        self.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                        logger.info("[NAVI] Navigation paused")
                    continue
                if paused:
                    paused = False
                    self.navi_x_pid.set_auto_mode(True, last_output=0)
                    self.navi_y_pid.set_auto_mode(True, last_output=0)
                    self.yaw_pid.set_auto_mode(True, last_output=0)
                    logger.info("[NAVI] Navigation resumed")
                if not available:
                    logger.warning("[NAVI] Pose not available")
                    time.sleep(0.1)
                    continue
                # self.fc.send_general_position(x=self.current_x, y=self.current_y)
                out_x = round(self.navi_x_pid(self.current_x))  # type: ignore
                if out_x is not None:
                    self.update_realtime_control(vel_x=out_x)
                out_y = round(self.navi_y_pid(self.current_y))  # type: ignore
                if out_y is not None:
                    self.update_realtime_control(vel_y=out_y)
                out_yaw = round(self.yaw_pid(self.current_yaw))  # type: ignore
                if out_yaw is not None:
                    self.update_realtime_control(yaw=out_yaw)
                logger_dbg.info(f"[NAVI] Pose PID output: {out_x}, {out_y}, {out_yaw}")
            except Exception as e:
                logger.exception(f"[NAVI] Navigation task error")
                self.update_realtime_control(vel_x=0, vel_y=0, yaw=0)

    def calibrate_realsense(self, wait=True):
        """
        根据雷达扫网定位数据校准T265的坐标系
        """
        if wait and not self.radar.rt_pose_update_event.wait(1):
            raise RuntimeError("Radar pose update timeout")
        x, y, yaw = self.radar.rt_pose
        dx = x - self.basepoint[0]  # -> t265 -z * 100
        dx = -dx / 100.0
        dy = y - self.basepoint[1]  # -> t265 -x * 100
        dy = -dy / 100.0
        dyaw = -yaw
        if not self.keep_height_by_rs:
            dz = self.fc.state.alt_add.value / 100.0
        else:
            dz = self.current_height_rs / 100.0
        logger_dbg.info(f"[NAVI] Calibrate T265: radar={self.radar.rt_pose} dz={dx}, dx={dy}, dy={dz}, dyaw={dyaw}")
        self._t265_trans_args = self.rs.establish_secondary_origin(
            force_level=True, z_offset=dx, x_offset=dy, yaw_offset=dyaw, y_offset=dz
        )

    def calibrate_realsense_ros(self, wait=True):
        """
        根据ROS建图数据校准T265的坐标系
        """
        if wait and not self.mapper.trans_update_event.wait(1):  # type: ignore
            raise RuntimeError("Mapper transform update timeout")
        x, y, _ = self.mapper.position  # type: ignore
        _, _, yaw = self.mapper.eular_rotation  # type: ignore
        dx = -x
        dy = -y
        dyaw = yaw
        if not self.keep_height_by_rs:
            dz = self.fc.state.alt_add.value / 100.0
        else:
            dz = self.current_height_rs / 100.0
        logger_dbg.info(f"[NAVI] Calibrate T265: map={self.mapper.position} dz={dx}, dx={dy}, dy={dz}, dyaw={dyaw}")  # type: ignore
        self._t265_trans_args = self.rs.establish_secondary_origin(
            force_level=True, z_offset=dx, x_offset=dy, yaw_offset=dyaw, y_offset=dz
        )

    def direct_set_waypoint(self, waypoint):
        """
        直接设置水平导航PID目标点 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        self.navi_x_pid.setpoint = waypoint[0]
        self.navi_y_pid.setpoint = waypoint[1]
        if len(waypoint) > 2:
            self.height_pid.setpoint = waypoint[2]

    def navigation_to_waypoint(self, waypoint, wait=True, dt: float = 0.1):
        """
        创建直线航线并导航到指定的目标点

        waypoint: (x, y, [z]) 相对于基地点的坐标 / cm / 匿名(ROS)坐标系 / 基地原点
        wait: 是否阻塞直到到达目标点
        dt: 轨迹精度 / s
        """
        logger.debug(f"[NAVI] Navigation to waypoint: {waypoint}")
        if len(waypoint) == 2:
            waypoint = [waypoint[0], waypoint[1], self.height_pid.setpoint]
        else:
            waypoint = [waypoint[0], waypoint[1], waypoint[2]]
        waypoint_cur = [self.current_x, self.current_y, self.current_height]
        length = np.linalg.norm(np.array(waypoint) - np.array(waypoint_cur))
        tT = float(length / self.navi_speed)
        traj = TrajectoryGenerator(start_pos=waypoint_cur, des_pos=waypoint, T=tT)
        traj.solve()
        traj_list = []
        for t in np.arange(0, tT, dt):
            traj_list.append(traj.calc_position_xyz(t))
        traj_list.append(waypoint)
        self.navigation_follow_trajectory(traj_list, wait=wait)  # type: ignore

    def _trajectory_task(self, traj_list: Union[List[Tuple[float, ...]], np.ndarray]):
        logger.debug("[NAVI] Trajectory task started")
        self.traj_running_event.set()
        for point in traj_list:
            while not self._reached_waypoint(30):
                time.sleep(0.02)
                if not self.traj_running_event.is_set():
                    self.traj_running_event.set()
                    logger.debug("[NAVI] Trajectory task forced to stop")
                    return
                if not (self.running and self.navigation_flag):
                    logger.debug("[NAVI] Trajectory task forced to stop")
                    return
            x, y = float(point[0]), float(point[1])
            self.navi_x_pid.setpoint = x
            self.navi_y_pid.setpoint = y
            if len(point) > 2:
                self.height_pid.setpoint = float(point[2])
            # logger.debug(f"[NAVI] Trajectory task: {x}, {y}")
        self.traj_running_event.clear()
        logger.debug("[NAVI] Trajectory task finished")

    def navigation_follow_trajectory(self, traj_list: Union[List[Tuple[float, ...]], np.ndarray], wait=True):
        """
        跟随轨迹导航

        traj_list: 轨迹点列表 / cm / 匿名(ROS)坐标系 / 基地原点
        wait: 是否阻塞直到到达目标点
        """
        logger.debug(f"[NAVI] Running on trajectory with {len(traj_list)} points")
        self.navi_x_pid.tunings = self.pid_tunings["navi"]
        self.navi_y_pid.tunings = self.pid_tunings["navi"]
        self.navi_x_pid.output_limits = (-self.navi_speed, self.navi_speed)
        self.navi_y_pid.output_limits = (-self.navi_speed, self.navi_speed)
        if wait:
            self._trajectory_task(traj_list)
            self.wait_for_waypoint()
        else:
            t = threading.Thread(target=self._trajectory_task, args=(traj_list,), daemon=True)
            t.start()
            self._thread_list.append(t)
            self.traj_running_event.wait()

    @property
    def navigation_target(self) -> np.ndarray:
        """
        当前导航目标点 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        return np.array([self.navi_x_pid.setpoint, self.navi_y_pid.setpoint])

    @navigation_target.setter
    def navigation_target(self, waypoint: np.ndarray):
        return self.navigation_to_waypoint(waypoint)

    @property
    def current_point(self) -> np.ndarray:
        """
        当前位置 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        return np.array([self.current_x, self.current_y])

    def navigation_stop_here(self) -> np.ndarray:
        """
        原地停止(设置目标点为当前位置)

        return: 原定目标点 / cm / 匿名(ROS)坐标系 / 基地原点
        """
        waypoint = self.navigation_target
        x, y = self.current_x, self.current_y
        if self.traj_running_event.is_set():
            self.traj_running_event.clear()
            self.traj_running_event.wait(0.1)  # 等待轨迹任务停止
            self.traj_running_event.clear()
        self.navi_x_pid.setpoint = x
        self.navi_y_pid.setpoint = y
        self._waypoint_param_switch()
        logger.debug(f"[NAVI] Navigation stopped at {x}, {y}")
        return waypoint

    def set_height(self, height: float):
        """
        设置飞行高度

        height: 激光高度 / cm
        """
        self.height_pid.setpoint = height
        logger.debug(f"[NAVI] Keep height set to {height}")

    def set_yaw(self, yaw: float):
        """
        设置飞行航向

        yaw: 相对于初始状态的航向角 / deg
        """
        self.yaw_pid.setpoint = yaw
        logger.debug(f"[NAVI] Keep yaw set to {yaw}")

    def navigation_to_waypoint_relative(self, waypoint_rel, *args, **kwargs):
        """
        导航到指定的目标点

        waypoint_rel: (x, y) 坐标 / cm / 匿名(ROS)坐标系 / 当前位置原点
        其余参数参考navigation_to_waypoint
        """
        self.navigation_to_waypoint(self.current_point + np.asarray(waypoint_rel), *args, **kwargs)

    def set_navigation_speed(self, speed):
        """
        设置导航速度

        speed: 速度 / cm/s
        """
        speed = abs(speed)
        self.navi_x_pid.output_limits = (-speed, speed)
        self.navi_y_pid.output_limits = (-speed, speed)
        self.navi_speed = speed
        logger.info(f"[NAVI] Navigation speed set to {speed}")

    def set_vertical_speed(self, speed):
        """
        设置垂直速度

        speed: 速度 / cm/s
        """
        speed = abs(speed)
        self.height_pid.output_limits = (-speed, speed)
        logger.info(f"[NAVI] Vertical speed set to {speed}")

    def set_yaw_speed(self, speed):
        """
        设置偏航速度

        speed: 速度 / deg/s
        """
        speed = abs(speed)
        self.yaw_pid.output_limits = (-speed, speed)
        logger.info(f"[NAVI] Yaw speed set to {speed}")

    def _reached_waypoint(self, pos_thres):
        return (
            abs(self.current_x - self.navi_x_pid.setpoint) < pos_thres
            and abs(self.current_y - self.navi_y_pid.setpoint) < pos_thres
        )

    def pointing_takeoff(self, point, target_height=140):
        """
        定点起飞

        point: (x, y) 坐标 / cm / 匿名(ROS)坐标系 / 基地原点
        target_height: 起飞高度 / cm
        """
        logger.info(f"[NAVI] Takeoff at {point}")
        self.navigation_flag = False
        self.keep_height_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        self.fc.unlock()
        inital_yaw = self.fc.state.yaw.value
        time.sleep(2)  # 等待电机启动
        self.fc.take_off(80)
        self.fc.wait_for_takeoff_done(timeout_s=8)
        self.fc.set_yaw(inital_yaw, 25)
        self.fc.wait_for_hovering(2)
        ######## 闭环定高
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        self.set_height(target_height)
        self.keep_height_flag = True
        self.wait_for_height()
        self.direct_set_waypoint(point)  # 初始化路径点
        self.switch_pid("hover")
        time.sleep(0.1)
        self.navigation_flag = True

    def pointing_landing(self, point):
        """
        定点降落

        point: (x, y) / cm / 匿名(ROS)坐标系 / 基地原点
        """
        logger.info(f"[NAVI] Landing at {point}")
        self.navigation_flag = True
        self.keep_height_flag = True
        self.navigation_to_waypoint(point)
        self.switch_pid("land")
        time.sleep(0.5)
        self.set_height(60)
        self.wait_for_height()
        self.set_height(30)
        time.sleep(1.5)
        self.set_height(20)
        time.sleep(2)
        self.wait_for_waypoint()
        self.set_height(0)
        # self.fc.land()
        time.sleep(2)
        # self.fc.wait_for_lock(5)
        self.fc.lock()
        self.navigation_flag = False
        self.keep_height_flag = False

    def _waypoint_param_switch(self):
        tuning = self.pid_tunings["hover"]
        self.navi_x_pid.tunings = tuning
        self.navi_y_pid.tunings = tuning
        self.navi_x_pid.output_limits = (-self.navi_speed, self.navi_speed)
        self.navi_y_pid.output_limits = (-self.navi_speed, self.navi_speed)
        logger.debug("[NAVI] Waypoint param switched")

    def wait_for_waypoint(self, time_thres=2, pos_thres=20, timeout=15):
        """
        等待到达目标点

        time_thres: 到达目标点后积累的时间/s
        pos_thres: 到达目标点的距离阈值/cm
        timeout: 超时时间/s
        """
        time_count = 0
        time_start = time.perf_counter()
        param_switched = False
        while True:
            time.sleep(0.05)
            if self._reached_waypoint(pos_thres):
                time_count += 0.05
                if not param_switched:
                    self._waypoint_param_switch()
                    param_switched = True
            if time_count >= time_thres:
                logger.info("[NAVI] Reached waypoint")
                return
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Waypoint overtime")
                return

    def wait_for_height(self, time_thres=0.5, height_thres=8, timeout=10):
        """
        等待到达目标高度(定高设定值)

        time_thres: 到达目标高度后积累的时间/s
        pos_thres: 到达目标高度的阈值/cm
        timeout: 超时时间/s
        """
        time_start = time.perf_counter()
        time_count = 0
        while True:
            time.sleep(0.05)
            if abs(self.current_height - self.height_pid.setpoint) < height_thres:
                time_count += 0.05
            if time_count >= time_thres:
                logger.info("[NAVI] Reached height")
                return
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Height overtime")
                return

    def wait_for_yaw(self, time_thres=0.5, yaw_thres=5, timeout=10):
        """
        等待到达目标偏航角

        time_thres: 到达目标偏航角后积累的时间/s
        pos_thres: 到达目标偏航角的阈值/deg
        timeout: 超时时间/s
        """
        time_start = time.perf_counter()
        time_count = 0
        remap = lambda x: x if x < 180 else 360 - x
        while True:
            time.sleep(0.05)
            if remap(abs(self.current_yaw - self.yaw_pid.setpoint)) < yaw_thres:
                time_count += 0.05
            if time_count >= time_thres:
                logger.info("[NAVI] Reached yaw")
                return
            if time.perf_counter() - time_start > timeout:
                logger.warning("[NAVI] Yaw overtime")
                return
