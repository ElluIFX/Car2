import threading
import time
from typing import Callable, Generator, List, Literal, Optional, Union

import cv2
import numpy as np
import serial
from FlightController import FC_Client, FC_Controller, FC_Like, FC_Server
from FlightController.Solutions.Radar_SLAM import ICPM, radar_resolve_rt_pose
from loguru import logger
from serial.tools.list_ports import comports

from .LDRadar_Resolver import (
    Map_Circle,
    Point_2D,
    Radar_Package,
    Radar_Package_Multi,
    resolve_radar_data,
    resolve_radar_data_multi,
)


def get_radar_com() -> Optional[str]:
    VID_PID = "10C4:EA60"
    for port, desc, hwid in sorted(comports()):
        if VID_PID in hwid:
            logger.info(f"[RADAR] Found radar hwid on port {port}")
            return port
    return None


class LD_Radar(object):
    """
    乐动激光雷达驱动
    """

    def __init__(self):
        self.map = Map_Circle()
        self.running = False
        self.connected = False
        self._thread_list = []
        self._package = Radar_Package()
        self._package_multi = Radar_Package_Multi()
        self._serial = None
        self._update_callback = None
        self.debug = False
        self.subtask_event = threading.Event()
        self.subtask_skip = 4
        self._data_buf = b""
        self._count = 0
        # 位姿估计
        self.rt_pose_update_event = threading.Event()
        self.rt_pose = [0, 0, 0]
        self._rtpose_flag = False
        self._rt_pose_inited = [False, False, False]
        # 解析函数
        self._map_funcs = []
        self.map_func_update_times = []
        self.map_func_results = []

    def start(
        self,
        com: Union[str, None, FC_Like, Literal["ros"]] = None,
        radar_type: str = "LD06",
        subtask_skip=4,
    ):
        """
        开始监听雷达数据
        com: 监听串口 / 飞控转发 / ROS订阅
        radar_type: LD08 or LD06
        subtask_skip: 多少次雷达数据更新后, 进行一次子任务
        """
        if self.running:
            self.stop()
        self.running = True
        self.subtask_event.clear()
        self.subtask_skip = subtask_skip
        self._ros_node = None
        if com == "ros":
            from .RosNode import RadarListenNode

            # 解耦:按需导入ROS相关的模块,防止非ROS环境下无法运行
            self._ros_node = RadarListenNode(self._ros_callback)
        elif isinstance(com, (FC_Client, FC_Controller, FC_Server)):
            com.register_radar_callback(self._fc_callback)
            logger.info("[RADAR] Registered radar on FC")
        elif isinstance(com, str) or com is None:
            if com is None:
                com = get_radar_com()
            if radar_type == "LD08":
                baudrate = 115200
            elif radar_type == "LD06":
                baudrate = 230400
            else:
                raise ValueError("Unknown radar type")
            self._serial = serial.Serial(com, baudrate=baudrate)
            thread = threading.Thread(target=self._read_serial_task)
            thread.daemon = True
            thread.start()
            self._thread_list.append(thread)
            logger.info("[RADAR] Listenning thread started")
        else:
            raise ValueError("Unknown com type")
        thread = threading.Thread(target=self._map_resolve_task)
        thread.daemon = True
        thread.start()
        self._thread_list.append(thread)
        logger.info("[RADAR] Map resolve thread started")
        self.start_time = time.perf_counter()

    def register_update_callback(self, callback: Callable[[Union[Radar_Package, Radar_Package_Multi]], None]):
        """
        注册雷达数据更新回调函数
        """
        self._update_callback = callback

    def _fc_callback(self, buf: bytes):
        if not self.running:
            return
        self.connected = True
        if resolve_radar_data_multi(buf, self._package_multi):
            self.map.update(self._package_multi)
            if self._update_callback is not None:
                self._update_callback(self._package_multi)
            self._count += 1
            if self._count >= self.subtask_skip:
                self._count = 0
                self.subtask_event.set()

    def _ros_callback(self, pack: Radar_Package):
        if not self.running:
            return
        self.connected = True
        self.map.update(pack)
        if self._update_callback is not None:
            self._update_callback(pack)
        self._count += 1
        if self._count >= self.subtask_skip:
            self._count = 0
            self.subtask_event.set()

    def stop(self, joined=True):
        """
        停止监听雷达数据
        """
        self.running = False
        if self._ros_node is not None:
            self._ros_node.stop()
        if joined:
            for thread in self._thread_list:
                thread.join()
        if self._serial is not None:
            self._serial.close()
        logger.info("[RADAR] Stopped all threads")

    def _read_serial_task(self):
        reading_flag = False
        start_bit = b"\x54\x2C"
        package_length = 45
        read_buffer = bytes()
        wait_buffer = bytes()
        while self.running:
            try:
                if self._serial.in_waiting > 0:
                    if not reading_flag:  # 等待包头
                        wait_buffer += self._serial.read(1)
                        if len(wait_buffer) >= 2:
                            if wait_buffer[-2:] == start_bit:
                                reading_flag = True
                                wait_buffer = bytes()
                                read_buffer = start_bit
                    else:  # 读取数据
                        read_buffer += self._serial.read(package_length)
                        reading_flag = False
                        self.connected = True
                        if resolve_radar_data(read_buffer, self._package):
                            self.map.update(self._package)
                            if self._update_callback is not None:
                                self._update_callback(self._package)
                            self._count += 1
                            if self._count >= self.subtask_skip:
                                self._count = 0
                                self.subtask_event.set()
                else:
                    time.sleep(0.001)
            except Exception as e:
                logger.exception(f"[RADAR] Listenning thread error")
                time.sleep(0.5)

    def _map_resolve_task(self):
        while self.running:
            try:
                if self.subtask_event.wait(1):
                    self.subtask_event.clear()
                    if self._rtpose_flag:
                        if self._rtpose_rotation_adapt:
                            rot_angle = self.rt_pose[2]
                        else:
                            rot_angle = 0
                        if self._rtpose_use_icpm:
                            pts = self.map.output_points(self._rtpose_scale_ratio, False, rot_angle=rot_angle)
                            self._icpm.match(
                                pts, debug=self.debug, debug_save_img=self.debug, debug_size=self._rtpose_size
                            )
                            x, y = self._icpm.translation
                        else:
                            yaw = self._icpm.rotation_as_euler
                            if self._rtpose_polyline:
                                img = self.map.output_polyline_cloud(
                                    size=int(self._rtpose_size),
                                    scale=0.1 * self._rtpose_scale_ratio,
                                    thickness=1,
                                    draw_outside=False,
                                    rot_angle=rot_angle,
                                )
                            else:
                                img = self.map.output_cloud(
                                    size=int(self._rtpose_size),
                                    scale=0.1 * self._rtpose_scale_ratio,
                                    rot_angle=rot_angle,
                                )
                            x, y, yaw = radar_resolve_rt_pose(
                                img,
                                skip_er=self._rtpose_polyline,
                                skip_di=self._rtpose_polyline,
                                debug=self.debug,
                                debug_save_img=self.debug,
                            )
                        if x is not None:
                            if self._rt_pose_inited[0]:
                                self.rt_pose[0] += (
                                    x / self._rtpose_scale_ratio - self.rt_pose[0]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[0] = x / self._rtpose_scale_ratio
                                self._rt_pose_inited[0] = True
                        if y is not None:
                            if self._rt_pose_inited[1]:
                                self.rt_pose[1] += (
                                    y / self._rtpose_scale_ratio - self.rt_pose[1]
                                ) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[1] = y / self._rtpose_scale_ratio
                                self._rt_pose_inited[1] = True
                        if yaw is not None:
                            if self._rtpose_rotation_adapt:
                                yaw += rot_angle
                            if self._rt_pose_inited[2]:
                                self.rt_pose[2] += (yaw - self.rt_pose[2]) * self._rtpose_low_pass_ratio
                            else:
                                self.rt_pose[2] = yaw
                                self._rt_pose_inited[2] = True
                        self.rt_pose_update_event.set()
                    for i in range(len(self._map_funcs)):
                        if self._map_funcs[i]:
                            func, args, kwargs = self._map_funcs[i]
                            result = func(*args, **kwargs)
                            if result:
                                self.map_func_results[i] = result
                                self.map_func_update_times[i] = time.perf_counter()
                else:
                    if self._rtpose_flag or len(self._map_funcs) > 0:
                        logger.warning("[RADAR] Map resolve thread wait timeout")
            except Exception as e:
                logger.exception(f"[RADAR] Map resolve thread error")
                time.sleep(0.5)

    def _init_radar_map(self):
        self._radar_map_img = np.zeros((600, 600, 3), dtype=np.uint8)
        a = np.sqrt(2) * 600
        b = (a - 600) / 2
        c = a - b
        b = int(b / np.sqrt(2))
        c = int(c / np.sqrt(2))
        cv2.line(self._radar_map_img, (b, b), (c, c), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (c, b), (b, c), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (300, 0), (300, 600), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (0, 300), (600, 300), (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 100, (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 200, (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 300, (255, 0, 0), 1)
        self._radar_map_img_scale = 1
        self._radar_map_info_angle = 0

    def _radar_map_on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:
                self._radar_map_img_scale *= 1.1
            else:
                self._radar_map_img_scale *= 0.9
        elif event == cv2.EVENT_LBUTTONDOWN or (event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON):
            self._radar_map_info_angle = (90 - np.arctan2(300 - y, x - 300) * 180 / np.pi) % 360

    def _radar_map_generator(self) -> Generator[np.ndarray, None, None]:
        self._init_radar_map()
        while True:
            img_ = self._radar_map_img.copy()
            cv2.putText(
                img_,
                f"{100/self._radar_map_img_scale:.0f}",
                (300, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img_,
                f"{200/self._radar_map_img_scale:.0f}",
                (300, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img_,
                f"{300/self._radar_map_img_scale:.0f}",
                (300, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            add_p: List[Point_2D] = []
            if len(self.map_func_results) > 0:
                for result in self.map_func_results:
                    add_p.extend(result)
                for i, p in enumerate(add_p):
                    cv2.putText(
                        img_,
                        f"AP-{i}: {p}",
                        (10, 520 - i * 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0),
                    )
            cv2.putText(
                img_,
                f"Angle: {self._radar_map_info_angle:.1f} (idx={round((self._radar_map_info_angle % 360) * self.map.ACC)})",
                (10, 540),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
            )
            cv2.putText(
                img_,
                f"Distance: {self.map.get_distance(self._radar_map_info_angle)}",
                (10, 560),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 0),
            )
            point = self.map.get_point(self._radar_map_info_angle)
            if point:
                xy = point.to_xy()
                cv2.putText(
                    img_,
                    f"Position: ( {xy[0]:.2f} , {xy[1]:.2f} )",
                    (10, 580),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                add_p.append(point)
                pos = point.to_cv_xy() * self._radar_map_img_scale + np.array([300, 300])
                cv2.line(img_, (300, 300), (int(pos[0]), int(pos[1])), (255, 255, 0), 1)

            self.map.draw_on_cv_image(img_, scale=self._radar_map_img_scale, add_points=add_p)
            cv2.putText(
                img_,
                f"RPM={self.map.rotation_spd:05.2f} PPS={self.map.update_count/(time.perf_counter()-self.start_time):05.2f}",
                (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
            )
            cv2.putText(
                img_,
                f"AVAIL={self.map.avail_points}/{self.map.total_points} CNT={self.map.update_count} ",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 255),
            )
            yield img_

    def show_radar_map(self, generator: Optional[Generator[np.ndarray, None, None]] = None):
        """
        显示雷达地图(调试用, 高占用且阻塞)
        """
        cv2.namedWindow("Radar Map", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(  # type: ignore
            "Radar Map", lambda *args, **kwargs: self._radar_map_on_mouse(*args, **kwargs)
        )
        if generator is None:
            generator = self._radar_map_generator()
        for img_ in generator:
            cv2.imshow("Radar Map", img_)
            key = cv2.waitKey(int(1000 / 50))
            if key == ord("q"):
                cv2.destroyWindow("Radar Map")
                break
            elif key == ord("w"):
                self._radar_map_img_scale *= 1.1
            elif key == ord("s"):
                self._radar_map_img_scale *= 0.9
            elif key == ord("a"):
                t0 = time.perf_counter()
                out = self.map.output_polyline_cloud(scale=self._radar_map_img_scale, size=800, draw_outside=False)
                t1 = time.perf_counter()
                print(f"output_polyline_cloud: {t1 - t0:.9f}s")
                cv2.imshow("Cloud(polyline)", out)
                t0 = time.perf_counter()
                out = self.map.output_cloud(scale=self._radar_map_img_scale, size=800)
                t1 = time.perf_counter()
                print(f"output_cloud: {t1 - t0:.9f}s")
                cv2.imshow("Cloud", out)

    def register_map_func(self, func, *args, **kwargs) -> int:
        """
        注册雷达地图解析函数

        func应为self.map包含的方法,所有附加参数将会传递给该func
        从列表map_func_results中获取结果,
        列表map_func_update_times储存了上一次该函数返回非空结果的时间,用于超时判断

        return: func_id
        """
        self._map_funcs.append((func, args, kwargs))
        self.map_func_results.append([])
        self.map_func_update_times.append(0)
        return len(self._map_funcs) - 1

    def unregister_map_func(self, func_id: int):
        """
        注销雷达地图解析函数
        """
        self._map_funcs[func_id] = None
        self.map_func_results[func_id] = []
        self.map_func_update_times[func_id] = 0

    def update_map_func_args(self, func_id: int, *args, **kwargs):
        """
        更新雷达地图解析函数参数
        """
        self._map_funcs[func_id][1] = args
        self._map_funcs[func_id][2] = kwargs

    def start_resolve_pose(
        self,
        size: int = 1000,
        scale_ratio: float = 1,
        low_pass_ratio: float = 0.5,
        polyline: bool = False,
        rotation_adapt: bool = False,
        use_icpm: bool = False,
    ):
        """
        开始使用点云图解算位姿
        size: 解算范围(长宽为size的正方形)
        scale_ratio: 降采样比例, 降低精度节省计算资源
        low_pass_ratio: 低通滤波比例
        polyline: 是否使用多边形点云
        rotation_adapt: 是否使用旋转补偿
        use_icpm: 是否使用ICPM算法
        """
        self._rtpose_flag = True
        self._rtpose_size = size
        self._rtpose_scale_ratio = scale_ratio
        self._rtpose_low_pass_ratio = low_pass_ratio
        self._rtpose_polyline = polyline
        self._rtpose_rotation_adapt = rotation_adapt
        self._rtpose_use_icpm = use_icpm
        if use_icpm:
            pts = self.map.output_points(scale_ratio, False)
            self._icpm = ICPM(pts)
        self.rt_pose = [0, 0, 0]
        self._rt_pose_inited = [False, False, False]

    def stop_resolve_pose(self):
        """
        停止使用点云图解算位姿
        """
        self._rtpose_flag = False
        self.rt_pose = [0, 0, 0]
        self._rt_pose_inited = [False, False, False]
        self.rt_pose_update_event.clear()

    def update_icpm_template(self):
        """
        更新ICPM算法模板点云
        """
        assert self._rtpose_use_icpm, "ICPM is not enabled"
        pts = self.map.output_points(self._rtpose_scale_ratio, False)
        self._icpm.update_template(pts)
