import atexit
import math
import multiprocessing
import threading
import time
from typing import Any, Callable, List, Optional, Union
from xmlrpc.client import Boolean

import numpy as np
import rclpy
from cartographer_ros_msgs.msg import SensorTopics, TrajectoryOptions
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory, WriteState
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped, Twist
from loguru import logger
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2 as pc2
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .LDRadar_Resolver import Point_2D, Radar_Package, Radar_Package_Multi

if not rclpy.ok():
    rclpy.init()
    logger.debug("[ROS] rclpy.init()")

_nodes_to_run: List[Node] = []


class RadarListenNode(Node):  # listen to the radar data
    def __init__(self, callback: Callable[[Radar_Package], None]):
        super().__init__("PyRadarListenNode")
        self.radar_sub = self.create_subscription(LaserScan, "/scan", self.radar_callback, 10)
        self.callback = callback
        _nodes_to_run.append(self)
        logger.info("[ROS] RadarListenNode ready to start")

    def radar_callback(self, radar_data: LaserScan):
        try:
            pack = Radar_Package()
            pack.start_degree = radar_data.angle_min / 2 / math.pi * 360
            pack.stop_degree = radar_data.angle_max / 2 / math.pi * 360
            pack.angle_increment = radar_data.angle_increment / 2 / math.pi * 360
            pack.distances = []
            for d in radar_data.ranges:
                try:
                    pack.distances.append(round(d * 1000))
                except:
                    pack.distances.append(0)
            pack.confidences = []
            for d in radar_data.intensities:
                try:
                    pack.confidences.append(round(d))
                except:
                    pack.confidences.append(0)
            pack.points = []
            for i in range(len(radar_data.ranges)):
                pt = Point_2D((pack.start_degree + pack.angle_increment * i), pack.distances[i]).to_xy()
                new_pt = Point_2D(confidence=pack.confidences[i])
                new_pt.from_xy(np.array([pt[0], -pt[1]]))
                pack.points.append(new_pt)
            self.callback(pack)
        except Exception as e:
            logger.exception("[ROS] RadarListenNode error")


class T265ListenNode(Node):  # listen to the T265 data
    def __init__(self, callback: Callable[[Odometry], None]):
        super().__init__("PyRsListenNode")
        self.t265_sub = self.create_subscription(
            Odometry, "/camera/pose/sample", self.t265_callback, qos_profile_sensor_data
        )
        self.callback = callback
        _nodes_to_run.append(self)
        logger.info("[ROS] T265ListenNode ready to start")

    def t265_callback(self, t265_data: Odometry):
        self.callback(t265_data)


class MapListenNode(Node):  # listen to the map data
    def __init__(self, callback: Callable[[OccupancyGrid], None]):
        super().__init__("PyMapListenNode")
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.callback = callback
        _nodes_to_run.append(self)
        logger.info("[ROS] MapListenNode ready to start")

    def map_callback(self, map_data: OccupancyGrid):
        self.callback(map_data)


class MapClientNode(Node):  # communicate with Cartographer
    def __init__(self):
        super().__init__("PyMapClientNode")
        self.start_trajectory_client = self.create_client(StartTrajectory, "/start_trajectory")
        self.finish_trajectory_client = self.create_client(FinishTrajectory, "/finish_trajectory")
        self.write_state_client = self.create_client(WriteState, "/write_state")
        self._start_trajectory_srv = StartTrajectory.Request()
        self._finish_trajectory_srv = FinishTrajectory.Request()
        self._write_state_srv = WriteState.Request()
        _nodes_to_run.append(self)
        logger.info("[ROS] MapClientNode ready to start")

    def start_trajectory(self, options: Optional[TrajectoryOptions] = None, topics: Optional[SensorTopics] = None):
        if options:
            self._start_trajectory_srv.options = options
        if topics:
            self._start_trajectory_srv.topics = topics
        while not self.start_trajectory_client.wait_for_service(timeout_sec=1.0):
            logger.debug("[ROS] Waiting for service /start_trajectory to appear...")
        logger.info("[ROS] Cartographer: Start trajectory")
        self.future = self.start_trajectory_client.call_async(self._start_trajectory_srv)
        while self.future.done() is False:
            time.sleep(0.1)
        ret = self.future.result()
        logger.info("[ROS] Cartographer: Start trajectory done")
        return ret

    def finish_trajectory(self, trajectory_id: int):
        self._finish_trajectory_srv.trajectory_id = trajectory_id
        while not self.finish_trajectory_client.wait_for_service(timeout_sec=1.0):
            logger.debug("[ROS] Waiting for service /finish_trajectory to appear...")
        logger.info("[ROS] Cartographer: Finish trajectory")
        self.future = self.finish_trajectory_client.call_async(self._finish_trajectory_srv)
        while self.future.done() is False:
            time.sleep(0.1)
        ret = self.future.result()
        logger.info("[ROS] Cartographer: Finish trajectory done")
        return ret

    def write_state(self, filename: str, include_unfinished_submaps: Boolean):
        self._write_state_srv.filename = filename
        self._write_state_srv.include_unfinished_submaps = include_unfinished_submaps
        while not self.write_state_client.wait_for_service(timeout_sec=1.0):
            logger.debug("[ROS] Waiting for service /write_state to appear...")
        logger.info("[ROS] Cartographer: Write state")
        self.future = self.write_state_client.call_async(self._write_state_srv)
        while self.future.done() is False:
            time.sleep(0.1)
        ret = self.future.result()
        logger.info("[ROS] Cartographer: Write state done")
        return ret


class PoseListenNode(Node):  # listen to the pose data
    def __init__(self, callback: Callable[[PoseStamped], None]):
        super().__init__("PyPoseListenNode")
        self.pose_sub = self.create_subscription(PoseStamped, "/tracked_pose", self.pose_callback, 10)
        self.callback = callback
        _nodes_to_run.append(self)
        logger.info("[ROS] PoseListenNode ready to start")

    def pose_callback(self, pose_data: PoseStamped):
        self.callback(pose_data)


class Tf2ListenNode(Node):  # listen to the pose data
    def __init__(
        self, callback: Callable[[Transform], None], source_frame: str, target_frame: str, frequency: float = 20
    ):
        self.node_name = f"PyTf2ListenNode_{source_frame.replace('/', '')}_{target_frame.replace('/', '')}"
        super().__init__(self.node_name)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.callback = callback
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.timer = self.create_timer(1 / frequency, self.timer_callback)
        self._last_transform_time = 0
        self.transform_established = False
        _nodes_to_run.append(self)
        logger.info(f"[ROS] {self.node_name} ready to start")

    def timer_callback(self):
        try:
            if not self.tf_buffer.can_transform(self.target_frame, self.source_frame, rclpy.time.Time()):
                if time.perf_counter() - self._last_transform_time > 1:
                    logger.warning(f"[ROS] {self.node_name}: No transform received over 1 second")
                    self._last_transform_time = time.perf_counter()
                    self.transform_established = False
                return
            trans = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rclpy.time.Time())
            self.callback(trans.transform)
            self._last_transform_time = time.perf_counter()
            self.transform_established = True
        except Exception as e:
            logger.warning(f"[ROS] {self.node_name}: Failed to get transform: {e}")


class RosNodeRunner:  # run all the listen nodes
    """
    ROS 节点运行器
    (单例模式)
    """

    __instance = None

    def __new__(cls, *args, **kwargs):
        if not RosNodeRunner.__instance:
            RosNodeRunner.__instance = object.__new__(cls)
        return RosNodeRunner.__instance

    def __init__(self):
        self._running_nodes = []
        self._excuter = rclpy.executors.MultiThreadedExecutor()
        self._thread = threading.Thread(target=self._excuter.spin, daemon=True)

    def add_nodes(self):
        """
        向执行器添加新增的节点
        """
        for n in _nodes_to_run:
            if n not in self._running_nodes:
                self._running_nodes.append(n)
                self._excuter.add_node(n)
        logger.debug(f"[ROS] Executor added {len(self._running_nodes)} nodes")
        return self

    def run(self):
        """
        启动执行器
        """
        if not self._thread.is_alive():
            self._thread.start()
            logger.info("[ROS] Executor started")
        return self

    def stop(self):
        """
        停止执行器并销毁所有节点
        """
        while len(self._running_nodes) > 0:
            self._running_nodes.pop().destroy_node()
        self._excuter.shutdown()
        self._thread.join()
        logger.info("[ROS] Executor stopped")


class LaserScanPubNode(Node):
    """
    将飞控转发的激光雷达数据转换为ROS的LaserScan消息并发布
    (多进程模式)
    """

    def __init__(self, radar_callback_pipe):
        super().__init__("laser2PointCloud")
        self.radar_callback_pipe = radar_callback_pipe
        self.last_scantime = 0
        self.ls_pub = self.create_publisher(LaserScan, "/scan", 10)

    def radar_callback(self, pack: Union[Radar_Package, Radar_Package_Multi]):
        header = Header()
        header.frame_id = "laser_link"
        header.stamp.sec = int(time.time())
        if isinstance(pack, Radar_Package_Multi):
            for i in range(pack.package_cnt):
                angle_inc = pack.angle_increments[i]
                scantime = pack.time_stamps[i] - self.last_scantime
                self.last_scantime = pack.time_stamps[i]
                if scantime < 0:
                    scantime += 30000
                result = LaserScan(
                    header=header,
                    angle_min=pack.start_degrees[i] / 360 * 2 * 3.1415926,
                    angle_max=pack.stop_degrees[i] / 360 * 2 * 3.1415926,
                    angle_increment=angle_inc / 360 * 2 * 3.1415926,
                    time_increment=angle_inc / pack.rotation_spd,
                    scan_time=scantime / 1000,
                    range_min=0.0,
                    range_max=100.0,
                    ranges=[d / 1000 for d in pack.distances[i]],
                    intensities=pack.confidences[i],
                )
                self.ls_pub.publish(result)

    def run(self):
        logger.info("[ROS] LaserScanNode start")
        threading.Thread(target=rclpy.spin, args=(self,), daemon=True).start()
        pack = Radar_Package_Multi()
        while rclpy.ok():
            while self.radar_callback_pipe.poll():  # Check if there is data in the pipe
                data = self.radar_callback_pipe.recv_bytes()  # Receive data from the pipe
                try:
                    pack.fill_data(data)
                    self.radar_callback(pack)
                except Exception as e:
                    logger.exception("[ROS] LaserScanNode error")
            time.sleep(0.001)
        logger.info("[ROS] LaserScanNode stop")


def _run_node(node, *args):
    if not rclpy.ok():
        rclpy.init()
        time.sleep(1)
    node = node(*args)
    node.run()


class LaserScanPubNodeManager:
    """
    激光雷达发布节点管理器
    """

    def __init__(self):
        self.radar_pipe, self.radar_callback_pipe = multiprocessing.Pipe()
        self.laser_process = multiprocessing.Process(
            target=_run_node, args=(LaserScanPubNode, self.radar_callback_pipe)
        )

    def start(self, fc):
        self.laser_process.start()  # Start the LaserScanNode process
        fc.register_radar_callback(self.radar_pipe.send_bytes)  # Register the radar_callback function
        atexit.register(self.stop)  # Register the stop function

    def stop(self):
        self.laser_process.terminate()  # Terminate the LaserScanNode process
        self.laser_process.join()  # Wait for the LaserScanNode process to finish
        rclpy.shutdown()  # Shutdown ROS2
