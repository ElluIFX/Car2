import struct
import time
from typing import Any, Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from loguru import logger
from scipy.signal import find_peaks

from .Utils import calculate_crc8


class Point_2D(object):
    degree = 0.0  # 0.0 ~ 359.9, 0 指向前方, 顺时针
    distance = 0.0  # 距离 mm
    confidence = 0  # 置信度 典型值=200

    def __init__(self, degree=0.0, distance=0.0, confidence=None):
        self.degree = degree
        self.distance = distance
        self.confidence = confidence

    def __str__(self):
        s = f"Point: deg = {self.degree:>6.2f}, dist = {self.distance:>4.0f}"
        if self.confidence is not None:
            s += f", conf = {self.confidence:>3.0f}"
        return s

    def __bool__(self):
        return bool(self.distance >= 0)

    def to_xy(self) -> np.ndarray:
        """
        转换到匿名坐标系下的坐标
        """
        return np.array(
            [
                self.distance * np.cos(self.degree * np.pi / 180),
                -self.distance * np.sin(self.degree * np.pi / 180),
            ]
        )

    def to_cv_xy(self) -> np.ndarray:
        """
        转换到OpenCV坐标系下的坐标
        """
        return np.array(
            [
                self.distance * np.sin(self.degree * np.pi / 180),
                -self.distance * np.cos(self.degree * np.pi / 180),
            ]
        )

    def to_origin_xy(self) -> np.ndarray:
        """
        转换到直角坐标系下的坐标
        """
        return np.array(
            [
                self.distance * np.sin(self.degree * np.pi / 180),
                self.distance * np.cos(self.degree * np.pi / 180),
            ]
        )

    def from_xy(self, xy: np.ndarray):
        """
        从匿名坐标系下的坐标转换到点
        """
        self.degree = np.arctan2(-xy[1], xy[0]) * 180 / np.pi % 360  # type: ignore
        self.distance = np.sqrt(xy[0] ** 2 + xy[1] ** 2)  # type: ignore

    def from_cv_xy(self, xy: np.ndarray):
        """
        从OpenCV坐标系下的坐标转换到点
        """
        self.degree = np.arctan2(xy[0], -xy[1]) * 180 / np.pi % 360  # type: ignore
        self.distance = np.sqrt(xy[0] ** 2 + xy[1] ** 2)  # type: ignore

    def from_origin_xy(self, xy: np.ndarray):
        """
        从直角坐标系下的坐标转换到点
        """
        self.degree = np.arctan2(xy[0], xy[1]) * 180 / np.pi % 360  # type: ignore
        self.distance = np.sqrt(xy[0] ** 2 + xy[1] ** 2)  # type: ignore

    def __eq__(self, __o: object) -> bool:
        if not isinstance(__o, Point_2D):
            return False
        return self.degree == __o.degree and self.distance == __o.distance

    def to_180_degree(self) -> float:
        """
        转换到-180~180度
        """
        if self.degree > 180:
            return self.degree - 360
        return self.degree

    def __add__(self, other):
        if not isinstance(other, Point_2D):
            raise TypeError("Point_2D can only add with Point_2D")
        return Point_2D().from_xy(self.to_xy() + other.to_xy())

    def __sub__(self, other):
        if not isinstance(other, Point_2D):
            raise TypeError("Point_2D can only sub with Point_2D")
        return Point_2D().from_xy(self.to_xy() - other.to_xy())


class Radar_Package(object):
    """
    解析后的数据包
    """

    rotation_spd = 0  # 转速 deg/s
    start_degree = 0.0  # 扫描开始角度
    stop_degree = 0.0  # 扫描结束角度
    angle_increment = 0.0  # 角度增量
    distances: List[float] = []  # 12个点的距离数据
    confidences: List[float] = []  # 12个点的置信度数据
    points: List[Point_2D] = []  # 12个点的数据
    time_stamp = 0  # 时间戳 ms 记满30000后重置

    def __init__(self, data: Optional[bytes] = None):
        self.points = [Point_2D() for _ in range(12)]
        self.distances = [0 for _ in range(12)]
        self.confidences = [0 for _ in range(12)]
        if data is not None:
            self.fill_data(data)
        self._radar_unpack_fmt = "<HH" + "HB" * 12 + "HH"  # 雷达数据解析格式

    def fill_data(self, data: bytes):
        datas = struct.unpack(self._radar_unpack_fmt, data)
        self.rotation_spd = datas[0]
        self.start_degree = datas[1] * 0.01
        self.stop_degree = datas[26] * 0.01
        self.time_stamp = datas[27]
        deg_step = (self.stop_degree - self.start_degree) % 360 / 11
        self.angle_increment = deg_step
        for n, point in enumerate(self.points):
            point.distance = datas[2 + n * 2]
            self.distances[n] = point.distance
            point.confidence = datas[3 + n * 2]
            self.confidences[n] = point.confidence
            point.degree = (self.start_degree + n * deg_step) % 360

    def __str__(self):
        string = (
            f"--- Radar Package < TS = {self.time_stamp:05d} ms > ---\n"
            f"Range: {self.start_degree:06.2f}° -> {self.stop_degree:06.2f}° {self.rotation_spd/360:3.2f}rpm\n"
        )
        for n, point in enumerate(self.points):
            string += f"#{n:02d} {point}\n"
        string += "--- End of Info ---"
        return string


class Radar_Package_Multi(object):
    """
    解析后的数据包, 不定长
    """

    rotation_spd = 0  # 转速 deg/s
    points: List[Point_2D] = []  # 多个点的数据
    points_cnt = 0  # 点数
    package_cnt = 0  # 包数
    start_degrees: List[float] = []  # 扫描开始角度
    stop_degrees: List[float] = []  # 扫描结束角度
    time_stamps: List[int] = []  # 时间戳 ms 记满30000后重置
    distances: List[List[float]] = []  # 多个点的距离数据
    confidences: List[List[int]] = []  # 多个点的置信度数据
    angle_increments: List[float] = []  # 角度增量

    def __init__(self, data: Optional[bytes] = None):
        if data is not None:
            self.fill_data(data)

    def fill_data(self, data: bytes):
        pts = (len(data) - 2) // 42
        datas = struct.unpack("<H" + ("H" + "HB" * 12 + "HH") * pts, data)
        self.points = []
        self.angle_increments = []
        self.start_degrees = []
        self.stop_degrees = []
        self.time_stamps = []
        self.distances = []
        self.rotation_spd = datas[0]
        self.points_cnt = pts * 12
        self.package_cnt = pts
        for i in range(pts):
            start_degree = datas[1 + i * 27] * 0.01
            stop_degree = datas[26 + i * 27] * 0.01
            timestamp = datas[27 + i * 27]
            deg_step = (stop_degree - start_degree) % 360 / 11
            self.angle_increments.append(deg_step)
            self.start_degrees.append(start_degree)
            self.stop_degrees.append(stop_degree)
            self.time_stamps.append(timestamp)
            self.distances.append([])
            self.confidences.append([])
            for n in range(12):
                self.points.append(
                    Point_2D(
                        degree=(start_degree + n * deg_step) % 360,
                        distance=datas[2 + n * 2 + i * 27],
                        confidence=datas[3 + n * 2 + i * 27],
                    )
                )
                self.distances[i].append(datas[2 + n * 2 + i * 27])
                self.confidences[i].append(datas[3 + n * 2 + i * 27])

    def __str__(self):
        string = f"--- Radar Package Multi Points ---\n"
        for n, point in enumerate(self.points):
            string += f"#{n:02d} {point}\n"
        string += "--- End of Info ---"
        return string


def resolve_radar_data(data: bytes, to_package: Optional[Radar_Package] = None) -> Optional[Radar_Package]:
    """
    解析雷达原始数据
    data: bytes 原始数据
    to_package: 传入一个RadarPackage对象, 如果不传入, 则会新建一个
    return: 解析后的RadarPackage对象
    """
    if len(data) != 47:  # fixed length of radar data
        logger.warning(f"[RADAR] Invalid data length: {len(data)}")
        return None
    if calculate_crc8(data[:-1]) != data[-1]:
        logger.warning("[RADAR] Invalid CRC8")
        return None
    if data[:2] != b"\x54\x2C":
        logger.warning(f"[RADAR] Invalid header")
        return None
    if to_package is None:
        return Radar_Package(data[2:-1])
    else:
        to_package.fill_data(data[2:-1])
        return to_package


def resolve_radar_data_multi(
    data: bytes, to_package: Optional[Radar_Package_Multi] = None
) -> Optional[Radar_Package_Multi]:
    """
    解析雷达不定长原始数据
    data: bytes 原始数据
    to_package: 传入一个RadarPackage对象, 如果不传入, 则会新建一个
    return: 解析后的RadarPackage对象
    """
    if (len(data) - 2) % 42 != 0:  # fixed length of radar data
        logger.warning(f"[RADAR] Invalid data length: {len(data)}")
        return None
    if to_package is None:
        return Radar_Package_Multi(data)
    else:
        to_package.fill_data(data)
        return to_package


class Map_Circle(object):
    """
    将点云数据映射到一个圆上
    """

    ######### 映射方法 ########
    ACC = 3  # 精度(总点数=360*ACC)
    REMAP = 2  # 映射范围(越大精度越低, 但是残影越少,实际映射范围=+-UPDATE_MAP/ACC度)
    MODE_MIN = 0  # 在范围内选择最近的点更新
    MODE_MAX = 1  # 在范围内选择最远的点更新
    MODE_AVG = 2  # 计算平均值更新
    ######### 设置 #########
    confidence_threshold = 40  # 置信度阈值
    distance_threshold = 10  # 距离阈值
    timeout_clear = True  # 超时清除
    timeout_time = 0.4  # 超时时间 s

    def __init__(self):
        ######### 状态 #########
        self.rotation_spd = 0.0  # 转速 rpm
        self.update_count = 0  # 更新计数
        self.update_mode = self.MODE_MIN
        ####### 辅助计算 #######
        self._rad_arr = np.deg2rad(np.arange(0, 360, 1 / self.ACC))  # 弧度
        self._deg_arr = np.arange(0, 360, 1 / self.ACC)  # 角度
        self._sin_arr = np.sin(self._rad_arr)
        self._cos_arr = np.cos(self._rad_arr)
        self.data = np.ones(360 * self.ACC, dtype=np.int64) * -1  # -1: 未知
        self.time_stamp = np.zeros(360 * self.ACC, dtype=np.float64)  # 时间戳
        self.avail_points = 0  # 有效点数
        self.total_points = 360 * self.ACC  # 总点数

    def update(self, data: Union[Radar_Package, Radar_Package_Multi]):
        """
        映射解析后的点云数据
        """
        deg_values_dict: Dict[Any, Any] = {}
        for point in data.points:
            if point.distance < self.distance_threshold or point.confidence < self.confidence_threshold:
                continue
            base = round(point.degree * self.ACC)
            if self.REMAP == 0:
                base %= 360 * self.ACC
                if base not in deg_values_dict:
                    deg_values_dict[base] = []
                deg_values_dict[base].append(point.distance)
            else:
                degs = np.arange(base - self.REMAP, base + self.REMAP + 1, dtype=int)
                degs %= 360 * self.ACC
                for deg in degs:
                    if deg not in deg_values_dict:
                        deg_values_dict[deg] = []
                    deg_values_dict[deg].append(point.distance)
        for deg, values in deg_values_dict.items():
            if self.update_mode == self.MODE_MIN:
                self.data[deg] = np.min(values)
            elif self.update_mode == self.MODE_MAX:
                self.data[deg] = np.max(values)
            elif self.update_mode == self.MODE_AVG:
                self.data[deg] = np.round(np.mean(values))
            if self.timeout_clear:
                self.time_stamp[deg] = time.perf_counter()
        if self.timeout_clear:
            self.data[self.time_stamp < time.perf_counter() - self.timeout_time] = -1
        self.rotation_spd = data.rotation_spd / 360 * 60
        self.update_count += 1
        self.avail_points = np.count_nonzero(self.data != -1)

    def in_deg(self, from_: float, to_: float) -> List[Point_2D]:
        """
        截取选定角度范围的点 / deg

        返回值: List[Point_2D] / mm
        """
        from_ = round(from_ * self.ACC)
        to_ = round(to_ * self.ACC)
        return [Point_2D(deg, self.data[deg]) for deg in range(from_, to_ + 1) if self.data[deg] != -1]

    def __getitem__(self, item):
        """
        获取指定角度的距离 / mm
        """
        return self.data[round(item * self.ACC)]

    def clear(self):
        """
        清空数据
        """
        self.data[:] = -1
        self.time_stamp[:] = 0

    def rotation(self, angle: float):
        """
        旋转整个地图, 正角度代表坐标系顺时针旋转, 地图逆时针旋转 / deg
        """
        angle = round(angle * self.ACC)
        self.data = np.roll(self.data, angle)
        self.time_stamp = np.roll(self.time_stamp, angle)

    def find_nearest(self, from_: float = 0, to_: float = 359, num=1, range_limit=1e7, view=None) -> List[Point_2D]:
        """
        在给定范围内查找给定个数的最近点
        from_:  起始角度 / deg
        to_:  结束角度(包含) / deg
        num:  查找点的个数
        range_limit:  距离限制 / mm
        view: numpy视图, 当指定时上述参数仅num生效

        返回:  Point_2D列表 / mm
        """
        if view is None:
            from_ %= 360
            to_ %= 360
            view = (self.data < range_limit) & (self.data >= 0)
            view &= (
                (self._deg_arr >= from_) & (self._deg_arr <= to_)
                if from_ <= to_
                else ((self._deg_arr >= from_) | (self._deg_arr <= to_))
            )
        indices = np.where(view)[0]
        if len(indices) == 0:
            return []
        elif len(indices) <= num:
            sorted_indices = np.argsort(self.data[indices])
        else:
            sorted_indices = np.argpartition(self.data[indices], num)[:num]
        points = [Point_2D(self._deg_arr[indices[i]], self.data[indices[i]]) for i in sorted_indices]
        return points

    def find_nearest_with_ext_point_opt(
        self, from_: float = 0, to_: float = 359, num=1, range_limit=1e7
    ) -> List[Point_2D]:
        """
        在给定范围内查找给定个数的最近点, 只查找极值点
        from_: 起始角度 / deg
        to_: 结束角度(包含) / deg
        num: 查找点的个数
        range_limit: 距离限制 / mm

        返回:  Point_2D列表 / mm
        """
        from_ %= 360
        to_ %= 360
        view = (self.data < range_limit) & (self.data != -1)
        view &= (
            (self._deg_arr >= from_) & (self._deg_arr <= to_)
            if from_ <= to_
            else ((self._deg_arr >= from_) | (self._deg_arr <= to_))
        )
        data_view = self.data[view]
        deg_arr = np.where(view)[0]
        peak = find_peaks(-data_view)[0]
        if len(data_view) > 2:
            if data_view[-1] < data_view[-2]:
                peak = np.append(peak, len(data_view) - 1)
        peak_deg = deg_arr[peak]
        new_view = np.zeros(360 * self.ACC, dtype=bool)
        new_view[peak_deg] = True
        return self.find_nearest(num=num, range_limit=range_limit, view=new_view)

    def find_two_point_with_given_distance(
        self,
        from_: float,
        to_: float,
        distance: int,
        range_limit=1e7,
        threshold=15,
    ) -> List[Point_2D]:
        """
        在给定范围内查找两个给定距离的点
        from_: 起始角度 / deg
        to_: 结束角度(包含) / deg
        distance: 给定的两点之间的距离
        range_limit: 距离限制 / mm
        threshold: 允许的距离误差

        return: [Point_2D, Point_2D] / mm
        """
        fd_points = self.find_nearest(from_, to_, 20, range_limit)
        num = len(fd_points)
        if num < 2:
            return []
        xy_points = np.array([p.to_xy() for p in fd_points])
        delta_dis = np.sqrt(((xy_points[:, None] - xy_points) ** 2).sum(axis=-1))
        mask = np.abs(delta_dis - distance) < threshold
        np.fill_diagonal(mask, False)

        indices = np.argwhere(mask)
        if len(indices) == 0:
            return []
        p1 = fd_points[indices[0][0]]  # type: ignore
        p2 = fd_points[indices[0][1]]  # type: ignore
        return [p1, p2]

    def draw_on_cv_image(
        self,
        img: np.ndarray,
        scale: float = 1,
        color: tuple = (0, 0, 255),
        point_size: int = 1,
        add_points: List[Point_2D] = [],
        add_points_color: tuple = (0, 255, 255),
        add_points_size: int = 2,
    ):
        """
        在给定的图像上绘制激光数据
        """
        img_size = img.shape
        center_point = np.array([img_size[1] / 2, img_size[0] / 2])  # type: ignore
        points_pos = (
            np.array(
                [
                    self.data * self._sin_arr,
                    -self.data * self._cos_arr,
                ]
            )
            * scale
        )
        for n in range(360 * self.ACC):
            pos = points_pos[:, n] + center_point
            if self.data[n] != -1:
                cv2.circle(img, tuple(pos.astype(int)), point_size, color, -1)
        for point in add_points:
            pos = center_point + point.to_cv_xy() * scale
            cv2.circle(img, (int(pos[0]), int(pos[1])), add_points_size, add_points_color, -1)
        return img

    def output_cloud(self, scale: float = 0.1, size=800) -> np.ndarray:
        """
        输出点云图
        """
        black_img = np.zeros((size, size, 1), dtype=np.uint8)
        select = self.data != -1
        points_pos = (
            np.array([self.data[select] * self._sin_arr[select], -self.data[select] * self._cos_arr[select]]) * scale
            + size // 2
        )
        select = np.logical_and(points_pos >= 0, points_pos < size)
        points_pos = points_pos[:, np.all(select, axis=0)]
        black_img[points_pos[1].astype(int), points_pos[0].astype(int)] = 255
        return black_img

    def output_points(self, scale: float = 0.1, remove_unavil=True) -> np.ndarray:
        """
        输出点云坐标
        remove_unavil: 是否移除无效点
        """
        if remove_unavil:
            select = self.data != -1
            points_pos = (
                np.array([self.data[select] * self._cos_arr[select], -self.data[select] * self._sin_arr[select]])
                * scale
            )
        else:
            points_pos = np.array([self.data * self._cos_arr, -self.data * self._sin_arr]) * scale
        return points_pos

    def output_polyline_cloud(
        self, scale: float = 0.1, size=800, thickness=1, draw_outside=True, boundary=None
    ) -> np.ndarray:
        """
        输出多边形点云图
        """
        black_img = np.zeros((size, size, 1), dtype=np.uint8)
        select = self.data != -1
        if boundary is not None:
            select = np.logical_and(select, self.data < boundary)
        points_pos = (
            np.array([self.data[select] * self._sin_arr[select], -self.data[select] * self._cos_arr[select]]) * scale
            + size // 2
        )
        if not draw_outside:
            points_pos = points_pos[:, np.all(np.logical_and(points_pos >= 0, points_pos < size), axis=0)]
        if points_pos.size > 0:
            cv2.polylines(black_img, [points_pos.T.astype(np.int32)], True, 255, thickness)
        return black_img

    def get_distance(self, angle: float) -> int:
        """
        获取给定角度的距离 / degree, mm
        """
        return self.data[round((angle % 360) * self.ACC)]

    def get_point(self, angle: float) -> Point_2D:
        """
        获取给定角度的点 / degree, mm
        """
        return Point_2D(angle, self.data[round((angle % 360) * self.ACC)])

    def __str__(self):
        string = "--- Circle Map ---\n"
        invalid_count = 0
        for deg in range(360 * self.ACC):
            if self.data[deg] == -1:
                invalid_count += 1
                continue
            string += f"{deg / self.ACC:.2f}° = {self.data[deg]} mm\n"
        if invalid_count > 0:
            string += f"Hided {invalid_count:03d} invalid points\n"
        string += "--- End of Info ---"
        return string

    def __repr__(self):
        return self.__str__()
