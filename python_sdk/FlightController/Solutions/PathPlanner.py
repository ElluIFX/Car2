import time
from collections import deque
from typing import Any, List, Literal, Optional, Tuple, Union

import numpy as np
from loguru import logger
from matplotlib import pyplot as plt


class TrajectoryGenerator:
    """五次多项式轨迹生成器"""

    def __init__(
        self, start_pos, des_pos, T, start_vel=[0, 0, 0], des_vel=[0, 0, 0], start_acc=[0, 0, 0], des_acc=[0, 0, 0]
    ):
        """五次多项式轨迹生成器

        Args:
            start_pos (List[float,...]): 起始位置
            des_pos (List[float,...]): 目标位置
            T (float): 轨迹总时间
            start_vel (List[float,...], optional): 起始速度矢量
            des_vel (List[float,...], optional): 目标速度矢量
            start_acc (List[float,...], optional): 起始加速度矢量
            des_acc (List[float,...], optional): 目标加速度矢量
        """
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]

        self.T = T

    def solve(self):
        A = np.array(
            [
                [0, 0, 0, 0, 0, 1],
                [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
                [0, 0, 0, 0, 1, 0],
                [5 * self.T**4, 4 * self.T**3, 3 * self.T**2, 2 * self.T, 1, 0],
                [0, 0, 0, 2, 0, 0],
                [20 * self.T**3, 12 * self.T**2, 6 * self.T, 2, 0, 0],
            ]
        )

        b_x = np.array(
            [[self.start_x], [self.des_x], [self.start_x_vel], [self.des_x_vel], [self.start_x_acc], [self.des_x_acc]]
        )

        b_y = np.array(
            [[self.start_y], [self.des_y], [self.start_y_vel], [self.des_y_vel], [self.start_y_acc], [self.des_y_acc]]
        )

        b_z = np.array(
            [[self.start_z], [self.des_z], [self.start_z_vel], [self.des_z_vel], [self.start_z_acc], [self.des_z_acc]]
        )

        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)

    def calc_position(self, axis: Literal["x", "y", "z"], t: float) -> float:
        c = getattr(self, axis + "_c")
        return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]

    def calc_position_xyz(self, t: float) -> Tuple[float, float, float]:
        return (
            self.calc_position("x", t),
            self.calc_position("y", t),
            self.calc_position("z", t),
        )

    def calc_velocity(self, axis: Literal["x", "y", "z"], t: float) -> float:
        c = getattr(self, axis + "_c")
        return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]

    def calc_velocity_xyz(self, t: float) -> Tuple[float, float, float]:
        return (
            self.calc_velocity("x", t),
            self.calc_velocity("y", t),
            self.calc_velocity("z", t),
        )

    def calc_acceleration(self, axis: Literal["x", "y", "z"], t: float) -> float:
        c = getattr(self, axis + "_c")
        return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]

    def calc_acceleration_xyz(self, t: float) -> Tuple[float, float, float]:
        return (
            self.calc_acceleration("x", t),
            self.calc_acceleration("y", t),
            self.calc_acceleration("z", t),
        )


class PFBPP(object):
    """
    Potential Field based path planner
    (基于势场的路径规划器)
    """

    # the number of previous positions used to check oscillations
    OSCILLATIONS_DETECTION_LENGTH = 4
    OSCILLATIONS_RETRY_FALLBACK = 10
    OSCILLATIONS_MAX_RETRY = 10

    def __init__(self) -> None:
        self._area_width = 30.0  # potential area width [m]
        self._grid_size = 0.5  # potential grid size [m]
        self._robot_radius = 1.0  # robot radius [m]
        self._atg = 5.0  # attractive potential
        self._rpg = 100.0  # repulsive potential
        self._attr_calced = False
        self._repu_calced = False

    def _calc_potential_field(self):
        """计算势场"""
        minx = min(min(self._ox), self._sx, self._gx) - self._area_width / 2.0
        miny = min(min(self._oy), self._sy, self._gy) - self._area_width / 2.0
        maxx = max(max(self._ox), self._sx, self._gx) + self._area_width / 2.0
        maxy = max(max(self._oy), self._sy, self._gy) + self._area_width / 2.0
        xw = int(round((maxx - minx) / self._grid_size))
        yw = int(round((maxy - miny) / self._grid_size))

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        if not self._attr_calced:
            self._ugxy = []
        if not self._repu_calced:
            self._uoxy = []
        i = 0
        for ix in range(xw):
            x = ix * self._grid_size + minx
            for iy in range(yw):
                y = iy * self._grid_size + miny
                if not self._attr_calced:
                    self._ugxy.append(self._calc_attractive_potential(x, y))
                if not self._repu_calced:
                    self._uoxy.append(self._calc_repulsive_potential(x, y))
                uf = self._ugxy[i] + self._uoxy[i]
                pmap[ix][iy] = uf
                i += 1
        self._attr_calced = self._repu_calced = True
        return pmap, minx, miny

    def _calc_attractive_potential(self, x, y):
        """计算吸引势"""
        return 0.5 * self._atg * np.hypot(x - self._gx, y - self._gy)

    def _calc_repulsive_potential(self, x, y):
        """计算斥力势"""
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(self._ox):
            d = np.hypot(x - self._ox[i], y - self._oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - self._ox[minid], y - self._oy[minid])

        if dq <= self._robot_radius:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * self._rpg * (1.0 / dq - 1.0 / self._robot_radius) ** 2
        else:
            return 0.0

    def _oscillations_detection(self, previous_ids, ix, iy):
        """振荡检测器"""
        previous_ids.append((ix, iy))

        if len(previous_ids) > PFBPP.OSCILLATIONS_DETECTION_LENGTH:
            previous_ids.popleft()

        # check if contains any duplicates by copying into a set
        previous_ids_set = set()
        for index in previous_ids:
            if index in previous_ids_set:
                return True
            else:
                previous_ids_set.add(index)
        return False

    def set_plan_path(self, start_point, goal_point):
        """设置规划起点和终点

        Args:
            start_point (二维点): 起点 / m
            goal_point (二维点): 终点 / m
        """
        self._start_point = start_point
        self._goal_point = goal_point
        self._sx, self._sy = self._start_point
        self._gx, self._gy = self._goal_point
        self._attr_calced = False

    def set_obstacle(self, obstacle_list):
        """设置障碍物

        Args:
            obstacle_list (二维点集): 障碍物列表 / m
        """
        self._obstacle_list = obstacle_list
        self._ox = []
        self._oy = []
        for point in self._obstacle_list:
            self._ox.append(point[0])
            self._oy.append(point[1])
        self._repu_calced = False

    def create_boundry(self, p1, p2, space=None) -> list:
        """创建边界

        Args:
            p1 (tuple): 点1 / m
            p2 (tuple): 点2 / m
            space (float): 点间距 / m 默认取网格间距

        Returns:
            list: 边界点列表 / m
        """
        obstacle = []
        s = space if space else self._grid_size
        s = abs(s)
        for x in np.arange(p1[0], p2[0], s if p1[0] < p2[0] else -s):
            obstacle.append((x, p1[1]))
            obstacle.append((x, p2[1]))
        for y in np.arange(p1[1], p2[1], s if p1[1] < p2[1] else -s):
            obstacle.append((p1[0], y))
            obstacle.append((p2[0], y))
        return obstacle

    def set_params(self, area_width, grid_size, robot_radius):
        """设置参数

        Args:
            area_width (float): 地图宽度 / m
            grid_size (float): 网格大小 / m
            robot_radius (float): 机器人半径 / m
        """
        self._area_width = area_width
        self._grid_size = grid_size
        self._robot_radius = robot_radius
        self._repu_calced = self._attr_calced = False

    def set_attractive_gain(self, attractive_gain=5.0):
        """设置吸引势增益

        Args:
            attractive_gain (float): 吸引势增益
        """
        self._atg = attractive_gain
        self._attr_calced = False

    def set_repulsive_gain(self, repulsive_gain=100.0):
        """设置斥力势增益

        Args:
            repulsive_gain (float): 斥力势增益
        """
        self._rpg = repulsive_gain
        self._repu_calced = False

    def calc_potential_field(self):
        """计算势场"""
        # calc potential field
        self._pmap, self._minx, self._miny = self._calc_potential_field()
        logger.info("Potential field calculation finished.")

    def run_planner(self, debug=False, osc_retry=True):
        """运行规划器

        Args:
            debug (bool): 是否显示调试图像
            osc_retry (bool): 检测到振荡是否重试

        Returns:
            path (list): 路径点列表 / m, None表示规划失败
        """
        if not (self._attr_calced and self._repu_calced):
            self.calc_potential_field()

        # search path
        d = np.hypot(self._sx - self._gx, self._sy - self._gy)
        ix = round((self._sx - self._minx) / self._grid_size)
        iy = round((self._sy - self._miny) / self._grid_size)
        gix = round((self._gx - self._minx) / self._grid_size)
        giy = round((self._gy - self._miny) / self._grid_size)

        if debug:
            plt.clf()
            plt.grid(True)
            plt.axis("equal")
            data = np.array(self._pmap).T
            plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)  # type: ignore
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        points = [(self._sx, self._sy)]

        motion = [[1, 0], [0, 1], [-1, 0], [0, -1], [-1, -1], [-1, 1], [1, -1], [1, 1]]
        previous_ids = deque()

        while d >= self._grid_size:
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(self._pmap) or iny >= len(self._pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    logger.error("Outside area! ({},{})".format(inx, iny))
                else:
                    p = self._pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * self._grid_size + self._minx
            yp = iy * self._grid_size + self._miny
            d = np.hypot(self._gx - xp, self._gy - yp)
            points.append((xp, yp))
            if self._oscillations_detection(previous_ids, ix, iy):
                logger.warning("Oscillation detected at ({},{})!".format(ix, iy))
                if not osc_retry:
                    return None
                old_radius = self._robot_radius
                points = points[: -PFBPP.OSCILLATIONS_RETRY_FALLBACK]
                old_start_point = self._start_point
                if PFBPP.OSCILLATIONS_RETRY_FALLBACK > PFBPP.OSCILLATIONS_DETECTION_LENGTH:
                    self._start_point = points[-1] if len(points) > 0 else old_start_point
                    self._sx, self._sy = self._start_point
                    self._attr_calced = False
                    logger.info(f"Oscillation fallback to {self._start_point}")
                new_point = None
                retry = 0
                while new_point is None:
                    self._robot_radius += self._grid_size * retry
                    self._repu_calced = False
                    retry += 1
                    if retry > PFBPP.OSCILLATIONS_MAX_RETRY:
                        logger.error("Oscillation retry reached max retry!")
                        return None
                    logger.debug(f"Retry#{retry} with radius {self._robot_radius}")
                    new_point = self.run_planner(debug=debug, osc_retry=False)
                self._robot_radius = old_radius
                self._repu_calced = False
                if PFBPP.OSCILLATIONS_RETRY_FALLBACK > PFBPP.OSCILLATIONS_DETECTION_LENGTH:
                    self._start_point = old_start_point
                    self._sx, self._sy = self._start_point
                    self._attr_calced = False
                    return points + new_point
                else:
                    return new_point
            if debug:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)
        logger.info(f"Plan success")
        points.append(self._goal_point)
        return points
