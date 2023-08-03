import math
import time
from typing import List, Literal, Optional, Tuple

import cv2
import numpy as np
from FlightController.Components.LDRadar_Resolver import Point_2D
from scipy.spatial.transform import Rotation

############参数设置##############
KERNAL_DI = 9  # 膨胀核大小
KERNAL_ER = 5  # 腐蚀核大小
HOUGH_THRESHOLD = 80
MIN_LINE_LENGTH = 60
# KERNAL_DI = 9  # 膨胀核大小
# KERNAL_ER = 5  # 腐蚀核大小
# HOUGH_THRESHOLD = 50
# MIN_LINE_LENGTH = 60
MAX_LINE_GAP = 200
#################################
kernel_di = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNAL_DI, KERNAL_DI))
kernel_er = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNAL_ER, KERNAL_ER))

x_dbg = 0.0
y_dbg = 0.0
yaw_dbg = 0.0


def radar_resolve_rt_pose(
    img, debug=False, debug_save_img=False, skip_di=False, skip_er=False
) -> Tuple[Optional[float], Optional[float], Optional[float]]:
    """从雷达点云图像中解析出中点位置

    Args:
        img: 雷达点云图像(灰度图)
        debug: 显示解析结果
        debug_save_img: 保存解析结果
        skip_di: 是否跳过膨胀
        skip_er: 是否跳过腐蚀

    Returns:
        位姿(x,y,yaw) / pixels/deg
    """
    if not skip_di:
        img = cv2.dilate(img, kernel_di)  # 膨胀
    if not skip_er:
        img = cv2.erode(img, kernel_er)  # 腐蚀
    lines = cv2.HoughLinesP(
        img,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=MAX_LINE_GAP,
    )

    if debug:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    if lines is None:
        return None, None, None

    x0, y0 = img.shape[0] // 2, img.shape[1] // 2
    x_out = None
    y_out = None
    yaw_out_1 = None
    yaw_out_2 = None

    # # mathod 1: 我也忘了这堆条件怎么想的了, 要配合选取最近线的条件
    # select_right = ((lines[:, :, 0] > x0) & (lines[:, :, 2] > x0)) & (
    #     ((lines[:, :, 1] > y0) & (lines[:, :, 3] < y0)) | ((lines[:, :, 1] < y0) & (lines[:, :, 3] > y0))
    # )
    # select_back = ((lines[:, :, 1] > y0) & (lines[:, :, 3] > y0)) & (
    #     ((lines[:, :, 0] > x0) & (lines[:, :, 2] < x0)) | ((lines[:, :, 0] < x0) & (lines[:, :, 2] > x0))
    # )

    # mathod 2: 过中心点画个X, 根据中点在X四个区域中的哪个判断是哪一侧的直线, 选取角度差最小的直线
    # midpoints = (lines[:, :, :2] + lines[:, :, 2:]) / 2  # 原点指向中点的向量角度
    # degs = np.degrees(np.arctan2(midpoints[:, :, 1] - y0, midpoints[:, :, 0] - x0))
    # select_right = (degs > -45) & (degs < 45)
    # select_back = (degs > 45) & (degs < 135)

    # mathod 3: 计算每条线的角度, 再根据中点的x,y坐标判断是哪一侧的直线
    angles = np.degrees(np.arctan2(lines[:, :, 3] - lines[:, :, 1], lines[:, :, 2] - lines[:, :, 0]))
    midpoints = (lines[:, :, :2] + lines[:, :, 2:]) / 2
    select_right = ((angles > 45) | (angles < -45)) & (midpoints[:, :, 0] > x0)
    select_back = ((angles > -45) & (angles < 45)) & (midpoints[:, :, 1] > y0)

    right_lines = lines[select_right]
    back_lines = lines[select_back]

    if right_lines.shape[0] != 0:
        dists, angles = get_point_line_distance_np([x0, y0], right_lines)
        line_index = np.argmin(dists)  # 选取距离最近的直线
        # line_index = np.argmin(np.abs(angles))  # 选取角度最小的直线
        y_out = dists[line_index]
        yaw_out_1 = -angles[line_index]

    if back_lines.shape[0] != 0:
        dists, angles = get_point_line_distance_np([x0, y0], back_lines)
        line_index = np.argmin(dists)
        # line_index = np.argmin(np.abs(angles - 90))
        x_out = dists[line_index]
        yaw_out_2 = -angles[line_index] + 90

    if yaw_out_1 is not None and yaw_out_2 is not None:
        if abs(yaw_out_1 - yaw_out_2) > 30:  # 太离谱的角度差直接舍弃
            yaw_out = None
        else:
            yaw_out = (yaw_out_1 + yaw_out_2) / 2
    elif yaw_out_1 is not None:
        yaw_out = yaw_out_1
    elif yaw_out_2 is not None:
        yaw_out = yaw_out_2
    else:
        yaw_out = None

    if debug:
        global x_dbg, y_dbg, yaw_dbg
        for x1, y1, x2, y2 in right_lines:
            cv2.line(img, (x1, y1), (x2, y2), (180, 180, 0), 1)
        for x1, y1, x2, y2 in back_lines:
            cv2.line(img, (x1, y1), (x2, y2), (0, 180, 180), 1)
        for x1, y1, x2, y2 in lines[~(select_right | select_back)]:
            cv2.line(img, (x1, y1), (x2, y2), (0, 120, 0), 1)
        if x_out is not None:
            x_dbg += (x_out - x_dbg) * 0.1  # type: ignore
        if y_out is not None:
            y_dbg += (y_out - y_dbg) * 0.1  # type: ignore
        if yaw_out is not None:
            yaw_dbg += (yaw_out - yaw_dbg) * 0.1
        if yaw_out_1 is not None:
            target = Point_2D(-yaw_out_1 + 90, y_out).to_cv_xy() + np.array([x0, y0])
            cv2.line(img, (x0, y0), (int(target[0]), int(target[1])), (150, 150, 50), 2)
        if yaw_out_2 is not None:
            target = Point_2D(-yaw_out_2 + 180, x_out).to_cv_xy() + np.array([x0, y0])
            cv2.line(img, (x0, y0), (int(target[0]), int(target[1])), (50, 150, 150), 2)
        target = Point_2D(-yaw_dbg, 50).to_cv_xy() + np.array([x0, y0])
        cv2.line(img, (x0, y0), (int(target[0]), int(target[1])), (0, 0, 255), 2)
        target = Point_2D(-yaw_dbg + 90, y_dbg).to_cv_xy() + np.array([x0, y0])
        cv2.line(img, (x0, y0), (int(target[0]), int(target[1])), (255, 255, 0), 2)
        target = Point_2D(-yaw_dbg + 180, x_dbg).to_cv_xy() + np.array([x0, y0])
        cv2.line(img, (x0, y0), (int(target[0]), int(target[1])), (0, 255, 255), 2)
        cv2.putText(
            img,
            f"({x_dbg:.1f}, {y_dbg:.1f}, {yaw_dbg:.1f})",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            1,
        )
        if debug_save_img:
            cv2.imwrite("radar_resolve_debug.png", img)
        else:
            cv2.imshow("Map Resolve", img)
    return x_out, y_out, yaw_out  # type: ignore


def get_point_line_distance_np(point, lines) -> Tuple[np.ndarray, np.ndarray]:
    """
    分别计算一个点到各条线的距离

    Args:
        point: 目标点 [x,y]
        lines: 线的两个端点 [[x1,y1,x2,y2],...]

    Returns:
        距离 / pixel, 角度(-90~90)
    """
    # point = np.asarray(point)
    # lines = np.asarray(lines)
    x1, y1, x2, y2 = lines.T

    # 计算线段长度
    line_lengths = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    # 计算点到线段的投影长度
    projection_lengths = ((point[0] - x1) * (x2 - x1) + (point[1] - y1) * (y2 - y1)) / line_lengths

    # 计算投影点坐标
    px = x1 + projection_lengths * (x2 - x1) / line_lengths
    py = y1 + projection_lengths * (y2 - y1) / line_lengths

    # 计算点到投影点的距离
    distances = np.sqrt((point[0] - px) ** 2 + (point[1] - py) ** 2)

    # 计算角度
    angles = np.degrees(np.arctan2(py - point[1], px - point[0]))

    return distances, angles


class ICPM(object):
    """
    具有奇异值分解的 2D Iterative Closest Point Matching (迭代最近点匹配)
    """

    # ICP parameters
    _EPS = 0.0001  # 收敛阈值
    _MAX_ITER = 100  # 最大迭代次数

    # return values
    CONVERGED = 0  # 收敛
    NOT_CONVERGED = 1  # 未收敛
    MAX_ITER_REACHED = 2  # 达到最大迭代次数

    @staticmethod
    def _update_homogeneous_matrix(Hin, R, T):
        """
        更新齐次矩阵
        """
        r_size = R.shape[0]
        H = np.zeros((r_size + 1, r_size + 1))
        H[0:r_size, 0:r_size] = R
        H[0:r_size, r_size] = T
        H[r_size, r_size] = 1.0
        if Hin is None:
            return H
        else:
            return Hin @ H

    @staticmethod
    def _nearest_neighbor_association(previous_points, current_points):
        """
        最近邻关联
        """
        delta_points = previous_points - current_points
        d = np.linalg.norm(delta_points, axis=0)
        error = sum(d)
        d = np.linalg.norm(
            np.repeat(current_points, previous_points.shape[1], axis=1)
            - np.tile(previous_points, (1, current_points.shape[1])),
            axis=0,
        )
        indexes = np.argmin(d.reshape(current_points.shape[1], previous_points.shape[1]), axis=1)
        return indexes, error

    @staticmethod
    def _svd_motion_estimation(previous_points, current_points):
        """
        奇异值分解运动估计
        """
        pm = np.mean(previous_points, axis=1)
        cm = np.mean(current_points, axis=1)
        p_shift = previous_points - pm[:, np.newaxis]
        c_shift = current_points - cm[:, np.newaxis]
        W = c_shift @ p_shift.T
        u, s, vh = np.linalg.svd(W)
        R = (u @ vh).T
        t = pm - (R @ cm)
        return R, t

    def __init__(self, template_points=None) -> None:
        """初始化

        Args:
            template_points: 二维模板点云, 用于匹配
        """
        self._template_points = template_points
        self._R = None
        self._T = None

    def update_template(self, template_points):
        """更新模板点云

        Args:
            template_points: 二维或三维的模板点云, 用于匹配
        """
        self._template_points = template_points

    @property
    def template_created(self):
        return self._template_points is not None

    def match(self, points, debug=False, debug_save_img=False, debug_size=500):
        """匹配新的点云

        Args:
            points: 二维点云
            debug: 是否显示调试图片
            debug_save_img: 是否保存调试图片

        Returns:
            int: 执行结果代码 (CONVERGED, NOT_CONVERGED, MAX_ITER_REACHED)
            int: 匹配误差 (最小二乘法残差和)
        """
        if self._template_points is None:
            raise ValueError("Template points not set")
        H = None  # homogeneous transformation matrix
        dError = np.inf
        preError = np.inf
        count = 0
        while dError > self._EPS:
            count += 1
            indexes, error = ICPM._nearest_neighbor_association(self._template_points, points)
            Rt, Tt = ICPM._svd_motion_estimation(self._template_points[:, indexes], points)
            points = (Rt @ points) + Tt[:, np.newaxis]
            dError = preError - error
            if dError < 0:
                if debug:
                    print(f"Not Converge: {preError}, {dError}, {count}")
                ret = self.NOT_CONVERGED
                error = preError
                break
            preError = error
            H = ICPM._update_homogeneous_matrix(H, Rt, Tt)
            if count > self._MAX_ITER:
                if debug:
                    print(f"Iter max: {error}, {dError}, {count}")
                ret = self.MAX_ITER_REACHED
                break
        else:
            if debug:
                print(f"Converge: {error}, {dError}, {count}")
            ret = self.CONVERGED
        self._R = np.array(H[0:-1, 0:-1])
        self._T = np.array(H[0:-1, -1])
        if debug:
            self._plot_points(self._template_points, points, debug_save_img, debug_size)
        return ret, error

    def _plot_points(self, previous_points, current_points, save_img, size):
        img = np.zeros((size, size, 3), dtype=np.uint8)
        select = np.all((previous_points >= -size // 2) & (previous_points < size // 2), axis=0)
        previous_points = previous_points[:, select] + size // 2
        select = np.all((current_points >= -size // 2) & (current_points < size // 2), axis=0)
        current_points = current_points[:, select] + size // 2
        previous_points = np.array(previous_points, dtype=np.int32)
        current_points = np.array(current_points, dtype=np.int32)
        previous_color = np.array([0, 0, 255])
        current_color = np.array([255, 0, 0])
        img[previous_points[1], previous_points[0]] = previous_color
        img[current_points[1], current_points[0]] = current_color
        cv2.putText(img, f"T: {self._T}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        cv2.putText(img, f"Yaw: {self.rotation_as_euler}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 1)
        if save_img:
            cv2.imwrite("icpm_debug.png", img)
        else:
            cv2.imshow("ICPM Debug", img)

    @property
    def rotation(self):
        """
        结果的旋转矩阵(2x2) / rad
        """
        return self._R

    @property
    def translation(self):
        """
        结果的平移矩阵(2x1) / pixel
        """
        return self._T

    @property
    def rotation_as_euler(self) -> float:
        """
        结果的欧拉角(yaw) / deg
        """
        assert self._R is not None, "ICPM not run"
        return np.degrees(np.arcsin(self._R[0, 1]))
