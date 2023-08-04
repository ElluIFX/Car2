import os
import sys

os.chdir(os.path.dirname(os.path.abspath(__file__)))
from PyQt5.QtCore import QLibraryInfo

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(QLibraryInfo.PluginsPath)

"""Fix error"""

sys.path.append(os.path.dirname(__file__))
PATH = os.path.dirname(os.path.abspath(__file__))

"""Add parent directory to path"""

from PyQt5.QtCore import (
    QCoreApplication,
    QDate,
    QDateTime,
    QLocale,
    QMetaObject,
    QObject,
    QPoint,
    QRect,
    QSize,
    Qt,
    QThread,
    QTime,
    QTimer,
    QUrl,
)
from PyQt5.QtCore import pyqtSignal as Signal
from PyQt5.QtCore import pyqtSlot as Slot
from PyQt5.QtGui import (
    QBrush,
    QColor,
    QConicalGradient,
    QCursor,
    QFont,
    QFontDatabase,
    QGradient,
    QIcon,
    QImage,
    QKeySequence,
    QLinearGradient,
    QPainter,
    QPalette,
    QPixmap,
    QRadialGradient,
    QTransform,
)
from PyQt5.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QProgressBar,
    QProgressDialog,
    QScrollArea,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

"""
pyside imports
"""
import os
import random
import shutil
import struct
import sys
import time
import warnings
from threading import Event

import cv2
import numpy as np
import qdarktheme
from fire_vision import Detector
from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from loguru import logger
from map_ui import Ui_MainWindow
from route import get_route
from SC import Controller, State

speed = 0.0

map_img = cv2.imread(os.path.join(PATH, "map1.jpg"))
fc = FC_Controller()
fc.start_listen_serial("/dev/ttyS6", print_state=True)
fc.wait_for_connection()
fc.motor_reset(fc.MOTOR_L | fc.MOTOR_R)
fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
radar = LD_Radar()
radar.debug = False
radar.start(subtask_skip=4)
radar.start_resolve_pose(800, 0.7, 0.3, rotation_adapt=True)
detector = Detector(fc)
base_x = 0
base_y = 0

pulse_list = [1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300]
deg_list = [35, 31, 24, 16, 7, 0, -6, -9, -13, -19, -20.5, -22, -23.7, -24]
rad_list = np.deg2rad(deg_list)
max_angle_r = deg_list[0]
max_angle_l = deg_list[-1]
max_angle_r_rad = rad_list[0]
max_angle_l_rad = rad_list[-1]

dl = 0.1
target_speed = 0.24
DT = 0.1


def get_pulse_from_deg(deg):
    if deg >= deg_list[0]:
        return pulse_list[0]
    if deg <= deg_list[-1]:
        return pulse_list[-1]
    for i in range(len(deg_list) - 1):
        if deg_list[i + 1] <= deg <= deg_list[i]:
            return np.interp(deg, [deg_list[i + 1], deg_list[i]], [pulse_list[i + 1], pulse_list[i]])


def motor_lr(vx: float, vz: float):
    Axle_spacing = 0.146  # 小车前后轴轴距 单位：m
    Wheel_spacing = 0.163  # 主动轮轮距 单位：m

    # 对于阿克曼小车vz代表右前轮转向角度
    R = Axle_spacing / np.tan(vz) - 0.5 * Wheel_spacing
    # 转弯半径 单位：m
    # 前轮转向角度限幅(舵机控制前轮转向角度)，单位：rad
    if vz > max_angle_r_rad:
        vz = max_angle_r_rad
    elif vz < max_angle_l_rad:
        vz = max_angle_l_rad
    # 运动学逆解
    if vz != 0:
        vl = vx * (R - 0.5 * Wheel_spacing) / R
        vr = vx * (R + 0.5 * Wheel_spacing) / R
    else:
        vl = vx
        vr = vx
    return vl, vr


def update_steer_and_speed(steer_rad: float, speed_mps: float):
    WHEEL_PERIMETER = 0.21049
    rpm = lambda speed_mps: speed_mps / WHEEL_PERIMETER * 60
    steer_deg = np.rad2deg(-steer_rad)
    pulse = get_pulse_from_deg(steer_deg)
    fc.set_steer_and_speed(pulse, rpm(speed_mps))
    # speed_l, speed_r = motor_lr(speed_mps, steer_rad)
    # fc.set_steer_and_two_speed(pulse, rpm(speed_l), rpm(speed_r))


def get_xy_yaw():
    x2_cm, y2_cm, yaw2 = radar.rt_pose
    return -y2_cm / 100, x2_cm / 100, np.deg2rad(-yaw2 + 90)


def calibrate():
    global base_x, base_y
    x, y, _ = get_xy_yaw()
    base_x = x - 1.35
    base_y = y - 0.25


def get_xyyaw_relative():
    x, y, yaw = get_xy_yaw()
    return x - base_x, y - base_y, yaw


def set_color(widget, rgb):
    color = f"rgb({rgb[0]},{rgb[1]},{rgb[2]})" if isinstance(rgb, tuple) else rgb
    widget.setStyleSheet(f"color: {color}")


def set_bar_color(widget, rgb):
    color = f"rgb({rgb[0]},{rgb[1]},{rgb[2]})" if isinstance(rgb, tuple) else rgb
    widget.setStyleSheet("QProgressBar::chunk " + "{" + f"background-color: {color};" + "}")


update_steer_and_speed(0, 0)
# time.sleep(2)
# calibrate()


def update_state(mpst, vel=None):
    x, y, yaw = get_xyyaw_relative()
    # v = (fc.state.motor_l_speed_mps.value + fc.state.motor_r_speed_mps.value) / 2 if vel is None else vel
    v = speed
    logger.debug(f"x: {x}, y: {y}, yaw: {yaw}, v: {v}")
    mpst.update_state(x, y, yaw, v)


class MySignal(QObject):
    def __init__(self, parent=None):
        super(MySignal, self).__init__(parent)

    image_signal = Signal(np.ndarray)
    key_event_signal = Signal(int)
    set_position_signal = Signal(float, float, float)
    reset_position_signal = Signal()
    takeoff_signal = Signal()


sig = MySignal()


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setupUi(self)
        self.init_widgets()
        self.init_timers()
        self.init_threads()
        self.init_signals()
        self.setGeometry(0, 0, 1024, 700)
        self.misThread.start()
        self.image_temp = None
        self.map_temp = map_img.copy()
        self.show_image(self.map_temp)
        self.last_px = None
        self.last_py = None

    def init_timers(self):
        pass

    @Slot()
    def on_pushButton_clicked(self):
        sig.takeoff_signal.emit()

    def init_threads(self):
        self.misThread = QThread()
        self.misWorker = MissionThread()
        self.misWorker.moveToThread(self.misThread)
        self.misThread.started.connect(self.misWorker.run)

    def init_signals(self):
        sig.image_signal.connect(self.show_image)
        sig.set_position_signal.connect(self.set_position)
        sig.reset_position_signal.connect(self.reset_position)

    def init_widgets(self):
        pass

    def set_position(self, x, y, dist):
        self.lcdX.display(f"{x:.2f}")
        self.lcdY.display(f"{y:.2f}")
        self.lcdDist.display(f"{dist:.1f}")
        self.draw_position(x, y)

    def reset_position(self):
        self.last_px = None
        self.last_py = None
        self.map_temp = map_img.copy()
        self.show_image(self.map_temp)

    def draw_position(self, x, y):
        width = self.map_temp.shape[1]
        height = self.map_temp.shape[0]
        MAP_W = 48
        MAP_H = 40  # left-down corner is (0, 0)
        x = round(x / MAP_W * width)
        y = height - round(y / MAP_H * height)
        if self.last_px is not None and self.last_py is not None:
            cv2.line(self.map_temp, (self.last_px, self.last_py), (x, y), (0, 0, 255), 3)
        else:
            cv2.circle(self.map_temp, (x, y), 3, (0, 0, 255), -1)
        img = self.map_temp.copy()
        cv2.line(img, (x - 40, y), (x + 40, y), (255, 0, 0), 2)
        cv2.line(img, (x, y - 40), (x, y + 40), (255, 0, 0), 2)
        self.last_px = x
        self.last_py = y
        self.show_image(img)

    def show_image(self, image: np.ndarray):
        self._image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.pixmap = QPixmap.fromImage(
            QImage(
                self._image,
                self._image.shape[1],
                self._image.shape[0],
                QImage.Format.Format_RGB888,
            )
        ).scaled(self.labelMap.width() - 5, self.labelMap.height() - 5, Qt.KeepAspectRatio)
        self.labelMap.setPixmap(self.pixmap)
        self.image_temp = self._image

    def resizeEvent(self, event) -> None:
        if self.image_temp is not None:
            self.pixmap = QPixmap.fromImage(
                QImage(
                    self.image_temp,
                    self.image_temp.shape[1],
                    self.image_temp.shape[0],
                    QImage.Format.Format_RGB888,
                )
            ).scaled(self.labelMap.width() - 5, self.labelMap.height() - 5, Qt.KeepAspectRatio)
            self.labelMap.setPixmap(self.pixmap)
        return super().resizeEvent(event)

    def showEvent(self, a0) -> None:
        self.showFullScreen()
        return super().showEvent(a0)

    # F11 全屏
    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        return super().keyPressEvent(event)

    def closeEvent(self, event) -> None:
        self.misThread.quit()
        return super().closeEvent(event)


class MissionThread(QObject):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.takeoff_flag = False
        sig.takeoff_signal.connect(self.takeoff)
        fc.register_wireless_callback(self.callback)
        self.start_event = Event()

    def show_image(self, image: np.ndarray):
        self._image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        sig.image_signal.emit(self._image)

    def run(self):
        logger.info("Mission Thread Started")
        while True:
            self.start_event.wait()
            calibrate()
            logger.info(f"Mission start, goto {self.fire_x}, {self.fire_y}")
            try:
                enter_p, leave_p, side = get_route(self.fire_x, self.fire_y, dl=dl)
                last_update = time.perf_counter()
                vel = target_speed
                cx, cy, cyaw, ck = enter_p
                x, y, yaw = get_xyyaw_relative()
                initial_state = State(x, y, yaw, 0)
                mpst = Controller(cx, cy, cyaw, ck, target_speed, dl, initial_state)
                fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
                update_state(mpst)
                for steer, acc in mpst.iter_output():
                    vel = mpst.get_speed()
                    update_steer_and_speed(steer, vel)
                    logger.debug(f"steer: {steer}, acc: {acc}, vel: {vel}")
                    while time.perf_counter() - last_update < DT:
                        time.sleep(0.02)
                    last_update = time.perf_counter()
                    update_state(mpst)

                update_steer_and_speed(0, 0)
                fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.BREAK)
                detector.x_base = getattr(detector, f"x_base_{side}")
                detector.go_to_base()
                time.sleep(1)
                detector.process()
                detector.x_base = detector.x_base_m
                detector.go_to_base()

                last_update = time.perf_counter()
                vel = target_speed
                cx, cy, cyaw, ck = leave_p
                x, y, yaw = get_xyyaw_relative()
                initial_state = State(x, y, yaw, 0)
                mpst = Controller(cx, cy, cyaw, ck, target_speed, dl, initial_state)
                fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.SPD_CTRL)
                update_state(mpst)
                for steer, acc in mpst.iter_output():
                    vel = mpst.get_speed()
                    update_steer_and_speed(steer, vel)
                    logger.debug(f"steer: {steer}, acc: {acc}, vel: {vel}")
                    while time.perf_counter() - last_update < DT:
                        time.sleep(0.02)
                    last_update = time.perf_counter()
                    update_state(mpst)

                update_steer_and_speed(0, 0)
                fc.set_motor_mode(fc.MOTOR_L | fc.MOTOR_R, fc.BREAK)
            finally:
                update_steer_and_speed(0, 0)
                time.sleep(1)
            while self.start_event.is_set():
                time.sleep(1)

    def takeoff(self):
        self.takeoff_flag = True

    def callback(self, data: bytes):
        try:
            c, px, py, dist, f, firex, firey = struct.unpack("<BhhHBhh", data)
            sig.set_position_signal.emit(-py / 100 + 3.5, px / 100 + 3.5, dist / 100)
            if not c:
                sig.reset_position_signal.emit()
                if self.takeoff_flag:
                    fc.send_to_wireless(b"\x01")
            else:
                self.takeoff_flag = False
            if f:
                self.fire_x = -firey / 1000 + 0.35
                self.fire_y = firex / 1000 + 0.35
                self.start_event.set()
            else:
                if self.start_event.is_set():
                    self.start_event.clear()
        except Exception as e:
            logger.exception(e)


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)  # type: ignore
    app.setStyleSheet(qdarktheme.load_stylesheet(theme="light"))
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
