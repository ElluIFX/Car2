import os
import sys

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
from FlightController import FC_Controller
from FlightController.Components import LD_Radar
from loguru import logger
from map_ui import Ui_MainWindow

map_img = cv2.imread(os.path.join(PATH, "map1.jpg"))
fc = FC_Controller()
# fc.start_listen_serial("COM1", print_state=True)
# radar = LD_Radar()
# radar.start()
# radar.start_resolve_pose()


def set_color(widget, rgb):
    color = f"rgb({rgb[0]},{rgb[1]},{rgb[2]})" if isinstance(rgb, tuple) else rgb
    widget.setStyleSheet(f"color: {color}")


def set_bar_color(widget, rgb):
    color = f"rgb({rgb[0]},{rgb[1]},{rgb[2]})" if isinstance(rgb, tuple) else rgb
    widget.setStyleSheet("QProgressBar::chunk " + "{" + f"background-color: {color};" + "}")


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
        self.start_event.wait()
        logger.info(f"Mission start, goto {self.fire_x}, {self.fire_y}")

    def takeoff(self):
        self.takeoff_flag = True

    def callback(self, data: bytes):
        c, px, py, dist, f, firex, firey = struct.unpack("<BhhHBhh", data)
        sig.set_position_signal.emit(-py / 100 + 3.5, px / 100 + 3.5, dist / 100)
        if not c:
            sig.reset_position_signal.emit()
            if self.takeoff_flag:
                fc.send_to_wireless(b"\x01")
        else:
            self.takeoff_flag = False
        if f:
            self.fire_x = -firey / 100 + 3.5
            self.fire_y = firex / 100 + 3.5
            self.start_event.set()


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)  # type: ignore
    app.setStyleSheet(qdarktheme.load_stylesheet(theme="light"))
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
