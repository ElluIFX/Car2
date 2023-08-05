"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import bisect
import math
import pathlib
import sys
from time import perf_counter
from typing import Generator, Tuple
from loguru import logger

import matplotlib.pyplot as plt
import numpy as np

sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

k = 0.5  # control gain
Kp = 12  # speed proportional gain
dt = 0.1  # [s] time difference
L = 0.163  # [m] Wheel base of vehicle
max_steer = np.radians(35.0)  # [rad] max steering angle

show_animation = True

GOAL_DIS = 0.2  # goal distance
STOP_SPEED = 0.05  # stop speed

DT = 0.1  # [s] time tick

# Vehicle parameters
LENGTH = 4.5 / 10  # [m]
WIDTH = 2.0 / 10  # [m]
BACKTOWHEEL = 1.0 / 10  # [m]
WHEEL_LEN = 0.3 / 10  # [m]
WHEEL_WIDTH = 0.2 / 10  # [m]
TREAD = 0.7 / 10  # [m]
WB = 0.163  # [m]


class CubicSpline1D:
    """
    1D Cubic Spline class

    Parameters
    ----------
    x : list
        x coordinates for data points. This x coordinates must be
        sorted
        in ascending order.
    y : list
        y coordinates for data points

    Examples
    --------
    You can interpolate 1D data points.

    >>> import numpy as np
    >>> import matplotlib.pyplot as plt
    >>> x = np.arange(5)
    >>> y = [1.7, -6, 5, 6.5, 0.0]
    >>> sp = CubicSpline1D(x, y)
    >>> xi = np.linspace(0.0, 5.0)
    >>> yi = [sp.calc_position(x) for x in xi]
    >>> plt.plot(x, y, "xb", label="Data points")
    >>> plt.plot(xi, yi , "r", label="Cubic spline interpolation")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.show()

    .. image:: cubic_spline_1d.png

    """

    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        """
        Calc `y` position for given `x`.

        if `x` is outside the data point's `x` range, return None.

        Returns
        -------
        y : float
            y position for given x.
        """
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + self.c[i] * dx**2.0 + self.d[i] * dx**3.0

        return position

    def calc_first_derivative(self, x):
        """
        Calc first derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        dy : float
            first derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2.0
        return dy

    def calc_second_derivative(self, x):
        """
        Calc second derivative at given x.

        if x is outside the input x, return None

        Returns
        -------
        ddy : float
            second derivative for given x.
        """

        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1] - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    """
    Cubic CubicSpline2D class

    Parameters
    ----------
    x : list
        x coordinates for data points.
    y : list
        y coordinates for data points.

    Examples
    --------
    You can interpolate a 2D data points.

    >>> import matplotlib.pyplot as plt
    >>> x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    >>> y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    >>> ds = 0.1  # [m] distance of each interpolated points
    >>> sp = CubicSpline2D(x, y)
    >>> s = np.arange(0, sp.s[-1], ds)
    >>> rx, ry, ryaw, rk = [], [], [], []
    >>> for i_s in s:
    ...     ix, iy = sp.calc_position(i_s)
    ...     rx.append(ix)
    ...     ry.append(iy)
    ...     ryaw.append(sp.calc_yaw(i_s))
    ...     rk.append(sp.calc_curvature(i_s))
    >>> plt.subplots(1)
    >>> plt.plot(x, y, "xb", label="Data points")
    >>> plt.plot(rx, ry, "-r", label="Cubic spline path")
    >>> plt.grid(True)
    >>> plt.axis("equal")
    >>> plt.xlabel("x[m]")
    >>> plt.ylabel("y[m]")
    >>> plt.legend()
    >>> plt.show()

    .. image:: cubic_spline_2d_path.png

    >>> plt.subplots(1)
    >>> plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.xlabel("line length[m]")
    >>> plt.ylabel("yaw angle[deg]")

    .. image:: cubic_spline_2d_yaw.png

    >>> plt.subplots(1)
    >>> plt.plot(s, rk, "-r", label="curvature")
    >>> plt.grid(True)
    >>> plt.legend()
    >>> plt.xlabel("line length[m]")
    >>> plt.ylabel("curvature [1/m]")

    .. image:: cubic_spline_2d_curvature.png
    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        x : float
            x position for given s.
        y : float
            y position for given s.
        """
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        k : float
            curvature for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx**2 + dy**2) ** (3 / 2))
        return k

    def calc_yaw(self, s):
        """
        calc yaw

        Parameters
        ----------
        s : float
            distance from the start point. if `s` is outside the data point's
            range, return None.

        Returns
        -------
        yaw : float
            yaw angle (tangent vector) for given s.
        """
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = CubicSpline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover
    outline = np.array(
        [
            [-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
            [WIDTH / 2, WIDTH / 2, -WIDTH / 2, -WIDTH / 2, WIDTH / 2],
        ]
    )

    fr_wheel = np.array(
        [
            [WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
            [
                -WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                WHEEL_WIDTH - TREAD,
                -WHEEL_WIDTH - TREAD,
            ],
        ]
    )

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2), -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle


def get_map_course(dl):
    ax = [1.35, 1.7, 1.7, 1.7, 2.3, 2.6, 4]
    ay = [0.25, 1.1, 1.6, 1.9, 2.3, 2.3, 2.3]
    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck


def get_map_course_2(dl):
    ax = [2.6, 3.1, 3.3, 3.3, 3, 1.35]
    ay = [2.3 + 0.05, 2.3 + 0.05, 1.9, 1.2, 0.6, 0.25]
    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck


def get_map_course_3(dl):
    ax = [
        1.5, 1.8864, 3.1968, 4.008, 4.3008, 4.5408, 4.651199999999999, 4.656, 4.651199999999999, 4.3056, 3.7056
    ]
    ay = [
        0.25, 0.8086642599277978, 0.30806257521058966, 0.25511432009626955, 0.2647412755716005, 0.6161251504211793, 1.0204572803850782, 1.6125150421179302, 1.9061371841155235, 2.2575210589651022, 2.2623345367027676
    ]
    cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=dl)
    return cx, cy, cyaw, ck


def main():
    """Plot an example of Stanley steering control on a cubic spline."""
    #  target course
    ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    # cx, cy, cyaw, ck, s = calc_spline_course(ax, ay, ds=0.1)
    cx, cy, cyaw, ck = get_map_course(0.01)

    target_speed = 0.2

    max_simulation_time = 100.0

    # Initial state
    # state = State(x=-0.0, y=5.0, yaw=np.radians(20.0), v=0.0)
    state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    while max_simulation_time >= time and last_idx > target_idx:
        ai = pid_control(target_speed, state.v)
        di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        state.update(ai, di)

        time += dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        print(f"state.x: {state.x:.2f}, state.y: {state.y:.2f}, state.yaw: {state.yaw:.2f}, state.v: {state.v:.2f}")

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                "key_release_event", lambda event: [exit(0) if event.key == "escape" else None]
            )
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert last_idx >= target_idx, "Cannot reach goal"

    if show_animation:  # pragma: no cover
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)

        plt.subplots(1)
        plt.plot(t, [iv * 3.6 for iv in v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        plt.show()


def check_goal(state, goal, tind, nind):
    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)
    isgoal = d <= GOAL_DIS

    if abs(tind - nind) >= 5:
        isgoal = False
    return isgoal


def pi_2_pi(angle):
    while angle > math.pi:
        angle = angle - 2.0 * math.pi

    while angle < -math.pi:
        angle = angle + 2.0 * math.pi

    return angle


def calc_speed_profile(cx, cy, cyaw, target_speed):
    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = -target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile


def smooth_yaw(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


class Controller(object):
    def __init__(self, cx, cy, cyaw, ck, target_speed, dl=0.01, initial_state=None) -> None:
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.dl = dl
        self.target_speed = target_speed
        logger.debug(f"len(cx): {len(cx)}, len(cy): {len(cy)}, len(cyaw): {len(cyaw)}, len(ck): {len(ck)}")
        self.sp = calc_speed_profile(cx, cy, cyaw, target_speed)
        self.state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0) if initial_state is None else initial_state
        self.goal = [cx[-1], cy[-1]]
        if self.state.yaw - cyaw[0] >= math.pi:
            self.state.yaw -= math.pi * 2.0
        elif self.state.yaw - cyaw[0] <= -math.pi:
            self.state.yaw += math.pi * 2.0
        self.time = 0.0
        self.target_idx, _ = calc_target_index(self.state, cx, cy)
        self.odelta, self.oa = None, None
        self.last_update = 0.0
        self.paused = False

    def update_state(self, x, y, yaw, v):
        self.state.x = x
        self.state.y = y
        self.state.yaw = yaw
        self.state.v = v

    def check_goal(self, goal):
        dx = self.state.x - goal[0]
        dy = self.state.y - goal[1]
        d = math.hypot(dx, dy)

        isgoal = d <= GOAL_DIS
        isstop = abs(self.state.v) <= STOP_SPEED
        return isgoal and isstop

    def pause(self):
        self.paused = True

    def get_speed(self):
        return self.sp[self.target_idx]

    def iter_output(self) -> Generator[Tuple[float, float], None, None]:
        """Generates the next output of the controller.

        Yields:
            di: steering angle
            ai: acceleration
        """
        x = [self.state.x]
        y = [self.state.y]
        yaw = [self.state.yaw]
        v = [self.state.v]
        t = [0.0]
        d = [0.0]
        a = [0.0]
        self.last_update = perf_counter()
        while True:
            if self.paused:
                self.paused = False
                self.last_update = perf_counter()
            # t0 = perf_counter()
            # xref, self.target_ind, dref = calc_ref_trajectory(
            #     self.state, self.cx, self.cy, self.cyaw, self.ck, self.sp, self.dl, self.target_ind
            # )
            # x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]  # current state
            # self.oa, self.odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(xref, x0, dref, self.oa, self.odelta)
            # di, ai = 0.0, 0.0
            # if self.odelta is not None:
            #     di, ai = self.odelta[0], self.oa[0]
            # target_speed = self.sp[self.target_idx]
            ai = pid_control(self.target_speed, self.state.v)
            di, self.target_idx = stanley_control(self.state, self.cx, self.cy, self.cyaw, self.target_idx)
            self.time = self.time + DT

            x.append(self.state.x)
            y.append(self.state.y)
            yaw.append(self.state.yaw)
            v.append(self.state.v)
            t.append(self.time)
            d.append(di)
            a.append(ai)
            if check_goal(self.state, self.goal, self.target_idx, len(self.cx)):
                print("Goal")
                break

            if show_animation:  # pragma: no cover
                plt.cla()
                plt.plot(self.cx, self.cy, "-r", label="course")
                plt.plot(x, y, "ob", label="trajectory")
                plt.plot(self.cx[self.target_idx], self.cy[self.target_idx], "xg", label="target")
                plot_car(self.state.x, self.state.y, self.state.yaw, steer=di)
                plt.axis("equal")
                plt.grid(True)
                plt.savefig("mpst.png")
            yield di, ai
            # t1 = perf_counter()
            # print(f"Time taken: {t1 - t0:.6f}s")

    def a_to_v(self, a, v):
        return v + a * DT


if __name__ == "__main__":
    main()
