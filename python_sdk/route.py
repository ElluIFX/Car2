import numpy as np
from SC import calc_spline_course

X = {
    1.0: ((3,), (4, 5, 6, 7, 8, 9, 10)),
    2.7: ((1, 2, 3, 4, 5), (6, 9, 10, 11, 12)),
    4.1: ((1, 2, 3, 4, 5, 6, 7, 8, 9, 11), (10, 12)),
    1.8: ((1, 2, 3, 4), (5, 6, 7, 8, 9, 10, 11, 12)),
    3.2: ((1, 2, 3, 4, 5, 6, 7, 8), (9, 10, 11, 12)),
}
Y = {
    3.05: ((1,), (2, 3, 4, 7, 8, 11, 12)),
    2.2: ((1, 2, 5, 6, 9, 10), (3, 4, 7, 8, 11, 12)),
    1.55: ((1, 2, 5, 6, 9, 10, 7), (8,)),
}
MATCH = {1: "x", 2: "x", 3: "y", 4: "y", 5: "y", 6: "y", 7: "x", 8: "x", 9: "y", 10: "y", 11: "y", 12: "y"}
SIDE = {
    1: "l",
    2: "r",
}


def get_id(x, y):
    possible = [0 for _ in range(12)]
    for key, value in X.items():
        for i in value[0 if x <= key else 1]:
            possible[i - 1] += 1
    for key, value in Y.items():
        for i in value[0 if y >= key else 1]:
            possible[i - 1] += 1
    return np.argmax(possible) + 1


enter_routes = {
    1: (
        [1.35, 1.6511999999999998, 1.8816, 2.1071999999999997, 2.1216, 1.7808, 1.584, 0.6],
        [
            0.25,
            1.0974729241877257,
            2.2478941034897715,
            2.7918170878459687,
            3.3838748495788207,
            3.643802647412756,
            3.6726835138387486,
            3.643802647412756,
        ],
    ),
    2: (
        [1.35, 1.7087999999999999, 1.8288, 1.6320000000000001, 1.3488, 0.7776, 0.624],
        [
            0.25,
            1.2611311672683514,
            2.267148014440433,
            2.4259927797833933,
            2.4693140794223827,
            2.459687123947052,
            2.4693140794223827,
        ],
    ),
}

leave_routes = {
    1: (
        [0.6, 0.3216, 0.288, 0.2976, 0.2976, 0.6911999999999999, 1.0655999999999999, 1.3344],
        [
            3.643802647412756,
            3.4320096269554754,
            3.210589651022864,
            1.3333333333333333,
            1.2226233453670277,
            0.8953068592057761,
            0.6113116726835138,
            0.2743682310469314,
        ],
    ),
    2: (
        [0.624, 0.2736, 0.3696, 0.792, 1.3104],
        [2.4693140794223827, 1.8050541516245486, 1.1696750902527075, 0.8664259927797834, 0.27918170878459686],
    ),
}


def get_route(x, y, dl=0.1):
    id = get_id(x, y)
    print(f"ID: {id}")
    enter = enter_routes[id]
    leave = leave_routes[id]
    enter_params = [[], [], [], []]
    leave_params = [[], [], [], []]
    cx, cy, cyaw, ck, _ = calc_spline_course(enter[0], enter[1], ds=dl)
    split_idx = len(cx) - 1
    m = MATCH[id]
    if m == "x":
        first_state = x < cx[split_idx]
        while (first_state) == (x < cx[split_idx]):
            split_idx -= 1
    else:
        first_state = y < cy[split_idx]
        while (first_state) == (y < cy[split_idx]):
            split_idx -= 1
    enter_params[0].extend(cx[:split_idx])
    enter_params[1].extend(cy[:split_idx])
    enter_params[2].extend(cyaw[:split_idx])
    enter_params[3].extend(ck[:split_idx])
    leave_params[0].extend(cx[split_idx:])
    leave_params[1].extend(cy[split_idx:])
    leave_params[2].extend(cyaw[split_idx:])
    leave_params[3].extend(ck[split_idx:])
    cx, cy, cyaw, ck, _ = calc_spline_course(leave[0], leave[1], ds=dl)
    leave_params[0].extend(cx)
    leave_params[1].extend(cy)
    leave_params[2].extend(cyaw)
    leave_params[3].extend(ck)
    return enter_params, leave_params, SIDE[id]
