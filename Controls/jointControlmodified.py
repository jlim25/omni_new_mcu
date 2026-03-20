# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import os
import math

import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar

import ServoControl


DEFAULT_MOVE_MS = 1000
WINDOW_W = 1450
WINDOW_H = 860
MAX_SERVO_ID = 254

BASE_DIR = os.path.dirname(os.path.abspath(__file__))


JOINT_TYPES = {
    "none": {
        "default_limits": (0.0, 0.0),
        "axis": "z",
    },
    "swivel": {
        "default_limits": (-math.pi, math.pi),
        "axis": "z",
    },
    "rotation": {
        "default_limits": (-math.pi / 2, math.pi / 2),
        "axis": "y",
    }
}


LINK_PRESETS_IN = [4, 6, 8]
LINK_PRESETS_M = {
    "4 in": 0.1016,
    "6 in": 0.1524,
    "8 in": 0.2032,
}


SERVO_CAL = {
    "J1": {
        "joint_min": -180.0,
        "joint_max": 180.0,
        "servo_min": 0.0,
        "servo_max": 360.0,
        "invert": False,
        "gear_ratio": 2.0,
    },
    "J2": {
        "joint_min": -120.0,
        "joint_max": 120.0,
        "servo_min": 0.0,
        "servo_max": 240.0,
        "invert": True,
        "gear_ratio": 1.0,
    },
    "J3": {
        "joint_min": -120.0,
        "joint_max": 120.0,
        "servo_min": 0.0,
        "servo_max": 240.0,
        "invert": False,
        "gear_ratio": 1.0,
    },
    "J4": {
        "joint_min": -120.0,
        "joint_max": 120.0,
        "servo_min": 0.0,
        "servo_max": 240.0,
        "invert": True,
        "gear_ratio": 1.0,
    },
    "J5": {
        "joint_min": -120.0,
        "joint_max": 120.0,
        "servo_min": 0.0,
        "servo_max": 240.0,
        "invert": True,
        "gear_ratio": 1.0,
    },
    "J6": {
        "joint_min": -120.0,
        "joint_max": 120.0,
        "servo_min": 0.0,
        "servo_max": 240.0,
        "invert": False,
        "gear_ratio": 1.0,
    },
    "J7": {
        "joint_min": -120.0,
        "joint_max": 120.0,
        "servo_min": 0.0,
        "servo_max": 240.0,
        "invert": False,
        "gear_ratio": 1.0,
    },
    "J8": {
        "joint_min": -120.0,
        "joint_max": 120.0,
        "servo_min": 0.0,
        "servo_max": 240.0,
        "invert": False,
        "gear_ratio": 1.0,
    },
    "GRIP": {
        "joint_min": -180.0,
        "joint_max": 180.0,
        "servo_min": 0.0,
        "servo_max": 360.0,
        "invert": False,
        "gear_ratio": 1.0,
    },
}



def joint_to_servo_deg(module):
    joint_name = module["name"]
    joint_rad = module["q"]

    if joint_name not in SERVO_CAL:
        return None

    c = dict(SERVO_CAL[joint_name])

    if module["type"] == "swivel":
        c["joint_min"] = -180.0
        c["joint_max"] = 180.0
        c["servo_min"] = 0.0
        c["servo_max"] = 360.0

    joint_deg = math.degrees(joint_rad)

    if c.get("invert", False):
        joint_deg = -joint_deg

    gear_ratio = float(c.get("gear_ratio", 1.0))
    if abs(gear_ratio) < 1e-9:
        gear_ratio = 1.0

    servo_side_joint_deg = joint_deg / gear_ratio
    servo_side_joint_min = c["joint_min"] / gear_ratio
    servo_side_joint_max = c["joint_max"] / gear_ratio

    servo_deg = map_range(
        servo_side_joint_deg,
        servo_side_joint_min, servo_side_joint_max,
        c["servo_min"], c["servo_max"]
    )

    servo_deg += float(module.get("servo_offset_deg", 0.0))
    return clamp(servo_deg, c["servo_min"], c["servo_max"])


def servo_deg_to_joint_rad(module, servo_deg):
    joint_name = module["name"]
    if joint_name not in SERVO_CAL:
        return None

    c = dict(SERVO_CAL[joint_name])

    if module["type"] == "swivel":
        c["joint_min"] = -180.0
        c["joint_max"] = 180.0
        c["servo_min"] = 0.0
        c["servo_max"] = 360.0

    servo_deg = float(servo_deg) - float(module.get("servo_offset_deg", 0.0))

    gear_ratio = float(c.get("gear_ratio", 1.0))
    if abs(gear_ratio) < 1e-9:
        gear_ratio = 1.0

    servo_side_joint_min = c["joint_min"] / gear_ratio
    servo_side_joint_max = c["joint_max"] / gear_ratio

    servo_side_joint_deg = map_range(
        servo_deg,
        c["servo_min"], c["servo_max"],
        servo_side_joint_min, servo_side_joint_max
    )

    joint_deg = servo_side_joint_deg * gear_ratio

    if c.get("invert", False):
        joint_deg = -joint_deg

    joint_rad = math.radians(joint_deg)

    if not module["fixed"] and module["type"] != "none":
        qmin, qmax = module["qlim"]
        joint_rad = clamp(joint_rad, qmin, qmax)

    return joint_rad





def default_servo_id_for_name(name):
    if name.startswith("J"):
        try:
            n = int(name[1:])
            return n if 1 <= n <= MAX_SERVO_ID else 0
        except Exception:
            return 0
    return 0


def make_default_modules(joint_count):
    modules = []

    if joint_count == 5:
        # =========================================================
        # CUSTOM 5-JOINT DEFAULT CONFIG
        # Edit this block however you want for your preferred setup.
        # =========================================================
        modules = [
            {
                "name": "J1",
                "type": "swivel",
                "axis": "z",
                "offset": np.array([0.0, 0.0, 0.0], dtype=float),
                "link": np.array([0.0, 0.0, 0.2032], dtype=float),
                "q": 0.0,
                "home_q": 0.0,
                "qlim": (-math.pi, math.pi),
                "fixed": False,
                "servo_id": 1,
                "servo_offset_deg": -37.8,
            },
            {
                "name": "J2",
                "type": "rotation",
                "axis": "y",
                "offset": np.array([0.0, 0.0, 0.0], dtype=float),
                "link": np.array([0.0, 0.0, 0.1524], dtype=float),
                "q": 0.0,
                "home_q": 0.0,
                "qlim": (-math.pi / 2, math.pi / 2),
                "fixed": False,
                "servo_id": 2,
                "servo_offset_deg": -12.72,
            },
            {
                "name": "J3",
                "type": "rotation",
                "axis": "y",
                "offset": np.array([0.0, 0.0, 0.0], dtype=float),
                "link": np.array([0.0, 0.0, 0.1016], dtype=float),
                "q": 0.0,
                "home_q": 0.0,
                "qlim": (-math.pi / 2, math.pi / 2),
                "fixed": False,
                "servo_id": 4,
                "servo_offset_deg": 5.52,
            },
            {
                "name": "J4",
                "type": "rotation",
                "axis": "y",
                "offset": np.array([0.0, 0.0, 0.0], dtype=float),
                "link": np.array([0.0, 0.0, 0.1016], dtype=float),
                "q": 0.0,
                "home_q": 0.0,
                "qlim": (-math.pi / 2, math.pi / 2),
                "fixed": False,
                "servo_id": 6,
                "servo_offset_deg": 30.7,
            },
            {
                "name": "J5",
                "type": "swivel",
                "axis": "y",
                "offset": np.array([0.0, 0.0, 0.0], dtype=float),
                "link": np.array([0.0, 0.0, 0.0], dtype=float),
                "q": 0.0,
                "home_q": 0.0,
                "qlim": (-math.pi, math.pi),
                "fixed": False,
                "servo_id": 5,
                "servo_offset_deg": -160,
            },
            {
                "name": "GRIP",
                "type": "swivel",
                "axis": "z",
                "offset": np.array([0.0, 0.0, 0.0], dtype=float),
                "link": np.array([0.0, 0.0, 0.0], dtype=float),
                "q": 0.0,
                "home_q": 0.0,
                "qlim": (-math.pi, math.pi),
                "fixed": True,
                "servo_id": 0,
                "servo_offset_deg": 0.0,
            }
        ]
        return modules

    if joint_count >= 1:
        modules.append({
            "name": "J1",
            "type": "swivel",
            "axis": "z",
            "offset": np.array([0.0, 0.0, 0.0], dtype=float),
            "link": np.array([0.0, 0.0, 0.0], dtype=float),
            "q": 0.0,
            "home_q": 0.0,
            "qlim": JOINT_TYPES["swivel"]["default_limits"],
            "fixed": False,
            "servo_id": 1,
            "servo_offset_deg": 0.0,
        })

    for i in range(2, joint_count + 1):
        modules.append({
            "name": f"J{i}",
            "type": "rotation",
            "axis": "y",
            "offset": np.array([0.0, 0.0, 0.0], dtype=float),
            "link": np.array([0.0, 0.0, 0.1524 if i < joint_count else 0.1016], dtype=float),
            "q": 0.0,
            "home_q": 0.0,
            "qlim": JOINT_TYPES["rotation"]["default_limits"],
            "fixed": False,
            "servo_id": i if i <= MAX_SERVO_ID else 0,
            "servo_offset_deg": 0.0,
        })

    modules.append({
        "name": "GRIP",
        "type": "swivel",
        "axis": "z",
        "offset": np.array([0.0, 0.0, 0.0], dtype=float),
        "link": np.array([0.0, 0.0, 0.0], dtype=float),
        "q": 0.0,
        "home_q": 0.0,
        "qlim": JOINT_TYPES["swivel"]["default_limits"],
        "fixed": True,
        "servo_id": 0,
        "servo_offset_deg": 0.0,
    })

    return modules


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def map_range(x, in_min, in_max, out_min, out_max):
    if abs(in_max - in_min) < 1e-9:
        return out_min
    return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min)


def joint_to_servo_deg(module):
    joint_name = module["name"]
    joint_rad = module["q"]

    if joint_name not in SERVO_CAL:
        return None

    c = dict(SERVO_CAL[joint_name])

    if module["type"] == "swivel":
        c["joint_min"] = -180.0
        c["joint_max"] = 180.0
        c["servo_min"] = 0.0
        c["servo_max"] = 360.0

    joint_deg = math.degrees(joint_rad)

    if c.get("invert", False):
        joint_deg = -joint_deg

    gear_ratio = float(c.get("gear_ratio", 1.0))
    if abs(gear_ratio) < 1e-9:
        gear_ratio = 1.0

    servo_side_joint_deg = joint_deg / gear_ratio
    servo_side_joint_min = c["joint_min"] / gear_ratio
    servo_side_joint_max = c["joint_max"] / gear_ratio

    servo_deg = map_range(
        servo_side_joint_deg,
        servo_side_joint_min, servo_side_joint_max,
        c["servo_min"], c["servo_max"]
    )

    servo_deg += float(module.get("servo_offset_deg", 0.0))
    return clamp(servo_deg, c["servo_min"], c["servo_max"])


def servo_deg_to_joint_rad(module, servo_deg):
    joint_name = module["name"]
    if joint_name not in SERVO_CAL:
        return None

    c = dict(SERVO_CAL[joint_name])

    if module["type"] == "swivel":
        c["joint_min"] = -180.0
        c["joint_max"] = 180.0
        c["servo_min"] = 0.0
        c["servo_max"] = 360.0

    servo_deg = float(servo_deg) - float(module.get("servo_offset_deg", 0.0))

    gear_ratio = float(c.get("gear_ratio", 1.0))
    if abs(gear_ratio) < 1e-9:
        gear_ratio = 1.0

    servo_side_joint_min = c["joint_min"] / gear_ratio
    servo_side_joint_max = c["joint_max"] / gear_ratio

    servo_side_joint_deg = map_range(
        servo_deg,
        c["servo_min"], c["servo_max"],
        servo_side_joint_min, servo_side_joint_max
    )

    joint_deg = servo_side_joint_deg * gear_ratio

    if c.get("invert", False):
        joint_deg = -joint_deg

    joint_rad = math.radians(joint_deg)

    if not module["fixed"] and module["type"] != "none":
        qmin, qmax = module["qlim"]
        joint_rad = clamp(joint_rad, qmin, qmax)

    return joint_rad


def servo_deg_to_uart_count(module, servo_deg):
    if module["type"] == "swivel":
        return int(round(clamp(servo_deg, 0.0, 360.0) * 1000.0 / 360.0))
    return int(round(clamp(servo_deg, 0.0, 240.0) * 1000.0 / 240.0))


def Tx(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def Rx(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s,  c, 0],
        [0, 0, 0, 1]
    ], dtype=float)


def Ry(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([
        [ c, 0, s, 0],
        [ 0, 1, 0, 0],
        [-s, 0, c, 0],
        [ 0, 0, 0, 1]
    ], dtype=float)


def Rz(q):
    c, s = math.cos(q), math.sin(q)
    return np.array([
        [c, -s, 0, 0],
        [s,  c, 0, 0],
        [0,  0, 1, 0],
        [0,  0, 0, 1]
    ], dtype=float)


def rot_from_axis(axis, q):
    axis = axis.lower()
    if axis == "x":
        return Rx(q)
    if axis == "y":
        return Ry(q)
    return Rz(q)


def nearest_link_preset_text(length_m):
    return min(LINK_PRESETS_M.keys(), key=lambda k: abs(LINK_PRESETS_M[k] - length_m))


def auto_fit_axes_cube(ax, X, Y, Z, scale=1.5, min_cube=0.4):
    x_min, x_max = float(np.min(X)), float(np.max(X))
    y_min, y_max = float(np.min(Y)), float(np.max(Y))
    z_min, z_max = float(np.min(Z)), float(np.max(Z))

    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    z_mid = 0.5 * (z_min + z_max)

    dx = x_max - x_min
    dy = y_max - y_min
    dz = z_max - z_min

    span = max(dx, dy, dz, min_cube)
    cube_side = span * scale
    half = 0.5 * cube_side

    z_low = z_mid - half
    z_high = z_mid + half

    if z_low < 0.0:
        z_high += -z_low
        z_low = 0.0

    ax.set_xlim([x_mid - half, x_mid + half])
    ax.set_ylim([y_mid - half, y_mid + half])
    ax.set_zlim([z_low, z_high])
    ax.set_box_aspect((1, 1, 1))


def forward_kin(modules):
    T = np.eye(4)

    chain_pts = [np.array([0.0, 0.0, 0.0], dtype=float)]
    joint_pts = [(0.0, 0.0, 0.0)]
    point_names = ["BASE"]

    frames = [{
        "name": "BASE",
        "origin": np.array([0.0, 0.0, 0.0], dtype=float),
        "x_axis": np.array([1.0, 0.0, 0.0], dtype=float),
        "y_axis": np.array([0.0, 1.0, 0.0], dtype=float),
        "z_axis": np.array([0.0, 0.0, 1.0], dtype=float),
    }]

    for m in modules:
        T = T @ Tx(*m["offset"])

        joint_origin = T[:3, 3].copy()
        R_before = T[:3, :3].copy()

        joint_pts.append((float(joint_origin[0]), float(joint_origin[1]), float(joint_origin[2])))
        point_names.append(m["name"])
        frames.append({
            "name": m["name"],
            "origin": joint_origin,
            "x_axis": R_before[:, 0].copy(),
            "y_axis": R_before[:, 1].copy(),
            "z_axis": R_before[:, 2].copy(),
        })

        q = 0.0 if m["fixed"] or m["type"] == "none" else m["q"]
        T = T @ rot_from_axis(m["axis"], q)
        T = T @ Tx(*m["link"])

        link_end = T[:3, 3].copy()
        chain_pts.append(link_end)

    X = np.array([p[0] for p in chain_pts], dtype=float)
    Y = np.array([p[1] for p in chain_pts], dtype=float)
    Z = np.array([p[2] for p in chain_pts], dtype=float)
    return X, Y, Z, joint_pts, point_names, frames


def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def get_end_effector_yaw(frames):
    if not frames:
        return 0.0
    ee = frames[-1]
    x_axis = ee["x_axis"]
    return math.atan2(float(x_axis[1]), float(x_axis[0]))


def create_plot():
    fig = plt.figure(figsize=(6.8, 5.6), facecolor="#0f172a")
    ax = fig.add_subplot(111, projection='3d')
    return fig, ax


def style_3d_axes(ax, show_plot_axes=True):
    bg = "#0b1220"
    pane = (0.11, 0.16, 0.25, 1.0)
    grid = (0.55, 0.65, 0.78, 0.18)
    text = "#e5e7eb"
    line = (0.65, 0.75, 0.85, 0.55)

    ax.set_facecolor(bg)
    ax.set_xlabel("X (m)", color=text)
    ax.set_ylabel("Y (m)", color=text)
    ax.set_zlabel("Z (m)", color=text)
    ax.tick_params(colors=text, grid_color=grid, grid_alpha=0.35)

    try:
        ax.xaxis.set_pane_color(pane)
        ax.yaxis.set_pane_color(pane)
        ax.zaxis.set_pane_color(pane)
    except Exception:
        pass

    try:
        ax.xaxis.line.set_color(line)
        ax.yaxis.line.set_color(line)
        ax.zaxis.line.set_color(line)
    except Exception:
        pass

    ax.grid(show_plot_axes, color=grid, linestyle='-', linewidth=0.6)

    if show_plot_axes:
        ax.set_axis_on()
    else:
        ax.set_axis_off()


def draw_frame_axes(ax, frames, show_axes_frames=True, axis_len=0.04):
    if not show_axes_frames:
        return

    try:
        xs = [f["origin"][0] for f in frames]
        ys = [f["origin"][1] for f in frames]
        zs = [f["origin"][2] for f in frames]
        span = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 0.2)
        axis_len = max(0.03, span * 0.08)
    except Exception:
        axis_len = 0.05

    for f in frames:
        o = f["origin"]
        x = f["x_axis"]
        y = f["y_axis"]
        z = f["z_axis"]

        ax.quiver(o[0], o[1], o[2], x[0], x[1], x[2], length=axis_len, normalize=True, color='#ef4444')
        ax.quiver(o[0], o[1], o[2], y[0], y[1], y[2], length=axis_len, normalize=True, color='#22c55e')
        ax.quiver(o[0], o[1], o[2], z[0], z[1], z[2], length=axis_len, normalize=True, color='#3b82f6')


def geometric_world_ik(modules, target_xyz, target_yaw, reach_tolerance=0.015, elbow_up=True):
    if len(modules) < 3:
        return False, "Need at least J1, J2, and J3 for geometric IK."

    if modules[0]["fixed"] or modules[0]["type"] == "none" or modules[0]["axis"].lower() != "z":
        return False, "Geometric IK expects J1 to be an active swivel about Z."

    pitch_ids = []
    for i in range(1, len(modules)):
        m = modules[i]
        if m["fixed"] or m["type"] == "none":
            continue
        if m["axis"].lower() == "y":
            pitch_ids.append(i)

    if len(pitch_ids) < 2:
        return False, "Geometric IK expects at least two active Y-axis joints after J1."

    base = modules[0]
    q1 = math.atan2(target_xyz[1], target_xyz[0])
    q1 = clamp(q1, base["qlim"][0], base["qlim"][1])

    r = math.hypot(target_xyz[0], target_xyz[1])
    z = target_xyz[2]

    base_height = float(base["link"][2] + base["offset"][2])
    pr = r
    pz = z - base_height

    l1 = float(np.linalg.norm(modules[pitch_ids[0]]["link"]))
    l2 = float(np.linalg.norm(modules[pitch_ids[1]]["link"]))

    extra_pitch_ids = pitch_ids[2:]
    extra_len = sum(float(np.linalg.norm(modules[j]["link"])) for j in extra_pitch_ids)

    wrist_r = pr
    wrist_z = pz

    if extra_len > 1e-9:
        wrist_r = pr - extra_len
        wrist_z = pz

    d = math.hypot(wrist_r, wrist_z)
    d = clamp(d, abs(l1 - l2) + 1e-6, l1 + l2 - 1e-6)

    cos_q3 = (d * d - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_q3 = clamp(cos_q3, -1.0, 1.0)

    q3_mag = math.acos(cos_q3)
    q3 = -q3_mag if elbow_up else q3_mag

    phi = math.atan2(wrist_z, wrist_r)
    psi = math.atan2(l2 * math.sin(q3), l1 + l2 * math.cos(q3))
    q2 = phi - psi

    candidate_q = [m["q"] for m in modules]
    candidate_q[0] = q1
    candidate_q[pitch_ids[0]] = q2
    candidate_q[pitch_ids[1]] = q3

    for j in extra_pitch_ids:
        candidate_q[j] = 0.0

    desired_pitch_sum = 0.0
    used_pitch_sum = candidate_q[pitch_ids[0]] + candidate_q[pitch_ids[1]]
    residual_pitch = desired_pitch_sum - used_pitch_sum

    if len(extra_pitch_ids) > 0:
        share = residual_pitch / len(extra_pitch_ids)
        for j in extra_pitch_ids:
            candidate_q[j] = clamp(share, modules[j]["qlim"][0], modules[j]["qlim"][1])

    yaw_candidates = []
    for i, m in enumerate(modules):
        if m["fixed"] or m["type"] == "none":
            continue
        if m["axis"].lower() == "z":
            yaw_candidates.append(i)

    if yaw_candidates:
        yaw_after_base = wrap_to_pi(target_yaw - q1)
        secondary = [j for j in yaw_candidates if j != 0]
        if secondary:
            share = yaw_after_base / len(secondary)
            for j in secondary:
                candidate_q[j] = clamp(share, modules[j]["qlim"][0], modules[j]["qlim"][1])

    for i, m in enumerate(modules):
        if m["fixed"] or m["type"] == "none":
            continue
        qmin, qmax = m["qlim"]
        candidate_q[i] = clamp(candidate_q[i], qmin, qmax)

    old_q = [m["q"] for m in modules]
    for i, q in enumerate(candidate_q):
        modules[i]["q"] = q

    X, Y, Z, pts, names, frames = forward_kin(modules)
    actual = np.array([X[-1], Y[-1], Z[-1]], dtype=float)
    pos_err = np.linalg.norm(target_xyz - actual)

    if pos_err > reach_tolerance:
        for i, q in enumerate(old_q):
            modules[i]["q"] = q
        return False, f"Target not reached. Error {pos_err:.3f} m exceeds tolerance {reach_tolerance:.3f} m."

    return True, f"Target reached. Error {pos_err:.3f} m."


def solve_world_geometric(modules, target_xyz, target_yaw, reach_tolerance=0.015):
    old_q = [m["q"] for m in modules]

    ok1, _ = geometric_world_ik(modules, target_xyz, target_yaw, reach_tolerance=reach_tolerance, elbow_up=True)
    if ok1:
        q_up = [m["q"] for m in modules]
        X, Y, Z, pts, names, frames = forward_kin(modules)
        err_up = np.linalg.norm(target_xyz - np.array([X[-1], Y[-1], Z[-1]], dtype=float))
    else:
        q_up = None
        err_up = 1e9

    for i, q in enumerate(old_q):
        modules[i]["q"] = q

    ok2, _ = geometric_world_ik(modules, target_xyz, target_yaw, reach_tolerance=reach_tolerance, elbow_up=False)
    if ok2:
        q_down = [m["q"] for m in modules]
        X, Y, Z, pts, names, frames = forward_kin(modules)
        err_down = np.linalg.norm(target_xyz - np.array([X[-1], Y[-1], Z[-1]], dtype=float))
    else:
        q_down = None
        err_down = 1e9

    if q_up is None and q_down is None:
        for i, q in enumerate(old_q):
            modules[i]["q"] = q
        return False, "Target unreachable for current geometry or tolerance."

    chosen = q_up if err_up <= err_down else q_down

    for i, q in enumerate(chosen):
        modules[i]["q"] = q

    X, Y, Z, pts, names, frames = forward_kin(modules)
    err = np.linalg.norm(target_xyz - np.array([X[-1], Y[-1], Z[-1]], dtype=float))
    return True, f"Target reached. Error {err:.3f} m."


def apply_app_style(app):
    app.setStyle("Fusion")
    app.setStyleSheet("""
        QWidget {
            background-color: #0f172a;
            color: #e5e7eb;
            font-family: Segoe UI, Arial, sans-serif;
            font-size: 10pt;
        }
        QMainWindow {
            background-color: #0f172a;
        }
        QLabel {
            color: #e5e7eb;
        }
        QTabWidget::pane {
            border: 1px solid #334155;
            background: #111827;
            border-radius: 8px;
            top: -1px;
        }
        QTabBar::tab {
            background: #1e293b;
            color: #cbd5e1;
            padding: 8px 14px;
            border: 1px solid #334155;
            border-bottom: none;
            border-top-left-radius: 8px;
            border-top-right-radius: 8px;
            min-width: 92px;
        }
        QTabBar::tab:selected {
            background: #2563eb;
            color: white;
        }
        QTabBar::tab:!selected {
            margin-top: 4px;
        }
        QPushButton {
            background-color: #1d4ed8;
            color: white;
            border: none;
            border-radius: 8px;
            padding: 7px 12px;
            min-height: 32px;
            font-weight: 600;
        }
        QPushButton:hover {
            background-color: #2563eb;
        }
        QPushButton:pressed {
            background-color: #1e40af;
        }
        QPushButton:disabled {
            background-color: #475569;
            color: #cbd5e1;
        }
        QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox, QListWidget {
            background-color: #111827;
            color: #e5e7eb;
            border: 1px solid #334155;
            border-radius: 8px;
            padding: 5px 7px;
            min-height: 28px;
        }
        QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus, QListWidget:focus {
            border: 1px solid #60a5fa;
        }
        QComboBox::drop-down {
            border: none;
            width: 22px;
        }
        QScrollArea, QScrollArea QWidget {
            background-color: #111827;
        }
        QCheckBox {
            spacing: 8px;
        }
        QCheckBox::indicator {
            width: 16px;
            height: 16px;
        }
        QCheckBox::indicator:unchecked {
            border: 1px solid #64748b;
            background: #0f172a;
            border-radius: 4px;
        }
        QCheckBox::indicator:checked {
            border: 1px solid #2563eb;
            background: #2563eb;
            border-radius: 4px;
        }
        QFrame[card="true"], QGroupBox {
            background-color: #111827;
            border: 1px solid #334155;
            border-radius: 10px;
            margin-top: 10px;
            padding: 10px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 6px 0 6px;
            color: #93c5fd;
            font-weight: 700;
        }
    """)


def style_toolbar(toolbar):
    toolbar.setIconSize(QtCore.QSize(20, 20))
    toolbar.setStyleSheet("""
        QToolBar {
            background: #1e293b;
            border: 1px solid #475569;
            border-radius: 8px;
            spacing: 6px;
            padding: 4px;
        }
        QToolButton {
            background: transparent;
            border: none;
            border-radius: 6px;
            padding: 5px;
            margin: 1px;
        }
        QToolButton:hover {
            background: #334155;
        }
        QToolButton:pressed {
            background: #475569;
        }
        QLabel {
            color: #e5e7eb;
            background: transparent;
        }
    """)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.fig, self.ax = create_plot()
        self.canvas = FigureCanvas(self.fig)
        self.toolbar = NavigationToolbar(self.canvas, self)

        self.struct_fig, self.struct_ax = create_plot()
        self.struct_canvas = FigureCanvas(self.struct_fig)
        self.struct_toolbar = NavigationToolbar(self.struct_canvas, self)

        self.cfg_fig, self.cfg_ax = create_plot()
        self.cfg_canvas = FigureCanvas(self.cfg_fig)
        self.cfg_toolbar = NavigationToolbar(self.cfg_canvas, self)


class ModularJointUI(object):
    def setupUi(self, win):
        self.window = win
        self.unit_mode = "Degrees"
        self.step_rad = 0.0872665
        self.step_degree = 5
        self.joint_count = 5
        self.modules = make_default_modules(self.joint_count)
        self.show_plot_axes = True
        self.show_frame_axes = True
        self.preview_mode = "Skeleton"
        self.control_mode = "Joint"
        self.world_target = np.array([0.0, 0.0, 0.0], dtype=float)
        self.world_yaw = 0.0
        self.world_step_pos = 0.01
        self.world_step_yaw = math.radians(3.0)
        self.world_reach_tolerance = 0.015

        self.control_rows = []
        self.structure_rows = []
        self.config_rows = []
        self.saved_poses = []

        self.uart_ready = False
        self.view_needs_refit = True

        win.setWindowTitle("OMNI Robot Arm Studio")
        win.resize(WINDOW_W, WINDOW_H)

        central = QtWidgets.QWidget()
        win.setCentralWidget(central)
        self.root = QtWidgets.QVBoxLayout(central)
        self.root.setSpacing(10)
        self.root.setContentsMargins(10, 10, 10, 10)

        title = QtWidgets.QLabel("OMNI Robot Arm Studio")
        title_font = QtGui.QFont("Segoe UI", 15)
        title_font.setBold(True)
        title.setFont(title_font)
        self.root.addWidget(title)

        top_group = QtWidgets.QGroupBox("Workspace")
        top_layout = QtWidgets.QHBoxLayout(top_group)
        top_layout.setSpacing(10)

        count_lbl = QtWidgets.QLabel("Joint count")
        self.countSpin = QtWidgets.QSpinBox()
        self.countSpin.setRange(1, 8)
        self.countSpin.setValue(self.joint_count)
        self.countSpin.valueChanged.connect(self.change_joint_count)

        unit_lbl = QtWidgets.QLabel("Angle units")
        self.unitBox = QtWidgets.QComboBox()
        self.unitBox.addItems(["Radians", "Degrees"])
        self.unitBox.setCurrentText(self.unit_mode)
        self.unitBox.currentTextChanged.connect(self.change_unit_mode)

        step_lbl = QtWidgets.QLabel("Step size")
        self.stepSpin = QtWidgets.QDoubleSpinBox()
        self.stepSpin.setDecimals(3)
        self.stepSpin.setRange(0.001, 360.0)
        self.stepSpin.setSingleStep(0.01)
        self.stepSpin.valueChanged.connect(self.change_step_size)

        preview_lbl = QtWidgets.QLabel("Preview")
        self.previewModeBox = QtWidgets.QComboBox()
        self.previewModeBox.addItems(["Skeleton", "3D Model"])
        self.previewModeBox.setCurrentText(self.preview_mode)
        self.previewModeBox.currentTextChanged.connect(self.change_preview_mode)

        mode_lbl = QtWidgets.QLabel("Control")
        self.controlModeBox = QtWidgets.QComboBox()
        self.controlModeBox.addItems(["Joint", "World"])
        self.controlModeBox.setCurrentText(self.control_mode)
        self.controlModeBox.currentTextChanged.connect(self.change_control_mode)

        self.stepUnitLbl = QtWidgets.QLabel()
        self.plotAxesCheck = QtWidgets.QCheckBox("Plot axes")
        self.plotAxesCheck.setChecked(True)
        self.plotAxesCheck.toggled.connect(self.toggle_plot_axes)

        self.frameAxesCheck = QtWidgets.QCheckBox("Frame axes")
        self.frameAxesCheck.setChecked(True)
        self.frameAxesCheck.toggled.connect(self.toggle_frame_axes)

        self.posLbl = QtWidgets.QLabel("End-effector: X=0.000, Y=0.000, Z=0.000")
        self.canStatusLbl = QtWidgets.QLabel("UART: not initialized")

        for w in [
            count_lbl, self.countSpin, unit_lbl, self.unitBox, step_lbl, self.stepSpin, self.stepUnitLbl,
            preview_lbl, self.previewModeBox, mode_lbl, self.controlModeBox,
            self.plotAxesCheck, self.frameAxesCheck, self.canStatusLbl
        ]:
            top_layout.addWidget(w)

        top_layout.addStretch()
        top_layout.addWidget(self.posLbl)

        self.root.addWidget(top_group)

        self.tabs = QtWidgets.QTabWidget()
        self.root.addWidget(self.tabs)

        self.icon_up = QtGui.QIcon(QtGui.QPixmap(os.path.join(BASE_DIR, "up.png")))
        self.icon_down = QtGui.QIcon(QtGui.QPixmap(os.path.join(BASE_DIR, "down.png")))

        style_toolbar(self.window.toolbar)
        style_toolbar(self.window.struct_toolbar)
        style_toolbar(self.window.cfg_toolbar)

        self.rebuild_tabs()
        self.refresh_step_spin()
        self.sync_world_target_to_current()
        self.init_uart()
        self.plot_data()

    def init_uart(self):
        try:
            _ = ServoControl.serialHandle
            self.uart_ready = True
            self.canStatusLbl.setText("UART: ready")
        except Exception as e:
            self.uart_ready = False
            self.canStatusLbl.setText(f"UART: init failed ({e})")

    def angle_to_display(self, rad_value):
        return math.degrees(rad_value) if self.unit_mode == "Degrees" else rad_value

    def angle_from_display(self, shown_value):
        return math.radians(shown_value) if self.unit_mode == "Degrees" else shown_value

    def angle_unit_short(self):
        return "deg" if self.unit_mode == "Degrees" else "rad"

    def format_angle(self, rad_value, decimals=3):
        return f"{self.angle_to_display(rad_value):.{decimals}f}"

    def refresh_step_spin(self):
        self.stepSpin.blockSignals(True)
        if self.unit_mode == "Degrees":
            self.stepSpin.setDecimals(2)
            self.stepSpin.setRange(0.01, 360.0)
            self.stepSpin.setSingleStep(1.0)
            self.stepSpin.setValue(math.degrees(self.step_rad))
        else:
            self.stepSpin.setDecimals(3)
            self.stepSpin.setRange(0.001, 6.283)
            self.stepSpin.setSingleStep(0.01)
            self.stepSpin.setValue(self.step_rad)
        self.stepSpin.blockSignals(False)
        self.stepUnitLbl.setText(self.angle_unit_short())

    def toggle_plot_axes(self, checked):
        self.show_plot_axes = checked
        self.plot_data()

    def toggle_frame_axes(self, checked):
        self.show_frame_axes = checked
        self.plot_data()

    def change_preview_mode(self, mode):
        self.preview_mode = mode
        self.plot_data()

    def change_control_mode(self, mode):
        self.control_mode = mode
        self.refresh_control_mode_ui()

    def rebuild_tabs(self):
        self.tabs.clear()
        self.control_rows = []
        self.structure_rows = []
        self.config_rows = []

        self.build_control_tab()
        self.build_structure_tab()
        self.build_config_tab()

        self.refresh_headers()
        self.refresh_all_control_rows()
        self.refresh_structure_fields()
        self.refresh_config_fields()
        self.refresh_world_fields()
        self.refresh_control_mode_ui()
        self.refresh_pose_list()

    def change_joint_count(self, new_count):
        old_modules = self.modules
        self.joint_count = new_count
        new_modules = make_default_modules(new_count)

        old_joint_map = {m["name"]: m for m in old_modules}
        for m in new_modules:
            if m["name"] in old_joint_map:
                old = old_joint_map[m["name"]]
                m["type"] = old["type"]
                m["axis"] = old["axis"]
                m["offset"] = old["offset"].copy()
                m["link"] = old["link"].copy()
                m["q"] = old["q"]
                m["home_q"] = old["home_q"]
                m["qlim"] = old["qlim"]
                m["fixed"] = old["fixed"]
                m["servo_id"] = old.get("servo_id", default_servo_id_for_name(m["name"]))
                m["servo_offset_deg"] = old.get("servo_offset_deg", 0.0)

        self.modules = new_modules
        self.rebuild_tabs()
        self.sync_world_target_to_current()
        self.view_needs_refit = True
        self.plot_data()

    def change_step_size(self, shown_value):
        self.step_rad = self.angle_from_display(shown_value)
        self.refresh_headers()

    def change_unit_mode(self, new_unit):
        self.unit_mode = new_unit
        self.refresh_step_spin()
        self.refresh_headers()
        self.refresh_all_control_rows()
        self.refresh_config_fields()

    def refresh_headers(self):
        self.control_angle_header.setText(f"Angle ({self.angle_unit_short()})")
        self.config_min_header.setText(f"Min ({self.angle_unit_short()})")
        self.config_max_header.setText(f"Max ({self.angle_unit_short()})")
        self.config_home_header.setText(f"Home ({self.angle_unit_short()})")
        self.help_lbl.setText(
            "Rotation joints use 240-degree servo mapping and swivel joints use 360-degree mapping. "
            "Servo direction can be inverted per joint in Configure Robot, and joint angles can be typed directly in Joint Control. "
            "Read Angles queries the Hiwonder controller using the configured Servo ID values."
        )


    def make_card(self):
        w = QtWidgets.QFrame()
        w.setProperty("card", True)
        return w

    def build_control_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)
        layout.setSpacing(10)

        left_card = self.make_card()
        left_layout = QtWidgets.QVBoxLayout(left_card)

        subtitle = QtWidgets.QLabel("Joint Control")
        subtitle_font = QtGui.QFont("Segoe UI", 11)
        subtitle_font.setBold(True)
        subtitle.setFont(subtitle_font)
        left_layout.addWidget(subtitle)

        grid_wrap = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(grid_wrap)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)

        headers = ["Joint", "Type", "Axis", "Servo ID", "Select", "Down", None, "Up", "Servo Cmd"]
        for c, text in enumerate(headers):
            lbl = QtWidgets.QLabel("" if text is None else text)
            lbl.setStyleSheet("color: #93c5fd; font-weight: 700;")
            grid.addWidget(lbl, 0, c)
            if c == 6:
                self.control_angle_header = lbl

        for i, m in enumerate(self.modules, start=1):
            name_lbl = QtWidgets.QLabel(m["name"])
            type_lbl = QtWidgets.QLabel(m["type"])
            axis_lbl = QtWidgets.QLabel(m["axis"].upper())

            servo_id_box = QtWidgets.QSpinBox()
            servo_id_box.setRange(0, 254)
            servo_id_box.setValue(int(m.get("servo_id", 0)))
            servo_id_box.setMaximumWidth(70)
            if m["fixed"]:
                servo_id_box.setEnabled(False)

            select_box = QtWidgets.QCheckBox()
            select_box.setChecked(not m["fixed"])
            if m["fixed"]:
                select_box.setEnabled(False)

            down_btn = QtWidgets.QPushButton()
            down_btn.setIcon(self.icon_down)
            down_btn.setIconSize(QtCore.QSize(20, 20))
            down_btn.setFixedSize(38, 38)

            angle_edit = QtWidgets.QDoubleSpinBox()
            angle_edit.setDecimals(2 if self.unit_mode == "Degrees" else 3)
            angle_edit.setRange(-100000.0, 100000.0)
            angle_edit.setSingleStep(1.0 if self.unit_mode == "Degrees" else 0.01)
            angle_edit.setMinimumWidth(95)
            angle_edit.setMinimumHeight(34)

            aux_lbl = QtWidgets.QLabel()
            aux_lbl.setAlignment(QtCore.Qt.AlignCenter)
            aux_lbl.setMinimumWidth(80)

            up_btn = QtWidgets.QPushButton()
            up_btn.setIcon(self.icon_up)
            up_btn.setIconSize(QtCore.QSize(20, 20))
            up_btn.setFixedSize(38, 38)

            if not m["fixed"]:
                down_btn.clicked.connect(lambda _, idx=i-1: self.change_angle(idx, -self.step_rad))
                up_btn.clicked.connect(lambda _, idx=i-1: self.change_angle(idx, self.step_rad))
                servo_id_box.valueChanged.connect(lambda val, idx=i-1: self.change_servo_id(idx, val))
                angle_edit.valueChanged.connect(lambda val, idx=i-1: self.set_joint_angle_from_display(idx, val))
            else:
                down_btn.setEnabled(False)
                up_btn.setEnabled(False)
                angle_edit.setEnabled(False)

            widgets = [name_lbl, type_lbl, axis_lbl, servo_id_box, select_box, down_btn, angle_edit, up_btn, aux_lbl]
            for c, w in enumerate(widgets):
                grid.addWidget(w, i, c)

            self.control_rows.append({
                "type_lbl": type_lbl,
                "axis_lbl": axis_lbl,
                "servo_id_box": servo_id_box,
                "select_box": select_box,
                "down_btn": down_btn,
                "angle_edit": angle_edit,
                "up_btn": up_btn,
                "aux_lbl": aux_lbl,
            })

        left_layout.addWidget(grid_wrap)

        self.worldGroup = QtWidgets.QGroupBox("World Control")
        world_layout = QtWidgets.QGridLayout(self.worldGroup)
        world_layout.setHorizontalSpacing(8)
        world_layout.setVerticalSpacing(8)

        self.worldStatusLbl = QtWidgets.QLabel("Move the end-effector in world X, Y, Z and yaw.")
        self.worldStatusLbl.setWordWrap(True)
        world_layout.addWidget(self.worldStatusLbl, 0, 0, 1, 6)

        world_layout.addWidget(QtWidgets.QLabel("X (m)"), 1, 0)
        self.worldXEdit = QtWidgets.QDoubleSpinBox()
        self.worldXEdit.setDecimals(3)
        self.worldXEdit.setRange(-2.0, 2.0)
        self.worldXEdit.setSingleStep(0.01)
        world_layout.addWidget(self.worldXEdit, 1, 1)

        world_layout.addWidget(QtWidgets.QLabel("Y (m)"), 1, 2)
        self.worldYEdit = QtWidgets.QDoubleSpinBox()
        self.worldYEdit.setDecimals(3)
        self.worldYEdit.setRange(-2.0, 2.0)
        self.worldYEdit.setSingleStep(0.01)
        world_layout.addWidget(self.worldYEdit, 1, 3)

        world_layout.addWidget(QtWidgets.QLabel("Z (m)"), 1, 4)
        self.worldZEdit = QtWidgets.QDoubleSpinBox()
        self.worldZEdit.setDecimals(3)
        self.worldZEdit.setRange(0.0, 2.5)
        self.worldZEdit.setSingleStep(0.01)
        world_layout.addWidget(self.worldZEdit, 1, 5)

        world_layout.addWidget(QtWidgets.QLabel("Yaw (deg)"), 2, 0)
        self.worldYawEdit = QtWidgets.QDoubleSpinBox()
        self.worldYawEdit.setDecimals(1)
        self.worldYawEdit.setRange(-180.0, 180.0)
        self.worldYawEdit.setSingleStep(3.0)
        world_layout.addWidget(self.worldYawEdit, 2, 1)

        self.worldSyncBtn = QtWidgets.QPushButton("Use Current Pose")
        self.worldSyncBtn.clicked.connect(self.sync_world_target_to_current)
        world_layout.addWidget(self.worldSyncBtn, 2, 2)

        self.worldGoBtn = QtWidgets.QPushButton("Move To Target")
        self.worldGoBtn.clicked.connect(self.move_world_target_absolute)
        world_layout.addWidget(self.worldGoBtn, 2, 3)

        world_layout.addWidget(QtWidgets.QLabel("Step (m)"), 2, 4)
        self.worldStepPosEdit = QtWidgets.QDoubleSpinBox()
        self.worldStepPosEdit.setDecimals(3)
        self.worldStepPosEdit.setRange(0.001, 0.200)
        self.worldStepPosEdit.setValue(0.01)
        world_layout.addWidget(self.worldStepPosEdit, 2, 5)

        world_layout.addWidget(QtWidgets.QLabel("Yaw step"), 3, 4)
        self.worldStepYawEdit = QtWidgets.QDoubleSpinBox()
        self.worldStepYawEdit.setDecimals(1)
        self.worldStepYawEdit.setRange(0.1, 45.0)
        self.worldStepYawEdit.setValue(3.0)
        world_layout.addWidget(self.worldStepYawEdit, 3, 5)

        world_layout.addWidget(QtWidgets.QLabel("Tolerance"), 4, 4)
        self.worldTolEdit = QtWidgets.QDoubleSpinBox()
        self.worldTolEdit.setDecimals(3)
        self.worldTolEdit.setRange(0.001, 0.200)
        self.worldTolEdit.setValue(self.world_reach_tolerance)
        world_layout.addWidget(self.worldTolEdit, 4, 5)

        btn_specs = [
            ("X-", 3, 0, 0, -1), ("X+", 3, 1, 0, 1),
            ("Y-", 3, 2, 1, -1), ("Y+", 3, 3, 1, 1),
            ("Z-", 4, 0, 2, -1), ("Z+", 4, 1, 2, 1),
            ("Yaw-", 4, 2, 3, -1), ("Yaw+", 4, 3, 3, 1),
        ]
        for text, row, col, axis_id, sign in btn_specs:
            btn = QtWidgets.QPushButton(text)
            btn.clicked.connect(lambda _, a=axis_id, s=sign: self.nudge_world_target(a, s))
            world_layout.addWidget(btn, row, col)

        left_layout.addWidget(self.worldGroup)

        self.poseGroup = QtWidgets.QGroupBox("Saved Poses")
        pose_layout = QtWidgets.QVBoxLayout(self.poseGroup)

        self.poseList = QtWidgets.QListWidget()
        self.poseList.setMinimumHeight(120)
        pose_layout.addWidget(self.poseList)

        pose_btn_row = QtWidgets.QHBoxLayout()

        self.savePoseBtn = QtWidgets.QPushButton("Save Current")
        self.savePoseBtn.clicked.connect(self.capture_current_pose)
        pose_btn_row.addWidget(self.savePoseBtn)

        self.loadPoseBtn = QtWidgets.QPushButton("Go To Pose")
        self.loadPoseBtn.clicked.connect(self.go_to_selected_pose)
        pose_btn_row.addWidget(self.loadPoseBtn)

        self.updatePoseBtn = QtWidgets.QPushButton("Update Pose")
        self.updatePoseBtn.clicked.connect(self.overwrite_selected_pose)
        pose_btn_row.addWidget(self.updatePoseBtn)

        self.deletePoseBtn = QtWidgets.QPushButton("Delete Pose")
        self.deletePoseBtn.clicked.connect(self.delete_selected_pose)
        pose_btn_row.addWidget(self.deletePoseBtn)

        pose_layout.addLayout(pose_btn_row)
        left_layout.addWidget(self.poseGroup)

        btn_row = QtWidgets.QHBoxLayout()
        self.resetBtn = QtWidgets.QPushButton("Reset")
        self.resetBtn.clicked.connect(self.reset_angles)
        btn_row.addWidget(self.resetBtn)

        self.homeBtn = QtWidgets.QPushButton("Home")
        self.homeBtn.clicked.connect(self.go_home)
        btn_row.addWidget(self.homeBtn)

        self.sendBtn = QtWidgets.QPushButton("Send")
        self.sendBtn.clicked.connect(self.send)
        btn_row.addWidget(self.sendBtn)

        self.readBtn = QtWidgets.QPushButton("Read Angles")
        self.readBtn.clicked.connect(self.read_current_angles)
        btn_row.addWidget(self.readBtn)

        self.powerOffSelectedBtn = QtWidgets.QPushButton("Power Off Selected")
        self.powerOffSelectedBtn.clicked.connect(self.power_off_selected_servos)
        btn_row.addWidget(self.powerOffSelectedBtn)

        self.powerOffAllBtn = QtWidgets.QPushButton("Power Off All")
        self.powerOffAllBtn.clicked.connect(self.power_off_all_servos)
        btn_row.addWidget(self.powerOffAllBtn)

        btn_row.addStretch()
        left_layout.addLayout(btn_row)

        right_card = self.make_card()
        right_layout = QtWidgets.QVBoxLayout(right_card)

        plot_title = QtWidgets.QLabel("3D Preview")
        plot_title.setFont(subtitle_font)
        right_layout.addWidget(plot_title)

        plot_info = QtWidgets.QLabel("Resizable window; UART send uses Hiwonder bus-servo position commands.")
        plot_info.setWordWrap(True)
        plot_info.setStyleSheet("color: #cbd5e1;")
        right_layout.addWidget(plot_info)

        right_layout.addWidget(self.window.toolbar)
        self.window.canvas.setMinimumHeight(480)
        self.window.canvas.setStyleSheet("background: #0b1220; border: 1px solid #334155; border-radius: 8px;")
        right_layout.addWidget(self.window.canvas, 1)

        layout.addWidget(left_card, 3)
        layout.addWidget(right_card, 2)

        self.tabs.addTab(tab, "Control")


    def build_structure_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)
        layout.setSpacing(10)

        left_card = self.make_card()
        left_layout = QtWidgets.QVBoxLayout(left_card)

        info_lbl = QtWidgets.QLabel("Quick structure editor: choose each joint type and each link length.")
        info_lbl.setWordWrap(True)
        left_layout.addWidget(info_lbl)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)

        content = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(content)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(10)

        hdr0 = QtWidgets.QLabel("Item")
        hdr1 = QtWidgets.QLabel("Setting")
        hdr0.setStyleSheet("color: #93c5fd; font-weight: 700;")
        hdr1.setStyleSheet("color: #93c5fd; font-weight: 700;")
        grid.addWidget(hdr0, 0, 0)
        grid.addWidget(hdr1, 0, 1)

        row_idx = 1
        joint_modules = self.modules[:-1] if self.modules and self.modules[-1]["name"] == "GRIP" else self.modules

        for i, m in enumerate(joint_modules):
            joint_lbl = QtWidgets.QLabel(m["name"])
            type_box = QtWidgets.QComboBox()
            type_box.addItems(["none", "swivel", "rotation"])
            type_box.setCurrentText(m["type"])

            grid.addWidget(joint_lbl, row_idx, 0)
            grid.addWidget(type_box, row_idx, 1)

            self.structure_rows.append({
                "kind": "joint",
                "module_index": i,
                "type_box": type_box,
            })
            row_idx += 1

            if i < len(joint_modules) - 1:
                link_lbl = QtWidgets.QLabel(f"L{i+1}")
                link_box = QtWidgets.QComboBox()
                link_box.addItems([f"{v} in" for v in LINK_PRESETS_IN])

                grid.addWidget(link_lbl, row_idx, 0)
                grid.addWidget(link_box, row_idx, 1)

                self.structure_rows.append({
                    "kind": "link",
                    "module_index": i,
                    "link_box": link_box,
                })
                row_idx += 1

        scroll.setWidget(content)
        left_layout.addWidget(scroll)

        btn_row = QtWidgets.QHBoxLayout()
        self.applyStructureBtn = QtWidgets.QPushButton("Apply Structure")
        self.applyStructureBtn.clicked.connect(self.apply_structure)
        btn_row.addWidget(self.applyStructureBtn)
        btn_row.addStretch()
        left_layout.addLayout(btn_row)

        right_card = self.make_card()
        right_layout = QtWidgets.QVBoxLayout(right_card)

        preview_title = QtWidgets.QLabel("3D Preview")
        preview_font = QtGui.QFont("Segoe UI", 11)
        preview_font.setBold(True)
        preview_title.setFont(preview_font)
        right_layout.addWidget(preview_title)

        preview_info = QtWidgets.QLabel("Preview current structure while editing joint types and link lengths.")
        preview_info.setWordWrap(True)
        preview_info.setStyleSheet("color: #cbd5e1;")
        right_layout.addWidget(preview_info)

        right_layout.addWidget(self.window.struct_toolbar)
        self.window.struct_canvas.setMinimumHeight(480)
        self.window.struct_canvas.setStyleSheet("background: #0b1220; border: 1px solid #334155; border-radius: 8px;")
        right_layout.addWidget(self.window.struct_canvas, 1)

        layout.addWidget(left_card, 3)
        layout.addWidget(right_card, 2)

        self.tabs.addTab(tab, "Structure")

    def build_config_tab(self):
        tab = QtWidgets.QWidget()
        layout = QtWidgets.QHBoxLayout(tab)
        layout.setSpacing(10)

        left_card = self.make_card()
        left_layout = QtWidgets.QVBoxLayout(left_card)

        self.help_lbl = QtWidgets.QLabel("")
        self.help_lbl.setWordWrap(True)
        left_layout.addWidget(self.help_lbl)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)

        content = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(content)
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)

        headers = [
            "Joint", "Type", "Axis",
            "Offset X", "Offset Y", "Offset Z",
            "Link X", "Link Y", "Link Z",
            None, None, None,
            "Servo Offset (deg)", "Invert"
        ]
        for c, text in enumerate(headers):
            lbl = QtWidgets.QLabel("" if text is None else text)
            lbl.setStyleSheet("color: #93c5fd; font-weight: 700;")
            grid.addWidget(lbl, 0, c)
            if c == 9:
                self.config_min_header = lbl
            elif c == 10:
                self.config_max_header = lbl
            elif c == 11:
                self.config_home_header = lbl

        for i, m in enumerate(self.modules, start=1):
            name_lbl = QtWidgets.QLabel(m["name"])

            type_box = QtWidgets.QComboBox()
            type_box.addItems(["none", "swivel", "rotation"])
            type_box.setCurrentText(m["type"])

            axis_box = QtWidgets.QComboBox()
            axis_box.addItems(["x", "y", "z"])
            axis_box.setCurrentText(m["axis"])

            ox = QtWidgets.QLineEdit(str(m["offset"][0]))
            oy = QtWidgets.QLineEdit(str(m["offset"][1]))
            oz = QtWidgets.QLineEdit(str(m["offset"][2]))
            lx = QtWidgets.QLineEdit(str(m["link"][0]))
            ly = QtWidgets.QLineEdit(str(m["link"][1]))
            lz = QtWidgets.QLineEdit(str(m["link"][2]))
            qmin_edit = QtWidgets.QLineEdit("" if m["fixed"] else self.format_angle(m["qlim"][0]))
            qmax_edit = QtWidgets.QLineEdit("" if m["fixed"] else self.format_angle(m["qlim"][1]))
            home_edit = QtWidgets.QLineEdit("" if m["fixed"] else self.format_angle(m["home_q"]))
            servo_offset_edit = QtWidgets.QLineEdit(str(float(m.get("servo_offset_deg", 0.0))))

            invert_box = QtWidgets.QCheckBox()
            invert_box.setChecked(bool(SERVO_CAL.get(m["name"], {}).get("invert", False)))

            widgets = [
                name_lbl, type_box, axis_box,
                ox, oy, oz,
                lx, ly, lz,
                qmin_edit, qmax_edit, home_edit,
                servo_offset_edit, invert_box
            ]
            for c, w in enumerate(widgets):
                if hasattr(w, "setMinimumWidth"):
                    w.setMinimumWidth(64)
                grid.addWidget(w, i, c)

            if m["fixed"]:
                type_box.setEnabled(False)
                axis_box.setEnabled(False)
                qmin_edit.setEnabled(False)
                qmax_edit.setEnabled(False)
                home_edit.setEnabled(False)
                servo_offset_edit.setEnabled(False)
                invert_box.setEnabled(False)

            self.config_rows.append({
                "type_box": type_box,
                "axis_box": axis_box,
                "ox": ox, "oy": oy, "oz": oz,
                "lx": lx, "ly": ly, "lz": lz,
                "qmin_edit": qmin_edit,
                "qmax_edit": qmax_edit,
                "home_edit": home_edit,
                "servo_offset_edit": servo_offset_edit,
                "invert_box": invert_box,
            })

        scroll.setWidget(content)
        left_layout.addWidget(scroll)

        btn_row = QtWidgets.QHBoxLayout()
        self.applyBtn = QtWidgets.QPushButton("Apply Geometry")
        self.applyBtn.clicked.connect(self.apply_geometry)
        btn_row.addWidget(self.applyBtn)
        btn_row.addStretch()
        left_layout.addLayout(btn_row)

        right_card = self.make_card()
        right_layout = QtWidgets.QVBoxLayout(right_card)

        preview_title = QtWidgets.QLabel("3D Preview")
        preview_font = QtGui.QFont("Segoe UI", 11)
        preview_font.setBold(True)
        preview_title.setFont(preview_font)
        right_layout.addWidget(preview_title)

        preview_info = QtWidgets.QLabel("Preview current robot geometry and calibration while editing Configure Robot values.")
        preview_info.setWordWrap(True)
        preview_info.setStyleSheet("color: #cbd5e1;")
        right_layout.addWidget(preview_info)

        right_layout.addWidget(self.window.cfg_toolbar)
        self.window.cfg_canvas.setMinimumHeight(480)
        self.window.cfg_canvas.setStyleSheet("background: #0b1220; border: 1px solid #334155; border-radius: 8px;")
        right_layout.addWidget(self.window.cfg_canvas, 1)

        layout.addWidget(left_card, 3)
        layout.addWidget(right_card, 2)

        self.tabs.addTab(tab, "Configure Robot")




    def change_servo_id(self, idx, value):
        self.modules[idx]["servo_id"] = int(value)

    def sync_world_target_to_current(self):
        X, Y, Z, pts, point_names, frames = forward_kin(self.modules)
        self.world_target = np.array([X[-1], Y[-1], Z[-1]], dtype=float)
        self.world_yaw = get_end_effector_yaw(frames)
        self.refresh_world_fields()
        if hasattr(self, "worldStatusLbl"):
            self.worldStatusLbl.setText("World target synced to current pose.")

    def refresh_world_fields(self):
        if not hasattr(self, "worldXEdit"):
            return

        self.worldXEdit.blockSignals(True)
        self.worldYEdit.blockSignals(True)
        self.worldZEdit.blockSignals(True)
        self.worldYawEdit.blockSignals(True)

        self.worldXEdit.setValue(float(self.world_target[0]))
        self.worldYEdit.setValue(float(self.world_target[1]))
        self.worldZEdit.setValue(float(self.world_target[2]))
        self.worldYawEdit.setValue(math.degrees(self.world_yaw))

        if hasattr(self, "worldTolEdit"):
            self.worldTolEdit.blockSignals(True)
            self.worldTolEdit.setValue(self.world_reach_tolerance)
            self.worldTolEdit.blockSignals(False)

        self.worldXEdit.blockSignals(False)
        self.worldYEdit.blockSignals(False)
        self.worldZEdit.blockSignals(False)
        self.worldYawEdit.blockSignals(False)

    def refresh_control_mode_ui(self):
        joint_enabled = self.control_mode == "Joint"

        for i, row in enumerate(self.control_rows):
            m = self.modules[i]
            if m["fixed"] or m["type"] == "none":
                row["down_btn"].setEnabled(False)
                row["up_btn"].setEnabled(False)
                row["angle_edit"].setEnabled(False)
            else:
                row["down_btn"].setEnabled(joint_enabled)
                row["up_btn"].setEnabled(joint_enabled)
                row["angle_edit"].setEnabled(joint_enabled)

        if hasattr(self, "worldGroup"):
            self.worldGroup.setVisible(True)
            self.worldGroup.setEnabled(self.control_mode == "World")

    def move_world_target_absolute(self):
        self.world_target = np.array([
            self.worldXEdit.value(),
            self.worldYEdit.value(),
            self.worldZEdit.value()
        ], dtype=float)
        self.world_yaw = math.radians(self.worldYawEdit.value())
        self.world_reach_tolerance = self.worldTolEdit.value()

        ok, msg = solve_world_geometric(self.modules, self.world_target, self.world_yaw, reach_tolerance=self.world_reach_tolerance)
        self.refresh_all_control_rows()
        self.plot_data()
        self.worldStatusLbl.setText(msg)

    def nudge_world_target(self, axis_id, sign):
        self.world_step_pos = self.worldStepPosEdit.value()
        self.world_step_yaw = math.radians(self.worldStepYawEdit.value())
        self.world_reach_tolerance = self.worldTolEdit.value()

        if axis_id == 0:
            self.world_target[0] += sign * self.world_step_pos
        elif axis_id == 1:
            self.world_target[1] += sign * self.world_step_pos
        elif axis_id == 2:
            self.world_target[2] = max(0.0, self.world_target[2] + sign * self.world_step_pos)
        elif axis_id == 3:
            self.world_yaw = wrap_to_pi(self.world_yaw + sign * self.world_step_yaw)

        self.refresh_world_fields()
        ok, msg = solve_world_geometric(self.modules, self.world_target, self.world_yaw, reach_tolerance=self.world_reach_tolerance)
        self.refresh_all_control_rows()
        self.plot_data()
        self.worldStatusLbl.setText(msg)

    def refresh_structure_fields(self):
        for row in self.structure_rows:
            if row["kind"] == "joint":
                m = self.modules[row["module_index"]]
                row["type_box"].setCurrentText(m["type"])
            elif row["kind"] == "link":
                m = self.modules[row["module_index"]]
                row["link_box"].setCurrentText(nearest_link_preset_text(float(np.linalg.norm(m["link"]))))

    def refresh_config_fields(self):
        for i, m in enumerate(self.modules):
            row = self.config_rows[i]
            row["type_box"].setCurrentText(m["type"])
            row["axis_box"].setCurrentText(m["axis"])
            row["ox"].setText(str(m["offset"][0]))
            row["oy"].setText(str(m["offset"][1]))
            row["oz"].setText(str(m["offset"][2]))
            row["lx"].setText(str(m["link"][0]))
            row["ly"].setText(str(m["link"][1]))
            row["lz"].setText(str(m["link"][2]))

            if "servo_offset_edit" in row:
                row["servo_offset_edit"].setText(str(float(m.get("servo_offset_deg", 0.0))))
                row["servo_offset_edit"].setEnabled(not m["fixed"])

            if "invert_box" in row:
                row["invert_box"].blockSignals(True)
                row["invert_box"].setChecked(bool(SERVO_CAL.get(m["name"], {}).get("invert", False)))
                row["invert_box"].setEnabled(not m["fixed"])
                row["invert_box"].blockSignals(False)

            if not m["fixed"]:
                row["qmin_edit"].setText(self.format_angle(m["qlim"][0]))
                row["qmax_edit"].setText(self.format_angle(m["qlim"][1]))
                row["home_edit"].setText(self.format_angle(m["home_q"]))

    def apply_structure(self):
        try:
            for row in self.structure_rows:
                if row["kind"] == "joint":
                    m = self.modules[row["module_index"]]
                    if m["fixed"]:
                        continue

                    new_type = row["type_box"].currentText()
                    m["type"] = new_type

                    if new_type == "none":
                        m["q"] = 0.0
                        m["home_q"] = 0.0
                        m["qlim"] = (0.0, 0.0)
                    else:
                        meta = JOINT_TYPES[new_type]
                        if m["qlim"] == (0.0, 0.0):
                            m["qlim"] = meta["default_limits"]

                        if m["name"] == "J1":
                            m["axis"] = "z"
                        elif new_type == "rotation":
                            m["axis"] = "y"
                        elif new_type == "swivel":
                            m["axis"] = "z"

                elif row["kind"] == "link":
                    m = self.modules[row["module_index"]]
                    selected = row["link_box"].currentText()
                    link_len = LINK_PRESETS_M[selected]

                    old_norm = float(np.linalg.norm(m["link"]))
                    if old_norm < 1e-9:
                        m["link"] = np.array([0.0, 0.0, link_len], dtype=float)
                    else:
                        m["link"] = (m["link"] / old_norm) * link_len

            self.refresh_all_control_rows()
            self.refresh_structure_fields()
            self.refresh_config_fields()
            self.sync_world_target_to_current()
            self.view_needs_refit = True
            self.plot_data()
        except Exception as e:
            print("Apply structure error:", e)


    def set_joint_angle_from_display(self, idx, shown_value):
        m = self.modules[idx]
        if m["fixed"] or m["type"] == "none":
            return

        q = self.angle_from_display(shown_value)
        qmin, qmax = m["qlim"]
        m["q"] = clamp(q, qmin, qmax)

        self.refresh_control_row(idx)
        self.sync_world_target_to_current()
        self.plot_data()


    def change_angle(self, idx, delta_rad):
        m = self.modules[idx]
        if m["fixed"] or m["type"] == "none":
            return

        qmin, qmax = m["qlim"]
        m["q"] = clamp(round(m["q"] + delta_rad, 6), qmin, qmax)

        self.refresh_control_row(idx)
        self.sync_world_target_to_current()
        self.plot_data()



    def refresh_control_row(self, idx):
        m = self.modules[idx]
        row = self.control_rows[idx]

        row["type_lbl"].setText(m["type"])
        row["axis_lbl"].setText(m["axis"].upper())

        row["servo_id_box"].blockSignals(True)
        row["servo_id_box"].setValue(int(m.get("servo_id", 0)))
        row["servo_id_box"].blockSignals(False)

        angle_edit = row["angle_edit"]
        angle_edit.blockSignals(True)

        if self.unit_mode == "Degrees":
            angle_edit.setDecimals(2)
            angle_edit.setSingleStep(max(0.1, math.degrees(self.step_rad)))
        else:
            angle_edit.setDecimals(3)
            angle_edit.setSingleStep(max(0.001, self.step_rad))

        if m["fixed"]:
            angle_edit.setRange(0.0, 0.0)
            angle_edit.setValue(0.0)
            angle_edit.setEnabled(False)
            servo_deg = joint_to_servo_deg(m)
            row["aux_lbl"].setText("-" if servo_deg is None else f"{servo_deg:.1f}")
            row["down_btn"].setEnabled(False)
            row["up_btn"].setEnabled(False)

        elif m["type"] == "none":
            angle_edit.setRange(0.0, 0.0)
            angle_edit.setValue(0.0)
            angle_edit.setEnabled(False)
            row["aux_lbl"].setText("-")
            row["down_btn"].setEnabled(False)
            row["up_btn"].setEnabled(False)

        else:
            qmin, qmax = m["qlim"]
            shown_q = self.angle_to_display(m["q"])
            shown_qmin = self.angle_to_display(qmin)
            shown_qmax = self.angle_to_display(qmax)

            angle_edit.setRange(min(shown_qmin, shown_qmax), max(shown_qmin, shown_qmax))
            angle_edit.setValue(shown_q)
            angle_edit.setEnabled(self.control_mode == "Joint")

            servo_deg = joint_to_servo_deg(m)
            row["aux_lbl"].setText("-" if servo_deg is None else f"{servo_deg:.1f}")
            row["down_btn"].setEnabled(self.control_mode == "Joint")
            row["up_btn"].setEnabled(self.control_mode == "Joint")

        angle_edit.blockSignals(False)


    def refresh_all_control_rows(self):
        for i in range(len(self.modules)):
            self.refresh_control_row(i)

    def apply_geometry(self):
        try:
            for i, m in enumerate(self.modules):
                row = self.config_rows[i]
                m["type"] = row["type_box"].currentText()
                m["axis"] = row["axis_box"].currentText()
                m["offset"] = np.array([
                    float(row["ox"].text()),
                    float(row["oy"].text()),
                    float(row["oz"].text())
                ], dtype=float)
                m["link"] = np.array([
                    float(row["lx"].text()),
                    float(row["ly"].text()),
                    float(row["lz"].text())
                ], dtype=float)

                if "servo_offset_edit" in row:
                    m["servo_offset_deg"] = float(row["servo_offset_edit"].text())

                if m["type"] == "none":
                    m["q"] = 0.0
                    m["home_q"] = 0.0
                    m["qlim"] = (0.0, 0.0)
                elif not m["fixed"]:
                    qmin = self.angle_from_display(float(row["qmin_edit"].text()))
                    qmax = self.angle_from_display(float(row["qmax_edit"].text()))
                    home_q = self.angle_from_display(float(row["home_edit"].text()))
                    if qmin > qmax:
                        qmin, qmax = qmax, qmin
                    m["qlim"] = (qmin, qmax)
                    m["home_q"] = max(qmin, min(qmax, home_q))
                    m["q"] = max(qmin, min(qmax, m["q"]))

                    if m["name"] in SERVO_CAL and "invert_box" in row:
                        SERVO_CAL[m["name"]]["invert"] = bool(row["invert_box"].isChecked())

            self.refresh_all_control_rows()
            self.refresh_structure_fields()
            self.refresh_config_fields()
            self.sync_world_target_to_current()
            self.view_needs_refit = True
            self.plot_data()
        except Exception as e:
            print("Apply geometry error:", e)

    def go_home(self):
        for m in self.modules:
            if not m["fixed"] and m["type"] != "none":
                qmin, qmax = m["qlim"]
                m["q"] = max(qmin, min(qmax, m["home_q"]))
        self.refresh_all_control_rows()
        self.sync_world_target_to_current()
        self.plot_data()

    def reset_angles(self):
        for m in self.modules:
            if not m["fixed"] and m["type"] != "none":
                m["q"] = 0.0
        self.refresh_all_control_rows()
        self.sync_world_target_to_current()
        self.plot_data()

    def get_active_servo_ids(self):
        servo_ids = []
        for m in self.modules:
            if m["fixed"] or m["type"] == "none":
                continue
            servo_id = int(m.get("servo_id", 0))
            if 1 <= servo_id <= MAX_SERVO_ID and servo_id not in servo_ids:
                servo_ids.append(servo_id)
        return servo_ids

    def get_selected_servo_ids(self):
        servo_ids = []
        for i, m in enumerate(self.modules):
            if m["fixed"] or m["type"] == "none":
                continue
            if not self.control_rows[i]["select_box"].isChecked():
                continue
            servo_id = int(m.get("servo_id", 0))
            if 1 <= servo_id <= MAX_SERVO_ID and servo_id not in servo_ids:
                servo_ids.append(servo_id)
        return servo_ids

    def send(self):
        try:
            if not self.uart_ready:
                print("[WARN] UART not ready")
                self.canStatusLbl.setText("UART: not ready")
                return

            servo_packets = []
            sent = 0

            for m in self.modules:
                if m["fixed"] or m["type"] == "none":
                    continue

                servo_id = int(m.get("servo_id", 0))
                if not (1 <= servo_id <= MAX_SERVO_ID):
                    print(f"[WARN] Skipping {m['name']}: invalid servo_id={servo_id}")
                    continue

                servo_deg = joint_to_servo_deg(m)
                if servo_deg is None:
                    print(f"[WARN] Skipping {m['name']}: no servo calibration")
                    continue

                pos = servo_deg_to_uart_count(m, servo_deg)
                servo_packets.extend([servo_id, pos])
                sent += 1
                print(f"[SEND] {m['name']} id={servo_id} angle={servo_deg:.1f} deg pos={pos} time={DEFAULT_MOVE_MS} ms")

            if sent == 1:
                ServoControl.setBusServoMove(servo_packets[0], servo_packets[1], DEFAULT_MOVE_MS)
            elif sent > 1:
                ServoControl.setMoreBusServoMove(servo_packets, sent, DEFAULT_MOVE_MS)

            self.canStatusLbl.setText(f"UART: sent {sent} servo command(s)")
        except Exception as e:
            self.canStatusLbl.setText(f"UART: send failed ({e})")
            print("Send error:", e)

    def read_current_angles(self):
        try:
            if not self.uart_ready:
                self.canStatusLbl.setText("UART: not ready")
                return

            servo_ids = self.get_active_servo_ids()
            if not servo_ids:
                self.canStatusLbl.setText("UART: no valid servo IDs configured")
                return

            readback = ServoControl.getControllerServoAngles(servo_ids)
            if not readback:
                self.canStatusLbl.setText("UART: no angle response")
                return

            updated = 0
            for m in self.modules:
                if m["fixed"] or m["type"] == "none":
                    continue

                servo_id = int(m.get("servo_id", 0))
                if servo_id not in readback:
                    continue

                servo_pos = float(readback[servo_id])
                if m["type"] == "swivel":
                    servo_deg = map_range(servo_pos, 0.0, 1000.0, 0.0, 360.0)
                else:
                    servo_deg = map_range(servo_pos, 0.0, 1000.0, 0.0, 240.0)

                joint_rad = servo_deg_to_joint_rad(m, servo_deg)
                if joint_rad is None:
                    continue

                m["q"] = joint_rad
                updated += 1

            self.refresh_all_control_rows()
            self.sync_world_target_to_current()
            self.plot_data()
            self.canStatusLbl.setText(f"UART: read {updated} servo angle(s)")
            print("[READ]", readback)
        except Exception as e:
            self.canStatusLbl.setText(f"UART: read failed ({e})")
            print("Read Angles error:", e)

    def power_off_selected_servos(self):
        try:
            if not self.uart_ready:
                self.canStatusLbl.setText("UART: not ready")
                return

            servo_ids = self.get_selected_servo_ids()
            if not servo_ids:
                self.canStatusLbl.setText("UART: no selected servos")
                return

            ServoControl.setMultiServoUnload(servo_ids)
            self.canStatusLbl.setText(f"UART: powered off selected {servo_ids}")
            print("[POWER OFF SELECTED]", servo_ids)
        except Exception as e:
            self.canStatusLbl.setText(f"UART: power off selected failed ({e})")
            print("Power Off Selected error:", e)

    def power_off_all_servos(self):
        try:
            if not self.uart_ready:
                self.canStatusLbl.setText("UART: not ready")
                return

            servo_ids = self.get_active_servo_ids()
            if not servo_ids:
                self.canStatusLbl.setText("UART: no valid servo IDs")
                return

            ServoControl.setMultiServoUnload(servo_ids)
            self.canStatusLbl.setText(f"UART: powered off all {servo_ids}")
            print("[POWER OFF ALL]", servo_ids)
        except Exception as e:
            self.canStatusLbl.setText(f"UART: power off all failed ({e})")
            print("Power Off All error:", e)

    def capture_current_pose(self):
        pose = {"name": f"Pose {len(self.saved_poses) + 1}", "joints": []}
        for m in self.modules:
            pose["joints"].append({
                "name": m["name"],
                "q": float(m["q"]),
                "servo_id": int(m.get("servo_id", 0)),
                "servo_offset_deg": float(m.get("servo_offset_deg", 0.0)),
            })
        self.saved_poses.append(pose)
        self.refresh_pose_list()
        self.canStatusLbl.setText(f"Saved {pose['name']}")

    def refresh_pose_list(self):
        if not hasattr(self, "poseList"):
            return
        self.poseList.blockSignals(True)
        self.poseList.clear()
        for pose in self.saved_poses:
            self.poseList.addItem(pose["name"])
        self.poseList.blockSignals(False)

    def go_to_selected_pose(self):
        row = self.poseList.currentRow()
        if row < 0 or row >= len(self.saved_poses):
            self.canStatusLbl.setText("Pose: no saved pose selected")
            return

        pose = self.saved_poses[row]
        joint_map = {j["name"]: j["q"] for j in pose["joints"]}

        for m in self.modules:
            if m["name"] in joint_map and not m["fixed"] and m["type"] != "none":
                qmin, qmax = m["qlim"]
                m["q"] = clamp(joint_map[m["name"]], qmin, qmax)

        self.refresh_all_control_rows()
        self.sync_world_target_to_current()
        self.plot_data()
        self.canStatusLbl.setText(f"Loaded {pose['name']}")

    def overwrite_selected_pose(self):
        row = self.poseList.currentRow()
        if row < 0 or row >= len(self.saved_poses):
            self.canStatusLbl.setText("Pose: no saved pose selected")
            return

        pose = self.saved_poses[row]
        pose["joints"] = []
        for m in self.modules:
            pose["joints"].append({
                "name": m["name"],
                "q": float(m["q"]),
                "servo_id": int(m.get("servo_id", 0)),
                "servo_offset_deg": float(m.get("servo_offset_deg", 0.0)),
            })

        self.refresh_pose_list()
        self.poseList.setCurrentRow(row)
        self.canStatusLbl.setText(f"Updated {pose['name']}")

    def delete_selected_pose(self):
        row = self.poseList.currentRow()
        if row < 0 or row >= len(self.saved_poses):
            self.canStatusLbl.setText("Pose: no saved pose selected")
            return

        name = self.saved_poses[row]["name"]
        del self.saved_poses[row]
        self.refresh_pose_list()
        self.canStatusLbl.setText(f"Deleted {name}")

    def plot_data(self):
        try:
            X, Y, Z, pts, point_names, frames = forward_kin(self.modules)

            targets = []
            if hasattr(self.window, "ax") and hasattr(self.window, "canvas"):
                targets.append((self.window.ax, self.window.canvas, self.window.fig, True))
            if hasattr(self.window, "struct_ax") and hasattr(self.window, "struct_canvas"):
                targets.append((self.window.struct_ax, self.window.struct_canvas, self.window.struct_fig, False))
            if hasattr(self.window, "cfg_ax") and hasattr(self.window, "cfg_canvas"):
                targets.append((self.window.cfg_ax, self.window.cfg_canvas, self.window.cfg_fig, False))

            for ax, canvas, fig, show_world_target in targets:
                prev_xlim = ax.get_xlim()
                prev_ylim = ax.get_ylim()
                prev_zlim = ax.get_zlim()

                ax.cla()
                style_3d_axes(ax, show_plot_axes=self.show_plot_axes)

                ax.plot(X, Y, Z, color='#60a5fa', marker='o', linewidth=2.2, markersize=5, label='Arm')
                ax.scatter([X[-1]], [Y[-1]], [Z[-1]], c='#f87171', s=80, label='End-effector')
                draw_frame_axes(ax, frames, show_axes_frames=self.show_frame_axes)

                for i, (x, y, z) in enumerate(pts):
                    ax.text(x, y, z + 0.02, point_names[i], fontsize=8, color='#e5e7eb')

                if show_world_target and self.control_mode == "World":
                    ax.scatter(
                        [self.world_target[0]], [self.world_target[1]], [self.world_target[2]],
                        c='#fbbf24', s=45, marker='x'
                    )

                if self.view_needs_refit:
                    auto_fit_axes_cube(ax, X, Y, Z, scale=1.5, min_cube=0.4)
                else:
                    ax.set_xlim(prev_xlim)
                    ax.set_ylim(prev_ylim)
                    ax.set_zlim(prev_zlim)
                    ax.set_box_aspect((1, 1, 1))

                ax.view_init(elev=22, azim=-58)

                leg = ax.legend(facecolor="#111827", edgecolor="#475569")
                for txt in leg.get_texts():
                    txt.set_color("#e5e7eb")

                fig.tight_layout()
                canvas.draw()

            self.view_needs_refit = False
            self.posLbl.setText(f"End-effector: X={X[-1]:.3f}, Y={Y[-1]:.3f}, Z={Z[-1]:.3f}")
        except Exception as e:
            print("Plot error:", e)


def main():
    app = QtWidgets.QApplication(sys.argv)
    apply_app_style(app)
    win = MainWindow()
    ui = ModularJointUI()
    ui.setupUi(win)
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
