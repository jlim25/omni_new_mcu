# -*- coding: utf-8 -*-
# MERGED: PyQt5 ARM CONTROL + VARIABLE-LENGTH INVERSE KINEMATICS
# Features: Real-time pose control, 3D visualization, configurable DH parameters

from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2, Matrix

# ========== CONFIGURABLE DH PARAMETERS FOR VARIABLE ARM LENGTHS ==========
DH = [  # Modified DH for KUKA KR210 (defaults); change 'a'/'d' for custom lengths
    {'a': 0.0,   'alpha': 0.0,     'd': 0,   'theta_offset': 0.0},     # Joint 1
    {'a': 0.8,  'alpha': -pi/2,   'd': 0.0,    'theta_offset': -pi/2},   # Joint 2
    {'a': 0.75,  'alpha': 0.0,     'd': 0.0,    'theta_offset': 0.0},     # Joint 3 (elbow)
    {'a': -0.054,'alpha': -pi/2,   'd': 1.5,    'theta_offset': 0.0},     # Joint 4 (wrist1)
    {'a': 0.0,   'alpha': pi/2,    'd': 0.0,    'theta_offset': 0.0},     # Joint 5 (wrist2)
    {'a': 0.0,   'alpha': -pi/2,   'd': 0.0,    'theta_offset': 0.0},     # Joint 6 (wrist3)
    {'a': 0.0,   'alpha': 0.0,     'd': 0.303,  'theta_offset': 0.0}      # Gripper offset
]

# ========== FORWARD/INVERSE KINEMATICS FUNCTIONS ==========

def pose(theta, alpha, a, d):
    """DH transformation matrix from i-1 to i frame."""
    r11, r12 = cos(theta), -sin(theta)
    r23, r33 = -sin(alpha), cos(alpha)
    r21 = sin(theta) * cos(alpha)
    r22 = cos(theta) * cos(alpha)
    r31 = sin(theta) * sin(alpha)
    r32 = cos(theta) * sin(alpha)
    y = -d * sin(alpha)
    z = d * cos(alpha)
    
    T = Matrix([
        [r11, r12, 0.0, a],
        [r21, r22, r23, y],
        [r31, r32, r33, z],
        [0.0, 0.0, 0.0, 1]
    ])
    return simplify(T)

def forward_kin(q1, q2, q3, q4, q5, q6, DH):
    """Forward kinematics: joint positions for visualization."""
    X, Y, Z = [], [], []
    T0g = Matrix.eye(4)
    qs = [q1, q2, q3, q4, q5, q6]
    
    for i in range(6):
        theta = qs[i] + DH[i]['theta_offset']
        T = pose(theta, DH[i]['alpha'], DH[i]['a'], DH[i]['d'])
        T0g = T0g * T
        px, py, pz = float(T0g[0,3]), float(T0g[1,3]), float(T0g[2,3])
        X.append(px); Y.append(py); Z.append(pz)
    
    # Gripper (link 6-7)
    theta = DH[6]['theta_offset']
    T6g = pose(theta, DH[6]['alpha'], DH[6]['a'], DH[6]['d'])
    T0g = T0g * T6g
    px, py, pz = float(T0g[0,3]), float(T0g[1,3]), float(T0g[2,3])
    X.append(px); Y.append(py); Z.append(pz)
    
    X = np.reshape(X, (1, 7)); Y = np.reshape(Y, (1, 7)); Z = np.reshape(Z, (1, 7))
    return X, Y, Z

def create_plot():
    """Create 3D matplotlib figure for arm visualization."""
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.set_xlim3d([-0.5, 2]); ax.set_ylim3d([-1.5, 1.5]); ax.set_zlim3d([0, 3.5])
    ax.set_autoscale_on(False)
    return fig, ax

def get_hypotenuse(a, b):
    """Pythagorean theorem: hypotenuse of right triangle."""
    return sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):
    """Cosine law angle between sides a and b."""
    cos_gamma = (a*a + b*b - c*c) / (2*a*b)
    sin_gamma = sqrt(1 - cos_gamma * cos_gamma)
    return atan2(sin_gamma, cos_gamma)

def get_wrist_center(gripper_point, R0g, DH):
    """Wrist center from gripper pose (variable dg)."""
    dg = DH[6]['d']
    xu, yu, zu = gripper_point
    nx, ny, nz = float(R0g[0, 2]), float(R0g[1, 2]), float(R0g[2, 2])
    return xu - dg * nx, yu - dg * ny, zu - dg * nz

def get_first_three_angles(wrist_center, DH):
    """q1,q2,q3 from wrist center (variable geometry)."""
    x, y, z = wrist_center
    a1, a2, a3 = DH[1]['a'], DH[2]['a'], DH[3]['a']
    d1, d4 = DH[0]['d'], DH[3]['d']
    l = sqrt(d4**2 + abs(a3)**2)
    phi = atan2(d4, abs(a3))
    
    x_prime = sqrt(x**2 + y**2)
    mx = x_prime - a1
    my = z - d1
    m = sqrt(mx**2 + my**2)
    alpha = atan2(my, mx)
    
    gamma = get_cosine_law_angle(l, a2, m)
    beta = get_cosine_law_angle(m, a2, l)
    
    q1 = atan2(y, x)
    q2 = pi/2 - beta - alpha
    q3 = -(gamma - phi)
    return q1, q2, q3

def get_last_three_angles(R):
    """q4,q5,q6 from R36 (wrist orientation)."""
    sin_q4 = R[2, 2]
    cos_q4 = -R[0, 2]
    sin_q5 = sqrt(R[0, 2]**2 + R[2, 2]**2)
    cos_q5 = R[1, 2]
    sin_q6 = -R[1, 1]
    cos_q6 = R[1, 0]
    q4 = atan2(sin_q4, cos_q4)
    q5 = atan2(sin_q5, cos_q5)
    q6 = atan2(sin_q6, cos_q6)
    return q4, q5, q6

def get_angles(x, y, z, roll, pitch, yaw, DH):
    """Full IK: position + Euler orientation -> joint angles."""
    gripper_point = x, y, z
    
    # Symbolic matrices
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')
    alpha, beta, gamma = symbols('alpha beta gamma', real=True)
    px, py, pz = symbols('px py pz', real=True)
    
    R03 = Matrix([
        [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
        [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3), cos(q1)],
        [cos(q2 + q3), -sin(q2 + q3), 0]])
    R03T = R03.T
    
    R36 = Matrix([
        [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
        [sin(q5)*cos(q6), -sin(q5)*sin(q6), cos(q5)],
        [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4), sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6), sin(q4)*sin(q5)]])
    
    # R0u: gripper rotation (yaw=alpha, pitch=beta, roll=gamma)
    R0u = Matrix([
        [cos(alpha)*cos(beta), -sin(alpha)*cos(gamma) + sin(beta)*sin(gamma)*cos(alpha), sin(alpha)*sin(gamma) + sin(beta)*cos(alpha)*cos(gamma)],
        [sin(alpha)*cos(beta), sin(alpha)*sin(beta)*sin(gamma) + cos(alpha)*cos(gamma), sin(alpha)*sin(beta)*cos(gamma) - sin(gamma)*cos(alpha)],
        [-sin(beta), sin(gamma)*cos(beta), cos(beta)*cos(gamma)]])
    
    RguT_eval = Matrix([[0, 0, 1], [0, -1, 0], [1, 0, 0]])  # Fixed gripper transform
    
    # Evaluate for given pose
    R0u_eval = R0u.evalf(subs={alpha: yaw, beta: pitch, gamma: roll})
    R0g_eval = R0u_eval * RguT_eval
    
    wrist_center = get_wrist_center(gripper_point, R0g_eval, DH)
    j1, j2, j3 = get_first_three_angles(wrist_center, DH)
    
    R03T_eval = R03T.evalf(subs={q1: j1.evalf(), q2: j2.evalf(), q3: j3.evalf()})
    R36_eval = R03T_eval * R0g_eval
    
    j4, j5, j6 = get_last_three_angles(R36_eval)
    
    return [j.evalf() for j in [j1, j2, j3, j4, j5, j6]]

# ========== PyQt5 UI CLASS ==========

class Ui_ArmControl(object):
    def setupUi(self, ArmControl):
        ArmControl.setObjectName("ArmControl")
        ArmControl.setWindowModality(QtCore.Qt.NonModal)
        ArmControl.resize(482, 537)
        ArmControl.setMaximumSize(QtCore.QSize(16777215, 16777215))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("../roboticArm.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        ArmControl.setWindowIcon(icon)
        ArmControl.setWindowOpacity(1.0)
        ArmControl.setAutoFillBackground(False)
        
        self.xUp = QtWidgets.QPushButton(ArmControl)
        self.xUp.setGeometry(QtCore.QRect(360, 130, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(16)
        self.xUp.setFont(font)
        self.xUp.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.xUp.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("up.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.xUp.setIcon(icon1)
        self.xUp.setIconSize(QtCore.QSize(32, 32))
        self.xUp.setFlat(True)
        self.xUp.setObjectName("xUp")
        
        self.xVal = QtWidgets.QLabel(ArmControl)
        self.xVal.setGeometry(QtCore.QRect(260, 130, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(14)
        self.xVal.setFont(font)
        self.xVal.setFrameShape(QtWidgets.QFrame.Box)
        self.xVal.setFrameShadow(QtWidgets.QFrame.Raised)
        self.xVal.setLineWidth(1)
        self.xVal.setAlignment(QtCore.Qt.AlignCenter)
        self.xVal.setObjectName("xVal")
        self.xVal.setText("0.4")
        
        self.yVal = QtWidgets.QLabel(ArmControl)
        self.yVal.setGeometry(QtCore.QRect(260, 190, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(14)
        self.yVal.setFont(font)
        self.yVal.setFrameShape(QtWidgets.QFrame.Box)
        self.yVal.setFrameShadow(QtWidgets.QFrame.Raised)
        self.yVal.setLineWidth(1)
        self.yVal.setAlignment(QtCore.Qt.AlignCenter)
        self.yVal.setObjectName("yVal")
        self.yVal.setText("1.3")
        
        self.rollVal = QtWidgets.QLabel(ArmControl)
        self.rollVal.setGeometry(QtCore.QRect(260, 310, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(14)
        self.rollVal.setFont(font)
        self.rollVal.setFrameShape(QtWidgets.QFrame.Box)
        self.rollVal.setFrameShadow(QtWidgets.QFrame.Raised)
        self.rollVal.setLineWidth(1)
        self.rollVal.setAlignment(QtCore.Qt.AlignCenter)
        self.rollVal.setObjectName("rollVal")
        self.rollVal.setText("1")
        
        self.zVal = QtWidgets.QLabel(ArmControl)
        self.zVal.setGeometry(QtCore.QRect(260, 250, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(14)
        self.zVal.setFont(font)
        self.zVal.setFrameShape(QtWidgets.QFrame.Box)
        self.zVal.setFrameShadow(QtWidgets.QFrame.Raised)
        self.zVal.setLineWidth(1)
        self.zVal.setAlignment(QtCore.Qt.AlignCenter)
        self.zVal.setObjectName("zVal")
        self.zVal.setText("3")
        
        self.xLabel = QtWidgets.QLabel(ArmControl)
        self.xLabel.setGeometry(QtCore.QRect(80, 130, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(12)
        self.xLabel.setFont(font)
        self.xLabel.setStyleSheet("background-color: rgb(237, 255, 248)")
        self.xLabel.setFrameShape(QtWidgets.QFrame.Panel)
        self.xLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.xLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.xLabel.setObjectName("xLabel")
        self.xLabel.setText("X")
        
        self.yUp = QtWidgets.QPushButton(ArmControl)
        self.yUp.setGeometry(QtCore.QRect(360, 190, 41, 41))
        self.yUp.setFont(font)
        self.yUp.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.yUp.setText("")
        self.yUp.setIcon(icon1)
        self.yUp.setIconSize(QtCore.QSize(32, 32))
        self.yUp.setFlat(True)
        self.yUp.setObjectName("yUp")
        
        self.rollup = QtWidgets.QPushButton(ArmControl)
        self.rollup.setGeometry(QtCore.QRect(360, 310, 41, 41))
        self.rollup.setFont(font)
        self.rollup.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.rollup.setText("")
        self.rollup.setIcon(icon1)
        self.rollup.setIconSize(QtCore.QSize(32, 32))
        self.rollup.setFlat(True)
        self.rollup.setObjectName("rollup")
        
        self.zUp = QtWidgets.QPushButton(ArmControl)
        self.zUp.setGeometry(QtCore.QRect(360, 250, 41, 41))
        self.zUp.setFont(font)
        self.zUp.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.zUp.setText("")
        self.zUp.setIcon(icon1)
        self.zUp.setIconSize(QtCore.QSize(32, 32))
        self.zUp.setFlat(True)
        self.zUp.setObjectName("zUp")
        
        self.xDown = QtWidgets.QPushButton(ArmControl)
        self.xDown.setGeometry(QtCore.QRect(200, 130, 41, 41))
        self.xDown.setFont(font)
        self.xDown.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.xDown.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("down.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.xDown.setIcon(icon2)
        self.xDown.setIconSize(QtCore.QSize(32, 32))
        self.xDown.setFlat(True)
        self.xDown.setObjectName("xDown")
        
        self.yDown = QtWidgets.QPushButton(ArmControl)
        self.yDown.setGeometry(QtCore.QRect(200, 190, 41, 41))
        self.yDown.setFont(font)
        self.yDown.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.yDown.setText("")
        self.yDown.setIcon(icon2)
        self.yDown.setIconSize(QtCore.QSize(32, 32))
        self.yDown.setFlat(True)
        self.yDown.setObjectName("yDown")
        
        self.zDown = QtWidgets.QPushButton(ArmControl)
        self.zDown.setGeometry(QtCore.QRect(200, 250, 41, 41))
        self.zDown.setFont(font)
        self.zDown.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.zDown.setText("")
        self.zDown.setIcon(icon2)
        self.zDown.setIconSize(QtCore.QSize(32, 32))
        self.zDown.setFlat(True)
        self.zDown.setObjectName("zDown")
        
        self.rollDown = QtWidgets.QPushButton(ArmControl)
        self.rollDown.setGeometry(QtCore.QRect(200, 310, 41, 41))
        self.rollDown.setFont(font)
        self.rollDown.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.rollDown.setText("")
        self.rollDown.setIcon(icon2)
        self.rollDown.setIconSize(QtCore.QSize(32, 32))
        self.rollDown.setFlat(True)
        self.rollDown.setObjectName("rollDown")
        
        self.yLabel = QtWidgets.QLabel(ArmControl)
        self.yLabel.setGeometry(QtCore.QRect(80, 190, 81, 41))
        self.yLabel.setFont(font)
        self.yLabel.setStyleSheet("background-color: rgb(237, 255, 248)")
        self.yLabel.setFrameShape(QtWidgets.QFrame.Panel)
        self.yLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.yLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.yLabel.setObjectName("yLabel")
        self.yLabel.setText("Y")
        
        self.zLabel = QtWidgets.QLabel(ArmControl)
        self.zLabel.setGeometry(QtCore.QRect(80, 250, 81, 41))
        self.zLabel.setFont(font)
        self.zLabel.setStyleSheet("background-color: rgb(237, 255, 248)")
        self.zLabel.setFrameShape(QtWidgets.QFrame.Panel)
        self.zLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.zLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.zLabel.setObjectName("zLabel")
        self.zLabel.setText("Z")
        
        self.rollLabel = QtWidgets.QLabel(ArmControl)
        self.rollLabel.setGeometry(QtCore.QRect(80, 310, 81, 41))
        self.rollLabel.setFont(font)
        self.rollLabel.setStyleSheet("background-color: rgb(237, 255, 248)")
        self.rollLabel.setFrameShape(QtWidgets.QFrame.Panel)
        self.rollLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.rollLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.rollLabel.setObjectName("rollLabel")
        self.rollLabel.setText("Roll")
        
        self.titleLabel = QtWidgets.QLabel(ArmControl)
        self.titleLabel.setGeometry(QtCore.QRect(30, 40, 161, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(20)
        self.titleLabel.setFont(font)
        self.titleLabel.setObjectName("titleLabel")
        self.titleLabel.setText("Arm Control")
        
        self.sendButton = QtWidgets.QPushButton(ArmControl)
        self.sendButton.setGeometry(QtCore.QRect(350, 30, 41, 51))
        self.sendButton.setText("")
        self.sendButton.setIcon(icon)
        self.sendButton.setIconSize(QtCore.QSize(32, 32))
        self.sendButton.setFlat(True)
        self.sendButton.setObjectName("sendButton")
        
        self.closeButton = QtWidgets.QPushButton(ArmControl)
        self.closeButton.setGeometry(QtCore.QRect(400, 30, 51, 51))
        self.closeButton.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("../closeIcon.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.closeButton.setIcon(icon3)
        self.closeButton.setIconSize(QtCore.QSize(48, 48))
        self.closeButton.setFlat(True)
        self.closeButton.setObjectName("closeButton")
        
        self.pitchVal = QtWidgets.QLabel(ArmControl)
        self.pitchVal.setGeometry(QtCore.QRect(260, 370, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(14)
        self.pitchVal.setFont(font)
        self.pitchVal.setFrameShape(QtWidgets.QFrame.Box)
        self.pitchVal.setFrameShadow(QtWidgets.QFrame.Raised)
        self.pitchVal.setLineWidth(1)
        self.pitchVal.setAlignment(QtCore.Qt.AlignCenter)
        self.pitchVal.setObjectName("pitchVal")
        self.pitchVal.setText("-1")
        
        self.pitchDown = QtWidgets.QPushButton(ArmControl)
        self.pitchDown.setGeometry(QtCore.QRect(200, 370, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(16)
        self.pitchDown.setFont(font)
        self.pitchDown.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.pitchDown.setText("")
        self.pitchDown.setIcon(icon2)
        self.pitchDown.setIconSize(QtCore.QSize(32, 32))
        self.pitchDown.setFlat(True)
        self.pitchDown.setObjectName("pitchDown")
        
        self.pitchLabel = QtWidgets.QLabel(ArmControl)
        self.pitchLabel.setGeometry(QtCore.QRect(80, 370, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(12)
        self.pitchLabel.setFont(font)
        self.pitchLabel.setStyleSheet("background-color: rgb(237, 255, 248)")
        self.pitchLabel.setFrameShape(QtWidgets.QFrame.Panel)
        self.pitchLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.pitchLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.pitchLabel.setObjectName("pitchLabel")
        self.pitchLabel.setText("Pitch")
        
        self.pitchUp = QtWidgets.QPushButton(ArmControl)
        self.pitchUp.setGeometry(QtCore.QRect(360, 370, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(16)
        self.pitchUp.setFont(font)
        self.pitchUp.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.pitchUp.setText("")
        self.pitchUp.setIcon(icon1)
        self.pitchUp.setIconSize(QtCore.QSize(32, 32))
        self.pitchUp.setFlat(True)
        self.pitchUp.setObjectName("pitchUp")
        
        self.yawVal = QtWidgets.QLabel(ArmControl)
        self.yawVal.setGeometry(QtCore.QRect(260, 430, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(14)
        self.yawVal.setFont(font)
        self.yawVal.setFrameShape(QtWidgets.QFrame.Box)
        self.yawVal.setFrameShadow(QtWidgets.QFrame.Raised)
        self.yawVal.setLineWidth(1)
        self.yawVal.setAlignment(QtCore.Qt.AlignCenter)
        self.yawVal.setObjectName("yawVal")
        self.yawVal.setText("-1")
        
        self.yawDown = QtWidgets.QPushButton(ArmControl)
        self.yawDown.setGeometry(QtCore.QRect(200, 430, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(16)
        self.yawDown.setFont(font)
        self.yawDown.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.yawDown.setText("")
        self.yawDown.setIcon(icon2)
        self.yawDown.setIconSize(QtCore.QSize(32, 32))
        self.yawDown.setFlat(True)
        self.yawDown.setObjectName("yawDown")
        
        self.yawLabel = QtWidgets.QLabel(ArmControl)
        self.yawLabel.setGeometry(QtCore.QRect(80, 430, 81, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(12)
        self.yawLabel.setFont(font)
        self.yawLabel.setStyleSheet("background-color: rgb(237, 255, 248)")
        self.yawLabel.setFrameShape(QtWidgets.QFrame.Panel)
        self.yawLabel.setFrameShadow(QtWidgets.QFrame.Raised)
        self.yawLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.yawLabel.setObjectName("yawLabel")
        self.yawLabel.setText("Yaw")
        
        self.yawUp = QtWidgets.QPushButton(ArmControl)
        self.yawUp.setGeometry(QtCore.QRect(360, 430, 41, 41))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(16)
        self.yawUp.setFont(font)
        self.yawUp.setStyleSheet("color : rgb(255, 255, 255);\n")
        self.yawUp.setText("")
        self.yawUp.setIcon(icon1)
        self.yawUp.setIconSize(QtCore.QSize(32, 32))
        self.yawUp.setFlat(True)
        self.yawUp.setObjectName("yawUp")
        
        self.subtitleLabel = QtWidgets.QLabel(ArmControl)
        self.subtitleLabel.setGeometry(QtCore.QRect(190, 10, 131, 21))
        font = QtGui.QFont()
        font.setFamily("Roboto Slab")
        font.setPointSize(10)
        self.subtitleLabel.setFont(font)
        self.subtitleLabel.setObjectName("subtitleLabel")
        self.subtitleLabel.setText("Team Avijatrik")
        
        # Connect signals
        self.closeButton.clicked.connect(ArmControl.close)
        self.sendButton.clicked.connect(self.send)
        self.xDown.clicked.connect(self.xDownMethod)
        self.xUp.clicked.connect(self.xUpMethod)
        self.yDown.clicked.connect(self.yDownMethod)
        self.yUp.clicked.connect(self.yUpMethod)
        self.zDown.clicked.connect(self.zDownMethod)
        self.zUp.clicked.connect(self.zUpMethod)
        self.rollDown.clicked.connect(self.rollDownMethod)
        self.rollup.clicked.connect(self.rollupMethod)
        self.pitchDown.clicked.connect(self.pitchDownMethod)
        self.pitchUp.clicked.connect(self.pitchUpMethod)
        self.yawDown.clicked.connect(self.yawDownMethod)
        self.yawUp.clicked.connect(self.yawUpMethod)
        
        self.window = ArmControl

    def send(self):
        """Send joint angles to hardware."""
        try:
            px = float(self.xVal.text())
            py = float(self.yVal.text())
            pz = float(self.zVal.text())
            roll = float(self.rollVal.text())
            pitch = float(self.pitchVal.text())
            yaw = float(self.yawVal.text())
            
            q = get_angles(px, py, pz, roll, pitch, yaw, DH)
            data = [f"{float(qi):.3f}" for qi in q]
            dataString = ','.join(data)
            print(f"SEND: {dataString}")
            # Uncomment to send to SerialCom:
            # self.window.SerialObj.sendString(dataString)
        except Exception as e:
            print(f"Send error: {e}")

    def plot_data(self):
        """Compute IK and update 3D visualization."""
        try:
            # Read UI values
            px = float(self.xVal.text())
            py = float(self.yVal.text())
            pz = float(self.zVal.text())
            roll = float(self.rollVal.text())
            pitch = float(self.pitchVal.text())
            yaw = float(self.yawVal.text())
            
            print(f"\n--- IK Computation ---")
            print(f"Target pose: ({px:.2f}, {py:.2f}, {pz:.2f}) RPY({roll:.2f}, {pitch:.2f}, {yaw:.2f})")
            
            # Compute joint angles
            q = get_angles(px, py, pz, roll, pitch, yaw, DH)
            q1, q2, q3, q4, q5, q6 = q
            print(f"IK joints: q1={q1:.3f}, q2={q2:.3f}, q3={q3:.3f}, q4={q4:.3f}, q5={q5:.3f}, q6={q6:.3f}")
            
            # Verify with forward kinematics
            X, Y, Z = forward_kin(q1, q2, q3, q4, q5, q6, DH)
            print(f"FK verify: ({X[0,6]:.3f}, {Y[0,6]:.3f}, {Z[0,6]:.3f})")
            
            # Update 3D plot
            if hasattr(self, 'window') and hasattr(self.window, 'ax'):
                self.window.ax.cla()
                self.window.ax.plot(X[0], Y[0], Z[0], 'b-o', linewidth=2, markersize=6, label='Arm')
                self.window.ax.scatter([X[0, -1]], [Y[0, -1]], [Z[0, -1]], c='r', s=100, label='End-effector')
                self.window.ax.set_xlabel('X (m)'); self.window.ax.set_ylabel('Y (m)'); self.window.ax.set_zlabel('Z (m)')
                self.window.ax.set_xlim([-0.5, 2]); self.window.ax.set_ylim([-1.5, 1.5]); self.window.ax.set_zlim([0, 3.5])
                self.window.ax.legend()
                self.window.fig.canvas.draw()
                
        except ValueError as e:
            print(f"Value error: {e}")
        except Exception as e:
            print(f"IK failed: {e}")

    def xUpMethod(self):
        val = float(self.xVal.text())
        if val < 2:
            val += 0.1
            self.xVal.setText(str(round(val, 2)))
            self.plot_data()

    def xDownMethod(self):
        val = float(self.xVal.text())
        if val > 0:
            val -= 0.1
            self.xVal.setText(str(round(val, 2)))
            self.plot_data()

    def yUpMethod(self):
        val = float(self.yVal.text())
        if val < 3:
            val += 0.1
            self.yVal.setText(str(round(val, 2)))
            self.plot_data()

    def yDownMethod(self):
        val = float(self.yVal.text())
        if val > 0:
            val -= 0.1
            self.yVal.setText(str(round(val, 2)))
            self.plot_data()

    def zUpMethod(self):
        val = float(self.zVal.text())
        if val < 5:
            val += 0.1
            self.zVal.setText(str(round(val, 2)))
            self.plot_data()

    def zDownMethod(self):
        val = float(self.zVal.text())
        if val > 0:
            val -= 0.1
            self.zVal.setText(str(round(val, 2)))
            self.plot_data()

    def rollupMethod(self):
        val = float(self.rollVal.text())
        if val < 3.14:
            val += 0.1
            self.rollVal.setText(str(round(val, 2)))
            self.plot_data()

    def rollDownMethod(self):
        val = float(self.rollVal.text())
        if val > -3.14:
            val -= 0.1
            self.rollVal.setText(str(round(val, 2)))
            self.plot_data()

    def pitchUpMethod(self):
        val = float(self.pitchVal.text())
        if val < 3.14:
            val += 0.1
            self.pitchVal.setText(str(round(val, 2)))
            self.plot_data()

    def pitchDownMethod(self):
        val = float(self.pitchVal.text())
        if val > -3.14:
            val -= 0.1
            self.pitchVal.setText(str(round(val, 2)))
            self.plot_data()

    def yawUpMethod(self):
        val = float(self.yawVal.text())
        if val < 3.14:
            val += 0.1
            self.yawVal.setText(str(round(val, 2)))
            self.plot_data()

    def yawDownMethod(self):
        val = float(self.yawVal.text())
        if val > -3.14:
            val -= 0.1
            self.yawVal.setText(str(round(val, 2)))
            self.plot_data()

# ========== MAIN APPLICATION WINDOW ==========

class Main_window(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.InitPlot()

    def InitPlot(self):
        """Initialize 3D matplotlib figure."""
        self.fig, self.ax = create_plot()
        plt.ion()  # Interactive mode

    def close(self):
        """Close application."""
        sys.exit()

# ========== APPLICATION ENTRY POINT ==========

def main():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = Main_window()
    ui = Ui_ArmControl()
    ui.setupUi(MainWindow)
    MainWindow.show()
    plt.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
