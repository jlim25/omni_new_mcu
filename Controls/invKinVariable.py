from sympy import symbols, cos, sin, pi, simplify, pprint, tan, expand_trig, sqrt, trigsimp, atan2
from sympy.matrices import Matrix
import math
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import time
import numpy as np

# ========== CONFIGURABLE DH PARAMETERS FOR VARIABLE ARM LENGTHS ==========
DH = [  # Modified DH for KUKA KR210 (defaults); change 'a'/'d' for custom lengths
    {'a': 0.0,   'alpha': 0.0,     'd': 0.75,   'theta_offset': 0.0},     # Joint 1
    {'a': 0.35,  'alpha': -pi/2,   'd': 0.0,    'theta_offset': -pi/2},   # Joint 2
    {'a': 1.25,  'alpha': 0.0,     'd': 0.0,    'theta_offset': 0.0},     # Joint 3 (elbow)
    {'a': -0.054,'alpha': -pi/2,   'd': 1.5,    'theta_offset': 0.0},     # Joint 4 (wrist1)
    {'a': 0.0,   'alpha': pi/2,    'd': 0.0,    'theta_offset': 0.0},     # Joint 5 (wrist2)
    {'a': 0.0,   'alpha': -pi/2,   'd': 0.0,    'theta_offset': 0.0},     # Joint 6 (wrist3)
    {'a': 0.0,   'alpha': 0.0,     'd': 0.303,  'theta_offset': 0.0}      # Gripper offset
]
# Example variation: DH[2]['a'] = 1.5  # Make link 3 longer
# DH[6]['d'] = 0.4  # Longer gripper

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

def forward_kin(q1,q2,q3,q4,q5,q6, DH):
    """Forward kinematics: joint positions for visualization."""
    X, Y, Z = [], [], []
    T0g = Matrix.eye(4)
    qs = [q1, q2, q3, q4, q5, q6]
    
    for i in range(6):
        theta = qs[i] + DH[i]['theta_offset']
        T = pose(theta, DH[i]['alpha'], DH[i]['a'], DH[i]['d'])
        T0g = T0g * T
        px,py,pz = float(T0g[0,3]), float(T0g[1,3]), float(T0g[2,3])
        X.append(px); Y.append(py); Z.append(pz)
    
    # Gripper (link 6-7)
    theta = DH[6]['theta_offset']
    T6g = pose(theta, DH[6]['alpha'], DH[6]['a'], DH[6]['d'])
    T0g = T0g * T6g
    px,py,pz = float(T0g[0,3]), float(T0g[1,3]), float(T0g[2,3])
    X.append(px); Y.append(py); Z.append(pz)
    
    X = np.reshape(X,(1,7)); Y = np.reshape(Y,(1,7)); Z = np.reshape(Z,(1,7))
    return X,Y,Z

def create_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_xlim3d([0, 2]); ax.set_ylim3d([0, 3]); ax.set_zlim3d([0, 3])
    ax.set_autoscale_on(False)
    return fig, ax

def update_plot(X,Y,Z,fig,ax):
    X,Y,Z = np.reshape(X,(1,7)), np.reshape(Y,(1,7)), np.reshape(Z,(1,7))
    ax.cla()
    ax.plot_wireframe(X,Y,Z, color='b', alpha=0.7)
    ax.set_xlabel('X'); ax.set_ylabel('Y'); ax.set_zlabel('Z')
    ax.set_xlim3d([0, 2]); ax.set_ylim3d([0, 3]); ax.set_zlim3d([0, 3])
    ax.set_autoscale_on(False)
    fig.canvas.draw()
    fig.canvas.flush_events()

# Euler angle helpers (unchanged)
def eulerAnglesToRotationMatrix(theta):
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]])
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]])
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]])
    return np.dot(R_z, np.dot(R_y, R_x))

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    return np.linalg.norm(I - shouldBeIdentity) < 1e-6

def rotationMatrixToEulerAngles(R):
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else:
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def get_hypotenuse(a, b):
    return sqrt(a*a + b*b)

def get_cosine_law_angle(a, b, c):
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
    """q4,q5,q6 from R36 (unchanged)."""
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
    
    # Symbolic matrices (unchanged)
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')
    alpha, beta, gamma = symbols('alpha beta gamma', real=True)
    px, py, pz = symbols('px py pz', real=True)
    
    R03 = Matrix([
        [sin(q2 + q3)*cos(q1), cos(q1)*cos(q2 + q3), -sin(q1)],
        [sin(q1)*sin(q2 + q3), sin(q1)*cos(q2 + q3), cos(q1)],
        [cos(q2 + q3), -sin(q2 + q3), 0]])
    R03T = R03.T
