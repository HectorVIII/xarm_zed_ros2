# config.py
import numpy as np

# ================= xArm Basic Parameters =================
ROBOT_IP = "192.168.1.225"

MOVE_SPEED = 150
MOVE_ACC = 6000
CLOSE_APPROACH_SPEED = 120  # slower speed for close approach
CLOSE_APPROACH_ACC = 4000
GRIPPER_SPEED = 1500
OPEN_POS = 450    # 85mm
CLOSE_POS = 50
MAX_JOINT_SPEED = 60.0  # deg/s, for safety during handover

# Poses (mm/deg)
P0 = dict(x=494, y=-29.4, z=80,  roll=179.2, pitch=-1.8, yaw=-0.1)   # above tool, ready to grip, avoid shelf collision
P1 = dict(x=494, y=-29.4, z=15,  roll=179.2, pitch=-1.8, yaw=-0.1)   # grip tool
P2_ORI = dict(roll=177.0, pitch=-8.7, yaw=96.4)  # P2 is detected position by camera, orientation fixed

SAFE_Z_MIN, SAFE_Z_MAX = 0.0, 600.0
APPROACH_Z_UP = 50.0    # approach height before/after P2

# ================= Calibration Extrinsics (m) =================

R_cb = np.array([
    [-0.152816883,  0.733177385, -0.662644642],
    [ 0.988175165,  0.104867217, -0.111860223],
    [-0.012523686, -0.671903110, -0.740533165]
], dtype=np.float64)

t_cb = np.array([0.694418397, 0.431523428, 0.596290570], dtype=np.float64)
#RMSE = 404 mm

#R_cb = np.array([
    #[-0.15403992, -0.35803142, 0.92091542],
    #[0.98670485, -0.00685953, 0.16237758],
    #[-0.05181922, 0.93368434, 0.35432798]
#])

#t_cb = np.array([1.01636897, 0.73143413, 0.40464918])
# RMSE = 306.59 mm


"""
# Old extrinsics (kept as comment for record)
R_cb = np.array([
    [ 0.00671984, -0.36547256,  0.93079786],
    [ 0.99966612, -0.02076919, -0.01537194],
    [ 0.02494994,  0.93059037,  0.36521097]
])  # Rotation from camera to base

t_cb = np.array([1.44587977, 0.37286003, 0.32676220])   # Translation from camera to base
"""

# ================= Right Hand Detection Parameters =================
CONF_THR = 0.60          # Confidence threshold
EMA_ALPHA = 0.7          # Smoothing factor
POS_TOL = 0.012          # Stability threshold (meter)
STABLE_FRAMES_REQUIRED = 120  # Consecutive stable frames

# ================= Pull-and-Release Detection Parameters  =================
FT_FORCE_RELEASE_N = 5.0     # N: Force threshold to trigger release
CHECK_PERIOD = 0.03          # seconds
DEBOUNCE_COUNT = 4           # number of consecutive triggers to confirm
ALLOW_TRIGGER_AFTER = 0.5    # seconds to wait before allowing trigger 
