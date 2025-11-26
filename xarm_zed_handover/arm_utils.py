# arm_utils.py
import time

from .config import (
    MOVE_SPEED, MOVE_ACC,
    GRIPPER_SPEED, OPEN_POS, CLOSE_POS,
)

# ------------------ xArm Control ------------------
def recover(arm):
    """Recover from error/warn states and set to position mode."""
    arm.clean_error()
    arm.clean_warn()
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.2)


def move(arm, pose, speed=MOVE_SPEED, acc=MOVE_ACC):
    """Move to an absolute pose (mm/deg)."""
    return arm.set_position(
        x=pose["x"], y=pose["y"], z=pose["z"],
        roll=pose["roll"], pitch=pose["pitch"], yaw=pose["yaw"],
        speed=speed, mvacc=acc, wait=True
    )


def gripper_open(arm):
    """Open gripper to OPEN_POS."""
    arm.set_gripper_speed(GRIPPER_SPEED)
    arm.set_gripper_position(OPEN_POS, wait=True)


def gripper_close(arm):
    """Close gripper to CLOSE_POS."""
    arm.set_gripper_speed(GRIPPER_SPEED)
    arm.set_gripper_position(CLOSE_POS, wait=True)


# FT read with error handling
def read_ft_wrench(arm):
    """
    Safe FT read, returns (code, [Fx, Fy, Fz, Tx, Ty, Tz]) or (1, None)
    """
    try:
        code, data = arm.get_ft_sensor_data()
        if code == 0 and data and len(data) >= 6:
            return 0, data
    except Exception:
        pass  # ignore
    return 1, None
