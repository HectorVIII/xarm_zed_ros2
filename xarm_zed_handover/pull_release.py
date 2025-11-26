# pull_release.py
import time

from .config import (
    GRIPPER_SPEED, OPEN_POS,
    FT_FORCE_RELEASE_N, CHECK_PERIOD,
    DEBOUNCE_COUNT, ALLOW_TRIGGER_AFTER,
)
from .arm_utils import read_ft_wrench


def detect_pull_then_release(arm):
    """
    使用力传感器检测“拉扯”动作：
    |F| 连续 DEBOUNCE_COUNT 次超过阈值 FT_FORCE_RELEASE_N 时，打开夹爪并返回 True。
    """
    print("[INFO] Waiting for pull trigger (FT mode)...")

    # Enable FT and zero if available
    try:
        arm.ft_sensor_enable(True)
        if hasattr(arm, "ft_sensor_set_zero"):
            arm.ft_sensor_set_zero()
    except Exception as e:
        print(f"[WARN] Failed to enable/zero FT: {e}")

    # Wait before allowing trigger
    time.sleep(ALLOW_TRIGGER_AFTER)

    hits = 0
    while True:
        time.sleep(CHECK_PERIOD)
        code, wrench = read_ft_wrench(arm)
        if code != 0 or wrench is None:
            print("[WARN] FT read failed, retrying...")
            hits = 0
            continue

        Fx, Fy, Fz = wrench[:3]
        F_total = (Fx * Fx + Fy * Fy + Fz * Fz) ** 0.5

        if F_total >= FT_FORCE_RELEASE_N:
            hits += 1
        else:
            hits = 0

        if hits >= DEBOUNCE_COUNT:
            print(f"[TRIGGERED] |F|={F_total:.2f} N ≥ {FT_FORCE_RELEASE_N:.1f} N → opening gripper.")
            arm.set_gripper_speed(GRIPPER_SPEED)
            arm.set_gripper_position(OPEN_POS, wait=True)
            return True
