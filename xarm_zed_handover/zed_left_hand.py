# zed_left_hand.py (now configured for right-hand detection)
import cv2
import numpy as np
import pyzed.sl as sl

from .config import (
    CONF_THR, EMA_ALPHA, POS_TOL, STABLE_FRAMES_REQUIRED,
    R_cb, t_cb, SAFE_Z_MIN, SAFE_Z_MAX, P2_ORI,
)


def detect_right_hand_stable_then_map_to_P2():
    """
    Detect right hand stable for ≥ N frames, then map to base coordinates (mm).
    返回: P2 = dict(x=..., y=..., z=..., roll=..., pitch=..., yaw=...)
    """
    zed = sl.Camera()
    ip = sl.InitParameters()
    ip.camera_resolution = sl.RESOLUTION.HD720
    ip.camera_fps = 60
    ip.depth_mode = sl.DEPTH_MODE.NEURAL
    ip.coordinate_units = sl.UNIT.METER
    ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    err = zed.open(ip)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(err)

    ptp = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(ptp)

    btp = sl.BodyTrackingParameters()
    btp.enable_tracking = True
    btp.enable_body_fitting = False
    btp.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
    btp.body_format = sl.BODY_FORMAT.BODY_34   # Compatible: BODY_34 keeps right hand index=11
    zed.enable_body_tracking(btp)

    bodies = sl.Bodies()
    brt = sl.BodyTrackingRuntimeParameters()
    brt.detection_confidence_threshold = 40
    rtp = sl.RuntimeParameters()
    image = sl.Mat()

    RH_IDX = 15  # Right hand keypoint index in BODY_34
    ema = None
    last_ema = None
    stable_frames = 0

    print(f"Please extend your right hand and keep it stable for ~{STABLE_FRAMES_REQUIRED / 60:.1f} seconds …")

    try:
        while True:
            if zed.grab(rtp) != sl.ERROR_CODE.SUCCESS:
                if cv2.waitKey(1) == ord('q'):
                    break
                continue

            zed.retrieve_bodies(bodies, brt)
            zed.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()

            if bodies.is_new:
                for body in bodies.body_list:
                    kc = body.keypoint_confidence
                    if len(kc) > RH_IDX and kc[RH_IDX] > CONF_THR:
                        rh = np.array(body.keypoint[RH_IDX], dtype=float)  # m
                        if np.any(np.isnan(rh)):
                            continue

                        # EMA smoothing
                        ema = rh if ema is None else EMA_ALPHA * rh + (1 - EMA_ALPHA) * ema

                        if last_ema is None:
                            last_ema = ema.copy()
                            stable_frames = 1
                        else:
                            diff = float(np.linalg.norm(ema - last_ema))
                            last_ema = ema.copy()
                            if diff <= POS_TOL:
                                stable_frames += 1
                            else:
                                stable_frames = 1  # Reset counter

                        # Overlay text
                        cv2.putText(
                            frame,
                            f"StableFrames: {stable_frames}/{STABLE_FRAMES_REQUIRED}",
                            (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0), 2
                        )
                        cv2.imshow("ZED Right Hand Fast (BODY_34)", frame)
                        cv2.waitKey(1)

                        # Trigger when stable frames reach threshold
                        if stable_frames >= STABLE_FRAMES_REQUIRED:
                            print(f"Right hand stable ：{ema}")
                            # Camera(m) -> Base(m) -> (mm)
                            p_base_m = R_cb @ ema + t_cb
                            x_mm, y_mm, z_mm = 1000 * p_base_m[0], 1000 * p_base_m[1], 1000 * p_base_m[2]
                            z_mm = max(z_mm, SAFE_Z_MIN)
                            P2 = dict(x=x_mm, y=y_mm, z=z_mm, **P2_ORI)
                            print(f"→ converted to base frame P2={P2}")
                            return P2

            # 仍然显示画面，避免窗口卡死
            cv2.imshow("ZED Right Hand Fast (BODY_34)", frame)
            if cv2.waitKey(1) == ord('q'):
                break

    finally:
        try:
            zed.disable_body_tracking()
            zed.disable_positional_tracking()
        except Exception:
            pass
        zed.close()
        cv2.destroyAllWindows()
