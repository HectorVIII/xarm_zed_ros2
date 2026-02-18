import threading

import cv2
import numpy as np
import pyzed.sl as sl

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from .config import (
    CONF_THR, EMA_ALPHA, POS_TOL, STABLE_FRAMES_REQUIRED,
    R_cb, t_cb,
)


class ZedRightHandNode(Node):
    def __init__(self):
        super().__init__('zed_right_hand_node')

        # === ROS parameter ===
        self.declare_parameter('topic_name', '/right_hand/point')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PointStamped, topic_name, 10)
        self.get_logger().info(f'Publishing right hand on topic: {topic_name}')

        # === ZED initialize ===
        self.zed = sl.Camera()
        ip = sl.InitParameters()
        ip.camera_resolution = sl.RESOLUTION.HD720
        ip.camera_fps = 60
        ip.depth_mode = sl.DEPTH_MODE.NEURAL
        ip.coordinate_units = sl.UNIT.METER
        ip.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

        err = self.zed.open(ip)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(err)

        ptp = sl.PositionalTrackingParameters()
        self.zed.enable_positional_tracking(ptp)

        btp = sl.BodyTrackingParameters()
        btp.enable_tracking = True
        btp.enable_body_fitting = False
        btp.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_FAST
        btp.body_format = sl.BODY_FORMAT.BODY_34
        self.zed.enable_body_tracking(btp)

        self.bodies = sl.Bodies()
        self.brt = sl.BodyTrackingRuntimeParameters()
        self.brt.detection_confidence_threshold = 40
        self.rtp = sl.RuntimeParameters()
        self.image = sl.Mat()

        # === 手部检测状态 ===
        self.RH_IDX = 15  # BODY_34 right hand index
        self.ema = None
        self.last_ema = None
        self.stable_frames = 0
        self.published_for_this_stable = False

        # 用于显示线程的当前画面（numpy 数组）
        self.current_frame = None
        self.frame_lock = threading.Lock()

        self.get_logger().info(
            f'Please extend your right hand and keep it stable for ~{STABLE_FRAMES_REQUIRED / 60:.1f} seconds …'
        )

        # 60Hz timer：负责 grab + 算法 + 更新 current_frame
        self.timer = self.create_timer(1.0 / 60.0, self.process_frame)

        # 单独的显示线程：负责 imshow + waitKey
        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.viewer_running = True
        self.viewer_thread.start()

    # ==================== ZED 帧处理（逻辑线程） ====================
    def process_frame(self):
        if not rclpy.ok():
            return

        if self.zed.grab(self.rtp) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_bodies(self.bodies, self.brt)
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        frame = self.image.get_data()

        # 默认直接显示当前帧
        display_frame = frame

        if self.bodies.is_new:
            for body in self.bodies.body_list:
                kc = body.keypoint_confidence
                if len(kc) <= self.RH_IDX or kc[self.RH_IDX] <= CONF_THR:
                    continue

                rh = np.array(body.keypoint[self.RH_IDX], dtype=float)  # m
                if np.any(np.isnan(rh)):
                    continue

                # EMA smoothing
                if self.ema is None:
                    self.ema = rh
                else:
                    self.ema = EMA_ALPHA * rh + (1.0 - EMA_ALPHA) * self.ema

                if self.last_ema is None:
                    self.last_ema = self.ema.copy()
                    self.stable_frames = 1
                else:
                    diff = float(np.linalg.norm(self.ema - self.last_ema))
                    self.last_ema = self.ema.copy()
                    if diff <= POS_TOL:
                        self.stable_frames += 1
                    else:
                        self.stable_frames = 1
                        self.published_for_this_stable = False

                # 在图像上叠加 StableFrames 文本
                cv2.putText(
                    frame,
                    f"StableFrames: {self.stable_frames}/{STABLE_FRAMES_REQUIRED}",
                    (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2
                )
                display_frame = frame

                if self.stable_frames >= STABLE_FRAMES_REQUIRED and not self.published_for_this_stable:
                    self.publish_right_hand_point(self.ema)
                    self.published_for_this_stable = True

                # 只用到第一个检测到的 body
                break

        # 更新显示线程用的画面
        with self.frame_lock:
            self.current_frame = display_frame.copy()

    # ==================== 显示线程：imshow + waitKey ====================
    def viewer_loop(self):
        """独立线程：持续显示 latest frame，避免被 ROS 回调阻塞。"""
        while self.viewer_running and rclpy.ok():
            frame = None
            with self.frame_lock:
                if self.current_frame is not None:
                    frame = self.current_frame.copy()

            if frame is not None:
                cv2.imshow("ZED Right Hand (ROS2)", frame)
            key = cv2.waitKey(1)
            # 允许用户按 q 关闭窗口
            if key & 0xFF == ord('q'):
                self.get_logger().info("Viewer window closed by user (q).")
                break

        cv2.destroyAllWindows()

    # ==================== 发布稳定手部点 ====================
    def publish_right_hand_point(self, rh_camera_m):
        """
        rh_camera_m: 右手在 camera 坐标系的位置 (m)
        使用 R_cb, t_cb 变到 base 坐标系，然后发布 PointStamped (单位 m)
        """
        p_base_m = R_cb @ rh_camera_m + t_cb
        x, y, z = p_base_m.tolist()

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published stable right hand point (base frame): x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

    # ==================== 清理 ====================
    def destroy_node(self):
        self.viewer_running = False
        # 给 viewer_loop 一点时间退出
        try:
            self.viewer_thread.join(timeout=1.0)
        except Exception:
            pass

        try:
            self.zed.disable_body_tracking()
            self.zed.disable_positional_tracking()
        except Exception:
            pass
        self.zed.close()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZedRightHandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

