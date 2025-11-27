# arm_handover/zed_left_hand_node.py

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


class ZedLeftHandNode(Node):
    def __init__(self):
        super().__init__('zed_left_hand_node')

        # 参数：发布的 topic 名字
        self.declare_parameter('topic_name', '/left_hand/point')
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(PointStamped, topic_name, 10)
        self.get_logger().info(f'Publishing left hand on topic: {topic_name}')

        # ==== 初始化 ZED ====
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

        # 滤波 & 稳定检测状态
        self.LH_IDX = 8
        self.ema = None
        self.last_ema = None
        self.stable_frames = 0
        self.published_for_this_stable = False

        self.get_logger().info(
            f'Please extend your left hand and keep it stable for ~{STABLE_FRAMES_REQUIRED / 60:.1f} seconds …'
        )

        # 使用 timer 周期性处理图像（60Hz）
        self.timer = self.create_timer(1.0 / 60.0, self.process_frame)

    def process_frame(self):
        # 让 rclpy 控制退出
        if not rclpy.ok():
            return

        # grab 一帧
        if self.zed.grab(self.rtp) != sl.ERROR_CODE.SUCCESS:
            return

        self.zed.retrieve_bodies(self.bodies, self.brt)
        self.zed.retrieve_image(self.image, sl.VIEW.LEFT)
        frame = self.image.get_data()

        if self.bodies.is_new:
            for body in self.bodies.body_list:
                kc = body.keypoint_confidence
                if len(kc) <= self.LH_IDX or kc[self.LH_IDX] <= CONF_THR:
                    continue

                lh = np.array(body.keypoint[self.LH_IDX], dtype=float)  # m
                if np.any(np.isnan(lh)):
                    continue

                # EMA smoothing
                if self.ema is None:
                    self.ema = lh
                else:
                    self.ema = EMA_ALPHA * lh + (1.0 - EMA_ALPHA) * self.ema

                if self.last_ema is None:
                    self.last_ema = self.ema.copy()
                    self.stable_frames = 1
                else:
                    diff = float(np.linalg.norm(self.ema - self.last_ema))
                    self.last_ema = self.ema.copy()
                    if diff <= POS_TOL:
                        self.stable_frames += 1
                    else:
                        # 手动明显移动了 → 重新计数，并允许再次发布
                        self.stable_frames = 1
                        self.published_for_this_stable = False

                # 画点 + 文本
                cv2.putText(
                    frame,
                    f"StableFrames: {self.stable_frames}/{STABLE_FRAMES_REQUIRED}",
                    (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0), 2
                )
                cv2.imshow("ZED Left Hand (ROS2)", frame)
                cv2.waitKey(1)

                if self.stable_frames >= STABLE_FRAMES_REQUIRED and not self.published_for_this_stable:
                    self.publish_left_hand_point(self.ema)
                    self.published_for_this_stable = True

        else:
            cv2.imshow("ZED Left Hand (ROS2)", frame)
            cv2.waitKey(1)

    def publish_left_hand_point(self, lh_camera_m):
        """
        lh_camera_m: 左手在 camera 坐标系的位置 (m)
        使用 R_cb, t_cb 变到 base 坐标系，然后发布 PointStamped (单位 m)
        """
        p_base_m = R_cb @ lh_camera_m + t_cb
        x, y, z = p_base_m.tolist()

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"   # 或使用你自己的 base frame 名字
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published stable left hand point (base frame): x={x:.3f}, y={y:.3f}, z={z:.3f}"
        )

    def destroy_node(self):
        # 清理 ZED
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
    node = ZedLeftHandNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

