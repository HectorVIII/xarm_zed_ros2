import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType

from std_srvs.srv import Trigger


class HandoverGUI(Node):
    def __init__(self):
        super().__init__('handover_gui')

        # 1) /start_handover service client
        self.cli = self.create_client(Trigger, 'start_handover')
        self.get_logger().info('Waiting for /start_handover service ...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/start_handover service not available, waiting...')

        self.get_logger().info('Connected to /start_handover service.')

        # 2) handover_node 参数服务 client
        #    默认节点名是 handover_node -> 服务名: /handover_node/get_parameters, /handover_node/set_parameters
        self.get_params_client = self.create_client(
            GetParameters, '/handover_node/get_parameters'
        )
        self.set_params_client = self.create_client(
            SetParameters, '/handover_node/set_parameters'
        )

        self.get_logger().info('Waiting for handover_node parameter services ...')
        while not self.get_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/handover_node/get_parameters not available, waiting...')
        while not self.set_params_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/handover_node/set_parameters not available, waiting...')

        # 本地维护的释放阈值
        self.release_force_threshold = 15.0
        self.last_sent_threshold = self.release_force_threshold
        self.last_send_time = 0.0

        # 从 handover_node 读取初始参数
        self.fetch_initial_threshold()

        self.running = True
        self.info_text = "Press 's' to start handover, 'q' to quit."

        # 开一个独立线程跑 GUI 循环
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

    # ---------- 参数相关 ----------

    def fetch_initial_threshold(self):
        """通过 /handover_node/get_parameters 读取 release_force_threshold 初始值。"""
        req = GetParameters.Request()
        req.names = ['release_force_threshold']

        future = self.get_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        try:
            resp = future.result()
            if resp is not None and len(resp.values) > 0:
                pv = resp.values[0]
                if pv.type == ParameterType.PARAMETER_DOUBLE:
                    self.release_force_threshold = pv.double_value
                elif pv.type == ParameterType.PARAMETER_INTEGER:
                    self.release_force_threshold = float(pv.integer_value)

                self.last_sent_threshold = self.release_force_threshold
                self.get_logger().info(
                    f'Fetched release_force_threshold={self.release_force_threshold:.2f} N'
                )
            else:
                self.get_logger().warn(
                    'get_parameters returned empty result, using default threshold.'
                )
        except Exception as e:
            self.get_logger().warn(
                f'Failed to fetch release_force_threshold, using default: {e}'
            )

    def update_threshold_from_slider(self, value: float):
        """滑条改变时回调：更新本地值，并节流后发送到 handover_node。"""
        self.release_force_threshold = value

        now = time.time()
        # 小变化 + 时间太短时就不发，防止刷爆服务
        if abs(value - self.last_sent_threshold) < 0.5 and (now - self.last_send_time) < 0.1:
            return

        self.last_sent_threshold = value
        self.last_send_time = now
        self.push_threshold_to_handover(value)
        self.info_text = f'Set release threshold to {value:.1f} N'

    def push_threshold_to_handover(self, value: float):
        """通过 /handover_node/set_parameters 设置 release_force_threshold。"""
        req = SetParameters.Request()

        param_value = ParameterValue()
        param_value.type = ParameterType.PARAMETER_DOUBLE
        param_value.double_value = float(value)

        param_msg = ParamMsg()
        param_msg.name = 'release_force_threshold'
        param_msg.value = param_value

        req.parameters = [param_msg]

        future = self.set_params_client.call_async(req)

        def done_cb(fut):
            try:
                resp = fut.result()
                if resp is not None and all(r.successful for r in resp.results):
                    self.get_logger().info(
                        f'Sent release_force_threshold={value:.2f} N to handover_node'
                    )
                else:
                    self.get_logger().warn(
                        'Failed to update release_force_threshold parameter on handover_node'
                    )
            except Exception as exc:
                self.get_logger().error(f'Error sending release_force_threshold: {exc}')

        future.add_done_callback(done_cb)

    # ---------- GUI & 交互 ----------

    def gui_loop(self):
        """OpenCV GUI + 键盘监听线程。"""
        win_name = "Handover Control"
        cv2.namedWindow(win_name)

        scale = 10  # 0.1 N resolution
        min_threshold = 0.0
        max_threshold = 30.0
        max_pos = int((max_threshold - min_threshold) * scale)
        init_pos = int(
            (np.clip(self.release_force_threshold, min_threshold, max_threshold) - min_threshold)
            * scale
        )

        def on_trackbar(val):
            new_value = min_threshold + val / scale
            self.update_threshold_from_slider(new_value)

        cv2.createTrackbar('Release Threshold (N)', win_name, init_pos, max_pos, on_trackbar)

        while self.running and rclpy.ok():
            img = np.zeros((220, 600, 3), dtype=np.uint8)
            cv2.putText(
                img,
                "Handover Control",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 255),
                2,
            )
            cv2.putText(
                img,
                "Press 's' : start handover",
                (30, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                1,
            )
            cv2.putText(
                img,
                "Press 'q' : quit GUI",
                (30, 130),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                1,
            )
            cv2.putText(
                img,
                f"Current release threshold: {self.release_force_threshold:.1f} N",
                (30, 160),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 200, 255),
                1,
            )
            cv2.putText(
                img,
                self.info_text,
                (30, 190),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )

            cv2.imshow(win_name, img)
            key = cv2.waitKey(100) & 0xFF

            if key == ord('q'):
                self.get_logger().info("User pressed 'q', closing GUI.")
                break
            elif key == ord('s'):
                self.get_logger().info("User pressed 's', calling /start_handover ...")
                self.call_start_handover()

        self.running = False
        cv2.destroyWindow(win_name)

    def call_start_handover(self):
        """异步调用 /start_handover。"""
        req = Trigger.Request()
        future = self.cli.call_async(req)

        def cb(fut):
            try:
                resp = fut.result()
                self.get_logger().info(
                    f"/start_handover response: success={resp.success}, message='{resp.message}'"
                )
                if resp.success:
                    self.info_text = "Handover started."
                else:
                    self.info_text = f"Failed: {resp.message}"
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                self.info_text = f"Error: {e}"

        future.add_done_callback(cb)

    def destroy_node(self):
        self.running = False
        try:
            self.gui_thread.join(timeout=1.0)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandoverGUI()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

