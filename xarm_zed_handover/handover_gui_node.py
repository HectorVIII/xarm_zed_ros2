import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class HandoverGUI(Node):
    def __init__(self):
        super().__init__('handover_gui')

        # 创建 service client
        self.cli = self.create_client(Trigger, 'start_handover')
        self.get_logger().info('Waiting for /start_handover service ...')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/start_handover service not available, waiting...')

        self.get_logger().info('Connected to /start_handover service.')

        self.running = True
        self.info_text = "Press 's' to start handover, 'q' to quit."

        # 开一个独立线程跑 GUI 循环
        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

    def gui_loop(self):
        """OpenCV GUI + 键盘监听线程。"""
        win_name = "Handover Control"
        cv2.namedWindow(win_name)

        while self.running and rclpy.ok():
            # 画一张简单的黑底图，上面写字
            img = np.zeros((200, 500, 3), dtype=np.uint8)
            cv2.putText(img, "Handover Control", (30, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
            cv2.putText(img, "Press 's' : start handover", (30, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(img, "Press 'q' : quit GUI", (30, 130),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
            cv2.putText(img, self.info_text, (30, 170),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

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
        # 异步调用 /start_handover
        req = Trigger.Request()
        future = self.cli.call_async(req)

        def cb(fut):
            try:
                resp = fut.result()
                self.get_logger().info(f"/start_handover response: success={resp.success}, message='{resp.message}'")
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

