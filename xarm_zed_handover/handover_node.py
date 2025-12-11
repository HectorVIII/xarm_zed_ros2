# xarm_zed_handover/handover_node.py

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult
from xarm.wrapper import XArmAPI

from .config import (
    ROBOT_IP,
    GRIPPER_SPEED,
    P0, P1,
    APPROACH_Z_UP, SAFE_Z_MAX,
    CLOSE_APPROACH_SPEED, CLOSE_APPROACH_ACC,
    P2_ORI,
    FT_FORCE_RELEASE_N,
)

from .arm_utils import recover, move, gripper_open, gripper_close
from .pull_release import detect_pull_then_release

# 交接点在 base 坐标系下的手动偏移（单位：mm）
# 如果发现改完之后从“偏左 2cm”变成“偏右 2cm”，就把 -20.0 改成 +20.0
P2_X_BIAS_MM = -120.0
P2_Z_BIAS_MM = 40.0
P2_Y_BIAS_MM = -40

class HandoverNode(Node):
    def __init__(self):
        super().__init__('handover_node')

        # ==== 连接机械臂 ====
        self.declare_parameter('robot_ip', ROBOT_IP)
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.declare_parameter('release_force_threshold', FT_FORCE_RELEASE_N)
        self.release_force_threshold = self.get_parameter('release_force_threshold').value
        self.add_on_set_parameters_callback(self.on_parameters_set)

        self.get_logger().info(f'Initial release_force_threshold = {self.release_force_threshold:.2f} N')

        self.get_logger().info(f'Connecting to xArm at {robot_ip} ...')
        self.arm = XArmAPI(robot_ip, is_radian=False)
        self.arm.connect()
        recover(self.arm)

        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_speed(GRIPPER_SPEED)

        # ==== 状态机相关 ====
        self.state = "IDLE"   # IDLE / PREPARE / WAIT_HAND / EXECUTING / DONE
        self.busy = False
        self.P2 = None
        self.P2_UP = None

        # ==== 订阅右手点 ====
        self.declare_parameter('right_hand_topic', '/right_hand/point')
        topic = self.get_parameter('right_hand_topic').get_parameter_value().string_value

        self.right_hand_sub = self.create_subscription(
            PointStamped,
            topic,
            self.right_hand_callback,
            10
        )
        self.get_logger().info(f'Subscribing right hand from topic: {topic}')

        # ==== 创建 /start_handover Service ====
        self.srv = self.create_service(Trigger, 'start_handover', self.start_handover_cb)
        self.get_logger().info('Service /start_handover ready. State = IDLE.')

    # --------- Service 回调：触发一轮 handover ---------
    def start_handover_cb(self, request, response):
        if self.busy:
            response.success = False
            response.message = 'Handover already running.'
            self.get_logger().warn('Received /start_handover but already running.')
            return response

        self.get_logger().info('Received /start_handover request, starting handover thread...')
        self.busy = True
        self.P2 = None
        self.P2_UP = None

        # 新开线程跑完整流程，避免阻塞 ROS 回调线程
        t = threading.Thread(target=self.run_handover_fsm, daemon=True)
        t.start()

        response.success = True
        response.message = 'Handover started.'
        return response

    # --------- 主流程状态机：一轮 handover ---------
    def run_handover_fsm(self):
        try:
            # 1) 准备工具
            self.state = 'PREPARE'
            self.prepare_tool()

            # 2) 等待稳定右手点
            self.state = 'WAIT_HAND'
            self.get_logger().info('Waiting for stable right hand point...')

            wait_start = time.time()
            while rclpy.ok() and self.P2 is None:
                time.sleep(0.05)
                # 可选：超时保护
                if time.time() - wait_start > 60.0:  # 超过 60s 还没等到就退出
                    self.get_logger().warn('Timeout waiting for right hand point, aborting handover.')
                    return

            if self.P2 is None:
                self.get_logger().warn('Handover aborted: P2 is None.')
                return

            # 3) 执行 handover 运动 + FT 检测
            self.state = 'EXECUTING'
            self.execute_handover()

            self.state = 'DONE'
            self.get_logger().info('Handover finished. State = DONE.')

        except Exception as e:
            self.get_logger().error(f'Handover error: {e}')

        finally:
            # 无论成功/失败，都回到 IDLE，允许下一轮
            self.state = 'IDLE'
            self.busy = False
            self.P2 = None
            self.P2_UP = None
            self.get_logger().info('State back to IDLE, ready for another /start_handover.')

    # --------- 抓取工具：P0 -> P1 -> P0 ---------
    def prepare_tool(self):
        arm = self.arm
        self.get_logger().info('Prepare: Move to P0')
        move(arm, P0)

        self.get_logger().info('Prepare: Open gripper')
        gripper_open(arm)

        self.get_logger().info('Prepare: Move to P1 and grip tool')
        move(arm, P1)

        self.get_logger().info('Prepare: Close gripper')
        gripper_close(arm)

        self.get_logger().info('Prepare: Back to P0')
        move(arm, P0)

    # --------- 右手话题回调：只在 WAIT_HAND 状态使用 ---------
    def right_hand_callback(self, msg: PointStamped):
        if self.state != "WAIT_HAND":
            return
        if self.P2 is not None:
            # 已经有一个点了，本轮不再覆盖
            return

        x = msg.point.x  # m
        y = msg.point.y
        z = msg.point.z

        self.get_logger().info(
            f"Received right hand point (base_link): x={x:.3f}, y={y:.3f}, z={z:.3f} (m)"
        )

        # 转成 mm，兼容你原来的配置

        x_mm, y_mm, z_mm = 1000.0 * x, 1000.0 * y, 1000.0 * z

        # 在 base 坐标系下对 Y 做一个固定偏移，补偿“总是偏左约 2cm”
        x_mm += P2_X_BIAS_MM
        z_mm += P2_Z_BIAS_MM
        y_mm += P2_Y_BIAS_MM

        self.P2 = dict(x=x_mm, y=y_mm, z=z_mm, **P2_ORI)
        self.P2_UP = dict(**self.P2)
        self.P2_UP["z"] = min(self.P2["z"] + APPROACH_Z_UP, SAFE_Z_MAX)

        self.get_logger().info(f"Computed P2 (mm): {self.P2}")
        # 状态不用在这里改，run_handover_fsm 会看到 P2 != None 然后继续执行

    # --------- 执行 P2 handover + FT 检测 ---------
    def execute_handover(self):
        arm = self.arm

        self.get_logger().info('Execute: Move to P2_UP')
        move(arm, self.P2_UP)

        self.get_logger().info('Execute: Move to P2 (close approach)')
        move(arm, self.P2, speed=CLOSE_APPROACH_SPEED, acc=CLOSE_APPROACH_ACC)

        self.get_logger().info('Execute: Waiting for pull trigger (FT)...')
        if detect_pull_then_release(arm, self.release_force_threshold):
            self.get_logger().info('Pull detected, gripper opened.')
            try:
                arm.ft_sensor_enable(False)
            except Exception as e:
                self.get_logger().warn(f'Failed to disable FT sensor: {e}')

            time.sleep(0.2)
            recover(arm)

            # 抬起
            self.get_logger().info('Execute: Lift up from P2 to P2_UP')
            ret = move(arm, self.P2_UP, speed=160, acc=5000)
            if ret != 0:
                self.get_logger().warn(f'Lift failed (code={ret}), fallback relative lift')
                code, cur = arm.get_position(is_radian=False)
                if code == 0 and cur:
                    x, y, z, r, p, yaw = cur[:6]
                    z2 = min(z + 60, SAFE_Z_MAX)
                    arm.set_position(
                        x=x, y=y, z=z2, roll=r, pitch=p, yaw=yaw,
                        speed=120, mvacc=4000, wait=True
                    )
                    recover(arm)
                    move(arm, self.P2_UP, speed=160, acc=5000)

            # 回 P0
            self.get_logger().info('Execute: Go back to P0')
            ret = move(arm, P0)
            if ret != 0:
                self.get_logger().warn(f'Return to P0 failed (code={ret}), fallback via lift')
                code, cur = arm.get_position(is_radian=False)
                if code == 0 and cur:
                    x, y, z, r, p, yaw = cur[:6]
                    z2 = min(z + 80, SAFE_Z_MAX)
                    arm.set_position(
                        x=x, y=y, z=z2, roll=r, pitch=p, yaw=yaw,
                        speed=120, mvacc=4000, wait=True
                    )
                    recover(arm)
                    move(arm, P0)

    def on_parameters_set(self, params):
        for param in params:
            if param.name == 'release_force_threshold':
                self.release_force_threshold = float(param.value)
                self.get_logger().info(
                    f'Updated release_force_threshold to {self.release_force_threshold:.2f} N'
                )
        return SetParametersResult(successful=True)

    def destroy_node(self):
        if self.arm:
            try:
                self.arm.disconnect()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandoverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
