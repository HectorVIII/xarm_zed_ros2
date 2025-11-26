# arm_handover/handover_node.py

import time

import rclpy
from rclpy.node import Node
from xarm.wrapper import XArmAPI

from .config import (
    ROBOT_IP,
    GRIPPER_SPEED,
    P0, P1,
    APPROACH_Z_UP, SAFE_Z_MAX,
    CLOSE_APPROACH_SPEED, CLOSE_APPROACH_ACC,
)
from .arm_utils import recover, move, gripper_open, gripper_close
from .zed_left_hand import detect_left_hand_stable_then_map_to_P2
from .pull_release import detect_pull_then_release


class HandoverNode(Node):
    def __init__(self):
        super().__init__('handover_node')

        # ROS2 参数，可以在 launch 里改 robot_ip
        self.declare_parameter('robot_ip', ROBOT_IP)
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.get_logger().info(f'Connecting to xArm at {robot_ip} ...')
        self.arm = XArmAPI(robot_ip, is_radian=False)
        self.arm.connect()

        # 初始化机械臂、夹爪
        recover(self.arm)
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_speed(GRIPPER_SPEED)

        # 这里没有用 timer，而是直接顺序执行一遍 handover
        self.run_handover()

        # 完成后断开连接（如果你想循环执行，可以改成 while + 状态机）
        self.arm.disconnect()
        self.get_logger().info('Handover finished, node will exit.')

    # 把你原来 main() 里的流程搬到这里
    def run_handover(self):
        arm = self.arm

        # ====== Step 1: P0 → P1 抓工具 → 回 P0 ======
        self.get_logger().info('Move to P0')
        move(arm, P0)

        self.get_logger().info('Open gripper')
        gripper_open(arm)

        self.get_logger().info('Move to P1 and grip tool')
        move(arm, P1)

        self.get_logger().info('Close gripper')
        gripper_close(arm)

        self.get_logger().info('Back to P0')
        move(arm, P0)

        # ====== Step 2: ZED 检测左手稳定位置 → P2 ======
        self.get_logger().info('Waiting for left hand stable position (ZED)...')
        P2 = detect_left_hand_stable_then_map_to_P2()
        self.get_logger().info(f'Got P2 from ZED: {P2}')

        P2_UP = dict(**P2)
        P2_UP["z"] = min(P2["z"] + APPROACH_Z_UP, SAFE_Z_MAX)

        self.get_logger().info('Move to P2_UP')
        move(arm, P2_UP)

        self.get_logger().info('Move to P2 (close approach)')
        move(arm, P2, speed=CLOSE_APPROACH_SPEED, acc=CLOSE_APPROACH_ACC)

        # ====== Step 3: 等人拉扯 → 用 FT 检测 → 放手 ======
        self.get_logger().info('Waiting for pull trigger (FT)...')
        if detect_pull_then_release(arm):
            self.get_logger().info('Pull detected, gripper opened.')

            # 可选：用完就关掉 FT
            try:
                arm.ft_sensor_enable(False)
            except Exception as e:
                self.get_logger().warn(f'Failed to disable FT sensor: {e}')

            time.sleep(0.2)
            recover(arm)

            # ====== Step 4: 抬起 → 回 P0 ======
            self.get_logger().info('Lift up from P2 to P2_UP')
            ret = move(arm, P2_UP, speed=160, acc=5000)
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
                    move(arm, P2_UP, speed=160, acc=5000)

            self.get_logger().info('Go back to P0')
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


def main(args=None):
    rclpy.init(args=args)
    node = HandoverNode()
    # 这里流程在 __init__ 里已经跑完了，所以不用 spin
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

