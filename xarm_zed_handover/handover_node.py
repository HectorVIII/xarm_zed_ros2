import time
import threading    # For running the old FSM in a separate thread, and for locks to protect arm access and shared variables

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from xarm.wrapper import XArmAPI

from .config import (
    ROBOT_IP,
    GRIPPER_SPEED,
    P0,
    P1,
    SAFE_Z_MAX,
    APPROACH_Z_UP,
    CLOSE_APPROACH_SPEED,
    CLOSE_APPROACH_ACC,
    FT_FORCE_RELEASE_N,
    CHECK_PERIOD,
    DEBOUNCE_COUNT,
    ALLOW_TRIGGER_AFTER,
)
from .arm_utils import recover, move, gripper_open, gripper_close
from .pull_release import detect_pull_then_release

# Manual offsets of the handover point in the base coordinate system (unit: mm)
P2_X_BIAS_MM = -120.0
P2_Y_BIAS_MM = -40.0
P2_Z_BIAS_MM = 40.0


class HandoverNode(Node):
    def __init__(self):
        super().__init__("handover_node")

        # --- Locks & SMACC2 related publishers / services ---
        self.arm_lock = threading.Lock()    #
        self.hand_ready_pub = self.create_publisher(Bool, "/handover/hand_ready", 10)

        self.srv_prepare = self.create_service(
            Trigger, "/handover/prepare_tool", self.cb_prepare_tool
        )
        self.srv_enable_wait = self.create_service(
            Trigger, "/handover/enable_wait_hand", self.cb_enable_wait_hand
        )
        self.srv_execute = self.create_service(
            Trigger, "/handover/execute_handover", self.cb_execute_handover
        )

        # --- robotics arm ip setting ---
        self.declare_parameter("robot_ip", ROBOT_IP)    # Declare the parameter 'robot_ip' with a default value from ROBOT_IP of config.py
        robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value    # Retrieve the parameter value and convert it to a string for use
        #example: ros2 run xarm_zed_handover handover_node --ros-args -p robot_ip:="192.168.1.155"
        
        # -- Force threshold parameter for pull-and-release detection, with dynamic reconfigure support ---
        self.declare_parameter("release_force_threshold", FT_FORCE_RELEASE_N)   # Declare the parameter 'release_force_threshold' with a default value from FT_FORCE_RELEASE_N of config.py
        self.release_force_threshold = self.get_parameter("release_force_threshold").value  # Retrieve the parameter value and store it in self.release_force_threshold for use in the handover logic
        # Register the callback function self.on_parameters_set to be called whenever parameters are set, 
        # allowing dynamic updates to parameters like release_force_threshold at runtime through ROS2 parameter services or command line tools.
        self.add_on_set_parameters_callback(self.on_parameters_set) 
        self.get_logger().info(f"Initial release_force_threshold = {self.release_force_threshold:.2f} N")   # Log the initial value of the release force threshold for verification

        # --- Connect to the robotic arm ---
        self.get_logger().info(f"Connecting to xArm at {robot_ip} ...")
        self.arm = XArmAPI(robot_ip, is_radian=False)   # Connect to the xArm using the IP address specified in the parameters, with angles in degrees (is_radian=False)
        self.arm.connect()
        if self.arm.connected:
            self.get_logger().info("Successfully connected to xArm.")
            try:
                # try to recover the arm state first in case it's in an error/warn state.
                recover(self.arm)
                self.get_logger().info("xArm state recovered and ready for motion.")
            except Exception as e:
                self.get_logger().error(f"Connected, but failed to recover arm: {e}")
        else:
            self.get_logger().error(f"Failed to connect to xArm at {robot_ip}. Please check the network or IP.")

        # --- State variables ---
        self.state = "IDLE"
        self.busy = False  # Only used for the old /start_handover FSM
        self.P2 = None
        self.P2_UP = None

        # --- ZED right hand subscription ---
        self.right_hand_sub = None
        self.latest_hand_point = None
        self.right_hand_lock = threading.Lock()

        self.declare_parameter("right_hand_topic", "/right_hand/point")
        topic = self.get_parameter("right_hand_topic").get_parameter_value().string_value

        self.right_hand_sub = self.create_subscription(
            PointStamped, topic, self.right_hand_callback, 10
        )
        self.get_logger().info(f"Subscribing right hand from topic: {topic}")

        # --- Old all-in-one service: /start_handover (can be kept for standalone debugging) ---
        self.srv = self.create_service(
            Trigger, "start_handover", self.start_handover_cb
        )
        self.get_logger().info("Service /start_handover ready. State = IDLE.")

    # ========== Old all-in-one FSM, for compatibility ==========
    def start_handover_cb(self, request, response):
        if self.busy:
            response.success = False
            response.message = "Handover already running."
            self.get_logger().warn("Received /start_handover but already running.")
            return response

        self.get_logger().info(
            "Received /start_handover request, starting handover thread..."
        )
        self.busy = True
        self.P2 = None
        self.P2_UP = None

        t = threading.Thread(target=self.run_handover_fsm, daemon=True)
        t.start()

        response.success = True
        response.message = "Handover started."
        return response

    def run_handover_fsm(self):
        try:
            self.state = "PREPARE_TOOL"
            self.get_logger().info("Handover FSM: PREPARE_TOOL")
            with self.arm_lock:
                self.prepare_tool()

            self.state = "WAIT_HAND"
            self.get_logger().info("Handover FSM: WAIT_HAND - waiting right hand pose ...")
            self.P2 = None
            self.P2_UP = None

            wait_start = time.time()
            while self.P2 is None and rclpy.ok():
                time.sleep(0.05)
                if time.time() - wait_start > 60.0:
                    self.get_logger().warn(
                        "Timeout waiting for right hand point, aborting handover."
                    )
                    return

            if self.P2 is None:
                self.get_logger().warn("Handover aborted: P2 is None.")
                return

            self.state = "EXECUTING"
            self.execute_handover()

            self.state = "DONE"
            self.get_logger().info("Handover finished. State = DONE.")
        except Exception as e:
            self.get_logger().error(f"Handover error: {e}")
        finally:
            self.state = "IDLE"
            self.busy = False
            self.P2 = None
            self.P2_UP = None
            self.get_logger().info("State back to IDLE, ready for another /start_handover.")

    # ========== Core action functions ==========
    def prepare_tool(self):
        """P0 -> P1 grasp tool -> back to P0

        Called in both SMACC2 synchronous service and the old FSM.
        """
        arm = self.arm

        self.get_logger().info("[prepare_tool] Calling recover() before motion.")
        recover(arm)

        # Print current position for easy troubleshooting
        code, cur = arm.get_position(is_radian=False)
        if code == 0 and cur:
            self.get_logger().info(
                f"[prepare_tool] Current pose BEFORE P0: {cur[:6]}"
            )
        else:
            self.get_logger().warn(
                f"[prepare_tool] get_position failed before P0, code={code}"
            )

        # ---- Encapsulate a safe move: log error, recover, and retry if failed ----
        def safe_move(target, desc: str, speed=None, acc=None):
            self.get_logger().info(f"[prepare_tool] Move to {desc}")
            ret = move(arm, target, speed=speed, acc=acc)
            self.get_logger().info(f"[prepare_tool] move to {desc} ret={ret}")
            if ret != 0:
                # Print more state information
                st = arm.get_state()
                err_warn = arm.get_err_warn_code()
                self.get_logger().warn(
                    f"[prepare_tool] move to {desc} failed, code={ret}, "
                    f"state={st}, err_warn={err_warn}. Trying recover() and retry..."
                )
                recover(arm)
                ret2 = move(arm, target, speed=speed, acc=acc)
                self.get_logger().info(
                    f"[prepare_tool] move to {desc} after recover ret={ret2}"
                )
                if ret2 != 0:
                    # Truly raise exception here so full info is visible in the log
                    raise RuntimeError(
                        f"move to {desc} failed after recover, code={ret2}, "
                        f"state={st}, err_warn={err_warn}"
                    )

        # 1) Move to P0
        safe_move(P0, "P0")

        # 2) Open gripper
        self.get_logger().info("[prepare_tool] Open gripper")
        gripper_open(arm)

        # 3) Move to P1 (tool tray)
        safe_move(P1, "P1 (tool tray)")

        # 4) Close gripper to grasp tool
        self.get_logger().info("[prepare_tool] Close gripper to grasp tool")
        gripper_close(arm)

        # 5) Back to P0
        safe_move(P0, "P0 (back with tool)")

        code, cur = arm.get_position(is_radian=False)
        if code == 0 and cur:
            self.get_logger().info(
                f"[prepare_tool] Current pose AFTER P0: {cur[:6]}"
            )

        self.get_logger().info("[prepare_tool] Finished successfully.")


    def right_hand_callback(self, msg: PointStamped):
        # Only update in WAIT_HAND state
        if self.state != "WAIT_HAND":
            return

        with self.right_hand_lock:
            self.latest_hand_point = msg

        # ZED: m -> mm
        x = msg.point.x * 1000.0
        y = msg.point.y * 1000.0
        z = msg.point.z * 1000.0

        # Manual offsets
        x += P2_X_BIAS_MM
        y += P2_Y_BIAS_MM
        z += P2_Z_BIAS_MM

        # Limit z within safe range
        z = max(0.0, min(z, SAFE_Z_MAX))

        # ✅ Keep P0 orientation, only change xyz
        roll = P0["roll"]
        pitch = P0["pitch"]
        yaw = P0["yaw"]

        self.P2 = {
            "x": x,
            "y": y,
            "z": z,
            "roll": 177.0,  # P2_ORI["roll"],
            "pitch": -8.7,  # P2_ORI["pitch"],
            "yaw": 96.4,   # P2_ORI["yaw"],
        }

        self.P2_UP = {
            "x": x,
            "y": y,
            "z": min(z + APPROACH_Z_UP, SAFE_Z_MAX),
            "roll": 177.0,  # P2_ORI["roll"],
            "pitch": -8.7,  # P2_ORI["pitch"],
            "yaw": 96.4,   # P2_ORI["yaw"],
        }

        self.get_logger().info(f"[right_hand] P2 = {self.P2}, P2_UP = {self.P2_UP}")

        # Tell SMACC2: hand is ready
        msg_bool = Bool()
        msg_bool.data = True
        self.hand_ready_pub.publish(msg_bool)



    def execute_handover(self):
        arm = self.arm

        if self.P2 is None or self.P2_UP is None:
            self.get_logger().warn("execute_handover called but P2/P2_UP is None.")
            return

        self.get_logger().info("[execute_handover] Start handover motion")

        # 1) Return to P0 first (ensure consistent starting point)
        self.get_logger().info("[execute_handover] Ensure at P0")
        move(arm, P0)

        # 2) Move to P2_UP (slightly above the hand)
        self.get_logger().info("[execute_handover] Move to P2_UP")
        ret = move(arm, self.P2_UP, speed=160, acc=5000)
        self.get_logger().info(f"[execute_handover] move to P2_UP ret={ret}")
        if ret != 0:
            self.get_logger().warn(
                f"[execute_handover] move to P2_UP failed (code={ret}), trying recover()..."
            )
            recover(arm)
            ret2 = move(arm, self.P2_UP, speed=160, acc=5000)
            self.get_logger().info(
                f"[execute_handover] move to P2_UP after recover ret={ret2}"
            )

        # 3) Slowly approach P2 (handover point) from P2_UP
        self.get_logger().info("[execute_handover] Approach P2 from P2_UP (slow)")
        ret = move(
            arm,
            self.P2,
            speed=CLOSE_APPROACH_SPEED,
            acc=CLOSE_APPROACH_ACC,
        )
        self.get_logger().info(f"[execute_handover] approach P2 ret={ret}")
        if ret != 0:
            self.get_logger().warn(
                f"[execute_handover] Approach P2 failed (code={ret}), fallback small step."
            )
            code, cur = arm.get_position(is_radian=False)
            if code == 0 and cur:
                x, y, z, r, p, yaw = cur[:6]
                z2 = max(z - 20, 0)
                arm.set_position(
                    x=x,
                    y=y,
                    z=z2,
                    roll=r,
                    pitch=p,
                    yaw=yaw,
                    speed=CLOSE_APPROACH_SPEED,
                    mvacc=CLOSE_APPROACH_ACC,
                    wait=True,
                )

        # 4) Force/Torque sensor detection + release
        self.get_logger().info(
            "[execute_handover] Start FT-based pull detection and release"
        )
        try:
            arm.ft_sensor_enable(True)
            arm.ft_sensor_set_zero()

            # ✅ Follow the definition in pull_release.py exactly, passing only two parameters
            # def detect_pull_then_release(arm, force_threshold_n):
            triggered = detect_pull_then_release(
                arm,
                self.release_force_threshold,
            )

            if not triggered:
                self.get_logger().warn(
                    "[execute_handover] FT trigger did not occur within timeout."
                )
            else:
                self.get_logger().info(
                    "[execute_handover] FT trigger detected, gripper opened."
                )
        finally:
            # Regardless of trigger, disable FT, recover state, and lift back to P2_UP / P0
            try:
                arm.ft_sensor_enable(False)
            except Exception as e:
                self.get_logger().warn(
                    f"[execute_handover] Failed to disable FT sensor: {e}"
                )

            time.sleep(0.2)
            recover(arm)

            self.get_logger().info(
                "[execute_handover] Lift from P2 back to P2_UP and return to P0"
            )
            ret = move(arm, self.P2_UP, speed=160, acc=5000)
            self.get_logger().info(
                f"[execute_handover] lift to P2_UP ret={ret}"
            )

            ret = move(arm, P0)
            self.get_logger().info(
                f"[execute_handover] return to P0 ret={ret}"
            )

        self.get_logger().info("[execute_handover] Handover finished.")


    # ========== Parameter callbacks ==========
    def on_parameters_set(self, params):
        for param in params:
            if param.name == "release_force_threshold":
                self.release_force_threshold = float(param.value)
                self.get_logger().info(
                    f"Updated release_force_threshold to {self.release_force_threshold:.2f} N"
                )
        return SetParametersResult(successful=True)

    # ========== 3 services connected to SMACC2 ==========
    def cb_prepare_tool(self, request, response):
        """SMACC2: Synchronous version of /handover/prepare_tool"""
        if self.busy:
            response.success = False
            response.message = "Node busy (start_handover running)."
            return response

        with self.arm_lock:
            try:
                self.get_logger().info("[Svc] prepare_tool start (sync)")
                self.prepare_tool()
                self.get_logger().info("[Svc] prepare_tool done (sync)")
                response.success = True
                response.message = "prepare_tool finished"
            except Exception as e:
                self.get_logger().error(f"[Svc] prepare_tool exception: {e}")
                response.success = False
                response.message = f"prepare_tool exception: {e}"
        return response

    def cb_enable_wait_hand(self, request, response):
        if self.busy:
            response.success = False
            response.message = "Node busy (start_handover running)."
            return response

        self.state = "WAIT_HAND"
        self.P2 = None
        self.P2_UP = None
        response.success = True
        response.message = "state set to WAIT_HAND, P2 cleared"
        self.get_logger().info("[Svc] Enabled WAIT_HAND and cleared P2")
        return response

    def cb_execute_handover(self, request, response):
        """SMACC2: Synchronous version of /handover/execute_handover"""
        if self.busy:
            response.success = False
            response.message = "Node busy (start_handover running)."
            return response

        if self.P2 is None or self.P2_UP is None:
            response.success = False
            response.message = "P2 not ready. Wait for /handover/hand_ready first."
            return response

        with self.arm_lock:
            try:
                self.get_logger().info("[Svc] execute_handover start (sync)")
                self.execute_handover()
                self.get_logger().info("[Svc] execute_handover done (sync)")
                response.success = True
                response.message = "execute_handover finished"
            except Exception as e:
                self.get_logger().error(f"[Svc] execute_handover exception: {e}")
                response.success = False
                response.message = f"execute_handover exception: {e}"
        return response

    # ========== Destroy ==========
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


if __name__ == "__main__":
    main()