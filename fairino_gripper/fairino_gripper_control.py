#!/usr/bin/env python3
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from fairino_msgs.srv import RemoteCmdInterface


class FairinoGripperNode(Node):
    """
    Pneumatic SoftGripper control via Fairino RemoteCmdInterface.

    Supplier rule:
      - DO_OPEN and DO_CLOSE must NEVER be ON at the same time.

    Mapping:
      - OPEN  => DO_OPEN=1, DO_CLOSE=0
      - CLOSE => DO_OPEN=0, DO_CLOSE=1
      - IDLE  => DO_OPEN=0, DO_CLOSE=0

    Additionally publishes a "fake" gripper joint state:
      - /gripper/joint_states  (JointState) with one joint: softgripper_joint
        position: 0.0=open, 1.0=close, 0.5=idle (configurable if you want)
    """

    def __init__(self):
        super().__init__('fairino_gripper_node')

        self.cb_group = ReentrantCallbackGroup()
        self.cmd_lock = threading.Lock()

        # ---------------- Parameters ----------------
        self.declare_parameter('remote_service', '/fairino_remote_command_service')
        self.declare_parameter('do_open', 0)
        self.declare_parameter('do_close', 1)
        self.declare_parameter('do_suction', 2)

        self.declare_parameter('interlock_delay_s', 0.15)
        self.declare_parameter('call_timeout_s', 5.0)

        self.declare_parameter('auto_idle', False)
        self.declare_parameter('hold_s', 0.30)

        # JointState publishing
        self.declare_parameter('gripper_joint_name', 'softgripper_joint')
        self.declare_parameter('gripper_joint_state_topic', '/gripper/joint_states')
        self.declare_parameter('gripper_joint_pub_rate_hz', 20.0)
        self.declare_parameter('pos_open', 0.0)
        self.declare_parameter('pos_close', 1.0)
        self.declare_parameter('pos_idle', 0.5)

        self.remote_service = self.get_parameter('remote_service').value
        self.do_open = int(self.get_parameter('do_open').value)
        self.do_close = int(self.get_parameter('do_close').value)
        self.do_suction = int(self.get_parameter('do_suction').value)

        self.interlock_delay_s = float(self.get_parameter('interlock_delay_s').value)
        self.call_timeout_s = float(self.get_parameter('call_timeout_s').value)

        self.auto_idle = bool(self.get_parameter('auto_idle').value)
        self.hold_s = float(self.get_parameter('hold_s').value)

        self.gripper_joint_name = self.get_parameter('gripper_joint_name').value
        self.gripper_joint_state_topic = self.get_parameter('gripper_joint_state_topic').value
        self.pub_rate_hz = float(self.get_parameter('gripper_joint_pub_rate_hz').value)
        self.pos_open = float(self.get_parameter('pos_open').value)
        self.pos_close = float(self.get_parameter('pos_close').value)
        self.pos_idle = float(self.get_parameter('pos_idle').value)

        # internal "fake joint" state
        self._gripper_pos = self.pos_idle
        self._state_lock = threading.Lock()

        # ---------------- Fairino remote service client ----------------
        self.cli = self.create_client(
            RemoteCmdInterface,
            self.remote_service,
            callback_group=self.cb_group
        )

        self.get_logger().info(f"Waiting for service {self.remote_service} ...")
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"Service {self.remote_service} not available. Is ros2_cmd_server running?")
        else:
            self.get_logger().info("Connected to Fairino remote command service.")

        # ---------------- ROS services ----------------
        self.create_service(Trigger, 'gripper/open', self.handle_open, callback_group=self.cb_group)
        self.create_service(Trigger, 'gripper/close', self.handle_close, callback_group=self.cb_group)
        self.create_service(Trigger, 'gripper/idle', self.handle_idle, callback_group=self.cb_group)
        self.create_service(Trigger, 'gripper/suction_on', self.handle_suction_on, callback_group=self.cb_group)
        self.create_service(Trigger, 'gripper/suction_off', self.handle_suction_off, callback_group=self.cb_group)

        # Optional topic command
        self.create_subscription(String, 'gripper/cmd', self.on_cmd, 10)

        # ---------------- JointState publisher ----------------
        self.gripper_js_pub = self.create_publisher(JointState, self.gripper_joint_state_topic, 10)
        period = 1.0 / max(self.pub_rate_hz, 1e-3)
        self.create_timer(period, self._publish_gripper_joint_state)

        self.get_logger().info(
            "Ready.\n"
            f"  DO mapping: open={self.do_open}, close={self.do_close}, suction={self.do_suction}\n"
            f"  interlock_delay_s={self.interlock_delay_s}\n"
            f"  auto_idle={self.auto_idle} hold_s={self.hold_s}\n"
            f"  call_timeout_s={self.call_timeout_s}\n"
            f"  publishing gripper joint: {self.gripper_joint_name} on {self.gripper_joint_state_topic} @ {self.pub_rate_hz} Hz"
        )

    # ---------------- Remote command call (NO nested spin) ----------------
    def call_cmd(self, cmd: str) -> bool:
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().error(f"Remote service not available: {self.remote_service}")
            return False

        req = RemoteCmdInterface.Request()
        req.cmd_str = cmd

        done_evt = threading.Event()
        result_holder = {"ok": False, "resp": None, "exc": None}

        future = self.cli.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                result_holder["resp"] = resp
                result_holder["ok"] = (str(resp.cmd_res).strip() == '1')
            except Exception as e:
                result_holder["exc"] = e
            finally:
                done_evt.set()

        future.add_done_callback(_done_cb)

        if not done_evt.wait(timeout=self.call_timeout_s):
            self.get_logger().error(f"Timeout calling cmd: {cmd}")
            return False

        if result_holder["exc"] is not None:
            self.get_logger().error(f"Service call exception for '{cmd}': {result_holder['exc']}")
            return False

        ok = result_holder["ok"]
        resp = result_holder["resp"]
        if ok:
            self.get_logger().info(f"OK: {cmd}")
        else:
            self.get_logger().error(f"Robot rejected: {cmd} -> cmd_res='{resp.cmd_res if resp else None}'")
        return ok

    def set_do(self, do_id: int, value: int) -> bool:
        return self.call_cmd(f"SetDO({do_id},{int(value)})")

    # ---------------- Safety helpers ----------------
    def _all_off(self) -> bool:
        ok0 = self.set_do(self.do_open, 0)
        ok1 = self.set_do(self.do_close, 0)
        return ok0 and ok1

    def _break_before_make(self, do_to_enable: int) -> bool:
        with self.cmd_lock:
            if not self._all_off():
                return False

            time.sleep(self.interlock_delay_s)

            if not self.set_do(do_to_enable, 1):
                return False

            if self.auto_idle:
                time.sleep(self.hold_s)
                return self._all_off()

            return True

    # ---------------- High-level commands ----------------
    def open_gripper(self) -> bool:
        ok = self._break_before_make(self.do_open)
        if ok:
            with self._state_lock:
                self._gripper_pos = self.pos_open
        return ok

    def close_gripper(self) -> bool:
        ok = self._break_before_make(self.do_close)
        if ok:
            with self._state_lock:
                self._gripper_pos = self.pos_close
        return ok

    def idle_gripper(self) -> bool:
        with self.cmd_lock:
            ok = self._all_off()
        if ok:
            with self._state_lock:
                self._gripper_pos = self.pos_idle
        return ok

    def suction_on(self) -> bool:
        with self.cmd_lock:
            return self.set_do(self.do_suction, 1)

    def suction_off(self) -> bool:
        with self.cmd_lock:
            return self.set_do(self.do_suction, 0)

    # ---------------- JointState publish ----------------
    def _publish_gripper_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.gripper_joint_name]
        with self._state_lock:
            msg.position = [float(self._gripper_pos)]
        self.gripper_js_pub.publish(msg)

    # ---------------- ROS service handlers ----------------
    def handle_open(self, req, res):
        res.success = self.open_gripper()
        res.message = "open" if res.success else "failed to open"
        return res

    def handle_close(self, req, res):
        res.success = self.close_gripper()
        res.message = "close" if res.success else "failed to close"
        return res

    def handle_idle(self, req, res):
        res.success = self.idle_gripper()
        res.message = "idle" if res.success else "failed to idle"
        return res

    def handle_suction_on(self, req, res):
        res.success = self.suction_on()
        res.message = "suction_on" if res.success else "failed suction_on"
        return res

    def handle_suction_off(self, req, res):
        res.success = self.suction_off()
        res.message = "suction_off" if res.success else "failed suction_off"
        return res

    # ---------------- Topic command handler ----------------
    def on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'open':
            self.open_gripper()
        elif cmd == 'close':
            self.close_gripper()
        elif cmd == 'idle':
            self.idle_gripper()
        elif cmd == 'suction_on':
            self.suction_on()
        elif cmd == 'suction_off':
            self.suction_off()
        else:
            self.get_logger().warn(f"Unknown gripper/cmd: '{cmd}'")


def main():
    rclpy.init()
    node = FairinoGripperNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()