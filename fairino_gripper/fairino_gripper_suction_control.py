#!/usr/bin/env python3
"""
Suction Cup control via Fairino RemoteCmdInterface (DO2).

Mapping:
  - ON  => SetDO(do_suction, 1)  -> suctioncup_joint = 1.0
  - OFF => SetDO(do_suction, 0)  -> suctioncup_joint = 0.0

Publishes fake joint state on /suction/joint_states.
"""
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


class FairinoSuctionNode(Node):

    def __init__(self):
        super().__init__('fairino_suction_node')

        self.cb_group = ReentrantCallbackGroup()
        self.cmd_lock = threading.Lock()

        # ---------------- Parameters ----------------
        self.declare_parameter('remote_service', '/fairino_remote_command_service')
        self.declare_parameter('do_suction', 2)
        self.declare_parameter('call_timeout_s', 5.0)

        # JointState publishing
        self.declare_parameter('suction_joint_name', 'suctioncup_joint')
        self.declare_parameter('suction_joint_state_topic', '/suction/joint_states')
        self.declare_parameter('suction_joint_pub_rate_hz', 20.0)
        self.declare_parameter('pos_on', 1.0)
        self.declare_parameter('pos_off', 0.0)

        self.remote_service = self.get_parameter('remote_service').value
        self.do_suction = int(self.get_parameter('do_suction').value)
        self.call_timeout_s = float(self.get_parameter('call_timeout_s').value)

        self.suction_joint_name = self.get_parameter('suction_joint_name').value
        self.suction_joint_state_topic = self.get_parameter('suction_joint_state_topic').value
        self.pub_rate_hz = float(self.get_parameter('suction_joint_pub_rate_hz').value)
        self.pos_on = float(self.get_parameter('pos_on').value)
        self.pos_off = float(self.get_parameter('pos_off').value)

        # internal "fake joint" state
        self._suction_pos = self.pos_off
        self._state_lock = threading.Lock()

        # ---------------- Fairino remote service client ----------------
        self.cli = self.create_client(
            RemoteCmdInterface,
            self.remote_service,
            callback_group=self.cb_group
        )

        self.get_logger().info(f"Waiting for service {self.remote_service} ...")
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                f"Service {self.remote_service} not available. Is ros2_cmd_server running?")
        else:
            self.get_logger().info("Connected to Fairino remote command service.")

        # ---------------- ROS services ----------------
        self.create_service(Trigger, 'suction/on',
                            self.handle_on, callback_group=self.cb_group)
        self.create_service(Trigger, 'suction/off',
                            self.handle_off, callback_group=self.cb_group)

        # Optional topic command
        self.create_subscription(String, 'suction/cmd', self.on_cmd, 10)

        # ---------------- JointState publisher ----------------
        self.suction_js_pub = self.create_publisher(
            JointState, self.suction_joint_state_topic, 10)
        period = 1.0 / max(self.pub_rate_hz, 1e-3)
        self.create_timer(period, self._publish_suction_joint_state)

        self.get_logger().info(
            "Ready.\n"
            f"  DO mapping: suction={self.do_suction}\n"
            f"  call_timeout_s={self.call_timeout_s}\n"
            f"  publishing suction joint: {self.suction_joint_name} "
            f"on {self.suction_joint_state_topic} @ {self.pub_rate_hz} Hz"
        )

    # ---------------- Remote command call ----------------
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
            self.get_logger().error(
                f"Service call exception for '{cmd}': {result_holder['exc']}")
            return False

        ok = result_holder["ok"]
        resp = result_holder["resp"]
        if ok:
            self.get_logger().info(f"OK: {cmd}")
        else:
            self.get_logger().error(
                f"Robot rejected: {cmd} -> cmd_res='{resp.cmd_res if resp else None}'")
        return ok

    def set_do(self, do_id: int, value: int) -> bool:
        return self.call_cmd(f"SetDO({do_id},{int(value)})")

    # ---------------- High-level commands ----------------
    def suction_on(self) -> bool:
        with self.cmd_lock:
            ok = self.set_do(self.do_suction, 1)
        if ok:
            with self._state_lock:
                self._suction_pos = self.pos_on
        return ok

    def suction_off(self) -> bool:
        with self.cmd_lock:
            ok = self.set_do(self.do_suction, 0)
        if ok:
            with self._state_lock:
                self._suction_pos = self.pos_off
        return ok

    # ---------------- JointState publish ----------------
    def _publish_suction_joint_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.suction_joint_name]
        with self._state_lock:
            msg.position = [float(self._suction_pos)]
        self.suction_js_pub.publish(msg)

    # ---------------- ROS service handlers ----------------
    def handle_on(self, req, res):
        res.success = self.suction_on()
        res.message = "suction_on" if res.success else "failed suction_on"
        return res

    def handle_off(self, req, res):
        res.success = self.suction_off()
        res.message = "suction_off" if res.success else "failed suction_off"
        return res

    # ---------------- Topic command handler ----------------
    def on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'on':
            self.suction_on()
        elif cmd == 'off':
            self.suction_off()
        else:
            self.get_logger().warn(f"Unknown suction/cmd: '{cmd}'")


def main():
    rclpy.init()
    node = FairinoSuctionNode()
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