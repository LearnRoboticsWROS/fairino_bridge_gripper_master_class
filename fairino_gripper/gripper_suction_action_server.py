#!/usr/bin/env python3
"""
MoveIt/RViz -> control_msgs/action/GripperCommand adapter for Suction Cup.

Translates GripperCommand to Trigger services:
  /suction/on, /suction/off

Position mapping:
  >= 0.5  -> suction ON  (position reported: 1.0)
  <  0.5  -> suction OFF (position reported: 0.0)
"""
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger


class SuctionActionServer(Node):

    def __init__(self):
        super().__init__("fairino_suction_action_server")
        self.cb = ReentrantCallbackGroup()

        # Params
        self.declare_parameter("on_service", "/suction/on")
        self.declare_parameter("off_service", "/suction/off")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("wait_service_timeout_s", 2.0)
        self.declare_parameter("call_timeout_s", 5.0)

        self.on_srv_name = self.get_parameter("on_service").value
        self.off_srv_name = self.get_parameter("off_service").value
        self.threshold = float(self.get_parameter("threshold").value)
        self.wait_timeout = float(self.get_parameter("wait_service_timeout_s").value)
        self.call_timeout = float(self.get_parameter("call_timeout_s").value)

        # Service clients
        self.on_cli = self.create_client(Trigger, self.on_srv_name, callback_group=self.cb)
        self.off_cli = self.create_client(Trigger, self.off_srv_name, callback_group=self.cb)

        # Action name MUST match MoveIt controller config:
        # /suctioncup_controller/gripper_command
        self._as = ActionServer(
            self,
            GripperCommand,
            "suctioncup_controller/gripper_command",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb
        )

        self.get_logger().info(
            "Ready. Action: /suctioncup_controller/gripper_command -> "
            f"{self.on_srv_name}, {self.off_srv_name}"
        )

    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        try:
            if self.off_cli.wait_for_service(timeout_sec=0.1):
                self.off_cli.call_async(Trigger.Request())
        except Exception:
            pass
        return CancelResponse.ACCEPT

    async def _call_trigger_async(self, client, srv_name: str) -> bool:
        end = self.get_clock().now().nanoseconds + int(self.wait_timeout * 1e9)
        while not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > end:
                self.get_logger().error(f"Service not available: {srv_name}")
                return False
            await asyncio.sleep(0.01)

        fut = client.call_async(Trigger.Request())
        try:
            resp = await asyncio.wait_for(fut, timeout=self.call_timeout)
            return bool(resp.success)
        except asyncio.TimeoutError:
            self.get_logger().error(f"Timeout calling {srv_name}")
            return False
        except Exception as e:
            self.get_logger().error(f"Exception calling {srv_name}: {e}")
            return False

    async def execute_cb(self, goal_handle):
        pos = float(goal_handle.request.command.position)

        if pos >= self.threshold:
            target = "on"
            ok = await self._call_trigger_async(self.on_cli, self.on_srv_name)
            reported_pos = 1.0
        else:
            target = "off"
            ok = await self._call_trigger_async(self.off_cli, self.off_srv_name)
            reported_pos = 0.0

        result = GripperCommand.Result()
        result.position = reported_pos
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = bool(ok)

        if ok:
            goal_handle.succeed()
            self.get_logger().info(f"Suction -> {target} OK")
        else:
            goal_handle.abort()
            self.get_logger().error(f"Suction -> {target} FAILED")

        return result


def main():
    rclpy.init()
    node = SuctionActionServer()
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


if __name__ == "__main__":
    main()