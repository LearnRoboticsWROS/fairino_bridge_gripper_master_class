#!/usr/bin/env python3
import asyncio

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import GripperCommand
from std_srvs.srv import Trigger


class GripperActionServer(Node):
    """
    MoveIt/RViz -> control_msgs/action/GripperCommand
    Translates to Trigger services:
      /gripper/open, /gripper/close, /gripper/idle

    IMPORTANT:
    - This action MUST always finish (succeed/abort), otherwise MoveIt execution can block.
    """

    def __init__(self):
        super().__init__("fairino_gripper_action_server")
        self.cb = ReentrantCallbackGroup()

        # Params
        self.declare_parameter("open_service", "/gripper/open")
        self.declare_parameter("close_service", "/gripper/close")
        self.declare_parameter("idle_service", "/gripper/idle")
        self.declare_parameter("threshold", 0.5)

        # timeouts
        self.declare_parameter("wait_service_timeout_s", 2.0)
        self.declare_parameter("call_timeout_s", 5.0)

        # If True, after open/close we call idle automatically
        self.declare_parameter("auto_idle_after_action", False)
        self.declare_parameter("auto_idle_delay_s", 0.05)

        self.open_srv_name = self.get_parameter("open_service").value
        self.close_srv_name = self.get_parameter("close_service").value
        self.idle_srv_name = self.get_parameter("idle_service").value
        self.threshold = float(self.get_parameter("threshold").value)

        self.wait_timeout = float(self.get_parameter("wait_service_timeout_s").value)
        self.call_timeout = float(self.get_parameter("call_timeout_s").value)

        self.auto_idle = bool(self.get_parameter("auto_idle_after_action").value)
        self.auto_idle_delay = float(self.get_parameter("auto_idle_delay_s").value)

        # Service clients
        self.open_cli = self.create_client(Trigger, self.open_srv_name, callback_group=self.cb)
        self.close_cli = self.create_client(Trigger, self.close_srv_name, callback_group=self.cb)
        self.idle_cli = self.create_client(Trigger, self.idle_srv_name, callback_group=self.cb)

        # Action name MUST match what MoveIt expects:
        # /softgripper_controller/gripper_command
        self._as = ActionServer(
            self,
            GripperCommand,
            "softgripper_controller/gripper_command",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
            callback_group=self.cb
        )

        self.get_logger().info(
            "Ready. Action: /softgripper_controller/gripper_command -> "
            f"{self.open_srv_name}, {self.close_srv_name}, {self.idle_srv_name}"
        )

    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        # best effort: idle
        # NOTE: cancel callback is sync; do not block hard, just request idle if possible
        try:
            if self.idle_cli.wait_for_service(timeout_sec=0.1):
                self.idle_cli.call_async(Trigger.Request())
        except Exception:
            pass
        return CancelResponse.ACCEPT

    async def _call_trigger_async(self, client, srv_name: str) -> bool:
        # wait for service (non-blocking)
        end = self.get_clock().now().nanoseconds + int(self.wait_timeout * 1e9)
        while not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > end:
                self.get_logger().error(f"Service not available: {srv_name}")
                return False
            await asyncio.sleep(0.01)

        # call async + timeout
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

        # --- state decisiono ---
        if 0.45 <= pos <= 0.55:
            target = "idle"
            ok = await self._call_trigger_async(self.idle_cli, self.idle_srv_name)
            reported_pos = 0.5

        elif pos >= 0.9:
            target = "close"
            ok = await self._call_trigger_async(self.close_cli, self.close_srv_name)
            reported_pos = 1.0

        else:
            target = "open"
            ok = await self._call_trigger_async(self.open_cli, self.open_srv_name)
            reported_pos = 0.0

        # --- Results ---
        result = GripperCommand.Result()
        result.position = reported_pos
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = bool(ok)

        if ok:
            goal_handle.succeed()
            self.get_logger().info(f"Gripper -> {target} OK")
        else:
            goal_handle.abort()
            self.get_logger().error(f"Gripper -> {target} FAILED")

        return result



def main():
    rclpy.init()
    node = GripperActionServer()
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
