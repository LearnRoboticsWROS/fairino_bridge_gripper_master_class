#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class JointStateMerger(Node):
    """
    Merges robot JointState + gripper JointState into a single /joint_states.

    - robot source:   /joint_states_robot (from fairino_bridge remapped)
    - gripper source: /gripper/joint_states (from fairino_gripper_control)
    - output:         /joint_states (single publisher for MoveIt/RSP)
    """

    def __init__(self):
        super().__init__('joint_state_merger')

        # -------- params --------
        self.declare_parameter('robot_joint_states_in', '/joint_states_robot')
        self.declare_parameter('gripper_joint_states_in', '/gripper/joint_states')
        self.declare_parameter('joint_states_out', '/joint_states')
        self.declare_parameter('publish_rate_hz', 50.0)

        self.robot_in = self.get_parameter('robot_joint_states_in').value
        self.gripper_in = self.get_parameter('gripper_joint_states_in').value
        self.out_topic = self.get_parameter('joint_states_out').value
        self.rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # -------- caches --------
        self._lock = threading.Lock()
        self._robot_msg = None
        self._gripper_msg = None

        # -------- pub/sub --------
        self.sub_robot = self.create_subscription(JointState, self.robot_in, self._on_robot, 50)
        self.sub_gripper = self.create_subscription(JointState, self.gripper_in, self._on_gripper, 50)
        self.pub = self.create_publisher(JointState, self.out_topic, 50)

        period = 1.0 / max(self.rate_hz, 1e-3)
        self.create_timer(period, self._publish_merged)

        self.get_logger().info(
            "JointStateMerger ready:\n"
            f"  robot_in:   {self.robot_in}\n"
            f"  gripper_in: {self.gripper_in}\n"
            f"  out:        {self.out_topic}\n"
            f"  rate:       {self.rate_hz} Hz"
        )

    def _on_robot(self, msg: JointState):
        with self._lock:
            self._robot_msg = msg

    def _on_gripper(self, msg: JointState):
        with self._lock:
            self._gripper_msg = msg

    def _publish_merged(self):
        with self._lock:
            r = self._robot_msg
            g = self._gripper_msg

        if r is None:
            # robot must exist, otherwise nothing useful for MoveIt
            return

        out = JointState()
        # time: use "now" to avoid stale timestamp issues
        out.header.stamp = self.get_clock().now().to_msg()

        # Merge by name -> keeps latest for each joint
        merged = {}

        def add(js: JointState):
            if js is None:
                return
            n = list(js.name)
            p = list(js.position) if len(js.position) == len(n) else []
            v = list(js.velocity) if len(js.velocity) == len(n) else []
            e = list(js.effort) if len(js.effort) == len(n) else []
            for i, name in enumerate(n):
                merged[name] = (
                    p[i] if p else 0.0,
                    v[i] if v else 0.0,
                    e[i] if e else 0.0
                )

        add(r)
        add(g)

        # Stable order: robot joints first (as in r.name), then the rest
        ordered_names = []
        seen = set()

        for name in r.name:
            if name in merged and name not in seen:
                ordered_names.append(name); seen.add(name)

        for name in merged.keys():
            if name not in seen:
                ordered_names.append(name); seen.add(name)

        out.name = ordered_names
        out.position = [merged[n][0] for n in ordered_names]
        out.velocity = [merged[n][1] for n in ordered_names]
        out.effort = [merged[n][2] for n in ordered_names]

        self.pub.publish(out)


def main():
    rclpy.init()
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()