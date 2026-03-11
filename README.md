# fairino_gripper

`fairino_gripper` is a ROS 2 Python package that provides a **hardware-agnostic gripper bridge** for Fairino-based systems.

Its purpose is to hide vendor-specific digital output commands such as:

```bash
{cmd_str: 'SetDO(0,0)'}
```
sent through:
```bash
/fairino_remote_command_service
```

and replace them with clean ROS 2 interfaces that are easier to reuse, maintain, and integrate with MoveIt.

This package follows a core ROS design principle:

keep the application layer hardware agnostic, and isolate vendor-specific syntax inside dedicated bridge nodes.

---

## Overview

The Fairino controller exposes a low-level service:
/fairino_remote_command_service
of type:
fairino_msgs/srv/RemoteCmdInterface

which accepts raw command strings such as:
SetDO(0,1)
SetDO(1,0)

That works, but it tightly couples your application logic to the Fairino SDK syntax.

fairino_gripper introduces two bridge layers:

Digital Output Bridge
Converts high-level ROS 2 services such as /gripper/open into Fairino-specific SetDO(...) commands.

MoveIt Gripper Bridge
Converts a standard control_msgs/action/GripperCommand action into those high-level ROS 2 gripper services.

As a result, your application can control the gripper using ROS-native interfaces instead of robot-vendor command strings.

---

## Architecture

Phase 1 — Digital Output Bridge

This phase provides a ROS 2 service-based interface for controlling the gripper.

/gripper/open
    ↓
fairino_gripper_node
    ↓
/fairino_remote_command_service
    ↓
SetDO(...)
    ↓
Robot controller digital outputs


Phase 2 — MoveIt Gripper Bridge

This phase exposes a standard MoveIt-compatible gripper action interface.

MoveIt / RViz
    ↓
control_msgs/action/GripperCommand
    ↓
fairino_gripper_action_server
    ↓
/gripper/open | /gripper/close | /gripper/idle
    ↓
fairino_gripper_node
    ↓
/fairino_remote_command_service
    ↓
SetDO(...)

Joint State Feedback

To let MoveIt work properly, the package also publishes a gripper joint state and merges it with the robot arm joint states.

Robot arm joint states      → /joint_states_robot
Gripper fake joint state    → /gripper/joint_states
Both streams                → joint_state_merger
Final output                → /joint_states

Prerequisites

This package must be installed in the same ROS 2 workspace that already contains fairino_hardware.

Example workspace:
```bash
~/fr_ws/src
```

