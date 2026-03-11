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
Before installing fairino_gripper, first install the fairino_bridge package:
```bash
git clone https://github.com/LearnRoboticsWROS/fairino_bridge_master_class
```
This is required because fairino_bridge provides the MoveIt bridge for the 6-axis Fairino arm.

---

## Installation 
1. Go to your workspace source folder
```bash
cd ~/fr_ws/src
```

2. Install fairino_bridge first
Follow the instructions in:
https://github.com/LearnRoboticsWROS/fairino_bridge

3. Clone this repository
```bash
git clone https://github.com/LearnRoboticsWROS/fairino_bridge_gripper_master_class
```

4. Build the workspace
```bash
cd ~/fr_ws
colcon build
```

5. Source the workspace
```bash
source ~/fr_ws/install/setup.bash
```

---

## Phase 1 - Digital Output Bridge

- Purpose

The first part of the package exposes a high-level ROS 2 interface for the gripper, instead of forcing the user to call raw vendor-specific commands.

Instead of doing this:
SetDO(0,1)
SetDO(1,0)

you can use:

/gripper/open

/gripper/close

/gripper/idle

/gripper/suction_on

/gripper/suction_off

This makes the application logic cleaner and easier to port to another robot platform in the future.

# Main Node
- The main node is:
fairino_gripper_node, created in the file fairino_gripper_control.py

It controls a pneumatic soft gripper through Fairino digital outputs.

- Exposed ROS 2 services

/gripper/open

/gripper/close

/gripper/idle

/gripper/suction_on

/gripper/suction_off

- Optional ROS 2 topic command

/gripper/cmd

Supported string commands:

open

close

idle

suction_on

suction_off

Published topic

/gripper/joint_states

This publishes a fake gripper joint state so that MoveIt and RViz can represent the gripper state.

# High-Level Behavior of fairino_gripper_node

This node performs five main tasks.

1. Connects to the Fairino remote command service

It creates a client for:
/fairino_remote_command_service

using
fairino_msgs/srv/RemoteCmdInterface
This is the low-level communication channel to the Fairino controller.

2. Exposes hardware-agnostic ROS 2 services

Instead of directly calling SetDO(...), external nodes can simply call:

/gripper/open
/gripper/close
/gripper/idle
This is the main abstraction layer introduced by the package.

3. Translates high-level commands into Fairino SDK strings

Internally, the node still needs to send vendor-specific commands.

For example:

- open may generate:
SetDO(open_DO,1)
SetDO(close_DO,0)

- close may generate:
SetDO(open_DO,0)
SetDO(close_DO,1)

- idle may generate:
SetDO(open_DO,0)
SetDO(close_DO,0)

The important point is that the rest of your ROS application never needs to deal with this syntax directly.

4. Enforces safe pneumatic control logic

The node applies a break-before-make rule.

This means:
. both outputs are first turned OFF
. the node waits for a short interlock delay
. only the desired output is enabled

This avoids having both DO_OPEN and DO_CLOSE active at the same time.

Supplier rule

. DO_OPEN and DO_CLOSE must never be ON simultaneously

State mapping
```bash
OPEN → DO_OPEN=1, DO_CLOSE=0
CLOSE → DO_OPEN=0, DO_CLOSE=1
IDLE → DO_OPEN=0, DO_CLOSE=0
```

5. Publishes a fake gripper joint state

The node publishes:
```bash
/gripper/joint_states
```
with a single joint, for example:
```bash
softgripper_joint
```
Typical software positions are:

0.0 = open
1.0 = close
0.5 = idle

This is not real encoder feedback.
It is a software representation used to make MoveIt and RViz behave correctly.

# Adapting the Node to Another Robot
This package is hardware agnostic at the ROS interface level, but the low-level digital output string still depends on the robot SDK.

The Fairino implementation uses:
```bash
def set_do(self, do_id: int, value: int) -> bool:
    return self.call_cmd(f"SetDO({do_id},{int(value)})")
```

For Fairino, the expected syntax is:
SetDO(<do_id>,<value>)

If your robot controller uses a different syntax, this is the part you need to adapt.

Examples of possible alternatives:

DO_SET(0,1)

set_digital_output(0, true)

or a custom service/API exposed by another robot brand.

- What changes

You adapt the function that generates the low-level robot command.

- What stays the same

Your high-level ROS 2 interface can remain unchanged:
/gripper/open
/gripper/close
/gripper/idle

That is the main architectural goal of this package.

# Phase 1 Test Procedure
1. Build the workspace
```bash
cd ~/fr_ws
colcon build
2. Source the workspace
```bash
source ~/fr_ws/install/setup.bash
```
3. Start the Fairino low-level command server
In a first terminal:
```bash
source ~/fr_ws/install/setup.bash
ros2 run fairino_hardware ros2_cmd_server
```

4. Start the gripper bridge node
In a second terminal:
```bash
source ~/fr_ws/install/setup.bash
ros2 run fairino_gripper fairino_gripper_node
```

5. Test the high-level gripper services
In a third terminal:
```bash
source ~/fr_ws/install/setup.bash
ros2 service call /gripper/idle std_srvs/srv/Trigger {}

ros2 service call /gripper/open std_srvs/srv/Trigger {}

ros2 service call /gripper/close std_srvs/srv/Trigger {}
```
If everything is configured correctly, you should see the digital outputs switching as expected.

## Phase 2 — MoveIt Gripper Bridge

Once the service-based gripper control works, the next step is to expose a standard MoveIt gripper interface.

MoveIt expects a gripper controller based on:
```bash
control_msgs/action/GripperCommand
```
To support that, this package adds a second bridge layer.

fairino_gripper_action_server (file gripper_action_server.py)

This node creates an action server:
```bash
/softgripper_controller/gripper_command
```
of type:
```bash
control_msgs/action/GripperCommand
```

Its role is to translate MoveIt gripper commands into the ROS 2 Trigger services already exposed by fairino_gripper_node.

# Flow
MoveIt
  ↓
GripperCommand action
  ↓
fairino_gripper_action_server
  ↓
/gripper/open or /gripper/close or /gripper/idle
  ↓
fairino_gripper_node
  ↓
/fairino_remote_command_service
  ↓
SetDO(...)

High-Level Behavior of fairino_gripper_action_server
1. Receives a GripperCommand goal

MoveIt sends a gripper position request.

The node interprets it as follows:

around 0.0 → open

around 0.5 → idle

around 1.0 → close

2. Calls the corresponding Trigger service

Depending on the requested position, it calls one of:

/gripper/open

/gripper/close

/gripper/idle

3. Always returns a result to MoveIt

This is very important.

The action must always finish with either:

succeed

abort

Otherwise MoveIt execution may remain blocked.

4. Supports cancellation

If the action is canceled, the node attempts a best-effort call to:

/gripper/idle

This is useful for safer gripper behavior.

Why This Action Bridge Is Needed

The service-based interface from Phase 1 is great for:

manual testing

direct scripting

custom ROS applications

But MoveIt expects a standard action interface for gripper execution.

So this node acts as the adapter between:

MoveIt expectations

your custom service-based gripper interface

Joint Feedback for MoveIt

MoveIt also requires a single coherent /joint_states stream.

At this point you have:

arm joint states from the Fairino arm bridge

fake gripper joint state from fairino_gripper_node

These must be merged before MoveIt consumes them.

joint_state_merger

This node merges:

/joint_states_robot

/gripper/joint_states

into:

/joint_states

Inputs

/joint_states_robot

/gripper/joint_states

Output

/joint_states

Why the Merger Is Needed

MoveIt and robot_state_publisher expect one complete joint state stream.

If the arm and gripper publish on different topics, MoveIt may not reconstruct the full robot configuration correctly.

joint_state_merger solves this by:

caching the latest robot arm joint state

caching the latest gripper joint state

periodically publishing a merged JointState

The merged output keeps a stable order:

arm joints first

gripper joint after that

Phase 2 Test Procedure

Before launching MoveIt, start the full bridge launch file:

source ~/fr_ws/install/setup.bash
ros2 launch fairino_bridge bridge_fr3wml_gripper.launch.py

This should bring up the full bridge stack needed for:

6-axis arm control

gripper action control

merged joint states

At this point, MoveIt should be able to control both:

the 6-axis Fairino arm

the gripper

using standard ROS interfaces.

Expected Final Control Flow
Arm side
MoveIt
  ↓
FollowJointTrajectory
  ↓
fairino_bridge
  ↓
Fairino controller
Gripper side
MoveIt
  ↓
GripperCommand
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
Feedback side
Arm feedback       → /joint_states_robot
Gripper feedback   → /gripper/joint_states
Both merged by     → joint_state_merger
Final topic        → /joint_states
Full Test Workflow
Terminal 1 — Fairino low-level command server
source ~/fr_ws/install/setup.bash
ros2 run fairino_hardware ros2_cmd_server
Terminal 2 — Full bridge stack
source ~/fr_ws/install/setup.bash
ros2 launch fairino_bridge bridge_fr3wml_gripper.launch.py
Terminal 3 — Launch MoveIt

Launch your MoveIt configuration as usual.

At this point, you should be able to:

move the 6-axis robot

command the gripper through GripperCommand

see a correct combined robot state in MoveIt and RViz

Package Philosophy

The important idea behind this package is not only gripper control.

It is the architectural pattern:

keep high-level logic ROS-native

isolate vendor-specific SDK syntax in a bridge node

expose reusable interfaces to the rest of the system

make the application easier to port to a different robot platform later

In other words:

application code should call ROS interfaces, not robot SDK strings.

Useful Commands
Build
cd ~/fr_ws
colcon build
Source
source ~/fr_ws/install/setup.bash
Run Fairino command server
ros2 run fairino_hardware ros2_cmd_server
Run gripper node
ros2 run fairino_gripper fairino_gripper_node
Test gripper services
ros2 service call /gripper/idle std_srvs/srv/Trigger {}
ros2 service call /gripper/open std_srvs/srv/Trigger {}
ros2 service call /gripper/close std_srvs/srv/Trigger {}
Run full bridge stack
ros2 launch fairino_bridge bridge_fr3wml_gripper.launch.py
Summary

fairino_gripper provides a ROS 2 bridge for gripper control on top of Fairino hardware.

It allows you to:

hide vendor-specific SetDO(...) syntax

expose high-level ROS 2 services for gripper control

expose a MoveIt-compatible GripperCommand action

publish gripper joint feedback

merge arm + gripper states into a single /joint_states

This makes the overall system much cleaner, more ROS-native, and easier to reuse across different robotic platforms.

