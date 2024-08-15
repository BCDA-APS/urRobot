---
layout: default
title: Usage
nav_order: 3
---

# Usage

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}

If you haven't yet added the EPICS urRobot support to an IOC, please see
the Quickstart page for instructions.

Most of the basic functionality can be done through the provided GUIs using either
MEDM, caQtDM, or CSS-Phoebus. To start the screens, three bash scripts are
provided: `start_phoebus_urRobot` `start_MEDM_urRobot`, and `start_caQtdm_urRobot`

## CSS Phoebus GUIs

After running the `start_phoebus_urRobot` a script, the below screen will open.
The various buttons here will open the additional screens.

<img src="./assets/GUIs/urRobot_top_css.png" alt="css-top" width="500">


**Dashboard:**  
<img src="./assets/GUIs/urRobot_dashboard_css.png" alt="css-dashboard" width="500">

**RTDE Receive:**  
<img src="./assets/GUIs/urRobot_receive_css.png" alt="css-receive" width="600">

**RTDE I/O**  
<img src="./assets/GUIs/urRobot_io_css.png" alt="css-io" width="600">

**RTDE Control:**  
<img src="./assets/GUIs/urRobot_control_css.png" alt="css-receive" width="600">

**Robotiq Gripper**  
<img src="./assets/GUIs/urRobot_gripper_css.png" alt="css-gripper" width="600">

**WaypointL**  
<img src="./assets/GUIs/urRobot_waypointL_css.png" alt="css-waypointL" width="600">

**Waypoint Action**  
<img src="./assets/GUIs/urRobot_waypointL_action_css.png" alt="css-waypointL-action" width="600">


## MEDM/caQtDM

MEDM screens are provided and can be started with provided `start_MEDM_urRobot` script. These screens have been
converted to caQtDM as well and those can be started with `start_caQTDM_urRobot`.

**TODO:** Add screenshots

## Scripting

It is often useful to program the robot by interacting with the available PVs in a script.
Below an example python script using [PyEpics](https://github.com/pyepics/pyepics) is provided which demonstrates how to move
joint 6 (wrist) +10deg, then -10deg back to where it started.


```python
from epics import caget, caput

PREFIX = "bcur:" # replace with your IOC prefix

def wait_motion():
    '''block execution until commanded motion finishes'''
    while True:
        if caget(f"{PREFIX}Control:Steady") == 0:
            break
    while True:
        if caget(f"{PREFIX}Control:Steady") == 1:
            break

# Disable auto move
# when enabled, changing commanded values will automatically move
# when disabled, you need to call moveJ to trigger the move
caput(f"{PREFIX}Control:AutoMoveJ", 0)

# Set commanded joint positions to current position
caput(f"{PREFIX}Control:ResetJCmd", 1)
# the above is the same as doing the following:
#  caput(f"{PREFIX}Control:J1Cmd", joint_angles[0])
#  caput(f"{PREFIX}Control:J2Cmd", joint_angles[1])
#  caput(f"{PREFIX}Control:J3Cmd", joint_angles[2])
#  caput(f"{PREFIX}Control:J4Cmd", joint_angles[3])
#  caput(f"{PREFIX}Control:J5Cmd", joint_angles[4])
#  caput(f"{PREFIX}Control:J6Cmd", joint_angles[5])

# Move J6 +20deg
print("Moving Joint 6 +10deg...")
joint_angles = caget(f"{PREFIX}Receive:ActualJointPositions")
caput(f"{PREFIX}Control:J6Cmd", joint_angles[5]+10)
caput(f"{PREFIX}Control:moveJ", 1)
wait_motion()

# Move J6 back to where it started
print("Moving Joint 6 -10deg...")
joint_angles = caget(f"{PREFIX}Receive:ActualJointPositions")
caput(f"{PREFIX}Control:J6Cmd", joint_angles[5]-10)
caput(f"{PREFIX}Control:moveJ", 1)
wait_motion()
print("Done!")
```
