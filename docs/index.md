---
layout: default
title: Home
nav_order: 1
---

# EPICS urRobot

<img src="./assets/EPICS_Logo-192x192.png" alt="Image 1" width="150">
<img src="./assets/1266x1776.webp" alt="Image 2" width="300">

This support module provides an EPICS interface to control a Universal
Robots e-series robot arm. It works by mapping function calls in the
[ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) library to EPICS PVs through asyn parameters.

## Key Features
- Access to large number of UR robot commands and status information
- Control individual joints and end-effector position
- Define joint or Cartesian space waypoints in EPICS PVs which have associated `move` functions which move the robot to a waypoint
- GUIs in CSS-Phoebus, MEDM, and caQtDM

## Interfaces
The primary method for controlling Universal Robots remotely is through connecting a client
to various servers hosted on the robot's controller. The `ur_rtde` library breaks up functionality
into four specific clients, or *interfaces*. For maximum flexibility, each interface in the
`ur_rtde` library can be loaded separately and each accepts multiple connections,
except the RTDE Control interface which you can only connect to in a single instance.

For an example on how to load all the available interfaces in a IOC startup script, see `urRobotApp/iocsh/urRobot.iocsh`.
In short, each "Config" function such as `URDashboardConfig` takes a name for the asyn port to create for the interface
and the IP address of the robot. Then to load the records, you must define the IOC prefix P,
asyn port PORT, and asyn address ADDR (which is typically zero).

### Dashboard Interface

The dashboard interface provides basic functionality for interacting with the robot such as:
- Loading, playing, pausing, and stopping URP programs that are saved in the controller
- Turning the robot and controller power on/off
- Releasing the breaks, closing popups, and restarting the safety configuration
- Basic status information such as the robot's mode, runtime state, and safety status

Below is an example of how the dashboard interface can be loaded from and an EPICS IOC startup script:
```
URDashboardConfig("asyn_dash", "192.168.1.1")
dbLoadRecords("$(URROBOT)/db/dashboard.db", "P=$(PREFIX), PORT=asyn_dash, ADDR=0")
```


### RTDE Receive Interface

The RTDE Receive Interface provides in-depth status information on the robot including:
- Runtime state and safety mode
- Joint positions, velocities, accelerations, moments, temperatures, currents, and voltages
- Tool center point (TCP) pose, speed, and force
- Tool accelerometer reading
- Momentum
- Main controller and robot voltages and currents
- Digital inputs and outputs
- Analog inputs and outputs

Below is an example of how the RTDE Receive interface can be loaded from an EPICS IOC startup script
```
RTDEReceiveConfig("asyn_rtde_recv", "192.168.1.1")
dbLoadRecords("$(URROBOT)/db/rtde_receive.db", "P=$(PREFIX), PORT=asyn_rtde_recv, ADDR=0")
```

### RTDE Control Interface

The RTDE Control Interface provides functions for moving the robot including:
- Moving individual joints to specified angles
- Moving the end-effector a specified pose

Below is an example of how the RTDE Control interface can be loaded from an EPICS IOC startup script
```
RTDEControlConfig("asyn_rtde_ctrl", "192.168.1.1")
dbLoadRecords("$(URROBOT)/db/rtde_control.db", "P=$(PREFIX), PORT=asyn_rtde_ctrl, ADDR=0")
```

### RTDE I/O Interface

The RTDE I/O interface provides functions for setting the digital and analog output pins on the robot
controller.

Below is an example of how the RTDE I/O interface can be loaded from an EPICS IOC startup script
```
RTDEInOutConfig("asyn_rtde_io", "192.168.1.1")
dbLoadRecords("$(URROBOT)/db/rtde_io.db", "P=$(PREFIX), PORT=asyn_rtde_io, ADDR=0")
```
