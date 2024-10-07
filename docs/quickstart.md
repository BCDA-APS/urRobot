---
layout: default
title: Quickstart
nav_order: 2
---

# Quickstart

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}

## Build Instructions

**Supported Operating Systems**

- Red Hat Enterprise Linux (RHEL) 9


**Dependencies**

- EPICS Base
- asyn
- [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) (included)

This support module has been tested to work with EPICS base 7.0.4.1, and asyn R4-42 (synApps 6-2-1), and newer versions (synApps 6-3).
The minimum required versions of base and asyn have not been tested.

The `ur_rtde` library comes pre-built for the supported operating systems.
See `urRobotApp/src/ur_rtde/release.txt` for the current included version of the `ur_rtde` library.

If you need the EPICS `urRobot` support on another operating system, so long as you have EPICS base and asyn, you'll just
need to build the `ur_rtde` library for your OS and replace the `ur_rtde` library files and headrs in urRobotApp/src.
Consult the `ur_rtde` library documentation on how to build it from source.

**Build**

- Once you have a working installation of EPICS base and asyn, clone or download [https://github.com/BCDA-APS/urRobot](https://github.com/BCDA-APS/urRobot)
- Next, open configure/RELEASE in a text editor and correct the paths to `EPICS_BASE` and `ASYN` if necessary
- Run `make` in the top level directory of the project

## Adding UR Robot support to an IOC

After you have successfully built the urRobot support module, follow the steps below to add
it to an IOC.

**1\.** Add the path to the urRobot support in configure/RELEASE

```
# file: configure/RELEASE
URROBOT=/path/to/urRobot
```

**2\.** Add rules in xxxApp/src/Makefile

```
# file: xxxApp/src/Makefile
ifdef URROBOT
  $(DBD_NAME)_DBD += urRobotSupport.dbd
  $(PROD_NAME)_LIBS := urRobot $($(PROD_NAME)_LIBS)
endif
```

**3\.** In your startup script, load the example script making sure to fill in the correct IP address for your robot.

```
# file: iocBoot/iocxxx/st.cmd.Linux
iocshLoad("$(URROBOT)/iocsh/urRobot.iocsh", "PREFIX=$(PREFIX), IP=127.0.0.1")
```

**4\.** If you would like to add support for waypoints and paths, load the associated substitions files:

```
# file: iocBoot/iocxxx/st.cmd.Linux
dbLoadTemplate("$(URROBOT)/iocsh/waypoints.substitutions", "P=$(PREFIX)")
dbLoadTemplate("$(URROBOT)/iocsh/paths.substitutions", "P=$(PREFIX)")
```

If you want to load only specific interfaces to the robot controller, you should copy the `urRobot.iocsh` file
to your IOC and modify it as needed. The same goes for the waypoints and paths substitutions files.

**5\.** Before starting the IOC, using the teach pendant, make sure the robot is powered on, breaks released, in Automatic mode,
and in Remote Control mode. Note, if you are not using the RTDE control interface, all you need to do is make sure the robot
controller is powered on.

At this point you should be able to start the IOC. If all went well,
your IOC console should report messages (beginning with "[info]") saying each interface to the robot was successfully connected:
```bash
# ### urRobot.iocsh ###
# Set up UR Dashboard server
URDashboardConfig("asyn_dash", "164.54.104.148")
[2024-10-07 14:53:57.008] [info] Connected to UR Dashboard server
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/dashboard.db", "P=bcur:, R=, PORT=asyn_dash, ADDR=0")
# Set up UR RTDE Receive interface
RTDEReceiveConfig("asyn_rtde_recv", "164.54.104.148")
[2024-10-07 14:53:57.562] [info] Connected to UR RTDE Receive interface
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/rtde_receive.db", "P=bcur:, R=, PORT=asyn_rtde_recv, ADDR=0")
# Set up UR RTDE I/O interface
RTDEInOutConfig("asyn_rtde_io", "164.54.104.148")
[2024-10-07 14:53:58.356] [info] Connected to UR RTDE IO interface
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/rtde_io.db", "P=bcur:, R=, PORT=asyn_rtde_io, ADDR=0")
# Set up UR RTDE Control interface
RTDEControlConfig("asyn_rtde_ctrl", "164.54.104.148")
[2024-10-07 14:53:59.623] [info] Connected to UR RTDE Control interface
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/rtde_control.db", "P=bcur:, R=, PORT=asyn_rtde_ctrl, ADDR=0")
# Set up Robotiq Gripper
URGripperConfig("asyn_gripper", "164.54.104.148")
[2024-10-07 14:53:59.647] [info] Connected to gripper
dbLoadRecords("/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/db/robotiq_gripper.db", "P=bcur:, R=, MIN_POS=3, MAX_POS=248, AUTO_ACTIVATE=YES, PORT=asyn_gripper, ADDR=0")
# add URROBOT/urRobotApp/src to LUA_SCRIPT_PATH for waypoint functionality
epicsEnvSet("LUA_SCRIPT_PATH", "lua_scripts:/net/s100dserv/xorApps/epics/synApps_6_3/support/urRobot/urRobotApp/src")
-- iocshLoad("simple_urRobot.iocsh", "PREFIX=$(PREFIX), IP=164.54.104.148")
dbLoadTemplate("waypoints.substitutions", "P=$(PREFIX)")
dbLoadTemplate("paths.substitutions", "P=$(PREFIX)")
```

If something went wrong, you will see messages in the console (beginning with "[error]") which should explain the issue. The
most common problems are an incorrect IP address or the robot not being powered on and breaks releases (if using RTDE control
interface).

**6\.** To start the provided GUIs, copy the example scripts of your choosing to the top level directory of your IOC.
For example, to start caQtDM, use `urRobot/iocs/urExample/start_caQtDM_urRobot`. If you are not at the APS, you will most likely
need to adjust the paths to the caQtDM, MEDM, and Phoebus executables in these scripts.
