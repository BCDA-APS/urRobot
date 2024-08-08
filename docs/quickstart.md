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
See urRobotApp/src/ur_rtde/release.txt for the current included version of the `ur_rtde` library.

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

1\. Add the path to the urRobot support in configure/RELEASE

```
# file: configure/RELEASE
URROBOT=/path/to/urRobot
```

2\. Add rules in xxxApp/src/Makefile

```
# file: xxxApp/src/Makefile
ifdef URROBOT
  $(DBD_NAME)_DBD += urRobotSupport.dbd
  $(PROD_NAME)_LIBS := urRobot $($(PROD_NAME)_LIBS)
endif
```

3\. In your startup script, load the example script making sure to fill in the correct IP address for your robot.

```
# file: iocBoot/iocxxx/st.cmd.Linux
iocshLoad("$(URROBOT)/iocsh/urRobot.iocsh", "PREFIX=$(PREFIX), IP=127.0.0.1")
```
