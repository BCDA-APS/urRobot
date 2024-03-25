# urRobot EPICS Support Module
This support module provides an EPICS interface to control a Universal
Robots e-series robot arm. It works by mapping function calls in the
[ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) library to EPICS PVs through asyn parameters.

## Build Instructions

**Supported Operating Systems**

64-bit Linux is the only supported operating system. It was developed on Red Hat Enterprise Linux (RHEL) 9,
however it has been testing on Ubuntu 22.04 and should work on most 64-bit Linux systems regardless of
distribution. If you believe it should work on your OS or distribution and it does not, open a GitHub issue.

**Dependencies**

- EPICS Base 7.0.8
- asyn R4-42
- [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) (included)
- spdlog (included)

Of these dependencies, you will only need to worry about EPICS base and asyn. Although the above versions are
recommended, epics-urRobot may build with other versions. The ur_rtde library comes pre-built for the supported
operating systems, and spdlog which is used for logging is a header only library which comes included with the
this support module as well.

**Build**

- Once you have a working installation of EPICS base and asyn, clone or download epics-urRobot.
- Next, open configure/RELEASE in a text editor and correct the paths to `SUPPORT` and `ASYN` if necessary
- Run `make` in the top level directory of the project

## Quickstart

- Add the path to the urRobot support in configure/RELEASE

```
# file: configure/RELEASE
URROBOT=/path/to/urRobot
```

- Add rules in xxxApp/src/Makefile
```
# file: xxxApp/src/Makefile
ifdef URROBOT
  $(DBD_NAME)_DBD += urRobotSupport.dbd
  $(PROD_NAME)_LIBS := urRobot $($(PROD_NAME)_LIBS)
endif
```

- In your startup script, load the example script making sure to fill in the correct IP address for your robot.
```
# file: iocBoot/iocxxx/st.cmd.Linux
iocshLoad("$(URROBOT)/iocsh/urRobot.iocsh", "PREFIX=$(PREFIX), IP=127.0.0.1")
```
