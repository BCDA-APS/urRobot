# urRobot
EPICS module for controlling a Universal Robots e-series robot arm.

## Dependencies
- EPICS Base 7.0.8
- asyn R4-42
- [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde)
- spdlog

Of these dependencies however, you will only need to worry about EPICS base and asyn. The `ur_rtde` library
comes pre-built for the supported operating systems, and `spdlog` which is used for logging is a header only
library which comes included with the epics-urRobot support as well. For both of these libraries, you will find
a "release.txt" file in their respective directories in urRobotApp/src/ which notes their version.
