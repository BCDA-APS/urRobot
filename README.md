# Universal Robots EPICS Support Module

[![Latest Release](https://img.shields.io/github/v/tag/BCDA-APS/urRobot?label=release)](https://github.com/BCDA-APS/urRobot/releases)

<p align="center">
     <a href="https://epics-controls.org/">
     <img src="https://github.com/user-attachments/assets/2cbe5ed1-ba67-444c-84c9-5b67479bb15a" width="20%" />
     </a>
     &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
     <a href="https://gitlab.com/sdurobotics/ur_rtde">
     <img src="https://github.com/user-attachments/assets/6f2e1e5c-58f9-46c8-9ad3-def3766d4f1f" width="10%" />
     </a>
     <a href="https://www.universal-robots.com/">
     <img src="https://github.com/user-attachments/assets/2fdcda46-18fa-49d6-8db2-c14c2cf0f9ec" width="20%" />
     </a>
</p>

This support module provides an EPICS interface to control a Universal
Robots e-series robot arm such as the UR3e and UR5e. It works by mapping function calls in the
[ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) library to EPICS PVs through asyn parameters.

## Key Features
- Dashboard control: power, brakes, program loading, popups
- Real-time joint positions, velocities, forces, temperatures, and safety status
- Motion control: moveJ, moveL, speed/acceleration parameters
- Digital and analog I/O, speed slider
- Robotiq Hand-E gripper support
- Joint-space and Cartesian-space waypoints with configurable actions
- Path definitions for sequencing waypoints
- GUIs for MEDM, caQtDM, and CSS-Phoebus

## Requirements
| Dependency | Version | Required |
|------------|---------|----------|
| EPICS Base | 7.0.4+  | Yes      |
| asyn       | R4-42+  | Yes      |
| lua        | synApps | No (waypoints/paths) |
| calc       | synApps | No (waypoints/paths) |
| std        | synApps | No (waypoints/paths) |
| busy       | synApps | No (waypoints/paths) |
The ur_rtde library (v1.5.7) ships pre-built for RHEL9 x86-64.

## Documentation
Full build instructions, IOC integration, usage guides, and PV reference:
**https://bcda-aps.github.io/urRobot/**

## License
Copyright (c) 2025 UChicago Argonne, LLC. See [LICENSE](LICENSE) for details.
