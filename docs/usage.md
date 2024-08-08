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

