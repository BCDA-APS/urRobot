---
layout: default
title: PV Reference Guide
nav_order: 4
---

# PV Reference Guide

## Table of contents
{: .no_toc .text-delta }

- TOC
{:toc}


This page describes the available process variables included in the databases provided with the urRobot EPICS support module.
Note that the PVs below will typically have a `$(P)` macro in their name for the IOC prefix
which is provided when loading the EPICS databases. e.g. `dbLoadRecords(dashboard.db, "P=MyIOC:")`,
which would result is PVs like "MyIOC:Dashboard:Connected"

Also note that many of the binary output records below are "triggers" and the VAL/RVAL fields do not matter. The associated
functions will be called whenever the record processes. For example, to play the currently loaded program in the
robot controller, `caput Dashboard:Play 1` and `caput Dashboard:Play.PROC 1` (and even `caput Dashboard:Play 0`) will work.

## dashboard.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Dashboard:Connected    | bi     | 1 if connected to the dashboard client, otherwise 0     |
| Dashboard:IsProgramSaved    | bi     |  1 if the currently loaded program is saved, otherwise 0    |
| Dashboard:IsInRemoteControl    | bi     | 1 if the controller is in remote control mode, otherwise 0     |
| Dashboard:Running    | bi     | 1 if the currently loaded program is running, otherwise 0    |
| Dashboard:PolyscopeVersion    | stringin     | Polyscope version   |
| Dashboard:SerialNumber    | stringin     | Serial number     |
| Dashboard:ProgramState    | stringin     | Program state     |
| Dashboard:RobotMode    | stringin     | Robot mode     |
| Dashboard:RobotModel    | stringin     | Robot model     |
| Dashboard:LoadedProgram    | stringin     | Current loaded program in the robot controller     |
| Dashboard:SafetyStatus    | stringin     | Safety status     |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Dashboard:Play    | bo     | Plays the currently loaded program    |
| Dashboard:Stop    | bo     | Stops the running program     |
| Dashboard:Pause    | bo     | Pause running program     |
| Dashboard:Connect    | bo     | Connects to the dashboard server     |
| Dashboard:Disconnect    | bo     | Closes connection to the dashboard server     |
| Dashboard:Shutdown    | bo     | Shuts down robot and controller     |
| Dashboard:ClosePopup    | bo     | Closes popup on the teach pendant     |
| Dashboard:CloseSafetyPopup    | bo     | Closes safety popup on the teach pendant     |
| Dashboard:PowerOn    | bo     | Powers on the robot     |
| Dashboard:PowerOff    | bo     | Powers off the robot     |
| Dashboard:BrakeRelease    | bo     | Releases brakes     |
| Dashboard:UnlockProtectiveStop    | bo     | Unlocks protective stop     |
| Dashboard:RestartSafety    | bo     | Restarts safety, power off robot     |
| Dashboard:Popup    | stringout     | Generates a popup message on the teach pendant with the provided string     |
| Dashboard:LoadURP    | stringout     | Loads a URP program saved in the robot controller, e.g. `caput Dashboard:LoadURP my_program.urp` loads the program "/programs/my_program.urp" from the robot controller if the program exists |

***


## rtde_receive.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Receive:Connected    | bi     | 1 if connected to RTDE Receive interface, otherwise 0     |
| Receive:ControllerTimestamp    | ai     | Time since controller started     |
| Receive:SafetyStatusBits    | ai     | Bits 0-10: Is normal mode, Is reduced mode, Is protective stopped, Is recovery mode, Is safeguard stopped, Is system emergency stopped, Is robot emergency stopped, Is emergency stopped, Is violation, Is fault, Is stopped due to safety      |
| Receive:DigitalInputBits    | ai     | Digital input bits (18bits)     |
| Receive:DigitalOutputBits    | ai     | Digital output bits (18bits)     |
| Receive:RuntimeState    | ai     | Bits 0-5: Stopping, Stopped, Playing, Pausing, Paused, Resuming     |
| Receive:RobotMode    | ai     | -1=NO CONTROLLER, 0=DISCONNECTED, 1=CONFIRM SAFETY, 2=BOOTING, 3=POWER OFF, 4=POWER ON, 5=IDLE, 6=BACKDRIVE, 7=RUNNING, 8=UPDATING FIRMWARE      |
| Receive:SafetyMode    | ai     | Safety mode     |
| Receive:AnalogInput0    | ai     | Standard A0 input     |
| Receive:AnalogInput1    | ai     | Standard A1 input     |
| Receive:AnalogOutput0    | ai     | Standard A0 output     |
| Receive:AnalogOutput1    | ai     | Standard A1 output     |
| Receive:SpeedScaling    | ai     | Speed scaling     |
| Receive:TargetSpeedFraction    | ai     | Target speed between 0 and 1 corresponding to a value between 0% and 100% of the maximum speed     |
| Receive:ActualMomentum    | ai     | Norm of Cartesian linear momentum     |
| Receive:ActualMainVoltage    | ai     | Safety control board main voltage     |
| Receive:ActualRobotVoltage    | ai     | Safety control board robot voltage     |
| Receive:ActualRobotCurrent    | ai     | Safety control board robot current     |
| Receive:ActualJointPositions    | waveform     | Actual joint positions     |
| Receive:ActualJointVelocities    | waveform     | Actual joint velocities     |
| Receive:ActualJointCurrents    | waveform     | Actual joint currents     |
| Receive:JointControlCurrents    | waveform     | Joint control currents     |
| Receive:ActualTCPPose    | waveform     | Actual TCP pose (x,y,z,r,p,y)     |
| Receive:ActualTCPSpeed    | waveform     | Actual TCP speed     |
| Receive:ActualTCPForce    | waveform     | Actual TCP force     |
| Receive:JointModes    | waveform     | Joint control modes     |
| Receive:ActualToolAccelerometer    | waveform     | Tool accelerometer (X,Y,Z)     |
| Receive:TargetJointPositions    | waveform     | Target joint positions     |
| Receive:TargetJointVelocities    | waveform     | Target joint speeds     |
| Receive:TargetJointAccelerations    | waveform     | Target joint accelerations     |
| Receive:TargetJointCurrents    | waveform     | Target joint currents     |
| Receive:TargetJointMoments    | waveform     | Target joint currents     |
| Receive:TargetTCPPose    | waveform     | Target TCP pose (x,y,z,r,p,y)     |
| Receive:TargetTCPSpeed    | waveform     | Target TCP speed     |
| Receive:JointTemperatures    | waveform     | Joint temperatures in celsius     |
| Receive:ActualJointVoltages    | waveform     | Actual joint voltages     |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Receive:Disconnect    | bo     | Disconnects from the RTDE interface     |
| Receive:Reconnect    | bo     | Tries reconnecting to the RTDE interface     |


***

## rtde_control.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Control:Connected    | bi     | RTDE Control connection status     |
| Control:Steady    | bi     | 1 if robot is fully at rest     |
| Control:ActualQ    | waveform     | Actual joint positions     |
| Control:ActualQ_index0    | subArray     | Actual joint 1 angle     |
| Control:ActualQ_index1    | subArray     | actual joint 2 angle     |
| Control:ActualQ_index2    | subArray     | Actual joint 3 angle     |
| Control:ActualQ_index3    | subArray     | Actual joint 4 angle     |
| Control:ActualQ_index4    | subArray     | actual joint 5 angle     |
| Control:ActualQ_index5    | subArray     | Actual joint 6 angle     |
| Control:ActualTCPPose    | waveform     | Actual TCP Pose     |
| Control:pose_index0    | subArray     | X component of TCP pose     |
| Control:pose_index1    | subArray     | Y component of TCP pose     |
| Control:pose_index2    | subArray     | Z component of TCP pose     |
| Control:pose_index3    | subArray     | Roll component of TCP pose     |
| Control:pose_index4    | subArray     | Pitch component of TCP pose     |
| Control:pose_index5    | subArray     | Yaw component of TCP pose     |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| Control:Disconnect    | bo     | Disconnects from the RTDE interface     |
| Control:Reconnect    | bo     | Tries reconnecting to the RTDE interface     |
| Control:ReuploadControlScript    | bo     | Reuploads control script to controller     |
| Control:StopControlScript    | bo     | Stops the control script     |
| Control:Asynchronous    | bo     | If 1, moves will be asynchronous     |
| Control:moveJ    | bo     | Calls moveJ for the commanded joint angles J1Cmd-J6Cmd     |
| Control:JointSpeed    | ao     | Joint speeds for moveJ     |
| Control:JointAcceleration    | ao     | Joint accelerations for moveJ     |
| Control:stopJ    | bo     | Stops motion from moveJ only if asynchronous=True     |
| Control:auto_moveJ    | bo     | If 1, moveJ runs when JxCmd changes     |
| Control:ResetJCmd    | seq     | Resets J1Cmd-J6Cmd to current joint angles     |
| Control:J1Cmd    | ao     | Commanded angle(deg) for joint 1     |
| Control:J1TweakVal    | ao     | Joint 1 tweak step size     |
| Control:J1TweakFwd    | bo     | Tweak joint 1 forward     |
| Control:J1TweakRev    | bo     | Tweak joint 1 backward     |
| Control:J1TweakCalcFwd    | calcout     | Tweak joint 1 forwards by J1TweakVal     |
| Control:J1TweakCalcRev    | calcout     | Tweak joint 1 backwards by J1TweakVal     |
| Control:J2Cmd    | ao     | Commanded angle(deg) for joint 2     |
| Control:J2TweakVal    | ao     | Joint 2 tweak step size     |
| Control:J2TweakFwd    | bo     | Tweak joint 2 forward by J2TweakVal     |
| Control:J2TweakRev    | bo     | Tweak joint 2 backward by J2TweakVal     |
| Control:J3Cmd    | ao     | Commanded angle(deg) for joint 3     |
| Control:J3TweakVal    | ao     | Joint 3 tweak step size     |
| Control:J3TweakFwd    | bo     | Tweak joint 3 forward by J3TweakVal     |
| Control:J3TweakRev    | bo     | Tweak joint 3 backward by J3TweakVal     |
| Control:J4Cmd    | ao     | Commanded angle(deg) for joint 4     |
| Control:J4TweakVal    | ao     | Joint 4 tweak step size     |
| Control:J4TweakFwd    | bo     | Tweak joint 4 forward by J4TweakVal     |
| Control:J4TweakRev    | bo     | Tweak joint 4 backward by J4TweakVal     |
| Control:J5Cmd    | ao     | Commanded angle(deg) for joint 5     |
| Control:J5TweakVal    | ao     | Joint 5 tweak step size     |
| Control:J5TweakFwd    | bo     | Tweak joint 5 forward by J5TweakVal     |
| Control:J5TweakRev    | bo     | Tweak joint 5 backward by J5TweakVal     |
| Control:J6Cmd    | ao     | Commanded angle(deg) for joint 6     |
| Control:J6TweakVal    | ao     | Joint 6 tweak step size     |
| Control:J6TweakFwd    | bo     | Tweak joint 6 forward by J6TweakVal     |
| Control:J6TweakRev    | bo     | Tweak joint 6 backward by J6TweakVal     |
| Control:moveL    | bo     | Move to TCP pose linearly     |
| Control:stopL    | bo     | Stops motion from moveL only if asynchronous=True     |
| Control:ResetPoseCmd    | seq     | Resets the commanded TCP pose to current TCP pose     |
| Control:PoseXCmd    | ao     | Commanded TCP X     |
| Control:PoseXTweakVal    | ao     | X TCP pose tweak step size     |
| Control:PoseXTweakFwd    | bo     | Tweak X TCP pose forward     |
| Control:PoseXTweakRev    | bo     | Tweak X TCP pose backward     |
| Control:PoseYCmd    | ao     | Commanded TCP Y     |
| Control:PoseYTweakVal    | ao     | Y TCP pose tweak step size     |
| Control:PoseYTweakFwd    | bo     | Tweak Y TCP pose forward     |
| Control:PoseYTweakRev    | bo     | Tweak Y TCP pose backward     |
| Control:PoseZCmd    | ao     | Commanded TCP Z     |
| Control:PoseZTweakVal    | ao     | TCP Z tweak step size     |
| Control:PoseZTweakFwd    | bo     | Tweak TCP Z forward     |
| Control:PoseZTweakRev    | bo     | Tweak TCP Z backward     |
| Control:PoseRollCmd    | ao     | Commanded TCP roll     |
| Control:PoseRollTweakVal    | ao     | TCP roll tweak step size     |
| Control:PoseRollTweakFwd    | bo     | Tweak TCP roll forward     |
| Control:PoseRollTweakRev    | bo     | Tweak TCP roll backward     |
| Control:PosePitchCmd    | ao     | Commanded TCP pitch     |
| Control:PosePitchTweakVal    | ao     | TCP pitch tweak step size     |
| Control:PosePitchTweakFwd    | bo     | Tweak TCP pitch forward     |
| Control:PosePitchTweakRev    | bo     | Tweak TCP pitch backward     |
| Control:PoseYawCmd    | ao     | Commanded TCP yaw     |
| Control:PoseYawTweakVal    | ao     | TCP yaw tweak step size     |
| Control:PoseYawTweakFwd    | bo     | Tweak TCP yaw forward     |
| Control:PoseYawTweakRev    | bo     | Tweak TCP yaw backward     |
| Control:PlayPosePath    | stringout     | Moves through CSV file of EE waypoints     |
| Control:PlayJointPath    | stringout     | Moves through CSV file of joint angles with given speed, acceleration, blend, and gripper state     |

***


## rtde_io.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| IO:SpeedSlider    | ao     | Number between 0 and 1 to set the speed between 0% and 100% of the maximum speed     |
| IO:SetVoltageAnalogOut0    | ao     | Analog output voltage 0     |
| IO:SetVoltageAnalogOut1    | ao     | Analog output voltage 1     |
| IO:SetCurrentAnalogOut0    | ao     | Analog output current 0     |
| IO:SetCurrentAnalogOut1    | ao     | Analog output current 1     |
| IO:SetStandardDigitalOut0    | bo     | Standard digital output 0     |
| IO:SetStandardDigitalOut1    | bo     | Standard digital output 1     |
| IO:SetStandardDigitalOut2    | bo     | Standard digital output 2     |
| IO:SetStandardDigitalOut3    | bo     | Standard digital output 3     |
| IO:SetStandardDigitalOut4    | bo     | Standard digital output 4     |
| IO:SetStandardDigitalOut5    | bo     | Standard digital output 5     |
| IO:SetStandardDigitalOut6    | bo     | Standard digital output 6     |
| IO:SetStandardDigitalOut7    | bo     | Standard digital output 7     |
| IO:SetConfigurableDigitalOut0    | bo     | Configurable digital output 0     |
| IO:SetConfigurableDigitalOut1    | bo     | Configurable digital output 1     |
| IO:SetConfigurableDigitalOut2    | bo     | Configurable digital output 2     |
| IO:SetConfigurableDigitalOut3    | bo     | Configurable digital output 3     |
| IO:SetConfigurableDigitalOut4    | bo     | Configurable digital output 4     |
| IO:SetConfigurableDigitalOut5    | bo     | Configurable digital output 5     |
| IO:SetConfigurableDigitalOut6    | bo     | Configurable digital output 6     |
| IO:SetConfigurableDigitalOut7    | bo     | Configurable digital output 7     |
| IO:SetToolDigitalOut0    | bo     | Tool digital output 0     |
| IO:SetToolDigitalOut1    | bo     | Tool digital output 1     |

***

## robotiq_gripper.db

***Inputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| RobotiqGripper:Connected    | bi     | Connection status     |
| RobotiqGripper:Calibrated    | bi     | Calibration status     |
| RobotiqGripper:IsActive    | bi     | Gripper active status     |
| RobotiqGripper:IsOpen    | bi     | 1 if gripper open     |
| RobotiqGripper:IsClosed    | bi     | 1 if gripper closed     |
| RobotiqGripper:CurrentPosition    | ai     | Current position of gripper  |
| RobotiqGripper:OpenPosition    | ai     | Open position of gripper     |
| RobotiqGripper:ClosedPosition    | ai     | Closed position of gripper     |
| RobotiqGripper:MoveStatus    | mbbi     | Enum strings (from MoveStatusRaw) for gripper motion status |
| RobotiqGripper:MoveStatusRaw    | longin     | Integer representing the gripper motion status (see MoveStatus) |

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| RobotiqGripper:Connect    | bo     | Connects to Robotiq gripper     |
| RobotiqGripper:Activate    | bo     | Activates Robotiq gripper     |
| RobotiqGripper:AutoCalibrate    | bo     | Calibrates open/close positions     |
| RobotiqGripper:Open    | bo     | Opens Robotiq gripper     |
| RobotiqGripper:Close    | bo     | Closes Robotiq gripper     |
| RobotiqGripper:SetPositionRange    | bo     | Sets min/max posititions     |
| RobotiqGripper:SetSpeed    | ao     | Default gripper speed     |
| RobotiqGripper:SetForce    | ao     | Default gripper force     |
| RobotiqGripper:PositionUnit    | mbbo     | Enum strings for position units  |
| RobotiqGripper:MinPosition    | longout     | Minimum gripper position     |
| RobotiqGripper:MaxPosition    | longout     | Maximum gripper position     |
| RobotiqGripper:PositionUnitRaw    | longout     | Integer representing position units     |

***

## waypointJ.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| WaypointJ:$(N)    | stringout     | Waypoint name     |
| WaypointJ:$(N):Enabled    | bo     | 1 if waypoint is enabled otherwise 0     |
| WaypointJ:$(N):SetEnabled    | bo     | Sets WaypointJ:$(N):Enabled to 1     |
| WaypointJ:$(N):J1    | ao     | Joint 1 angle for waypoint (deg)    |
| WaypointJ:$(N):J2    | ao     | Joint 2 angle for waypoint (deg)    |
| WaypointJ:$(N):J3    | ao     | Joint 3 angle for waypoint (deg)    |
| WaypointJ:$(N):J4    | ao     | Joint 4 angle for waypoint (deg)    |
| WaypointJ:$(N):J5    | ao     | Joint 5 angle for waypoint (deg)    |
| WaypointJ:$(N):J6    | ao     | Joint 6 angle for waypoint (deg)    |
| WaypointJ:$(N):Action    | bo     | Used to define FLNK that points to waypoint action PV     |
| WaypointJ:$(N):ActionOpt    | mbbo     | Enum of available waypoint actions |
| WaypointJ:$(N):Speed    | ao     | Speed when moving to waypoint     |
| WaypointJ:$(N):Acceleration    | ao     | Acceleration when moving to waypoint     |
| WaypointJ:$(N):Blend    | ao     | Blend when moving to waypoint     |
| WaypointJ:$(N):Reached    | luascript     | 1 when robot is at waypoint otherwise 0     |
| WaypointJ:$(N):Reset    | seq     | Sets waypoint to the current location     |
| WaypointJ:$(N):moveJ    | bo     | Executes a moveJ to the waypoint if waypoint is enabled    |

***

## waypointL.db

***Outputs***

| Record  | Type   | Description   |
|-------------- | -------------- | -------------- |
| WaypointL:$(N)    | stringout     | Waypoint name     |
| WaypointL:$(N):Enabled    | bo     | 1 if waypoint is enabled otherwise 0     |
| WaypointL:$(N):SetEnabled    | bo     | Sets WaypointL:$(N):Enabled to 1     |
| WaypointL:$(N):X    | ao     | X position of waypoint     |
| WaypointL:$(N):Y    | ao     | Y position of waypoint     |
| WaypointL:$(N):Z    | ao     | Z position of waypoint     |
| WaypointL:$(N):Roll    | ao     | Roll angle of waypoint (deg)    |
| WaypointL:$(N):Pitch    | ao     | Pitch angle of waypoint (deg)   |
| WaypointL:$(N):Yaw    | ao     | Yaw angle of waypoint (deg)    |
| WaypointL:$(N):Action    | bo     | Used to define FLNK that points to waypoint action PV     |
| WaypointL:$(N):ActionOpt    | mbbo     | Enum of available waypoint actions |
| WaypointL:$(N):Speed    | ao     | Speed when moving to waypoint     |
| WaypointL:$(N):Acceleration    | ao     | Acceleration when moving to waypoint     |
| WaypointL:$(N):Blend    | ao     | Blend when moving to waypoint     |
| WaypointL:$(N):Reached    | luascript     | 1 when robot is at waypoint otherwise 0     |
| WaypointL:$(N):Reset    | seq     | Sets waypoint to the current location     |
| WaypointL:$(N):moveL    | bo     | Executes a moveL to the waypoint if waypoint is enabled    |

