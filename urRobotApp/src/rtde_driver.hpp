#ifndef _RTDE_DRIVER_HPP_
#define _RTDE_DRIVER_HPP_

#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_io_interface.h"
#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>

// RTDE Receive Interface
static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char CONTROLLER_TIMESTAMP_STRING[] = "CONTROLLER_TIMESTAMP";
static constexpr char SAFETY_STATUS_BITS_STRING[] = "SAFETY_STATUS_BITS";
static constexpr char RUNTIME_STATE_STRING[] = "RUNTIME_STATE";
static constexpr char ROBOT_MODE_STRING[] = "ROBOT_MODE";
static constexpr char SAFETY_MODE_STRING[] = "SAFETY_MODE";
static constexpr char JOINT_MODES_STRING[] = "JOINT_MODES";
static constexpr char STD_ANALOG_INPUT0_STRING[] = "STD_ANALOG_INPUT0";
static constexpr char STD_ANALOG_INPUT1_STRING[] = "STD_ANALOG_INPUT1";
static constexpr char STD_ANALOG_OUTPUT0_STRING[] = "STD_ANALOG_OUTPUT0";
static constexpr char STD_ANALOG_OUTPUT1_STRING[] = "STD_ANALOG_OUTPUT1";

static constexpr char DIGITAL_INPUT_BITS_STRING[] = "DIGITAL_INPUT_BITS";
static constexpr char DIGITAL_OUTPUT_BITS_STRING[] = "DIGITAL_OUTPUT_BITS";

static constexpr char ACTUAL_JOINT_POS_STRING[] = "ACTUAL_JOINT_POSITIONS";
static constexpr char ACTUAL_JOINT_VEL_STRING[] = "ACTUAL_JOINT_VELOCITIES";
static constexpr char ACTUAL_JOINT_CURRENTS_STRING[] = "ACTUAL_JOINT_CURRENTS";
static constexpr char JOINT_CONTROL_CURRENTS_STRING[] = "JOINT_CONTROL_CURRENTS";
static constexpr char ACTUAL_TCP_POSE_STRING[] = "ACTUAL_TCP_POSE";
static constexpr char ACTUAL_TCP_SPEED_STRING[] = "ACTUAL_TCP_SPEED";
static constexpr char ACTUAL_TCP_FORCE_STRING[] = "ACTUAL_TCP_FORCE";
static constexpr char ACTUAL_TOOL_ACCEL_STRING[] = "ACTUAL_TOOL_ACCEL";

// RTDE IO Interface
static constexpr char SPEED_SLIDER_STRING[] = "SPEED_SLIDER";

// TODO: RTDE Control Interface

constexpr int MAX_CONTROLLERS = 1;
constexpr double POLL_PERIOD = 0.10; // seconds
constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class URRobotRTDE : public asynPortDriver {
  public:
    URRobotRTDE(const char *asyn_port_name, const char *robot_port_name);
    virtual void poll(void);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

  private:
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::RTDEIOInterface> rtde_io_;

  protected:
    asynUser *pasynUserURRobot_;

    // rtde_receive
    int isConnectedIndex_;
    int safetyStatusBitsIndex_;
    int runtimeStateIndex_;
    int robotModeIndex_;
    int controllerTimestampIndex_;
    int stdAnalogInput0Index_;
    int stdAnalogInput1Index_;
    int stdAnalogOutput0Index_;
    int stdAnalogOutput1Index_;
    int actualJointPosIndex_;
    int digitalInputBitsIndex_;
    int digitalOutputBitsIndex_;
    int actualJointVelIndex_;
    int actualJointCurrentsIndex_;
    int jointControlCurrentsIndex_;
    int actualTCPPoseIndex_;
    int actualTCPSpeedIndex_;
    int actualTCPForceIndex_;
    int safetyModeIndex_;
    int jointModesIndex_;
    int actualToolAccelIndex_;

    // rtde_io
    int speedSliderIndex_;

    // TODO: rtde_control
};

#endif
