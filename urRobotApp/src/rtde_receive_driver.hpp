#ifndef _RTDE_RECEIVE_DRIVER_HPP_
#define _RTDE_RECEIVE_DRIVER_HPP_

#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>

// RTDE Receive Interface
static constexpr char DISCONNECT_STRING[] = "DISCONNECT";
static constexpr char RECONNECT_STRING[] = "RECONNECT";
static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char CONTROLLER_TIMESTAMP_STRING[] = "CONTROLLER_TIMESTAMP";
static constexpr char TARGET_JOINT_POS_STRING[] = "TARGET_JOINT_POSITIONS";
static constexpr char TARGET_JOINT_VEL_STRING[] = "TARGET_JOINT_VELOCITIES";
static constexpr char TARGET_JOINT_ACCEL_STRING[] = "TARGET_JOINT_ACCELERATIONS";
static constexpr char TARGET_JOINT_CURRENTS_STRING[] = "TARGET_JOINT_CURRENTS";
static constexpr char TARGET_JOINT_MOMENTS_STRING[] = "TARGET_JOINT_MOMENTS";
static constexpr char ACTUAL_JOINT_POS_STRING[] = "ACTUAL_JOINT_POSITIONS";
static constexpr char ACTUAL_JOINT_VEL_STRING[] = "ACTUAL_JOINT_VELOCITIES";
static constexpr char ACTUAL_JOINT_CURRENTS_STRING[] = "ACTUAL_JOINT_CURRENTS";
static constexpr char JOINT_CONTROL_CURRENTS_STRING[] = "JOINT_CONTROL_CURRENTS";
static constexpr char ACTUAL_TCP_POSE_STRING[] = "ACTUAL_TCP_POSE";
static constexpr char ACTUAL_TCP_SPEED_STRING[] = "ACTUAL_TCP_SPEED";
static constexpr char ACTUAL_TCP_FORCE_STRING[] = "ACTUAL_TCP_FORCE";
static constexpr char TARGET_TCP_POSE_STRING[] = "TARGET_TCP_POSE";
static constexpr char TARGET_TCP_SPEED_STRING[] = "TARGET_TCP_SPEED";
static constexpr char JOINT_TEMPERATURES_STRING[] = "JOINT_TEMPERATURES";
static constexpr char DIGITAL_INPUT_BITS_STRING[] = "DIGITAL_INPUT_BITS";
static constexpr char DIGITAL_OUTPUT_BITS_STRING[] = "DIGITAL_OUTPUT_BITS";
static constexpr char STD_ANALOG_INPUT0_STRING[] = "STD_ANALOG_INPUT0";
static constexpr char STD_ANALOG_INPUT1_STRING[] = "STD_ANALOG_INPUT1";
static constexpr char STD_ANALOG_OUTPUT0_STRING[] = "STD_ANALOG_OUTPUT0";
static constexpr char STD_ANALOG_OUTPUT1_STRING[] = "STD_ANALOG_OUTPUT1";
static constexpr char ROBOT_MODE_STRING[] = "ROBOT_MODE";
static constexpr char JOINT_MODES_STRING[] = "JOINT_MODES";
static constexpr char SAFETY_MODE_STRING[] = "SAFETY_MODE";
static constexpr char SAFETY_STATUS_BITS_STRING[] = "SAFETY_STATUS_BITS";
static constexpr char ACTUAL_TOOL_ACCEL_STRING[] = "ACTUAL_TOOL_ACCEL";
static constexpr char SPEED_SCALING_STRING[] = "SPEED_SCALING";
static constexpr char TARGET_SPEED_FRACTION_STRING[] = "TARGET_SPEED_FRACTION";
static constexpr char ACTUAL_MOMENTUM_STRING[] = "ACTUAL_MOMENTUM";
static constexpr char ACTUAL_MAIN_VOLTAGE_STRING[] = "ACTUAL_MAIN_VOLTAGE";
static constexpr char ACTUAL_ROBOT_VOLTAGE_STRING[] = "ACTUAL_ROBOT_VOLTAGE";
static constexpr char ACTUAL_ROBOT_CURRENT_STRING[] = "ACTUAL_ROBOT_CURRENT";
static constexpr char ACTUAL_JOINT_VOLTAGES_STRING[] = "ACTUAL_JOINT_VOLTAGES";
static constexpr char RUNTIME_STATE_STRING[] = "RUNTIME_STATE";

constexpr int MAX_CONTROLLERS = 1;
constexpr double POLL_PERIOD = 0.10; // seconds
constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class RTDEReceive : public asynPortDriver {
  public:
    RTDEReceive(const char *asyn_port_name, const char *robot_port_name);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual void poll(void);

  private:
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    const std::string robot_ip_ = "";
    bool try_connect();

  protected:
    asynUser *pasynUserURRobot_;

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
    int targetJointPosIndex_;
    int targetJointVelIndex_;
    int targetJointAccelIndex_;
    int targetJointCurrentsIndex_;
    int targetJointMomentsIndex_;
    int targetTCPPoseIndex_;
    int targetTCPSpeedIndex_;
    int jointTemperaturesIndex_;
    int speedScalingIndex_;
    int targetSpeedFractionIndex_;
    int actualMomentumIndex_;
    int actualMainVoltageIndex_;
    int actualRobotVoltageIndex_;
    int actualRobotCurrentIndex_;
    int actualJointVoltagesIndex_;
    int disconnectIndex_;
    int reconnectIndex_;

};

#endif
