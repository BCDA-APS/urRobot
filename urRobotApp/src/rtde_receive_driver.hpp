#ifndef _RTDE_RECEIVE_DRIVER_HPP_
#define _RTDE_RECEIVE_DRIVER_HPP_

#include <asynPortDriver.h>
#include "ur_rtde/rtde_receive_interface.h"

constexpr int MAX_CONTROLLERS = 1;

class RTDEReceive : public asynPortDriver {
  public:
    RTDEReceive(const char *asyn_port_name, const char *robot_port_name, double poll_period);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual void poll(void);

  private:
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    const std::string robot_ip_ = "";
    const double poll_period_ = 0.0;
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
