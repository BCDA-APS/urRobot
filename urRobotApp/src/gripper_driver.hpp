#pragma once
#include "ur_rtde/dashboard_client.h"
#include "ur_rtde/robotiq_gripper.h"
#include <asynPortDriver.h>

class URGripper : public asynPortDriver {
  public:
    URGripper(const char* asyn_port_name, const char* robot_port_name, double poll_period);
    void poll(void);
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;
    asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value) override;

  private:
    std::unique_ptr<ur_rtde::RobotiqGripper> gripper_;
    std::unique_ptr<ur_rtde::DashboardClient> ur_dashboard_;
    const std::string robot_ip_ = "";
    const double poll_period_ = 0.0;
    bool try_connect();
    bool robot_on_ = false;

  protected:
    int isConnectedIndex_;
    int isOpenIndex_;
    int isClosedIndex_;
    int isStoppedInnerIndex_;
    int isStoppedOuterIndex_;
    int isActiveIndex_;
    int connectIndex_;
    int activateIndex_;
    int openIndex_;
    int closeIndex_;
    int setSpeedIndex_;
    int setForceIndex_;
    int autoCalibrateIndex_;
    int openPositionIndex_;
    int closedPositionIndex_;
    int currentPositionIndex_;
    int moveStatusIndex_;
    int minPositionIndex_;
    int maxPositionIndex_;
    int setPositionRangeIndex_;
    int positionUnitIndex_;
    int isCalibratedIndex_;
};
