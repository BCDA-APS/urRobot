#ifndef _GRIPPER_DRIVER_HPP_
#define _GRIPPER_DRIVER_HPP_

#include "ur_rtde/robotiq_gripper.h"
#include "ur_rtde/dashboard_client.h"
#include <asynPortDriver.h>

static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char IS_OPEN_STRING[] = "IS_OPEN";
static constexpr char IS_CLOSED_STRING[] = "IS_CLOSED";
static constexpr char IS_STOPPED_INNER_STRING[] = "IS_STOPPED_INNER";
static constexpr char IS_STOPPED_OUTER_STRING[] = "IS_STOPPED_OUTER";
static constexpr char IS_ACTIVE_STRING[] = "IS_ACTIVE";
static constexpr char CONNECT_STRING[] = "CONNECT";
static constexpr char ACTIVATE_STRING[] = "ACTIVATE";
static constexpr char OPEN_STRING[] = "OPEN";
static constexpr char CLOSE_STRING[] = "CLOSE";
static constexpr char AUTO_CALIBRATE_STRING[] = "AUTO_CALIBRATE";
static constexpr char OPEN_POSITION_STRING[] = "OPEN_POSITION";
static constexpr char CLOSED_POSITION_STRING[] = "CLOSED_POSITION";
static constexpr char CURRENT_POSITION_STRING[] = "CURRENT_POSITION";
static constexpr char SET_SPEED_STRING[] = "SET_SPEED";
static constexpr char SET_FORCE_STRING[] = "SET_FORCE";
static constexpr char MOVE_STATUS_STRING[] = "MOVE_STATUS";
static constexpr char MIN_POSITION_STRING[] = "MIN_POSITION";
static constexpr char MAX_POSITION_STRING[] = "MAX_POSITION";
static constexpr char SET_POSITION_RANGE_STRING[] = "SET_POSITION_RANGE";
static constexpr char POSITION_UNIT_STRING[] = "POSITION_UNIT";
static constexpr char IS_CALIBRATED_STRING[] = "IS_CALIBRATED";

static constexpr int NUM_JOINTS = 6;
static constexpr int MAX_CONTROLLERS = 1;
static constexpr double POLL_PERIOD = 0.02; // 50Hz
static constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class URGripper : public asynPortDriver {
  public:
    URGripper(const char *asyn_port_name, const char *robot_port_name);
    virtual void poll(void);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

  private:
    std::unique_ptr<ur_rtde::RobotiqGripper> gripper_;
    std::unique_ptr<ur_rtde::DashboardClient> ur_dashboard_;
    const std::string robot_ip_ = "";
    bool try_connect();
    bool robot_on_ = false;

  protected:
    asynUser *pasynUserURRobot_;

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

#endif
