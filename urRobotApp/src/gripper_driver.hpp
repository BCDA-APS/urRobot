#ifndef _GRIPPER_DRIVER_HPP_
#define _GRIPPER_DRIVER_HPP_

#include "ur_rtde/robotiq_gripper.h"
#include <asynPortDriver.h>

static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char CONNECT_STRING[] = "CONNECT";

static constexpr int NUM_JOINTS = 6;
static constexpr int MAX_CONTROLLERS = 1;
static constexpr double POLL_PERIOD = 0.02; // 50Hz
static constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class Gripper : public asynPortDriver {
  public:
    Gripper(const char *asyn_port_name, const char *robot_port_name);
    virtual void poll(void);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    // virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    // virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);

  private:
    std::unique_ptr<ur_rtde::RobotiqGripper> robotiq_gripper_;
    const char *robot_ip_;
    bool try_connect();

  protected:
    asynUser *pasynUserURRobot_;

    int isConnectedIndex_;
    int connectIndex_;
};

#endif
