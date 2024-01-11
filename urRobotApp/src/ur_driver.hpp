#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>

// RTDE Receive Interface:
static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char CONTROLLER_TIMESTAMP_STRING[] = "CONTROLLER_TIMESTAMP";
static constexpr char SAFETY_STATUS_STRING[] = "SAFETY_STATUS";
static constexpr char RUNTIME_STATE_STRING[] = "RUNTIME_STATE";
static constexpr char ROBOT_MODE_STRING[] = "ROBOT_MODE";

constexpr int MAX_CONTROLLERS = 1;
constexpr double DEFAULT_POLL_TIME = 0.20;
constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class URRobot : public asynPortDriver {
  public:
    URRobot(const char *asyn_port_name, const char *robot_port_name);
    virtual void main_loop(void);

  private:
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    double poll_time_;

  protected:
    asynUser *pasynUserURRobot_;
    int isConnectedIndex_;
    int safetyStatusIndex_;
    int runtimeStateIndex_;
    int robotModeIndex_;
    int controllerTimestampIndex_;
};
