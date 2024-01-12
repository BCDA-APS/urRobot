#include "ur_rtde/dashboard_client.h"
#include <asynPortDriver.h>

// Dashboard interface
static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";

constexpr int MAX_CONTROLLERS = 1;
constexpr double DEFAULT_POLL_TIME = 0.20;
constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class URRobotDashboard : public asynPortDriver {
  public:
    URRobotDashboard(const char *asyn_port_name, const char *robot_port_name);
    virtual void main_loop(void);

  private:
    std::unique_ptr<ur_rtde::DashboardClient> ur_dashboard_;
    double poll_time_;

  protected:
    asynUser *pasynUserURRobot_;
    int isConnectedIndex_;
};
