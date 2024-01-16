#include "ur_rtde/dashboard_client.h"
#include <asynPortDriver.h>

// Dashboard interface
static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char CLOSE_POPUP_STRING[] = "CLOSE_POPUP";
static constexpr char POPUP_STRING[] = "POPUP";
static constexpr char LOAD_URP_STRING[] = "LOAD_URP";
static constexpr char PLAY_STRING[] = "PLAY";
static constexpr char STOP_STRING[] = "STOP";
static constexpr char PAUSE_STRING[] = "PAUSE";
static constexpr char QUIT_STRING[] = "QUIT";
static constexpr char SHUTDOWN_STRING[] = "SHUTDOWN";
static constexpr char IS_RUNNING_STRING[] = "IS_RUNNING";

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
    int closePopupIndex_;
    int popupIndex_;
    int loadURPIndex_;
    int playIndex_;
    int stopIndex_;
    int pauseIndex_;
    int quitIndex_;
    int shutdownIndex_;
    int isRunningIndex_;
};
