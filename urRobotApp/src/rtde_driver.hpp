#include "ur_rtde/rtde_receive_interface.h"
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_io_interface.h"
#include <asynPortDriver.h>

// RTDE Receive Interface
static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char CONTROLLER_TIMESTAMP_STRING[] = "CONTROLLER_TIMESTAMP";
static constexpr char SAFETY_STATUS_STRING[] = "SAFETY_STATUS";
static constexpr char RUNTIME_STATE_STRING[] = "RUNTIME_STATE";
static constexpr char ROBOT_MODE_STRING[] = "ROBOT_MODE";
static constexpr char STD_ANALOG_INPUT0_STRING[] = "STD_ANALOG_INPUT0";
static constexpr char STD_ANALOG_INPUT1_STRING[] = "STD_ANALOG_INPUT1";
static constexpr char STD_ANALOG_OUTPUT0_STRING[] = "STD_ANALOG_OUTPUT0";
static constexpr char STD_ANALOG_OUTPUT1_STRING[] = "STD_ANALOG_OUTPUT1";
static constexpr char ACTUAL_JOINT_POS_STRING[] = "ACTUAL_JOINT_POSITIONS";

// TODO: RTDE Control Interface
// TODO: RTDE IO Interface


constexpr int MAX_CONTROLLERS = 1;
constexpr double DEFAULT_POLL_TIME = 0.20;
constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class URRobotRTDE : public asynPortDriver {
  public:
    URRobotRTDE(const char *asyn_port_name, const char *robot_port_name);
    virtual void main_loop(void);

  private:
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::RTDEIOInterface> rtde_io_;
    double poll_time_;

  protected:
    asynUser *pasynUserURRobot_;

    // rtde_receive
    int isConnectedIndex_;
    int safetyStatusIndex_;
    int runtimeStateIndex_;
    int robotModeIndex_;
    int controllerTimestampIndex_;
    int stdAnalogInput0Index_;
    int stdAnalogInput1Index_;
    int stdAnalogOutput0Index_;
    int stdAnalogOutput1Index_;
    int actualJointPosIndex_;

    // TODO: rtde_control
    // TODO: rtde_io
};





