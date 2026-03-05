#ifndef _RTDE_CONTROL_DRIVER_HPP_
#define _RTDE_CONTROL_DRIVER_HPP_

#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>


static constexpr int NUM_JOINTS = 6;
static constexpr int MAX_CONTROLLERS = 1;
static constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

enum class AsyncMotionStatus : int { WaitingMotion, WaitingAction, Done };
enum class AsyncRunning : int {False, Cartesian, Joint};

class RTDEControl : public asynPortDriver {
  public:
    RTDEControl(const char *asyn_port_name, const char *robot_port_name, double poll_period);
    virtual void poll(void);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);

  private:
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;

    const std::string robot_ip_ = "0.0.0.0";
    const double poll_period_ = 0.0;
    bool try_connect();

    // Commanded joint angles
    std::vector<double> cmd_joints_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Commanded end-effector pose (x,y,z,roll,pitch,yaw)
    std::vector<double> cmd_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // TCP pose offset
    std::vector<double> tcp_offset_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Parameters for moveJ and moveL
    double joint_speed_ = 0.5;   // rad/s
    double joint_accel_ = 1.4;   // rad/s/s
    double joint_blend_ = 0.0;   // m?
    double linear_speed_ = 0.05; // m/s
    double linear_accel_ = 0.5;  // m/s/s
    double linear_blend_ = 0.0;  // m?

    // handle asynchronous motion through paths, etc.
    bool async_move_ = true;
    AsyncRunning async_running_ = AsyncRunning::False;
    AsyncMotionStatus async_status_ = AsyncMotionStatus::Done;
    std::vector<double> waypoint_;

  protected:
    asynUser *pasynUserURRobot_;

    int disconnectIndex_;
    int reconnectIndex_;
    int isConnectedIndex_;
    int isSteadyIndex_;
    int actualQIndex_;
    int moveJIndex_;
    int stopJIndex_;
    int j1CmdIndex_;
    int j2CmdIndex_;
    int j3CmdIndex_;
    int j4CmdIndex_;
    int j5CmdIndex_;
    int j6CmdIndex_;
    int actualTCPPoseIndex_;
    int moveLIndex_;
    int stopLIndex_;
    int poseXCmdIndex_;
    int poseYCmdIndex_;
    int poseZCmdIndex_;
    int poseRollCmdIndex_;
    int posePitchCmdIndex_;
    int poseYawCmdIndex_;
    int tcpOffsetXIndex_;
    int tcpOffsetYIndex_;
    int tcpOffsetZIndex_;
    int tcpOffsetRollIndex_;
    int tcpOffsetPitchIndex_;
    int tcpOffsetYawIndex_;
    int reuploadCtrlScriptIndex_;
    int stopCtrlScriptIndex_;
    int asyncMoveIndex_;
    int asyncMoveDoneIndex_;
    int jointSpeedIndex_;
    int jointAccelIndex_;
    int jointBlendIndex_;
    int linearSpeedIndex_;
    int linearAccelIndex_;
    int linearBlendIndex_;
    int waypointMoveJIndex_;
    int waypointMoveLIndex_;
    int runWaypointActionIndex_;
    int waypointActionDoneIndex_;
    int teachModeIndex_;
    int triggerProtStopIndex_;
};

#endif
