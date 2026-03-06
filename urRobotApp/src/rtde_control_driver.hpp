#pragma once
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>

enum class AsyncMotionStatus : int { WaitingMotion, WaitingAction, Done };

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
    AsyncMotionStatus async_status_ = AsyncMotionStatus::Done;
    std::string traj_file_path_;
    std::vector<double> waypoint_;
    std::function<void()> async_motion_func_;

  protected:
    asynUser *pasynUserURRobot_;

    int disconnectIndex_;
    int reconnectIndex_;
    int isConnectedIndex_;
    int isSteadyIndex_;
    int actualQIndex_;
    int actualTCPPoseIndex_;
    int moveJIndex_;
    int stopJIndex_;
    int moveLIndex_;
    int stopLIndex_;
    int jointCmdIndex_;
    int poseCmdIndex_;
    int tcpOffsetIndex_;
    int reuploadCtrlScriptIndex_;
    int stopCtrlScriptIndex_;
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
    int trajFileIndex_;
    int trajTypeIndex_;
    int trajMoveIndex_;
};
