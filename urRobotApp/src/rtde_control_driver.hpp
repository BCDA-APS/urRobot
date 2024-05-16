#ifndef _RTDE_CONTROL_DRIVER_HPP_
#define _RTDE_CONTROL_DRIVER_HPP_

#include "ur_rtde/robotiq_gripper.h"
#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>

static constexpr char DISCONNECT_STRING[] = "DISCONNECT";
static constexpr char RECONNECT_STRING[] = "RECONNECT";
static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";
static constexpr char IS_STEADY_STRING[] = "IS_STEADY";

static constexpr char ACTUAL_Q_STRING[] = "ACTUAL_Q";
static constexpr char MOVEJ_STRING[] = "MOVEJ";
static constexpr char STOPJ_STRING[] = "STOPJ";
static constexpr char J1CMD_STRING[] = "J1CMD";
static constexpr char J2CMD_STRING[] = "J2CMD";
static constexpr char J3CMD_STRING[] = "J3CMD";
static constexpr char J4CMD_STRING[] = "J4CMD";
static constexpr char J5CMD_STRING[] = "J5CMD";
static constexpr char J6CMD_STRING[] = "J6CMD";

static constexpr char ACTUAL_TCP_POSE_STRING[] = "ACTUAL_TCP_POSE";
static constexpr char MOVEL_STRING[] = "MOVEL";
static constexpr char STOPL_STRING[] = "STOPL";
static constexpr char POSE_X_CMD_STRING[] = "POSE_X_CMD";
static constexpr char POSE_Y_CMD_STRING[] = "POSE_Y_CMD";
static constexpr char POSE_Z_CMD_STRING[] = "POSE_Z_CMD";
static constexpr char POSE_ROLL_CMD_STRING[] = "POSE_ROLL_CMD";
static constexpr char POSE_PITCH_CMD_STRING[] = "POSE_PITCH_CMD";
static constexpr char POSE_YAW_CMD_STRING[] = "POSE_YAW_CMD";

static constexpr char PLAY_POSE_PATH_STRING[] = "PLAY_POSE_PATH";
static constexpr char PLAY_JOINT_PATH_STRING[] = "PLAY_JOINT_PATH";
static constexpr char REUPLOAD_CTRL_SCRIPT_STRING[] = "REUPLOAD_CONTROL_SCRIPT";
static constexpr char STOP_CTRL_SCRIPT_STRING[] = "STOP_CONTROL_SCRIPT";
static constexpr char JOINT_SPEED_STRING[] = "JOINT_SPEED";
static constexpr char JOINT_ACCEL_STRING[] = "JOINT_ACCELERATION";
static constexpr char JOINT_BLEND_STRING[] = "JOINT_BLEND";
static constexpr char ASYNC_MOVE_STRING[] = "ASYNC_MOVE";

static constexpr char ASYNC_MOVE_DONE_STRING[] = "ASYNC_MOVE_DONE";

static constexpr char LINEAR_SPEED_STRING[] = "LINEAR_SPEED";
static constexpr char LINEAR_ACCEL_STRING[] = "LINEAR_ACCELERATION";

static constexpr char WAYPOINT_MOVEJ_STRING[] = "WAYPOINT_MOVEJ";
static constexpr char WAYPOINT_GRIPPER_ACTION_STRING[] = "WAYPOINT_GRIPPER_ACTION";

static constexpr int NUM_JOINTS = 6;
static constexpr int MAX_CONTROLLERS = 1;
static constexpr double POLL_PERIOD = 0.02; // 50Hz
static constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

enum class AsyncMotionStatus : int { WaitingMotion, WaitingGripper, Done };

class RTDEControl : public asynPortDriver {
  public:
    RTDEControl(const char *asyn_port_name, const char *robot_port_name);
    virtual void poll(void);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);

  private:
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    std::unique_ptr<ur_rtde::RobotiqGripper> gripper_;

    const char *robot_ip_;
    bool try_connect();

    // Commanded joint angles
    std::vector<double> cmd_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Commanded end-effector pose (x,y,z,roll,pitch,yaw)
    std::vector<double> cmd_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Joint speed and acceleration to use with moveJ
    double joint_speed_ = 0.5;   // rad/s
    double joint_accel_ = 1.4;   // rad/s/s
    double joint_blend_ = 0.0;   // m
    double linear_speed_ = 0.05; // m/s
    double linear_accel_ = 0.5;  // m/s/s

    // handle asynchronous motion through paths, etc.
    bool async_move = true;
    bool async_running_ = false;
    AsyncMotionStatus async_status_ = AsyncMotionStatus::Done;
    int gripper_action_ = 0;
    std::vector<std::vector<double>> joint_path_;
    std::vector<std::vector<double>>::iterator joint_path_iter_;

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
    int playPosePathIndex_;
    int playJointPathIndex_;
    int reuploadCtrlScriptIndex_;
    int stopCtrlScriptIndex_;

    int asyncMoveIndex_;
    int asyncMoveDoneIndex_;

    int jointSpeedIndex_;
    int jointAccelIndex_;
    int jointBlendIndex_;

    int linearSpeedIndex_;
    int linearAccelIndex_;

    int waypointMoveJIndex_;
    int waypointGripperActionIndex_;
};

#endif
