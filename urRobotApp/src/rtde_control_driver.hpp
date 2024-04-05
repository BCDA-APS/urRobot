#ifndef _RTDE_CONTROL_DRIVER_HPP_
#define _RTDE_CONTROL_DRIVER_HPP_

#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"
#include <asynPortDriver.h>

// TODO: params for moveL/moveJ speed, accel,

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

static constexpr char LOAD_POSE_PATH_STRING[] = "LOAD_POSE_PATH";

static constexpr int NUM_JOINTS = 6;
static constexpr int MAX_CONTROLLERS = 1;
static constexpr double POLL_PERIOD = 0.02; // 50Hz
static constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

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
    const char *robot_ip_;
    bool try_connect();

    // Commanded joint angles
    std::vector<double> cmd_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // Commanded end-effector pose (x,y,z,roll,pitch,yaw)
    std::vector<double> cmd_pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  
    // List of end-effector waypoints to follow
    std::vector<std::vector<double>> pose_path;

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

    int loadPosePathIndex_;
};

#endif
