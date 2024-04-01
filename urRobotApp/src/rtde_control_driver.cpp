#include <cstddef>
#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>
#include <stdexcept>

#include "rtde_control_driver.hpp"
#include "rtde_control_interface.h"
#include "spdlog/spdlog.h"
#include "ur_rtde/dashboard_client.h"

static void poll_thread_C(void *pPvt) {
    RTDEControl *pRTDEControl = (RTDEControl *)pPvt;
    pRTDEControl->poll();
}

bool RTDEControl::try_connect() {
    // RTDE class construction automatically tries connecting.
    // If this function hasn't been called or if connecting fails,
    // the rtde_control_ object will be a nullptr
    bool connected = false;

    if (rtde_control_ == nullptr) {
        try {
            // Can only connect to control interface when "Robotmode: RUNNING"
            auto dash = std::make_unique<ur_rtde::DashboardClient>(robot_ip_);
            dash->connect();
            if (dash->robotmode() == "Robotmode: RUNNING") {
                rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
            } else {
                spdlog::error("Unable to connect to UR RTDE Control Interface: Ensure robot is on "
                              "and brakes released");
                connected = false;
                dash->disconnect();
            }
            if (rtde_control_ != nullptr) {
                if (rtde_control_->isConnected()) {
                    spdlog::info("Connected to UR RTDE Control interface");
                    connected = true;

                    // connect to RTDE Receive to get current joint states,
                    // only if we successfully connect to RTDE Control
                    rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
                    if (rtde_receive_ == nullptr) {
                        // HACK: should never get here
                        throw std::runtime_error(
                            "Failed connecting to receive interface from RTDEControl class");
                    }
                }
            }
        } catch (const std::exception &e) {
            spdlog::error("Failed to connected to UR RTDE Control interface\n{}", e.what());
            connected = false;
        }
    } else {
        if (not rtde_control_->isConnected()) {
            spdlog::debug("Reconnecting to UR RTDE Control interface");
            rtde_control_->reconnect();
            rtde_receive_->reconnect();
            connected = true;
        }
    }
    return connected;
}

RTDEControl::RTDEControl(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask |
                         asynFloat64ArrayMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask |
                         asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1
                     0, 0),
      rtde_control_(nullptr), rtde_receive_(nullptr), robot_ip_(robot_ip) {

    createParam(DISCONNECT_STRING, asynParamInt32, &disconnectIndex_);
    createParam(RECONNECT_STRING, asynParamInt32, &reconnectIndex_);
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);
    createParam(IS_STEADY_STRING, asynParamInt32, &isSteadyIndex_);

    createParam(MOVEJ_STRING, asynParamInt32, &moveJIndex_);
    createParam(STOPJ_STRING, asynParamInt32, &stopJIndex_);
    createParam(ACTUAL_Q_STRING, asynParamFloat64Array, &actualQIndex_);
    createParam(J1CMD_STRING, asynParamFloat64, &j1CmdIndex_);
    createParam(J2CMD_STRING, asynParamFloat64, &j2CmdIndex_);
    createParam(J3CMD_STRING, asynParamFloat64, &j3CmdIndex_);
    createParam(J4CMD_STRING, asynParamFloat64, &j4CmdIndex_);
    createParam(J5CMD_STRING, asynParamFloat64, &j5CmdIndex_);
    createParam(J6CMD_STRING, asynParamFloat64, &j6CmdIndex_);

    createParam(MOVEL_STRING, asynParamInt32, &moveLIndex_);
    createParam(STOPL_STRING, asynParamInt32, &stopLIndex_);
    createParam(ACTUAL_TCP_POSE_STRING, asynParamFloat64Array, &actualTCPPoseIndex_);
    createParam(POSE_X_CMD_STRING, asynParamFloat64, &poseXCmdIndex_);
    createParam(POSE_Y_CMD_STRING, asynParamFloat64, &poseYCmdIndex_);
    createParam(POSE_Z_CMD_STRING, asynParamFloat64, &poseZCmdIndex_);
    createParam(POSE_ROLL_CMD_STRING, asynParamFloat64, &poseRollCmdIndex_);
    createParam(POSE_PITCH_CMD_STRING, asynParamFloat64, &posePitchCmdIndex_);
    createParam(POSE_YAW_CMD_STRING, asynParamFloat64, &poseYawCmdIndex_);

    // TODO: can be set with shell environment variable
    spdlog::set_level(spdlog::level::debug);

    try_connect();

    epicsThreadCreate("RTDEControlPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEControl::poll() {
    while (true) {
        lock();

        if (rtde_control_ != nullptr) {
            if (rtde_control_->isConnected()) {
                setIntegerParam(isConnectedIndex_, 1);
                setIntegerParam(isSteadyIndex_, rtde_control_->isSteady());

                std::vector<double> jvec = rtde_receive_->getActualQ(); // rad
                for (double &j : jvec) {                                // convert to deg
                    j = j * 180.0 / M_PI;
                }
                doCallbacksFloat64Array(jvec.data(), NUM_JOINTS, actualQIndex_, 0);

                std::vector<double> pose_vec = rtde_receive_->getActualTCPPose();
                for (int i = 3; i < pose_vec.size(); i++) { // convert to deg
                    pose_vec.at(i) = pose_vec.at(i) * 180.0 / M_PI;
                }
                doCallbacksFloat64Array(pose_vec.data(), NUM_JOINTS, actualTCPPoseIndex_, 0);

            } else {
                setIntegerParam(isConnectedIndex_, 0);
            }
        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus RTDEControl::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    static std::map<int, int> JxCmd_map = {
        {j1CmdIndex_, 0}, {j2CmdIndex_, 1}, {j3CmdIndex_, 2},
        {j4CmdIndex_, 3}, {j5CmdIndex_, 4}, {j6CmdIndex_, 5},
    };

    static std::map<int, int> PoseCmd_map = {
        {poseXCmdIndex_, 0},    {poseYCmdIndex_, 1},     {poseZCmdIndex_, 2},
        {poseRollCmdIndex_, 3}, {posePitchCmdIndex_, 4}, {poseYawCmdIndex_, 5},
    };

    if (rtde_control_ == nullptr) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }

    if (not rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }
    
    // When commanded joint angles change, update the values
    if (JxCmd_map.count(function) > 0) {
        // convert commanded joint angles to radians
        const double val = value * M_PI / 180.0;
        cmd_joints.at(JxCmd_map[function]) = val;
    }

    // When commanded TCP pose values change, update the values
    else if (PoseCmd_map.count(function) > 0) {
        // note we just need to convert roll, pitch, yaw to degrees
        int ind = PoseCmd_map[function];
        const double val = (ind >= 3) ? (value * M_PI / 180.0) : value;
        cmd_pose.at(ind) = val;
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("RTDE communincation error in RTDEControl::writeFloat64");
        return asynError;
    }
}

asynStatus RTDEControl::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == reconnectIndex_) {
        comm_ok = try_connect();
        goto skip;
    }

    if (rtde_control_ == nullptr) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }
    if (function == disconnectIndex_) {
        spdlog::debug("Disconnecting from RTDE control interface");
        rtde_control_->disconnect();
        rtde_receive_->disconnect();
        comm_ok = not rtde_control_->isConnected() and not rtde_receive_->isConnected();
        goto skip;
    }

    // Check that it's connected before conntinuing
    if (not rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    // RTDE Control interface function calls go here
    if (function == moveJIndex_) {

        spdlog::info("moveJ({:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}) rad", cmd_joints.at(0),
                     cmd_joints.at(1), cmd_joints.at(2), cmd_joints.at(3), cmd_joints.at(4),
                     cmd_joints.at(5));

        bool safe = rtde_control_->isJointsWithinSafetyLimits(cmd_joints);
        if (safe) {
            spdlog::debug("moving joints but not actually");
            // rtde_control_->moveJ(cmd_joints);
        } else {
            spdlog::warn("Requested joint angles not within safety limits. No action taken.");
        }
    } else if (function == stopJIndex_) {
        spdlog::debug("stopping joint move, but not actually");
        // rtde_control_->stopJ();
    }

    else if (function == moveLIndex_) {

        spdlog::info("moveL({:.4f} mm, {:.4f} mm, {:.4f} mm, {:.4f} rad , {:.4f} rad, {:.4f} rad)",
                     cmd_pose.at(0), cmd_pose.at(1), cmd_pose.at(2), cmd_pose.at(3), cmd_pose.at(4),
                     cmd_pose.at(5));

        bool safe = rtde_control_->isPoseWithinSafetyLimits(cmd_pose);
        if (safe) {
            spdlog::debug("moving EE but not actually");
            // rtde_control_->moveL(cmd_pose);
        } else {
            spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
        }
    } else if (function == stopLIndex_) {
        spdlog::debug("stopping linear TCP pose move, but not actually");
        // rtde_control_->stopL();
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("RTDE communincation error in RTDEControl::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int RTDEControlConfig(const char *asyn_port_name, const char *robot_ip) {
    RTDEControl *pRTDEControl = new RTDEControl(asyn_port_name, robot_ip);
    pRTDEControl = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"RTDEControlConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) { RTDEControlConfig(args[0].sval, args[1].sval); }

void RTDEControlRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEControlRegister);
}
