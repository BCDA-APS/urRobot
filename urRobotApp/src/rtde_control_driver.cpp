#include <cstddef>
#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <fstream>
#include <iocsh.h>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#include "rtde_control_driver.hpp"
#include "rtde_control_interface.h"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include "ur_rtde/dashboard_client.h"

// splits a string by a delimiter into a vector<T> and returns it as
// a std::optional(vector<T>). If conversion fails, std::nullopt is returned
template <typename T>
std::optional<std::vector<T>> split_string(const std::string &msg, const char delimiter) {
    std::vector<T> out;
    std::string token;
    std::istringstream token_stream(msg);
    static_assert(std::is_same_v<T, double> or std::is_same_v<T, int> or std::is_same_v<T, std::string>);

    while (std::getline(token_stream, token, delimiter)) {
        try {
            if constexpr (std::is_same_v<T, double>) {
                const double value = std::stod(token);
                out.push_back(value);
            } else if constexpr (std::is_same_v<T, int>) {
                const int value = std::stoi(token);
                out.push_back(value);
            } else if constexpr (std::is_same_v<T, std::string>) {
                out.push_back(token);
            }
        } catch (const std::exception &e) {
            spdlog::error("Failed to split string. Caugh exception: {}", e.what());
            return std::nullopt;
        }
    }
    return std::optional(out);
}

// Reads a CSV file, splitting each row into a vector<double> and returns
// an optional<vector<vector<double>>>. If the file cannot be opened, or
// splitting/conversion fails, std::nullopt is returned
std::optional<std::vector<std::vector<double>>> read_traj_file(std::string filename) {
    std::vector<std::vector<double>> out;
    std::ifstream file(filename);
    if (not file.is_open()) {
        spdlog::error("Failed to open file {}", filename);
        return std::nullopt;
    } else {
        std::string line;
        while (std::getline(file, line)) {
            std::optional<std::vector<double>> v = split_string<double>(line, ',');
            if (v.has_value()) {
                out.push_back(v.value());
            } else {
                return std::nullopt;
            }
        }
        return std::optional(out);
    }
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
                spdlog::error("Unable to connect to UR RTDE Control Interface:"
                              "Ensure robot is on, in normal mode, and brakes released");
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
                        // NOTE: unlikely this could ever happen
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

static void poll_thread_C(void *pPvt) {
    RTDEControl *pRTDEControl = (RTDEControl *)pPvt;
    pRTDEControl->poll();
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

    createParam(PLAY_POSE_PATH_STRING, asynParamOctet, &playPosePathIndex_);
    createParam(PLAY_JOINT_PATH_STRING, asynParamOctet, &playJointPathIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

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
                for (size_t i = 3; i < pose_vec.size(); i++) { // convert to deg
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
        // cmd_joints.at(JxCmd_map[function]) = val;
        cmd_joints.at(JxCmd_map.at(function)) = val;
    }

    // When commanded TCP pose values change, update the values
    else if (PoseCmd_map.count(function) > 0) {
        // note we just need to convert roll, pitch, yaw to degrees
        // int ind = PoseCmd_map[function];
        int ind = PoseCmd_map.at(function);
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

    if (function == moveJIndex_) {

        spdlog::info("moveJ({:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}) rad", cmd_joints.at(0),
                     cmd_joints.at(1), cmd_joints.at(2), cmd_joints.at(3), cmd_joints.at(4),
                     cmd_joints.at(5));

        bool safe = rtde_control_->isJointsWithinSafetyLimits(cmd_joints);
        if (safe) {
            rtde_control_->moveJ(cmd_joints);
        } else {
            spdlog::warn("Requested joint angles not within safety limits. No action taken.");
        }
    } else if (function == stopJIndex_) {
        spdlog::debug("Stopping joint move (only works in asynchronous mode)");
        rtde_control_->stopJ();
    }

    else if (function == moveLIndex_) {

        spdlog::info("moveL({:.4f} mm, {:.4f} mm, {:.4f} mm, {:.4f} rad , {:.4f} rad, {:.4f} rad)",
                     cmd_pose.at(0), cmd_pose.at(1), cmd_pose.at(2), cmd_pose.at(3), cmd_pose.at(4),
                     cmd_pose.at(5));

        bool safe = rtde_control_->isPoseWithinSafetyLimits(cmd_pose);
        if (safe) {
            rtde_control_->moveL(cmd_pose);
        } else {
            spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
        }
    } else if (function == stopLIndex_) {
        spdlog::debug("Stopping linear TCP move (only works in asynchronous mode)");
        rtde_control_->stopL();
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

asynStatus RTDEControl::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual) {
    int function = pasynUser->reason;
    bool comm_ok = true;

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

    if (function == playPosePathIndex_) {
        auto pose_path = read_traj_file(value);
        if (pose_path.has_value()) {
            for (const auto &p : pose_path.value()) {
                spdlog::info("moveL({:.4f} mm, {:.4f} mm, {:.4f} mm, {:.4f} rad , {:.4f} rad, {:.4f} rad)",
                             p.at(0), p.at(1), p.at(2), p.at(3), p.at(4), p.at(5));
            }
        }
    }

    else if (function == playJointPathIndex_) {
        auto joint_path = read_traj_file(value);
        if (joint_path.has_value()) {
            for (const auto &p : joint_path.value()) {
                spdlog::info("moveJ({:.4f} rad, {:.4f} rad, {:.4f} rad, {:.4f} rad , {:.4f} rad, {:.4f} rad)",
                             p.at(0), p.at(1), p.at(2), p.at(3), p.at(4), p.at(5));
            }
        }
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
