#include <cstddef>
#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <fstream>
#include <iocsh.h>
#include <optional>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <type_traits>

#include "rtde_control_driver.hpp"
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
// TODO: ignore empty newlines
std::optional<std::vector<std::vector<double>>> read_traj_file(std::string filename) {
    std::vector<std::vector<double>> out;
    std::ifstream file(filename);
    if (not file.is_open()) {
        spdlog::error("Failed to open file: {}", filename);
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
    bool robot_running = false;
    try {
        auto dash = std::make_unique<ur_rtde::DashboardClient>(robot_ip_);
        dash->connect();
        if (dash->robotmode() == "Robotmode: RUNNING") {
            robot_running = true;
        } else {
            spdlog::error("Unable to connect to UR RTDE Control Interface: "
                          "Ensure robot is on, in normal mode, and brakes released");
            robot_running = false;
            connected = false;
            dash->disconnect();
        }
    } catch (const std::exception &e) {
        spdlog::error("Caught exception: {}", e.what());
        spdlog::error("RTDE Control: Failed to connect to dashboard to check robot mode");
    }

    if (robot_running) {
        if (rtde_control_ == nullptr) {
            try {
                rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
                if (rtde_control_ != nullptr) {
                    if (rtde_control_->isConnected()) {
                        spdlog::info("Connected to UR RTDE Control interface");
                        connected = true;

                        // connect to RTDE Receive and Robotiq Gripper,
                        // only if we successfully connect to RTDE Control
                        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
                        gripper_->connect();
                        if (not gripper_->isActive()) {
                            gripper_->activate();
                        }
                        if (rtde_receive_ == nullptr) {
                            // this probably isn't possible
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
      rtde_control_(nullptr), rtde_receive_(nullptr),
      gripper_(std::make_unique<ur_rtde::RobotiqGripper>(robot_ip)), robot_ip_(robot_ip) {

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
    createParam(PLAY_CARTESIAN_PATH_CSV_STRING, asynParamOctet, &playCartesianPathCsvIndex_);
    createParam(PLAY_JOINT_PATH_CSV_STRING, asynParamOctet, &playJointPathCsvIndex_);
    createParam(REUPLOAD_CTRL_SCRIPT_STRING, asynParamInt32, &reuploadCtrlScriptIndex_);
    createParam(STOP_CTRL_SCRIPT_STRING, asynParamInt32, &stopCtrlScriptIndex_);
    createParam(JOINT_SPEED_STRING, asynParamFloat64, &jointSpeedIndex_);
    createParam(JOINT_ACCEL_STRING, asynParamFloat64, &jointAccelIndex_);
    createParam(JOINT_BLEND_STRING, asynParamFloat64, &jointBlendIndex_);
    createParam(LINEAR_SPEED_STRING, asynParamFloat64, &linearSpeedIndex_);
    createParam(LINEAR_ACCEL_STRING, asynParamFloat64, &linearAccelIndex_);
    createParam(LINEAR_BLEND_STRING, asynParamFloat64, &linearBlendIndex_);
    createParam(ASYNC_MOVE_STRING, asynParamInt32, &asyncMoveIndex_);
    createParam(ASYNC_MOVE_DONE_STRING, asynParamInt32, &asyncMoveDoneIndex_);
    createParam(WAYPOINT_MOVEJ_STRING, asynParamInt32, &waypointMoveJIndex_);
    createParam(WAYPOINT_MOVEL_STRING, asynParamInt32, &waypointMoveLIndex_);
    createParam(WAYPOINT_GRIPPER_ACTION_STRING, asynParamInt32, &waypointGripperActionIndex_);

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

                // NOTE: This code supports moving to individual waypoints
                // as well as moving through a vector<vector<double>> when reading a CSV file
                if (async_running_ != AsyncRunning::False) {
                    if (async_status_ == AsyncMotionStatus::Done) {
                        if (waypoint_path_iter_ != waypoint_path_.end()) {

                            // HACK: not needed if joint path is length 1
                            std::vector<double> wp = *waypoint_path_iter_;
                            std::stringstream ss;
                            ss << "Moving to waypoint: ";
                            for (const auto &i : wp) {
                                ss << i << ", ";
                            }
                            spdlog::debug("{}", ss.str());

                            std::vector<double> waypoint(wp.begin(), wp.end() - 1); // last is gripper
                            gripper_action_ = wp.back();

                            if (async_running_ == AsyncRunning::Joint) {
                                if (rtde_control_->isJointsWithinSafetyLimits(
                                        {waypoint.begin(), waypoint.end() - 3})) {
                                    rtde_control_->moveJ(std::vector<std::vector<double>>{waypoint}, true);
                                    async_status_ = AsyncMotionStatus::WaitingMotion;
                                    setIntegerParam(asyncMoveDoneIndex_, 0);
                                } else {
                                    spdlog::warn(
                                        "Requested joint angles not within safety limits. No action taken.");
                                    async_status_ = AsyncMotionStatus::Done;
                                    async_running_ = AsyncRunning::False;
                                }
                            }
                            else if (async_running_ == AsyncRunning::Cartesian) {
                                if (rtde_control_->isPoseWithinSafetyLimits(
                                        {waypoint.begin(), waypoint.end() - 3})) {
                                    rtde_control_->moveL(std::vector<std::vector<double>>{waypoint}, true);
                                    async_status_ = AsyncMotionStatus::WaitingMotion;
                                    setIntegerParam(asyncMoveDoneIndex_, 0);
                                } else {
                                    spdlog::warn(
                                        "Requested TCP pose not within safety limits. No action taken.");
                                    async_status_ = AsyncMotionStatus::Done;
                                    async_running_ = AsyncRunning::False;
                                }
                            }
                        } else {
                            spdlog::debug("Asynchronous move done");
                            async_running_ = AsyncRunning::False;
                            setIntegerParam(asyncMoveDoneIndex_, 1);
                        }
                    } else {
                        if (async_status_ == AsyncMotionStatus::WaitingMotion) {
                            auto op_status = rtde_control_->getAsyncOperationProgressEx();
                            if (not op_status.isAsyncOperationRunning()) {
                                spdlog::debug("Waypoint reached");
                                if (gripper_action_ == 0) {
                                    gripper_->open();
                                } else {
                                    gripper_->close();
                                }
                                async_status_ = AsyncMotionStatus::WaitingGripper;
                            }
                        } else if (async_status_ == AsyncMotionStatus::WaitingGripper) {
                            if (gripper_->objectDetectionStatus() !=
                                ur_rtde::RobotiqGripper::eObjectStatus::MOVING) {
                                async_status_ = AsyncMotionStatus::Done;
                                waypoint_path_iter_ = std::next(waypoint_path_iter_);
                            }
                        }
                    }
                }
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
        this->cmd_joints.at(JxCmd_map.at(function)) = val;
    }

    // When commanded TCP pose values change, update the values
    else if (PoseCmd_map.count(function) > 0) {
        // convert commanded roll, pitch, yaw to radians
        int ind = PoseCmd_map.at(function);
        const double val = (ind >= 3) ? (value * M_PI / 180.0) : value;
        this->cmd_pose.at(ind) = val;
    }

    // Dynamics for joint moves (moveJ)
    else if (function == jointSpeedIndex_) {
        this->joint_speed_ = value;
        spdlog::debug("Setting joint speed to {}", joint_speed_);
    } else if (function == jointAccelIndex_) {
        this->joint_accel_ = value;
        spdlog::debug("Setting joint acceleration to {}", joint_accel_);
    } else if (function == jointBlendIndex_) {
        this->joint_blend_ = value;
        spdlog::debug("Setting joint blend to {}", joint_blend_);
    }

    // Dynamics for linear moves (moveL)
    else if (function == linearSpeedIndex_) {
        this->linear_speed_ = value;
        spdlog::debug("Setting linear speed to {}", linear_speed_);
    } else if (function == linearAccelIndex_) {
        this->linear_accel_ = value;
        spdlog::debug("Setting linear acceleration to {}", linear_accel_);
    } else if (function == linearBlendIndex_) {
        this->linear_blend_ = value;
        spdlog::debug("Setting linear blend to {}", linear_blend_);
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
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
            rtde_control_->moveJ(cmd_joints, joint_speed_, joint_accel_, async_move);
        } else {
            spdlog::warn("Requested joint angles not within safety limits. No action taken.");
        }
    } else if (function == stopJIndex_) {
        spdlog::debug("Stopping joint move (only works in asynchronous mode)");
        rtde_control_->stopJ(); // asynchronous=true
        // async_running_ = AsyncRunning::False;
        // async_status_ = AsyncMotionStatus::Done;
    }

    else if (function == moveLIndex_) {

        spdlog::info("moveL({:.4f} mm, {:.4f} mm, {:.4f} mm, {:.4f} rad , {:.4f} rad, {:.4f} rad)",
                     cmd_pose.at(0), cmd_pose.at(1), cmd_pose.at(2), cmd_pose.at(3), cmd_pose.at(4),
                     cmd_pose.at(5));

        bool safe = rtde_control_->isPoseWithinSafetyLimits(cmd_pose);
        if (safe) {
            rtde_control_->moveL(cmd_pose, linear_speed_, linear_accel_, async_move);
        } else {
            spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
        }
    } else if (function == stopLIndex_) {
        spdlog::debug("Stopping linear TCP move (only works in asynchronous mode)");
        rtde_control_->stopL();
    }

    else if (function == waypointMoveJIndex_) {
        if (async_running_ == AsyncRunning::False) {
            this->waypoint_path_.clear();
            std::vector<double> wp = this->cmd_joints;
            wp.push_back(this->joint_speed_);
            wp.push_back(this->joint_accel_);
            wp.push_back(this->joint_blend_);
            wp.push_back(this->gripper_action_);
            this->waypoint_path_ = {wp};
            this->waypoint_path_iter_ = this->waypoint_path_.begin();
            this->async_running_ = AsyncRunning::Joint;
        } else {
            spdlog::warn("Asynchronous motion in progress...please wait");
        }
    } else if (function == waypointMoveLIndex_) {
        if (async_running_ == AsyncRunning::False) {
            this->waypoint_path_.clear();
            std::vector<double> wp = this->cmd_pose;
            wp.push_back(this->linear_speed_);
            wp.push_back(this->linear_accel_);
            wp.push_back(this->linear_blend_);
            wp.push_back(this->gripper_action_);
            this->waypoint_path_ = {wp};
            this->waypoint_path_iter_ = this->waypoint_path_.begin();
            this->async_running_ = AsyncRunning::Cartesian;
        } else {
            spdlog::warn("Asynchronous motion in progress...please wait");
        }
    }

    else if (function == waypointGripperActionIndex_) {
        this->gripper_action_ = value;
        spdlog::debug("Setting gripper action to {}", gripper_action_);
    }

    else if (function == reuploadCtrlScriptIndex_) {
        spdlog::debug("Reuploading control script");
        rtde_control_->reuploadScript();
    }

    else if (function == stopCtrlScriptIndex_) {
        spdlog::debug("Stopping control script");
        // this->async_running_ = AsyncRunning::False;
        // this->async_status_ = AsyncMotionStatus::Done;
        // this->waypoint_path_.clear();
        rtde_control_->stopScript();
    }

    else if (function == asyncMoveIndex_) {
        async_move = value;
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

    if (function == playCartesianPathCsvIndex_) {
        this->waypoint_path_.clear();

        std::optional<std::vector<std::vector<double>>> pose_path = read_traj_file(value);

        if (pose_path.has_value()) {
            for (auto &wp : pose_path.value()) {
                for (int i = 3; i < 6; i++) {
                    wp.at(i) *= M_PI / 180.0; // convert angles to rad
                }
            }

            for (const auto &wp : pose_path.value()) {
                for (const auto &i : wp) {
                    std::cout << i << ", ";
                }
                std::cout << std::endl;
            }
            this->waypoint_path_ = pose_path.value();
            this->waypoint_path_iter_ = this->waypoint_path_.begin();
            async_running_ = AsyncRunning::Cartesian;
        } else {
            comm_ok = false;
            goto skip;
        }
    }

    else if (function == playJointPathCsvIndex_) {
        this->waypoint_path_.clear();

        std::optional<std::vector<std::vector<double>>> joint_path = read_traj_file(value);

        if (joint_path.has_value()) {
            for (auto &wp : joint_path.value()) {
                for (int i = 0; i < 6; i++) {
                    wp.at(i) *= M_PI / 180.0; // convert angles to rad
                }
            }

            for (const auto &wp : joint_path.value()) {
                for (const auto &i : wp) {
                    std::cout << i << ", ";
                }
                std::cout << std::endl;
            }
            this->waypoint_path_ = joint_path.value();
            this->waypoint_path_iter_ = this->waypoint_path_.begin();
            async_running_ = AsyncRunning::Joint;
        } else {
            comm_ok = false;
            goto skip;
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
