#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>
#include <optional>
#include <sstream>
#include <stdexcept>

#include "rtde_control_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/fmt/ranges.h"
#include "spdlog/spdlog.h"
#include "ur_rtde/dashboard_client.h"

using OptTrajectory = std::optional<std::vector<std::vector<double>>>;
OptTrajectory read_traj_file(const std::string& filepath);

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
    } catch (const std::exception& e) {
        spdlog::error("Caught exception: {}", e.what());
        spdlog::error("RTDE Control: Failed to connect to dashboard to check robot mode");
    }

    if (robot_running) {
        if (!rtde_control_) {
            try {
                rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
                if (rtde_control_) {
                    if (rtde_control_->isConnected()) {
                        spdlog::info("Connected to UR RTDE Control interface");
                        connected = true;

                        // connect to RTDE Receive, only if already connected to RTDE Control
                        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
                        if (!rtde_receive_) {
                            // this probably isn't possible
                            throw std::runtime_error(
                                "Failed connecting to receive interface from RTDEControl class");
                        }
                    }
                }
            } catch (const std::exception& e) {
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

static void poll_thread_C(void* pPvt) {
    RTDEControl* pRTDEControl = (RTDEControl*)pPvt;
    pRTDEControl->poll();
}

constexpr int NUM_JOINTS = 6;
constexpr int MAX_ADDR = NUM_JOINTS;
constexpr int ASYN_INTERFACE_MASK =
    asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask | asynDrvUserMask;
constexpr int ASYN_INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask;

RTDEControl::RTDEControl(const char* asyn_port_name, const char* robot_ip, double poll_period)
    : asynPortDriver(asyn_port_name, MAX_ADDR, ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0),
      rtde_control_(nullptr), rtde_receive_(nullptr), robot_ip_(robot_ip), poll_period_(poll_period) {

    createParam("DISCONNECT", asynParamInt32, &disconnectIndex_);
    createParam("RECONNECT", asynParamInt32, &reconnectIndex_);
    createParam("IS_CONNECTED", asynParamInt32, &isConnectedIndex_);
    createParam("IS_STEADY", asynParamInt32, &isSteadyIndex_);
    createParam("MOVEJ", asynParamInt32, &moveJIndex_);
    createParam("STOPJ", asynParamInt32, &stopJIndex_);
    createParam("ACTUAL_Q", asynParamFloat64Array, &actualQIndex_);
    createParam("JOINT_CMD", asynParamFloat64, &jointCmdIndex_);
    createParam("MOVEL", asynParamInt32, &moveLIndex_);
    createParam("STOPL", asynParamInt32, &stopLIndex_);
    createParam("ACTUAL_TCP_POSE", asynParamFloat64Array, &actualTCPPoseIndex_);
    createParam("POSE_CMD", asynParamFloat64, &poseCmdIndex_);
    createParam("TCP_OFFSET", asynParamFloat64, &tcpOffsetIndex_);
    createParam("REUPLOAD_CONTROL_SCRIPT", asynParamInt32, &reuploadCtrlScriptIndex_);
    createParam("STOP_CONTROL_SCRIPT", asynParamInt32, &stopCtrlScriptIndex_);
    createParam("JOINT_SPEED", asynParamFloat64, &jointSpeedIndex_);
    createParam("JOINT_ACCELERATION", asynParamFloat64, &jointAccelIndex_);
    createParam("JOINT_BLEND", asynParamFloat64, &jointBlendIndex_);
    createParam("LINEAR_SPEED", asynParamFloat64, &linearSpeedIndex_);
    createParam("LINEAR_ACCELERATION", asynParamFloat64, &linearAccelIndex_);
    createParam("LINEAR_BLEND", asynParamFloat64, &linearBlendIndex_);
    createParam("ASYNC_MOVE_DONE", asynParamInt32, &asyncMoveDoneIndex_);
    createParam("WAYPOINT_MOVEJ", asynParamInt32, &waypointMoveJIndex_);
    createParam("WAYPOINT_MOVEL", asynParamInt32, &waypointMoveLIndex_);
    createParam("RUN_WAYPOINT_ACTION", asynParamInt32, &runWaypointActionIndex_);
    createParam("WAYPOINT_ACTION_DONE", asynParamInt32, &waypointActionDoneIndex_);
    createParam("TEACH_MODE", asynParamInt32, &teachModeIndex_);
    createParam("TRIGGER_PROT_STOP", asynParamInt32, &triggerProtStopIndex_);
    createParam("TRAJ_FILE", asynParamOctet, &trajFileIndex_);
    createParam("TRAJ_TYPE", asynParamInt32, &trajTypeIndex_);
    createParam("TRAJ_MOVE", asynParamInt32, &trajMoveIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    // tries connecting to the control and receive servers on the robot controller
    try_connect();

    epicsThreadCreate("RTDEControlPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEControl::poll() {
    int run_action_val = 0;

    while (true) {
        lock();

        if (rtde_control_ and rtde_control_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);
            setIntegerParam(isSteadyIndex_, rtde_control_->isSteady());

            // Get safety bits so we can abort motion in safety event
            if (rtde_receive_->getSafetyStatusBits() != 1) {
                if (pending_motion_) {
                    spdlog::debug("Motion stopped due to safety.");
                    set_motion_done();
                }
                continue;
            }

            if (pending_motion_) {
                if (motion_status_ == AsyncMotionStatus::Done) {
                    // starting new asynchronous motion
                    if (pending_motion_->type == MotionType::Joint) {
                        rtde_control_->moveJ(cmd_joints_, joint_speed_, joint_accel_, true);
                    } else if (pending_motion_->type == MotionType::Cartesian) {
                        rtde_control_->moveL(cmd_pose_, linear_speed_, linear_accel_, true);
                    }
                    set_motion_start();
                } else { // async motion task in progress
                    if (motion_status_ == AsyncMotionStatus::WaitingMotion) {
                        auto op_status = rtde_control_->getAsyncOperationProgressEx();
                        if (!op_status.isAsyncOperationRunning()) {
                            if (pending_motion_->action) {
                                spdlog::debug("Waypoint reached. Starting action...");
                                run_action_val = 1 ^ run_action_val; // ensures action PV processes
                                setIntegerParam(waypointActionDoneIndex_, 0);
                                setIntegerParam(runWaypointActionIndex_, run_action_val);
                                motion_status_ = AsyncMotionStatus::WaitingAction;
                            } else {
                                spdlog::debug("Motion complete.");
                                set_motion_done();
                            }
                        }
                    } else if (motion_status_ == AsyncMotionStatus::WaitingAction) {
                        int done = 0;
                        getIntegerParam(waypointActionDoneIndex_, &done);
                        if (done) {
                            spdlog::debug("Waypoint action complete.");
                            set_motion_done();
                        }
                    }
                }
            }

        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_period_);
    }
}

asynStatus RTDEControl::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    int addr = 0;
    getAddress(pasynUser, &addr);

    if (!rtde_control_) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }

    if (not rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    if (function == jointCmdIndex_) {
        // convert commanded joint angles to radians
        const double val = value * M_PI / 180.0;
        this->cmd_joints_.at(addr) = val;
    }

    else if (function == poseCmdIndex_) {
        // convert commanded x,y,z to meters and roll, pitch, yaw to radians
        const double val = (addr >= 3) ? (value * M_PI / 180.0) : (value / 1000.0);
        this->cmd_pose_.at(addr) = val;
    }

    else if (function == tcpOffsetIndex_) {
        // convert commanded x,y,z from mm to meters. Assume roll, pitch, yaw is radians
        const double val = (addr >= 3) ? value : (value / 1000.0);
        this->tcp_offset_.at(addr) = val;
        spdlog::debug("Setting TCP offset to [{:.4f}] m,rad", fmt::join(tcp_offset_, ","));
        rtde_control_->setTcp(this->tcp_offset_);
    }

    // Dynamics for joint moves (moveJ)
    // convert from deg -> rad
    else if (function == jointSpeedIndex_) {
        this->joint_speed_ = value * M_PI / 180.0;
        spdlog::debug("Setting joint speed to {}", joint_speed_);
    } else if (function == jointAccelIndex_) {
        this->joint_accel_ = value * M_PI / 180.0;
        spdlog::debug("Setting joint acceleration to {}", joint_accel_);
    } else if (function == jointBlendIndex_) {
        this->joint_blend_ = value / 1000.0;
        spdlog::debug("Setting joint blend to {}", joint_blend_);
    }

    // Dynamics for linear moves (moveL)
    // convert from m -> mm
    else if (function == linearSpeedIndex_) {
        this->linear_speed_ = value / 1000.0;
        spdlog::debug("Setting linear speed to {}", linear_speed_);
    } else if (function == linearAccelIndex_) {
        this->linear_accel_ = value / 1000.0;
        spdlog::debug("Setting linear acceleration to {}", linear_accel_);
    } else if (function == linearBlendIndex_) {
        this->linear_blend_ = value / 1000.0;
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

asynStatus RTDEControl::writeInt32(asynUser* pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == reconnectIndex_) {
        comm_ok = try_connect();
        goto skip;
    }

    if (!rtde_control_) {
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
    if (!rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    if (function == moveJIndex_ || function == waypointMoveJIndex_) {
        if (!pending_motion_) {
            spdlog::debug("moveJ({:.4f}) rad", fmt::join(cmd_joints_, ","));
            if (rtde_control_->isJointsWithinSafetyLimits(cmd_joints_)) {
                const bool do_action = function == waypointMoveJIndex_;
                pending_motion_ = MotionTask{MotionType::Joint, do_action};
            } else {
                spdlog::warn("Requested joint angles not within safety limits. No action taken.");
            }
        } else {
            spdlog::warn("Motion already in progress...please wait");
        }
    }

    else if (function == moveLIndex_ || function == waypointMoveLIndex_) {
        if (!pending_motion_) {
            spdlog::debug("moveL({:.4f}) m,rad", fmt::join(cmd_pose_, ","));
            if (rtde_control_->isPoseWithinSafetyLimits(cmd_pose_)) {
                const bool do_action = function == waypointMoveLIndex_;
                pending_motion_ = MotionTask{MotionType::Cartesian, do_action};
            } else {
                spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
            }
        } else {
            spdlog::warn("Motion already in progress...please wait");
        }
    }

    else if (function == stopJIndex_) {
        set_motion_done();
        spdlog::debug("Stopping (linear in joint space)");
        rtde_control_->stopJ();
    }

    else if (function == stopLIndex_) {
        set_motion_done();
        spdlog::debug("Stopping (linear in tool space)");
        rtde_control_->stopL();
    }

    else if (function == trajTypeIndex_) {
        spdlog::debug("Setting trajectory to type {}", value ? "Cartesian" : "Joint");
        traj_type_ = static_cast<MotionType>(value);
    }

    else if (function == waypointActionDoneIndex_) {
        setIntegerParam(waypointActionDoneIndex_, value);
    }

    else if (function == reuploadCtrlScriptIndex_) {
        spdlog::debug("Reuploading control script");
        rtde_control_->reuploadScript();
    }

    else if (function == stopCtrlScriptIndex_) {
        spdlog::debug("Stopping control script");
        rtde_control_->stopScript();
    }

    else if (function == triggerProtStopIndex_) {
        spdlog::debug("Triggering protective stop");
        rtde_control_->triggerProtectiveStop();
    }

    else if (function == teachModeIndex_) {
        if (value) {
            spdlog::debug("Enabling teach mode");
            rtde_control_->teachMode();
        } else {
            spdlog::debug("Disabling teach mode");
            rtde_control_->endTeachMode();
        }
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("RTDE communication error in RTDEControl::writeInt32");
        return asynError;
    }
}

asynStatus RTDEControl::writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual) {
    int function = pasynUser->reason;
    bool comm_ok = true;

    if (!rtde_control_) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }

    if (not rtde_control_->isConnected()) {
        spdlog::error("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    if (function == trajFileIndex_) {
        spdlog::debug("Setting trajectory file: {}", value);
        traj_file_path_ = value;
    }

skip:
    *nActual = strlen(value);
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("RTDE communication error in RTDEControl::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int RTDEControlConfig(const char* asyn_port_name, const char* robot_ip, double poll_period) {
    new RTDEControl(asyn_port_name, robot_ip, poll_period);
    return asynSuccess;
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg urRobotArg2 = {"Poll period", iocshArgDouble};
static const iocshArg* const urRobotArgs[3] = {&urRobotArg0, &urRobotArg1, &urRobotArg2};
static const iocshFuncDef urRobotFuncDef = {"RTDEControlConfig", 3, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf* args) {
    RTDEControlConfig(args[0].sval, args[1].sval, args[2].dval);
}

void RTDEControlRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEControlRegister);
}

OptTrajectory read_traj_file(const std::string& filepath) {

    // This is fast enough until number of lines in csv file become
    // very large. A rough test showed it took ~5ms to parse a csv with 1000 lines.
    // A csv with 25,000 line takes ~100ms to parse.

    constexpr char COMMENT_CHAR = '#';
    constexpr size_t TRAJ_ROW_SIZE = 9; // 6 positions, speed, accel, blend

    std::ifstream fs(filepath);
    if (!fs.is_open()) {
        return std::nullopt;
    }

    std::string line;
    std::vector<std::vector<double>> out;

    while (std::getline(fs, line)) {
        if (line.size() > 0) {
            if (line[0] != COMMENT_CHAR) {

                // trim whitespace
                auto i0 = line.find_first_not_of(" \t");
                if (i0 == std::string::npos)
                    continue; // skip empty lines
                auto i1 = line.find_last_not_of(' ');
                auto line_trim = line.substr(i0, i1 - i0 + 1);

                // parse the line
                std::stringstream ss(line_trim);
                std::string token;
                std::vector<double> row;
                while (std::getline(ss, token, ',')) {
                    try {
                        row.push_back(std::stod(token));
                    } catch (...) {
                        return std::nullopt;
                    }
                }
                if (row.size() != TRAJ_ROW_SIZE) {
                    std::cerr << "Invalid .csv row length\n";
                    return std::nullopt;
                }
                out.push_back(row);
            }
        }
    }

    return out;
}
