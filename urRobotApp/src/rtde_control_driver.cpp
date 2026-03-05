#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>
#include <sstream>
#include <stdexcept>

#include "rtde_control_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include "ur_rtde/dashboard_client.h"

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

                        // connect to RTDE Receive, only if already connected to RTDE Control
                        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
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

constexpr int NUM_JOINTS = 6;
constexpr int MAX_ADDR = NUM_JOINTS;
constexpr int ASYN_INTERFACE_MASK =
    asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask | asynDrvUserMask;
constexpr int ASYN_INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask;

RTDEControl::RTDEControl(const char *asyn_port_name, const char *robot_ip, double poll_period)
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

        if (rtde_control_) {
            if (rtde_control_->isConnected()) {
                setIntegerParam(isConnectedIndex_, 1);
                setIntegerParam(isSteadyIndex_, rtde_control_->isSteady());

                std::vector<double> jvec = rtde_receive_->getActualQ();
                for (double &j : jvec) {
                    j = j * 180.0 / M_PI; // convert rad -> deg
                }
                doCallbacksFloat64Array(jvec.data(), NUM_JOINTS, actualQIndex_, 0);

                std::vector<double> pose_vec = rtde_receive_->getActualTCPPose();
                for (size_t i = 0; i < pose_vec.size(); i++) {
                    if (i <= 2) {
                        pose_vec.at(i) = pose_vec.at(i) * 1000.0; // convert m -> mm
                    } else {
                        pose_vec.at(i) = pose_vec.at(i) * 180.0 / M_PI; // convert rad -> deg
                    }
                }
                doCallbacksFloat64Array(pose_vec.data(), NUM_JOINTS, actualTCPPoseIndex_, 0);

                uint32_t safety_bits = rtde_receive_->getSafetyStatusBits();

                if (async_running_ != AsyncRunning::False) {
                    if (async_status_ == AsyncMotionStatus::Done) {
                        std::stringstream ss;
                        ss << "Moving to waypoint: ";
                        for (const auto &i : waypoint_) {
                            ss << i << ", ";
                        }
                        spdlog::debug("{}", ss.str());

                        if (async_running_ == AsyncRunning::Joint) {
                            if (rtde_control_->isJointsWithinSafetyLimits(
                                    {waypoint_.begin(), waypoint_.end() - 3})) {
                                rtde_control_->moveJ(std::vector<std::vector<double>>{waypoint_}, true);
                                async_status_ = AsyncMotionStatus::WaitingMotion;
                                setIntegerParam(asyncMoveDoneIndex_, 0);
                            } else {
                                spdlog::warn(
                                    "Requested joint angles not within safety limits. No action taken.");
                                async_status_ = AsyncMotionStatus::Done;
                                async_running_ = AsyncRunning::False;
                            }
                        } else if (async_running_ == AsyncRunning::Cartesian) {
                            if (rtde_control_->isPoseWithinSafetyLimits(
                                    {waypoint_.begin(), waypoint_.end() - 3})) {
                                rtde_control_->moveL(std::vector<std::vector<double>>{waypoint_}, true);
                                async_status_ = AsyncMotionStatus::WaitingMotion;
                                setIntegerParam(asyncMoveDoneIndex_, 0);
                            } else {
                                spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
                                async_status_ = AsyncMotionStatus::Done;
                                async_running_ = AsyncRunning::False;
                            }
                        }
                    } else {
                        if (async_status_ == AsyncMotionStatus::WaitingMotion) {
                            auto op_status = rtde_control_->getAsyncOperationProgressEx();
                            if (not op_status.isAsyncOperationRunning()) {
                                if (safety_bits != 1) {
                                    async_status_ = AsyncMotionStatus::Done;
                                    async_running_ = AsyncRunning::False;
                                    setIntegerParam(asyncMoveDoneIndex_, 1);
                                } else {
                                    spdlog::debug("Waypoint reached. Starting action...");
                                    run_action_val =
                                        1 ^ run_action_val; // ensures the action PV link is processed
                                    setIntegerParam(waypointActionDoneIndex_, 0);
                                    setIntegerParam(runWaypointActionIndex_, run_action_val);
                                    async_status_ = AsyncMotionStatus::WaitingAction;
                                }
                            }
                        } else if (async_status_ == AsyncMotionStatus::WaitingAction) {
                            int done = 0;
                            getIntegerParam(waypointActionDoneIndex_, &done);
                            if (done) {
                                spdlog::debug("Action done!");
                                async_status_ = AsyncMotionStatus::Done;
                                spdlog::debug("Asynchronous move done");
                                async_running_ = AsyncRunning::False;
                                setIntegerParam(asyncMoveDoneIndex_, 1);
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
        epicsThreadSleep(poll_period_);
    }
}

asynStatus RTDEControl::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    int addr = 0;
    getAddress(pasynUser, &addr);

    // static std::map<int, int> tcp_offset_map = {
    // {tcpOffsetXIndex_, 0},    {tcpOffsetYIndex_, 1},     {tcpOffsetZIndex_, 2},
    // {tcpOffsetRollIndex_, 3}, {tcpOffsetPitchIndex_, 4}, {tcpOffsetYawIndex_, 5},
    // };

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
        std::stringstream ss;
        ss << "Setting TCP offset to ";
        for (const auto &i : tcp_offset_) {
            ss << i << ", ";
        }
        spdlog::debug("{}", ss.str());
        rtde_control_->setTcp(this->tcp_offset_);
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

        spdlog::info("moveJ({:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}, {:.4f}) rad", cmd_joints_.at(0),
                     cmd_joints_.at(1), cmd_joints_.at(2), cmd_joints_.at(3), cmd_joints_.at(4),
                     cmd_joints_.at(5));

        bool safe = rtde_control_->isJointsWithinSafetyLimits(cmd_joints_);
        if (safe) {
            rtde_control_->moveJ(cmd_joints_, joint_speed_, joint_accel_, true);
        } else {
            spdlog::warn("Requested joint angles not within safety limits. No action taken.");
        }
    } else if (function == stopJIndex_) {
        this->async_running_ = AsyncRunning::False;
        this->async_status_ = AsyncMotionStatus::Done;
        spdlog::debug("Stopping joint move");
        setIntegerParam(asyncMoveDoneIndex_, 1);
        rtde_control_->stopJ(); // asynchronous=true
    }

    else if (function == moveLIndex_) {

        spdlog::info("moveL({:.4f} m, {:.4f} m, {:.4f} m, {:.4f} rad , {:.4f} rad, {:.4f} rad)",
                     cmd_pose_.at(0), cmd_pose_.at(1), cmd_pose_.at(2), cmd_pose_.at(3), cmd_pose_.at(4),
                     cmd_pose_.at(5));

        bool safe = rtde_control_->isPoseWithinSafetyLimits(cmd_pose_);
        if (safe) {
            rtde_control_->moveL(cmd_pose_, linear_speed_, linear_accel_, true);
        } else {
            spdlog::warn("Requested TCP pose not within safety limits. No action taken.");
        }
    } else if (function == stopLIndex_) {
        this->async_running_ = AsyncRunning::False;
        this->async_status_ = AsyncMotionStatus::Done;
        setIntegerParam(asyncMoveDoneIndex_, 1);
        spdlog::debug("Stopping linear move");
        rtde_control_->stopL();
    }

    else if (function == waypointMoveJIndex_) {
        if (async_running_ == AsyncRunning::False) {
            this->waypoint_.clear();
            std::vector<double> wp = this->cmd_joints_;
            wp.push_back(this->joint_speed_);
            wp.push_back(this->joint_accel_);
            wp.push_back(this->joint_blend_);
            this->waypoint_ = wp;
            this->async_running_ = AsyncRunning::Joint;
        } else {
            spdlog::warn("Asynchronous motion in progress...please wait");
        }
    } else if (function == waypointMoveLIndex_) {
        if (async_running_ == AsyncRunning::False) {
            this->waypoint_.clear();
            std::vector<double> wp = this->cmd_pose_;
            wp.push_back(this->linear_speed_);
            wp.push_back(this->linear_accel_);
            wp.push_back(this->linear_blend_);
            this->waypoint_ = wp;
            this->async_running_ = AsyncRunning::Cartesian;
        } else {
            spdlog::warn("Asynchronous motion in progress...please wait");
        }
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
        spdlog::debug("RTDE communincation error in RTDEControl::writeInt32");
        return asynError;
    }
}

asynStatus RTDEControl::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual) {
    // int function = pasynUser->reason;
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
extern "C" int RTDEControlConfig(const char *asyn_port_name, const char *robot_ip, double poll_period) {
    RTDEControl *pRTDEControl = new RTDEControl(asyn_port_name, robot_ip, poll_period);
    (void)pRTDEControl;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg urRobotArg2 = {"Poll period", iocshArgDouble};
static const iocshArg *const urRobotArgs[3] = {&urRobotArg0, &urRobotArg1, &urRobotArg2};
static const iocshFuncDef urRobotFuncDef = {"RTDEControlConfig", 3, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    RTDEControlConfig(args[0].sval, args[1].sval, args[2].dval);
}

void RTDEControlRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEControlRegister);
}
