#include <cstddef>
#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>
#include <stdexcept>

#include "rtde_control_driver.hpp"
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
                spdlog::error("Unable to connect to UR RTDE Control Interface:\nEnsure robot is on "
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
            spdlog::info("Reconnecting to UR RTDE Control interface");
            rtde_control_->reconnect();
            rtde_receive_->reconnect();
            connected = true;
        }
    }
    return connected;
}

RTDEControl::RTDEControl(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(
          asyn_port_name, MAX_CONTROLLERS,
          asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask | asynFloat64ArrayMask |
              asynInt32ArrayMask,
          asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask | asynInt32ArrayMask,
          ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1
          0, 0),
      rtde_control_(nullptr), rtde_receive_(nullptr), robot_ip_(robot_ip) {

    createParam(DISCONNECT_STRING, asynParamInt32, &disconnectIndex_);
    createParam(RECONNECT_STRING, asynParamInt32, &reconnectIndex_);
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);
    createParam(J1CMD_STRING, asynParamFloat64, &J1CmdIndex_);
    createParam(J2CMD_STRING, asynParamFloat64, &J2CmdIndex_);
    createParam(J3CMD_STRING, asynParamFloat64, &J3CmdIndex_);
    createParam(J4CMD_STRING, asynParamFloat64, &J4CmdIndex_);
    createParam(J5CMD_STRING, asynParamFloat64, &J5CmdIndex_);
    createParam(J6CMD_STRING, asynParamFloat64, &J6CmdIndex_);
    createParam(MOVEJ_STRING, asynParamInt32, &moveJIndex_);
    createParam(ACTUAL_Q_STRING, asynParamFloat64Array, &actualQIndex_);

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
                // any additional rtde calls go here
                std::vector<double> jvec = rtde_receive_->getActualQ();
                doCallbacksFloat64Array(jvec.data(), NUM_JOINTS, actualQIndex_, 0);
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
        {J1CmdIndex_, 0},
        {J2CmdIndex_, 1},
        {J3CmdIndex_, 2},
        {J4CmdIndex_, 3},
        {J5CmdIndex_, 4},
        {J6CmdIndex_, 5}
    };

    if (rtde_control_ == nullptr) {
        spdlog::error("RTDE Control interface not initialized");
        comm_ok = false;
        goto skip;
    }

    if (not rtde_control_->isConnected()) {
        spdlog::warn("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    // Whenever commanded joint angles change, we update the values
    if (JxCmd_map.count(function) > 0) {
        cmd_joints.at(JxCmd_map[function]) = value;
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communincation error in RTDEControl::writeFloat64");
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
        spdlog::info("Disconnecting from RTDE control interface");
        rtde_control_->disconnect();
        rtde_receive_->disconnect();
        comm_ok = not rtde_control_->isConnected() and not rtde_receive_->isConnected();
        goto skip;
    }

    // Check that it's connected before conntinuing
    if (not rtde_control_->isConnected()) {
        spdlog::warn("RTDE Control interface not connected");
        comm_ok = false;
        goto skip;
    }

    // RTDE Control interface function calls go here
    if (function == moveJIndex_) {
        spdlog::info("moveJ({}, {}, {}, {}, {}, {})", cmd_joints.at(0), cmd_joints.at(1), cmd_joints.at(2),
                     cmd_joints.at(3), cmd_joints.at(4), cmd_joints.at(5));
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communincation error in RTDEControl::writeInt32");
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
