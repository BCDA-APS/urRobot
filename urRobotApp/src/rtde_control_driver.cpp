#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "rtde_control_driver.hpp"
#include "spdlog/spdlog.h"
#include "ur_rtde/dashboard_client.h"

static void poll_thread_C(void *pPvt) {
    RTDEControl *pRTDEControl = (RTDEControl *)pPvt;
    pRTDEControl->poll();
}

// Wraps class construction/connection to fail gracefully.
// Unlike the DashboardClient class, construction of the object tries
// connecting automatically. If connection fails (IP is wrong) the RTDE
// constructor throws and error, so subsequent calls like
// rtde_control_->isConnected() will segfault since the rtde_recieve_ object
// does not point to anything. This is why we have the `this->connected` variable
bool RTDEControl::try_connect() {
    bool connected = false;

    if (rtde_control_ == nullptr) {
        try {
            auto dash = std::make_unique<ur_rtde::DashboardClient>(robot_ip_);
            dash->connect();
            if (dash->robotmode() == "Robotmode: RUNNING") {
                rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
            } else {
                spdlog::error("Unable to connect to UR RTDE Control Interface\nEnsure robot is on "
                              "and brakes released");
                connected = false;
                dash->disconnect();
            }
            if (rtde_control_ != nullptr) {
                if (rtde_control_->isConnected()) {
                    spdlog::info("Connected to UR RTDE Control interface");
                    connected = true;
                }
            }
        } catch (const std::exception &e) {
            spdlog::error("Failed to connected to UR RTDE Control interface\n{}", e.what());
        }
    } else {
        if (not rtde_control_->isConnected()) {
            rtde_control_->reconnect();
            spdlog::info("Reconnecting to UR RTDE Control interface");
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
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      rtde_control_(nullptr), robot_ip_(robot_ip) {

    // RTDE Control
    createParam(DISCONNECT_STRING, asynParamInt32, &disconnectIndex_);
    createParam(RECONNECT_STRING, asynParamInt32, &reconnectIndex_);
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);

    // TODO: make log level an arg to the constructor?
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug

    try_connect();

    epicsThreadCreate("RTDEControlPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEControl::poll() {
    while (true) {
        lock();

        if (rtde_control_ != nullptr) {
            if (rtde_control_->isConnected()) {
                setIntegerParam(isConnectedIndex_, 1);
            } else {
                setIntegerParam(isConnectedIndex_, 0);
            }
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus RTDEControl::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;

    callParamCallbacks();
    return asynSuccess;
}

asynStatus RTDEControl::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == reconnectIndex_) {
        comm_ok = try_connect();
        if (not comm_ok) {
            return asynError;
        }
    } else if (function == disconnectIndex_) {
        rtde_control_->disconnect();
    }

    if (rtde_control_ == nullptr) {
        spdlog::error("RTDE Control interface not initialized");
        return asynError;
    }

    // if control is not connected, return
    if (not rtde_control_->isConnected()) {
        spdlog::warn("RTDE Control + I/O interfaces not connected");
        return asynError;
    }

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

static void urRobotCallFunc(const iocshArgBuf *args) {
    RTDEControlConfig(args[0].sval, args[1].sval);
}

void RTDEControlRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEControlRegister);
}
