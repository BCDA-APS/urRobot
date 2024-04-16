#include <cstddef>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "gripper_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

bool Gripper::try_connect() {
    bool connected = false;
    // TODO: uncomment
    //
    // try {
    // robotiq_gripper_->connect();
    // if (robotiq_gripper_->isConnected()) {
    // spdlog::info("Connected to Robotiq gripper");
    // connected = true;
    // }
    // } catch (const std::exception &e) {
    // spdlog::error(e.what());
    // }
    return connected;
}

static void poll_thread_C(void *pPvt) {
    Gripper *pGripper = (Gripper *)pPvt;
    pGripper->poll();
}

Gripper::Gripper(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask |
                         asynFloat64ArrayMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask |
                         asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1
                     0, 0),
      robotiq_gripper_(std::make_unique<ur_rtde::RobotiqGripper>(robot_ip)), robot_ip_(robot_ip) {

    createParam(CONNECT_STRING, asynParamInt32, &connectIndex_);
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    try_connect();

    epicsThreadCreate("GripperPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void Gripper::poll() {
    while (true) {
        lock();
        if (robotiq_gripper_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);
            // other stuff goes here
        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }
        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus Gripper::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    // Check that it's connected before continuing
    // TODO: uncommment
    // if (not robotiq_gripper_->isConnected()) {
    // spdlog::error("Robotiq gripper not connected");
    // comm_ok = false;
    // goto skip;
    // }

    if (function == connectIndex_) {
        spdlog::debug("Connecting to Robotiq gripper");
        try_connect();
    }

    // skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("Communincation error in Gripper::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int GripperConfig(const char *asyn_port_name, const char *robot_ip) {
    Gripper *pGripper = new Gripper(asyn_port_name, robot_ip);
    pGripper = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"GripperConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) { GripperConfig(args[0].sval, args[1].sval); }

void GripperRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(GripperRegister);
}
