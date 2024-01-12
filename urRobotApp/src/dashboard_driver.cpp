#include "dashboard_driver.hpp"
#include "ur_rtde/dashboard_client.h"

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <iocsh.h>

#include <iostream>
#include <stdexcept>

static void main_loop_thread_C(void *pPvt) {
    URRobotDashboard *pURRobotDashboard = (URRobotDashboard *)pPvt;
    pURRobotDashboard->main_loop();
}

URRobotDashboard::URRobotDashboard(const char *asyn_port_name,
                                   const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask |
                         asynOctetMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask |
                         asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      ur_dashboard_(std::make_unique<ur_rtde::DashboardClient>(robot_ip)),
      poll_time_(DEFAULT_POLL_TIME) {

    // create asyn parameters
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);

    ur_dashboard_->connect();
    if (ur_dashboard_->isConnected()) {
        std::cout << "Connected to dashboard server" << std::endl;
        setIntegerParam(isConnectedIndex_, 1);
    } else {
        setIntegerParam(isConnectedIndex_, 0);
        throw std::runtime_error("Failed to connect to UR dashboard server");
    }

    std::cout << "starting main loop..." << std::endl;
    epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)main_loop_thread_C, this);
}

void URRobotDashboard::main_loop() {
    while (true) {
        lock();

        if (ur_dashboard_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);
            std::string mode = ur_dashboard_->robotmode();
            std::cout << mode << std::endl;
        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_time_);
    }
}

// register function for iocsh
extern "C" int URRobotDashboardConfig(const char *asyn_port_name,
                                      const char *robot_ip) {
    URRobotDashboard *pURRobotDashboard =
        new URRobotDashboard(asyn_port_name, robot_ip);
    pURRobotDashboard = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URRobotDashboardConfig", 2,
                                            urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    URRobotDashboardConfig(args[0].sval, args[1].sval);
}

void URRobotDashboardRegister(void) {
    iocshRegister(&urRobotFuncDef, urRobotCallFunc);
}

extern "C" {
epicsExportRegistrar(URRobotDashboardRegister);
}
