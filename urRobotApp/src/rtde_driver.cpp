#include "rtde_driver.hpp"

#include <asynOctetSyncIO.h>
#include <bitset>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <iocsh.h>

#include <iostream>
#include <memory>
#include <stdexcept>

#include "ur_rtde/rtde_receive_interface.h"

static void main_loop_thread_C(void *pPvt) {
    URRobotRTDE *pURRobotRTDE = (URRobotRTDE *)pPvt;
    pURRobotRTDE->main_loop();
}

// debugging function for development
template <typename T>
void print_vector(const std::vector<T> &vec, const std::string_view pref = "",
                  const std::string_view sep = ",") {
    std::cout << pref;
    for (const auto &element : vec) {
        std::cout << element << sep;
    }
    std::cout << std::endl;
}

URRobotRTDE::URRobotRTDE(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask |
                         asynOctetMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask |
                         asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      rtde_receive_(std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip)),
      poll_time_(DEFAULT_POLL_TIME) {

    // create asyn parameters
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);
    createParam(RUNTIME_STATE_STRING, asynParamInt32, &runtimeStateIndex_);
    createParam(ROBOT_MODE_STRING, asynParamInt32, &robotModeIndex_);
    createParam(SAFETY_STATUS_STRING, asynParamInt32, &safetyStatusIndex_);
    createParam(CONTROLLER_TIMESTAMP_STRING, asynParamFloat64,
                &controllerTimestampIndex_);

    // Check that robot is connected
    if (rtde_receive_->isConnected()) {
        std::cout << "Robot connected successfully" << std::endl;
        setIntegerParam(isConnectedIndex_, 1);
    } else {
        std::cout << "Failed to connect to robot host " << robot_ip
                  << std::endl;
        setIntegerParam(isConnectedIndex_, 0);
        throw std::runtime_error("Could not connect to robot");
    }

    std::cout << "starting main loop..." << std::endl;
    epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)main_loop_thread_C, this);
}

void URRobotRTDE::main_loop() {
    while (true) {
        lock();

        if (rtde_receive_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);

            // print controller timestamp for debugging
            std::cout << "Controller timestamp: "
                      << rtde_receive_->getTimestamp() << std::endl;

            // set asyn parameters
            setDoubleParam(controllerTimestampIndex_,
                           rtde_receive_->getTimestamp());

            // get safety status (TODO: extract individual bits in EPICS)
            uint32_t safety_status_int = rtde_receive_->getSafetyStatusBits();
            std::bitset<32> bits(safety_status_int);
            setIntegerParam(safetyStatusIndex_, safety_status_int);

            // get runtime state
            uint32_t runtime_state = rtde_receive_->getRuntimeState();
            std::cout << "Runtime state: " << runtime_state << std::endl;
            setIntegerParam(runtimeStateIndex_, runtime_state);

            // get robot mode
            int32_t robot_mode = rtde_receive_->getRobotMode();
            std::cout << "Robot mode: " << robot_mode << std::endl;
            setIntegerParam(robotModeIndex_, robot_mode);

        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_time_);
    }
}

// register function for iocsh
extern "C" int URRobotRTDEConfig(const char *asyn_port_name, const char *robot_ip) {
    URRobotRTDE *pURRobotRTDE = new URRobotRTDE(asyn_port_name, robot_ip);
    pURRobotRTDE = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URRobotRTDEConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    URRobotRTDEConfig(args[0].sval, args[1].sval);
}

void URRobotRTDERegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(URRobotRTDERegister);
}
