#include "ur_driver.hpp"

#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <iocsh.h>

#include <iostream>
#include <memory>
#include <stdexcept>

#include "ur_rtde/rtde_receive_interface.h"

static void main_loop_thread_C(void *pPvt) {
    URRobot *pURRobot = (URRobot *)pPvt;
    pURRobot->main_loop();
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

URRobot::URRobot(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask |
                         asynOctetMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      rtde_receive_(std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip)),
      poll_time_(DEFAULT_POLL_TIME), count_(0), connected_(0) {
    // create asyn parameters
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);

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

    pose_ = rtde_receive_->getActualTCPPose();
    print_vector(pose_, "Pose: ");

    std::cout << "starting main loop..." << std::endl;
    epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)main_loop_thread_C, this);
}

void URRobot::main_loop() {
    while (true) {
        lock();

        std::cout << "count = " << count_ << std::endl;
        count_ += 1;

        if (rtde_receive_) {
            pose_ = rtde_receive_->getActualTCPPose();
            print_vector(pose_, "Pose: ");
        }

        callParamCallbacks();

        unlock();
        epicsThreadSleep(poll_time_);
    }
}

// register function for iocsh
extern "C" int URRobotConfig(const char *asyn_port_name, const char *robot_ip) {
    URRobot *pURRobot = new URRobot(asyn_port_name, robot_ip);
    pURRobot = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URRobotConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    URRobotConfig(args[0].sval, args[1].sval);
}

void URRobotRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(URRobotRegister);
}
