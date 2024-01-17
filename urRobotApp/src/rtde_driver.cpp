#include "rtde_driver.hpp"
#include <asynOctetSyncIO.h>
#include <bitset>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>
#include <memory>

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

static constexpr int NUM_JOINTS = 6;

URRobotRTDE::URRobotRTDE(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask |
                         asynOctetMask | asynFloat64ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask |
                         asynFloat64ArrayMask,
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
    createParam(CONTROLLER_TIMESTAMP_STRING, asynParamFloat64, &controllerTimestampIndex_);
    createParam(STD_ANALOG_INPUT0_STRING, asynParamFloat64, &stdAnalogInput0Index_);
    createParam(STD_ANALOG_INPUT1_STRING, asynParamFloat64, &stdAnalogInput1Index_);
    createParam(STD_ANALOG_OUTPUT0_STRING, asynParamFloat64, &stdAnalogOutput0Index_);
    createParam(STD_ANALOG_OUTPUT1_STRING, asynParamFloat64, &stdAnalogOutput1Index_);
    createParam(ACTUAL_JOINT_POS_STRING, asynParamFloat64Array, &actualJointPosIndex_);

    // Check that robot is connected
    if (rtde_receive_->isConnected()) {
        std::cout << "Robot connected successfully" << std::endl;
        setIntegerParam(isConnectedIndex_, 1);
    } else {
        std::cout << "Failed to connect to robot host " << robot_ip << std::endl;
        setIntegerParam(isConnectedIndex_, 0);
    }

    epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)main_loop_thread_C, this);

}

void URRobotRTDE::main_loop() {
    while (true) {
        lock();

        if (rtde_receive_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);

            // TODO: extract individual bits in EPICS?
            uint32_t safety_status_int = rtde_receive_->getSafetyStatusBits();
            std::bitset<32> bits(safety_status_int);

            setDoubleParam(controllerTimestampIndex_, rtde_receive_->getTimestamp());
            setIntegerParam(safetyStatusIndex_, safety_status_int);
            setIntegerParam(runtimeStateIndex_, rtde_receive_->getRuntimeState());
            setIntegerParam(robotModeIndex_, rtde_receive_->getRobotMode());

            setDoubleParam(stdAnalogInput0Index_,rtde_receive_->getStandardAnalogInput0());
            setDoubleParam(stdAnalogInput1Index_,rtde_receive_->getStandardAnalogInput1());
            setDoubleParam(stdAnalogOutput0Index_, rtde_receive_->getStandardAnalogOutput0());
            setDoubleParam(stdAnalogOutput1Index_, rtde_receive_->getStandardAnalogOutput1());
            
            std::vector<double> Qvec = rtde_receive_->getActualQ();
            epicsFloat64 Q[NUM_JOINTS];
            for (int i = 0; i < NUM_JOINTS; i++) {
                Q[i] = Qvec.at(i);
            }
            doCallbacksFloat64Array(Q, NUM_JOINTS, actualJointPosIndex_, 0);


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
