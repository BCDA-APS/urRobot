#include <asynOctetSyncIO.h>
#include <bitset>
#include <cstdint>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <memory>

#include "rtde_driver.hpp"
#include "spdlog/spdlog.h"
#include "ur_rtde/rtde_receive_interface.h"

static void poll_thread_C(void *pPvt) {
    URRobotRTDE *pURRobotRTDE = (URRobotRTDE *)pPvt;
    pURRobotRTDE->poll();
}

static constexpr int NUM_JOINTS = 6;

URRobotRTDE::URRobotRTDE(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask |
                         asynFloat64ArrayMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask |
                         asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      rtde_receive_(nullptr), rtde_io_(nullptr) {

    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);
    createParam(RUNTIME_STATE_STRING, asynParamInt32, &runtimeStateIndex_);
    createParam(ROBOT_MODE_STRING, asynParamInt32, &robotModeIndex_);
    createParam(SAFETY_STATUS_BITS_STRING, asynParamInt32, &safetyStatusBitsIndex_);
    createParam(CONTROLLER_TIMESTAMP_STRING, asynParamFloat64, &controllerTimestampIndex_);
    createParam(STD_ANALOG_INPUT0_STRING, asynParamFloat64, &stdAnalogInput0Index_);
    createParam(STD_ANALOG_INPUT1_STRING, asynParamFloat64, &stdAnalogInput1Index_);
    createParam(STD_ANALOG_OUTPUT0_STRING, asynParamFloat64, &stdAnalogOutput0Index_);
    createParam(STD_ANALOG_OUTPUT1_STRING, asynParamFloat64, &stdAnalogOutput1Index_);
    createParam(ACTUAL_JOINT_POS_STRING, asynParamFloat64Array, &actualJointPosIndex_);
    createParam(DIGITAL_INPUT_BITS_STRING, asynParamInt32, &digitalInputBitsIndex_);
    createParam(DIGITAL_OUTPUT_BITS_STRING, asynParamInt32, &digitalOutputBitsIndex_);
    createParam(ACTUAL_JOINT_VEL_STRING, asynParamFloat64Array, &actualJointVelIndex_);
    createParam(ACTUAL_JOINT_CURRENTS_STRING, asynParamFloat64Array, &actualJointCurrentsIndex_);
    createParam(JOINT_CONTROL_CURRENTS_STRING, asynParamFloat64Array, &jointControlCurrentsIndex_);
    createParam(ACTUAL_TCP_POSE_STRING, asynParamFloat64Array, &actualTCPPoseIndex_);
    createParam(ACTUAL_TCP_SPEED_STRING, asynParamFloat64Array, &actualTCPSpeedIndex_);
    createParam(ACTUAL_TCP_FORCE_STRING, asynParamFloat64Array, &actualTCPForceIndex_);
    createParam(SAFETY_MODE_STRING, asynParamInt32, &safetyModeIndex_);
    createParam(JOINT_MODES_STRING, asynParamInt32Array, &jointModesIndex_);
    createParam(ACTUAL_TOOL_ACCEL_STRING, asynParamFloat64Array, &actualToolAccelIndex_);
    createParam(SPEED_SLIDER_STRING, asynParamFloat64, &speedSliderIndex_);
    createParam(SET_STANDARD_DOUT0_STRING, asynParamInt32, &setStandardDOUT0Index_);
    createParam(SET_STANDARD_DOUT1_STRING, asynParamInt32, &setStandardDOUT1Index_);
    createParam(SET_STANDARD_DOUT2_STRING, asynParamInt32, &setStandardDOUT2Index_);
    createParam(SET_STANDARD_DOUT3_STRING, asynParamInt32, &setStandardDOUT3Index_);
    createParam(SET_STANDARD_DOUT4_STRING, asynParamInt32, &setStandardDOUT4Index_);
    createParam(SET_STANDARD_DOUT5_STRING, asynParamInt32, &setStandardDOUT5Index_);
    createParam(SET_STANDARD_DOUT6_STRING, asynParamInt32, &setStandardDOUT6Index_);
    createParam(SET_STANDARD_DOUT7_STRING, asynParamInt32, &setStandardDOUT7Index_);
    createParam(SET_CONFIG_DOUT0_STRING, asynParamInt32, &setConfigDOUT0Index_);
    createParam(SET_CONFIG_DOUT1_STRING, asynParamInt32, &setConfigDOUT1Index_);
    createParam(SET_CONFIG_DOUT2_STRING, asynParamInt32, &setConfigDOUT2Index_);
    createParam(SET_CONFIG_DOUT3_STRING, asynParamInt32, &setConfigDOUT3Index_);
    createParam(SET_CONFIG_DOUT4_STRING, asynParamInt32, &setConfigDOUT4Index_);
    createParam(SET_CONFIG_DOUT5_STRING, asynParamInt32, &setConfigDOUT5Index_);
    createParam(SET_CONFIG_DOUT6_STRING, asynParamInt32, &setConfigDOUT6Index_);
    createParam(SET_CONFIG_DOUT7_STRING, asynParamInt32, &setConfigDOUT7Index_);
    createParam(SET_TOOL_DOUT0_STRING, asynParamInt32, &setToolDOUT0Index_);
    createParam(SET_TOOL_DOUT1_STRING, asynParamInt32, &setToolDOUT1Index_);
    createParam(SET_VOLTAGE_AOUT0_STRING, asynParamFloat64, &setVoltageAOUT0Index_);
    createParam(SET_VOLTAGE_AOUT1_STRING, asynParamFloat64, &setVoltageAOUT1Index_);
    createParam(SET_CURRENT_AOUT0_STRING, asynParamFloat64, &setCurrentAOUT0Index_);
    createParam(SET_CURRENT_AOUT1_STRING, asynParamFloat64, &setCurrentAOUT1Index_);

    bool connected = false;
    try {
        // construction of the RTDE objects automatically tries connecting
        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip);
        rtde_io_ = std::make_unique<ur_rtde::RTDEIOInterface>(robot_ip);

        // Check that RTDE receive interface is connected
        if (rtde_receive_->isConnected()) {
            spdlog::info("Connected to UR RTDE interface");
            setIntegerParam(isConnectedIndex_, 1);
            connected = true;
        } else {
            spdlog::error("Failed to connect to UR RTDE interface");
            setIntegerParam(isConnectedIndex_, 0);
        }

    } catch (const std::exception &e) {
        spdlog::error(e.what());
    }

    if (connected) {
        epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)poll_thread_C, this);
    }
}

void URRobotRTDE::poll() {
    while (true) {
        lock();

        if (rtde_receive_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);

            const uint32_t safety_status_bits = rtde_receive_->getSafetyStatusBits();
            const std::bitset<32> bits(safety_status_bits);

            setDoubleParam(controllerTimestampIndex_, rtde_receive_->getTimestamp());
            setIntegerParam(safetyStatusBitsIndex_, safety_status_bits);
            setIntegerParam(runtimeStateIndex_, rtde_receive_->getRuntimeState());
            setIntegerParam(robotModeIndex_, rtde_receive_->getRobotMode());
            setIntegerParam(safetyModeIndex_, rtde_receive_->getSafetyMode());
            setIntegerParam(digitalInputBitsIndex_, rtde_receive_->getActualDigitalInputBits());
            setIntegerParam(digitalOutputBitsIndex_, rtde_receive_->getActualDigitalOutputBits());

            setDoubleParam(stdAnalogInput0Index_, rtde_receive_->getStandardAnalogInput0());
            setDoubleParam(stdAnalogInput1Index_, rtde_receive_->getStandardAnalogInput1());
            setDoubleParam(stdAnalogOutput0Index_, rtde_receive_->getStandardAnalogOutput0());
            setDoubleParam(stdAnalogOutput1Index_, rtde_receive_->getStandardAnalogOutput1());

            const std::vector<int32_t> joint_modes_vec = rtde_receive_->getJointMode();
            epicsInt32 joint_modes[NUM_JOINTS];
            std::copy(joint_modes_vec.begin(), joint_modes_vec.end(), joint_modes);
            doCallbacksInt32Array(joint_modes, NUM_JOINTS, jointModesIndex_, 0);

            const std::vector<double> Qvec = rtde_receive_->getActualQ();
            epicsFloat64 Q[NUM_JOINTS];
            std::copy(Qvec.begin(), Qvec.end(), Q);
            doCallbacksFloat64Array(Q, NUM_JOINTS, actualJointPosIndex_, 0);

            const std::vector<double> Qdvec = rtde_receive_->getActualQd();
            epicsFloat64 Qd[NUM_JOINTS];
            std::copy(Qdvec.begin(), Qdvec.end(), Qd);
            doCallbacksFloat64Array(Qd, NUM_JOINTS, actualJointVelIndex_, 0);

            const std::vector<double> actual_current_vec = rtde_receive_->getActualCurrent();
            epicsFloat64 actual_current[NUM_JOINTS];
            std::copy(actual_current_vec.begin(), actual_current_vec.end(), actual_current);
            doCallbacksFloat64Array(actual_current, NUM_JOINTS, actualJointCurrentsIndex_, 0);

            const std::vector<double> control_currents_vec = rtde_receive_->getJointControlOutput();
            epicsFloat64 control_currents[NUM_JOINTS];
            std::copy(control_currents_vec.begin(), control_currents_vec.end(), control_currents);
            doCallbacksFloat64Array(control_currents, NUM_JOINTS, jointControlCurrentsIndex_, 0);

            const std::vector<double> actual_pose_vec = rtde_receive_->getActualTCPPose();
            epicsFloat64 actual_pose[NUM_JOINTS];
            std::copy(actual_pose_vec.begin(), actual_pose_vec.end(), actual_pose);
            doCallbacksFloat64Array(actual_pose, NUM_JOINTS, actualTCPPoseIndex_, 0);

            const std::vector<double> actual_speed_vec = rtde_receive_->getActualTCPSpeed();
            epicsFloat64 actual_speed[NUM_JOINTS];
            std::copy(actual_speed_vec.begin(), actual_speed_vec.end(), actual_speed);
            doCallbacksFloat64Array(actual_speed, NUM_JOINTS, actualTCPSpeedIndex_, 0);

            const std::vector<double> actual_force_vec = rtde_receive_->getActualTCPForce();
            epicsFloat64 actual_force[NUM_JOINTS];
            std::copy(actual_force_vec.begin(), actual_force_vec.end(), actual_force);
            doCallbacksFloat64Array(actual_force, NUM_JOINTS, actualTCPForceIndex_, 0);

            const std::vector<double> tool_accel_vec = rtde_receive_->getActualToolAccelerometer();
            epicsFloat64 tool_accel[3];
            std::copy(tool_accel_vec.begin(), tool_accel_vec.end(), tool_accel);
            doCallbacksFloat64Array(tool_accel, 3, actualToolAccelIndex_, 0);

        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus URRobotRTDE::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;

    bool rtde_io_ok = true;
    if (function == speedSliderIndex_) {
        rtde_io_ok = rtde_io_->setSpeedSlider(value);
    } else if (function == setVoltageAOUT0Index_) {
        rtde_io_ok = rtde_io_->setAnalogOutputVoltage(0, value);
    } else if (function == setVoltageAOUT1Index_) {
        rtde_io_ok = rtde_io_->setAnalogOutputVoltage(1, value);
    } else if (function == setCurrentAOUT0Index_) {
        rtde_io_ok = rtde_io_->setAnalogOutputCurrent(0, value);
    } else if (function == setCurrentAOUT1Index_) {
        rtde_io_ok = rtde_io_->setAnalogOutputCurrent(1, value);
    }

    callParamCallbacks();
    if (not rtde_io_ok) {
        spdlog::error("Call to setSpeedSlider failed");
        return asynError;
    } else {
        return asynSuccess;
    }
}

asynStatus URRobotRTDE::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;

    bool rtde_io_ok = true;
    if (function == setStandardDOUT0Index_) {
        spdlog::info("Setting standard digital output 0 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(0, static_cast<bool>(value));
    } else if (function == setStandardDOUT1Index_) {
        spdlog::info("Setting standard digital output 1 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(1, static_cast<bool>(value));
    } else if (function == setStandardDOUT2Index_) {
        spdlog::info("Setting standard digital output 2 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(2, static_cast<bool>(value));
    } else if (function == setStandardDOUT3Index_) {
        spdlog::info("Setting standard digital output 3 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(3, static_cast<bool>(value));
    } else if (function == setStandardDOUT4Index_) {
        spdlog::info("Setting standard digital output 4 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(4, static_cast<bool>(value));
    } else if (function == setStandardDOUT5Index_) {
        spdlog::info("Setting standard digital output 5 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(5, static_cast<bool>(value));
    } else if (function == setStandardDOUT6Index_) {
        spdlog::info("Setting standard digital output 6 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(6, static_cast<bool>(value));
    } else if (function == setStandardDOUT7Index_) {
        spdlog::info("Setting standard digital output 7 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setStandardDigitalOut(7, static_cast<bool>(value));
    }

    else if (function == setConfigDOUT0Index_) {
        spdlog::info("Setting configurable digital output 0 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(0, static_cast<bool>(value));
    } else if (function == setConfigDOUT1Index_) {
        spdlog::info("Setting configurable digital output 1 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(1, static_cast<bool>(value));
    } else if (function == setConfigDOUT2Index_) {
        spdlog::info("Setting configurable digital output 2 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(2, static_cast<bool>(value));
    } else if (function == setConfigDOUT3Index_) {
        spdlog::info("Setting configurable digital output 3 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(3, static_cast<bool>(value));
    } else if (function == setConfigDOUT4Index_) {
        spdlog::info("Setting configurable digital output 4 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(4, static_cast<bool>(value));
    } else if (function == setConfigDOUT5Index_) {
        spdlog::info("Setting configurable digital output 5 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(5, static_cast<bool>(value));
    } else if (function == setConfigDOUT6Index_) {
        spdlog::info("Setting configurable digital output 6 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(6, static_cast<bool>(value));
    } else if (function == setConfigDOUT7Index_) {
        spdlog::info("Setting configurable digital output 7 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setConfigurableDigitalOut(7, static_cast<bool>(value));
    }

    else if (function == setToolDOUT0Index_) {
        spdlog::info("Setting tool digital output 0 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setToolDigitalOut(0, static_cast<bool>(value));
    } else if (function == setToolDOUT1Index_) {
        spdlog::info("Setting tool digital output 1 {}", value == 1 ? "high" : "low");
        rtde_io_ok = rtde_io_->setToolDigitalOut(1, static_cast<bool>(value));
    }

    callParamCallbacks();
    if (not rtde_io_ok) {
        spdlog::error("Call to setStandardDigitalOut failed");
        return asynError;
    } else {
        return asynSuccess;
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
