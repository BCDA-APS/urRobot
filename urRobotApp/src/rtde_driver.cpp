#include <asynOctetSyncIO.h>
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

// Wraps class construction/connection to fail gracefully.
// Unlike the DashboardClient class, construction of the object tries
// connecting automatically. If connection fails (IP is wrong) the RTDE
// constructor throws and error, so subsequent calls like
// rtde_receive_->isConnected() will segfault since the rtde_recieve_ object
// does not point to anything. This is why we have the `this->connected` variable
bool URRobotRTDE::try_construction(const char *robot_ip) {
    bool connected = false;
    try {
        // construction of the RTDE objects automatically tries connecting
        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip);
        rtde_io_ = std::make_unique<ur_rtde::RTDEIOInterface>(robot_ip);
        if (rtde_receive_->isConnected()) {
            spdlog::info("Connected to UR RTDE interface");
            connected = true;
        }
    } catch (const std::exception &e) {
        spdlog::error(e.what());
    }
    return connected;
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

    createParam(DISCONNECT_STRING, asynParamInt32, &disconnectIndex_);
    createParam(RECONNECT_STRING, asynParamInt32, &reconnectIndex_);
    createParam(SET_CONFIG_DOUT7_STRING, asynParamInt32, &setConfigDOUT7Index_);
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
    createParam(TARGET_JOINT_POS_STRING, asynParamFloat64Array, &targetJointPosIndex_);
    createParam(TARGET_JOINT_VEL_STRING, asynParamFloat64Array, &targetJointVelIndex_);
    createParam(TARGET_JOINT_ACCEL_STRING, asynParamFloat64Array, &targetJointAccelIndex_);
    createParam(TARGET_JOINT_CURRENTS_STRING, asynParamFloat64Array, &targetJointCurrentsIndex_);
    createParam(TARGET_JOINT_MOMENTS_STRING, asynParamFloat64Array, &targetJointMomentsIndex_);
    createParam(TARGET_TCP_POSE_STRING, asynParamFloat64Array, &targetTCPPoseIndex_);
    createParam(TARGET_TCP_SPEED_STRING, asynParamFloat64Array, &targetTCPSpeedIndex_);
    createParam(JOINT_TEMPERATURES_STRING, asynParamFloat64Array, &jointTemperaturesIndex_);
    createParam(SPEED_SCALING_STRING, asynParamFloat64, &speedScalingIndex_);
    createParam(TARGET_SPEED_FRACTION_STRING, asynParamFloat64, &targetSpeedFractionIndex_);
    createParam(ACTUAL_MOMENTUM_STRING, asynParamFloat64, &actualMomentumIndex_);
    createParam(ACTUAL_MAIN_VOLTAGE_STRING, asynParamFloat64, &actualMainVoltageIndex_);
    createParam(ACTUAL_ROBOT_VOLTAGE_STRING, asynParamFloat64, &actualRobotVoltageIndex_);
    createParam(ACTUAL_ROBOT_CURRENT_STRING, asynParamFloat64, &actualRobotCurrentIndex_);
    createParam(ACTUAL_JOINT_VOLTAGES_STRING, asynParamFloat64Array, &actualJointVoltagesIndex_);

    // TODO: make log level an arg to the constructor?
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug

    this->connected = this->try_construction(robot_ip);

    epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)poll_thread_C, this);
}

void URRobotRTDE::poll() {
    while (true) {
        lock();

        if (this->connected) {
            if (rtde_receive_->isConnected()) {
                this->connected = true;
                setIntegerParam(isConnectedIndex_, 1);
            } else {
                setIntegerParam(isConnectedIndex_, 0);
                this->connected = false;
                spdlog::warn("UR RTDE interface disconnected");
            }
            setDoubleParam(controllerTimestampIndex_, rtde_receive_->getTimestamp());
            setIntegerParam(safetyStatusBitsIndex_, rtde_receive_->getSafetyStatusBits());
            setIntegerParam(runtimeStateIndex_, rtde_receive_->getRuntimeState());
            setIntegerParam(robotModeIndex_, rtde_receive_->getRobotMode());
            setIntegerParam(safetyModeIndex_, rtde_receive_->getSafetyMode());
            setIntegerParam(digitalInputBitsIndex_, rtde_receive_->getActualDigitalInputBits());
            setIntegerParam(digitalOutputBitsIndex_, rtde_receive_->getActualDigitalOutputBits());
            setDoubleParam(stdAnalogInput0Index_, rtde_receive_->getStandardAnalogInput0());
            setDoubleParam(stdAnalogInput1Index_, rtde_receive_->getStandardAnalogInput1());
            setDoubleParam(stdAnalogOutput0Index_, rtde_receive_->getStandardAnalogOutput0());
            setDoubleParam(stdAnalogOutput1Index_, rtde_receive_->getStandardAnalogOutput1());
            setDoubleParam(speedScalingIndex_, rtde_receive_->getSpeedScaling());
            setDoubleParam(targetSpeedFractionIndex_, rtde_receive_->getTargetSpeedFraction());
            setDoubleParam(actualMomentumIndex_, rtde_receive_->getActualMomentum());
            setDoubleParam(actualMainVoltageIndex_, rtde_receive_->getActualMainVoltage());
            setDoubleParam(actualRobotVoltageIndex_, rtde_receive_->getActualRobotVoltage());
            setDoubleParam(actualRobotCurrentIndex_, rtde_receive_->getActualRobotCurrent());

            // all the rtde functions that return vector<double> use this array and vector
            epicsFloat64 arr_f64[NUM_JOINTS];
            std::vector<double> vec_f64(NUM_JOINTS);

            const std::vector<int32_t> joint_modes_vec = rtde_receive_->getJointMode();
            epicsInt32 joint_modes[NUM_JOINTS];
            std::copy(joint_modes_vec.begin(), joint_modes_vec.end(), joint_modes);
            doCallbacksInt32Array(joint_modes, NUM_JOINTS, jointModesIndex_, 0);

            vec_f64 = rtde_receive_->getActualQ();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualJointPosIndex_, 0);

            vec_f64 = rtde_receive_->getActualQd();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, actualJointVelIndex_, 0);

            vec_f64 = rtde_receive_->getActualCurrent();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, actualJointCurrentsIndex_, 0);

            vec_f64 = rtde_receive_->getJointControlOutput();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, jointControlCurrentsIndex_, 0);

            vec_f64 = rtde_receive_->getActualTCPPose();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, actualTCPPoseIndex_, 0);

            vec_f64 = rtde_receive_->getActualTCPSpeed();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, actualTCPSpeedIndex_, 0);

            vec_f64 = rtde_receive_->getActualTCPForce();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, actualTCPForceIndex_, 0);

            vec_f64 = rtde_receive_->getActualToolAccelerometer();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, 3, actualToolAccelIndex_, 0);

            vec_f64 = rtde_receive_->getTargetQ();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, targetJointPosIndex_, 0);

            vec_f64 = rtde_receive_->getTargetQd();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, targetJointVelIndex_, 0);

            vec_f64 = rtde_receive_->getTargetQdd();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, targetJointAccelIndex_, 0);

            vec_f64 = rtde_receive_->getTargetCurrent();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, targetJointCurrentsIndex_, 0);

            vec_f64 = rtde_receive_->getTargetMoment();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, targetJointMomentsIndex_, 0);

            vec_f64 = rtde_receive_->getTargetTCPPose();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, targetTCPPoseIndex_, 0);

            vec_f64 = rtde_receive_->getTargetTCPSpeed();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, targetTCPSpeedIndex_, 0);

            vec_f64 = rtde_receive_->getJointTemperatures();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, jointTemperaturesIndex_, 0);

            vec_f64 = rtde_receive_->getActualJointVoltage();
            std::copy(vec_f64.begin(), vec_f64.end(), arr_f64);
            doCallbacksFloat64Array(arr_f64, NUM_JOINTS, actualJointVoltagesIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus URRobotRTDE::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;

    bool rtde_ok = true;
    if (function == speedSliderIndex_) {
        rtde_ok = rtde_io_->setSpeedSlider(value);
    } else if (function == setVoltageAOUT0Index_) {
        rtde_ok = rtde_io_->setAnalogOutputVoltage(0, value);
    } else if (function == setVoltageAOUT1Index_) {
        rtde_ok = rtde_io_->setAnalogOutputVoltage(1, value);
    } else if (function == setCurrentAOUT0Index_) {
        rtde_ok = rtde_io_->setAnalogOutputCurrent(0, value);
    } else if (function == setCurrentAOUT1Index_) {
        rtde_ok = rtde_io_->setAnalogOutputCurrent(1, value);
    }

    callParamCallbacks();
    if (rtde_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communication error in URRobotRTDE::writeFloat64");
        return asynError;
    }
}

asynStatus URRobotRTDE::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;

    bool rtde_ok = true;
    if (function == setStandardDOUT0Index_) {
        spdlog::info("Setting standard digital output 0 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(0, static_cast<bool>(value));
    } else if (function == setStandardDOUT1Index_) {
        spdlog::info("Setting standard digital output 1 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(1, static_cast<bool>(value));
    } else if (function == setStandardDOUT2Index_) {
        spdlog::info("Setting standard digital output 2 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(2, static_cast<bool>(value));
    } else if (function == setStandardDOUT3Index_) {
        spdlog::info("Setting standard digital output 3 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(3, static_cast<bool>(value));
    } else if (function == setStandardDOUT4Index_) {
        spdlog::info("Setting standard digital output 4 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(4, static_cast<bool>(value));
    } else if (function == setStandardDOUT5Index_) {
        spdlog::info("Setting standard digital output 5 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(5, static_cast<bool>(value));
    } else if (function == setStandardDOUT6Index_) {
        spdlog::info("Setting standard digital output 6 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(6, static_cast<bool>(value));
    } else if (function == setStandardDOUT7Index_) {
        spdlog::info("Setting standard digital output 7 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setStandardDigitalOut(7, static_cast<bool>(value));
    }

    else if (function == setConfigDOUT0Index_) {
        spdlog::info("Setting configurable digital output 0 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(0, static_cast<bool>(value));
    } else if (function == setConfigDOUT1Index_) {
        spdlog::info("Setting configurable digital output 1 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(1, static_cast<bool>(value));
    } else if (function == setConfigDOUT2Index_) {
        spdlog::info("Setting configurable digital output 2 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(2, static_cast<bool>(value));
    } else if (function == setConfigDOUT3Index_) {
        spdlog::info("Setting configurable digital output 3 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(3, static_cast<bool>(value));
    } else if (function == setConfigDOUT4Index_) {
        spdlog::info("Setting configurable digital output 4 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(4, static_cast<bool>(value));
    } else if (function == setConfigDOUT5Index_) {
        spdlog::info("Setting configurable digital output 5 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(5, static_cast<bool>(value));
    } else if (function == setConfigDOUT6Index_) {
        spdlog::info("Setting configurable digital output 6 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(6, static_cast<bool>(value));
    } else if (function == setConfigDOUT7Index_) {
        spdlog::info("Setting configurable digital output 7 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setConfigurableDigitalOut(7, static_cast<bool>(value));
    }

    else if (function == setToolDOUT0Index_) {
        spdlog::info("Setting tool digital output 0 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setToolDigitalOut(0, static_cast<bool>(value));
    } else if (function == setToolDOUT1Index_) {
        spdlog::info("Setting tool digital output 1 {}", value == 1 ? "high" : "low");
        rtde_ok = rtde_io_->setToolDigitalOut(1, static_cast<bool>(value));
    }

    else if (function == disconnectIndex_) {
        spdlog::info("Disconnecting from RTDE interface");
        rtde_receive_->disconnect();
    } else if (function == reconnectIndex_) {
        try {
            rtde_ok = rtde_receive_->reconnect();
            this->connected = rtde_ok;
            spdlog::info("Reconnected to RTDE interface");
        } catch (const std::exception &e) {
            spdlog::error(e.what());
        }
    }

    callParamCallbacks();
    if (rtde_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communincation error in URRobotRTDE::writeInt32");
        return asynError;
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
