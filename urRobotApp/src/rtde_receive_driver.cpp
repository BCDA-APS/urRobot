#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "rtde_receive_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

static void poll_thread_C(void *pPvt) {
    RTDEReceive *pRTDEReceive = (RTDEReceive *)pPvt;
    pRTDEReceive->poll();
}

bool RTDEReceive::try_connect() {
    bool connected = false;

    if (rtde_receive_ == nullptr) {
        try {
            rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
            if (rtde_receive_ != nullptr) {
                if (rtde_receive_->isConnected()) {
                    spdlog::info("Connected to UR RTDE Receive interface");
                    connected = true;
                }
            }
        } catch (const std::exception &e) {
            spdlog::error("Failed to connected to UR RTDE Receive interface\n{}", e.what());
            connected = false;
        }
    } else {
        if (not rtde_receive_->isConnected()) {
            spdlog::info("Reconnecting to UR RTDE Receive interface");
            rtde_receive_->reconnect();
            connected = true; // HACK: check if reconnection worked here?
        }
    }
    return connected;
}

static constexpr int NUM_JOINTS = 6;
static constexpr int ASYN_ADDR = 0;

RTDEReceive::RTDEReceive(const char *asyn_port_name, const char *robot_ip, const double poll_period)
    : asynPortDriver(
          asyn_port_name, MAX_CONTROLLERS,
          asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask | asynFloat64ArrayMask |
              asynInt32ArrayMask,
          asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask | asynInt32ArrayMask,
          ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
          0, 0),
      rtde_receive_(nullptr), robot_ip_(robot_ip), poll_period_(poll_period) {

    createParam(DISCONNECT_STRING, asynParamInt32, &disconnectIndex_);
    createParam(RECONNECT_STRING, asynParamInt32, &reconnectIndex_);
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

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    try_connect();

    epicsThreadCreate("RTDEReceivePoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEReceive::poll() {
    std::vector<double> vec_f64(NUM_JOINTS);
    std::vector<int> joint_modes_vec(NUM_JOINTS);

    while (true) {
        lock();

        if (rtde_receive_ != nullptr) {

            if (rtde_receive_->isConnected()) {
                setIntegerParam(isConnectedIndex_, 1);

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

                joint_modes_vec = rtde_receive_->getJointMode();
                doCallbacksInt32Array(joint_modes_vec.data(), NUM_JOINTS, jointModesIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualToolAccelerometer();
                doCallbacksFloat64Array(vec_f64.data(), 3, actualToolAccelIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualQ();
                for (double &j : vec_f64) { // convert to degrees
                    j = j * 180.0 / M_PI;
                }
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualJointPosIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualTCPPose();
                for (size_t i = 0; i < vec_f64.size(); i++) {
                    if (i <= 2) {
                        vec_f64.at(i) = vec_f64.at(i) * 1000.0; // convert m -> mm
                    } else {
                        vec_f64.at(i) = vec_f64.at(i) * 180.0 / M_PI; // convert rad -> deg
                    }
                }
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualTCPPoseIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualQd();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualJointVelIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualCurrent();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualJointCurrentsIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getJointControlOutput();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, jointControlCurrentsIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualTCPSpeed();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualTCPSpeedIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualTCPForce();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualTCPForceIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getTargetQ();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, targetJointPosIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getTargetQd();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, targetJointVelIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getTargetQdd();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, targetJointAccelIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getTargetCurrent();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, targetJointCurrentsIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getTargetMoment();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, targetJointMomentsIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getTargetTCPPose();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, targetTCPPoseIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getTargetTCPSpeed();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, targetTCPSpeedIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getJointTemperatures();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, jointTemperaturesIndex_, ASYN_ADDR);

                vec_f64 = rtde_receive_->getActualJointVoltage();
                doCallbacksFloat64Array(vec_f64.data(), NUM_JOINTS, actualJointVoltagesIndex_, ASYN_ADDR);

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

asynStatus RTDEReceive::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == reconnectIndex_) {
        comm_ok = try_connect();
        goto skip;
    }

    if (rtde_receive_ == nullptr) {
        spdlog::error("RTDE Receive interface not initialized");
        comm_ok = false;
        goto skip;
    }

    if (function == disconnectIndex_) {
        spdlog::info("Disconnecting from RTDE receive interface");
        rtde_receive_->disconnect();
        if (not rtde_receive_->isConnected()) {
            comm_ok = true;
        } else {
            comm_ok = false;
        }
        goto skip;
    }

    if (not rtde_receive_->isConnected()) {
        spdlog::warn("RTDE Receive interface not connected");
        comm_ok = false;
        goto skip;
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::error("Communincation error in RTDEReceive::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int RTDEReceiveConfig(const char *asyn_port_name, const char *robot_ip, double poll_period) {
    RTDEReceive *pRTDEReceive = new RTDEReceive(asyn_port_name, robot_ip, poll_period);
    (void)pRTDEReceive;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg urRobotArg2 = {"Poll period", iocshArgDouble};
static const iocshArg *const urRobotArgs[3] = {&urRobotArg0, &urRobotArg1, &urRobotArg2};
static const iocshFuncDef urRobotFuncDef = {"RTDEReceiveConfig", 3, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) { RTDEReceiveConfig(args[0].sval, args[1].sval, args[2].dval); }

void RTDEReceiveRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEReceiveRegister);
}
