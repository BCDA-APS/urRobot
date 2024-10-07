#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "rtde_io_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/common.h"
#include "spdlog/spdlog.h"

static void poll_thread_C(void *pPvt) {
    RTDEInOut *pRTDEInOut = (RTDEInOut *)pPvt;
    pRTDEInOut->poll();
}

bool RTDEInOut::try_connect() {
    // We assume if there is no error, IO interface is connected
    // because the RTDEIOInterface class has no isConnected() method
    bool connected = false;
    if (rtde_io_ == nullptr) {
        try {
            rtde_io_ = std::make_unique<ur_rtde::RTDEIOInterface>(robot_ip_);
            spdlog::info("Connected to UR RTDE IO interface");
            connected = true;
        } catch (const std::exception &e) {
            spdlog::error("Failed to connected to UR RTDE I/O interface\n{}", e.what());
        }
    } else {
        rtde_io_->reconnect();
        spdlog::info("Reconnecting to UR RTDE I/O interface");
    }
    return connected;
}

RTDEInOut::RTDEInOut(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(
          asyn_port_name, MAX_CONTROLLERS,
          asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask | asynFloat64ArrayMask |
              asynInt32ArrayMask,
          asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask | asynInt32ArrayMask,
          ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
          0, 0),
      rtde_io_(nullptr), robot_ip_(robot_ip) {

    // RTDE IO
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

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    try_connect();

    epicsThreadCreate("RTDEInOutPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEInOut::poll() {
    while (true) {
        lock();

        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus RTDEInOut::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (rtde_io_ == nullptr) {
        spdlog::error("RTDE IO interface not initialized");
        comm_ok = false;
        goto skip;
    }

    // RTDEIOInterface has no isConnected field for us to check

    if (function == speedSliderIndex_) {
        if ((static_cast<int>(value * 1000) % 10) == 0) {
            // so we don't print too many values
            spdlog::debug("Setting speed slider to {:.2f}", value);
        }
        comm_ok = rtde_io_->setSpeedSlider(value);
    } else if (function == setVoltageAOUT0Index_) {
        spdlog::debug("Setting analog output voltage 0 to {}", value);
        comm_ok = rtde_io_->setAnalogOutputVoltage(0, value);
    } else if (function == setVoltageAOUT1Index_) {
        spdlog::debug("Setting analog output voltage 1 to {}", value);
        comm_ok = rtde_io_->setAnalogOutputVoltage(1, value);
    } else if (function == setCurrentAOUT0Index_) {
        spdlog::debug("Setting analog output current 0 to {}", value);
        comm_ok = rtde_io_->setAnalogOutputCurrent(0, value);
    } else if (function == setCurrentAOUT1Index_) {
        spdlog::debug("Setting analog output current 1 to {}", value);
        comm_ok = rtde_io_->setAnalogOutputCurrent(1, value);
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communication error in RTDEInOut::writeFloat64");
        return asynError;
    }
}

asynStatus RTDEInOut::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == reconnectIndex_) {
        comm_ok = try_connect();
    } else if (function == disconnectIndex_) {
        rtde_io_->disconnect();
    }

    if (rtde_io_ == nullptr) {
        spdlog::error("RTDE IO interface not initialized");
        return asynError;
    }

    if (function == setStandardDOUT0Index_) {
        spdlog::debug("Setting standard digital output 0 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(0, static_cast<bool>(value));
    } else if (function == setStandardDOUT1Index_) {
        spdlog::debug("Setting standard digital output 1 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(1, static_cast<bool>(value));
    } else if (function == setStandardDOUT2Index_) {
        spdlog::debug("Setting standard digital output 2 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(2, static_cast<bool>(value));
    } else if (function == setStandardDOUT3Index_) {
        spdlog::debug("Setting standard digital output 3 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(3, static_cast<bool>(value));
    } else if (function == setStandardDOUT4Index_) {
        spdlog::debug("Setting standard digital output 4 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(4, static_cast<bool>(value));
    } else if (function == setStandardDOUT5Index_) {
        spdlog::debug("Setting standard digital output 5 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(5, static_cast<bool>(value));
    } else if (function == setStandardDOUT6Index_) {
        spdlog::debug("Setting standard digital output 6 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(6, static_cast<bool>(value));
    } else if (function == setStandardDOUT7Index_) {
        spdlog::debug("Setting standard digital output 7 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(7, static_cast<bool>(value));
    }

    else if (function == setConfigDOUT0Index_) {
        spdlog::debug("Setting configurable digital output 0 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(0, static_cast<bool>(value));
    } else if (function == setConfigDOUT1Index_) {
        spdlog::debug("Setting configurable digital output 1 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(1, static_cast<bool>(value));
    } else if (function == setConfigDOUT2Index_) {
        spdlog::debug("Setting configurable digital output 2 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(2, static_cast<bool>(value));
    } else if (function == setConfigDOUT3Index_) {
        spdlog::debug("Setting configurable digital output 3 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(3, static_cast<bool>(value));
    } else if (function == setConfigDOUT4Index_) {
        spdlog::debug("Setting configurable digital output 4 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(4, static_cast<bool>(value));
    } else if (function == setConfigDOUT5Index_) {
        spdlog::debug("Setting configurable digital output 5 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(5, static_cast<bool>(value));
    } else if (function == setConfigDOUT6Index_) {
        spdlog::debug("Setting configurable digital output 6 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(6, static_cast<bool>(value));
    } else if (function == setConfigDOUT7Index_) {
        spdlog::debug("Setting configurable digital output 7 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(7, static_cast<bool>(value));
    }

    else if (function == setToolDOUT0Index_) {
        spdlog::debug("Setting tool digital output 0 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setToolDigitalOut(0, static_cast<bool>(value));
    } else if (function == setToolDOUT1Index_) {
        spdlog::debug("Setting tool digital output 1 {}", value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setToolDigitalOut(1, static_cast<bool>(value));
    }

    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communincation error in RTDEInOut::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int RTDEInOutConfig(const char *asyn_port_name, const char *robot_ip) {
    RTDEInOut *pRTDEInOut = new RTDEInOut(asyn_port_name, robot_ip);
    pRTDEInOut = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"RTDEInOutConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) { RTDEInOutConfig(args[0].sval, args[1].sval); }

void RTDEInOutRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEInOutRegister);
}
