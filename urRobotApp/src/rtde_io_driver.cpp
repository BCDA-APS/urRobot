#include <epicsExport.h>
#include <iocsh.h>

#include "rtde_io_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

bool RTDEInOut::try_connect() {
    // We assume if there is no error, IO interface is connected
    // because the RTDEIOInterface class has no isConnected() method
    bool connected = false;
    if (!rtde_io_) {
        try {
            rtde_io_ = std::make_unique<ur_rtde::RTDEIOInterface>(robot_ip_);
            spdlog::info("Connected to UR RTDE IO interface");
            connected = true;
        } catch (const std::exception& e) {
            spdlog::error("Failed to connected to UR RTDE I/O interface\n{}", e.what());
        }
    } else {
        rtde_io_->reconnect();
        spdlog::info("Reconnecting to UR RTDE I/O interface");
        connected = true;
    }
    return connected;
}

constexpr int MAX_ADDR = 8;
constexpr int ASYN_INTERFACE_MASK = asynInt32Mask | asynFloat64Mask | asynDrvUserMask;
constexpr int ASYN_INTERRUPT_MASK = asynInt32Mask | asynFloat64Mask;

RTDEInOut::RTDEInOut(const char* asyn_port_name, const char* robot_ip, double poll_period)
    : asynPortDriver(asyn_port_name, MAX_ADDR, ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, 0, 0),
      rtde_io_(nullptr), robot_ip_(robot_ip) {

    // RTDE IO
    createParam("SPEED_SLIDER", asynParamFloat64, &speedSliderIndex_);
    createParam("SET_STANDARD_DIGITAL_OUT", asynParamInt32, &setStandardDOUTIndex_);
    createParam("SET_CONFIG_DIGITAL_OUT", asynParamInt32, &setConfigDOUTIndex_);
    createParam("SET_TOOL_DIGITAL_OUT", asynParamInt32, &setToolDOUTIndex_);
    createParam("SET_VOLTAGE_ANALOG_OUT", asynParamFloat64, &setVoltageAOUTIndex_);
    createParam("SET_CURRENT_ANALOG_OUT", asynParamFloat64, &setCurrentAOUTIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    try_connect();
}

asynStatus RTDEInOut::writeFloat64(asynUser* pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (rtde_io_ == nullptr) {
        spdlog::error("RTDE IO interface not initialized");
        comm_ok = false;
        return asynError;
    }

    int addr = 0;
    getAddress(pasynUser, &addr);

    if (function == speedSliderIndex_) {
        if ((static_cast<int>(value * 1000) % 10) == 0) {
            // so we don't print too many values
            spdlog::debug("Setting speed slider to {:.2f}", value);
        }
        comm_ok = rtde_io_->setSpeedSlider(value);
    } else if (function == setVoltageAOUTIndex_) {
        spdlog::debug("Setting analog output voltage {} to {}", addr, value);
        comm_ok = rtde_io_->setAnalogOutputVoltage(addr, value);
    } else if (function == setCurrentAOUTIndex_) {
        spdlog::debug("Setting analog output current {} to {}", addr, value);
        comm_ok = rtde_io_->setAnalogOutputCurrent(addr, value);
    }

    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communication error in RTDEInOut::writeFloat64");
        return asynError;
    }
}

asynStatus RTDEInOut::writeInt32(asynUser* pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (rtde_io_ == nullptr) {
        spdlog::error("RTDE IO interface not initialized");
        return asynError;
    }

    int addr = 0;
    getAddress(pasynUser, &addr);

    if (function == setStandardDOUTIndex_) {
        spdlog::debug("Setting standard digital output {} {}", addr, value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setStandardDigitalOut(addr, static_cast<bool>(value));
    } else if (function == setConfigDOUTIndex_) {
        spdlog::debug("Setting configurable digital output {} {}", addr, value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setConfigurableDigitalOut(addr, static_cast<bool>(value));
    } else if (function == setToolDOUTIndex_) {
        spdlog::debug("Setting tool digital output {} {}", addr, value == 1 ? "high" : "low");
        comm_ok = rtde_io_->setToolDigitalOut(addr, static_cast<bool>(value));
    }

    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communication error in RTDEInOut::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int RTDEInOutConfig(const char* asyn_port_name, const char* robot_ip, double poll_period) {
    new RTDEInOut(asyn_port_name, robot_ip, poll_period);
    return asynSuccess;
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg urRobotArg2 = {"Poll period", iocshArgDouble};
static const iocshArg* const urRobotArgs[3] = {&urRobotArg0, &urRobotArg1, &urRobotArg2};
static const iocshFuncDef urRobotFuncDef = {"RTDEInOutConfig", 3, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf* args) {
    RTDEInOutConfig(args[0].sval, args[1].sval, args[2].dval);
}

void RTDEInOutRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEInOutRegister);
}
