#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "rtde_control_driver.hpp"
#include "spdlog/spdlog.h"

static void poll_thread_C(void *pPvt) {
    RTDEControl *pRTDEControl = (RTDEControl *)pPvt;
    pRTDEControl->poll();
}

// Wraps class construction/connection to fail gracefully.
// Unlike the DashboardClient class, construction of the object tries
// connecting automatically. If connection fails (IP is wrong) the RTDE
// constructor throws and error, so subsequent calls like
// rtde_control_->isConnected() will segfault since the rtde_recieve_ object
// does not point to anything. This is why we have the `this->connected` variable

bool RTDEControl::try_connect_io() {
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

bool RTDEControl::try_connect_control() {
    bool connected = false;

    if (rtde_control_ == nullptr) {
        try {
            rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
            if (rtde_control_->isConnected()) {
                spdlog::info("Connected to UR RTDE Control interface");
                connected = true;
            }
        } catch (const std::exception &e) {
            spdlog::error("Failed to connected to UR RTDE Control interface\n{}", e.what());
        }
    } else {
        if (not rtde_control_->isConnected()) {
            rtde_control_->reconnect();
            spdlog::info("Reconnecting to UR RTDE Control interface");
        }
    }
    return connected;
}

static constexpr int NUM_JOINTS = 6;

RTDEControl::RTDEControl(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask |
                         asynFloat64ArrayMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask |
                         asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      rtde_control_(nullptr), rtde_io_(nullptr), robot_ip_(robot_ip) {

    // RTDE Control
    createParam(DISCONNECT_STRING, asynParamInt32, &disconnectIndex_);
    createParam(RECONNECT_STRING, asynParamInt32, &reconnectIndex_);
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);

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

    // TODO: make log level an arg to the constructor?
    spdlog::set_level(spdlog::level::debug); // Set global log level to debug

    try_connect_io();
    try_connect_control();

    epicsThreadCreate("RTDEControlPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)poll_thread_C, this);
}

void RTDEControl::poll() {
    while (true) {
        lock();

        if (rtde_control_ != nullptr) {
            if (rtde_control_->isConnected()) {
                setIntegerParam(isConnectedIndex_, 1);
            } else {
                setIntegerParam(isConnectedIndex_, 0);
            }
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus RTDEControl::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    if (rtde_io_ == nullptr) {
        spdlog::error("RTDE IO interface not initialized");
        return asynError;
    }

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
        spdlog::error("RTDE communication error in RTDEControl::writeFloat64");
        return asynError;
    }
}

asynStatus RTDEControl::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool rtde_ok = true;

    if (function == reconnectIndex_) {
        bool res = false;
        res = try_connect_control();
        res = try_connect_io();
        if (not res) {
            return asynError;
        }
    } else if (function == disconnectIndex_) {
        rtde_control_->disconnect();
        rtde_io_->disconnect();
    }
    
    // only checking io since rtde_control_ not used later in this func
    if (rtde_io_ == nullptr) {
        spdlog::error("RTDE IO interface not initialized");
        return asynError;
    }
    
    // if control is not connected, neither is I/O
    if (not rtde_control_->isConnected()) {
        spdlog::warn("RTDE Control + I/O interfaces not connected");
        return asynError;
    }

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

    callParamCallbacks();
    if (rtde_ok) {
        return asynSuccess;
    } else {
        spdlog::error("RTDE communincation error in RTDEControl::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int RTDEControlConfig(const char *asyn_port_name, const char *robot_ip) {
    RTDEControl *pRTDEControl = new RTDEControl(asyn_port_name, robot_ip);
    pRTDEControl = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"RTDEControlConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    RTDEControlConfig(args[0].sval, args[1].sval);
}

void RTDEControlRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(RTDEControlRegister);
}
