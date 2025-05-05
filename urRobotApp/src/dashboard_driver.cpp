#include <epicsExport.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>

#include "dashboard_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"
#include "ur_rtde/dashboard_client.h"

static void poll_thread_C(void *pPvt) {
    URDashboard *pURDashboard = (URDashboard *)pPvt;
    pURDashboard->poll();
}

// wraps the ur_dashboard_->connect() function to fail gracefully
// TODO: check nullptr?
bool URDashboard::try_connect() {
    bool connected = false;
    try {
        ur_dashboard_->connect();
        if (ur_dashboard_->isConnected()) {
            spdlog::info("Connected to UR Dashboard server");
            connected = true;
        }
    } catch (const std::exception &e) {
        spdlog::error(e.what());
    }
    return connected;
}

URDashboard::URDashboard(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      ur_dashboard_(std::make_unique<ur_rtde::DashboardClient>(robot_ip)), poll_time_(DEFAULT_POLL_TIME) {

    // create asyn parameters
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);
    createParam(LOAD_URP_STRING, asynParamOctet, &loadURPIndex_);
    createParam(PLAY_STRING, asynParamInt32, &playIndex_);
    createParam(STOP_STRING, asynParamInt32, &stopIndex_);
    createParam(PAUSE_STRING, asynParamInt32, &pauseIndex_);
    createParam(CONNECT_STRING, asynParamInt32, &connectIndex_);
    createParam(DISCONNECT_STRING, asynParamInt32, &disconnectIndex_);
    createParam(SHUTDOWN_STRING, asynParamInt32, &shutdownIndex_);
    createParam(IS_RUNNING_STRING, asynParamInt32, &isRunningIndex_);
    createParam(CLOSE_POPUP_STRING, asynParamInt32, &closePopupIndex_);
    createParam(POPUP_STRING, asynParamOctet, &popupIndex_);
    createParam(CLOSE_SAFETY_POPUP_STRING, asynParamInt32, &closeSafetyPopupIndex_);
    createParam(POWER_ON_STRING, asynParamInt32, &powerOnIndex_);
    createParam(POWER_OFF_STRING, asynParamInt32, &powerOffIndex_);
    createParam(BRAKE_RELEASE_STRING, asynParamInt32, &brakeReleaseIndex_);
    createParam(UNLOCK_PROTECTIVE_STOP_STRING, asynParamInt32, &unlockProtectiveStopIndex_);
    createParam(RESTART_SAFETY_STRING, asynParamInt32, &restartSafetyIndex_);
    createParam(POLYSCOPE_VERSION_STRING, asynParamOctet, &polyscopeVersionIndex_);
    createParam(SERIAL_NUMBER_STRING, asynParamOctet, &serialNumberIndex_);
    createParam(ROBOT_MODE_STRING, asynParamOctet, &robotModeIndex_);
    createParam(PROGRAM_STATE_STRING, asynParamOctet, &programStateIndex_);
    createParam(ROBOT_MODEL_STRING, asynParamOctet, &robotModelIndex_);
    createParam(LOADED_PROGRAM_STRING, asynParamOctet, &loadedProgramIndex_);
    createParam(SAFETY_STATUS, asynParamOctet, &safetyStatusIndex_);
    createParam(IS_PROGRAM_SAVED, asynParamInt32, &isProgramSavedIndex_);
    createParam(IS_IN_REMOTE_CONTROL, asynParamInt32, &isInRemoteControlIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    bool connected = this->try_connect();
    if (connected) {
        setIntegerParam(isConnectedIndex_, 1);
        setStringParam(polyscopeVersionIndex_, ur_dashboard_->polyscopeVersion());
        setStringParam(serialNumberIndex_, ur_dashboard_->getSerialNumber());
        setStringParam(robotModelIndex_, ur_dashboard_->getRobotModel());
    } else {
        setIntegerParam(isConnectedIndex_, 0);
    }

    epicsThreadCreate("URDashboardPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void URDashboard::poll() {
    while (true) {
        lock();

        if (ur_dashboard_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);
            setIntegerParam(isRunningIndex_, ur_dashboard_->running());
            setStringParam(programStateIndex_, ur_dashboard_->programState());
            setStringParam(robotModeIndex_, ur_dashboard_->robotmode());
            setStringParam(loadedProgramIndex_, ur_dashboard_->getLoadedProgram());
            setStringParam(safetyStatusIndex_, ur_dashboard_->safetystatus());
            setIntegerParam(isProgramSavedIndex_, ur_dashboard_->isProgramSaved());
            setIntegerParam(isInRemoteControlIndex_, ur_dashboard_->isInRemoteControl());
        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_time_);
    }
}

asynStatus URDashboard::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    if (function == playIndex_) {
        spdlog::debug("Playing loaded program");
        try {
            ur_dashboard_->play();
        } catch (const std::exception &e) {
            spdlog::error("{}", e.what());
            comm_ok = false;
        }
    } else if (function == stopIndex_) {
        spdlog::debug("Stopping current program");
        try {
            ur_dashboard_->stop();
        } catch (const std::exception &e) {
            spdlog::error("{}", e.what());
            comm_ok = false;
        }
    } else if (function == pauseIndex_) {
        spdlog::debug("Pausing current program");
        try {
            ur_dashboard_->pause();
        } catch (const std::exception &e) {
            spdlog::error("{}", e.what());
            comm_ok = false;
        }
    } else if (function == connectIndex_) {
        spdlog::debug("Connecting to dashboard server");
        try_connect();
    } else if (function == disconnectIndex_) {
        spdlog::debug("Disconnecting from dashboard server");
        ur_dashboard_->disconnect();
    } else if (function == shutdownIndex_) {
        spdlog::debug("Shutting down robot and controller");
        ur_dashboard_->shutdown();
    } else if (function == closePopupIndex_) {
        spdlog::debug("Closing popup");
        ur_dashboard_->closePopup();
    } else if (function == closeSafetyPopupIndex_) {
        spdlog::debug("Closing safety popup");
        ur_dashboard_->closeSafetyPopup();
    } else if (function == powerOnIndex_) {
        spdlog::debug("Powering on robot");
        ur_dashboard_->powerOn();
    } else if (function == powerOffIndex_) {
        spdlog::debug("Powering off robot");
        ur_dashboard_->powerOff();
    } else if (function == brakeReleaseIndex_) {
        spdlog::debug("Releasing brakes");
        ur_dashboard_->brakeRelease();
    } else if (function == unlockProtectiveStopIndex_) {
        spdlog::debug("Unlocking protective stop");
        ur_dashboard_->unlockProtectiveStop();
    } else if (function == restartSafetyIndex_) {
        spdlog::debug("Restarting safety configuration");
        ur_dashboard_->restartSafety();
    }

    callParamCallbacks();
    return comm_ok ? asynSuccess : asynError;
}

asynStatus URDashboard::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual) {
    // TODO: Check this. Seems to work but gives weird warning
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    if (function == popupIndex_) {
        std::stringstream ss;
        spdlog::debug("Popup text: {}", value);
        ur_dashboard_->popup(value);
    } else if (function == loadURPIndex_) {
        std::stringstream ss;
        spdlog::debug("Loading program {}", value);
        try {
            ur_dashboard_->loadURP(value);
        } catch (const std::exception &e) {
            spdlog::error("{}", e.what());
            status = asynError;
        }
    }

    callParamCallbacks();
    return status;
}
// register function for iocsh
extern "C" int URDashboardConfig(const char *asyn_port_name, const char *robot_ip) {
    URDashboard *pURDashboard = new URDashboard(asyn_port_name, robot_ip);
    (void)pURDashboard;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URDashboardConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) { URDashboardConfig(args[0].sval, args[1].sval); }

void URDashboardRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(URDashboardRegister);
}
