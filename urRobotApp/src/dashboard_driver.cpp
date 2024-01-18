#include "dashboard_driver.hpp"
#include "ur_rtde/dashboard_client.h"

#include <asynOctetSyncIO.h>
#include <cstring>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <exception>
#include <iocsh.h>
#include <iostream>
#include <unistd.h>


static void main_loop_thread_C(void *pPvt) {
    URRobotDashboard *pURRobotDashboard = (URRobotDashboard *)pPvt;
    pURRobotDashboard->poll();
}

// TODO: use better logging than std::cout <<

URRobotDashboard::URRobotDashboard(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask |
                         asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
                     0, 0),
      ur_dashboard_(std::make_unique<ur_rtde::DashboardClient>(robot_ip)),
      poll_time_(DEFAULT_POLL_TIME) {

    // create asyn parameters
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);
    createParam(LOAD_URP_STRING, asynParamOctet, &loadURPIndex_);
    createParam(PLAY_STRING, asynParamInt32, &playIndex_);
    createParam(STOP_STRING, asynParamInt32, &stopIndex_);
    createParam(PAUSE_STRING, asynParamInt32, &pauseIndex_);
    createParam(QUIT_STRING, asynParamInt32, &quitIndex_);
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

    // connect to the UR dashboard server
    bool connected = false;
    try {
        ur_dashboard_->connect();
        if (ur_dashboard_->isConnected()) {
            std::cout << "Connected to dashboard server" << std::endl;
            setIntegerParam(isConnectedIndex_, 1);
            connected = true;
        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }
    } catch (const std::exception &e) {
        std::cout << "Caught exception: " << e.what() << std::endl;
    }

    if (connected) {
        // set parameters that are constant
        setStringParam(polyscopeVersionIndex_, ur_dashboard_->polyscopeVersion());
        setStringParam(serialNumberIndex_, ur_dashboard_->getSerialNumber());
        setStringParam(robotModelIndex_, ur_dashboard_->getRobotModel());

        epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)main_loop_thread_C, this);
    }
}

void URRobotDashboard::poll() {
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

asynStatus URRobotDashboard::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;

    if (function == playIndex_) {
        std::cout << "Playing loaded program" << std::endl;
        ur_dashboard_->play();
    } else if (function == stopIndex_) {
        std::cout << "Stopping current program" << std::endl;
        ur_dashboard_->stop();
    } else if (function == pauseIndex_) {
        std::cout << "Pausing current program" << std::endl;
        ur_dashboard_->pause();
    } else if (function == quitIndex_) {
        std::cout << "Disconnecting from dashboard server" << std::endl;
        ur_dashboard_->quit();
    } else if (function == shutdownIndex_) {
        std::cout << "Shutting down robot and controller" << std::endl;
        ur_dashboard_->shutdown();
    } else if (function == closePopupIndex_) {
        std::cout << "Closing popup" << std::endl;
        ur_dashboard_->closePopup();
    } else if (function == closeSafetyPopupIndex_) {
        std::cout << "Closing safety popup" << std::endl;
        ur_dashboard_->closeSafetyPopup();
    } else if (function == powerOnIndex_) {
        std::cout << "Powering on robot" << std::endl;
        ur_dashboard_->powerOn();
    } else if (function == powerOffIndex_) {
        std::cout << "Powering off robot" << std::endl;
        ur_dashboard_->powerOff();
    } else if (function == brakeReleaseIndex_) {
        std::cout << "Releasing brakes" << std::endl;
        ur_dashboard_->brakeRelease();
    } else if (function == unlockProtectiveStopIndex_) {
        std::cout << "Unlocking protective stop" << std::endl;
        ur_dashboard_->unlockProtectiveStop();
    } else if (function == restartSafetyIndex_) {
        std::cout << "Restarting safety configuration" << std::endl;
        ur_dashboard_->restartSafety();
    }

    callParamCallbacks();
    return asynSuccess;
}

asynStatus URRobotDashboard::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars,
                                        size_t *nActual) {
    // TODO: Check this. Seems to work but gives weird warning
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    if (function == popupIndex_) {
        std::cout << "Popup text: " << value << std::endl;
        ur_dashboard_->popup(value);
    } else if (function == loadURPIndex_) {
        std::cout << "Loading program " << value << std::endl;
        try {
            ur_dashboard_->loadURP(value);
        } catch (const std::exception &e) {
            std::cout << "Caught exception: " << e.what() << std::endl;
            status = asynError;
        }
    }

    callParamCallbacks();
    return status;
}

// register function for iocsh
extern "C" int URRobotDashboardConfig(const char *asyn_port_name, const char *robot_ip) {
    URRobotDashboard *pURRobotDashboard = new URRobotDashboard(asyn_port_name, robot_ip);
    pURRobotDashboard = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URRobotDashboardConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    URRobotDashboardConfig(args[0].sval, args[1].sval);
}

void URRobotDashboardRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(URRobotDashboardRegister);
}
