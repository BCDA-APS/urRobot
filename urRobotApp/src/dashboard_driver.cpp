#include "dashboard_driver.hpp"
#include "ur_rtde/dashboard_client.h"

#include <asynOctetSyncIO.h>
#include <cstring>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>

static constexpr int BUFF_SIZE = 100;

static void main_loop_thread_C(void *pPvt) {
    URRobotDashboard *pURRobotDashboard = (URRobotDashboard *)pPvt;
    pURRobotDashboard->main_loop();
}

URRobotDashboard::URRobotDashboard(const char *asyn_port_name,
                                   const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask |
                         asynOctetMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask |
                         asynInt32ArrayMask,
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
    ur_dashboard_->connect();
    if (ur_dashboard_->isConnected()) {
        std::cout << "Connected to dashboard server" << std::endl;
        setIntegerParam(isConnectedIndex_, 1);
    } else {
        setIntegerParam(isConnectedIndex_, 0);
        // TODO: maybe shouldn't throw here
        throw std::runtime_error("Failed to connect to UR dashboard server");
    }

    // set parameters that are constant
    setStringParam(polyscopeVersionIndex_, ur_dashboard_->polyscopeVersion());
    setStringParam(serialNumberIndex_, ur_dashboard_->getSerialNumber());
    setStringParam(robotModelIndex_, ur_dashboard_->getRobotModel());

    std::cout << "starting main loop..." << std::endl;
    epicsThreadCreate("UrRobotMainLoop", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      (EPICSTHREADFUNC)main_loop_thread_C, this);
}

void URRobotDashboard::main_loop() {
    int trigger = 0;
    char buffer[BUFF_SIZE] = {};
    while (true) {
        lock();

        // TODO: should only need to getParam until non-zero or strlen > 0.
        if (ur_dashboard_->isConnected()) {
            setIntegerParam(isConnectedIndex_, 1);

            // check if program is running
            if (ur_dashboard_->running()) {
                setIntegerParam(isRunningIndex_, 1);
            } else {
                setIntegerParam(isRunningIndex_, 0);
            }
            
            setStringParam(programStateIndex_, ur_dashboard_->programState());
            setStringParam(robotModeIndex_, ur_dashboard_->robotmode());
            setStringParam(loadedProgramIndex_, ur_dashboard_->getLoadedProgram());
            setStringParam(safetyStatusIndex_, ur_dashboard_->safetystatus());
            setIntegerParam(isProgramSavedIndex_, ur_dashboard_->isProgramSaved());
            setIntegerParam(isInRemoteControlIndex_, ur_dashboard_->isInRemoteControl());

            // -----------------------------------------

            // Load program
            getStringParam(loadURPIndex_, BUFF_SIZE, buffer);
            if (strlen(buffer) > 0) {
                std::cout << "loading program " << buffer << ".urp"
                          << std::endl;
                setStringParam(loadURPIndex_, "");
                // ur_dashboard_->loadURP(buffer);
            }
            memset(buffer, '\0', sizeof(buffer));

            // Play loaded program
            getIntegerParam(playIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Playing loaded program" << std::endl;
                setIntegerParam(playIndex_, 0);
                // ur_dashboard_->play();
            }

            // Stop program
            getIntegerParam(stopIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Stopping program" << std::endl;
                setIntegerParam(stopIndex_, 0);
                // ur_dashboard_->stop();
            }

            // Pause program
            getIntegerParam(pauseIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Pausing program" << std::endl;
                setIntegerParam(pauseIndex_, 0);
                // ur_dashboard_->pause();
            }

            // Quit
            getIntegerParam(quitIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Closing connection to dashboard server"
                          << std::endl;
                setIntegerParam(quitIndex_, 0);
                // ur_dashboard_->quit();
            }

            // Shutdown
            getIntegerParam(shutdownIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Shutting down robot and controller" << std::endl;
                setIntegerParam(shutdownIndex_, 0);
                // ur_dashboard_->shutdown();
            }

            // Popup message
            getStringParam(popupIndex_, BUFF_SIZE, buffer);
            if (strlen(buffer) > 0) {
                std::cout << "Popup text = " << buffer << std::endl;
                setStringParam(popupIndex_, "");
                ur_dashboard_->popup(buffer);
            }
            memset(buffer, '\0', sizeof(buffer));

            // Close popup
            getIntegerParam(closePopupIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Closing popup" << std::endl;
                setIntegerParam(closePopupIndex_, 0);
                ur_dashboard_->closePopup();
            }

            // Close safety popup
            getIntegerParam(closeSafetyPopupIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Closings safety popup" << std::endl;
                setIntegerParam(closeSafetyPopupIndex_, 0);
                // ur_dashboard_->closeSafetyPopup();
            }

            // Power on
            getIntegerParam(powerOnIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Powering robot on" << std::endl;
                setIntegerParam(powerOnIndex_, 0);
                // ur_dashboard_->powerOn();
            }

            // Power off
            getIntegerParam(powerOffIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Powering robot off" << std::endl;
                setIntegerParam(powerOffIndex_, 0);
                // ur_dashboard_->powerOff();
            }

            // Brake release
            getIntegerParam(brakeReleaseIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Releasing brakes" << std::endl;
                setIntegerParam(brakeReleaseIndex_, 0);
                // ur_dashboard_->brakeRelease();
            }

            // Unlock protective stop
            getIntegerParam(unlockProtectiveStopIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Unlocking protective stop" << std::endl;
                setIntegerParam(unlockProtectiveStopIndex_, 0);
                // ur_dashboard_->unlockProtectiveStop();
            }

            // Restart safety
            getIntegerParam(restartSafetyIndex_, &trigger);
            if (trigger != 0) {
                std::cout << "Restarting safety" << std::endl;
                setIntegerParam(restartSafetyIndex_, 0);
                // ur_dashboard_->restartSafety();
            }

        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_time_);
    }
}

// register function for iocsh
extern "C" int URRobotDashboardConfig(const char *asyn_port_name,
                                      const char *robot_ip) {
    URRobotDashboard *pURRobotDashboard =
        new URRobotDashboard(asyn_port_name, robot_ip);
    pURRobotDashboard = NULL;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URRobotDashboardConfig", 2,
                                            urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    URRobotDashboardConfig(args[0].sval, args[1].sval);
}

void URRobotDashboardRegister(void) {
    iocshRegister(&urRobotFuncDef, urRobotCallFunc);
}

extern "C" {
epicsExportRegistrar(URRobotDashboardRegister);
}
