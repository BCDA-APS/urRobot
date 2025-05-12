#include <asynOctetSyncIO.h>
#include <cstdio>
#include <cstdlib>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "rtde_control_driver.hpp"
#include "rtde_control_interface.h"
#include "spdlog/spdlog.h"
#include "ur_rtde/dashboard_client.h"

bool URMotorController::try_connect() {
    // RTDE class construction automatically tries connecting.
    // If this function hasn't been called or if connecting fails,
    // the rtde_control_ object will be a nullptr

    bool connected = false;
    bool robot_running = false;
    try {
        auto dash = std::make_unique<ur_rtde::DashboardClient>(robot_ip_);
        dash->connect();
        if (dash->robotmode() == "Robotmode: RUNNING") {
            robot_running = true;
        } else {
            spdlog::error("Unable to connect to UR RTDE Control Interface: "
                          "Ensure robot is on, in normal mode, and brakes released");
            robot_running = false;
            connected = false;
            dash->disconnect();
        }
    } catch (const std::exception &e) {
        spdlog::error("Caught exception: {}", e.what());
        spdlog::error("RTDE Control: Failed to connect to dashboard to check robot mode");
    }

    if (robot_running) {
        if (rtde_control_ == nullptr) {
            try {
                rtde_control_ = std::make_unique<ur_rtde::RTDEControlInterface>(robot_ip_);
                if (rtde_control_ != nullptr) {
                    if (rtde_control_->isConnected()) {
                        spdlog::info("Connected to UR RTDE Control interface");
                        connected = true;

                        // connect to RTDE Receive, only if already connected to RTDE Control
                        rtde_receive_ = std::make_unique<ur_rtde::RTDEReceiveInterface>(robot_ip_);
                        if (rtde_receive_ == nullptr) {
                            // this probably isn't possible
                            throw std::runtime_error(
                                "Failed connecting to receive interface from RTDEControl class");
                        }
                    }
                }
            } catch (const std::exception &e) {
                spdlog::error("Failed to connected to UR RTDE Control interface\n{}", e.what());
                connected = false;
            }
        } else {
            if (not rtde_control_->isConnected()) {
                spdlog::debug("Reconnecting to UR RTDE Control interface");
                rtde_control_->reconnect();
                rtde_receive_->reconnect();
                connected = true;
            }
        }
    }
    return connected;
}



URMotorController::URMotorController(const char *portName, const char *URMotorPortName,
                                           int numAxes, double movingPollPeriod,
                                           double idlePollPeriod)
    : asynMotorController(portName, numAxes, 0,
                          0, // No additional interfaces beyond the base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    int axis;
    URMotorAxis *pAxis;
    static const char *functionName = "URMotorController::URMotorController";

    // // Connect to motor controller
    // status = pasynOctetSyncIO->connect(URMotorPortName, 0, &pasynUserController_, NULL);
    // if (status) {
        // asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to UR robot controller\n",
                  // functionName);
    // }

    // Create URMotorAxis object for each axis
    // if not done here, user must call URMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new URMotorAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

extern "C" int URMotorCreateController(const char *portName, const char *URMotorPortName,
                                          int numAxes, int movingPollPeriod, int idlePollPeriod) {
    URMotorController *pURMotorController = new URMotorController(
        portName, URMotorPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    pURMotorController = NULL;
    return (asynSuccess);
}

void URMotorController::report(FILE *fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "UR Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

URMotorAxis *URMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<URMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

URMotorAxis *URMotorController::getAxis(int axisNo) {
    return static_cast<URMotorAxis *>(asynMotorController::getAxis(axisNo));
}

URMotorAxis::URMotorAxis(URMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "URMotorAxis created with axis index %d\n",
              axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void URMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

asynStatus URMotorAxis::stop(double acceleration) {

    asynStatus asyn_status = asynSuccess;

    callParamCallbacks();
    return asyn_status;
}

asynStatus URMotorAxis::move(double position, int relative, double minVelocity,
                                double maxVelocity, double acceleration) {
    asynStatus asyn_status = asynStatus::asynSuccess;
   
    callParamCallbacks();
    return asyn_status;
}

asynStatus URMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    callParamCallbacks();
    return asyn_status;
}

asynStatus URMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    asynStatus asyn_status = asynSuccess;
    int function = pasynUser->reason;
    URMotorAxis *pAxis;

    pAxis = getAxis(pasynUser);
    if (!pAxis) {
        return asynError;
    }

    pAxis->callParamCallbacks();
    return asyn_status;
}


asynStatus URMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    asynStatus asyn_status = asynSuccess;
    int function = pasynUser->reason;
    URMotorAxis *pAxis;

    pAxis->callParamCallbacks();
    return asyn_status;
}


// ==================
// iosch registration
// ==================

static const iocshArg URMotorCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg URMotorCreateControllerArg1 = {"drvAsynIPPort name", iocshArgString};
static const iocshArg URMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg URMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg URMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const URMotorCreateControllerArgs[] = {
    &URMotorCreateControllerArg0, &URMotorCreateControllerArg1,
    &URMotorCreateControllerArg2, &URMotorCreateControllerArg3,
    &URMotorCreateControllerArg4};
static const iocshFuncDef URMotorCreateControllerDef = {"URMotorCreateController", 5,
                                                           URMotorCreateControllerArgs};

static void URMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    URMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                               args[4].ival);
}

static void URMotorRegister(void) {
    iocshRegister(&URMotorCreateControllerDef, URMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(URMotorRegister);
}
