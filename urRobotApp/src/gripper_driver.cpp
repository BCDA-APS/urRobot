#include <cstddef>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "dashboard_client.h"
#include "gripper_driver.hpp"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

bool URGripper::try_connect() {
    bool connected = false;
    try {
        ur_dashboard_->connect();
        const std::string robot_mode = ur_dashboard_->robotmode();
        if (robot_mode == "Robotmode: IDLE" or robot_mode == "Robotmode: RUNNING") {
            gripper_->connect();
            if (gripper_->isConnected()) {
                spdlog::info("Connected to gripper");
                connected = true;
            }
        } else {
            spdlog::error("Unable to connect to gripper. Robot must be powered on.\nCurrent robot mode: {}",
                          robot_mode);
            connected = false;
            robot_on_ = false;
        }

    } catch (const std::exception &e) {
        spdlog::error(e.what());
    }
    return connected;
}

static void poll_thread_C(void *pPvt) {
    URGripper *pGripper = (URGripper *)pPvt;
    pGripper->poll();
}

URGripper::URGripper(const char *asyn_port_name, const char *robot_ip)
    : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
                     asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask |
                         asynFloat64ArrayMask | asynInt32ArrayMask,
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynFloat64ArrayMask |
                         asynInt32ArrayMask,
                     ASYN_MULTIDEVICE | ASYN_CANBLOCK,
                     1, // ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1
                     0, 0),
      gripper_(std::make_unique<ur_rtde::RobotiqGripper>(robot_ip)),
      ur_dashboard_(std::make_unique<ur_rtde::DashboardClient>(robot_ip)), robot_ip_(robot_ip) {

    createParam(CONNECT_STRING, asynParamInt32, &connectIndex_);
    createParam(IS_CONNECTED_STRING, asynParamInt32, &isConnectedIndex_);
    createParam(IS_OPEN_STRING, asynParamInt32, &isOpenIndex_);
    createParam(IS_CLOSED_STRING, asynParamInt32, &isClosedIndex_);

    createParam(IS_STOPPED_INNER_STRING, asynParamInt32, &isStoppedInnerIndex_);
    createParam(IS_STOPPED_OUTER_STRING, asynParamInt32, &isStoppedOuterIndex_);

    createParam(IS_ACTIVE_STRING, asynParamInt32, &isActiveIndex_);
    createParam(ACTIVATE_STRING, asynParamInt32, &activateIndex_);
    createParam(OPEN_STRING, asynParamInt32, &openIndex_);
    createParam(CLOSE_STRING, asynParamInt32, &closeIndex_);
    createParam(SET_SPEED_STRING, asynParamFloat64, &setSpeedIndex_);
    createParam(SET_FORCE_STRING, asynParamFloat64, &setForceIndex_);
    createParam(AUTO_CALIBRATE_STRING, asynParamInt32, &autoCalibrateIndex_);
    createParam(OPEN_POSITION_STRING, asynParamFloat64, &openPositionIndex_);
    createParam(CLOSED_POSITION_STRING, asynParamFloat64, &closedPositionIndex_);
    createParam(CURRENT_POSITION_STRING, asynParamFloat64, &currentPositionIndex_);
    createParam(MOVE_STATUS_STRING, asynParamInt32, &moveStatusIndex_);

    createParam(SET_POSITION_RANGE_STRING, asynParamInt32, &setPositionRangeIndex_);
    createParam(MIN_POSITION_STRING, asynParamInt32, &minPositionIndex_);
    createParam(MAX_POSITION_STRING, asynParamInt32, &maxPositionIndex_);

    createParam(POSITION_UNIT_STRING, asynParamInt32, &positionUnitIndex_);
    createParam(IS_CALIBRATED_STRING, asynParamInt32, &isCalibratedIndex_);

    // gets log level from SPDLOG_LEVEL environment variable
    spdlog::cfg::load_env_levels();

    // attempt to connect to the gripper
    try_connect();

    // create epics polling thread
    epicsThreadCreate("GripperPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);
}

void URGripper::poll() {
    while (true) {
        lock();

        const std::string robot_mode = ur_dashboard_->robotmode();
        if (robot_mode == "Robotmode: IDLE" or robot_mode == "Robotmode: RUNNING") {
            robot_on_ = true;
        } else {
            robot_on_ = false;
            gripper_->disconnect(); // needed to prevent error when powering back on
        }

        if (robot_on_) {
            if (gripper_->isConnected()) {
                setIntegerParam(isConnectedIndex_, 1);
                setIntegerParam(isActiveIndex_, gripper_->isActive());
                setIntegerParam(isOpenIndex_, gripper_->isOpen());
                setIntegerParam(isClosedIndex_, gripper_->isClosed());
                setDoubleParam(currentPositionIndex_, gripper_->getCurrentPosition());
                setDoubleParam(openPositionIndex_, gripper_->getOpenPosition());
                setDoubleParam(closedPositionIndex_, gripper_->getClosedPosition());

                ur_rtde::RobotiqGripper::eObjectStatus move_status = gripper_->objectDetectionStatus();
                setIntegerParam(moveStatusIndex_, move_status);
                switch (move_status) {
                    case ur_rtde::RobotiqGripper::eObjectStatus::STOPPED_INNER_OBJECT:
                        setIntegerParam(isStoppedInnerIndex_, 1);
                        setIntegerParam(isStoppedOuterIndex_, 0);
                        break;
                    case ur_rtde::RobotiqGripper::eObjectStatus::STOPPED_OUTER_OBJECT:
                        setIntegerParam(isStoppedInnerIndex_, 0);
                        setIntegerParam(isStoppedOuterIndex_, 1);
                        break;
                    default:
                        setIntegerParam(isStoppedInnerIndex_, 0);
                        setIntegerParam(isStoppedOuterIndex_, 0);
                        break;
                }
            } else {
                setIntegerParam(isConnectedIndex_, 0);
            }
        } else {
            setIntegerParam(isConnectedIndex_, 0);
        }
        callParamCallbacks();
        unlock();
        epicsThreadSleep(POLL_PERIOD);
    }
}

asynStatus URGripper::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {
    int function = pasynUser->reason;
    bool comm_ok = true;

    // Check that robot is powered on
    if (not robot_on_) {
        spdlog::error("Robot must be powered on to use gripper");
        comm_ok = false;
        goto skip;
    }

    // Check that it's connected before continuing
    if (not gripper_->isConnected()) {
        spdlog::error("Robotiq gripper not connected");
        comm_ok = false;
        goto skip;
    }

    if (function == setSpeedIndex_) {
        spdlog::debug("Setting speed to {}", value);
        gripper_->setSpeed(value);
    } else if (function == setForceIndex_) {
        spdlog::debug("Setting force to {}", value);
        gripper_->setForce(value);
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("Communincation error in Gripper::writeFloat64");
        return asynError;
    }
}

asynStatus URGripper::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    int function = pasynUser->reason;
    bool comm_ok = true;

    // Check that robot is powered on
    if (not robot_on_) {
        spdlog::error("Robot must be powered on to use gripper");
        comm_ok = false;
        goto skip;
    }

    if (function == connectIndex_) {
        spdlog::debug("Connecting to gripper");
        try_connect();
    }

    // Check that gripper connected before continuing
    if (not gripper_->isConnected()) {
        spdlog::error("Robotiq gripper not connected");
        comm_ok = false;
        goto skip;
    }

    if (function == activateIndex_) {
        spdlog::debug("Activating gripper");
        gripper_->activate();
    } else if (function == openIndex_) {
        spdlog::debug("Opening gripper");
        gripper_->open();
    } else if (function == closeIndex_) {
        spdlog::debug("Closing gripper");
        gripper_->close();
    } else if (function == setPositionRangeIndex_) {
        int minpos = 0;
        int maxpos = 0;
        getIntegerParam(minPositionIndex_, &minpos);
        getIntegerParam(maxPositionIndex_, &maxpos);
        gripper_->setNativePositionRange(minpos, maxpos);
        spdlog::debug("setNativePositionRange(min={}, max={})", minpos, maxpos);
    } else if (function == minPositionIndex_) {
        spdlog::debug("setting min={}", value);
        setIntegerParam(minPositionIndex_, value);
    } else if (function == maxPositionIndex_) {
        spdlog::debug("setting max={}", value);
        setIntegerParam(maxPositionIndex_, value);
    } else if (function == positionUnitIndex_) {
        constexpr auto epos = ur_rtde::RobotiqGripper::eMoveParameter::POSITION;
        constexpr auto eunit_device = ur_rtde::RobotiqGripper::eUnit::UNIT_DEVICE;
        constexpr auto eunit_normalized = ur_rtde::RobotiqGripper::eUnit::UNIT_NORMALIZED;
        constexpr auto eunit_percent = ur_rtde::RobotiqGripper::eUnit::UNIT_PERCENT;
        constexpr auto eunit_mm = ur_rtde::RobotiqGripper::eUnit::UNIT_MM;
        switch (value) {
        case 0:
            spdlog::debug("Setting position unit to 'Device' (0,255)");
            gripper_->setUnit(epos, eunit_device);
            break;
        case 1:
            spdlog::debug("Setting position unit to 'Normalized' (0,1.0)");
            gripper_->setUnit(epos, eunit_normalized);
            break;
        case 2:
            spdlog::debug("Setting position unit to 'Percent' (0,100%)");
            gripper_->setUnit(epos, eunit_percent);
            break;
        case 3:
            spdlog::debug("Setting position unit to 'mm' (must define range)");
            gripper_->setUnit(epos, eunit_mm);
            break;
        default:
            spdlog::warn("Unit {} undefined, no action taken.", value);
        }
    } else if (function == autoCalibrateIndex_) {
        spdlog::debug("Auto calibrating open/close positions");
        if (gripper_->isActive()) {
            gripper_->autoCalibrate();
            spdlog::debug("Auto calibration done");
            setIntegerParam(isCalibratedIndex_, 1);
        } else {
            spdlog::error("Activate gripper before autocalibrating");
        }
    }

skip:
    callParamCallbacks();
    if (comm_ok) {
        return asynSuccess;
    } else {
        spdlog::debug("Communincation error in Gripper::writeInt32");
        return asynError;
    }
}

// register function for iocsh
extern "C" int URGripperConfig(const char *asyn_port_name, const char *robot_ip) {
    URGripper *pURGripper = new URGripper(asyn_port_name, robot_ip);
    (void)pURGripper;
    return (asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot IP address", iocshArgString};
static const iocshArg *const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URGripperConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) { URGripperConfig(args[0].sval, args[1].sval); }

void URGripperRegister(void) { iocshRegister(&urRobotFuncDef, urRobotCallFunc); }

extern "C" {
epicsExportRegistrar(URGripperRegister);
}
