#include <iocsh.h>
#include <epicsExport.h>
#include <epicsString.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>
#include "ur_driver.hpp"

static void main_loop_thread_C(void *pPvt) {
    URRobot *pURRobot = (URRobot*)pPvt;
    pURRobot->main_loop();
}

URRobot::URRobot(const char *asyn_port_name, const char* robot_port_name) : asynPortDriver(asyn_port_name, MAX_CONTROLLERS,
		asynInt32Mask | asynFloat64Mask | asynDrvUserMask | asynOctetMask,
		asynInt32Mask | asynFloat64Mask | asynOctetMask,
		ASYN_MULTIDEVICE | ASYN_CANBLOCK, 1, /* ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=1, autoConnect=1 */
		0, 0), poll_time(DEFAULT_POLL_TIME)
{

    // static const char *function_name = "URRobot";
    this->count = 0;

    epicsThreadCreate(
	"UrRobotMainLoop",
	epicsThreadPriorityLow,
	epicsThreadGetStackSize(epicsThreadStackMedium),
	(EPICSTHREADFUNC)main_loop_thread_C,
	this
    );

    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Starting UR robot driver\n");
    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Poll time = %lf\n", poll_time);

}

void URRobot::main_loop(void) {
    while (true) {
	lock();
	this->count += 1;
	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "count = %d\n", count);
	callParamCallbacks();
	unlock();
	epicsThreadSleep(poll_time);
    }
}


extern "C" int URRobotConfig(const char *asyn_port_name, const char* robot_port_name) {
    URRobot *pURRobot = new URRobot(asyn_port_name, robot_port_name);
    pURRobot = NULL;
    return(asynSuccess);
}

static const iocshArg urRobotArg0 = {"Asyn port name", iocshArgString};
static const iocshArg urRobotArg1 = {"Robot port name", iocshArgString};
static const iocshArg * const urRobotArgs[2] = {&urRobotArg0, &urRobotArg1};
static const iocshFuncDef urRobotFuncDef = {"URRobotConfig", 2, urRobotArgs};

static void urRobotCallFunc(const iocshArgBuf *args) {
    URRobotConfig(args[0].sval, args[1].sval);
}

void URRobotRegister(void) {
    iocshRegister(&urRobotFuncDef, urRobotCallFunc);
}

extern "C" {
    epicsExportRegistrar(URRobotRegister);
}
