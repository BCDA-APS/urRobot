#pragma once
#include <asynPortDriver.h>
#include "ur_rtde/rtde_io_interface.h"


class RTDEInOut : public asynPortDriver {
  public:
    RTDEInOut(const char *asyn_port_name, const char *robot_port_name, double poll_period);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  private:
    std::unique_ptr<ur_rtde::RTDEIOInterface> rtde_io_;
    const std::string robot_ip_ = "";
    bool try_connect();

  protected:
    asynUser *pasynUserURRobot_;

    // rtde_io
    int speedSliderIndex_;
    int setStandardDOUTIndex_;
    int setConfigDOUTIndex_;
    int setToolDOUTIndex_;
    int setVoltageAOUTIndex_;
    int setCurrentAOUTIndex_;
};
