/// @file rtde_io_driver.hpp
/// @brief asynPortDriver for UR robot I/O control via the ur_rtde library.
///
/// Provides access to the robot's speed slider, digital outputs (standard,
/// configurable, tool), and analog outputs (voltage, current) through the
/// ur_rtde RTDEIOInterface. Uses multi-device addressing for per-channel access.

#pragma once
#include "ur_rtde/rtde_io_interface.h"
#include <asynPortDriver.h>

/// asynPortDriver that wraps ur_rtde::RTDEIOInterface for I/O control.
///
/// Handles writeFloat64 for speed slider and analog outputs, and writeInt32
/// for digital outputs. The asyn address selects the output channel number.
class RTDEInOut : public asynPortDriver {
  public:
    RTDEInOut(const char* asyn_port_name, const char* robot_port_name, double poll_period);
    asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value) override;
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;

  private:
    std::unique_ptr<ur_rtde::RTDEIOInterface> rtde_io_;
    const std::string robot_ip_ = "";

    /// Connect (or reconnect) to the RTDE I/O interface on the robot.
    bool try_connect();

  protected:
    /// asyn parameter indices — each maps to a named parameter string registered
    /// in the constructor via createParam(). See rtde_io.db for the
    /// corresponding EPICS records.

    int speedSliderIndex_;     ///< speed slider fraction (0.0–1.0)
    int setStandardDOUTIndex_; ///< standard digital output (addr 0-7)
    int setConfigDOUTIndex_;   ///< configurable digital output (addr 0-7)
    int setToolDOUTIndex_;     ///< tool digital output (addr 0-1)
    int setVoltageAOUTIndex_;  ///< analog output voltage (addr 0-1)
    int setCurrentAOUTIndex_;  ///< analog output current (addr 0-1)
};
