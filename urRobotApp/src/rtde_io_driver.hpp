#ifndef _RTDE_CONTROL_DRIVER_HPP_
#define _RTDE_CONTROL_DRIVER_HPP_

#include "ur_rtde/rtde_io_interface.h"
#include <asynPortDriver.h>

// RTDE IO Interface
static constexpr char SPEED_SLIDER_STRING[] = "SPEED_SLIDER";
static constexpr char SET_STANDARD_DOUT0_STRING[] = "SET_STANDARD_DIGITAL_OUT0";
static constexpr char SET_STANDARD_DOUT1_STRING[] = "SET_STANDARD_DIGITAL_OUT1";
static constexpr char SET_STANDARD_DOUT2_STRING[] = "SET_STANDARD_DIGITAL_OUT2";
static constexpr char SET_STANDARD_DOUT3_STRING[] = "SET_STANDARD_DIGITAL_OUT3";
static constexpr char SET_STANDARD_DOUT4_STRING[] = "SET_STANDARD_DIGITAL_OUT4";
static constexpr char SET_STANDARD_DOUT5_STRING[] = "SET_STANDARD_DIGITAL_OUT5";
static constexpr char SET_STANDARD_DOUT6_STRING[] = "SET_STANDARD_DIGITAL_OUT6";
static constexpr char SET_STANDARD_DOUT7_STRING[] = "SET_STANDARD_DIGITAL_OUT7";
static constexpr char SET_CONFIG_DOUT0_STRING[] = "SET_CONFIG_DIGITAL_OUT0";
static constexpr char SET_CONFIG_DOUT1_STRING[] = "SET_CONFIG_DIGITAL_OUT1";
static constexpr char SET_CONFIG_DOUT2_STRING[] = "SET_CONFIG_DIGITAL_OUT2";
static constexpr char SET_CONFIG_DOUT3_STRING[] = "SET_CONFIG_DIGITAL_OUT3";
static constexpr char SET_CONFIG_DOUT4_STRING[] = "SET_CONFIG_DIGITAL_OUT4";
static constexpr char SET_CONFIG_DOUT5_STRING[] = "SET_CONFIG_DIGITAL_OUT5";
static constexpr char SET_CONFIG_DOUT6_STRING[] = "SET_CONFIG_DIGITAL_OUT6";
static constexpr char SET_CONFIG_DOUT7_STRING[] = "SET_CONFIG_DIGITAL_OUT7";
static constexpr char SET_TOOL_DOUT0_STRING[] = "SET_TOOL_DIGITAL_OUT0";
static constexpr char SET_TOOL_DOUT1_STRING[] = "SET_TOOL_DIGITAL_OUT1";
static constexpr char SET_VOLTAGE_AOUT0_STRING[] = "SET_VOLTAGE_ANALOG_OUT0";
static constexpr char SET_VOLTAGE_AOUT1_STRING[] = "SET_VOLTAGE_ANALOG_OUT1";
static constexpr char SET_CURRENT_AOUT0_STRING[] = "SET_CURRENT_ANALOG_OUT0";
static constexpr char SET_CURRENT_AOUT1_STRING[] = "SET_CURRENT_ANALOG_OUT1";
static constexpr char RECONNECT_STRING[] = "RECONNECT";
static constexpr char DISCONNECT_STRING[] = "DISCONNECT";

static constexpr int NUM_JOINTS = 6;
static constexpr int MAX_CONTROLLERS = 1;
static constexpr double DEFAULT_CONTROLLER_TIMEOUT = 1.0;

class RTDEInOut : public asynPortDriver {
  public:
    RTDEInOut(const char *asyn_port_name, const char *robot_port_name, double poll_period);
    virtual void poll(void);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

  private:
    std::unique_ptr<ur_rtde::RTDEIOInterface> rtde_io_;
    const std::string robot_ip_ = "";
    const double poll_period_ = 0.0;
    bool try_connect();

  protected:
    asynUser *pasynUserURRobot_;

    // rtde_io
    int speedSliderIndex_;
    int setStandardDOUT0Index_;
    int setStandardDOUT1Index_;
    int setStandardDOUT2Index_;
    int setStandardDOUT3Index_;
    int setStandardDOUT4Index_;
    int setStandardDOUT5Index_;
    int setStandardDOUT6Index_;
    int setStandardDOUT7Index_;
    int setConfigDOUT0Index_;
    int setConfigDOUT1Index_;
    int setConfigDOUT2Index_;
    int setConfigDOUT3Index_;
    int setConfigDOUT4Index_;
    int setConfigDOUT5Index_;
    int setConfigDOUT6Index_;
    int setConfigDOUT7Index_;
    int setToolDOUT0Index_;
    int setToolDOUT1Index_;
    int setVoltageAOUT0Index_;
    int setVoltageAOUT1Index_;
    int setCurrentAOUT0Index_;
    int setCurrentAOUT1Index_;
    int reconnectIndex_;
    int disconnectIndex_;

};

#endif
