#pragma once
#include "ur_rtde/dashboard_client.h"
#include <asynPortDriver.h>

constexpr int MAX_CONTROLLERS = 1;

class URDashboard : public asynPortDriver {
  public:
    URDashboard(const char* asyn_port_name, const char* robot_port_name, double poll_period);
    void poll(void);
    asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value) override;
    asynStatus writeOctet(asynUser* pasynUser, const char* value, size_t maxChars, size_t* nActual) override;

  private:
    std::unique_ptr<ur_rtde::DashboardClient> ur_dashboard_;
    double poll_period_;
    bool try_connect();

  protected:
    int isConnectedIndex_;
    int closePopupIndex_;
    int popupIndex_;
    int loadURPIndex_;
    int playIndex_;
    int stopIndex_;
    int pauseIndex_;
    int connectIndex_;
    int disconnectIndex_;
    int shutdownIndex_;
    int isRunningIndex_;
    int closeSafetyPopupIndex_;
    int powerOnIndex_;
    int powerOffIndex_;
    int brakeReleaseIndex_;
    int unlockProtectiveStopIndex_;
    int restartSafetyIndex_;
    int polyscopeVersionIndex_;
    int serialNumberIndex_;
    int robotModeIndex_;
    int programStateIndex_;
    int robotModelIndex_;
    int loadedProgramIndex_;
    int safetyStatusIndex_;
    int isProgramSavedIndex_;
    int isInRemoteControlIndex_;
};
