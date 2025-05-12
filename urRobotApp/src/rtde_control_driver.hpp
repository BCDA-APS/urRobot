#include <memory>

#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"

#include "ur_rtde/rtde_control_interface.h"
#include "ur_rtde/rtde_receive_interface.h"

class epicsShareClass URMotorAxis : public asynMotorAxis {
  public:
    URMotorAxis(class URMotorController *pC, int axisNo);
    void report(FILE *fp, int level);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);

  private:
    URMotorController *pC_;
    int axisIndex_;
    bool moveStarted_ = false;
    int targetPos_;
    int doneTolerance_ = 50; // nanometers

    friend class URMotorController;
};

class epicsShareClass URMotorController : public asynMotorController {
  public:
    /// \brief Create a new URMotorController object
    ///
    /// \param[in] portName             The name of the asyn port that will be created for this
    /// driver
    /// \param[in] URPortName       The name of the drvAsynIPPort that was created previously
    /// \param[in] numAxes              The number of axes that this controller supports
    /// \param[in] movingPollPeriod     The time between polls when any axis is moving
    /// \param[in] idlePollPeriod       The time between polls when no axis is moving
    URMotorController(const char *portName, const char *URMotorController, int numAxes,
                         double movingPollPeriod, double idlePollPeriod);

    /// \brief Returns a pointer to a URMotorAxis object
    /// \param[in] asynUser structure that encodes the axis index number
    /// \returns NULL if the axis number encoded in pasynUser is invalid
    URMotorAxis *getAxis(asynUser *pasynUser);

    /// \brief Returns a pointer to a URMotorAxis object
    /// \param[in] axisNo Axis index number
    /// \returns NULL if the axis number is invalid
    URMotorAxis *getAxis(int axisNo);

    void report(FILE *fp, int level);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

  protected:
    friend class URMotorAxis;

  private:
    const std::string robot_ip_ = "0.0.0.0";
    
    std::unique_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
    std::unique_ptr<ur_rtde::RTDEReceiveInterface> rtde_receive_;
    
    bool try_connect();


};
