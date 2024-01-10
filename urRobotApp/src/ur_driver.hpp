#include <asynPortDriver.h>
#include "ur_rtde/rtde_receive_interface.h"

static const char *driver_name = "urRobot";

constexpr int MAX_CONTROLLERS = 1;
constexpr double DEFAULT_POLL_TIME = 0.20; // 5Hz?

#define DEFAULT_CONTROLLER_TIMEOUT 1.0

class URRobot: public asynPortDriver {
    public:
	URRobot(const char *asyn_port_name, const char* robot_port_name);
	virtual void main_loop(void);

    private:
	double poll_time;
	int count;
	ur_rtde::RTDEReceiveInterface rtde_receive_;
	std::vector<double> pose_;

    protected:
	asynUser* pasynUserURRobot_;
};
