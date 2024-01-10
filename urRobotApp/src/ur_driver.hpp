#include <asynPortDriver.h>
#include "ur_rtde/rtde_receive_interface.h"

static constexpr char IS_CONNECTED_STRING[] = "IS_CONNECTED";

constexpr int MAX_CONTROLLERS = 1;
constexpr double DEFAULT_POLL_TIME = 0.20; // 5Hz?

#define DEFAULT_CONTROLLER_TIMEOUT 1.0

class URRobot: public asynPortDriver {
    public:
	URRobot(const char *asyn_port_name, const char* robot_port_name);
	virtual void main_loop(void);

    private:
	ur_rtde::RTDEReceiveInterface rtde_receive_;
	std::vector<double> pose_;
	double poll_time_;
	int count_;
	int connected_;

    protected:
	asynUser* pasynUserURRobot_;
        int isConnectedIndex_;
};
