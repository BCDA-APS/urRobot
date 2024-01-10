#include <asynPortDriver.h>

static const char *driver_name = "urRobot";

constexpr int MAX_CONTROLLERS = 1;
constexpr double DEFAULT_POLL_TIME = 0.20; // 5Hz?

#define DEFAULT_CONTROLLER_TIMEOUT 1.0

class URRobot: public asynPortDriver {
    public:
	URRobot(const char *asyn_port_name, const char* robot_port_name);
	virtual void main_loop(void);

    private:
	int count;
	double poll_time;

    protected:
	asynUser* pasynUserURRobot_;
};
