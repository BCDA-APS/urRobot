#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace ur_rtde;
using namespace std::chrono;

constexpr char ROBOT_IP[] = "164.54.104.148";

constexpr double inc = 0.05;   // 5cm
constexpr double speed = 0.05; // 5cm/s ?
constexpr double accel = 0.5;  // 50cm/s/s

int main(int argc, char* argv[]) {
    RTDEControlInterface rtde_control(ROBOT_IP);
    RTDEReceiveInterface rtde_receive(ROBOT_IP);

    // Stop the RTDE control script
    rtde_control.stopScript();
    return 0;
}
