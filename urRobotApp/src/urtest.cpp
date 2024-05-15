// #include "ur_rtde/dashboard_client.h"
// #include "ur_rtde/rtde_control_interface.h"
// #include "ur_rtde/rtde_receive_interface.h"
// #include <sstream>
//
constexpr char ROBOT_IP[] = "164.54.104.148";
//
// int main() {
//
// auto ctrl = ur_rtde::RTDEControlInterface(ROBOT_IP);
// auto recv = ur_rtde::RTDEReceiveInterface(ROBOT_IP);
//
// std::vector<double> q = recv.getActualTCPPose();
// std::stringstream ss;
// ss << q.at(0);
// for (const auto &v : q) {
// ss << ", " << v;
// }
// std::cout << ss.str() << std::endl;
//
// ctrl.stopScript();
// return 0;
// }

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <thread>

using namespace ur_rtde;
using namespace std::chrono;

constexpr double inc = 0.05; // 5cm
constexpr double speed = 0.05; // 5cm/s ?
constexpr double accel = 0.5; // 50cm/s/s

int main(int argc, char *argv[]) {
    RTDEControlInterface rtde_control(ROBOT_IP);
    RTDEReceiveInterface rtde_receive(ROBOT_IP);
    std::vector<double> init_q = rtde_receive.getActualQ();

    std::vector<double> target = rtde_receive.getActualTCPPose();
    target[2] += inc;
    rtde_control.moveL(target, speed, accel, false);

    target = rtde_receive.getActualTCPPose();
    target[2] -= inc;
    rtde_control.moveL(target, 0.05, 0.5, false);

    target = rtde_receive.getActualTCPPose();
    target[0] += inc;
    rtde_control.moveL(target, 0.05, 0.5, false);

    target = rtde_receive.getActualTCPPose();
    target[0] -= inc;
    rtde_control.moveL(target, 0.05, 0.5, false);

    target = rtde_receive.getActualTCPPose();
    target[1] += inc;
    rtde_control.moveL(target, 0.05, 0.5, false);

    target = rtde_receive.getActualTCPPose();
    target[1] -= inc;
    rtde_control.moveL(target, 0.05, 0.5, false);

    // Move to initial joint position with a regular moveJ
    rtde_control.moveJ(init_q);

    // Stop the RTDE control script
    rtde_control.stopScript();
    return 0;
}
