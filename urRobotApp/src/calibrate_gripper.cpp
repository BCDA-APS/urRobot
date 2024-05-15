#include <ur_rtde/robotiq_gripper.h>
#include <ur_rtde/rtde_control_interface.h>
#include <iostream>


int main(int argc, char *argv[]) {
    
    if (argc != 2) {
        std::cout << "Please provide robot IP" << std::endl;
        return 1;
    } 
    
    const char *robot_ip = argv[1];
    
    ur_rtde::RobotiqGripper gripper(robot_ip);
    try {
        gripper.connect();
    } catch (const std::exception &e) {
        std::cout << "Error connecting to gripper, check that IP address is correct" << std::endl << std::endl;
        return 1;
    }
    if (not gripper.isActive()) {
        gripper.activate();
    }

    std::cout << "Auto calibrating gripper..." << std::endl;
    gripper.autoCalibrate();
    std::cout << "Gripper calibrated" << std::endl << std::endl;
    int min_pos, max_pos = 0;
    gripper.getNativePositionRange(min_pos, max_pos);
    std::cout << "Min (closed) = " << min_pos << std::endl;
    std::cout << "Max (open)   = " << max_pos << std::endl;

    return 0;
}
