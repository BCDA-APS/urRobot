#include <csignal>
#include <ur_rtde/dashboard_client.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/script_client.h>

using namespace ur_rtde;
using namespace std::chrono_literals;

constexpr char ROBOT_IP[] = "164.54.104.148";

// To catch CTRL+C to quit
volatile std::sig_atomic_t g_signal_caught = 0;
void signal_handler(int signal) {
    if (signal == SIGINT) {
        g_signal_caught = 1;
    }
}

// std::pair<int, int> get_polyscope_version(ur_rtde::DashboardCleint dash) {
    // int major = 0;
    // int minor = 0;
//
    // char version_str[256];
    // int id;
    // dash->findParam("POLYSCOPE_VERSION", &id);
    // dash->lock();
    // dash->getStringParam(id, 256, version_str);
    // dash->unlock();
//
    // int parsed = std::sscanf(version_str, "%d.%d", &major, &minor);
    // // if (parsed == 2) {
    // // success
    // // }
    // return {major, minor};
// }

// UR_CONTROLLER_RDY_FOR_CMD = 1
// UR_CONTROLLER_DONE_WITH_CMD = 2

std::string wrap_script(const std::string& script) {

    // register_offset_ is 0 since we pass no flags to RTDEControlInterface
    // to specify otherwise

    std::string cmd_str;
    std::string line;
    std::stringstream ss(script);
    const std::string function_name = "custom_func";
    cmd_str += "def " + function_name + "():\n";
    // cmd_str += "\twrite_output_integer_register(12, 1)\n";
    // cmd_str += "\twrite_output_integer_register(12, 1)\n";

    while (std::getline(ss, line)) {
        cmd_str += "\t" + line + "\n";
    }

    // Signal when motions are finished
    cmd_str += "\twrite_output_integer_register(12, read_output_integer_register(12)+1)\n";
    cmd_str += "end\n";

    return cmd_str;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    RTDEReceiveInterface recv(ROBOT_IP);
    RTDEControlInterface ctrl(ROBOT_IP);
    DashboardClient dash(ROBOT_IP);

    // Connect to dashboard to get polyscope version
    // needed for ScriptClient constructor
    dash.connect();
    if (!dash.isConnected()) {
        std::cout << "not connected" << std::endl;
        return EXIT_FAILURE;
    }
    int major = 0;
    int minor = 0;
    std::sscanf(dash.polyscopeVersion().c_str(), "%d.%d", &major, &minor);
    std::cout << "Major.Minor = " << major << "." << minor << std::endl;

    std::this_thread::sleep_for(1s);
    std::cout << "Robot status = " << ctrl.getRobotStatus() << std::endl;

    // TODO: get from dashboard
    ScriptClient script_client(ROBOT_IP, major, minor);
    script_client.connect();

    int last_count = recv.getOutputIntRegister(12);

    auto script = wrap_script("set_standard_digital_out(1,False)\nsleep(3.0)\nset_standard_digital_out(2,False)\nsleep(3.0)\n");
    std::cout << script << std::endl;
    ctrl.stopScript();
    script_client.sendScriptCommand(script);

    int count = 0;
    while (recv.getOutputIntRegister(12) == last_count) {
        std::cout << "Program running! " << ++count << std::endl;
        std::cout << "ctrl.isProgramRunning() = " << ctrl.isProgramRunning() << std::endl;
        std::this_thread::sleep_for(20ms);
    }
    std::cout << "ctrl.isProgramRunning() = " << ctrl.isProgramRunning() << std::endl;
    std::cout << "Done!\n" << std::endl;

    ctrl.reuploadScript();
    std::this_thread::sleep_for(1s);
    std::cout << "Robot status = " << ctrl.getRobotStatus() << std::endl;

    recv.disconnect();
    ctrl.stopScript();
    ctrl.disconnect();
    dash.disconnect();

    std::cout << "Quiting\n" << std::endl;

    return 0;
}
