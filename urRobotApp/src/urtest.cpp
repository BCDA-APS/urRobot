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
    cmd_str += "\twrite_output_integer_register(0, 1)\n";

    while (std::getline(ss, line)) {
        cmd_str += "\t" + line + "\n";
    }

    // Signal when motions are finished
    cmd_str += "\twrite_output_integer_register(0, 2)\n";
    cmd_str += "end\n";

    return cmd_str;
}

// bool RTDEControlInterface::sendCustomScript(const std::string &script)
// {
  // custom_script_running_ = true;
  // // First stop the running RTDE control script
  // stopScript();
//
  // auto start_time = std::chrono::high_resolution_clock::now();
//
  // // Send custom script function
  // script_client_->sendScriptCommand(script);
//
  // while (getControlScriptState() != UR_CONTROLLER_DONE_WITH_CMD)
  // {
    // // Wait until the controller is done with command
    // auto current_time = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
    // if (duration > UR_PATH_EXECUTION_TIMEOUT)
      // return false;
    // // Sleep to avoid high CPU load
    // std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // }
//
  // sendClearCommand();
//
  // // Re-upload RTDE script to the UR Controller
  // script_client_->sendScript();
//
  // waitForProgramRunning();
//
  // custom_script_running_ = false;
  // return true;
// }

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    RTDEReceiveInterface recv(ROBOT_IP);
    RTDEControlInterface ctrl(ROBOT_IP);
    DashboardClient dash(ROBOT_IP);

    // // Connect to dashboard to get polyscope version
    // // needed for ScriptClient constructor
    // dash.connect();
    // if (!dash.isConnected()) {
        // std::cout << "not connected" << std::endl;
        // return EXIT_FAILURE;
    // }
    // std::cout << dash.polyscopeVersion() << std::endl;

    ScriptClient script_client(ROBOT_IP, 5, 14);
    script_client.connect();

    auto script = wrap_script("set_standard_digital_out(1,True)\nsleep(3.0)\nset_standard_digital_out(2,True)\nsleep(3.0)\n");
    ctrl.stopScript();
    script_client.sendScriptCommand(script);

    std::cout << "Running..." << std::endl;
    // while (g_signal_caught == 0) {
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    // }
    std::cout << "ctrl.isProgramRunning() = " << ctrl.isProgramRunning() << std::endl;
    int count = 0;
    while (recv.getOutputIntRegister(0) != UR_CONTROLLER_DONE_WITH_CMD) {
        std::cout << "Program running! " << ++count << std::endl;
        std::cout << "ctrl.isProgramRunning() = " << ctrl.isProgramRunning() << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::cout << "ctrl.isProgramRunning() = " << ctrl.isProgramRunning() << std::endl;

    std::cout << "Done!" << std::endl;

    recv.disconnect();
    ctrl.stopScript();
    ctrl.disconnect();
    dash.disconnect();

    std::cout << "\nProgram terminated via keyboard interrupt (Ctrl+C)" << std::endl;

    return 0;
}
