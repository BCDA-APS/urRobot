#include "ur_rtde/dashboard_client.h"
#include "spdlog/cfg/env.h"
#include "spdlog/spdlog.h"

constexpr char ROBOT_IP[] = "164.54.104.148";

int main() {
    spdlog::cfg::load_env_levels();
    
    auto dash = ur_rtde::DashboardClient(ROBOT_IP);
    dash.connect();

    std::string c = dash.isConnected() ? "Connected!" : "Disconnected";
    spdlog::info("{}", c);

    dash.disconnect();

    c = dash.isConnected() ? "Connected!" : "Disconnected";
    spdlog::info("{}", c);

    return 0;
}
