#include "arha_tcp.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <cmath>

using namespace arha_tcp_driver;

static std::atomic<bool> running{true};
void signal_handler(int) { running.store(false); }
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;

bool check(DriverError err, const std::string& ctx) {
    if (err != DriverError::SUCCESS) {
        std::cerr << "[FAIL] " << ctx << std::endl;
        return false;
    }
    std::cout << "[ OK ] " << ctx << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::string ip = "192.168.197.123";
    int port = 5000;
    if (argc >= 2) ip = argv[1];
    if (argc >= 3) port = std::stoi(argv[2]);

    std::cout << "\n ARHA - Direct Torque Test" << std::endl;
    std::cout << " WARNING: Motor will apply torque for 3 seconds!" << std::endl;
    std::cout << " Hold the motor loosely, keep clear of moving parts.\n" << std::endl;

    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 5000;

    arhaTCPDriver driver(config);
    driver.registerLimb({"right_arm", {2}});
    if (!check(driver.connect(), "Connect")) return 1;

    // Write PID to RAM first
    struct PidParam { uint8_t index; double value; };
    const PidParam defaults[] = {
        {0x01, 50.0}, {0x02, 50.0}, {0x04, 100.0}, {0x05, 5.0},
        {0x07, 100.0}, {0x08, 0.0}, {0x09, 0.0},
    };


    // Read initial position
    double start_deg = 0;
    {
        std::vector<double> pos, vel, eff;
        if (driver.getStates("right_arm", pos, vel, eff) == DriverError::SUCCESS) {
            start_deg = pos[0] * RAD_TO_DEG;
            std::cout << "Start position: " << start_deg << " deg\n" << std::endl;
        }
    }

    // Send direct torque command (0xA1) via setEfforts
    // 0.5A should produce noticeable torque without danger
    double effort_amps = 0.5;
    std::cout << "=== Sending torque command: " << effort_amps << " A ===" << std::endl;
    {
        std::vector<double> efforts = {effort_amps};
        check(driver.setEfforts("right_arm", efforts), "setEfforts(0.5 A)");
    }

    // Monitor for 3 seconds
    std::cout << "\n--- Monitor (3s) ---" << std::endl;
    for (int i = 0; i < 30 && running.load(); ++i) {
        std::vector<double> pos, vel, eff;
        if (driver.getStates("right_arm", pos, vel, eff) != DriverError::SUCCESS) break;
        double deg = pos[0] * RAD_TO_DEG;
        double moved = deg - start_deg;
        std::cout << "  [" << i << "] " << deg << " deg (moved " << moved << ")" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop
    std::cout << "\n--- Stop ---" << std::endl;
    {
        std::vector<double> efforts = {0.0};
        driver.setEfforts("right_arm", efforts);
    }
    driver.enableLimbMotors("right_arm", false);
    driver.disconnect();
    std::cout << "[ OK ] Done" << std::endl;
    return 0;
}
