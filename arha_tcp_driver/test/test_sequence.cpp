#include "arha_tcp.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <csignal>
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

    std::cout << "\n ARHA - TCP Hardware Sequence Validation Test" << std::endl;
    std::cout << " This test sequentially moves Motors 1 through 6 to 15 degrees, then 0 degrees." << std::endl;

    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 5000;

    arhaTCPDriver driver(config);
    driver.registerLimb({"right_arm", {1, 2, 3, 4, 5, 6}});
    
    std::cout << "\n--- Connecting to STM32 ---" << std::endl;
    if (!check(driver.connect(), "Connect")) return 1;

    // Enable Motors
    std::cout << "\n--- Enabling Motors ---" << std::endl;
    if (!check(driver.enableLimbMotors("right_arm", true), "Enable Motors")) return 1;

    // Execute the Sequence: 15->0 degrees for each motor sequentially
    std::cout << "\n--- Executing 15->0 Sequence ---" << std::endl;
    for (int m = 0; m < 6 && running.load(); ++m) {
        std::cout << "\nMotor " << (m + 1) << ":" << std::endl;
        
        // Command all motors to 0, except the active one to 15 degrees
        std::vector<double> target_15(6, 0.0);
        target_15[m] = 15.0 * DEG_TO_RAD;
        
        std::cout << "  Commanding 15 degrees..." << std::endl;
        check(driver.setPositions("right_arm", target_15), "setPositions(15 deg)");
        
        // Wait 3 seconds
        for (int w = 0; w < 30 && running.load(); ++w) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Command all motors to 0 degrees
        std::vector<double> target_0(6, 0.0);
        
        std::cout << "  Commanding 0 degrees..." << std::endl;
        check(driver.setPositions("right_arm", target_0), "setPositions(0 deg)");
        
        // Wait 3 seconds
        for (int w = 0; w < 30 && running.load(); ++w) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Stop and Disconnect
    std::cout << "\n--- Sequence Complete ---" << std::endl;
    driver.enableLimbMotors("right_arm", false);
    driver.disconnect();
    std::cout << "[ OK ] Disconnected" << std::endl;
    
    return 0;
}
