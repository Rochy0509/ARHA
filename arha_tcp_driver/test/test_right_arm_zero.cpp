#include "arha_tcp.hpp"
#include <iostream>
#include <string>
#include <vector>

using namespace arha_tcp_driver;

bool check(DriverError err, const std::string& ctx, arhaTCPDriver* drv = nullptr) {
    if (err != DriverError::SUCCESS) {
        std::cerr << "[FAIL] " << ctx;
        if (drv) std::cerr << " — " << drv->getLastErrorMessage();
        std::cerr << std::endl;
        return false;
    }
    std::cout << "[ OK ] " << ctx << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    std::string ip = "192.168.197.123";
    int port = 5000;
    if (argc >= 2) ip = argv[1];
    if (argc >= 3) port = std::stoi(argv[2]);

    std::cout << "\n ARHA - Right Arm Encoder Zero (Minimal Version)" << std::endl;

    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 45000; // Matches test_both_arms.cpp

    arhaTCPDriver driver(config);
    // Registering both limbs just in case the firmware expects the same configuration as test_both_arms.cpp
    driver.registerLimb({"left_arm",  {1, 2, 3, 4, 5, 6}});
    driver.registerLimb({"right_arm", {1, 2, 3, 4, 5, 6}});
    
    std::cout << "Connecting..." << std::endl;
    if (!check(driver.connect(), "Connect", &driver)) return 1;

    std::cout << "Zeroing Right Arm (~30s)..." << std::endl;
    if (check(driver.setEncoderZero("right_arm"), "setEncoderZero", &driver)) {
        std::cout << "SUCCESS: Right arm encoders zeroed." << std::endl;
    } else {
        std::cerr << "FAILED." << std::endl;
    }

    driver.disconnect();
    return 0;
}
