#include "arha_tcp.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>

using namespace arha_tcp_driver;

static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr double RAD_TO_DEG = 180.0 / M_PI;

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

    std::cout << "\n ARHA - Single Arm Auto-Zero Utility" << std::endl;

    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 45000;

    arhaTCPDriver driver(config);
    // Register both to find which one responds
    driver.registerLimb({"left_arm",  {1, 2, 3, 4, 5, 6}});
    driver.registerLimb({"right_arm", {1, 2, 3, 4, 5, 6}});
    
    std::cout << "Connecting to STM32..." << std::endl;
    if (!check(driver.connect(), "Connect", &driver)) return 1;

    std::string responsive_limb = "";
    
    // Test both limbs to see which one has responsive motors
    for (const std::string limb : {"left_arm", "right_arm"}) {
        std::cout << "\nChecking " << limb << "..." << std::endl;
        std::vector<double> pos, vel, eff;
        if (driver.getStates(limb, pos, vel, eff) == DriverError::SUCCESS) {
            std::cout << "[FOUND] " << limb << " responded successfully." << std::endl;
            responsive_limb = limb;
            break; 
        } else {
            std::cout << "  " << limb << " did not respond (this is normal if only one arm is connected)." << std::endl;
        }
    }

    if (responsive_limb.empty()) {
        std::cerr << "\n[ERROR] No responsive arm found. Check power and CAN cabling." << std::endl;
        driver.disconnect();
        return 1;
    }

    std::cout << "\n--- Checking Zero Offsets for " << responsive_limb << " ---" << std::endl;
    static constexpr double ZERO_THRESH = 0.05 * DEG_TO_RAD;

    std::vector<double> pos, vel, eff;
    if (driver.getStates(responsive_limb, pos, vel, eff) == DriverError::SUCCESS) {
        std::cout << "  Current positions:";
        for (size_t i = 0; i < pos.size(); ++i) {
            std::cout << " " << pos[i] * RAD_TO_DEG << "°";
        }
        std::cout << std::endl;

        bool needs_zero = false;
        for (size_t i = 0; i < pos.size(); ++i) {
            if (std::abs(pos[i]) > ZERO_THRESH) {
                std::cout << "  " << responsive_limb << " J" << (i+1) << " = " 
                          << pos[i] * RAD_TO_DEG << "° — needs zero" << std::endl;
                needs_zero = true;
            }
        }

        if (needs_zero) {
            std::cout << "\nTarget: All motors on " << responsive_limb << std::endl;
            std::cout << "Action: Zeroing encoders and saving to ROM..." << std::endl;
            std::cout << "This will take ~30-45 seconds. Do not interrupt." << std::endl;

            if (check(driver.setEncoderZero(responsive_limb), "setEncoderZero", &driver)) {
                std::cout << "\nSUCCESS: " << responsive_limb << " zeroed successfully." << std::endl;
                
                // Read state after zero
                if (driver.getStates(responsive_limb, pos, vel, eff) == DriverError::SUCCESS) {
                    std::cout << "  State After Zero:";
                    for (size_t i = 0; i < pos.size(); ++i) {
                        std::cout << " " << pos[i] * RAD_TO_DEG << "°";
                    }
                    std::cout << std::endl;
                }
            } else {
                std::cerr << "\nFAILED to zero " << responsive_limb << "." << std::endl;
            }
        } else {
            std::cout << "\nSUCCESS: " << responsive_limb << " already zeroed ✓" << std::endl;
        }
    } else {
        std::cerr << "\nFAILED to get states for " << responsive_limb << "." << std::endl;
    }

    driver.disconnect();
    return 0;
}
