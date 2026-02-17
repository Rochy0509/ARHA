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

void signal_handler(int) {
    running.store(false);
}

static constexpr double DEG_TO_RAD = M_PI / 180.0;

void print_error(DriverError err, const std::string& context) {
    if (err != DriverError::SUCCESS) {
        std::cerr << "[FAIL] " << context << " (error code: "
                  << static_cast<int>(err) << ")" << std::endl;
    }
}

bool check(DriverError err, const std::string& context) {
    if (err != DriverError::SUCCESS) {
        print_error(err, context);
        return false;
    }
    std::cout << "[ OK ] " << context << std::endl;
    return true;
}

void print_states(const std::string& limb_name,
                  const std::vector<uint32_t>& motor_ids,
                  const std::vector<double>& pos,
                  const std::vector<double>& vel,
                  const std::vector<double>& eff) {
    std::cout << "  " << limb_name << " states:" << std::endl;
    for (size_t i = 0; i < pos.size(); ++i) {
        std::cout << "    motor " << motor_ids[i]
                  << "  pos=" << pos[i] << " rad"
                  << " (" << (pos[i] / DEG_TO_RAD) << " deg)"
                  << "  vel=" << vel[i]
                  << "  eff=" << eff[i] << std::endl;
    }
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::string ip = "192.168.197.123";
    int port = 5000;

    if (argc >= 2) ip = argv[1];
    if (argc >= 3) port = std::stoi(argv[2]);

    std::cout << " ARHA TCP Driver - Real Motor Test" << std::endl;
    std::cout << " Target: " << ip << ":" << port << std::endl;
    std::cout << " Motors: right_arm IDs {2, 6}" << std::endl;
    std::cout << " Command: move to 30 degrees" << std::endl;

    // Target: 30 degrees in radians
    const double target_rad = 30.0 * DEG_TO_RAD; // ~0.5236 rad

    // Configure driver
    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 2000;
    config.verbose = false;

    arhaTCPDriver driver(config);

    driver.setErrorCallback([](DriverError err, const std::string& msg) {
        std::cerr << "[CALLBACK] Error " << static_cast<int>(err)
                  << ": " << msg << std::endl;
    });

    // Register right_arm with only motors 2 and 6
    const std::vector<uint32_t> motor_ids = {2, 6};
    std::cout << "\n--- Register limb ---" << std::endl;
    check(driver.registerLimb({"right_arm", motor_ids}),
          "Register right_arm (motors 2, 6)");

    // Connect
    std::cout << "\n--- Connect ---" << std::endl;
    if (!check(driver.connect(), "Connect to STM32")) {
        std::cerr << "Last error: " << driver.getLastErrorMessage() << std::endl;
        return 1;
    }

    // Step 1: Read initial states
    std::cout << "\n--- Step 1: Read initial motor states ---" << std::endl;
    {
        std::vector<double> pos, vel, eff;
        if (check(driver.getStates("right_arm", pos, vel, eff),
                  "getStates(right_arm)")) {
            print_states("right_arm", motor_ids, pos, vel, eff);
        }
    }

    // Step 2: Enable motors for right_arm
    std::cout << "\n--- Step 2: Enable right_arm motors ---" << std::endl;
    if (!check(driver.enableLimbMotors("right_arm", true),
               "enableLimbMotors(right_arm, true)")) {
        std::cerr << "Failed to enable motors, aborting." << std::endl;
        driver.disconnect();
        return 1;
    }

    // Step 3: Command both motors to 30 degrees
    std::cout << "\n--- Step 3: Move motors to 30 deg (" << target_rad << " rad) ---" << std::endl;
    {
        // positions[0] = motor 2 target, positions[1] = motor 6 target
        std::vector<double> positions = {target_rad, target_rad};
        if (!check(driver.setPositions("right_arm", positions),
                   "setPositions(right_arm, {30 deg, 30 deg})")) {
            std::cerr << "Failed to send positions, aborting." << std::endl;
            driver.enableLimbMotors("right_arm", false);
            driver.disconnect();
            return 1;
        }
    }

    // Step 4: Monitor motors until they reach the target (or timeout)
    std::cout << "\n--- Step 4: Monitor motor positions (Ctrl+C to stop) ---" << std::endl;
    {
        int cycle = 0;
        const int max_cycles = 200; // 20 seconds at 100ms intervals
        std::vector<double> pos, vel, eff;
        bool reached = false;

        while (running.load() && cycle < max_cycles) {
            auto err = driver.getStates("right_arm", pos, vel, eff);
            if (err != DriverError::SUCCESS) {
                print_error(err, "getStates failed");
                break;
            }

            // Print every cycle so we can see the motors converging
            std::cout << "  [" << cycle << "] "
                      << "motor 2: " << pos[0] << " rad ("
                      << (pos[0] / DEG_TO_RAD) << " deg)  |  "
                      << "motor 6: " << pos[1] << " rad ("
                      << (pos[1] / DEG_TO_RAD) << " deg)"
                      << std::endl;

            // Check if both motors are close to target
            if (std::abs(pos[0] - target_rad) < 0.05 &&
                std::abs(pos[1] - target_rad) < 0.05) {
                std::cout << "\n  Both motors reached target ("
                          << (target_rad / DEG_TO_RAD) << " deg)!" << std::endl;
                reached = true;
                break;
            }

            ++cycle;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        if (!reached && running.load()) {
            std::cout << "\n  Timeout waiting for motors to reach target." << std::endl;
        }

        // Final read
        if (driver.getStates("right_arm", pos, vel, eff) == DriverError::SUCCESS) {
            std::cout << "\n  Final states:" << std::endl;
            print_states("right_arm", motor_ids, pos, vel, eff);
        }
    }

    // Hold position for 2 seconds before disabling
    std::cout << "\n--- Holding position for 2 seconds ---" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Cleanup: disable motors and disconnect
    std::cout << "\n--- Cleanup ---" << std::endl;
    check(driver.enableLimbMotors("right_arm", false),
          "enableLimbMotors(right_arm, false)");
    driver.disconnect();
    std::cout << "[ OK ] Disconnected" << std::endl;

    return 0;
}
