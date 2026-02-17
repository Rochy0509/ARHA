/*
 * torque_test.cpp
 *
 * Test sending torque commands to motor 6 (X6-8) via the ARHA TCP driver,
 * while holding motor 2 at its initial position using position control.
 *
 * Motor 2 (index 0): position-held at startup position
 * Motor 6 (index 1): torque-controlled with commanded current
 *
 * The firmware's motor_set_effort() expects current in Amps.
 * We convert torque (Nm) → current (A) using the X6-8 torque constant (1.25 Nm/A).
 *
 * Usage: torque_test [ip] [port] [torque_Nm]
 *   default: 192.168.197.123  5000  0.5 Nm
 */

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

//Signal handling
static std::atomic<bool> running{true};

void signal_handler(int) {
    running.store(false);
}

//X6-8 Motor Constants
static constexpr double TORQUE_CONSTANT = 1.25;   // Nm/A
static constexpr double RATED_TORQUE    = 4.5;    // Nm
static constexpr double RATED_CURRENT   = 3.6;    // A
static constexpr double MAX_TORQUE      = 6.0;    // Nm safety clamp

static constexpr double DEG_TO_RAD = M_PI / 180.0;

// Convert torque (Nm) to current (A) for the firmware's effort interface
static double torque_to_current(double torque_Nm) {
    return torque_Nm / TORQUE_CONSTANT;
}

// Clamp torque to safe range
static double clamp_torque(double torque_Nm) {
    if (torque_Nm >  MAX_TORQUE) return  MAX_TORQUE;
    if (torque_Nm < -MAX_TORQUE) return -MAX_TORQUE;
    return torque_Nm;
}

bool check(DriverError err, const std::string& context) {
    if (err != DriverError::SUCCESS) {
        std::cerr << "[FAIL] " << context << " (error code: "
                  << static_cast<int>(err) << ")" << std::endl;
        return false;
    }
    std::cout << "[ OK ] " << context << std::endl;
    return true;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Parse arguments
    std::string ip = "192.168.197.123";
    int port = 5000;
    double command_torque_Nm = 0.8; // Default: gentle 0.5 Nm

    if (argc >= 2) ip = argv[1];
    if (argc >= 3) port = std::stoi(argv[2]);
    if (argc >= 4) command_torque_Nm = std::stod(argv[3]);

    // Clamp to safe range
    command_torque_Nm = clamp_torque(command_torque_Nm);
    double command_current_A = torque_to_current(command_torque_Nm);

    std::cout << " ARHA TCP Driver - Torque Test" << std::endl;
    std::cout << " Target: " << ip << ":" << port << std::endl;
    std::cout << " Motor 2: position hold (right_arm index 0)" << std::endl;
    std::cout << " Motor 6: torque control (right_arm index 1)" << std::endl;
    std::cout << " Motor constants:" << std::endl;
    std::cout << "   Torque constant: " << TORQUE_CONSTANT << " Nm/A" << std::endl;
    std::cout << "   Rated torque:    " << RATED_TORQUE << " Nm" << std::endl;
    std::cout << "   Rated current:   " << RATED_CURRENT << " A" << std::endl;
    std::cout << "   Max torque:      " << MAX_TORQUE << " Nm" << std::endl;
    std::cout << " Command:" << std::endl;
    std::cout << "   Torque:  " << command_torque_Nm << " Nm" << std::endl;
    std::cout << "   Current: " << command_current_A << " A" << std::endl;

    // Configure driver
    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 2000;
    config.verbose = false;

    arhaTCPDriver driver(config);

    driver.setErrorCallback([](DriverError err, const std::string& msg) {
        std::cerr << "[ERROR] " << static_cast<int>(err) << ": " << msg << std::endl;
    });

    // Register right_arm with motor 6 only
    const std::vector<uint32_t> motor_ids = {2, 6};
    std::cout << "--- Setup ---" << std::endl;
    check(driver.registerLimb({"right_arm", motor_ids}),
          "Register right_arm (motor 2 and 6)");

    if (!check(driver.connect(), "Connect to STM32")) {
        std::cerr << "Last error: " << driver.getLastErrorMessage() << std::endl;
        return 1;
    }

    // Read initial state — capture motor 2 position to hold
    double motor2_hold_pos = 0.0;
    std::cout << "\n--- Initial state ---" << std::endl;
    {
        std::vector<double> pos, vel, eff;
        if (check(driver.getStates("right_arm", pos, vel, eff),
                  "getStates(right_arm)")) {
            motor2_hold_pos = pos[0];  // index 0 = motor 2
            std::cout << "  motor 2: pos=" << pos[0] << " rad ("
                      << (pos[0] / DEG_TO_RAD) << " deg) ← HOLD TARGET" << std::endl;
            std::cout << "  motor 6: pos=" << pos[1] << " rad ("
                      << (pos[1] / DEG_TO_RAD) << " deg)" << std::endl;
        } else {
            std::cerr << "Failed to read initial state, cannot hold motor 2!" << std::endl;
            driver.disconnect();
            return 1;
        }
    }

    // Enable motor
    if (!check(driver.enableLimbMotors("right_arm", true),
               "enableLimbMotors(right_arm, true)")) {
        driver.disconnect();
        return 1;
    }

    // Send torque command and monitor
    std::cout << "\n--- Sending torque: " << command_torque_Nm << " Nm ("
              << command_current_A << " A) ---" << std::endl;

    // The driver's setEffort sends the value as a float to the STM32.
    // The firmware's motor_set_effort expects current in Amps.
    // effort_to_mya_current: (effort * 100) → 0.01A/LSB for CAN.
    // So we send current in Amps.

    int cycle = 0;
    const int print_period = 10; // Print every 1 second (at 100ms loop)

    std::cout << "  Motor 2 hold target: " << motor2_hold_pos << " rad ("
              << (motor2_hold_pos / DEG_TO_RAD) << " deg)" << std::endl;

    while (running.load()) {
        // Motor 2 (index 0): hold at initial position
        auto err = driver.setPosition("right_arm", 0, motor2_hold_pos);
        if (err != DriverError::SUCCESS) {
            std::cerr << "setPosition(motor 2) failed!" << std::endl;
            break;
        }

        // Motor 6 (index 1): apply torque command
        err = driver.setEffort("right_arm", 1, command_current_A);
        if (err != DriverError::SUCCESS) {
            std::cerr << "setEffort(motor 6) failed!" << std::endl;
            break;
        }

        // Read state periodically
        if (cycle % print_period == 0) {
            std::vector<double> pos, vel, eff;
            err = driver.getStates("right_arm", pos, vel, eff);
            if (err == DriverError::SUCCESS) {
                printf("  [%4d] m2: %7.3f rad (%7.1f deg) [hold]  "
                       "m6: %7.3f rad (%7.1f deg) [%.2f Nm / %.2f A]\n",
                       cycle,
                       pos[0], pos[0] / DEG_TO_RAD,
                       pos[1], pos[1] / DEG_TO_RAD,
                       command_torque_Nm, command_current_A);
                fflush(stdout);
            }
        }

        ++cycle;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Cleanup: safely stop motor 
    std::cout << "\n\n--- Shutdown sequence ---" << std::endl;

    // Step 1: Zero torque on motor 6
    std::cout << "  Zeroing torque on motor 6..." << std::endl;
    driver.setEffort("right_arm", 1, 0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Step 2: Disable motors (motor_stop on firmware)
    check(driver.enableLimbMotors("right_arm", false),
          "enableLimbMotors(right_arm, false)");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Final state
    {
        std::vector<double> pos, vel, eff;
        if (driver.getStates("right_arm", pos, vel, eff) == DriverError::SUCCESS) {
            std::cout << "\n  Final state:" << std::endl;
            std::cout << "    motor 2: pos=" << pos[0] << " rad ("
                      << (pos[0] / DEG_TO_RAD) << " deg)  "
                      << "[hold was " << motor2_hold_pos << " rad]" << std::endl;
            std::cout << "    motor 6: pos=" << pos[1] << " rad ("
                      << (pos[1] / DEG_TO_RAD) << " deg)" << std::endl;
        }
    }

    driver.disconnect();
    std::cout << "[ OK ] Disconnected" << std::endl;
    return 0;
}
