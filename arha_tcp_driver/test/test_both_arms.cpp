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

static const double RIGHT_DIRS[6] = {-1.0, 1.0, -1.0, -1.0, -1.0, -1.0};

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
    std::signal(SIGINT, signal_handler);

    std::string ip = "192.168.197.123";
    int port = 5000;
    if (argc >= 2) ip = argv[1];
    if (argc >= 3) port = std::stoi(argv[2]);

    std::cout << "\n ARHA - Right Arm Only TCP Test" << std::endl;
    std::cout << " (Left arm disabled — CAN bus damage)" << std::endl;
    std::cout << " Zeros right arm and holds at 0°.\n" << std::endl;

    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 45000; // Allow 45s for zero reset to complete

    arhaTCPDriver driver(config);
    driver.registerLimb({"right_arm", {1, 2, 3, 4, 5, 6}});

    std::cout << "--- Connecting ---" << std::endl;
    if (!check(driver.connect(), "Connect", &driver)) return 1;

    // Read and fix acceleration parameters — target 5000 dps/s for all motors
    static constexpr uint32_t TARGET_ACCEL = 5000;
    std::cout << "\n--- Acceleration Parameters (target=" << TARGET_ACCEL << " dps/s) ---" << std::endl;
    {
        const char* limb = "right_arm";
        std::vector<uint32_t> accel;
        if (driver.readAccel(limb, accel) != DriverError::SUCCESS) {
            std::cerr << "  Failed to read accel for " << limb << std::endl;
        } else {
            for (int attempt = 1; attempt <= 3; ++attempt) {
                bool needs_fix = false;
                for (size_t i = 0; i < accel.size(); ++i) {
                    if (accel[i] != TARGET_ACCEL) needs_fix = true;
                }

                if (!needs_fix) {
                    if (attempt == 1) std::cout << "  " << limb << ": all " << TARGET_ACCEL << " ✓" << std::endl;
                    else std::cout << "  " << limb << ": successfully fixed ✓" << std::endl;
                    break;
                }

                std::cout << "  " << limb << " (attempt " << attempt << "): mismatch — writing " << TARGET_ACCEL << " to ROM..." << std::endl;
                std::vector<uint32_t> target(6, TARGET_ACCEL);
                driver.writeAccel(limb, target);
                
                // Motors need time to write to ROM and recover
                std::this_thread::sleep_for(std::chrono::milliseconds(500));

                // Verify
                if (driver.readAccel(limb, accel) == DriverError::SUCCESS) {
                    for (size_t i = 0; i < accel.size(); ++i)
                        printf("    M%zu: %u%s\n", i + 1, accel[i], (accel[i] == TARGET_ACCEL) ? " ✓" : " ✗");
                } else {
                    std::cerr << "  Failed to verify accel after write" << std::endl;
                }
            }
        }
    }

    // Read initial state
    std::cout << "\n--- Initial State ---" << std::endl;
    {
        std::vector<double> pos, vel, eff;
        if (driver.getStates("right_arm", pos, vel, eff) == DriverError::SUCCESS) {
            std::cout << "  right_arm:";
            for (size_t i = 0; i < pos.size(); ++i)
                std::cout << " " << pos[i] * RAD_TO_DEG << "°";
            std::cout << std::endl;
        }
    }

    // Zero reset only if needed (any motor outside 0.05° threshold)
    static constexpr double ZERO_THRESH = 0.05 * DEG_TO_RAD;
    std::cout << "\n--- Checking Zero Offsets ---" << std::endl;
    {
        std::vector<double> pos, vel, eff;
        if (driver.getStates("right_arm", pos, vel, eff) == DriverError::SUCCESS) {
            bool needs_zero = false;
            for (size_t i = 0; i < pos.size(); ++i) {
                if (std::abs(pos[i]) > ZERO_THRESH) {
                    std::cout << "  right_arm J" << (i+1) << " = " 
                              << pos[i] * RAD_TO_DEG << "° — needs zero" << std::endl;
                    needs_zero = true;
                }
            }
            if (needs_zero) {
                std::cout << "  Zeroing right_arm (~30s)..." << std::endl;
                check(driver.setEncoderZero("right_arm"), "Zero right_arm", &driver);
            } else {
                std::cout << "  right_arm already zeroed ✓" << std::endl;
            }
        }
    }

    // Read state after zero
    std::cout << "\n--- State After Zero ---" << std::endl;
    {
        std::vector<double> pos, vel, eff;
        if (driver.getStates("right_arm", pos, vel, eff) == DriverError::SUCCESS) {
            std::cout << "  right_arm:";
            for (size_t i = 0; i < pos.size(); ++i)
                std::cout << " " << pos[i] * RAD_TO_DEG << "°";
            std::cout << std::endl;
        }
    }

    // Send 0° to right arm and hold
    std::cout << "\n--- Moving to 0.0 and Holding ---" << std::endl;
    std::vector<double> zero(6, 0.0);
    check(driver.setPositions("right_arm", zero), "right_arm 0°", &driver);

    std::cout << "Holding position at 0.0. Press Ctrl+C to exit." << std::endl;
    while (running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Monitor positions occasionally
        static int s = 0;
        if (++s % 5 == 0) { // Print every 1 second
            std::vector<double> rp, rv, re;
            driver.getStates("right_arm", rp, rv, re);
            if (rp.size() > 0) {
                printf("  [Right J1: %.2f°]\r", rp[0] * RAD_TO_DEG);
                fflush(stdout);
            }
        }
    }


    std::cout << "\n--- Complete ---" << std::endl;
    driver.enableLimbMotors("right_arm", false);
    driver.disconnect();
    std::cout << "[ OK ] Disconnected" << std::endl;

    return 0;
}
