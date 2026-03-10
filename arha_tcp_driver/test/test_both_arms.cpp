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

/* Left motor 2 moves outward with negative rotation */
static const double LEFT_DIRS[6]  = {1.0, -1.0, 1.0, 1.0, 1.0, 1.0};
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

    std::cout << "\n ARHA - Dual Arm Parallel TCP Test" << std::endl;
    std::cout << " Moves both arms in sync: each joint to 15 deg and back.\n" << std::endl;

    DriverConfig config;
    config.ip_address = ip;
    config.port = port;
    config.socket_timeout_ms = 45000; // Allow 45s for zero reset to complete

    arhaTCPDriver driver(config);
    driver.registerLimb({"left_arm",  {1, 2, 3, 4, 5, 6}});
    driver.registerLimb({"right_arm", {1, 2, 3, 4, 5, 6}});

    std::cout << "--- Connecting ---" << std::endl;
    if (!check(driver.connect(), "Connect", &driver)) return 1;
    // Read and fix acceleration parameters — target 2500 dps/s for all motors
    static constexpr uint32_t TARGET_ACCEL = 2500;
    std::cout << "\n--- Acceleration Parameters (target=" << TARGET_ACCEL << " dps/s) ---" << std::endl;
    std::cout << "  Motor   left_arm   right_arm" << std::endl;
    for (const auto& limb : {"left_arm", "right_arm"}) {
        std::vector<uint32_t> accel;
        if (driver.readAccel(limb, accel) != DriverError::SUCCESS) {
            std::cerr << "  Failed to read accel for " << limb << std::endl;
            continue;
        }

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

    // Read initial states
    std::cout << "\n--- Initial State ---" << std::endl;
    for (const auto& limb : {"left_arm", "right_arm"}) {
        std::vector<double> pos, vel, eff;
        if (driver.getStates(limb, pos, vel, eff) == DriverError::SUCCESS) {
            std::cout << "  " << limb << ":";
            for (size_t i = 0; i < pos.size(); ++i)
                std::cout << " " << pos[i] * RAD_TO_DEG << "°";
            std::cout << std::endl;
        }
    }

    // Zero reset only if needed (any motor outside 0.05° threshold)
    static constexpr double ZERO_THRESH = 0.05 * DEG_TO_RAD;
    std::cout << "\n--- Checking Zero Offsets ---" << std::endl;
    for (const auto& limb : {"left_arm", "right_arm"}) {
        std::vector<double> pos, vel, eff;
        if (driver.getStates(limb, pos, vel, eff) != DriverError::SUCCESS) continue;

        bool needs_zero = false;
        for (size_t i = 0; i < pos.size(); ++i) {
            if (std::abs(pos[i]) > ZERO_THRESH) {
                std::cout << "  " << limb << " J" << (i+1) << " = " 
                          << pos[i] * RAD_TO_DEG << "° — needs zero" << std::endl;
                needs_zero = true;
            }
        }
        if (needs_zero) {
            std::cout << "  Zeroing " << limb << " (~30s)..." << std::endl;
            check(driver.setEncoderZero(limb), std::string("Zero ") + limb, &driver);
        } else {
            std::cout << "  " << limb << " already zeroed ✓" << std::endl;
        }
    }

    // Read state after zero
    std::cout << "\n--- State After Zero ---" << std::endl;
    for (const auto& limb : {"left_arm", "right_arm"}) {
        std::vector<double> pos, vel, eff;
        if (driver.getStates(limb, pos, vel, eff) == DriverError::SUCCESS) {
            std::cout << "  " << limb << ":";
            for (size_t i = 0; i < pos.size(); ++i)
                std::cout << " " << pos[i] * RAD_TO_DEG << "°";
            std::cout << std::endl;
        }
    }

    // Execute: for each joint, send 15° to both arms, wait, send 0° to both
    std::cout << "\n--- Executing Parallel 15->0 Sequence ---" << std::endl;
    for (int m = 0; m < 6 && running.load(); ++m) {
        std::cout << "\nJoint " << (m + 1) << ":" << std::endl;

        // Build targets: only the active joint moves, rest stay at 0
        std::vector<double> left_15(6, 0.0), right_15(6, 0.0);
        left_15[m]  = 15.0 * LEFT_DIRS[m]  * DEG_TO_RAD;
        right_15[m] = 15.0 * RIGHT_DIRS[m] * DEG_TO_RAD;

        // Send to both arms back-to-back
        std::cout << "  -> 15 deg..." << std::endl;
        check(driver.setPositions("left_arm",  left_15),  "left_arm  15°", &driver);
        check(driver.setPositions("right_arm", right_15), "right_arm 15°", &driver);

        // Monitor positions every 200ms for 3 seconds
        std::cout << "  t(ms)  left(deg)  left(dps)  right(deg) right(dps)" << std::endl;
        for (int s = 0; s < 15 && running.load(); ++s) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::vector<double> lp, lv, le, rp, rv, re;
            driver.getStates("left_arm",  lp, lv, le);
            driver.getStates("right_arm", rp, rv, re);
            if (lp.size() > (size_t)m && lv.size() > (size_t)m && rp.size() > (size_t)m && rv.size() > (size_t)m) {
                printf("  %4d   %8.2f   %8.2f   %8.2f   %8.2f\n",
                       (s + 1) * 200,
                       lp[m] * RAD_TO_DEG, lv[m] * RAD_TO_DEG,
                       rp[m] * RAD_TO_DEG, rv[m] * RAD_TO_DEG);
            } else {
                printf("  %4d      [Error: Failed to read state]\n", (s + 1) * 200);
            }
        }

        // Back to 0
        std::vector<double> zero(6, 0.0);
        std::cout << "  -> 0 deg..." << std::endl;
        check(driver.setPositions("left_arm",  zero), "left_arm  0°", &driver);
        check(driver.setPositions("right_arm", zero), "right_arm 0°", &driver);

        for (int w = 0; w < 15 && running.load(); ++w)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    std::cout << "\n--- Complete ---" << std::endl;
    driver.enableLimbMotors("left_arm",  false);
    driver.enableLimbMotors("right_arm", false);
    driver.disconnect();
    std::cout << "[ OK ] Disconnected" << std::endl;

    return 0;
}
