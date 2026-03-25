#include <iostream>
#include <thread>
#include <chrono>
#include "arha_tcp.hpp"

using namespace arha_tcp_driver;

int main(int argc, char** argv) {
    DriverConfig config;
    if (argc > 1) {
        config.ip_address = argv[1];
    } else {
        config.ip_address = "192.168.197.123";
    }
    config.port = 5000;
    config.socket_timeout_ms = 3000;

    std::cout << "Testing Gripper API at " << config.ip_address << ":" << config.port << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    arhaTCPDriver driver(config);

    DriverError err = driver.connect();
    if (err != DriverError::SUCCESS) {
        std::cerr << "Failed to connect: " << driver.getLastErrorMessage() << std::endl;
        return 1;
    }
    std::cout << "Connected successfully." << std::endl;

    // Select gripper from command line: argv[2] = 0 (right) or 1 (left), default 0
    uint8_t gripper_index = GRIPPER_RIGHT;
    if (argc > 2) {
        gripper_index = static_cast<uint8_t>(std::stoi(argv[2]));
    }
    std::cout << "Active gripper: " << (gripper_index == GRIPPER_RIGHT ? "RIGHT (0)" : "LEFT (1)") << std::endl;

    // PING
    std::cout << "\nSending PING..." << std::endl;
    err = driver.gripperPing(gripper_index);
    if (err == DriverError::SUCCESS) {
        std::cout << "  [SUCCESS] PING acknowledged" << std::endl;
    } else {
        std::cout << "  [FAIL] PING failed: " << driver.getLastErrorMessage() << std::endl;
        if (!driver.isConnected()) {
            std::cout << "  [DIAG] Socket closed — reconnecting..." << std::endl;
            driver.reconnect();
        }
    }

    std::cout << "\n============================================\n";
    std::cout << "Interactive Gripper Control\n";
    std::cout << "  'o' -> OPEN\n";
    std::cout << "  'c' -> CLOSE\n";
    std::cout << "  's' -> PRINT STATUS\n";
    std::cout << "  'n' -> SCAN BUS (Find IDs)\n";
    std::cout << "  'i' -> SET SERVO ID (Addressable)\n";
        std::cout << "  'f' -> FORCE SET ID (Broadcast - Connect ONE servo!)\n";
    std::cout << "  'q' -> QUIT\n";
    std::cout << "============================================\n";

    char cmd;
    while (true) {
        std::cout << "\nEnter command (o/c/s/n/i/q): ";
        std::cin >> cmd;

        if (cmd == 'q' || cmd == 'Q') {
            break;

        } else if (cmd == 'o' || cmd == 'O') {
            std::cout << "Sending OPEN..." << std::endl;
            err = driver.gripperOpen(gripper_index);
            if (err == DriverError::SUCCESS) {
                std::cout << "  [SUCCESS] OPEN acknowledged" << std::endl;
            } else {
                std::cout << "  [FAIL] OPEN failed: " << driver.getLastErrorMessage() << std::endl;
                if (!driver.isConnected()) {
                    std::cout << "  [DIAG] Socket closed — stream desynced. Reconnecting..." << std::endl;
                    if (driver.reconnect() == DriverError::SUCCESS)
                        std::cout << "  [DIAG] Reconnected." << std::endl;
                    else
                        std::cout << "  [DIAG] Reconnect failed: " << driver.getLastErrorMessage() << std::endl;
                } else {
                    std::cout << "  [DIAG] Socket alive — firmware returned error (resp=0), servo UART issue." << std::endl;
                }
            }

        } else if (cmd == 'c' || cmd == 'C') {
            std::cout << "Sending CLOSE..." << std::endl;
            err = driver.gripperClose(gripper_index);
            if (err == DriverError::SUCCESS) {
                std::cout << "  [SUCCESS] CLOSE acknowledged" << std::endl;
            } else {
                std::cout << "  [FAIL] CLOSE failed: " << driver.getLastErrorMessage() << std::endl;
                if (!driver.isConnected()) {
                    std::cout << "  [DIAG] Socket closed — stream desynced. Reconnecting..." << std::endl;
                    if (driver.reconnect() == DriverError::SUCCESS)
                        std::cout << "  [DIAG] Reconnected." << std::endl;
                    else
                        std::cout << "  [DIAG] Reconnect failed: " << driver.getLastErrorMessage() << std::endl;
                } else {
                    std::cout << "  [DIAG] Socket alive — firmware returned error (resp=0), servo UART issue." << std::endl;
                }
            }

        } else if (cmd == 's' || cmd == 'S') {
            std::cout << "Sending GET_STATUS..." << std::endl;
            uint16_t pos;
            int16_t speed, load;
            uint8_t volt, temp;

            err = driver.gripperGetStatus(gripper_index, pos, speed, load, volt, temp);
            if (err == DriverError::SUCCESS) {
                std::cout << "  [SUCCESS] Status received:" << std::endl;
                std::cout << "    Position: " << pos << std::endl;
                std::cout << "    Speed:    " << speed << std::endl;
                std::cout << "    Load:     " << load << std::endl;
                std::cout << "    Voltage:  " << (volt / 10.0) << "V" << std::endl;
                std::cout << "    Temp:     " << (int)temp << "°C" << std::endl;
            } else {
                std::cout << "  [FAIL] Status failed: " << driver.getLastErrorMessage() << std::endl;
            }

        } else if (cmd == 'n' || cmd == 'N') {
            std::cout << "Scanning bus for servos..." << std::endl;
            std::vector<uint8_t> ids;
            err = driver.gripperScan(gripper_index, ids);
            if (err == DriverError::SUCCESS) {
                std::cout << "  [SUCCESS] Found " << ids.size() << " servos:" << std::endl;
                for (auto id : ids) {
                    if (id & 0x80) {
                        std::cout << "    ID: " << (int)(id & 0x7F) << " [COLLISION DETECTED!]" << std::endl;
                    } else {
                        std::cout << "    ID: " << (int)id << std::endl;
                    }
                }
            } else {
                std::cout << "  [FAIL] Scan failed: " << driver.getLastErrorMessage() << std::endl;
            }

        } else if (cmd == 'i' || cmd == 'I') {
            int new_id;
            std::cout << "Current gripper context index: " << (int)gripper_index << std::endl;
            std::cout << "Enter NEW ID for this servo (1-253): ";
            std::cin >> new_id;
            
            if (new_id < 1 || new_id > 253) {
                std::cout << "  [ERR] Invalid ID range" << std::endl;
            } else {
                std::cout << "Sending SET_ID to " << (int)new_id << "..." << std::endl;
                err = driver.gripperSetID(gripper_index, (uint8_t)new_id);
                if (err == DriverError::SUCCESS) {
                    std::cout << "  [SUCCESS] ID updated. Restart the test or re-plug to use new ID." << std::endl;
                } else {
                    std::cout << "  [FAIL] SET_ID failed: " << driver.getLastErrorMessage() << std::endl;
                }
            }

        } else if (cmd == 'f' || cmd == 'F') {
            int new_id;
            std::cout << "!!! WARNING: Connect ONLY ONE servo to the bus !!!" << std::endl;
            std::cout << "Enter NEW ID for the connected servo (1-253): ";
            std::cin >> new_id;

            if (new_id < 1 || new_id > 253) {
                std::cout << "  [ERR] Invalid ID range" << std::endl;
            } else {
                std::cout << "Sending FORCE SET_ID to " << (int)new_id << " (Broadcast)..." << std::endl;
                err = driver.gripperForceSetID(gripper_index, (uint8_t)new_id);
                if (err == DriverError::SUCCESS) {
                    std::cout << "  [SUCCESS] ID updated via Broadcast. Re-plug the servo to complete." << std::endl;
                } else {
                    std::cout << "  [FAIL] Force SET_ID failed: " << driver.getLastErrorMessage() << std::endl;
                }
            }

        } else {
            std::cout << "Invalid command. Use o, c, s, or q." << std::endl;
        }
    }

    driver.disconnect();
    std::cout << "Disconnected." << std::endl;
    return 0;
}