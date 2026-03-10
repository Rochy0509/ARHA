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
    config.socket_timeout_ms = 2000; // 2 seconds timeout

    std::cout << "Testing Gripper API at " << config.ip_address << ":" << config.port << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    arhaTCPDriver driver(config);
    
    DriverError err = driver.connect();
    if (err != DriverError::SUCCESS) {
        std::cerr << "Failed to connect: " << driver.getLastErrorMessage() << std::endl;
        return 1;
    }
    std::cout << "Connected successfully." << std::endl;

    // 1. PING
    std::cout << "\nSending PING..." << std::endl;
    err = driver.gripperPing();
    if (err == DriverError::SUCCESS) {
        std::cout << "  [SUCCESS] PING acknowledged" << std::endl;
    } else {
        std::cout << "  [FAIL] PING failed: " << driver.getLastErrorMessage() << std::endl;
    }

    std::cout << "\n============================================\n";
    std::cout << "Interactive Gripper Control\n";
    std::cout << "  'o' -> OPEN\n";
    std::cout << "  'c' -> CLOSE\n";
    std::cout << "  's' -> PRINT STATUS\n";
    std::cout << "  'q' -> QUIT\n";
    std::cout << "============================================\n";

    char cmd;
    while (true) {
        std::cout << "\nEnter command (o/c/s/q): ";
        std::cin >> cmd;

        if (cmd == 'q' || cmd == 'Q') {
            break;
        } else if (cmd == 'o' || cmd == 'O') {
            std::cout << "Sending OPEN..." << std::endl;
            err = driver.gripperOpen(0); // default speed
            if (err == DriverError::SUCCESS) {
                std::cout << "  [SUCCESS] OPEN acknowledged" << std::endl;
            } else {
                std::cout << "  [FAIL] OPEN failed: " << driver.getLastErrorMessage() << std::endl;
            }
        } else if (cmd == 'c' || cmd == 'C') {
            std::cout << "Sending CLOSE..." << std::endl;
            err = driver.gripperClose(0); // default speed
            if (err == DriverError::SUCCESS) {
                std::cout << "  [SUCCESS] CLOSE acknowledged" << std::endl;
            } else {
                std::cout << "  [FAIL] CLOSE failed: " << driver.getLastErrorMessage() << std::endl;
            }
        } else if (cmd == 's' || cmd == 'S') {
            std::cout << "Sending GET_STATUS..." << std::endl;
            uint16_t pos;
            int16_t speed, load;
            uint8_t volt, temp;
            
            err = driver.gripperGetStatus(pos, speed, load, volt, temp);
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
        } else {
            std::cout << "Invalid command. Use o, c, s, or q." << std::endl;
        }
    }

    driver.disconnect();
    std::cout << "Disconnected." << std::endl;
    return 0;
}
