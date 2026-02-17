# arha_tcp_driver

> ⚠️ **This package is still under active development.** API and protocol may change without notice.

C++ driver library for communicating with the ARHA robot's STM32H723ZG controller over TCP/IP. It provides a limb-based API to command and read the state of MyActuator RMD motors (position, velocity, effort) through a custom binary protocol.

## Architecture

```
  PC (this driver)              STM32H723ZG                 Motors
 ┌──────────────┐   TCP/IP    ┌─────────────┐    CAN bus   ┌───────┐
 │ arhaTCPDriver │───────────▶│  LWIP +      │────────────▶│  RMD  │
 │  (C++ lib)   │◀───────────│  FreeRTOS    │◀────────────│ X6/X8 │
 └──────────────┘  port 5000  └─────────────┘              └───────┘
```

The driver sends framed binary packets to the STM32, which routes commands to the correct CAN channel based on the limb name. Motor IDs can repeat across limbs since each limb maps to a separate CAN bus.

### Wire Protocol

Each packet has the following structure:

```
[0xAA] [CMD] [LEN_LO] [LEN_HI] [PAYLOAD...] [XOR_CHECKSUM] [0x55]
```

- All multi-byte values are **little-endian**
- Motor values (position, velocity) are **IEEE 754 float32 in radians**
- Effort values are **float32 in Amps** (firmware converts to 0.01A/LSB for CAN)

## Building

The package uses plain CMake (not ament):

```bash
colcon build --packages-select arha_tcp_driver
```

Or standalone:

```bash
mkdir build && cd build
cmake ..
make
```

## Quick Start

```cpp
#include "arha_tcp.hpp"
using namespace arha_tcp_driver;

int main() {
    DriverConfig config;
    config.ip_address = "192.168.197.123";
    config.port = 5000;

    arhaTCPDriver driver(config);
    driver.registerLimb({"right_arm", {1, 2, 3, 4, 5, 6}});
    driver.connect();
    driver.enableLimbMotors("right_arm", true);

    // Read all joint states
    std::vector<double> pos, vel, eff;
    driver.getStates("right_arm", pos, vel, eff);

    // Command a single joint position (radians)
    driver.setPosition("right_arm", 0, 0.5236);

    // Command effort on a single joint (Amps)
    driver.setEffort("right_arm", 5, 0.4);

    // Batch position command
    driver.setPositions("right_arm", {0.0, 0.5, 0.0, 0.0, 0.0, 0.0});

    driver.enableLimbMotors("right_arm", false);
    driver.disconnect();
}
```

## API Overview

| Method | Description |
|---|---|
| `registerLimb(LimbConfig)` | Register a limb with its motor IDs |
| `connect()` / `disconnect()` | Manage TCP connection to STM32 |
| `setPositions(limb, values)` | Set all joint positions (rad) |
| `setVelocities(limb, values)` | Set all joint velocities (rad/s) |
| `setEfforts(limb, values)` | Set all joint efforts (Amps) |
| `getStates(limb, pos, vel, eff)` | Read all joint states |
| `setPosition(limb, index, val)` | Set single joint position |
| `setEffort(limb, index, val)` | Set single joint effort |
| `enableLimbMotors(limb, enable)` | Enable/disable motors on a limb |
| `emergencyStop()` | Stop all motors immediately |

## Test Programs

- **`test_driver`** — Position control test: moves motors to a target angle and monitors convergence.
- **`torque_test`** — Effort control test: applies a constant torque to a motor while holding another at its initial position.

```bash
# After building, from the workspace:
./build/arha_tcp_driver/test_driver [ip] [port]
./build/arha_tcp_driver/torque_test [ip] [port] [torque_Nm]
```
