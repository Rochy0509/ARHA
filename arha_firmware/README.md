# ARHA STM32H7 Firmware

Firmware for the ARHA (Autonomous Retirement Home Assistant) project, running on the STM32H723ZG microcontroller. This firmware provides real-time motor control capabilities via CAN bus with Ethernet connectivity for high-level system integration.

## Overview

This project implements a robust embedded system for motor control and communication, featuring MyActuator motor protocol support, dual-interface networking (CAN + Ethernet), and RTOS-based task management. The firmware is designed for robotics applications requiring precise actuator control and network integration.

## Hardware Platform

- **MCU**: STM32H723ZGT6
- **Core**: ARM Cortex-M7 @ 550 MHz
- **Memory**: 1 MB Flash, 564 KB RAM
- **Key Peripherals**:
  - FDCAN (CAN-FD) interface
  - Ethernet MAC with PHY (LAN8742)
  - CORDIC co-processor
  - FMAC (Filter Math Accelerator)

## STM32CubeMX Configuration

### Clock Configuration
- **Clock Source**: HSI (64 MHz)
- **PLL Configuration**: M=4, N=32, P=1, Q=4, R=2, FRACN=4096
- **SYSCLK**: 520 MHz (from PLL)
- **AHB Clock (HCLK)**: 260 MHz (SYSCLK / 2)
- **APB1**: 130 MHz (HCLK / 2)
- **APB2**: 130 MHz (HCLK / 2)
- **APB3**: 130 MHz (HCLK / 2)
- **APB4**: 130 MHz (HCLK / 2)
- **Voltage Scale**: Scale 0 (highest performance)
- **Power Supply**: LDO

### Peripherals
- **FDCAN1**: 
  - Mode: Classic CAN (non-FD)
  - Pins: PA11 (CAN1_RX), PA12 (CAN1_TX)
  - Nominal Prescaler: 10
  - Nominal Bit Time: Seg1=10, Seg2=2, SJW=1
  - Nominal Bitrate: ~1 Mbps
  - RX FIFO0: 8 elements
  - TX FIFO: 32 elements
  
- **ETH (Ethernet)**:
  - Interface: RMII
  - PHY: LAN8742A
  - Pins: PA1 (ETH_REF_CLK), PA2 (ETH_MDIO), PA7 (ETH_CRS_DV), PC1 (ETH_MDC), PC4 (ETH_RXD0), PC5 (ETH_RXD1), PG11 (ETH_TX_EN), PG13 (ETH_TXD0), PB13 (ETH_TXD1)
  - Hardware checksum offload enabled
  - MPU Configuration for DMA:
    - Region 0: 4GB, no access (default protection)
    - Region 1: 0x30000000, 256B, full access, bufferable (Ethernet descriptors)
    - Region 2: 0x30004000, 16KB, full access, shareable (LwIP heap)

- **CORDIC**: Enabled for trigonometric/hyperbolic calculations with DMA
- **FMAC**: Enabled with DMA support for filtering operations
- **USART3**: 115200 baud, 8N1 for debugging
- **DMA**: 4 streams (CORDIC read/write, FMAC read/write)
- **SWD Debug**: PA13 (SWDIO), PA14 (SWCLK), PB3 (SWO)

### Middleware
- **FreeRTOS**: 
  - API: CMSIS-RTOS v2
  - Tick Rate: 1 kHz
  - Heap: 25 KB
  - Scheduler: Preemptive
  - Max Priorities: 56
  
- **LwIP**: 
  - Version: v2.1.2
  - RTOS Integration: FreeRTOS
  - Heap: 14 KB (at 0x30004000)
  - Hardware checksum: Enabled
  - TCP MSS: 1460 bytes
  - PBUF Pool: 1536 bytes
  - Features: TCP, UDP, DHCP, DNS

## Features

### Currently Implemented

#### MyActuator Motor Protocol
The firmware includes a complete implementation of the MyActuator communication protocol. All commands are coded and available, with the following functions tested and verified:
- **Error Checking**: Diagnostics and fault detection (tested)
- **Status Monitoring**: Real-time retrieval of motor state information (tested)
- **Position Control**: Absolute, incremental, and single-turn positioning (tested)

Additional implemented commands include PID tuning, acceleration settings, encoder configuration, torque control, speed control, system operations, and motor shutdown/stop functions.

#### CAN Bus Communication
The FDCAN (CAN-FD) peripheral has been successfully configured and tested. The implementation in `fdcan.c` provides reliable communication with motor controllers over the CAN bus.

#### Ethernet Connectivity
Network connectivity is fully operational using the onboard Ethernet MAC with LAN8742 PHY. The LwIP TCP/IP stack has been integrated and tested, providing a foundation for higher-level network protocols. The Ethernet interface implementation can be found in the `LWIP/` directory.

#### Real-Time Operating System
FreeRTOS has been integrated to provide task scheduling and real-time capabilities. The multi-threaded architecture allows for concurrent operations, with task management implemented in `Core/Src/freertos.c`.

#### Hardware Acceleration
The STM32H7's hardware accelerators have been enabled:
- **CORDIC**: Available for trigonometric and hyperbolic calculations
- **FMAC**: Configured for digital filtering and signal processing operations

## Project Structure

```
arha_firmware/
├── Core/
│   ├── Inc/              # Header files
│   │   ├── main.h
│   │   ├── myactuator.h  # MyActuator protocol definitions
│   │   ├── fdcan.h       # CAN bus interface
│   │   └── FreeRTOSConfig.h
│   ├── Src/              # Source files
│   │   ├── main.c        # Main application
│   │   ├── myactuator.c  # Motor protocol implementation
│   │   ├── fdcan.c       # CAN driver
│   │   └── freertos.c    # RTOS tasks
│   └── Startup/          # MCU startup code
├── Drivers/
│   ├── CMSIS/            # ARM CMSIS libraries
│   ├── STM32H7xx_HAL_Driver/  # STM32 HAL drivers
│   └── BSP/
│       └── Components/
│           └── lan8742/  # Ethernet PHY driver
├── LWIP/                 # LwIP TCP/IP stack
│   ├── App/
│   └── Target/
│       └── ethernetif.c  # Ethernet interface
├── Middlewares/
│   └── Third_Party/
│       ├── FreeRTOS/     # RTOS kernel
│       └── LwIP/         # Network stack
└── Debug/                # Build output

```

## Development Environment

- **IDE**: STM32CubeIDE / STM32CubeMX
- **Toolchain**: ARM GCC
- **Build System**: Make
- **Configuration**: `.ioc` file for peripheral configuration

## Building the Project

1. Open the project in STM32CubeIDE
2. Build the project: `Project > Build All` (Ctrl+B)
3. Flash to target: `Run > Debug` (F11)

Alternatively, use the command line:
```bash
cd Debug/
make -j$(nproc)
```

## Current Status

The firmware has reached a functional milestone with the following components completed:
- CAN bus driver initialization and testing
- MyActuator protocol implementation (error check, status, position control)
- Ethernet MAC/PHY configuration
- LwIP network stack integration
- FreeRTOS task scheduling
- CORDIC and FMAC hardware acceleration

Current work focuses on communication testing and system integration validation.

## Future Work

### ROS2 Integration
Work is in progress to develop an Ethernet-based `ros2_control` hardware interface. This will bridge the firmware with ROS2's control framework, enabling:
- Direct control from high-level ROS2 planning systems
- Integration with the `ros2_control` framework
- Real-time joint state publishing
- Command reception via ROS2 topics and services

The Ethernet communication layer has been established and tested, providing the foundation for this integration. Protocol design and hardware interface architecture are currently being developed.

### Planned Enhancements
Several improvements are planned for future releases:
- Extended MyActuator protocol features (velocity control, torque control)
- Multi-motor management over CAN bus
- UDP/TCP command interface over Ethernet
- Real-time performance monitoring and logging
- Configuration storage in flash memory
- Bootloader for firmware updates over Ethernet

### CAN Bus
- **Interface**: FDCAN
- **Baud Rate**: Configured in `fdcan.c`
- **Purpose**: Motor control communication

### Ethernet
- **PHY**: LAN8742A
- **MAC**: STM32H7 integrated
- **Stack**: LwIP
- **Configuration**: See `LWIP/Target/lwipopts.h`

### Peripherals
- CORDIC: Hardware trigonometric acceleration
- FMAC: Digital filtering acceleration
- TIM: HAL timebase for FreeRTOS

## MyActuator Protocol

The firmware provides a complete MyActuator protocol implementation with all command functions coded. Tested functions include error checking, status monitoring (Status 1/2/3), and position control commands. Additional implemented commands cover:

**Parameter Configuration**
- PID tuning (current, speed, position loops)
- Acceleration/deceleration settings for motion planning
- CAN baud rate configuration

**Encoder Operations**
- Multi-turn and single-turn position reading
- Encoder zero offset configuration
- Absolute and incremental angle reading

**Motor Control**
- Torque closed-loop control
- Speed closed-loop control
- Position control (absolute, incremental, single-turn)
- Motor shutdown and stop commands

**System Management**
- Operating mode queries
- System reset and runtime monitoring

See `Core/Inc/myactuator.h` and `Core/Src/myactuator.c` for complete implementation details.

---

**Last Updated**: January 2026  
**Firmware Version**: 1.0-dev  
**Target Hardware**: STM32H723ZG Nucleo-144 or custom board