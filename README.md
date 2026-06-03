# Project Rat 🐀

An innovative computer mouse prototype featuring advanced sensor technology and haptic feedback.

## Overview

Project Rat is a next-generation mouse design that combines:

- **IMU Sensor (6-DOF)** - For precise motion tracking and acceleration measurement
- **Acoustic Time-of-Flight Sensors** - For contactless surface tracking and cursor positioning
- **Haptic Scroll Strip** - Tactile feedback-enabled scroll input replacing traditional scroll wheels

## Development Status

### Current Phase: Proof-of-Concept Prototype 🚀

This repository contains the firmware and driver code for the **proof-of-concept prototype** using readily available development hardware:

| Component | Current (PoC) | Final Product |
|-----------|--------------|---------------|
| Microcontroller | PJRC Teensy 4.1 | *Custom SoC (TBD)* |
| IMU | SparkFun LSM6DSV16X | *High-precision industrial IMU* |
| ToF Sensor | TDK InvenSense EV_MOD_ICU-10201-00 (x2) | *Integrated custom sensor* |
| Haptic Strip | *To be selected* | *Custom haptic actuator* |
| Form Factor | Development board | *Ergonomic mouse form* |

**Goal**: Validate core sensor fusion algorithms, gesture recognition, and haptic feedback concepts before committing to custom hardware design.

## Key Features

- High-precision 6-DOF motion tracking (IMU)
- Dual acoustic ToF sensors for robust surface tracking and height measurement
- Haptic feedback for scroll interactions
- Native USB HID support via Teensy 4.1
- Low-latency sensor fusion and gesture recognition
- Modular sensor abstraction for easy hardware swapping

## Bill of Materials (Current PoC)

| Component | Part Number | Quantity | Status |
|-----------|------------|----------|--------|
| Microcontroller | PJRC Teensy 4.1 | 1 | ✅ |
| IMU Sensor | SparkFun LSM6DSV16X (Qwiic) | 1 | ✅ |
| ToF Sensor | TDK InvenSense EV_MOD_ICU-10201-00 | 2 | ✅ |
| Haptic Strip | *To be selected* | 1 | 🔄 |
| Haptic Driver | *To be selected* | 1 | 🔄 |
| Supporting Components | Resistors, capacitors, connectors | Various | 🔄 |

## Project Structure

```
Project-Rat/
├── src/                    # Source code
│   ├── firmware/          # Teensy 4.1 firmware (PoC)
│   │   ├── hal/           # Hardware abstraction layer
│   │   ├── sensors/       # Sensor drivers
│   │   ├── usb/           # USB HID interface
│   │   └── main.c
│   ├── drivers/           # Host drivers (future)
│   ├── application/       # Application logic
│   └── utils/             # Utility functions
├── hardware/              # Hardware documentation
│   ├── poc/              # PoC prototype schematics
│   ├── final/            # Final product designs (future)
│   └── models/
├── docs/                  # Documentation
│   ├── ARCHITECTURE.md    # System architecture
│   ├── SENSORS.md         # Sensor specifications (current PoC)
│   ├── API.md             # USB HID API reference
│   └── HARDWARE.md        # PoC hardware setup guide
├── tests/                 # Test suites
├── tools/                 # Build tools and utilities
└── examples/              # Example code and usage

```

## Getting Started

### PoC Prototype Setup

#### Prerequisites

- PJRC Teensy 4.1 board
- SparkFun LSM6DSV16X (Qwiic)
- TDK InvenSense EV_MOD_ICU-10201-00 (x2)
- Teensy Loader
- Arduino IDE or PlatformIO
- USB Type-C cable for programming

#### Hardware Setup

See [HARDWARE.md](docs/HARDWARE.md) for complete wiring instructions and pinout diagrams.

Quick connections:
- **IMU (Qwiic)**: Connect to Teensy I2C (pins 18/19)
- **ToF Sensors**: Connection via SPI or I2C (TBD based on datasheet review)

#### Building

```bash
# Clone the repository
git clone https://github.com/Kazzong/Project-Rat.git
cd Project-Rat

# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build
make
```

#### Flashing Firmware to Teensy 4.1

```bash
# After building, the firmware will be in build/src/firmware/
# Use Teensy Loader to upload the .hex file to your Teensy 4.1
```

## Architecture

The proof-of-concept uses a modular architecture designed to be hardware-agnostic:

```
┌─────────────────────────────────────────────────────────┐
│                    Host System                          │
│              (Windows/Mac/Linux)                        │
└────────────────────────┬────────────────────────────────┘
                         │
                    USB HID Interface
                         │
        ┌────────────────┼────────────────┐
        │                │                │
   ┌────▼──────┐  ┌──────▼──────┐  ┌─────▼──────┐
   │ Movement  │  │   Scroll    │  │   Haptic   │
   │  Events   │  │   Events    │  │  Feedback  │
   └────▲──────┘  └──────▲──────┘  └─────▲──────┘
        │                │                │
        └────────────────┼────────────────┘
                         │
        ┌────────────────▼────────────────┐
        │   Firmware (Teensy 4.1 - PoC)   │
        │  - Sensor fusion                │
        │  - Gesture recognition          │
        │  - USB HID stack                │
        └────┬───────────┬───────────┬────┘
             │           │           │
        ┌────▼──┐  ┌─────▼──┐  ┌────▼──────┐
        │  IMU  │  │  ToF   │  │  Haptic   │
        │Sensor │  │Sensors │  │  Strip    │
        │(PoC)  │  │ (PoC)  │  │  (TBD)    │
        └───────┘  └────────┘  └───────────┘
```

## Hardware Components (PoC)

### Microcontroller: PJRC Teensy 4.1
- **CPU**: ARM Cortex-M7 at 600 MHz
- **Memory**: 512 KB RAM, 1984 KB Flash
- **USB**: Native USB Device support (perfect for HID mouse)
- **I2C/SPI**: Multiple interfaces for sensor integration
- **Rationale**: Rapid prototyping, mature development ecosystem, native USB support

### IMU: SparkFun LSM6DSV16X (Qwiic)
- **Type**: 6-DOF (3-axis accelerometer + 3-axis gyroscope)
- **Interface**: I2C (Qwiic connector)
- **Sample Rate**: Up to 6.66 kHz
- **Rationale**: Easy prototyping with Qwiic connector, proven accuracy

### ToF Sensors: TDK InvenSense EV_MOD_ICU-10201-00 (x2)
- **Type**: Acoustic time-of-flight rangefinder
- **Range**: ~10-2000 mm typical
- **Frequency**: 100 kHz ultrasonic
- **Rationale**: Contactless operation, suitable for height/proximity sensing

## Documentation

- [ARCHITECTURE.md](docs/ARCHITECTURE.md) - System design overview
- [SENSORS.md](docs/SENSORS.md) - Sensor specifications (current PoC components)
- [API.md](docs/API.md) - USB HID API reference
- [HARDWARE.md](docs/HARDWARE.md) - PoC hardware setup and pinout guide

## Development Roadmap

### Phase 1: PoC Hardware Validation ✅ In Progress
- [x] Select accessible development components
- [x] Document hardware connections
- [ ] Assemble prototype
- [ ] Verify sensor communication
- [ ] Validate basic sensor data

### Phase 2: Core Firmware (Current Focus)
- [ ] Sensor drivers (IMU, ToF)
- [ ] USB HID device stack
- [ ] Basic cursor tracking
- [ ] Sensor fusion algorithms

### Phase 3: Advanced Features
- [ ] Gesture recognition
- [ ] Haptic feedback control
- [ ] Sensitivity calibration
- [ ] Profile management

### Phase 4: Validation & Final Design
- [ ] Performance benchmarking
- [ ] User testing with PoC
- [ ] Custom hardware specification
- [ ] Production design handoff

## Contributing

Contributions are welcome! This is an active proof-of-concept project seeking community input on:
- Sensor selection and integration
- Firmware optimization
- Algorithm improvements
- Hardware design feedback

Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact

For questions or suggestions about the PoC or final design, please open an issue on GitHub.

---

**Status**: Early Development - PoC Firmware Development 🚀

**Note**: This is a proof-of-concept using development hardware. Final product will feature integrated custom hardware optimized for performance, size, and cost.
