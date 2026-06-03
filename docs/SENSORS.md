# Sensor Specifications

## IMU Sensor: SparkFun LSM6DSV16X (Qwiic)

### Overview
6-DOF Inertial Measurement Unit (IMU) with integrated accelerometer and gyroscope on a Qwiic breakout board from SparkFun.

### Specifications

#### Accelerometer
- **Ranges**: ±2g, ±4g, ±8g, ±16g (selectable)
- **Resolution**: 16-bit output
- **Sensitivity**: ~61 mg/LSB @ ±16g
- **Sample Rate**: 15 Hz to 6.66 kHz (ODR configurable)
- **Accuracy**: ±60 mg (typical)
- **Output Noise**: ~4.5 mg/√Hz

#### Gyroscope
- **Ranges**: ±125°/s, ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
- **Resolution**: 16-bit output
- **Sensitivity**: ~4.375 mdps/LSB @ ±2000°/s
- **Sample Rate**: 15 Hz to 6.66 kHz (ODR configurable)
- **Output Noise**: ~0.007°/s/√Hz

#### Temperature Sensor
- **Range**: -40°C to +85°C
- **Accuracy**: ±2°C (typical)
- **Resolution**: 0.256°C/LSB

### Physical Specifications
- **Package**: QFN 14-pin
- **Size (Qwiic Board)**: 0.5" × 1.3"
- **Weight**: ~1g
- **Connector**: Qwiic (4-pin JST SH)

### Electrical Specifications
- **Operating Voltage**: 1.71V to 3.6V (typ 3.3V)
- **Current Draw**:
  - Active mode: ~6 mA (typical)
  - Low-power mode: ~0.6 mA
  - Sleep mode: <1 µA
- **Interface**: I2C (Qwiic)
  - I2C Address: 0x6A or 0x6B (selectable via SA0 pin)
  - Clock Speed: Up to 1 MHz

### Embedded Features
- 8KB FIFO (First In First Out) buffer
- Interrupts for data ready, FIFO full, FIFO threshold
- Self-test capability
- Embedded functions (high-pass filter, low-pass filter)
- Temperature compensation

### Data Output Format

```c
struct LSM6DSV16X_Data {
    int16_t accel_x;        // Acceleration X-axis (LSBs)
    int16_t accel_y;        // Acceleration Y-axis (LSBs)
    int16_t accel_z;        // Acceleration Z-axis (LSBs)
    int16_t gyro_x;         // Angular velocity X-axis (LSBs)
    int16_t gyro_y;         // Angular velocity Y-axis (LSBs)
    int16_t gyro_z;         // Angular velocity Z-axis (LSBs)
    int8_t temperature;     // Temperature (°C, from 25°C offset)
};
```

### Typical Operating Conditions (Recommended)
- **Accelerometer**: ±16g range, 1.66 kHz ODR
- **Gyroscope**: ±2000°/s range, 1.66 kHz ODR
- **Power Mode**: Normal (not low-power)

### Calibration
- **Accelerometer Self-Test**: Available (±1.5g amplitude)
- **Gyroscope Self-Test**: Available (±25°/s amplitude)
- **Offset Calibration**: User-implemented (zero-g/zero-rate)

### I2C Communication

**Write Transaction**:
```
START + Address(7-bit) + W + ACK + Register Address + ACK + Data + ACK + STOP
```

**Read Transaction**:
```
START + Address(7-bit) + R + ACK + Data + ACK/NACK + STOP
```

### Common Registers

| Register | Address | Description |
|----------|---------|-------------|
| CTRL1_XL | 0x10 | Accelerometer control register 1 |
| CTRL2_G | 0x11 | Gyroscope control register 1 |
| CTRL3_C | 0x12 | Control register 3 |
| OUTX_L_A | 0x28 | Accel X-axis LSB |
| OUTX_H_A | 0x29 | Accel X-axis MSB |
| OUTY_L_A | 0x2A | Accel Y-axis LSB |
| OUTY_H_A | 0x2B | Accel Y-axis MSB |
| OUTZ_L_A | 0x2C | Accel Z-axis LSB |
| OUTZ_H_A | 0x2D | Accel Z-axis MSB |
| OUTX_L_G | 0x22 | Gyro X-axis LSB |
| OUTX_H_G | 0x23 | Gyro X-axis MSB |
| OUTY_L_G | 0x24 | Gyro Y-axis LSB |
| OUTY_H_G | 0x25 | Gyro Y-axis MSB |
| OUTZ_L_G | 0x26 | Gyro Z-axis LSB |
| OUTZ_H_G | 0x27 | Gyro Z-axis MSB |
| OUT_TEMP_L | 0x20 | Temperature LSB |
| OUT_TEMP_H | 0x21 | Temperature MSB |

---

## Time-of-Flight Sensors: TDK InvenSense EV_MOD_ICU-10201-00 (x2)

### Overview
Acoustic (ultrasonic) time-of-flight range sensor module for contactless distance measurement. Dual sensors provide stereo triangulation capability.

### Specifications

#### Measurement Capabilities
- **Sensing Technology**: Acoustic time-of-flight (ToF)
- **Operating Frequency**: 100 kHz (ultrasonic)
- **Range**: 
  - Minimum: ~10 mm
  - Maximum: ~2000 mm (typical)
  - Accurate range: 50-1500 mm
- **Resolution**: ~1-3 mm (depends on surface)
- **Accuracy**: ±2% of reading (typical) or ±20 mm (whichever is larger)
- **Measurement Rate**: Up to 100 Hz (configurable)

#### Directional Characteristics
- **Beam Pattern**: Approximately conical, ~30° opening angle (TBD)
- **Typical Working Angle**: ±15° optimal
- **Frequency Response**: Calibrated for human skin and common mouse pad materials

### Physical Specifications
- **Module Dimensions**: ~20mm × 15mm × 10mm (estimated)
- **Weight**: <5g
- **Connector Type**: [To be confirmed from datasheet]
- **Mounting**: Mounting holes or adhesive backing (TBD)

### Electrical Specifications
- **Operating Voltage**: 3.3V ± 5%
- **Current Draw**:
  - Idle: ~5 mA
  - Active measurement: ~15-25 mA per sensor
  - Total (2 sensors): ~30-50 mA
- **Interface**: [To be confirmed - likely SPI, I2C, or UART]

### Communication Protocol

**[To be completed after datasheet review]**

Estimated frame format:
```
[Header] [Distance Data] [Signal Quality] [Ambient] [Temperature] [Checksum]
```

### Performance Parameters

#### Signal Quality Factors
- Surface reflectivity: Affects signal strength
- Temperature: Built-in compensation (TBD)
- Ambient lighting: Acoustic is immune to optical interference
- Surface distance: Affects signal return time

#### Typical Performance
- **Response Time**: ~5-10 ms per measurement
- **Jitter**: <5 mm RMS (typical)
- **Hysteresis**: <10 mm

### Dual Sensor Configuration

With two EV_MOD_ICU-10201-00 sensors:

**Stereo Triangulation**:
```
        Mouse Surface
        ┌─────────┐
        │  Left  │  Right
        │ Sensor │ Sensor
        │   ║      ║
        │   ▼      ▼
        └─────────┘
           │     │
           └─┬─┬─┘
             └─┘
           Distance measurements
           → Height calculation
           → Surface normal estimation
```

**Configuration Strategies**:
1. **Parallel Setup**: Both sensors pointed downward (redundancy)
2. **Angled Setup**: Sensors at complementary angles (triangulation)
3. **Sequential Setup**: Sensors for separate features (left/right tracking)

### Sensor Fusion with IMU

Combined sensor data:
```
IMU (6-DOF):
  - High-frequency motion tracking (relative positioning)
  - Acceleration and rotation rates
  
ToF (dual):
  - Absolute height measurement (reference point)
  - Surface proximity and contact detection
  - Gesture detection (rapid height changes)
```

### Calibration Procedures

#### Factory Calibration
- Baseline distance offset
- Temperature compensation coefficients
- Sensitivity scaling factors

#### User Calibration (On-Device)
- Zero-distance offset (sensor-to-surface)
- Temperature baseline (if operating in varying conditions)
- Signal strength normalization

### Operating Ranges

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Operating Temp | -10 | 25 | 60 | °C |
| Storage Temp | -20 | 25 | 85 | °C |
| Humidity | 0 | 50 | 95 | %RH |
| Supply Voltage | 3.135 | 3.3 | 3.465 | V |

---

## Haptic Scroll Strip (To Be Selected)

### Requirements Specification

#### Functional Requirements
- **Input Method**: Capacitive or resistive touch strip
- **Length**: 50-150 mm (TBD based on ergonomics)
- **Resolution**: ≥20 positions across strip
- **Gesture Support**: Swipe detection, multi-touch (optional)

#### Haptic Requirements
- **Feedback Types**: Vibration, pulse, pattern
- **Frequency Range**: 50-300 Hz (typical for haptic)
- **Intensity Levels**: ≥16 distinct levels
- **Response Time**: <50 ms from control to feedback

#### Physical Requirements
- **Form Factor**: Low-profile, integrated into mouse body
- **Durability**: ≥10 million touch cycles
- **Surface Material**: Smooth, ergonomic surface
- **Mounting**: Adhesive or mechanical, low-profile

#### Electrical Requirements
- **Operating Voltage**: 3.3V or 5V
- **Current Draw**: <100 mA total (strip + driver)
- **Interface**: SPI, I2C, or direct PWM/GPIO
- **Haptic Control**: PWM or direct feedback waveform

### Candidate Components to Evaluate

1. **Haptic Feedback Devices**:
   - Adafruit DRV2605L (I2C haptic motor driver)
   - Texas Instruments DRV8662 (H-bridge with PWM)
   - Linear Resonant Actuator (LRA) for tactile feedback

2. **Touch Sensing**:
   - Atmel QT series capacitive touch controller
   - MPR121 I2C capacitive touch sensor
   - Custom resistive strip with ADC

### Status
- 🔄 **Selection Pending**: Component evaluation and prototyping

---

## Integration Notes

### Power Sequencing
```
Power On:
1. Supply 3.3V to all sensors
2. Wait 10 ms for stabilization
3. Initialize I2C/SPI communication
4. Configure sensor modes
5. Start sensor readout loop
```

### Noise and EMI Considerations
- Acoustic sensors: Sensitive to electronic noise on power rails
- Digital communication: Twisted pair recommended for I2C/SPI
- Grounding: Star ground configuration preferred
- Filtering: 100nF bypass capacitors on all power inputs

### Thermal Management
- Operating temperature range: -10°C to +60°C
- Thermal drift compensation for ToF sensors
- User warning if temperature exceeds safe operating range

---

## References and Datasheets

- [SparkFun LSM6DSV16X Hookup Guide](https://learn.sparkfun.com/tutorials/lsm6dsv16x-hookup-guide)
- [LSM6DSV16X Datasheet](https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf)
- [TDK InvenSense EV_MOD_ICU-10201-00](https://www.invensense.com/) (datasheet to be obtained)
- [PJRC Teensy 4.1 Pin Description](https://www.pjrc.com/teensy/teensy41.html)

---

## Testing Procedures

### IMU Testing Checklist
- [ ] I2C communication verified
- [ ] Self-test passed (accelerometer and gyroscope)
- [ ] Output values reasonable (6DOF readable)
- [ ] No data dropout over 1-hour test
- [ ] Temperature measurement verified

### ToF Testing Checklist
- [ ] Serial/SPI communication verified
- [ ] Distance measurement within spec
- [ ] Signal quality stable over time
- [ ] Ambient measurements consistent
- [ ] Dual sensor synchronization verified

### Integration Testing
- [ ] IMU + ToF data fusion works
- [ ] Sensor data latency acceptable (<10ms)
- [ ] No I2C/SPI bus conflicts
- [ ] Power consumption within budget

