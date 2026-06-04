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
    int16_t gyro_x;         // Angular velocity X-axis (LSBs)
    int16_t gyro_y;         // Angular velocity Y-axis (LSBs)
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
| OUTX_L_G | 0x22 | Gyro X-axis LSB |
| OUTX_H_G | 0x23 | Gyro X-axis MSB |
| OUTY_L_G | 0x24 | Gyro Y-axis LSB |
| OUTY_H_G | 0x25 | Gyro Y-axis MSB |
| OUT_TEMP_L | 0x20 | Temperature LSB |
| OUT_TEMP_H | 0x21 | Temperature MSB |

---

## Piezo Strip Sensor: Generic piezo vibration strip

### Overview
Passive vibration pickup for texture and surface topology sensing. The piezo strip is sampled by the Teensy analog input and used to detect surface material, texture, and contact events.

### Specifications

#### Measurement Capabilities
- **Sensing Technology**: Passive piezo vibration sensing
- **Interface**: Analog input (A0)
- **Sensitivity**: Dependent on amplifier/buffer stage and biasing
- **Sampling Rate**: Limited by ADC sampling and firmware loop rate
- **Measurement Output**: Relative vibration magnitude and transient events

#### Physical Characteristics
- **Sensor Type**: Flexible piezo strip or piezo film
- **Typical Size**: 20-100 mm length (customizable)
- **Weight**: Minimal (<5 g)
- **Connector Type**: Analog signal output
- **Mounting**: Adhesive backing or mechanical clamp

### Electrical Specifications
- **Operating Voltage**: Passive sensor; analog front-end uses 3.3V reference
- **Current Draw**: Passive sensor only, amplifier/buffer dependent
- **Interface**: One analog input GPIO, high-impedance stage recommended

### Signal Conditioning
- Use a high-impedance amplifier or buffer to preserve the piezo waveform.
- Add a bias network if the piezo output is AC-coupled.
- Protect the analog input from large transients with a small series resistor and clamp diodes if needed.

### Performance Parameters

#### Signal Quality Factors
- Surface texture: Determines vibration signature
- Contact pressure: Modulates signal amplitude
- Sensor mounting: Affects mechanical coupling and sensitivity
- Noise: Minimize by using shielded wiring and stable ground

#### Typical Performance
- **Response Time**: Nearly instantaneous at the sensor; limited by ADC sampling
- **Resolution**: Depends on ADC resolution and sensor conditioning
- **Repeatability**: Best with a stable mounting and consistent bias network

### Sensor Fusion with IMU

Combined sensor data:
```
IMU (6-DOF):
  - High-frequency motion tracking (relative positioning)
  - X/Y acceleration and rotation rates only
  - Relative motion

Piezo strip:
  - Surface texture and vibration magnitude
  - Contact and texture event detection
  - Topological signatures for surface mapping
```

### Calibration Procedures

#### Factory Calibration
- Baseline voltage offset
- Bias network calibration
- Sensitivity scaling factors

#### User Calibration (On-Device)
- Noise floor estimation
- Surface texture normalization
- Signal amplitude threshold tuning

### Operating Ranges

| Parameter | Min | Typ | Max | Unit |
|-----------|-----|-----|-----|------|
| Operating Temp | -10 | 25 | 60 | °C |
| Storage Temp | -20 | 25 | 85 | °C |
| Humidity | 0 | 50 | 95 | %RH |
| Supply Voltage | 3.135 | 3.3 | 3.465 | V |

---

## Haptic Scroll Strip (To Be Selected)

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
- Piezo sensors: Sensitive to mechanical noise and microphonic coupling
- Digital communication: Twisted pair recommended for I2C/SPI
- Grounding: Star ground configuration preferred
- Filtering: 100nF bypass capacitors on all power inputs

### Thermal Management
- Operating temperature range: -10°C to +60°C
- Drift compensation for piezo baseline changes
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

### Piezo Strip Testing Checklist
- [ ] Analog input signal verified
- [ ] Vibration magnitude responds to surface texture
- [ ] Signal conditioning stable over time
- [ ] Baseline noise floor consistent
- [ ] Event detection reliable during motion

### Integration Testing
- [ ] IMU + piezo data fusion works
- [ ] Sensor data latency acceptable (<10ms)
- [ ] No I2C/SPI bus conflicts
- [ ] Power consumption within budget

