# Hardware Setup Guide - Proof of Concept

## Overview

This guide covers the physical setup and wiring of the **Project Rat proof-of-concept prototype** using the PJRC Teensy 4.1 microcontroller and readily available sensor breakout boards.

**Note**: This is a development and validation setup. The final product will use custom integrated hardware.

## Bill of Materials (PoC)

### Core Components
| Component | Part Number | Qty | Notes |
|-----------|------------|-----|-------|
| Microcontroller | PJRC Teensy 4.1 | 1 | USB Native support |
| IMU Sensor | SparkFun LSM6DSV16X (Qwiic) | 1 | 6-DOF motion tracking |
| ToF Sensor | TDK InvenSense EV_MOD_ICU-10201-00 | 2 | Acoustic rangefinder |
| Qwiic Cable | Standard Qwiic | 1 | For IMU connection |

### Supporting Components
- USB Type-C cable (for programming and power)
- Breadboard or custom PCB
- Jumper wires (22 AWG recommended)
- 100nF bypass capacitors (for each sensor)
- Optional: Level shifters if needed for ToF interface

## Teensy 4.1 Pinout

```
                    ┌─────────────────────┐
                    │   TEENSY 4.1        │
        USB_DP ──── │ ① USB_DP        GND │ ──── GND (pin 39)
        USB_DN ──── │ ② USB_DN        GND │ ──── GND (pin 38)
         GND ────── │ ③ GND           3V3 │ ──── 3.3V (pin 37)
         VIN ────── │ ④ VIN           GND │ ──── GND (pin 36)
         GND ────── │ ⑤ GND           22  │ ──── GPIO 22 (pin 35)
          0 ────── │ ⑥ 0             21  │ ──── GPIO 21 (pin 34)
          1 ────── │ ⑦ 1             20  │ ──── GPIO 20 (pin 33)
          2 ────── │ ⑧ 2             19  │ ──── I2C SCL (pin 32)
          3 ────── │ ⑨ 3             18  │ ──── I2C SDA (pin 31)
          4 ────── │ ⑩ 4             17  │ ──── GPIO 17 (pin 30)
         GND ────── │ ⑪ GND           16  │ ──── GPIO 16 (pin 29)
          5 ────── │ ⑫ 5             15  │ ──── GPIO 15 (pin 28)
          6 ────── │ ⑬ 6             14  │ ──── GPIO 14 (pin 27)
          7 ────── │ ⑭ 7             13  │ ──── GPIO 13 (pin 26) [LED]
          8 ────── │ ⑮ 8             12  │ ──── GPIO 12 (pin 25)
          9 ────── │ ⑯ 9             11  │ ──── GPIO 11 (pin 24)
         10 ────── │ ⑰ 10            GND │ ──── GND (pin 23)
                    └─────────────────────┘
                    Back view
```

## Wiring Diagrams

### IMU Sensor (SparkFun LSM6DSV16X - Qwiic)

The Qwiic connector provides I2C communication. If your Teensy 4.1 has a Qwiic connector, connect directly. Otherwise, use jumper wires:

```
Qwiic Connector Pinout:
┌─ GND (Black)
├─ 3.3V (Red)
├─ SDA (Blue)
└─ SCL (Yellow)

Teensy 4.1 Connection:
GND (Black)  → GND (any pin)
3.3V (Red)   → 3V3 (pin 37)
SDA (Blue)   → Pin 18 (I2C SDA)
SCL (Yellow) → Pin 19 (I2C SCL)
```

### ToF Sensors (TDK InvenSense EV_MOD_ICU-10201-00)

**⚠️ Important**: The exact pinout and interface (I2C/SPI/UART) must be confirmed from the official datasheet before wiring.

**Preliminary Setup** (assuming SPI interface - **verify with datasheet**):

```
ToF Sensor #1:
GND    → GND
VCC    → 3.3V
MOSI   → Pin 11 (MOSI)
MISO   → Pin 12 (MISO)
SCK    → Pin 13 (SCK)
CS     → Pin 10 (Chip Select)

ToF Sensor #2 (if using same bus):
GND    → GND
VCC    → 3.3V
MOSI   → Pin 11 (MOSI)
MISO   → Pin 12 (MISO)
SCK    → Pin 13 (SCK)
CS     → Pin 9 (separate CS for sensor #2)
```

**Alternative** (if I2C interface - **verify with datasheet**):

```
ToF Sensor #1:
GND    → GND
VCC    → 3.3V
SDA    → Pin 18 (I2C SDA)
SCL    → Pin 19 (I2C SCL)
ADDR   → GND or 3.3V (to set I2C address)

ToF Sensor #2 (I2C with address configuration):
GND    → GND
VCC    → 3.3V
SDA    → Pin 18 (I2C SDA)
SCL    → Pin 19 (I2C SCL)
ADDR   → Opposite of Sensor #1
```

## Power Considerations

### Current Draw (PoC)
- **Teensy 4.1**: ~100 mA typical (at 600 MHz)
- **LSM6DSV16X**: ~6 mA typical
- **ToF Sensors (x2)**: ~50-100 mA total (depends on mode)
- **Total**: ~200-250 mA estimated

### Power Supply Options

1. **USB Power** (Recommended for PoC): 
   - Connect Teensy via USB Type-C
   - Provides 500 mA @ 5V (negotiable)
   - Simplest for development and testing

2. **Battery Power** (For integration):
   - LiPo battery: 1S (3.7V) or 2S (7.4V) with 3.3V regulator
   - Ensure sufficient current capacity (≥500 mA recommended)

## Bypass Capacitors

Add 100nF ceramic capacitors close to the power pins of each sensor:

```
Sensor VCC ──┬── 3.3V
             │
            ===  100nF ceramic
             │
            GND
```

## I2C Bus Configuration

If using I2C for multiple sensors:

```
         ┌─────────────────┐
Teensy   │ SDA (Pin 18)    │◄────────┬── LSM6DSV16X SDA
4.1      │ SCL (Pin 19)    │◄────┐   │
         │                 │      ├──┤ (with pull-up resistors)
         │ GND             │      │   │
         └─────────────────┘      │   │
                                  │   │
              I2C Pull-ups:        │   │
              ┌─ 4.7kΩ ─ 3.3V ────┴────┤ ToF Sensor SDA
              └─ 4.7kΩ ─ 3.3V ───────── ToF Sensor SCL
```

**Note**: Pull-up resistors may already be on the sensor breakout boards.

## Testing the Connection

### 1. Visual Inspection
- Verify secure connections
- Check for bent pins
- Confirm correct power supply polarity

### 2. I2C Bus Scan (Firmware Test)
```c
// Upload this to Teensy to scan I2C bus
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  Serial.println("Scanning I2C bus...");
  for (uint8_t addr = 8; addr < 120; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.println(addr, HEX);
    }
  }
  delay(2000);
}
```

### 3. Verify Power
- Check 3.3V rail voltage: Should be ≥ 3.0V
- Check GND continuity
- Use multimeter to verify all connections

## Schematic Template (PoC)

Below is a simplified schematic layout:

```
┌─────────────────────────────────────────────────────────┐
│                    TEENSY 4.1                           │
│  ┌──────────────────────────────────────────────────┐   │
│  │ USB_DP  USB_DN  GND  VIN  GND (39,38,37,36,35) │   │
│  │                                                  │   │
│  │  I2C:   Pin 18 (SDA)   Pin 19 (SCL)             │   │
│  │         │                 │                      │   │
│  │         ├─────────────────┤                      │   │
│  │         │                 │                      │   │
│  │  3V3 ──(┘                 └)── GND               │   │
│  └──────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
         │                    │
    ┌────▼────┐         ┌─────▼─────┐
    │          │         │           │
    │LSM6DSV16X│         │ ToF Sens. │
    │  (Qwiic) │         │  (x2)     │
    │          │         │           │
    └──────────┘         └───────────┘
```

## Setup Checklist

- [ ] Teensy 4.1 board obtained
- [ ] SparkFun LSM6DSV16X sensor obtained
- [ ] TDK InvenSense EV_MOD_ICU-10201-00 sensors (x2) obtained
- [ ] Teensy connected to computer via USB Type-C
- [ ] IMU sensor wired to Teensy I2C pins
- [ ] ToF sensors wired according to datasheet
- [ ] Bypass capacitors installed
- [ ] I2C bus scan test passed
- [ ] Power supply verified (3.3V stable)
- [ ] Ready to flash firmware

## Troubleshooting

### I2C Not Detecting Device
- Check wiring and connections
- Verify pull-up resistors (4.7kΩ typical)
- Check I2C address with I2C scan sketch
- Verify 3.3V supply voltage

### Intermittent Data
- Add bypass capacitors if not present
- Check for loose connections
- Reduce I2C clock speed if needed
- Move away from RF interference

### Power Supply Issues
- Measure 3.3V rail voltage
- Check USB power current limit
- Verify regulator output
- Check for shorts

### ToF Sensor Communication
- Confirm interface (SPI vs I2C) from datasheet
- Verify CS pin pulled high (if SPI)
- Check for address conflicts (if I2C)
- Confirm sensor is powered

## Resources

- [Teensy 4.1 Documentation](https://www.pjrc.com/teensy/teensy41.html)
- [SparkFun LSM6DSV16X Hookup Guide](https://learn.sparkfun.com/tutorials/lsm6dsv16x-hookup-guide)
- [TDK InvenSense Documentation](https://www.invensense.com/)
- [Arduino Wire Library Reference](https://www.arduino.cc/reference/en/language/functions/communication/wire/)
- [Teensy Serial Library](https://www.pjrc.com/teensy/td_serial.html)

## Next Steps

1. **Assemble Hardware**: Wire up all components according to this guide
2. **Test Communication**: Run I2C scan to verify sensor detection
3. **Flash Firmware**: Upload sensor test firmware to Teensy
4. **Validate Sensors**: Verify sensor data is reading correctly
5. **Develop Algorithms**: Implement sensor fusion and gesture recognition

---

**Note**: This is the proof-of-concept hardware setup. The final production mouse will use integrated custom hardware optimized for size, power, and performance.

