#ifndef PIEZO_STRIP_H
#define PIEZO_STRIP_H

#include <Arduino.h>

class PiezoStrip {
public:
  explicit PiezoStrip(uint8_t analogPin);

  void begin();
  uint16_t readRaw();
  float getMagnitude() const;

private:
  uint8_t _analogPin;
  uint16_t _lastRaw;
  float _baseline;
};

#endif // PIEZO_STRIP_H
