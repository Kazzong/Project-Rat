#include "piezo_strip.h"
#include "config.h"

static const float PIEZO_BASELINE_ALPHA = 0.01f;

PiezoStrip::PiezoStrip(uint8_t analogPin)
    : _analogPin(analogPin), _lastRaw(0), _baseline(512.0f) {}

void PiezoStrip::begin() {
  pinMode(_analogPin, INPUT);
  _lastRaw = analogRead(_analogPin);
  _baseline = (float)_lastRaw;
}

uint16_t PiezoStrip::readRaw() {
  _lastRaw = analogRead(_analogPin);
  _baseline = _baseline * (1.0f - PIEZO_BASELINE_ALPHA) + _lastRaw * PIEZO_BASELINE_ALPHA;
  return _lastRaw;
}

float PiezoStrip::getMagnitude() const {
  return fabsf((float)_lastRaw - _baseline);
}
