#include "buttons.h"

/*
 * ---------------------------------------------------------
 * Internal button state storage
 * ---------------------------------------------------------
 * These two instances hold all runtime state for Button A
 * and Button B. They are not exposed outside this file.
 */
static ButtonState g_buttonA;
static ButtonState g_buttonB;

/*
 * ---------------------------------------------------------
 * Helper: map ButtonId → GPIO pin
 * ---------------------------------------------------------
 * Keeps pin assignments centralized and avoids scattering
 * pin numbers throughout the code.
 */
static uint8_t buttonIdToPin(ButtonId id) {
  switch (id) {
    case ButtonId::A: return BUTTON_A_PIN;
    case ButtonId::B: return BUTTON_B_PIN;
    default:          return BUTTON_A_PIN; // fallback
  }
}

/*
 * ---------------------------------------------------------
 * Buttons::begin()
 * ---------------------------------------------------------
 * Initializes GPIO pins and seeds the internal state
 * machine with the current raw button levels.
 */
void Buttons::begin() {
  pinMode(BUTTON_A_PIN, INPUT_PULLUP);
  pinMode(BUTTON_B_PIN, INPUT_PULLUP);

  uint32_t now = millis();

  // *** THIS WAS BROKEN IN YOUR FILE — NOW FIXED ***
  auto initButton = [now](ButtonState& s, uint8_t pin) {
    bool rawDown = (digitalRead(pin) == LOW); // active‑low
    s.isDown        = rawDown;
    s.wasDown       = rawDown;
    s.longActive    = false;
    s.lastChangeMs  = now;
    s.downStartMs   = now;
    s.lastEvent     = ButtonEvent::None;
  };

  initButton(g_buttonA, BUTTON_A_PIN);
  initButton(g_buttonB, BUTTON_B_PIN);
}

/*
 * ---------------------------------------------------------
 * Buttons::update()
 * ---------------------------------------------------------
 * Called once per main loop. Updates both buttons using
 * the shared debounce + state machine logic.
 */
void Buttons::update() {
  uint32_t now = millis();
  updateButton(ButtonId::A, BUTTON_A_PIN, g_buttonA, now);
  updateButton(ButtonId::B, BUTTON_B_PIN, g_buttonB, now);
}

/*
 * ---------------------------------------------------------
 * Buttons::updateButton()
 * ---------------------------------------------------------
 * Core debounce + event generation logic.
 */
void Buttons::updateButton(ButtonId id, uint8_t pin, ButtonState& s, uint32_t nowMs) {
  s.lastEvent = ButtonEvent::None;

  bool rawDown = (digitalRead(pin) == LOW); // active‑low

  if (rawDown != s.isDown) {
    if (rawDown != s.wasDown) {
      s.lastChangeMs = nowMs;
      s.wasDown = rawDown;
    }

    if ((nowMs - s.lastChangeMs) >= BUTTON_DEBOUNCE_MS) {
      s.wasDown = s.isDown;
      s.isDown  = rawDown;

      if (s.isDown) {
        s.downStartMs = nowMs;
        s.longActive  = false;
        s.lastEvent   = ButtonEvent::Pressed;
      } else {
        uint32_t heldMs = nowMs - s.downStartMs;
        if (s.longActive || heldMs >= BUTTON_LONG_PRESS_MS) {
          s.lastEvent = ButtonEvent::LongPressed;
        } else {
          s.lastEvent = ButtonEvent::Clicked;
        }
      }
    }
  } else {
    if (s.isDown && !s.longActive) {
      uint32_t heldMs = nowMs - s.downStartMs;
      if (heldMs >= BUTTON_LONG_PRESS_MS) {
        s.longActive = true;
      }
    }
  }
}

/*
 * ---------------------------------------------------------
 * Buttons::getEvent()
 * ---------------------------------------------------------
 */
ButtonEvent Buttons::getEvent(ButtonId id) {
  switch (id) {
    case ButtonId::A: return g_buttonA.lastEvent;
    case ButtonId::B: return g_buttonB.lastEvent;
    default:          return ButtonEvent::None;
  }
}

/*
 * ---------------------------------------------------------
 * Buttons::isDown()
 * ---------------------------------------------------------
 */
bool Buttons::isDown(ButtonId id) {
  switch (id) {
    case ButtonId::A: return g_buttonA.isDown;
    case ButtonId::B: return g_buttonB.isDown;
    default:          return false;
  }
}
