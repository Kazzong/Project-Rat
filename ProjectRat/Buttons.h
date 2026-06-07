#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include "config.h"

/*
 * ---------------------------------------------------------
 * ButtonId
 * ---------------------------------------------------------
 * Logical identifiers for your two physical buttons.
 * These are NOT tied to left/right shells or orientation.
 * They simply represent:
 *
 *   Button A  → primary action
 *   Button B  → secondary action
 *
 * The mapping to GPIO pins is handled in buttons.cpp.
 */
enum class ButtonId : uint8_t {
  A = 0,
  B = 1
};

/*
 * ---------------------------------------------------------
 * ButtonEvent
 * ---------------------------------------------------------
 * These are the high-level, debounced events generated
 * by the button subsystem each frame.
 *
 * None         → no new event
 * Pressed      → clean rising edge (debounced)
 * Released     → clean falling edge (debounced)
 * Clicked      → short press + release
 * LongPressed  → long press (reported on release)
 *
 * This abstraction allows HID logic to remain clean.
 */
enum class ButtonEvent : uint8_t {
  None = 0,
  Pressed,
  Released,
  Clicked,
  LongPressed
};

/*
 * ---------------------------------------------------------
 * ButtonState
 * ---------------------------------------------------------
 * Internal per-button state machine.
 *
 * isDown         → current debounced state
 * wasDown        → previous debounced state
 * longActive     → true once long-press threshold crossed
 * lastChangeMs   → last time the *raw* signal changed
 * downStartMs    → timestamp when button was debounced-pressed
 * lastEvent      → event generated during the last update()
 *
 * This struct is private to the subsystem and not exposed
 * to the rest of the firmware.
 */
struct ButtonState {
  bool isDown;
  bool wasDown;
  bool longActive;
  uint32_t lastChangeMs;
  uint32_t downStartMs;
  ButtonEvent lastEvent;
};

/*
 * ---------------------------------------------------------
 * Buttons (static subsystem)
 * ---------------------------------------------------------
 * A fully self-contained, production-grade button handler.
 *
 * Usage:
 *   Buttons::begin();   // in setup()
 *   Buttons::update();  // once per loop()
 *
 * Query events:
 *   ButtonEvent e = Buttons::getEvent(ButtonId::A);
 *
 * Query continuous state:
 *   bool held = Buttons::isDown(ButtonId::A);
 *
 * This keeps all button logic out of your main .ino file.
 */
class Buttons {
public:
  // Initialize GPIO pins and internal state
  static void begin();

  // Update debounce logic and generate events
  static void update();

  // Retrieve the most recent event for a button
  static ButtonEvent getEvent(ButtonId id);

  // Check if a button is currently held (debounced)
  static bool isDown(ButtonId id);

private:
  // Internal helper for updating a single button
  static void updateButton(ButtonId id, uint8_t pin, ButtonState& s, uint32_t nowMs);
};

#endif // BUTTONS_H
