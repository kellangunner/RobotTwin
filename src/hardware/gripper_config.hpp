// Calibration for the hobby-servo gripper axis, split out of hardware_config.hpp
// so it can travel to an 8-bit target.
//
// The gripper does not run on the ESP32 any more: an Arduino Uno drives the
// SG-90 from its own supply and the ESP32 only relays commands to it (see
// docs/wiring-and-bringup.md). That Uno runs the very same rt::GripperAxis as
// the twin and the native tests — which it can only do if the types it needs
// are free of <string>, <array>, and the YAML reader that hardware_config.hpp
// pulls in. Hence this header: a plain aggregate, no includes, no allocation.
//
// Keep it that way. Anything added here has to compile for an ATmega328P.
#pragma once

namespace rt {

// Pulse widths are stored in seconds like every other time in the core; the
// YAML states them in microseconds because that is how servo datasheets do.
// closedPulse may be greater than openPulse — that is simply a servo mounted
// the other way round.
struct GripperConfig {
  int pin;              // Arduino Uno pin carrying the servo signal
  int relayPin;         // ESP32 GPIO that relays commands to the Uno (UART TX)
  int relayRxPin;       // Arduino pin receiving that link (SoftwareSerial RX)
  int relayBaud;        // baud of that one-way link
  int knobPin;          // Uno analog pin reading the manual opening knob
  double pwmHz;
  double closedPulse;   // s — pulse width with the jaws shut (opening 0)
  double openPulse;     // s — pulse width at maxOpening
  double maxOpening;    // m — jaw opening (pad face to pad face) at openPulse
  double speed;         // m/s — commanded jaw slew rate
  double bootOpening;   // m — opening the jaws are assumed to sit at on power-up
  double releaseAfter;  // s of stillness before the PWM stops; 0 = drive forever
};

} // namespace rt
