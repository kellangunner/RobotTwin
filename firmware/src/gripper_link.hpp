// The ESP32 half of the gripper axis: a one-way serial link to the Arduino
// that actually drives the SG-90.
//
// Replaces the LEDC channel this board used to own. The servo moved off the
// ESP32 for two reasons — a stalled SG-90 pulls ~650 mA, which is exactly the
// transient that browns out a board generating step pulses, and a hobby servo
// wants 5 V logic rather than the ESP32's marginal 3.3 V. See
// docs/wiring-and-bringup.md.
//
// Deliberately thin, exactly like the driver it replaces. All of the
// mechanism's arithmetic (opening → pulse width, slew rate limiting, idle
// release) still lives in rt::GripperAxis, which is part of the shared core and
// unit tested natively. The Arduino runs that same class; this side only names
// a destination opening and lets it get there.
//
// Wire format, newline terminated, ASCII so a human can read it in a terminal:
//   G <mm>   go to this opening
//   R        release — stop driving, let the servo go slack
//
// Nothing comes back. The link is a single wire plus a shared ground, so the
// ESP32 cannot know whether the Arduino is powered, listening, or even present.
// That is the accepted cost of the split: the Arduino echoes what it receives on
// its own USB console, which is where you look during bring-up.
#pragma once

#include "hardware/gripper_config.hpp"

namespace fw {

class GripperLink {
 public:
  /** Configure the UART and its TX pin. Call once; throws on a bad pin. */
  void init(const rt::GripperConfig& config);

  /** Ask for an opening in metres. Repeated values cost nothing. */
  void send(double openingM);

  /** Tell the Arduino to stop driving the servo. */
  void release();

 private:
  bool ready_ = false;
  bool driving_ = false;
  double lastSent_ = -1;  // m; negative = nothing sent yet
};

}  // namespace fw
