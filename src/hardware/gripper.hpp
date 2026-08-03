// Gripper axis: the pure logic behind the arm's one hobby-servo actuator.
//
// The mechanism (SG-90 → gear/rack → parallel jaws) is modelled by exactly two
// calibration points, closed and open, from config/firmware.yaml. Opening maps
// linearly onto pulse width between them, and the servo's internal loop does
// everything else — there is no feedback, no torque model, and no homing.
//
// Deliberately free of any hardware dependency: this class holds the commanded
// state, rate-limits it, and reports the pulse width that should be on the wire.
// All of the arithmetic below is natively unit tested like the rest of the core.
//
// It runs in two places at once. The Arduino in arduino/gripper_node/ uses it to
// actually drive the servo; the ESP32 keeps its own copy purely as a model, so
// it can time EV GRIP_DONE and report grip= over a link that never answers. They
// only stay in step because both are fed the same calibration — which is why
// this header must keep compiling for an 8-bit target (see gripper_config.hpp).
//
// Two behaviours worth knowing:
//   - the jaws are slewed at gripper.speed rather than snapped to the target,
//     so a full-stroke command does not fling the mechanism (or brown out the
//     5 V rail) the instant it arrives;
//   - after release_after_s of stillness the axis stops asking to be driven, so
//     an idle servo stops buzzing. The mechanism is not self-locking, so that
//     knob defaults to off — see the config comments.
#pragma once

#include "gripper_config.hpp"

namespace rt {

class GripperAxis {
 public:
  /** Adopt a calibration; the jaws are assumed to be at cfg.bootOpening. */
  void init(const GripperConfig& config);

  /** Command an opening in metres. False (and no change) if out of range. */
  bool setOpening(double opening);

  /** Hold the jaws where they are (STOP), re-energizing if they were released. */
  void freeze();

  /**
   * Advance the slew by `dt` seconds.
   * @return true exactly once per command, on the tick the jaws land on it.
   */
  bool advance(double dt);

  double opening() const { return opening_; }
  double target() const { return target_; }
  double maxOpening() const { return config_.maxOpening; }
  bool moving() const { return opening_ != target_; }

  /** Seconds the jaws still need to reach the target at the configured speed. */
  double timeToTarget() const;

  /** Pulse width for the current opening, in seconds. */
  double pulseWidth() const;

  /** Pulse width as a fraction of the PWM period — what a timer wants. */
  double dutyFraction() const { return pulseWidth() * config_.pwmHz; }

  /** Whether the servo should be energized right now (see release_after_s). */
  bool driving() const;

 private:
  GripperConfig config_{};
  double opening_ = 0;   // m, where the jaws are believed to be
  double target_ = 0;    // m, where they were last told to go
  double idleTime_ = 0;  // s since the jaws last moved
  bool pending_ = false; // a command is in flight (drives the landing report)
};

} // namespace rt
