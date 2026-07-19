// Fixed-rate step-pulse generator for the three TMC2209 step/dir channels.
//
// One GPTimer ISR at step_tick_hz services every joint with an integer DDA:
// each tick a channel accumulates its Q16.16 rate, and on overflow emits one
// step toward its target position. The ISR only compares integers and writes
// GPIO registers; all trajectory math stays in the 1 kHz control loop, which
// feeds absolute microstep targets plus a tracking rate.
//
// Consequences of the design:
//   - max step rate = step_tick_hz (one step per tick per joint)
//   - position can never overshoot its target (DDA stops when pos == target)
//   - step/dir pins must be in the low GPIO bank (< 32): the ISR uses the
//     out_w1ts/out_w1tc set/clear registers directly
#pragma once

#include <cstdint>

#include "hardware/hardware_config.hpp"

namespace fw {

class StepEngine {
 public:
  /** Configure GPIO + start the step timer. Call once; throws on bad pins. */
  void init(const rt::HardwareConfig& hw);

  // ---- control-loop API (any task; fields are 32-bit atomic) ----
  void setTarget(int joint, int32_t steps);
  void setRate(int joint, double stepsPerSecond); // magnitude, clamped to tick rate
  int32_t position(int joint) const;

  /** Re-datum a joint (homing). Only call while the joint is halted. */
  void setPosition(int joint, int32_t steps);

  /** Freeze every joint where it is (target := position). */
  void haltAll();

  /** Drive the shared TMC2209 EN line. */
  void setEnabled(bool on);
  bool enabled() const { return enabled_; }

  /** Internal: one DDA pass over all channels. Called only from the timer ISR. */
  void isrTick();

 private:
  struct Channel {
    volatile int32_t pos = 0;
    volatile int32_t target = 0;
    volatile uint32_t rateQ16 = 0; // steps per tick, Q16.16, <= 1.0
    uint32_t acc = 0;              // ISR-only accumulator
    uint32_t stepMask = 0;         // 1 << step_pin
    uint32_t dirMask = 0;          // 1 << dir_pin
    bool invertDir = false;
  };

  Channel channels_[rt::kNumJoints];
  double stepTickHz_ = 0;
  int enablePin_ = -1;
  bool enableActiveLow_ = true;
  bool enabled_ = false;
};

} // namespace fw
