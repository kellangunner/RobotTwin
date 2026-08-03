#include "gripper.hpp"

// <math.h>, not <cmath>: this file also compiles for the Arduino Uno that
// drives the servo, and avr-libc does not reliably put fabs/copysign in
// namespace std. The C spellings exist everywhere.
#include <math.h>

namespace rt {

void GripperAxis::init(const GripperConfig& config) {
  config_ = config;
  // Open loop, exactly like the steppers before SETHOME: the datum is whatever
  // the config says the jaws were left at, and it is only as true as that.
  opening_ = config.bootOpening;
  target_ = config.bootOpening;
  idleTime_ = 0;
  pending_ = false;
}

bool GripperAxis::setOpening(double opening) {
  if (!(opening >= 0 && opening <= config_.maxOpening)) return false;
  target_ = opening;
  idleTime_ = 0;  // re-energize even if the jaws are already there
  pending_ = true;
  return true;
}

void GripperAxis::freeze() {
  target_ = opening_;
  idleTime_ = 0;    // holding still is not the same as letting go
  pending_ = false; // a command that never landed does not get to report one
}

bool GripperAxis::advance(double dt) {
  if (dt > 0 && opening_ != target_) {
    const double step = config_.speed * dt;
    const double error = target_ - opening_;
    opening_ = fabs(error) <= step ? target_ : opening_ + copysign(step, error);
    idleTime_ = 0;
  } else {
    idleTime_ += dt;
  }

  if (pending_ && opening_ == target_) {
    pending_ = false;
    return true;
  }
  return false;
}

double GripperAxis::timeToTarget() const {
  return fabs(target_ - opening_) / config_.speed;
}

double GripperAxis::pulseWidth() const {
  const double fraction = opening_ / config_.maxOpening;
  return config_.closedPulse + fraction * (config_.openPulse - config_.closedPulse);
}

bool GripperAxis::driving() const {
  return config_.releaseAfter <= 0 || idleTime_ < config_.releaseAfter;
}

} // namespace rt
