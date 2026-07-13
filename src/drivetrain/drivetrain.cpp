#include "drivetrain.hpp"

#include <algorithm>
#include <cmath>

#include "../math/units.hpp"

namespace rt {

double motorTorqueAtSpeed(const MotorParams& motor, double motorSpeed) {
  const double s = std::abs(motorSpeed) / motor.maxSpeed;
  return std::max(0.0, motor.holdingTorque * (1.0 - s));
}

double availableJointTorque(const MotorParams& motor, const GearboxParams& gb,
                            double jointSpeed) {
  const double motorSpeed = jointSpeed * gb.ratio;
  const double throughGearbox = motorTorqueAtSpeed(motor, motorSpeed) * gb.ratio * gb.efficiency;
  return std::min(throughGearbox, gb.maxTorque);
}

double maxJointSpeed(const MotorParams& motor, const GearboxParams& gb) {
  return motor.maxSpeed / gb.ratio;
}

double torqueLimitedSpeed(const MotorParams& motor, const GearboxParams& gb,
                          double requiredTorque) {
  const double standstillJointTorque = motor.holdingTorque * gb.ratio * gb.efficiency;
  // If even standstill can't meet the requirement, no safe speed exists.
  if (standstillJointTorque <= requiredTorque) return 0.0;
  // Also respect the printed-gear strength cap.
  if (gb.maxTorque <= requiredTorque) return 0.0;

  // Speed at which the motor-side torque, after reduction, equals required.
  const double fraction = 1.0 - requiredTorque / standstillJointTorque;
  const double speedFromMotor = (motor.maxSpeed / gb.ratio) * fraction;

  // Speed at which the gearbox hits its own torque cap (printed-gear limit).
  const double gearCapFraction = 1.0 - gb.maxTorque / standstillJointTorque;
  const double speedFromGearCap = gearCapFraction > 0.0
                                      ? (motor.maxSpeed / gb.ratio) * gearCapFraction
                                      : motor.maxSpeed / gb.ratio; // cap not binding

  return std::max(0.0, std::min(speedFromMotor, speedFromGearCap));
}

double reflectedInertia(const MotorParams& motor, const GearboxParams& gb) {
  return (motor.rotorInertia + gb.inertia) * gb.ratio * gb.ratio;
}

double jointResolution(const MotorParams& motor, const GearboxParams& gb) {
  return motor.stepAngle / motor.microstepping / gb.ratio;
}

double backlashTcpError(const Jacobian& J, const std::array<double, kNumJoints>& backlash) {
  double e = 0.0;
  for (int i = 0; i < kNumJoints; ++i) e += norm(J[i]) * backlash[i];
  return e;
}

double maxTcpSpeed(const Jacobian& J, const std::array<double, kNumJoints>& jointSpeedLimits) {
  double v = 0.0;
  for (int i = 0; i < kNumJoints; ++i) v += norm(J[i]) * jointSpeedLimits[i];
  return v;
}

} // namespace rt
