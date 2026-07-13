// Motor + gearbox model. This is where the twin's independent variables live:
// every metric the UI shows flows through these functions.
//
// Stepper model: available torque falls off linearly from holding torque at
// standstill to zero at maxSpeed (a deliberate first-order approximation of
// the real torque-speed curve; refine when firmware data exists).

import type { GearboxParams, MotorParams } from './config';
import type { Vec3 } from './kinematics';
import { columnNorm } from './kinematics';

/** Motor shaft torque available at a given motor speed (rad/s). */
export function motorTorqueAtSpeed(motor: MotorParams, motorSpeed: number): number {
  const s = Math.abs(motorSpeed) / motor.maxSpeed;
  return Math.max(0, motor.holdingTorque * (1 - s));
}

/**
 * Torque available at the joint (gearbox output) for a given joint speed.
 * Capped by the printed gearbox's own strength limit.
 */
export function availableJointTorque(
  motor: MotorParams,
  gb: GearboxParams,
  jointSpeed = 0,
): number {
  const motorSpeed = jointSpeed * gb.ratio;
  const throughGearbox = motorTorqueAtSpeed(motor, motorSpeed) * gb.ratio * gb.efficiency;
  return Math.min(throughGearbox, gb.maxTorque);
}

/** Maximum joint speed (rad/s): the motor speed ceiling divided down. */
export function maxJointSpeed(motor: MotorParams, gb: GearboxParams): number {
  return motor.maxSpeed / gb.ratio;
}

/**
 * Maximum joint speed (rad/s) at which the drivetrain can still produce at
 * least `requiredTorque` N·m at the joint output.  Returns 0 when even
 * standstill torque can't meet the requirement (hold failure).
 *
 * Derived from the linear stepper torque-speed model:
 *   τ_motor(ω) = holdingTorque × (1 − ω/ωmax)
 *   τ_joint    = τ_motor × ratio × η,  capped at gb.maxTorque
 *
 * Solving τ_joint ≥ required for ω_joint:
 *   ω_joint ≤ (ωmax / ratio) × (1 − required / (holdingTorque × ratio × η))
 *
 * The result is further clamped so it never exceeds the gear-cap-limited
 * speed (the speed at which gearbox maxTorque equals the motor's output).
 */
export function torqueLimitedSpeed(
  motor: MotorParams,
  gb: GearboxParams,
  requiredTorque: number,
): number {
  const standstillJointTorque = motor.holdingTorque * gb.ratio * gb.efficiency;
  // If even standstill can't meet the requirement, no safe speed exists.
  if (standstillJointTorque <= requiredTorque) return 0;
  // Also respect the printed-gear strength cap.
  if (gb.maxTorque <= requiredTorque) return 0;

  // Speed at which the motor-side torque, after reduction, equals required.
  const fraction = 1 - requiredTorque / standstillJointTorque;
  const speedFromMotor = (motor.maxSpeed / gb.ratio) * fraction;

  // Speed at which the gearbox hits its own torque cap (printed-gear limit).
  // τ_motor(ω) × ratio × η = maxTorque  →  ω_motor = ωmax × (1 − maxTorque/(hold×ratio×η))
  const gearCapFraction = 1 - gb.maxTorque / standstillJointTorque;
  const speedFromGearCap = gearCapFraction > 0
    ? (motor.maxSpeed / gb.ratio) * gearCapFraction
    : motor.maxSpeed / gb.ratio; // gear cap is above motor capability → not binding

  return Math.max(0, Math.min(speedFromMotor, speedFromGearCap));
}

/** Rotor + gearbox input inertia reflected to the joint side (kg·m²). */
export function reflectedInertia(motor: MotorParams, gb: GearboxParams): number {
  return (motor.rotorInertia + gb.inertia) * gb.ratio * gb.ratio;
}

/** Joint-space position resolution (rad) after microstepping and reduction. */
export function jointResolution(motor: MotorParams, gb: GearboxParams): number {
  return motor.stepAngle / motor.microstepping / gb.ratio;
}

/**
 * Worst-case TCP position error (m) caused by output backlash on each joint:
 * each joint can sit anywhere inside its backlash band, so errors add as
 * Σ ‖J_i‖ · backlash_i.
 */
export function backlashTcpError(J: [Vec3, Vec3, Vec3], backlash: [number, number, number]): number {
  return (
    columnNorm(J[0]) * backlash[0] +
    columnNorm(J[1]) * backlash[1] +
    columnNorm(J[2]) * backlash[2]
  );
}

/** Upper bound on TCP speed (m/s) with every joint at its speed limit. */
export function maxTcpSpeed(J: [Vec3, Vec3, Vec3], jointSpeedLimits: [number, number, number]): number {
  return (
    columnNorm(J[0]) * jointSpeedLimits[0] +
    columnNorm(J[1]) * jointSpeedLimits[1] +
    columnNorm(J[2]) * jointSpeedLimits[2]
  );
}
