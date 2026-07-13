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
