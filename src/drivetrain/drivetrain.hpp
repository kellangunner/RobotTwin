// Motor + gearbox model — the twin's independent variables feed through here.
// Stepper model: available torque falls linearly from holding torque at
// standstill to zero at maxSpeed (first-order approximation; refine with
// measured data once firmware exists). Mirrors web/src/core/drivetrain.ts.
#pragma once

#include "../config/config.hpp"
#include "../kinematics/kinematics.hpp"

namespace rt {

/** Motor shaft torque available at a given motor speed (rad/s). */
double motorTorqueAtSpeed(const MotorParams& motor, double motorSpeed);

/**
 * Torque available at the joint (gearbox output) for a given joint speed,
 * capped by the printed gearbox's own strength limit.
 */
double availableJointTorque(const MotorParams& motor, const GearboxParams& gb,
                            double jointSpeed = 0.0);

/** Maximum joint speed (rad/s): the motor speed ceiling divided down. */
double maxJointSpeed(const MotorParams& motor, const GearboxParams& gb);

/**
 * Maximum joint speed (rad/s) at which the drivetrain can still produce at
 * least `requiredTorque` at the joint output. 0 when even standstill fails.
 */
double torqueLimitedSpeed(const MotorParams& motor, const GearboxParams& gb,
                          double requiredTorque);

/** Rotor + gearbox input inertia reflected to the joint side (kg·m²). */
double reflectedInertia(const MotorParams& motor, const GearboxParams& gb);

/** Joint-space position resolution (rad) after microstepping and reduction. */
double jointResolution(const MotorParams& motor, const GearboxParams& gb);

/**
 * Worst-case TCP position error (m) from output backlash on each joint:
 * errors add as Σ ‖J_i‖ · backlash_i.
 */
double backlashTcpError(const Jacobian& J, const std::array<double, kNumJoints>& backlash);

/** Upper bound on TCP speed (m/s) with every joint at its speed limit. */
double maxTcpSpeed(const Jacobian& J, const std::array<double, kNumJoints>& jointSpeedLimits);

} // namespace rt
