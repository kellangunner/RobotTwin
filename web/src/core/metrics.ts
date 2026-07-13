// Aggregates kinematics + drivetrain + dynamics into the live metrics the UI
// displays. Pure functions of (config, gearbox overrides, pose, payload) —
// this is the "digital twin evaluation" layer.

import type { GearboxParams, JointName, MotorParams, RobotConfig } from './config';
import { JOINT_NAMES } from './config';
import {
  availableJointTorque,
  backlashTcpError,
  jointResolution,
  maxJointSpeed,
  maxTcpSpeed,
  reflectedInertia,
} from './drivetrain';
import { gravityTorques, worstCaseLinkInertia } from './dynamics';
import type { JointAngles } from './kinematics';
import { jacobian, singularityMeasure } from './kinematics';
import type { TrajectoryPlan } from './trajectory';
import { sampleTrajectory } from './trajectory';

export interface JointMetrics {
  name: JointName;
  availableTorque: number;   // N·m at standstill
  requiredTorque: number;    // N·m gravity load at current pose
  utilization: number;       // required / available (∞-safe)
  maxSpeed: number;          // rad/s
  maxAccel: number;          // rad/s², torque budget / total inertia
  reflectedInertia: number;  // kg·m²
  linkInertia: number;       // kg·m² (worst-case pose)
  resolution: number;        // rad per microstep
  holdFails: boolean;        // cannot even hold this pose statically
}

export interface TwinMetrics {
  joints: JointMetrics[];
  backlashErrorTcp: number;  // m, worst case at current pose
  maxTcpSpeed: number;       // m/s bound at current pose
  singularity: number;       // 0 (singular) .. 1
  vmax: [number, number, number];
  amax: [number, number, number];
}

/** Fraction of the static torque budget reserved for acceleration. */
const ACCEL_TORQUE_FRACTION = 0.7;

export interface TrajectoryAudit {
  peakUtilization: number; // worst required/available torque ratio over the move
  peakJoint: JointName;
  skippedSteps: boolean;   // >1 → open-loop steppers would lose position
}

/**
 * Predictive torque audit of a planned move: at each sample, gravity plus
 * inertial torque is compared against what the drivetrain can deliver at that
 * joint speed. Deterministic — evaluated at plan time, not during playback.
 */
export function auditTrajectory(
  config: RobotConfig,
  gearboxes: Record<JointName, GearboxParams>,
  plan: TrajectoryPlan,
  payload: number,
  samples = 120,
): TrajectoryAudit {
  const linkInertias = worstCaseLinkInertia(config.links, config.masses, payload);
  let peakUtilization = 0;
  let peakJoint: JointName = 'base';
  for (let k = 0; k <= samples; k++) {
    const s = sampleTrajectory(plan, (plan.duration * k) / samples);
    const tauGrav = gravityTorques(s.q, config.links, config.masses, payload);
    JOINT_NAMES.forEach((name, i) => {
      const gb = gearboxes[name];
      const inertia = linkInertias[i] + reflectedInertia(config.motor, gb);
      const required = Math.abs(tauGrav[i]) + inertia * Math.abs(s.qdd[i]);
      const available = availableJointTorque(config.motor, gb, s.qd[i]);
      const util = available > 1e-9 ? required / available : Infinity;
      if (util > peakUtilization) {
        peakUtilization = util;
        peakJoint = name;
      }
    });
  }
  return { peakUtilization, peakJoint, skippedSteps: peakUtilization > 1 };
}

export function computeMetrics(
  config: RobotConfig,
  gearboxes: Record<JointName, GearboxParams>,
  q: JointAngles,
  payload: number,
): TwinMetrics {
  const motor: MotorParams = config.motor;
  const J = jacobian(q, config.links);
  const tauGrav = gravityTorques(q, config.links, config.masses, payload);
  const linkInertias = worstCaseLinkInertia(config.links, config.masses, payload);

  const joints: JointMetrics[] = JOINT_NAMES.map((name, i) => {
    const gb = gearboxes[name];
    const available = availableJointTorque(motor, gb, 0);
    const required = Math.abs(tauGrav[i]);
    const refl = reflectedInertia(motor, gb);
    const totalInertia = refl + linkInertias[i];
    const budget = ACCEL_TORQUE_FRACTION * (available - required);
    return {
      name,
      availableTorque: available,
      requiredTorque: required,
      utilization: available > 1e-9 ? required / available : Infinity,
      maxSpeed: maxJointSpeed(motor, gb),
      maxAccel: Math.max(0, budget / totalInertia),
      reflectedInertia: refl,
      linkInertia: linkInertias[i],
      resolution: jointResolution(motor, gb),
      holdFails: required > available,
    };
  });

  const backlash = JOINT_NAMES.map((n) => gearboxes[n].backlash) as [number, number, number];
  const vmax = joints.map((j) => j.maxSpeed) as [number, number, number];
  const amax = joints.map((j) => j.maxAccel) as [number, number, number];

  return {
    joints,
    backlashErrorTcp: backlashTcpError(J, backlash),
    maxTcpSpeed: maxTcpSpeed(J, vmax),
    singularity: singularityMeasure(q, config.links),
    vmax,
    amax,
  };
}
