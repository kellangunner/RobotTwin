// Point-to-point joint-space trajectories: synchronized quintic time scaling
// (zero boundary velocity/acceleration, C² smooth — jerk-bounded by construction).
//
// s(τ) = 10τ³ − 15τ⁴ + 6τ⁵,  max|ṡ| = 1.875/T,  max|s̈| ≈ 5.7735/T²

import type { JointAngles } from './kinematics';

const QUINTIC_VMAX = 1.875;
const QUINTIC_AMAX = 10 / Math.sqrt(3); // ≈ 5.7735

export interface TrajectoryPlan {
  from: JointAngles;
  to: JointAngles;
  duration: number; // s
  /** true when some joint had no usable acceleration budget (torque deficit) */
  infeasible: boolean;
}

/**
 * Choose the shortest duration that respects every joint's velocity and
 * acceleration limits. Limits come from the drivetrain (gearbox-dependent),
 * so the same move gets slower/faster as the user changes gearboxes.
 */
export function planTrajectory(
  from: JointAngles,
  to: JointAngles,
  vmax: [number, number, number], // rad/s per joint
  amax: [number, number, number], // rad/s² per joint
  minDuration = 0.25,
): TrajectoryPlan {
  let T = minDuration;
  let infeasible = false;
  for (let i = 0; i < 3; i++) {
    const dq = Math.abs(to[i] - from[i]);
    if (dq < 1e-9) continue;
    if (vmax[i] <= 0 || amax[i] <= 0) {
      infeasible = true;
      continue;
    }
    T = Math.max(T, (QUINTIC_VMAX * dq) / vmax[i], Math.sqrt((QUINTIC_AMAX * dq) / amax[i]));
  }
  return { from, to, duration: T, infeasible };
}

export interface TrajectorySample {
  q: JointAngles;
  qd: JointAngles;  // rad/s
  qdd: JointAngles; // rad/s²
}

export function sampleTrajectory(plan: TrajectoryPlan, t: number): TrajectorySample {
  const T = plan.duration;
  const tau = Math.min(1, Math.max(0, t / T));
  const s = tau * tau * tau * (10 + tau * (-15 + 6 * tau));
  const sd = (tau * tau * (30 + tau * (-60 + 30 * tau))) / T;
  const sdd = (tau * (60 + tau * (-180 + 120 * tau))) / (T * T);

  const q = [0, 0, 0] as JointAngles;
  const qd = [0, 0, 0] as JointAngles;
  const qdd = [0, 0, 0] as JointAngles;
  for (let i = 0; i < 3; i++) {
    const dq = plan.to[i] - plan.from[i];
    q[i] = plan.from[i] + dq * s;
    qd[i] = dq * sd;
    qdd[i] = dq * sdd;
  }
  return { q, qd, qdd };
}
