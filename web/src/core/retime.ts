// Torque-governed retiming: stretch a planned move's duration until the
// predictive torque audit fits inside the budget, making it impossible to
// command a move fast enough to exceed the torque limit.
//
// Physics: slowing a quintic profile by k divides velocities by k and
// accelerations by k², so inertial torque shrinks and the stepper's
// speed-dependent available torque grows, while the gravity load stays fixed.
// Utilization therefore falls monotonically toward its static floor as the
// duration grows. If even that floor exceeds the ceiling (the arm cannot hold
// the pose at all), no timing helps and the move is flagged instead.
//
// This is pure orchestration over plan + audit; the physics lives in the
// injected audit callback (core auditTrajectory), so it works identically
// over the WASM core and the TS mirror.

import type { TrajectoryAudit } from './metrics';
import type { TrajectoryPlan } from './trajectory';

export interface RetimeResult {
  plan: TrajectoryPlan;
  audit: TrajectoryAudit;
  /** Duration multiplier applied (1 = the planner's timing was already safe). */
  stretch: number;
  /** True when no stretch ≤ maxStretch fits the budget (static overload). */
  limited: boolean;
}

/**
 * Find the smallest duration stretch that brings peak torque utilization at
 * or below `ceiling`. Exponential search up to `maxStretch`, then bisection,
 * so moves stay as quick as the torque budget allows.
 */
export function retimeForTorque(
  plan: TrajectoryPlan,
  audit: (plan: TrajectoryPlan) => TrajectoryAudit,
  ceiling = 0.95,
  maxStretch = 32,
): RetimeResult {
  const stretched = (k: number): TrajectoryPlan => ({ ...plan, duration: plan.duration * k });

  const first = audit(plan);
  if (first.peakUtilization <= ceiling) {
    return { plan, audit: first, stretch: 1, limited: false };
  }

  // Exponential search for any passing stretch; `lo` stays known-failing.
  let lo = 1;
  let hi = 1;
  let hiAudit = first;
  do {
    lo = hi;
    hi = Math.min(maxStretch, hi * 2);
    hiAudit = audit(stretched(hi));
  } while (hiAudit.peakUtilization > ceiling && hi < maxStretch);

  if (hiAudit.peakUtilization > ceiling) {
    // Static overload: gravity alone beats the budget, so no speed is slow
    // enough. Keep the planner's timing and let the caller flag the failure.
    return { plan, audit: first, stretch: 1, limited: true };
  }

  // Bisect down to (near) the smallest passing stretch.
  let pass = hi;
  let passAudit = hiAudit;
  for (let i = 0; i < 8; i++) {
    const mid = (lo + pass) / 2;
    const midAudit = audit(stretched(mid));
    if (midAudit.peakUtilization <= ceiling) {
      pass = mid;
      passAudit = midAudit;
    } else {
      lo = mid;
    }
  }
  return { plan: stretched(pass), audit: passAudit, stretch: pass, limited: false };
}
