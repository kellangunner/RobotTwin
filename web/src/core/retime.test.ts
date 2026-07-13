import { describe, expect, it } from 'vitest';
import type { TrajectoryAudit } from './metrics';
import { retimeForTorque } from './retime';
import type { TrajectoryPlan } from './trajectory';

const basePlan: TrajectoryPlan = {
  from: [0, 0, 0],
  to: [1, 1, 1],
  duration: 1,
  infeasible: false,
};

/**
 * Synthetic audit matching the real physics shape: a fixed gravity floor plus
 * an inertial term that scales with 1/duration² (quintic time scaling).
 */
const gravityPlusInertia =
  (grav: number, dyn: number) =>
  (plan: TrajectoryPlan): TrajectoryAudit => {
    const util = grav + dyn / (plan.duration * plan.duration);
    return { peakUtilization: util, peakJoint: 'shoulder', skippedSteps: util > 1 };
  };

describe('retimeForTorque', () => {
  it('leaves an already-safe plan untouched', () => {
    const res = retimeForTorque(basePlan, gravityPlusInertia(0.3, 0.2), 0.95);
    expect(res.stretch).toBe(1);
    expect(res.limited).toBe(false);
    expect(res.plan.duration).toBe(basePlan.duration);
    expect(res.audit.peakUtilization).toBeLessThanOrEqual(0.95);
  });

  it('stretches an overloaded move until utilization fits the ceiling', () => {
    const ceiling = 0.95;
    const audit = gravityPlusInertia(0.4, 2.0); // util(1) = 2.4 → overloaded
    const res = retimeForTorque(basePlan, audit, ceiling);
    expect(res.limited).toBe(false);
    expect(res.audit.peakUtilization).toBeLessThanOrEqual(ceiling);
    // Minimal stretch solves grav + dyn/k² = ceiling → k* = √(dyn/(ceiling−grav)).
    const kStar = Math.sqrt(2.0 / (ceiling - 0.4));
    expect(res.stretch).toBeGreaterThanOrEqual(kStar);
    expect(res.stretch).toBeLessThan(kStar * 1.1); // bisection stays near-minimal
    expect(res.plan.duration).toBeCloseTo(res.stretch * basePlan.duration, 10);
  });

  it('never returns a plan above the ceiling when one is achievable', () => {
    for (const dyn of [0.1, 1, 10, 100, 700]) {
      const res = retimeForTorque(basePlan, gravityPlusInertia(0.2, dyn), 0.95);
      expect(res.limited).toBe(false);
      expect(res.audit.peakUtilization).toBeLessThanOrEqual(0.95);
    }
  });

  it('flags moves that would need a stretch beyond the cap', () => {
    // Needs k ≈ 34.6 > default maxStretch of 32.
    const res = retimeForTorque(basePlan, gravityPlusInertia(0.2, 900), 0.95);
    expect(res.limited).toBe(true);
  });

  it('flags a static overload instead of stretching forever', () => {
    // Gravity floor above the ceiling: no duration can fix this.
    const res = retimeForTorque(basePlan, gravityPlusInertia(1.2, 0.5), 0.95);
    expect(res.limited).toBe(true);
    expect(res.stretch).toBe(1);
    expect(res.plan.duration).toBe(basePlan.duration); // planner timing kept
    expect(res.audit.skippedSteps).toBe(true);
  });

  it('recovers from infinite utilization (speed beyond the motor ceiling)', () => {
    const audit = (plan: TrajectoryPlan): TrajectoryAudit => {
      const util = plan.duration < 3 ? Infinity : 0.5;
      return { peakUtilization: util, peakJoint: 'base', skippedSteps: util > 1 };
    };
    const res = retimeForTorque(basePlan, audit, 0.95);
    expect(res.limited).toBe(false);
    expect(res.plan.duration).toBeGreaterThanOrEqual(3);
    expect(res.audit.peakUtilization).toBeLessThanOrEqual(0.95);
  });
});
