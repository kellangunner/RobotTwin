import { describe, expect, it } from 'vitest';
import type { GearboxParams, MotorParams } from './config';
import {
  availableJointTorque,
  backlashTcpError,
  jointResolution,
  maxJointSpeed,
  motorTorqueAtSpeed,
  reflectedInertia,
} from './drivetrain';
import { gravityTorques } from './dynamics';
import { planTrajectory, sampleTrajectory } from './trajectory';
import { jacobian, type JointAngles } from './kinematics';
import { deg2rad, rpm2radps } from './units';

const motor: MotorParams = {
  holdingTorque: 0.45,
  maxSpeed: rpm2radps(600),
  rotorInertia: 57e-7,
  stepAngle: deg2rad(1.8),
  microstepping: 16,
};

const gb = (over: Partial<GearboxParams> = {}): GearboxParams => ({
  type: 'planetary',
  ratio: 5,
  efficiency: 0.85,
  backlash: deg2rad(0.8),
  maxTorque: 3,
  inertia: 12e-7,
  ...over,
});

describe('motor model', () => {
  it('full torque at standstill, zero at max speed', () => {
    expect(motorTorqueAtSpeed(motor, 0)).toBeCloseTo(0.45);
    expect(motorTorqueAtSpeed(motor, motor.maxSpeed)).toBeCloseTo(0);
    expect(motorTorqueAtSpeed(motor, motor.maxSpeed * 2)).toBe(0);
  });
});

describe('gearbox model', () => {
  it('multiplies torque by ratio × efficiency', () => {
    expect(availableJointTorque(motor, gb(), 0)).toBeCloseTo(0.45 * 5 * 0.85);
  });

  it('is capped by printed-gear strength', () => {
    const strong = gb({ ratio: 50, maxTorque: 3 });
    expect(availableJointTorque(motor, strong, 0)).toBe(3);
  });

  it('derates with joint speed via reflected motor speed', () => {
    const half = availableJointTorque(motor, gb(), maxJointSpeed(motor, gb()) / 2);
    expect(half).toBeCloseTo((0.45 * 5 * 0.85) / 2);
  });

  it('divides speed and squares inertia by the ratio', () => {
    expect(maxJointSpeed(motor, gb({ ratio: 10 }))).toBeCloseTo(motor.maxSpeed / 10);
    expect(reflectedInertia(motor, gb({ ratio: 10 }))).toBeCloseTo((57e-7 + 12e-7) * 100);
  });

  it('improves joint resolution by the ratio', () => {
    expect(jointResolution(motor, gb({ ratio: 5 }))).toBeCloseTo(deg2rad(1.8) / 16 / 5);
  });
});

describe('gravity torques', () => {
  const geom = { baseHeight: 0.09, upperArm: 0.12, forearm: 0.12 };
  const masses = {
    upperArm: 0.16,
    forearm: 0.12,
    elbowMotor: 0.35,
    gripper: 0.09,
    payloadDefault: 0.1,
    payloadMax: 0.5,
  };

  it('worst case at full horizontal extension', () => {
    const [tb, ts, te] = gravityTorques([0, 0, 0], geom, masses, 0.1);
    expect(tb).toBe(0);
    expect(ts).toBeCloseTo(1.166, 2); // hand-computed in docs/linkage-geometry.md
    expect(te).toBeCloseTo(0.294, 2);
  });

  it('vanishes with the arm vertical', () => {
    const [, ts, te] = gravityTorques([0, Math.PI / 2, 0], geom, masses, 0.1);
    expect(ts).toBeCloseTo(0, 9);
    expect(te).toBeCloseTo(0, 9);
  });

  it('backlash error grows with reach', () => {
    const b: [number, number, number] = [deg2rad(0.8), deg2rad(0.8), deg2rad(0.8)];
    const qExt: JointAngles = [0, 0, -0.1];
    const qFold: JointAngles = [0, 0.8, -2.0];
    const eExt = backlashTcpError(jacobian(qExt, geom), b);
    const eFold = backlashTcpError(jacobian(qFold, geom), b);
    expect(eExt).toBeGreaterThan(eFold);
    expect(eExt).toBeGreaterThan(0.001); // > 1 mm with cheap gearboxes everywhere
  });
});

describe('trajectory planner', () => {
  it('respects velocity and acceleration limits', () => {
    const from: JointAngles = [0, 0.2, -0.4];
    const to: JointAngles = [1.5, 1.2, -1.8];
    const vmax: [number, number, number] = [1.0, 0.8, 1.2];
    const amax: [number, number, number] = [3, 2, 4];
    const plan = planTrajectory(from, to, vmax, amax);
    expect(plan.infeasible).toBe(false);

    let peakV = [0, 0, 0];
    let peakA = [0, 0, 0];
    for (let t = 0; t <= plan.duration; t += plan.duration / 400) {
      const s = sampleTrajectory(plan, t);
      for (let i = 0; i < 3; i++) {
        peakV[i] = Math.max(peakV[i], Math.abs(s.qd[i]));
        peakA[i] = Math.max(peakA[i], Math.abs(s.qdd[i]));
      }
    }
    for (let i = 0; i < 3; i++) {
      expect(peakV[i]).toBeLessThanOrEqual(vmax[i] * 1.001);
      expect(peakA[i]).toBeLessThanOrEqual(amax[i] * 1.001);
    }
    // endpoints exact, at rest
    const end = sampleTrajectory(plan, plan.duration);
    for (let i = 0; i < 3; i++) {
      expect(end.q[i]).toBeCloseTo(to[i], 9);
      expect(end.qd[i]).toBeCloseTo(0, 9);
    }
  });

  it('slower gearbox limits (lower vmax) lengthen the move', () => {
    const from: JointAngles = [0, 0, 0];
    const to: JointAngles = [1, 1, 1];
    const fast = planTrajectory(from, to, [2, 2, 2], [10, 10, 10]);
    const slow = planTrajectory(from, to, [0.5, 0.5, 0.5], [10, 10, 10]);
    expect(slow.duration).toBeGreaterThan(fast.duration * 3);
  });

  it('flags infeasible when a joint has no torque budget', () => {
    const plan = planTrajectory([0, 0, 0], [1, 1, 1], [1, 1, 1], [1, 0, 1]);
    expect(plan.infeasible).toBe(true);
  });
});

describe('trajectory audit (predictive skipped-step detection)', async () => {
  const { readFileSync } = await import('node:fs');
  const { fileURLToPath } = await import('node:url');
  const { parseRobotConfig } = await import('./config');
  const { auditTrajectory } = await import('./metrics');

  const config = parseRobotConfig(
    readFileSync(fileURLToPath(new URL('../../../config/robot.yaml', import.meta.url)), 'utf8'),
  );
  // slow "move" that holds near full horizontal extension
  const plan = planTrajectory([0, 0.05, -0.1], [0, 0.1, -0.15], [0.5, 0.5, 0.5], [2, 2, 2]);

  it('predicts skipped steps with a direct-drive shoulder', () => {
    const gearboxes = {
      ...config.gearboxes,
      shoulder: { ...config.gearboxes.shoulder, ratio: 1, efficiency: 0.98 },
    };
    const audit = auditTrajectory(config, gearboxes, plan, 0.1);
    expect(audit.skippedSteps).toBe(true);
    expect(audit.peakJoint).toBe('shoulder');
  });

  it('passes with the default cycloidal shoulder', () => {
    const audit = auditTrajectory(config, config.gearboxes, plan, 0.1);
    expect(audit.skippedSteps).toBe(false);
    expect(audit.peakUtilization).toBeGreaterThan(0.1);
  });
});
