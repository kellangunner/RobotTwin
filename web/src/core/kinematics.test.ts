import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { parseRobotConfig } from './config';
import {
  forwardKinematics,
  inverseKinematics,
  jacobian,
  singularityMeasure,
  type JointAngles,
  type Vec3,
} from './kinematics';
import { deg2rad } from './units';

const yamlPath = fileURLToPath(new URL('../../../config/robot.yaml', import.meta.url));
const config = parseRobotConfig(readFileSync(yamlPath, 'utf8'));
const { links: geom, limits } = config;

const L1 = geom.upperArm;
const L2 = geom.forearm;
const H = geom.baseHeight;

describe('config', () => {
  it('loads geometry in SI units', () => {
    expect(H).toBeCloseTo(0.09);
    expect(L1).toBeCloseTo(0.12);
    expect(L2).toBeCloseTo(0.12);
    expect(limits.base.min).toBeCloseTo(deg2rad(-135));
    expect(config.gearboxes.shoulder.ratio).toBeGreaterThan(1);
  });
});

describe('forward kinematics', () => {
  it('reference pose: arm straight out along +X', () => {
    const { tcp } = forwardKinematics([0, 0, 0], geom);
    expect(tcp[0]).toBeCloseTo(L1 + L2);
    expect(tcp[1]).toBeCloseTo(0);
    expect(tcp[2]).toBeCloseTo(H);
  });

  it('arm straight up', () => {
    const { tcp } = forwardKinematics([0, Math.PI / 2, 0], geom);
    expect(tcp[0]).toBeCloseTo(0);
    expect(tcp[2]).toBeCloseTo(H + L1 + L2);
  });

  it('base yaw rotates the arm plane', () => {
    const { tcp } = forwardKinematics([Math.PI / 2, 0, 0], geom);
    expect(tcp[0]).toBeCloseTo(0);
    expect(tcp[1]).toBeCloseTo(L1 + L2);
  });

  it('elbow folded 90° down', () => {
    const { tcp, elbow } = forwardKinematics([0, 0, -Math.PI / 2], geom);
    expect(elbow[0]).toBeCloseTo(L1);
    expect(tcp[0]).toBeCloseTo(L1);
    expect(tcp[2]).toBeCloseTo(H - L2);
  });
});

describe('inverse kinematics', () => {
  it('rejects unreachable targets', () => {
    expect(inverseKinematics([0.5, 0, H], geom, limits).reachable).toBe(false);
    expect(inverseKinematics([0.3, 0.3, 0.3], geom, limits).reachable).toBe(false);
  });

  it('returns elbow-up/down for both base orientations of an interior target', () => {
    const res = inverseKinematics([0.15, 0.05, 0.15], geom, limits);
    expect(res.reachable).toBe(true);
    expect(res.solutions).toHaveLength(4);
    const front = res.solutions.filter((s) => !s.baseFlipped).map((s) => s.branch).sort();
    expect(front).toEqual(['elbow-down', 'elbow-up']);
    // every solution must land on the target
    for (const s of res.solutions) {
      const p = forwardKinematics(s.q, geom).tcp;
      expect(p[0]).toBeCloseTo(0.15, 9);
      expect(p[1]).toBeCloseTo(0.05, 9);
      expect(p[2]).toBeCloseTo(0.15, 9);
    }
  });

  it('flags near-singular straight-arm targets', () => {
    const res = inverseKinematics([L1 + L2 - 1e-5, 0, H], geom, limits);
    expect(res.reachable).toBe(true);
    expect(res.solutions.every((s) => s.nearSingularity)).toBe(true);
  });

  it('flags base-axis targets as base singular', () => {
    const res = inverseKinematics([0, 0, H + L1 + L2 - 0.01], geom, limits);
    expect(res.baseSingular).toBe(true);
  });

  it('marks limit violations', () => {
    // target at azimuth 180°, past the ±135° yaw limit for the front-facing
    // solutions; only the base-flipped (over-the-top) branches may satisfy it
    const res = inverseKinematics([-0.15, -1e-9, 0.15], geom, limits);
    expect(res.reachable).toBe(true);
    for (const s of res.solutions.filter((sol) => !sol.baseFlipped)) {
      expect(s.violated).toContain('base');
      expect(s.withinLimits).toBe(false);
    }
    expect(res.solutions.some((s) => s.baseFlipped && !s.violated.includes('base'))).toBe(true);
  });
});

describe('FK → IK → FK round trip', () => {
  it('reconstructs the TCP over a grid of poses within limits', () => {
    for (let a = -120; a <= 120; a += 60) {
      for (let b = 10; b <= 170; b += 40) {
        for (let c = -140; c <= 140; c += 35) {
          if (Math.abs(c) < 10) continue; // skip singular straight arm
          const q: JointAngles = [deg2rad(a), deg2rad(b), deg2rad(c)];
          const { tcp } = forwardKinematics(q, geom);
          const res = inverseKinematics(tcp, geom, limits);
          expect(res.reachable).toBe(true);
          // one of the branches must reproduce the original joints & pose
          const best = res.solutions.reduce((p, s) => {
            const err = (sol: typeof s) =>
              Math.abs(sol.q[0] - q[0]) + Math.abs(sol.q[1] - q[1]) + Math.abs(sol.q[2] - q[2]);
            return err(s) < err(p) ? s : p;
          });
          expect(best.q[0]).toBeCloseTo(q[0], 6);
          expect(best.q[1]).toBeCloseTo(q[1], 6);
          expect(best.q[2]).toBeCloseTo(q[2], 6);
          const back = forwardKinematics(best.q, geom).tcp;
          expect(back[0]).toBeCloseTo(tcp[0], 9);
          expect(back[1]).toBeCloseTo(tcp[1], 9);
          expect(back[2]).toBeCloseTo(tcp[2], 9);
        }
      }
    }
  });
});

describe('jacobian', () => {
  it('matches central finite differences', () => {
    const poses: JointAngles[] = [
      [0.3, 0.8, -1.1],
      [-1.2, 1.4, 0.7],
      [0.0, 0.5, -2.0],
    ];
    const eps = 1e-6;
    for (const q of poses) {
      const J = jacobian(q, geom);
      for (let i = 0; i < 3; i++) {
        const qp = [...q] as JointAngles;
        const qm = [...q] as JointAngles;
        qp[i] += eps;
        qm[i] -= eps;
        const fp = forwardKinematics(qp, geom).tcp;
        const fm = forwardKinematics(qm, geom).tcp;
        const numeric: Vec3 = [
          (fp[0] - fm[0]) / (2 * eps),
          (fp[1] - fm[1]) / (2 * eps),
          (fp[2] - fm[2]) / (2 * eps),
        ];
        for (let k = 0; k < 3; k++) {
          expect(J[i][k]).toBeCloseTo(numeric[k], 6);
        }
      }
    }
  });

  it('singularity measure vanishes with a straight arm', () => {
    expect(singularityMeasure([0, 0.5, 0], geom)).toBeCloseTo(0, 9);
    expect(singularityMeasure([0, 0.5, -1.2], geom)).toBeGreaterThan(0.1);
  });
});
