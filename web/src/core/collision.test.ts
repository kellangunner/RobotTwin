import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import {
  checkPath,
  checkPose,
  pointCylinderDistance,
  pointSegmentDistance,
  segmentSegmentDistance,
} from './collision';
import { parseRobotConfig } from './config';
import type { JointAngles, Vec3 } from './kinematics';
import { planTrajectory } from './trajectory';
import { deg2rad } from './units';

const yamlPath = fileURLToPath(new URL('../../../config/robot.yaml', import.meta.url));
const config = parseRobotConfig(readFileSync(yamlPath, 'utf8'));
const geom = config.links;
const model = config.collision;

const pose = (a: number, b: number, c: number): JointAngles => [
  deg2rad(a),
  deg2rad(b),
  deg2rad(c),
];

describe('distance primitives', () => {
  it('point-segment distance', () => {
    const a: Vec3 = [0, 0, 0];
    const b: Vec3 = [1, 0, 0];
    expect(pointSegmentDistance([0.5, 1, 0], a, b)).toBeCloseTo(1);
    expect(pointSegmentDistance([2, 0, 0], a, b)).toBeCloseTo(1); // past the end
    expect(pointSegmentDistance([-3, 4, 0], a, b)).toBeCloseTo(5);
  });

  it('segment-segment distance', () => {
    // skew perpendicular segments 1 apart
    expect(
      segmentSegmentDistance([0, 0, 0], [1, 0, 0], [0.5, -1, 1], [0.5, 1, 1]),
    ).toBeCloseTo(1);
    // parallel
    expect(segmentSegmentDistance([0, 0, 0], [1, 0, 0], [0, 0, 2], [1, 0, 2])).toBeCloseTo(2);
    // disjoint colinear
    expect(segmentSegmentDistance([0, 0, 0], [1, 0, 0], [3, 0, 0], [4, 0, 0])).toBeCloseTo(2);
    // crossing
    expect(segmentSegmentDistance([-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0])).toBeCloseTo(0);
  });

  it('point-cylinder distance has a flat top, not a dome', () => {
    // beside the wall
    expect(pointCylinderDistance([2, 0, 0.5], 1, 1)).toBeCloseTo(1);
    // directly above the top face: vertical distance only
    expect(pointCylinderDistance([0, 0, 3], 1, 1)).toBeCloseTo(2);
    expect(pointCylinderDistance([0.9, 0, 1.4], 1, 1)).toBeCloseTo(0.4);
    // top rim corner
    expect(pointCylinderDistance([1 + 3, 0, 1 + 4], 1, 1)).toBeCloseTo(5);
    // inside
    expect(pointCylinderDistance([0.5, 0, 0.5], 1, 1)).toBe(0);
  });
});

describe('pose collision checks', () => {
  it('home pose is collision-free', () => {
    expect(checkPose(pose(0, 90, -90), geom, model).colliding).toBe(false);
  });

  it('straight horizontal arm is collision-free', () => {
    expect(checkPose(pose(0, 0, 0), geom, model).colliding).toBe(false);
  });

  it('detects the forearm reaching below the table', () => {
    // θ2=0, θ3=-60° puts the TCP at z = 90 - 120·sin60 ≈ -14 mm
    const res = checkPose(pose(0, 0, -60), geom, model);
    expect(res.colliding).toBe(true);
    expect(res.issues.join()).toContain('ground');
  });

  it('detects the forearm folding into the base column', () => {
    // reaching for a point low and close to the yaw axis
    const res = checkPose(pose(0, 24.3, -138.6), geom, model);
    expect(res.colliding).toBe(true);
    expect(res.issues.join()).toContain('column');
  });

  it('detects deep elbow folds against the shoulder joint', () => {
    const res = checkPose(pose(0, 90, -150), geom, model);
    expect(res.colliding).toBe(true);
    expect(res.issues.join()).toContain('shoulder');
    // slightly less fold clears it
    expect(checkPose(pose(0, 90, -140), geom, model).issues).toEqual([]);
  });

  it('is base-yaw invariant for the symmetric structure', () => {
    for (const q1 of [-120, -45, 60, 130]) {
      expect(checkPose(pose(q1, 0, -60), geom, model).colliding).toBe(true);
      expect(checkPose(pose(q1, 90, -90), geom, model).colliding).toBe(false);
    }
  });
});

describe('path collision checks', () => {
  const vmax: [number, number, number] = [2, 2, 2];
  const amax: [number, number, number] = [10, 10, 10];

  it('flags a path whose endpoint collides', () => {
    const from = pose(0, 90, -90);
    const to = pose(0, 0, -60);
    expect(checkPose(from, geom, model).colliding).toBe(false); // start is safe
    const plan = planTrajectory(from, to, vmax, amax);
    expect(checkPath(plan, geom, model).colliding).toBe(true);
  });

  it('passes a clearly safe move', () => {
    const plan = planTrajectory(pose(0, 90, -90), pose(45, 60, -60), vmax, amax);
    expect(checkPath(plan, geom, model).colliding).toBe(false);
  });
});
