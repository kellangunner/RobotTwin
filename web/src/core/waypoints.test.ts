import { describe, expect, it } from 'vitest';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { parseRobotConfig } from './config';
import { forwardKinematics, type JointAngles } from './kinematics';
import { parseWaypointCsv } from './waypoints';
import { deg2rad } from './units';

const yamlPath = fileURLToPath(new URL('../../../config/robot.yaml', import.meta.url));
const config = parseRobotConfig(readFileSync(yamlPath, 'utf8'));
const HOME: JointAngles = [0, deg2rad(90), deg2rad(-90)];

describe('joint-space CSV', () => {
  it('parses degrees, skipping a header row', () => {
    const res = parseWaypointCsv(
      'theta1,theta2,theta3\n0,90,-90\n45,45,-45\n',
      'joints',
      config,
      'elbow-up',
      HOME,
    );
    expect(res.targets).toHaveLength(2);
    expect(res.skipped).toBe(0);
    expect(res.targets[0][1]).toBeCloseTo(deg2rad(90));
    expect(res.targets[1][0]).toBeCloseTo(deg2rad(45));
  });

  it('skips rows outside joint limits with a reason', () => {
    const res = parseWaypointCsv('170,90,-90\n0,90,-90', 'joints', config, 'elbow-up', HOME);
    expect(res.targets).toHaveLength(1);
    expect(res.skipped).toBe(1);
    expect(res.firstIssue).toContain('base out of limits');
  });

  it('skips colliding poses with a reason', () => {
    const res = parseWaypointCsv('0,0,-60\n0,90,-90', 'joints', config, 'elbow-up', HOME);
    expect(res.targets).toHaveLength(1);
    expect(res.firstIssue).toContain('ground');
  });

  it('skips malformed rows and ignores comments/blank lines', () => {
    const res = parseWaypointCsv(
      '# demo path\n\n0,90,-90\nnot,a,row\n10,80',
      'joints',
      config,
      'elbow-up',
      HOME,
    );
    expect(res.targets).toHaveLength(1);
    expect(res.skipped).toBe(2);
  });
});

describe('cartesian CSV', () => {
  it('converts mm targets through IK and lands on them', () => {
    const res = parseWaypointCsv('x,y,z\n120,0,210\n150,50,150', 'cartesian', config, 'elbow-up', HOME);
    expect(res.targets).toHaveLength(2);
    expect(res.skipped).toBe(0);
    const tcp = forwardKinematics(res.targets[1], config.links).tcp;
    expect(tcp[0]).toBeCloseTo(0.15, 6);
    expect(tcp[1]).toBeCloseTo(0.05, 6);
    expect(tcp[2]).toBeCloseTo(0.15, 6);
  });

  it('skips unreachable targets', () => {
    const res = parseWaypointCsv('500,0,210\n120,0,210', 'cartesian', config, 'elbow-up', HOME);
    expect(res.targets).toHaveLength(1);
    expect(res.firstIssue).toContain('unreachable');
  });

  it('skips targets whose every IK branch collides', () => {
    // directly above the base housing, too low: inside the collision envelope
    const res = parseWaypointCsv('60,0,30\n120,0,210', 'cartesian', config, 'elbow-up', HOME);
    expect(res.targets).toHaveLength(1);
    expect(res.skipped).toBe(1);
  });

  it('yields nothing useful for an empty file', () => {
    const res = parseWaypointCsv('', 'cartesian', config, 'elbow-up', HOME);
    expect(res.targets).toHaveLength(0);
  });
});
