// CSV waypoint parsing: rows of either Cartesian TCP targets (x,y,z in mm) or
// joint angles (θ1,θ2,θ3 in degrees), converted to validated joint-space
// targets. Cartesian rows go through the analytical IK, chaining branch
// selection from the previous waypoint so the arm doesn't flip mid-sequence.

import type { RobotConfig } from './config';
import { JOINT_NAMES } from './config';
import type { IkBranch, JointAngles } from './kinematics';
import { inverseKinematics } from './kinematics';
import { deg2rad, mm2m } from './units';

export type WaypointMode = 'cartesian' | 'joints';

export interface WaypointParseResult {
  targets: JointAngles[];
  /** Rows dropped: malformed, out of limits, or unreachable. */
  skipped: number;
  firstIssue: string | null;
}

export function parseWaypointCsv(
  text: string,
  mode: WaypointMode,
  config: RobotConfig,
  branch: IkBranch,
  fromQ: JointAngles,
): WaypointParseResult {
  const targets: JointAngles[] = [];
  let skipped = 0;
  let firstIssue: string | null = null;
  let sawData = false;
  let prevQ = fromQ;

  const issue = (msg: string) => {
    skipped++;
    if (!firstIssue) firstIssue = msg;
  };

  text.split(/\r?\n/).forEach((raw, idx) => {
    const line = raw.trim();
    if (!line || line.startsWith('#')) return;

    const nums = line.split(/[,;\t]+|\s+/).filter(Boolean).map(Number);
    if (nums.length !== 3 || nums.some((n) => !Number.isFinite(n))) {
      // tolerate one leading header row (e.g. "x,y,z") silently
      if (!sawData && nums.some((n) => !Number.isFinite(n))) {
        sawData = true;
        return;
      }
      issue(`line ${idx + 1}: expected 3 numbers`);
      return;
    }
    sawData = true;

    if (mode === 'joints') {
      const q = nums.map(deg2rad) as JointAngles;
      const bad = JOINT_NAMES.find(
        (name, i) => q[i] < config.limits[name].min - 1e-9 || q[i] > config.limits[name].max + 1e-9,
      );
      if (bad) {
        issue(`line ${idx + 1}: ${bad} out of limits`);
        return;
      }
      targets.push(q);
      prevQ = q;
    } else {
      const p = nums.map(mm2m) as [number, number, number];
      const res = inverseKinematics(p, config.links, config.limits);
      const usable = res.reachable ? res.solutions.filter((s) => s.withinLimits) : [];
      if (usable.length === 0) {
        issue(`line ${idx + 1}: unreachable or outside joint limits`);
        return;
      }
      const preferred = usable.filter((s) => s.branch === branch);
      const pool = preferred.length > 0 ? preferred : usable;
      const dist = (q: JointAngles) =>
        Math.abs(q[0] - prevQ[0]) + Math.abs(q[1] - prevQ[1]) + Math.abs(q[2] - prevQ[2]);
      const best = pool.reduce((a, b) => (dist(b.q) < dist(a.q) ? b : a));
      targets.push(best.q);
      prevQ = best.q;
    }
  });

  return { targets, skipped, firstIssue };
}
