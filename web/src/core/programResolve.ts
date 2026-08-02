// Turns program rows into validated joint-space targets.
//
// Split out from program.ts because this half goes through the C++ core (via
// core/api), which the pure model and its unit tests must not drag in.
//
// Every row is resolved ahead of time, not while the arm is moving: the panel
// can then show per-row reachability the moment a value is typed, and a run
// never discovers a bad waypoint halfway through.

import { checkPose, inverseKinematics } from './api';
import type { RobotConfig } from './config';
import { JOINT_NAMES } from './config';
import type { IkBranch, JointAngles } from './kinematics';
import type { MoveKind, ProgramMove } from './program';
import { deg2rad, mm2m, rad2deg } from './units';

export type MoveResolution =
  | { ok: true; q: JointAngles; nearSingularity: boolean }
  | { ok: false; reason: string };

export interface ResolvedStep {
  move: ProgramMove;
  q: JointAngles;
  dwell: number;
}

export interface ProgramResolution {
  /** Runnable steps, in order — enabled rows that resolved cleanly. */
  steps: ResolvedStep[];
  /** Per-row outcome for the editor, including disabled and failed rows. */
  byId: Record<string, MoveResolution>;
  firstError: string | null;
}

/**
 * A positional row as a world-frame TCP target in metres.
 *
 * Polar is cylindrical about the base yaw axis, matching the arm's own FK
 * (`tcp = (r·cosθ1, r·sinθ1, z)`): r is planar reach, θ the base angle, z the
 * height. A negative r is allowed and means "reach the other way" — the same
 * point as (|r|, θ+180°) — because the base can yaw past ±90° and the IK
 * already models that as the base-flipped branch.
 */
function toCartesian(move: ProgramMove): [number, number, number] {
  const [a, b, c] = move.values;
  if (move.kind === 'polar') {
    const th = deg2rad(b);
    return [mm2m(a * Math.cos(th)), mm2m(a * Math.sin(th)), mm2m(c)];
  }
  return [mm2m(a), mm2m(b), mm2m(c)];
}

/** The inverse of toCartesian, for capturing the current pose as a row. */
export function fromCartesianMm(
  kind: MoveKind,
  xyzMm: [number, number, number],
): [number, number, number] {
  const [x, y, z] = xyzMm;
  if (kind !== 'polar') return [x, y, z];
  return [Math.hypot(x, y), rad2deg(Math.atan2(y, x)), z];
}

/**
 * Resolve one row against the robot. `fromQ` is the pose the arm will be in
 * when this move starts; it only breaks ties (elbow branch and base flip), so
 * a sequence never flips configuration between neighbouring waypoints.
 */
export function resolveMove(
  move: ProgramMove,
  config: RobotConfig,
  branch: IkBranch,
  fromQ: JointAngles,
): MoveResolution {
  if (move.kind === 'joints') {
    const q = move.values.map(deg2rad) as JointAngles;
    const bad = JOINT_NAMES.findIndex(
      (name, i) => q[i] < config.limits[name].min - 1e-9 || q[i] > config.limits[name].max + 1e-9,
    );
    if (bad >= 0) {
      const lim = config.limits[JOINT_NAMES[bad]];
      return {
        ok: false,
        reason: `θ${bad + 1} outside ${JOINT_NAMES[bad]} limits (${rad2deg(lim.min).toFixed(
          0,
        )}…${rad2deg(lim.max).toFixed(0)}°)`,
      };
    }
    const pose = checkPose(q, config.links, config.collision);
    if (pose.colliding) return { ok: false, reason: pose.issues[0] ?? 'pose collides' };
    return { ok: true, q, nearSingularity: false };
  }

  const target = toCartesian(move);
  const res = inverseKinematics(target, config.links, config.limits);
  if (!res.reachable) return { ok: false, reason: 'out of reach' };

  const withinLimits = res.solutions.filter((s) => s.withinLimits);
  if (withinLimits.length === 0) return { ok: false, reason: 'reachable only outside joint limits' };

  const usable = withinLimits.filter(
    (s) => !checkPose(s.q, config.links, config.collision).colliding,
  );
  if (usable.length === 0) {
    const { issues } = checkPose(withinLimits[0].q, config.links, config.collision);
    return { ok: false, reason: issues[0] ?? 'pose collides' };
  }

  const preferred = usable.filter((s) => s.branch === branch);
  const pool = preferred.length > 0 ? preferred : usable;
  const dist = (q: JointAngles) =>
    Math.abs(q[0] - fromQ[0]) + Math.abs(q[1] - fromQ[1]) + Math.abs(q[2] - fromQ[2]);
  const best = pool.reduce((a, b) => (dist(b.q) < dist(a.q) ? b : a));
  return { ok: true, q: best.q, nearSingularity: best.nearSingularity };
}

/** Resolve the whole program, chaining each move's start pose from the last. */
export function resolveProgram(
  moves: ProgramMove[],
  config: RobotConfig,
  branch: IkBranch,
  fromQ: JointAngles,
): ProgramResolution {
  const steps: ResolvedStep[] = [];
  const byId: Record<string, MoveResolution> = {};
  let firstError: string | null = null;
  let prevQ = fromQ;

  moves.forEach((move, i) => {
    const res = resolveMove(move, config, branch, prevQ);
    byId[move.id] = res;
    if (!res.ok) {
      if (move.enabled && firstError === null) firstError = `move ${i + 1}: ${res.reason}`;
      return;
    }
    // A disabled row still resolves (so the editor can show its status) but
    // must not shift the chain the enabled rows are planned from.
    if (!move.enabled) return;
    steps.push({ move, q: res.q, dwell: move.dwell });
    prevQ = res.q;
  });

  return { steps, byId, firstError };
}
