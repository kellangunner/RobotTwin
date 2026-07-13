// Collision detection: conservative capsule/sphere envelopes around the links
// and static structure, plus the ground plane (table, z = 0).
//
// Checked pairs — chosen from what this arm can physically hit given its
// joint limits (θ2 ∈ [0°,180°] keeps the upper arm at or above the shoulder):
//   forearm+gripper vs ground          (reaching below the table)
//   forearm+gripper vs base column     (folding down across the yaw axis)
//   forearm+gripper vs base housing    (reaching low near the base)
//   forearm+gripper vs shoulder joint  (deep elbow folds)
//   upper arm       vs ground          (defensive; limits should prevent it)
//
// The forearm capsule is trimmed near its own elbow joint for the shoulder
// check — adjacent links legitimately meet at their shared joint.

import type { LinkGeometry } from './config';
import type { JointAngles, Vec3 } from './kinematics';
import { forwardKinematics } from './kinematics';
import type { TrajectoryPlan } from './trajectory';
import { sampleTrajectory } from './trajectory';

export interface CollisionModel {
  groundClearance: number; // m
  columnRadius: number;
  columnTop: number;       // column capsule spans z = 0 .. columnTop
  baseRadius: number;
  baseTop: number;
  shoulderRadius: number;
  upperArmRadius: number;
  forearmRadius: number;
  gripperExtent: number;   // m past the TCP along the forearm axis
  elbowTrim: number;       // fraction of the forearm ignored next to the elbow
}

export interface CollisionCheck {
  colliding: boolean;
  /** Human-readable descriptions of every contact found. */
  issues: string[];
}

const sub = (a: Vec3, b: Vec3): Vec3 => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a: Vec3, b: Vec3): Vec3 => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a: Vec3, k: number): Vec3 => [a[0] * k, a[1] * k, a[2] * k];
const dot = (a: Vec3, b: Vec3): number => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a: Vec3): number => Math.hypot(a[0], a[1], a[2]);
const clamp01 = (t: number): number => Math.min(1, Math.max(0, t));

/** Minimum distance from point p to segment [a, b]. */
export function pointSegmentDistance(p: Vec3, a: Vec3, b: Vec3): number {
  const ab = sub(b, a);
  const len2 = dot(ab, ab);
  const t = len2 > 0 ? clamp01(dot(sub(p, a), ab) / len2) : 0;
  return norm(sub(p, add(a, scale(ab, t))));
}

/** Minimum distance between segments [p1, q1] and [p2, q2]. */
export function segmentSegmentDistance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3): number {
  const d1 = sub(q1, p1);
  const d2 = sub(q2, p2);
  const r = sub(p1, p2);
  const a = dot(d1, d1);
  const e = dot(d2, d2);
  const f = dot(d2, r);

  let s: number;
  let t: number;
  if (a <= 1e-12 && e <= 1e-12) {
    s = 0;
    t = 0;
  } else if (a <= 1e-12) {
    s = 0;
    t = clamp01(f / e);
  } else {
    const c = dot(d1, r);
    if (e <= 1e-12) {
      t = 0;
      s = clamp01(-c / a);
    } else {
      const b = dot(d1, d2);
      const denom = a * e - b * b;
      s = denom > 1e-12 ? clamp01((b * f - c * e) / denom) : 0;
      t = clamp01((b * s + f) / e);
      s = clamp01((b * t - c) / a);
    }
  }
  return norm(sub(add(p1, scale(d1, s)), add(p2, scale(d2, t))));
}

/**
 * Distance from point p to a flat-topped cylinder of radius r spanning
 * z = 0..top, centered on the world Z axis. Returns 0 inside. A capsule would
 * round the top into a dome of radius r — far too conservative for the squat,
 * wide base housing.
 */
export function pointCylinderDistance(p: Vec3, r: number, top: number): number {
  const radial = Math.hypot(p[0], p[1]) - r;
  const vertical = p[2] > top ? p[2] - top : p[2] < 0 ? -p[2] : 0;
  if (radial <= 0) return vertical; // inside radially → 0 when also inside vertically
  return Math.hypot(radial, vertical);
}

/** Minimum point-cylinder distance over a densely sampled segment [a, b]. */
function segmentCylinderDistance(a: Vec3, b: Vec3, r: number, top: number, samples = 32): number {
  let min = Infinity;
  for (let k = 0; k <= samples; k++) {
    const t = k / samples;
    const p: Vec3 = [
      a[0] + (b[0] - a[0]) * t,
      a[1] + (b[1] - a[1]) * t,
      a[2] + (b[2] - a[2]) * t,
    ];
    min = Math.min(min, pointCylinderDistance(p, r, top));
  }
  return min;
}

/** Check a single pose for self-collision and ground contact. */
export function checkPose(
  q: JointAngles,
  geom: LinkGeometry,
  model: CollisionModel,
): CollisionCheck {
  const { shoulder: S, elbow: E, tcp: T } = forwardKinematics(q, geom);
  const issues: string[] = [];

  // forearm capsule, extended past the TCP to cover the gripper
  const fLen = norm(sub(T, E));
  const u = fLen > 1e-9 ? scale(sub(T, E), 1 / fLen) : ([1, 0, 0] as Vec3);
  const fEnd = add(T, scale(u, model.gripperExtent));
  const fStartTrimmed = add(E, scale(u, fLen * model.elbowTrim));

  // ground plane
  const forearmMinZ = Math.min(E[2], fEnd[2]) - model.forearmRadius;
  if (forearmMinZ < model.groundClearance) issues.push('forearm/gripper hits the ground');
  const upperMinZ = Math.min(S[2], E[2]) - model.upperArmRadius;
  if (upperMinZ < model.groundClearance) issues.push('upper arm hits the ground');

  // static structure (flat-topped cylinders on the yaw axis)
  if (segmentCylinderDistance(E, fEnd, model.columnRadius, model.columnTop) < model.forearmRadius) {
    issues.push('forearm/gripper hits the base column');
  }
  if (segmentCylinderDistance(E, fEnd, model.baseRadius, model.baseTop) < model.forearmRadius) {
    issues.push('forearm/gripper hits the base housing');
  }

  // shoulder joint housing (adjacent-link fold; trimmed at the elbow)
  if (pointSegmentDistance(S, fStartTrimmed, fEnd) < model.forearmRadius + model.shoulderRadius) {
    issues.push('forearm/gripper hits the shoulder joint');
  }

  return { colliding: issues.length > 0, issues };
}

/**
 * Check every pose along a planned trajectory (dense sampling, endpoints
 * included). Returns the first contact found.
 */
export function checkPath(
  plan: TrajectoryPlan,
  geom: LinkGeometry,
  model: CollisionModel,
  samples = 60,
): CollisionCheck {
  for (let k = 0; k <= samples; k++) {
    const { q } = sampleTrajectory(plan, (plan.duration * k) / samples);
    const res = checkPose(q, geom, model);
    if (res.colliding) return res;
  }
  return { colliding: false, issues: [] };
}
