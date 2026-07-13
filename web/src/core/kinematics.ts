// Analytical kinematics for the 3-DOF arm (base yaw θ1, shoulder pitch θ2, elbow pitch θ3).
//
// Conventions (see docs/linkage-geometry.md):
//   world Z up, X forward; θ2 measured from horizontal (positive up);
//   θ3 relative to the upper arm (0 = straight arm).
//
//   r   = L1·cos θ2 + L2·cos(θ2+θ3)
//   z   = h  + L1·sin θ2 + L2·sin(θ2+θ3)
//   tcp = (r·cos θ1, r·sin θ1, z)
//
// This module is pure math — no rendering, no state.

import type { JointLimits, LinkGeometry, JointName } from './config';
import { JOINT_NAMES } from './config';

export type Vec3 = [number, number, number];
export type JointAngles = [number, number, number]; // [θ1, θ2, θ3] rad

/** Angular tolerance below which the straight-arm / base singularities are flagged. */
const SINGULARITY_TOL = 0.05; // rad ≈ 2.9°
const REACH_TOL = 1e-9;

export interface FkResult {
  shoulder: Vec3;
  elbow: Vec3;
  tcp: Vec3;
}

export function forwardKinematics(q: JointAngles, geom: LinkGeometry): FkResult {
  const [q1, q2, q3] = q;
  const { baseHeight: h, upperArm: L1, forearm: L2 } = geom;
  const c1 = Math.cos(q1), s1 = Math.sin(q1);
  const c2 = Math.cos(q2), s2 = Math.sin(q2);
  const c23 = Math.cos(q2 + q3), s23 = Math.sin(q2 + q3);

  const rElbow = L1 * c2;
  const r = L1 * c2 + L2 * c23;
  return {
    shoulder: [0, 0, h],
    elbow: [rElbow * c1, rElbow * s1, h + L1 * s2],
    tcp: [r * c1, r * s1, h + L1 * s2 + L2 * s23],
  };
}

export type IkBranch = 'elbow-up' | 'elbow-down';

export interface IkSolution {
  q: JointAngles;
  branch: IkBranch;
  /** Base yawed 180° with the arm reaching back over the top. */
  baseFlipped: boolean;
  withinLimits: boolean;
  violated: JointName[];
  nearSingularity: boolean;
}

export interface IkResult {
  reachable: boolean;
  /** Up to 4 solutions: {front, base-flipped} × {elbow-up, elbow-down}. */
  solutions: IkSolution[];
  /** Target is on/near the base yaw axis: θ1 is undefined. */
  baseSingular: boolean;
}

const normalizeAngle = (a: number): number => Math.atan2(Math.sin(a), Math.cos(a));

export function inverseKinematics(
  target: Vec3,
  geom: LinkGeometry,
  limits: Record<JointName, JointLimits>,
): IkResult {
  const { baseHeight: h, upperArm: L1, forearm: L2 } = geom;
  const [x, y, z] = target;
  const r = Math.hypot(x, y);
  const baseSingular = r < 1e-6;
  const q1Front = baseSingular ? 0 : Math.atan2(y, x);

  const dz = z - h;
  const d2 = r * r + dz * dz;
  const c3 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);

  if (c3 > 1 + REACH_TOL || c3 < -1 - REACH_TOL) {
    return { reachable: false, solutions: [], baseSingular };
  }
  const c3c = Math.min(1, Math.max(-1, c3));

  const solutions: IkSolution[] = [];
  // The same TCP is reachable facing the target (planar radius +r) or yawed
  // 180° reaching back over the top (planar radius −r, needs θ2 > 90°).
  for (const flipped of [false, true] as const) {
    const q1 = flipped ? normalizeAngle(q1Front + Math.PI) : q1Front;
    const rp = flipped ? -r : r;
    for (const sign of [1, -1] as const) {
      const q3 = sign * Math.acos(c3c);
      const q2 = normalizeAngle(
        Math.atan2(dz, rp) - Math.atan2(L2 * Math.sin(q3), L1 + L2 * Math.cos(q3)),
      );
      const q: JointAngles = [q1, q2, q3];

      const violated = JOINT_NAMES.filter((name, i) => {
        const lim = limits[name];
        return q[i] < lim.min - 1e-9 || q[i] > lim.max + 1e-9;
      });

      solutions.push({
        q,
        branch: sign > 0 ? 'elbow-down' : 'elbow-up',
        baseFlipped: flipped,
        withinLimits: violated.length === 0,
        violated,
        nearSingularity: Math.abs(Math.sin(q3)) < SINGULARITY_TOL || baseSingular,
      });

      // Straight arm: both elbow signs coincide — keep one per flip.
      if (Math.abs(q3) < 1e-9) break;
    }
  }

  return { reachable: true, solutions, baseSingular };
}

/**
 * 3×3 position Jacobian, columns = ∂tcp/∂θi (m/rad).
 * Column norms are also used for backlash → TCP error propagation.
 */
export function jacobian(q: JointAngles, geom: LinkGeometry): [Vec3, Vec3, Vec3] {
  const [q1, q2, q3] = q;
  const { upperArm: L1, forearm: L2 } = geom;
  const c1 = Math.cos(q1), s1 = Math.sin(q1);
  const c2 = Math.cos(q2), s2 = Math.sin(q2);
  const c23 = Math.cos(q2 + q3), s23 = Math.sin(q2 + q3);

  const r = L1 * c2 + L2 * c23;
  const drdq2 = -L1 * s2 - L2 * s23;
  const dzdq2 = L1 * c2 + L2 * c23;

  return [
    [-r * s1, r * c1, 0],
    [drdq2 * c1, drdq2 * s1, dzdq2],
    [-L2 * s23 * c1, -L2 * s23 * s1, L2 * c23],
  ];
}

export const columnNorm = (col: Vec3): number => Math.hypot(col[0], col[1], col[2]);

/**
 * Manipulability-style singularity measure: |det J| normalized by L1·L2·reach.
 * → 0 at the straight-arm boundary and on the base axis.
 */
export function singularityMeasure(q: JointAngles, geom: LinkGeometry): number {
  const { upperArm: L1, forearm: L2 } = geom;
  const r = L1 * Math.cos(q[1]) + L2 * Math.cos(q[1] + q[2]);
  // det(J) for this arm factors to r · L1 · L2 · sin(q3)
  return Math.abs((r / (L1 + L2)) * Math.sin(q[2]));
}

/**
 * Boundary polyline of the reachable region in the (r, z) arm plane, honoring
 * shoulder/elbow limits. Used by the renderer to lathe a workspace shell —
 * the math lives here so rendering stays computation-free.
 */
export function workspaceBoundary(
  geom: LinkGeometry,
  limits: Record<JointName, JointLimits>,
  samples = 40,
): Array<[number, number]> {
  const { baseHeight: h, upperArm: L1, forearm: L2 } = geom;
  const s = limits.shoulder;
  const e = limits.elbow;
  // Only the elbow-up half (q3 <= 0) plus straight-arm outer edge; mirrored
  // configurations trace the same outer boundary.
  const point = (q2: number, q3: number): [number, number] => [
    L1 * Math.cos(q2) + L2 * Math.cos(q2 + q3),
    h + L1 * Math.sin(q2) + L2 * Math.sin(q2 + q3),
  ];

  const pts: Array<[number, number]> = [];
  // outer edge: straight arm, sweep shoulder
  for (let i = 0; i <= samples; i++) {
    pts.push(point(s.min + ((s.max - s.min) * i) / samples, 0));
  }
  // shoulder at max, fold elbow toward its negative limit
  for (let i = 0; i <= samples; i++) {
    pts.push(point(s.max, (e.min * i) / samples));
  }
  // inner edge: elbow at its negative limit, sweep shoulder back down
  for (let i = 0; i <= samples; i++) {
    pts.push(point(s.max - ((s.max - s.min) * i) / samples, e.min));
  }
  // shoulder at min, unfold elbow back to straight
  for (let i = 0; i <= samples; i++) {
    pts.push(point(s.min, e.min - (e.min * i) / samples));
  }
  return pts;
}
