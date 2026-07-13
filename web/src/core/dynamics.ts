// Gravity loading and effective inertia. Point/rod mass model:
//   upper arm     — rod, CoM at L1/2 from the shoulder
//   elbow motor   — point mass at the elbow (NEMA 17 mounted there)
//   forearm       — rod, CoM at L2/2 from the elbow
//   gripper+load  — point mass at the TCP

import type { LinkGeometry, MassModel } from './config';
import type { JointAngles } from './kinematics';
import { GRAVITY } from './units';

/** Static torque (N·m) each joint must hold against gravity at pose q. */
export function gravityTorques(
  q: JointAngles,
  geom: LinkGeometry,
  masses: MassModel,
  payload: number,
): [number, number, number] {
  const { upperArm: L1, forearm: L2 } = geom;
  const c2 = Math.cos(q[1]);
  const c23 = Math.cos(q[1] + q[2]);
  const mTip = masses.gripper + payload;

  // moment arms are horizontal distances from each joint axis
  const tauElbow = GRAVITY * (masses.forearm * (L2 / 2) * c23 + mTip * L2 * c23);
  const tauShoulder =
    GRAVITY *
      ((masses.upperArm * (L1 / 2) + masses.elbowMotor * L1) * c2 +
        masses.forearm * (L1 * c2 + (L2 / 2) * c23) +
        mTip * (L1 * c2 + L2 * c23)) ;

  return [0, tauShoulder, tauElbow]; // base axis is vertical → no gravity load
}

/**
 * Conservative (fully-extended pose) link-side inertia about each joint axis,
 * used for trajectory acceleration limits. Rods as m·L²/3, point masses m·L².
 */
export function worstCaseLinkInertia(
  geom: LinkGeometry,
  masses: MassModel,
  payload: number,
): [number, number, number] {
  const { upperArm: L1, forearm: L2 } = geom;
  const mTip = masses.gripper + payload;
  const reach = L1 + L2;

  const aboutElbow = (masses.forearm * L2 * L2) / 3 + mTip * L2 * L2;
  const aboutShoulder =
    (masses.upperArm * L1 * L1) / 3 +
    masses.elbowMotor * L1 * L1 +
    masses.forearm * ((L1 + L2 / 2) ** 2) +
    mTip * reach * reach;
  // base spins the whole extended arm about the vertical axis
  const aboutBase = aboutShoulder;

  return [aboutBase, aboutShoulder, aboutElbow];
}
