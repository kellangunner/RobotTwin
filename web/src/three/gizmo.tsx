// Shared primitives for the in-scene reference annotations: axis arrows, floating
// labels, and joint-rotation arrows. Pure visuals — nothing here reads the pose.

import { useMemo, type ReactNode } from 'react';
import * as THREE from 'three';
import { Html, Line } from '@react-three/drei';

export const AXIS_RED = '#dc2626'; // X
export const AXIS_GREEN = '#059669'; // Y — shoulder + elbow pitch axis
export const AXIS_BLUE = '#2563eb'; // Z — base yaw axis

export function Label({
  position,
  color,
  size = 11,
  children,
}: {
  position: [number, number, number];
  color: string;
  size?: number;
  children: ReactNode;
}) {
  return (
    <Html position={position} center style={{ pointerEvents: 'none' }}>
      <span
        style={{
          color,
          font: `700 ${size}px ui-monospace, SFMono-Regular, Menlo, monospace`,
          textShadow: '0 0 2px #fff, 0 0 3px #fff, 0 0 3px #fff',
          userSelect: 'none',
          whiteSpace: 'nowrap',
        }}
      >
        {children}
      </span>
    </Html>
  );
}

// An arrow built along +Y, then rotated onto a world axis by the caller.
export function Arrow({
  color,
  length,
  rotation,
  shaftR = 0.0014,
  headR = 0.0045,
  headLen = 0.013,
}: {
  color: string;
  length: number;
  rotation: [number, number, number];
  shaftR?: number;
  headR?: number;
  headLen?: number;
}) {
  return (
    <group rotation={rotation}>
      <mesh position={[0, length / 2, 0]}>
        <cylinderGeometry args={[shaftR, shaftR, length, 12]} />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh position={[0, length + headLen / 2, 0]}>
        <coneGeometry args={[headR, headLen, 16]} />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
}

/**
 * A curved arrow in the plane whose normal is `axis`, sweeping in the joint's
 * positive direction with an arrowhead at the leading end. The parameterization
 * makes increasing t the positive joint rotation: right-hand about +Z for the
 * base, right-hand about −Y for the pitch joints (matching RobotModel).
 */
export function RotationArc({
  axis,
  color,
  center,
  radius,
  start = Math.PI / 6,
  sweep = (5 * Math.PI) / 3,
  headR = 0.0038,
  headLen = 0.011,
}: {
  axis: 'Z' | 'Y';
  color: string;
  center: [number, number, number];
  radius: number;
  start?: number;
  sweep?: number;
  headR?: number;
  headLen?: number;
}) {
  const { points, headPos, headQuat } = useMemo(() => {
    const at = (t: number): [number, number, number] =>
      axis === 'Z'
        ? [radius * Math.cos(t), radius * Math.sin(t), 0]
        : [radius * Math.cos(t), 0, radius * Math.sin(t)];

    const segments = 48;
    const pts: [number, number, number][] = [];
    for (let i = 0; i <= segments; i++) pts.push(at(start + (sweep * i) / segments));

    const end = start + sweep;
    const tangent =
      axis === 'Z'
        ? new THREE.Vector3(-Math.sin(end), Math.cos(end), 0)
        : new THREE.Vector3(-Math.sin(end), 0, Math.cos(end));
    const quat = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      tangent.normalize(),
    );
    return { points: pts, headPos: at(end), headQuat: quat };
  }, [axis, radius, start, sweep]);

  return (
    <group position={center}>
      <Line points={points} color={color} lineWidth={2} />
      <mesh position={headPos} quaternion={headQuat}>
        <coneGeometry args={[headR, headLen, 12]} />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
}
