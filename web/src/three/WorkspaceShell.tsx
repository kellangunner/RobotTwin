// Translucent lathe of the reachable (r, z) region swept through the base yaw
// range. Geometry math comes from core/kinematics; this file only meshes it.

import { useMemo } from 'react';
import * as THREE from 'three';
import { config, useTwinStore } from '../state/store';
import { workspaceBoundary } from '../core/api';

export function WorkspaceShell() {
  const show = useTwinStore((s) => s.showWorkspace);

  const geometry = useMemo(() => {
    const boundary = workspaceBoundary(config.links, config.limits);
    const pts = boundary.map(([r, z]) => new THREE.Vector2(Math.max(0, r), z));
    const yaw = config.limits.base;
    // LatheGeometry revolves around +Y; the mesh is rotated below so +Y → +Z,
    // which shifts lathe φ relative to world azimuth by +90°.
    return new THREE.LatheGeometry(pts, 64, yaw.min + Math.PI / 2, yaw.max - yaw.min);
  }, []);

  if (!show) return null;
  return (
    <mesh geometry={geometry} rotation={[Math.PI / 2, 0, 0]}>
      <meshBasicMaterial
        color="#0369a1"
        transparent
        opacity={0.08}
        side={THREE.DoubleSide}
        depthWrite={false}
      />
    </mesh>
  );
}
