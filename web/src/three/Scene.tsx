// The 3D viewport: robot, workspace shell, target marker, TCP trace, and the
// motion clock (useFrame → store.tick). Z-up world to match robot convention.

import { Canvas } from '@react-three/fiber';
import { Line, OrbitControls } from '@react-three/drei';
import { useTwinStore } from '../state/store';
import { RobotModel } from './RobotModel';
import { ReferenceFrame } from './ReferenceFrame';
import { WorkspaceQuadrants } from './WorkspaceQuadrants';
import { WorkspaceShell } from './WorkspaceShell';

function TargetMarker() {
  const target = useTwinStore((s) => s.target);
  const status = useTwinStore((s) => s.ikStatus);
  const color =
    status.kind === 'ok' ? (status.nearSingularity ? '#d97706' : '#059669') : '#dc2626';
  return (
    <group position={target}>
      <mesh>
        <sphereGeometry args={[0.008, 24, 24]} />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh>
        <sphereGeometry args={[0.016, 24, 24]} />
        <meshBasicMaterial color={color} transparent opacity={0.25} />
      </mesh>
    </group>
  );
}

function TcpTrace() {
  const trace = useTwinStore((s) => s.trace);
  if (trace.length < 2) return null;
  return <Line points={trace} color="#b45309" lineWidth={1.5} transparent opacity={0.85} />;
}

export function Scene() {
  return (
    <Canvas
      shadows
      camera={{ position: [0.45, -0.42, 0.34], up: [0, 0, 1], fov: 42, near: 0.01, far: 10 }}
    >
      <color attach="background" args={['#e3e7ea']} />
      <ambientLight intensity={0.55} />
      <directionalLight
        position={[0.5, -0.8, 1.2]}
        intensity={1.4}
        castShadow
        shadow-mapSize={[1024, 1024]}
      />
      <directionalLight position={[-0.6, 0.5, 0.4]} intensity={0.35} />

      {/* table */}
      <mesh position={[0, 0, -0.003]} rotation={[Math.PI / 2, 0, 0]} receiveShadow>
        <cylinderGeometry args={[0.34, 0.34, 0.006, 64]} />
        <meshStandardMaterial color="#dfe3e7" roughness={0.9} />
      </mesh>
      <gridHelper
        args={[0.68, 17, '#8d99a6', '#b4bdc6']}
        rotation={[Math.PI / 2, 0, 0]}
        position={[0, 0, 0.0005]}
      />
      <WorkspaceQuadrants />

      <RobotModel />
      <ReferenceFrame />
      <WorkspaceShell />
      <TargetMarker />
      <TcpTrace />

      <OrbitControls makeDefault target={[0, 0, 0.14]} maxDistance={2} minDistance={0.15} />
    </Canvas>
  );
}
