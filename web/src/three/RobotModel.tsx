// Renders the arm from config geometry as nested groups whose rotations are
// the joint angles. Purely visual — all poses come from the store.

import { useTwinStore, config } from '../state/store';

const { baseHeight: H, upperArm: L1, forearm: L2 } = config.links;

// visual sizes (m) echoing the real hardware
const NEMA = 0.0423;
const ARM_W = 0.034;
const ARM_T = 0.046;
const JOINT_R = 0.024;

const bodyColor = '#8fa3b8';
const jointColor = '#3d4c5e';
const motorColor = '#22272e';

function JointCylinder({ radius = JOINT_R, width = 0.05 }: { radius?: number; width?: number }) {
  // pitch axes run along Y — a default three cylinder is already Y-aligned
  return (
    <mesh castShadow>
      <cylinderGeometry args={[radius, radius, width, 32]} />
      <meshStandardMaterial color={jointColor} metalness={0.3} roughness={0.5} />
    </mesh>
  );
}

export function RobotModel() {
  const q = useTwinStore((s) => s.q);

  return (
    <group>
      {/* static base housing */}
      <mesh position={[0, 0, 0.03]} rotation={[Math.PI / 2, 0, 0]} castShadow>
        <cylinderGeometry args={[0.07, 0.075, 0.06, 48]} />
        <meshStandardMaterial color={jointColor} metalness={0.2} roughness={0.6} />
      </mesh>

      {/* θ1: base yaw about Z */}
      <group rotation={[0, 0, q[0]]}>
        {/* rotating column up to the shoulder */}
        <mesh position={[0, 0, 0.06 + (H - 0.06) / 2]} rotation={[Math.PI / 2, 0, 0]} castShadow>
          <cylinderGeometry args={[0.026, 0.032, H - 0.06, 32]} />
          <meshStandardMaterial color={bodyColor} metalness={0.1} roughness={0.6} />
        </mesh>
        {/* shoulder motor block beside the column */}
        <mesh position={[-0.045, 0, H - 0.02]} castShadow>
          <boxGeometry args={[NEMA, NEMA, 0.048]} />
          <meshStandardMaterial color={motorColor} roughness={0.7} />
        </mesh>

        {/* θ2: shoulder pitch about local Y at height H */}
        <group position={[0, 0, H]} rotation={[0, -q[1], 0]}>
          <JointCylinder width={0.056} />
          <mesh position={[L1 / 2, 0, 0]} castShadow>
            <boxGeometry args={[L1, ARM_W, ARM_T]} />
            <meshStandardMaterial color={bodyColor} metalness={0.1} roughness={0.55} />
          </mesh>

          {/* θ3: elbow pitch, motor mounted here (mass model matches) */}
          <group position={[L1, 0, 0]} rotation={[0, -q[2], 0]}>
            <JointCylinder />
            <mesh position={[0, ARM_W + 0.002, 0]} castShadow>
              <boxGeometry args={[NEMA, 0.048, NEMA]} />
              <meshStandardMaterial color={motorColor} roughness={0.7} />
            </mesh>
            <mesh position={[L2 / 2, 0, 0]} castShadow>
              <boxGeometry args={[L2, ARM_W * 0.85, ARM_T * 0.8]} />
              <meshStandardMaterial color={bodyColor} metalness={0.1} roughness={0.55} />
            </mesh>

            {/* gripper fingers at the TCP */}
            <group position={[L2, 0, 0]}>
              <mesh position={[0.012, 0.011, 0]} castShadow>
                <boxGeometry args={[0.03, 0.006, 0.014]} />
                <meshStandardMaterial color={jointColor} roughness={0.6} />
              </mesh>
              <mesh position={[0.012, -0.011, 0]} castShadow>
                <boxGeometry args={[0.03, 0.006, 0.014]} />
                <meshStandardMaterial color={jointColor} roughness={0.6} />
              </mesh>
              <mesh>
                <sphereGeometry args={[0.007, 16, 16]} />
                <meshStandardMaterial color="#e8b339" emissive="#7a5a10" />
              </mesh>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}
