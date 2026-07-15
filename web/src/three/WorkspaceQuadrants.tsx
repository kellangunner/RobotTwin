// Splits the workspace floor into four light quadrants and labels the world axes
// on the ground, so cardinal directions and the base-yaw orientation (θ1 about
// +Z) read at a glance. The vertical axis (Z) and pitch joints are annotated by
// the corner ReferenceFrame. Pure visuals — nothing here reads the pose.

import { Line } from '@react-three/drei';
import { AXIS_BLUE, AXIS_GREEN, AXIS_RED, Label, RotationArc } from './gizmo';

const TABLE_R = 0.34; // matches the table/grid radius in Scene
const RIM = TABLE_R + 0.022; // axis labels just past the table edge

// Two restrained tints on the diagonal pairs — a drafting-sheet checkerboard,
// kept light so the grid and shadows still read through them.
const COOL = '#5b8fbf';
const WARM = '#d08a3c';
const FILL_OPACITY = 0.14;

const FILL_Z = 0.0003; // above the table top (0), below the grid (0.0005)
const LINE_Z = 0.0009; // colored axes sit above the grid so they stay visible

function Quadrant({ startRad, color }: { startRad: number; color: string }) {
  return (
    <mesh position={[0, 0, FILL_Z]}>
      <circleGeometry args={[TABLE_R, 48, startRad, Math.PI / 2]} />
      <meshBasicMaterial color={color} transparent opacity={FILL_OPACITY} depthWrite={false} />
    </mesh>
  );
}

export function WorkspaceQuadrants() {
  return (
    <group>
      {/* four light quadrants, split by the world X and Y axes */}
      <Quadrant startRad={0} color={COOL} />
      <Quadrant startRad={Math.PI / 2} color={WARM} />
      <Quadrant startRad={Math.PI} color={COOL} />
      <Quadrant startRad={(3 * Math.PI) / 2} color={WARM} />

      {/* world axes painted on the floor, colored to match the labels */}
      <Line
        points={[
          [-TABLE_R, 0, LINE_Z],
          [TABLE_R, 0, LINE_Z],
        ]}
        color={AXIS_RED}
        lineWidth={1.5}
        transparent
        opacity={0.55}
      />
      <Line
        points={[
          [0, -TABLE_R, LINE_Z],
          [0, TABLE_R, LINE_Z],
        ]}
        color={AXIS_GREEN}
        lineWidth={1.5}
        transparent
        opacity={0.55}
      />

      {/* rim labels give the quadrants meaning */}
      <Label position={[RIM, 0, 0.005]} color={AXIS_RED}>
        +X
      </Label>
      <Label position={[-RIM, 0, 0.005]} color={AXIS_RED}>
        −X
      </Label>
      <Label position={[0, RIM, 0.005]} color={AXIS_GREEN}>
        +Y
      </Label>
      <Label position={[0, -RIM, 0.005]} color={AXIS_GREEN}>
        −Y
      </Label>

      {/* base yaw (θ1): the base spins this way about +Z — shown flat on the floor */}
      <RotationArc
        axis="Z"
        color={AXIS_BLUE}
        center={[0, 0, LINE_Z]}
        radius={0.11}
        start={Math.PI / 9}
        sweep={Math.PI / 2}
      />
      <Label position={[0.052, 0.108, 0.006]} color={AXIS_BLUE}>
        θ1
      </Label>
    </group>
  );
}
