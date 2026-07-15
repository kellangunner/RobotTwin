// A small corner gizmo for the vertical reference the flat floor can't show: the
// +Z (up) axis and the shoulder/elbow pitch direction (θ2/θ3 about −Y, the arm's
// up/down swing). The horizontal plane — X, Y and base yaw — is annotated by
// WorkspaceQuadrants. Pure visuals: it never reads the live pose.

import { Arrow, AXIS_BLUE, AXIS_GREEN, Label, RotationArc } from './gizmo';

// Sit on the table, front corner toward the default camera and outside the
// arm's ~0.24 m reach, so the gizmo never overlaps the moving robot.
const PLACEMENT: [number, number, number] = [0.2, -0.2, 0.002];

const Z_LEN = 0.055; // m — up arrow
const Y_LEN = 0.03; // m — short pitch-axis anchor

const TO_Y: [number, number, number] = [0, 0, 0]; // default arrow already points +Y
const TO_Z: [number, number, number] = [Math.PI / 2, 0, 0]; // +Y → +Z

export function ReferenceFrame() {
  return (
    <group position={PLACEMENT}>
      <Arrow color={AXIS_BLUE} length={Z_LEN} rotation={TO_Z} />
      <Label position={[0, 0, Z_LEN + 0.016]} color={AXIS_BLUE}>
        Z
      </Label>

      {/* short Y stub anchors the pitch axis so the ring below reads as "about Y" */}
      <Arrow color={AXIS_GREEN} length={Y_LEN} rotation={TO_Y} shaftR={0.0012} headR={0.004} />

      {/* shoulder + elbow pitch: vertical ring about Y (the arm's up/down swing) */}
      <RotationArc axis="Y" color={AXIS_GREEN} center={[0, 0.014, 0.03]} radius={0.016} />
      <Label position={[0.004, 0.014, 0.055]} color={AXIS_GREEN}>
        θ2·θ3
      </Label>
    </group>
  );
}
