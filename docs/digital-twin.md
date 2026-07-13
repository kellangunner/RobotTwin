# Digital Twin (web) — Subsystem Documentation

## Overview

Interactive 3D digital twin of the 3-DOF arm, built so that **the drivetrain is the
independent variable**: per joint, the user picks a drive type (direct 1:1 / planetary /
cycloidal) and a reduction ratio. Efficiency, output backlash, torque cap, and gearbox inertia
are *derived* from the chosen type via characteristic models in the config — the twin exists to
dial in ratios, not to hand-tune gearbox internals. Every displayed quantity — torque margins,
achievable speeds, TCP backlash error, position resolution, move durations, skipped-step risk —
follows from those two choices.
Linkage geometry is fixed (see [linkage-geometry.md](linkage-geometry.md)) and loaded from
[config/robot.yaml](../config/robot.yaml).

Run it:

```
cd web
npm install
npm run dev     # http://localhost:5173
npm test        # core math unit tests (vitest)
```

## Architecture

```
web/src/
  core/          pure math, no rendering, no React — mirrors the future C++ core API
    config.ts        YAML → typed SI-unit RobotConfig
    units.ts         unit conversions, constants
    kinematics.ts    FK, analytical IK (4 branches), Jacobian, workspace boundary
    drivetrain.ts    stepper torque-speed model + gearbox (ratio/efficiency/backlash/cap)
    dynamics.ts      gravity torques, worst-case link inertias
    trajectory.ts    synchronized quintic point-to-point planner
    metrics.ts       aggregation: per-joint budgets, TCP metrics, trajectory audit
  state/         zustand store: independent variables, pose command, motion playback
  three/         React Three Fiber rendering ONLY (no kinematic computation)
  ui/            Tailwind control/metrics panels
```

The layering enforces the project rule that visualization is independent of robot math:
`three/` and `ui/` import from `state/` and `core/`, never the reverse. The C++20 reference
implementation of every `core/` module now lives in [`src/`](cpp-core.md); both test suites
assert the same numeric fixtures, and the remaining Phase 3 step is compiling it with emsdk
and swapping the store's imports to the WASM module.

## Algorithms & equations

**FK** (θ₂ from horizontal, θ₃ relative, world Z-up):
`r = L1·cosθ2 + L2·cos(θ2+θ3)`, `z = h + L1·sinθ2 + L2·sin(θ2+θ3)`,
`tcp = (r·cosθ1, r·sinθ1, z)`.

**IK** — analytical, O(1): law of cosines for θ₃ (`cosθ3 = (d² − L1² − L2²) / 2L1L2`), then
θ₂ by angle subtraction. Four branches are enumerated: {elbow-up, elbow-down} × {front,
base-flipped}. The base-flipped branch (θ₁ + 180°, negative planar radius, arm over the top)
matters because θ₂ spans 0–180°. Unreachable targets, joint-limit violations, straight-arm
singularity (|sin θ₃| small) and base-axis singularity (target on yaw axis) are all reported.

**Jacobian** — analytic 3×3 position Jacobian, validated against central finite differences in
tests. `det J ∝ r·sinθ3` gives the singularity measure.

**Stepper + gearbox model** — motor torque falls linearly from holding torque at standstill to
zero at max speed. At the joint: `τ_avail = min(τ_motor(ω·N)·N·η, τ_cap)`;
`ω_max,joint = ω_max,motor / N`; reflected inertia `= (J_rotor + J_gb)·N²`;
resolution `= step / µstep / N`. Backlash → TCP error via `Σ ‖J_i‖·b_i`.

**Drive-type models** (`core/gearboxModel.ts`, constants in `config/robot.yaml`
`gearbox_models`) — *direct*: ratio locked to 1, near-lossless. *Planetary*: a printed stage
tops out at 6:1; higher ratios stack stages, compounding efficiency (`0.88^stages`) and
accumulating backlash (0.6°/stage) and inertia. *Cycloidal*: 8:1–40:1 in one stage, 75 %
efficient, **negligible backlash by design assumption**, highest printed-torque cap.

**Trajectories** — synchronized quintic time scaling; duration chosen from per-joint velocity
(80 % of drivetrain ceiling) and acceleration (70 % of static torque margin over total inertia)
limits. Every planned move gets a **predictive audit**: 120 dense samples comparing
`|τ_gravity| + I_total·|q̈|` against the speed-derated available torque; a peak over 100 %
means an open-loop stepper would skip steps.

**Collision detection** (`core/collision.ts`, envelopes in `config/robot.yaml` `collision`) —
conservative envelopes: forearm+gripper capsule vs the ground plane, vs flat-topped cylinders
for the base housing and rotating column (a capsule would dome a squat cylinder's top by its
full radius — far too conservative), and vs a shoulder-joint sphere for deep elbow folds (the
forearm capsule is trimmed near its own elbow, where adjacent links legitimately meet). Every
command path is gated: IK drops colliding branches, joint-slider commands are refused, planned
trajectories are sampled densely pose-by-pose before executing (endpoints being safe does not
make the sweep safe), and CSV waypoints are validated row-by-row with per-line reasons.

**Waypoint sequences** — a CSV of rows interpreted per the active tab (`x,y,z` mm via IK with
branch continuity, or `θ1,θ2,θ3` deg) runs as a chain of quintic legs with a 0.8 s dwell at
each waypoint; the final report aggregates the worst torque utilization across all legs.

## Assumptions & limitations

- Rod/point-mass model (no full inertia tensors yet); base-joint inertia uses the extended-arm
  worst case regardless of pose.
- Linear torque-speed curve; real steppers have resonance dips and a nonlinear knee.
- No Coriolis/centrifugal terms; acceptable at these speeds, revisit with dynamics phase.
- Backlash treated as a static worst-case position band, not simulated as hysteresis.
- Drive-type characteristics (per-stage efficiency, backlash, torque caps) are representative
  estimates for printed PETG gearboxes; replace with measured values once prototypes exist.
- Cycloidal backlash is assumed negligible (a property of preloaded cycloidal drives, adopted
  as a design requirement for ours).

## Future improvements

- Replace `core/` with the C++20 library compiled to WASM (identical API).
- Pose-dependent inertia matrix; inverse dynamics along trajectories.
- Draggable 3D target (transform gizmo), joint-space trace, torque-over-time plots.
- Persist gearbox configurations back to YAML for the CAD pipeline (Phase 4).
