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
make the sweep safe), and program rows are validated individually with a per-row reason.

**The move program** (`core/program.ts`, `core/programResolve.ts`, `ui/ProgramPanel.tsx`) — the
sliders *pose* the arm; the program is how you *command* it, and it is the twin's model of how
the physical robot actually works: a queue of moves, each planned, executed to completion, then
held for a dwell before the next starts. Nothing follows the cursor.

Each row is one of three frames, plus a dwell and an enable flag:

| Badge | Frame | Values |
|---|---|---|
| `XYZ` | Cartesian | `x, y, z` mm — solved by IK, elbow branch chained from the previous row |
| `RθZ` | Polar (cylindrical) | `r` mm, `θ` deg, `z` mm — about the base yaw axis |
| `θ` | Joint space | `θ1, θ2, θ3` deg — commanded directly, no IK |

Polar is the arm's own natural frame: FK is literally `tcp = (r·cosθ1, r·sinθ1, z)`, so `r` is
planar reach, `θ` *is* the base angle, and `z` is height. Sweeping the base at fixed radius is
one number here and two coupled ones in Cartesian — a program of `RθZ` rows differing only in
`θ` exports as `MOVEJ` lines with identical shoulder and elbow angles. Negative `r` is legal and
means the same point as `(|r|, θ+180°)`; the IK already models that as the base-flipped branch.
Positional rows (`XYZ`, `RθZ`) share one code path — polar converts to Cartesian, then both go
through the same IK, limit and collision gates.

Rows resolve as you type — reach, joint limits and collision are reported inline per row — so an
unreachable waypoint is something you see and fix, not something silently dropped at load time.

`program.ts` is deliberately kinematics-free (rows, units, text round-trips) so it unit tests in
isolation; `programResolve.ts` is the half that goes through the C++ core.

Running resolves every enabled row up front and commits that snapshot: editing mid-run affects
the *next* run, matching the firmware's contract that a queued move is fixed once accepted. The
runner supports run / pause / resume / single-step / loop. Pause shifts the motion clock rather
than replanning, so a paused leg resumes on the exact profile it was already following. A
swept-path collision that only appears between two individually-safe waypoints stops the run
where it is and names the offending move. The final report aggregates the worst torque
utilization across every leg.

**Program I/O** — CSV rows are `[kind,] a, b, c [, dwell]`; the optional leading `C`/`P`/`J` tag
(or the spelled-out `cartesian`/`polar`/`joints`) lets a mixed program round-trip, and `!` before
it marks a disabled row. Import *appends to the editor* rather than auto-running, so an imported
file can be inspected and corrected first.

Export as CSV (everything, disabled rows included) or as a **robot script** for
`python/run_script.py` — only the enabled rows that resolved, emitted as `MOVEJ` with the joint
angles the twin validated. Positional rows go out as joint angles too, with the authored target
kept as a comment: the twin has already chosen an IK branch and proved that pose safe, so sending
angles guarantees the hardware reproduces what was simulated instead of re-solving IK on the
ESP32 and possibly landing on the other elbow branch. It also lets the runner split each move
joint-by-joint for the shared-rail bench, which it cannot do for `MOVEL` — and there is no polar
verb on the wire at all, so `RθZ` rows could not round-trip any other way.

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
