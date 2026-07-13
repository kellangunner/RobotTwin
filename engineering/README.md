# Engineering — CAD generation

CAD is downstream of simulation: link geometry comes from
[config/robot.yaml](../config/robot.yaml) (90 / 120 / 120 mm, frozen per
[docs/linkage-geometry.md](../docs/linkage-geometry.md)); this folder turns it
into printable rigid members.

## Layout

```
engineering/
  fusion/robot_linkages/   Fusion script that generates the linkages
  f3d/                     Fusion archive exports   (generated)
  step/                    STEP exports             (generated)
  stl/                     per-part STL exports     (generated)
```

## Running the generator

1. Open Autodesk Fusion.
2. UTILITIES → ADD-INS → Scripts and Add-Ins → Scripts tab → "+" →
   select `engineering/fusion/robot_linkages/`.
3. Run `robot_linkages`. It creates a new design (Direct Modeling — the
   script *is* the parametric layer; edit a constant and re-run) and writes:
   - `f3d/rt-arm-3dof_linkages.f3d` — full assembly archive
   - `step/rt-arm-3dof_linkages.step` — full assembly STEP
   - `stl/rt_base_pan.stl`, `rt_yaw_column.stl`, `rt_upper_arm.stl`,
     `rt_forearm.stl` — one per printed member

A summary dialog reports exports, build-volume checks, and any config
fallbacks.

## The four rigid members

| Part | Role | Bounding (mm) |
|---|---|---|
| `rt_base_pan` | Stationary base; hollow drive bay for the future base motor + gearbox cartridge; Ø68 output opening + Ø78 bolt circle in the top plate | Ø140 × 50 |
| `rt_yaw_column` | θ₁ output: hub disc over the pan opening, clevis ears up to the shoulder axis (z = 90), shoulder gearbox drum | 76 × 90 × 78 |
| `rt_upper_arm` | Shoulder hub → elbow clevis (axes exactly 120 mm apart), elbow gearbox drum + elbow NEMA 17 mount | 178 × 82 × 76 — print diagonally on the A1 Mini |
| `rt_forearm` | Elbow hub → gripper interface plate; outer plate face is the TCP plane | 140 × 40 × 40 |

## Joint architecture (both pitch joints identical)

- **Structural loads** ride on two **608 bearings (8×22×7)**, one per clevis
  ear: Ø22.2 pockets from the outer faces, Ø16 retention shoulder inboard.
- **Torque** flows: gearbox output → **8 mm hardened shaft** → driven link's
  hub (Ø8.4 bore, Ø30 bolt-circle for a shaft-clamp disc). Gearboxes transmit
  torque only, per the project architecture.
- **Axial retention** by **8 mm shaft collars**: one exposed outboard of the
  +Y ear, one inside the gearbox drum bearing against the 608 inner race.
  The yaw column is retained by a collar in the Ø20 recess atop its hub.
- **Reserved cycloidal envelope**: Ø66 × 24 mm interior (Ø68 with slip fit)
  in a drum on the −Y ear; the drum's 5 mm back wall carries a **NEMA 17**
  mount (Ø22.5 pilot, 31 mm bolt square), motor hanging outboard. Gearbox
  design itself is out of scope — this is the space it must fit.
- The Ø66 envelope is the packaging limit: the drum (Ø76 with walls) swings
  2 mm above the 50 mm base pan through full ±135° yaw. A larger gearbox
  requires raising the shoulder axis (geometry is frozen).

## Gripper interface (forearm tip)

The 12 mm end plate carries a **T8 lead-screw nut** pattern on the TCP
centerline: Ø10.5 through-bore, Ø22.5 × 3.6 flange recess (nut inserted from
the inner face), 4 × Ø3.5 on a Ø16 bolt circle. A future lead-screw gripper
bolts to the plate; the grasp point sits beyond the TCP plane, matching
`gripper_extent_mm` in the twin's collision model.

## Purchased hardware (BOM)

| Item | Qty | Where |
|---|---|---|
| 608 bearing (8×22×7) | 4 | shoulder + elbow clevis ears |
| 8 mm hardened steel shaft, ~85 mm | 2 | shoulder + elbow joint axles |
| 8 mm shaft collar | 5 | 2 per pitch joint + 1 yaw retention |
| T8 lead screw + flanged nut | 1 | gripper actuation (gripper TBD) |
| NEMA 17 (42.3 sq, 48 long) | 3 | one per joint, per config |
| M3 / M4 fasteners + heat-set inserts | — | flanges, mounts, table bolts |

## Assumptions and limitations

- Direct-modeling generator: no Fusion timeline; parameters live in the
  script (geometry read from `robot.yaml` at run time).
- No fillets/chamfers yet — add stress relief at the beam-to-ear transitions
  before printing structural versions.
- The base drive bay reserves volume only; the base motor + cycloidal
  arrangement (coaxial vs belt primary) is decided with the gearbox design.
- Elbow fold clearance: the bridge block at the elbow clevis is z-trimmed so
  the forearm beam clears it out to ±154° (joint limit ±150°). Extreme
  down-folds near the base are prevented at runtime by the twin's collision
  checks, not by mechanical stops.
- Print settings assumed by the mass model: PETG, ~30 % infill.
