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
   - `step/rt_<part>.step` — one STEP per part
   - `stl/rt_<part>.stl` — one STL per part

A summary dialog reports exports, build-volume checks, and any config
fallbacks.

## The four rigid members

| Part | Role | Bounding (mm) |
|---|---|---|
| `rt_base_pan` | Stationary base: direct-drive NEMA 17 mount (motor stands through the floor, face on a 2 mm web) + the yaw bearing boss on top | Ø140 × 65 |
| `rt_shaft_coupling` | Printed split-clamp coupling: 5 mm D-bore clamp → Ø25 bearing journal → Ø30 bolt flange. A standard part with its own STEP | Ø30 × 19.5 |
| `rt_yaw_column` | θ₁ output: hub bolts down onto the coupling, clevis ears up to the shoulder axis (z = 90), shoulder gearbox drum | 76 × 90 × 76 |
| `rt_upper_arm` | Shoulder hub → elbow clevis (axes exactly 120 mm apart), elbow gearbox drum + elbow NEMA 17 mount | 178 × 82 × 76 — print diagonally on the A1 Mini |
| `rt_forearm` | Elbow hub → gripper interface plate; outer plate face is the TCP plane | 140 × 40 × 40 |

## Base: direct drive with a supported yaw axis

The base joint is driven directly by its NEMA 17 — no gearbox. The yaw axis
fights no gravity torque, and the twin's torque governor already paces yaw
moves against the motor's unassisted torque curve (`config/robot.yaml` sets
`gearboxes.base: direct, 1`).

The arm's weight and overturning moment do **not** ride on the bare motor
shaft. Stack, bottom to top (z planes 48 / 48.5–58 / 58–65 / 65–68 / 68+):

1. Motor body spans exactly table (z = 0) to face (z = 48) through the pan's
   floor opening; its face bolts to the 2 mm web (use **low-profile M3
   heads**: they pass ~1 mm inside the shoulder drum's sweep circle).
2. `rt_shaft_coupling` clamps the 5 mm D-shaft: printed D-bore for positive
   torque drive plus a split clamp squeezed by two M3 cross-bolts with
   captured nuts.
3. A **6805-2RS (25×37×7)** rides the coupling's Ø25 journal, seated in a
   boss on the pan: four legs pass between the motor screws (driver-access
   notches over each screw), and bearing loads go legs → web → the motor's
   steel face **in compression**, straight down the body to the table.
4. The coupling's Ø30 flange presses the inner race and carries the column:
   4 × M3 down through counterbored holes in the hub into heat-set inserts
   in the flange. Weight goes flange → bearing → boss; the bearing pairs
   with the motor's front bearing (~17 mm below) to resist moment as a
   spaced couple.

## Pitch joint architecture (shoulder and elbow identical)

- **Structural loads** ride on two **608 bearings (8×22×7)**, one per clevis
  ear: Ø22.2 pockets from the outer faces, Ø16 retention shoulder inboard.
- **Torque** flows: gearbox output → **8 mm hardened shaft** → driven link's
  hub (Ø8.4 bore, Ø30 bolt-circle with insert-ready Ø4.6 holes for a
  shaft-clamp disc). Gearboxes transmit torque only, per the project
  architecture.
- **Axial retention** by **8 mm shaft collars**: one exposed outboard of the
  +Y ear, one inside the gearbox drum bearing against the 608 inner race.
- **Reserved cycloidal envelope**: Ø66 × 24 mm interior (Ø68 with slip fit)
  in a drum on the −Y ear; the drum's 5 mm back wall carries a **NEMA 17**
  mount (Ø22.5 pilot, 31 mm bolt square), motor hanging outboard. Gearbox
  design itself is out of scope — this is the space it must fit.
- The Ø66 envelope is the packaging limit: the drum (Ø76 with walls) swings
  2 mm above the 50 mm base pan through full ±135° yaw. A larger gearbox
  requires raising the shoulder axis (geometry is frozen).

## Fasteners

M3 everywhere. Every joint where a part bolts **to a printed member** gets
Ø4.6 × 6.5 pockets for **M3 × 5.7 heat-set inserts** (coupling flange, hub
flange circles, T8 nut pattern). Screws that thread into a motor's own
tapped holes (NEMA mounts) and the table-mount holes are plain Ø3.4
clearance; the coupling's clamp bolts run against captured nuts.

## Gripper interface (forearm tip)

The 12 mm end plate carries a **T8 lead-screw nut** pattern on the TCP
centerline: Ø10.5 through-bore, Ø22.5 × 3.6 flange recess (nut inserted from
the inner face), 4 × M3 into blind heat-set inserts behind the recess floor.
A future lead-screw gripper bolts to the plate; the grasp point sits beyond
the TCP plane, matching `gripper_extent_mm` in the twin's collision model.

## Purchased hardware (BOM)

| Item | Qty | Where |
|---|---|---|
| 608 bearing (8×22×7) | 4 | shoulder + elbow clevis ears |
| 6805-2RS bearing (25×37×7) | 1 | yaw support, on the coupling journal |
| 8 mm hardened steel shaft, ~85 mm | 2 | shoulder + elbow joint axles |
| 8 mm shaft collar | 4 | 2 per pitch joint |
| T8 lead screw + flanged nut | 1 | gripper actuation (gripper TBD) |
| NEMA 17 (42.3 sq, 48 long) | 3 | one per joint, per config |
| M3 × 5.7 heat-set inserts | 20 | coupling flange (4), hub flanges (2×6), T8 plate (4) |
| M3 nuts | 2 | captured in the coupling's clamp pockets |
| M3 screws (low-profile head for the base motor face) | — | all connections |

## Assumptions and limitations

- Direct-modeling generator: no Fusion timeline; parameters live in the
  script (geometry read from `robot.yaml` at run time).
- No fillets/chamfers yet — add stress relief at the beam-to-ear transitions
  before printing structural versions.
- The base motor's mounting web is 2 mm (constrained by motor length 48 +
  web = pan height 50); the stiffening ring provides the rigidity, and the
  bearing boss loads the web only in compression against the motor face.
  Assembly order: motor in from below → face screws through the boss notches
  → pan to table → coupling clamped to the shaft → 6805 onto the journal and
  into its pocket → column bolted down onto the coupling flange.
- Elbow fold clearance: the bridge block at the elbow clevis is z-trimmed so
  the forearm beam clears it out to ±154° (joint limit ±150°). Extreme
  down-folds near the base are prevented at runtime by the twin's collision
  checks, not by mechanical stops.
- Print settings assumed by the mass model: PETG, ~30 % infill.
