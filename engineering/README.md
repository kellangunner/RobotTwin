# Engineering — CAD generation

CAD is downstream of simulation: link geometry comes from
[config/robot.yaml](../config/robot.yaml) (90 / 120 / 120 mm, frozen per
[docs/linkage-geometry.md](../docs/linkage-geometry.md)); this folder turns it
into printable rigid members.

## Layout

```
engineering/
  fusion/robot_linkages/     linkage generator
    linkage_geometry.py        the parametric layer (pure Python, no Fusion):
                               every dimension, derived joint stacks, and ~50
                               analytic clearance self-checks
    robot_linkages.py          Fusion wrapper: replays the geometry and exports
  fusion/cycloidal_gearbox/  Fusion script that generates the 15:1 / 20:1 gearboxes
  gearboxes/                 gearbox design doc + per-ratio exports (see its README)
  f3d/                       Fusion archive exports   (generated)
  step/                      STEP exports             (generated)
  stl/                       per-part STL exports     (generated)
```

## Verifying and running the generator

Before (or without) opening Fusion:

```
python engineering/fusion/robot_linkages/linkage_geometry.py   # clearance report
python python/audit_linkages.py                                # swept interference audit
```

The audit poses the assembly through full yaw, shoulder 0–180° and elbow
±150° and grid-samples for solid overlap between the printed members,
purchased hardware (motor bodies, screw heads, clamp bolt) and the reserved
gearbox envelopes. Adjacent-link clearance is guaranteed mechanically;
whole-arm world collisions (e.g. folded forearm vs the base at extreme
combined poses) remain the twin's runtime collision layer, per
`collision:` in robot.yaml.

Then, in Fusion: UTILITIES → ADD-INS → Scripts and Add-Ins → Scripts → "+" →
select `engineering/fusion/robot_linkages/` → Run. It re-runs the self-checks
(refusing to build if any fail), creates a new Direct-Modeling design, and
writes `f3d/rt-arm-3dof_linkages.f3d` plus one STEP and one STL per part.

## The five printed members

| Part | Role | Bounding (mm) |
|---|---|---|
| `rt_base_pan` | Stationary base: hanging NEMA 17 mount + yaw bearing boss | Ø140 × 59 |
| `rt_shaft_coupling` | Printed coupling: Ø25 bearing journal → Ø28 D-bore pinch body → bolt face. A standard part with its own STEP | Ø28 × 19 |
| `rt_yaw_column` | θ₁ output: low plate bolted to the coupling, clevis ears up to the shoulder axis (z = 90), gearbox mounting face | 76 × 52 × 76 |
| `rt_upper_arm` | Twin-beam clevis: shoulder hub → central beam → web → side beams → elbow clevis (axes exactly 120 mm apart) | 171 × 52 × 76 |
| `rt_forearm` | Elbow hub → beam → gripper interface plate; outer plate face is the TCP plane | 135 × 40 × 40 |

## Base: direct drive with a supported yaw axis

The base joint is driven directly by its NEMA 17 — no gearbox (the yaw axis
fights no gravity torque; the twin's torque governor paces yaw moves).

The motor is a 17HS4401 (42.3 sq × **40** long, 5 mm D-shaft ~23.5 mm): it
**hangs from the pan's 4 mm top plate** (inserted through the floor opening,
body floating above the floor — any 38–42 mm body fits), face bolted to the
plate's underside at z = 46. The arm's weight and overturning moment do
**not** ride on the bare motor shaft. Stack, bottom to top:

1. **Ø25 journal** (z 50.5–58.5) at the *bottom* of `rt_shaft_coupling`; the
   **6805-2RS (25×37×7)** slides on from this end and seats in the pan boss
   (outer race on a ledge at z 51.5, driver-access notches over the four
   motor screws double as head room).
2. **Ø28 pinch body** (z 58.5–69.5): printed D-bore for positive torque
   drive plus a split pinch squeezed by one M3 cross-bolt with a captured
   nut — *above* the bearing, so it is tightened after the coupling is on
   the shaft. Its bottom shoulder presses the bearing's inner race; weight
   goes body → inner race → outer race → boss, and the 6805 pairs with the
   motor's front bearing to resist moment as a spaced couple.
3. The **column plate** (Ø68 × 5) bolts down with 4 × M3 through counterbored
   holes into heat-set inserts in the coupling's top face. Everything on the
   column side near the yaw axis stays below z = 74.5 so the upper arm's
   shoulder hub (+ its pinch hardware) swings clear through the full 0–180°
   shoulder range — this ceiling is what sizes the Ø26 hub.

The shoulder's drive-side ear disc reaches down to z = 52 and orbits the
stationary boss as the base yaws; the column carries two revolved relief
cuts (Ø47 to z 60.4, Ø52 to z 54.5) that keep it clear of the boss and the
motor screw heads while sparing the gearbox flange seat.

## Pitch joint architecture (shoulder and elbow identical)

- **Structural loads** ride on two **608 bearings (8×22×7)**, one per clevis
  ear: Ø22.2 pockets from the outer faces, Ø16 retention shoulder inboard.
- **Torque** flows: gearbox output → **8 mm hardened shaft** → driven hub,
  which grips the shaft with a printed **pinch clamp** (horizontal slit,
  vertical M3 + captured nut, all inside the hub's Ø26 swing envelope).
- **Axial retention**: the gearbox output's depth stop locates the shaft;
  a **shaft collar** outboard of the +Y ear is the keeper.
- **Gearbox mounting face** (no enclosed drum — v1's drum could never admit
  the cartridge): the drive-side ear is a Ø76 disc whose outboard face
  carries **6 × M3 heat-set inserts on a Ø66 bolt circle**. The cycloidal
  cartridge (Ø62 × 38.4, see [gearboxes/README.md](gearboxes/README.md))
  bolts on via a Ø76 × 4 front flange and carries its NEMA 17 on its own
  back plate. Cartridge + flange + motor are audited as keep-out envelopes.
- The upper arm is a **twin-beam clevis**: the central beam ends at a
  derived hand-off station (x ≈ 62 for 120 mm links), a full-width web
  bridges to two side beams flanking the forearm's swing plane, and the
  forearm folds *between* them — this is what makes the full ±150° elbow
  range mechanically reachable (a single mid-plane beam cannot coexist with
  a forearm folded 150°).

## Fasteners

M3 everywhere. Every joint where a part bolts **to a printed member** gets
Ø4.6 × 6.5 pockets for **M3 × 5.7 heat-set inserts** (coupling top face ×4,
gearbox faces ×6 each, T8 plate ×4). Screws into a motor's own tapped holes
(NEMA mounts) and the table-mount holes are plain Ø3.4 clearance; the
coupling and hub pinch bolts run against captured nuts.

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
| 8 mm hardened steel shaft, ~80 mm | 2 | shoulder + elbow joint axles |
| 8 mm shaft collar | 2 | one per pitch joint, +Y keeper |
| T8 lead screw + flanged nut | 1 | gripper actuation (gripper TBD) |
| NEMA 17 17HS4401 (42.3 sq, 40 long) | 3 | one per joint, per config |
| M3 × 5.7 heat-set inserts | 20 | coupling (4), gearbox faces (2×6), T8 plate (4) |
| M3 nuts | 3 | coupling pinch (1) + hub pinches (2) |
| M3 screws | — | all connections |

## Assembly order

Motor in from below → face screws through the boss notches → pan to table →
6805 into the boss pocket → coupling (journal down) through the bearing onto
the D-shaft → tighten its pinch bolt → column plate bolted down into the
coupling's inserts → 608s into the ear pockets → hub between the ears, shaft
through, hub pinch tightened, collar on → gearbox cartridge (bench-assembled,
motor on its back) over the shaft end, 6 × M3 into the ear-disc face. Every
fastener is reachable at its step.

## Assumptions and limitations

- Direct-modeling generator: no Fusion timeline; `linkage_geometry.py` is the
  parametric layer (geometry read from `robot.yaml` at run time) and refuses
  to build if any clearance self-check fails.
- No fillets/chamfers yet — add stress relief at the beam-to-ear transitions
  before printing structural versions.
- The gearbox side of the mounting interface (Ø76 front flange, Ø66 bolt
  circle, output depth stop for the 8 mm shaft) is a contract the cycloidal
  cartridge must adopt; its current revision predates this interface.
- The hub pinch on polished 8 mm rod is friction drive — knurl or flat the
  shaft ends, or pin them, if a gearbox trade study exceeds ~2 N·m at the hub.
- Print settings assumed by the mass model: PETG, ~30 % infill.
