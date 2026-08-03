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
  gripper/                   parallel-jaw gripper: parametric layer + Fusion
                             script + design doc + exports (see its README)
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
purchased hardware (motor bodies, screw heads, clamp bolt, the spinning
drive sleeves) and the reserved gearbox envelopes. Adjacent-link clearance is guaranteed mechanically;
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
| `rt_shaft_coupling` | Printed coupling (base only): Ø25 bearing journal → Ø28 D-bore pinch body → bolt face. A standard part with its own STEP | Ø28 × 18 |
| `rt_yaw_column` | θ₁ output: low plate bolted to the coupling, clevis ears up to the shoulder axis (z = 90), gearbox mounting face | 76 × 68 × 76 |
| `rt_upper_arm` | Twin-beam clevis: shoulder hub → central beam → web → side beams → elbow clevis (axes exactly 120 mm apart) | 173 × 52 × 76 |
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
   shoulder hub swings clear through the full 0–180° shoulder range — this
   ceiling is what sizes the Ø30 hub (no hardware protrudes from it since
   the pinch clamp is gone).

The shoulder's drive-side ear disc reaches down to z = 52 and orbits the
stationary boss as the base yaws; the column carries two revolved relief
cuts (Ø47 to z 60.4, Ø52 to z 54.5) that keep it clear of the boss and the
motor screw heads while sparing the gearbox flange seat.

## Pitch joint architecture (shoulder and elbow identical)

- **Live shaft** (this rev, v5): the **8 mm shaft IS the torque path** and
  spins with the driven link. It stays a smooth rod — no flats, same
  68 mm cut as the dead-axle rev, so existing shafts are reused — and is
  grabbed at both ends by press/slip bores closed with **split pinches**
  (the base coupling's proven slit + M3-cross-bolt + captured-nut
  pattern).
- **Torque** flows: gearbox output → its **shaft-clamp extension** (Ø24.6
  pinch past the lid face) → shaft → the **arm hub's own pinch** (Ø30
  boss, slit on the −X side, two M3 cross-bolts) → arm.
- **Bearings live in the ears** now, not the hub: the +Y ear houses a
  **608** (outer race in a Ø22.2 pocket behind a Ø16 lip, inner race on
  the shaft); the drive ear houses a **6805** (Ø37.3 pocket from the
  gearbox face) riding the output extension's Ø25.05 seat. Structural
  loads still go through rolling bearings only.
- **Asymmetric clevis**: the drive ear sits at y 23.5–35.5 (9.75 mm
  further outboard than the +Y ear) so the output's pinch section has
  open tool access in the gap between ear and hub. The gearbox mounting
  face, Ø66 insert BCD, and M3 × 40 through-bolts are unchanged.
- **Axial retention**: both pinches fix the shaft; the +Y collar remains
  as a keeper snugged ~0.5 mm off the ear face (it must not rub the ear).
- **Torque-capacity caveat**: a pinch on smooth hardened rod is a
  friction joint. Degrease the shaft, torque the pinch bolts firmly, and
  treat slip under reversing loads as a real possibility — a drop of
  retaining compound (Loctite 638) in each bore is cheap insurance. This
  is the same interface the v2 design retired; it returns by explicit
  decision, traded for a serviceable joint that matches the purchased
  shaft stock.
- **Gearbox mounting face** (no enclosed drum — v1's drum could never admit
  the cartridge): the drive-side ear is a Ø76 disc whose outboard face
  carries **6 × M3 heat-set inserts on a Ø66 bolt circle**. The cycloidal
  cartridge (Ø76 × 36, see [gearboxes/README.md](gearboxes/README.md))
  bolts on with M3 × 40 through-screws from its back and carries its NEMA 17
  on its own back plate. Cartridge + screw heads + motor are audited as
  keep-out envelopes.
- The upper arm is a **twin-beam clevis**: the central beam ends at a
  derived hand-off station (x ≈ 62 for 120 mm links), a full-width web
  bridges to two side beams flanking the forearm's swing plane, and the
  forearm folds *between* them — this is what makes the full ±150° elbow
  range mechanically reachable (a single mid-plane beam cannot coexist with
  a forearm folded 150°).

## Fasteners

M3 everywhere. Every joint where a part bolts **to a printed member** gets
Ø4.6 × 6.5 pockets for **M3 × 5.7 heat-set inserts** (coupling top face ×4,
gearbox faces ×6 each, gripper plate ×4). Screws into a motor's own tapped
holes (NEMA mounts) and the table-mount holes are plain Ø3.4 clearance; the
base coupling's pinch bolt runs against a captured nut (the only pinch
left — the hub pinches died with the dead-axle rev).

## Gripper interface (forearm tip)

The 12 mm end plate's outer face is the TCP plane and the gripper's
mounting face (see [gripper/README.md](gripper/README.md)): a Ø10.5
through-bore on the TCP centerline registers the gripper's pilot boss and
passes its wiring (a bottom-edge notch feeds the harness under the beam),
and **4 × M3 heat-set inserts on a Ø16 BCD open on the OUTER face** — the
gripper's back plate bolts straight on with M3 × 8. (The earlier
inner-face insert pockets and T8-nut recess sat unreachable under the
forearm beam; the v2 gripper is SG-90 rack-and-pinion driven and uses no
screw hardware at all.) The fingertips reach `gripper_extent_mm` beyond
the TCP plane in the twin's collision model.

## Purchased hardware (BOM)

| Item | Qty | Where |
|---|---|---|
| 608 bearing (8×22×7) | 2 | one per +Y ear, inner race on the live shaft |
| 6805-2RS bearing (25×37×7) | 3 | yaw support + one per drive ear (on the gearbox output's seat) |
| 8 mm hardened steel shaft, cut to 68 mm | 2 | LIVE torque shafts — still smooth, no flats; both ends pinch-clamped (see docs/BOM.md) |
| 8 mm shaft collar | 2 | one per pitch joint, +Y keeper (snug 0.5 mm off the ear) |
| M3 × 20 + nut | 4 | arm-hub pinch bolts, two per hub |
| M3 × 16 + nut | 2 | gearbox-output pinch bolts, one per output |
| SG-90 micro servo | 1 | gripper actuation (lantern pinion + hobbed racks) |
| NEMA 17 17HS4401 (42.3 sq, 40 long) | 3 | one per joint, per config |
| M3 × 5.7 heat-set inserts | 20 | coupling (4), gearbox faces (2×6), gripper plate (4) — the gripper body adds its own |
| M3 nuts | 1 | coupling pinch cross-bolt |
| M3 screws | — | all connections |

## Assembly order

Motor in from below → face screws through the boss notches → pan to table →
6805 into the boss pocket → coupling (journal down) through the bearing onto
the D-shaft → tighten its pinch bolt → column plate bolted down into the
coupling's inserts → 608 pressed into the +Y ear, 6805 pressed into the
drive ear's outboard pocket → arm between the ears, shaft in from the +Y
side through the ear 608 and the hub bore (hub pinch loose) until ~10 mm
protrudes past the hub on the −Y side → gearbox cartridge (bench-assembled,
motor on its back) offered up: its extension slides through the drive-ear
6805 and its bore swallows the shaft end; 6 × M3 × 40 from the gearbox back
into the ear-disc inserts → tighten the OUTPUT pinch bolt (tool access in
the ear-to-hub gap), then the two HUB pinch bolts, then the +Y collar
0.5 mm off the ear face. Every fastener is reachable at its step.

## Assumptions and limitations

- Direct-modeling generator: no Fusion timeline; `linkage_geometry.py` is the
  parametric layer (geometry read from `robot.yaml` at run time) and refuses
  to build if any clearance self-check fails.
- No fillets/chamfers yet — add stress relief at the beam-to-ear transitions
  before printing structural versions.
- The gearbox interface (Ø76 face, Ø66 bolt circle, 6805 pocket, extension
  diameters, pinch length) is a mirrored contract with
  `cycloidal_gearbox.py` — the `GBX_*` constants here and the `ext_*` yaml
  keys there must move together.
- The pinch-on-smooth-shaft couplings are friction joints; their torque
  limit is unproven and lower than the retired sleeve-flat form lock (see
  the gearboxes README's open issues and the caveat above).
- Print settings assumed by the mass model: PETG, ~30 % infill.
