# Gripper — parallel-jaw, SG-90 rack-and-pinion (v2, in design)

Two parallel jaws driven by a single SG-90 micro servo through a printed
**lantern pinion** meshing two **hobbed racks** — one integrated into each
jaw slider, above and below the pinion, so one rotation input produces
symmetric open/close. Mounts on the forearm-tip plate. Replaces the v1
NEMA 8 + lead-screw design (kept in git history): the servo drive drops the
part count, the bench ops, the homing switch, and about two thirds of the
mass. Same generator pattern as the linkages and gearboxes:

```
engineering/gripper/
  gripper_geometry.py     the parametric layer (pure Python, no Fusion):
                          every dimension, derived drive-train stations,
                          rack hobbing math, 70 analytic self-checks
  gripper.py              Fusion wrapper: replays the geometry and exports
  gripper_parameters.yaml design parameters (opening, pin gearing, walls)
  *.step / *.stl          per-part exports            (generated)
```

Verify and run:

```
python engineering/gripper/gripper_geometry.py   # clearance report + numbers
```

Then, in Fusion: UTILITIES → ADD-INS → Scripts and Add-Ins → "+" → select
`engineering/gripper/` → Run. It re-runs the self-checks (refusing to build
if any fail), creates a Direct-Modeling design with the printed parts plus a
reference solid for the servo, and writes `f3d/rt-gripper.f3d` plus one STEP
and one STL per printed part into this folder.

## Headline numbers (defaults; all from the yaml)

| Quantity | Value |
|---|---|
| Jaw opening | 0–36 mm (pad face to pad face) |
| Servo swing | ~115° of the SG-90's ~160° usable travel |
| Grip force | ~6.5 N per jaw working (~8.8 N at stall) |
| Self-locking | **no** — keep the servo powered while gripping |
| Reach beyond TCP | 71 mm to the fingertips (v1: 138 mm) |
| Mass estimate | ~56 g (≈39 g printed + ≈18 g hardware; v1: ~165 g) |
| Homing | none needed — the servo is its own absolute reference |

**Twin updates required before this design is accepted** (simulation first —
do not print until the torque budget passes):
`collision.gripper_extent_mm: 140 → 75` and `masses.gripper_g: 170 → 60`
in `config/robot.yaml`, then re-run `python python/validate_cycloid.py` and
check the shoulder margins. The generator prints these numbers on every run.

## Architecture: lantern pinion, two mirrored racks

The gripper is self-contained along the tool axis (x = 0 at the TCP plane,
+X outward, jaws travel in Y, Z up):

```
x:  0    5  8      27.2  30.6 31.6  34.6    41.6  43   52  55       71
    |back|   SG-90 case  |plate|flange|  pins   |  |head|fp | fingers →
    |plate|  (nose +X)   |3.4  | Ø22.5| Ø4.5 x8 |  |tunnel|  | + pads
```

- **Back plate** (part of `rt_grip_body`) bolts to the forearm tip with
  4 × M3 × 8 on the Ø16 BCD, heads sunk in counterbores. A Ø10.3 **pilot
  boss** registers in the plate's Ø10.5 bore; the servo lead exits through
  its Ø7 center hole to the plate's bottom-edge wire notch. **The printed
  forearm is untouched — the v1 mounting contract is byte-identical.**
- **`rt_grip_bulkhead`** — the servo cartridge: the SG-90 is bench-screwed
  nose-first to this plate (its two tab self-tappers into printed Ø1.8
  pilot holes; the nose passes a 13.0 × 23.4 window). The loaded plate
  slides down vertical grooves in the housing walls and the lid captures
  its top edge — the whole drive drops in and out with **zero fasteners
  into the housing**. Grip reaction torque runs tab → plate → wall grooves.
- **`rt_grip_pinion`** — a lantern gear, printed flange-down with no
  supports: Ø22.5 flange + Ø11.4 core press onto the servo's Ø4.8 21T
  spline (Ø4.5 bore broaches on assembly), retained by the servo's own
  horn screw down the core's center bore (driver comes straight down the
  tool axis through the face-plate slot before the jaws go in). Eight
  Ø4.5 × 7 pins on the Ø18 pitch circle are the teeth.
- **Racks**: each jaw slider carries a toothed bar through the housing's
  rack window — `rt_grip_jaw_p` above the pinion, `rt_grip_jaw_n` below.
  Tooth spaces are **hobbed**: the pin-radius cutter (+0.25 mm print
  clearance) is subtracted along the cycloid the pin centers trace in the
  rack's frame — the same simulated-hobbing construction as the cycloidal
  gearbox discs. Pin gearing is exact conjugate action with no undercut;
  a numeric sweep in the self-check suite confirms at least one pin bears
  in each drive direction at every angle of the stroke. Tooth separation
  loads are backed by rails: the floor under the lower rack, a rib hanging
  from the lid over the upper one.
- **Jaws**: transverse carriers in the head tunnel (v1 pattern), necks out
  through the face-plate slot, fingers with 1 mm-keyed **TPU pad** pockets
  (`rt_grip_pad`, print 2 in TPU, faces 2 mm proud). Each rack bar reaches
  its carrier through an outer arm past the pin cage and a riser through
  its own passage in the jaw head (top passage for jaw_p, bottom for
  jaw_n).
- **Kinematics**: 9 mm pitch radius × ~2 rad of servo swing = 18 mm of
  stroke per jaw = 36 mm opening. Rack backlash from the 0.25 mm hob
  clearance is ~0.5 mm at the pads — irrelevant for gripping (the servo
  pushes through it), only visible in free positioning.

## Printed parts

| Part | Role | Notes |
|---|---|---|
| `rt_grip_body` | housing: back plate, servo bay, grooves, rack window, jaw head | print upright, back plate down |
| `rt_grip_lid` | top plate + upper-rack rail rib, 6 × M3 × 8 into inserts | print top face down |
| `rt_grip_bulkhead` | servo cartridge plate | |
| `rt_grip_pinion` | lantern pinion | flange down, pins vertical, no supports |
| `rt_grip_jaw_p` / `rt_grip_jaw_n` | jaw sliders with integrated racks | supports under the fingers |
| `rt_grip_pad` × 2 | finger pads | **TPU** |

## Hardware

| Item | Qty | Note |
|---|---|---|
| SG-90 9 g micro servo | 1 | clones fine; pockets carry ±0.3 mm envelope slack |
| M3 × 8 | 10 | 4 mount (into the forearm plate inserts), 6 lid |
| M3 × 5.7 heat-set inserts | 6 | lid pockets in the housing |
| servo tab self-tappers | 2 | supplied with the servo, into printed pilots |
| servo horn screw (~M2 × 8) | 1 | supplied with the servo, retains the pinion |

No lead screw, no flanged nut, no 623 bearings, no microswitch, no NEMA 8 —
and no bench filing/drilling/tapping ops at all.

## Assembly order

1. Bolt the housing to the forearm plate (4 × M3 × 8, ball-end or stubby
   driver — do this FIRST, the heads sit under the servo bay).
2. Bench: screw the SG-90 to `rt_grip_bulkhead` (2 self-tappers), command
   the servo to its jaw-OPEN endpoint (firmware or servo tester), press
   `rt_grip_pinion` onto the spline and drive the horn screw home down the
   core bore.
3. Slide the cartridge down the wall grooves; route the lead down the bay
   and out the pilot-boss hole (through the forearm plate's wire notch).
4. Slide the jaws in from their sides at the OPEN width — the racks mesh
   the pin cage as they enter; fingertip faces should sit 36 mm apart.
   Grease the pins and rack pockets (PTFE or silicone).
5. Lid on (its rib captures the upper rack), 6 × M3 × 8. Press the TPU
   pads into the finger pockets.

Every step is reversible without touching the others: the drive cartridge
lifts straight out once the lid and jaws are off.

## Firmware note

The gripper axis is a **hobby-servo PWM channel** (50 Hz, 1–2 ms), not a
stepper: no driver, no limit switch, no homing move. The v1 plan (step/dir on
13/14, limit on 27) is obsolete.

**It does not run on the ESP32.** An Arduino Uno drives the servo from its own
bench supply (5.0 V, 1 A limit) and the ESP32 relays `GRIP` to it over one wire
on GPIO 13. That closes the "still open" item this note used to carry — the
~650 mA stall transient now lands on a rail of its own instead of the one
generating step pulses — and it gives the servo proper 5 V logic besides.

It is implemented: `gripper:` in `config/firmware.yaml` (read by both boards),
`rt::GripperAxis` (`src/hardware/gripper.cpp`, the shared opening → pulse-width
model, which runs on the Arduino too), `fw::GripperLink` (the relay) and the
`GRIP <mm>` verb. `arduino/gripper_node/` is the Arduino side, and it also takes
a manual knob so the mechanism can be calibrated with no computer attached. The
firmware knows the mechanism only as two calibration pulses and the opening
between them, so the endpoints below must be measured at bring-up — see
docs/wiring-and-bringup.md "Gripper calibration".

## Assumptions and limitations

- **Not self-locking**: the SG-90's plastic gearing holds light grips
  unpowered by friction only. Keep the servo powered while carrying, and
  treat grip force as ~6.5 N per jaw sustained.
- Servo envelope constants assume a standard SG-90/clone (22.8 × 12.4 mm
  case, 27.6 mm hole pitch); measure an odd clone before printing.
- Direct-modeling generator, no fillets/chamfers yet; printed sliding fits
  use 0.35 mm side clearance — tune on the printer before trusting them.
- The swept interference audit lives in this module's own checks (plus the
  numeric mesh-contact sweep); folding the gripper's prismatic DOF into
  `python/audit_linkages.py` is future work, as is the jaw state in the
  twin's collision model (today it is a lumped extent + mass).
- Servo position ↔ jaw opening is linear (18 mm per 2 rad); calibrate the
  two PWM endpoints once at bring-up.
