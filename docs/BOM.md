# Bill of Materials

Purchased hardware for the full build: base + two pitch joints + the two
cycloidal gearboxes (one 15:1, one 20:1 — put the 20:1 at the shoulder) +
electronics. Sources of truth: `engineering/README.md` (linkage),
`engineering/gearboxes/README.md` (gearboxes, v3),
`config/firmware.yaml` / `docs/firmware.md` (electronics). Quantities are
for one robot with no spares unless noted.

## Motors and drivers

| Item | Qty | Notes |
|---|---|---|
| NEMA 17 stepper, 17HS4401 (5 mm D-shaft, ~40 mm body) | 3 | base direct drive, shoulder, elbow. The CAD assumes the measured 40 mm body and 23.5 mm usable shaft |
| TMC2209 driver module | 3 | step/dir mode, shared EN; UART optional (`tmc_uart.enabled: false`) |
| Electrolytic capacitor ≥100 µF, ≥35 V | 3 | one across VM per driver |

**A4988 substitution:** needs one firmware tweak — the step engine's STEP
pulses are tens of ns (fine for the TMC2209's 100 ns minimum, below the
A4988's 1 µs), so `step_engine.cpp` must stretch them first. Otherwise
the interface matches: the firmware drives step/dir/enable only, EN is
active-low on both, and `robot.yaml motor.microstepping: 16` is exactly
the A4988's maximum
(strap MS1=MS2=MS3 high). Set the current-limit pot (VREF) per your
carrier's formula; without a fan hold it to ~1 A/phase, which is ~60 % of
the 17HS4401's rated current — then derate
`robot.yaml motor.holding_torque_nm` (0.45 → ~0.28) so the twin and the
firmware torque governor plan against the torque you actually have.
DRV8825 is the same story with more current headroom (1/16 also available).

## Bearings, shafts, pins

| Item | Qty | Notes |
|---|---|---|
| 608-2RS bearing (8×22×7) | 4 | two per driven arm hub (dead axle: outer races in the hub, inner races on the static shaft) |
| 6805-2RS bearing (25×37×7) | 1 | yaw support in the base pan |
| 6802-2RS bearing (15×24×5) | 4 | two per gearbox, on the eccentric cam |
| 8 mm hardened steel shaft, **cut to 68 mm** | 2 | see "Shaft preparation" below |
| 8 mm set-screw shaft collar (~Ø18×9) | 2 | outboard axial keeper, one per pitch joint |
| Ø6 × 25 mm steel dowel pin | 49 (buy ~55) | 20:1 gearbox: 21 ring + 6 output; 15:1: 16 ring + 6 output |

### Shaft preparation (per shaft, ×2)

- **Cut to 68 mm. That is all** — the shaft is a *static, torque-free
  axle* since the dead-axle rev: the arm hub rides it on the two 608s and
  torque enters the arm through the gearbox output's drive sleeve, so
  there is no flat to grind and nothing clamps onto the rod.
- Length derivation (functional 67.5): ear-outer-face span 51.5
  (2 × EAR_OUT) + 9 mm collar outboard of the +Y ear + 7 mm tip reach
  into the gearbox output's Ø8.4 journal; the spare 0.5 mm disappears
  into the journal's 1 mm depth margin. If the linkage or casing changes,
  re-read `shaft_len` from `linkage_geometry.py`'s dry-run output.

## Fasteners

M3 socket head throughout. An M3 assortment box (6–40 mm) plus the specific
×40 count covers everything with spares.

| Item | Qty | Where |
|---|---|---|
| M3 × 5.7 heat-set insert (for Ø4.6 pockets) | 26 (+ spares) | coupling top ×4, ear discs 2×6, forearm plate ×4, gripper lid ×6 |
| M3 × 40 | 12 | gearbox through-bolts, 6 per gearbox |
| M3 × 8 | 16 | gearbox cup → motor (2×4), base pan → motor (4), optional hub → sleeve axial locks (2, self-tap into the flat's Ø2.5 pilot), spares |
| M3 × 10 | 4 | column plate → coupling |
| gripper screws (M3 × 8 ×10) | — | per the table in `engineering/gripper/README.md` |
| M3 × 20 + nut | 1 | base shaft-coupling cross-bolt |
| M3, length to suit the bench | 4 | pan table mount (Ø120 BCD) |

## Electronics

| Item | Qty | Notes |
|---|---|---|
| ESP32 DevKit (WROOM-32) | 1 | pins per `config/firmware.yaml` |
| Limit switch (lever microswitch) | 3 | one per joint, active-low to GND, internal pull-ups |
| Motor PSU, 12–24 V | 1 | ≥5 A at 12 V for three motors; 24 V preferred for speed headroom |
| 1 kΩ resistor | 1 | only if enabling the TMC2209 UART (TX → PDN_UART) |
| Hookup wire, dupont/JST, USB cable | — | UART0 console at 115200 |

## Gripper (designed — see `engineering/gripper/`)

Parallel-jaw gripper, designed and audited in
[engineering/gripper/README.md](../engineering/gripper/README.md). The v2
design replaced the NEMA 8 + lead-screw drive: a single **SG-90 micro
servo** turns a printed **lantern pinion** (eight Ø4.5 pins on a Ø18
pitch circle) meshing two **hobbed racks** integrated into the jaw
sliders — jaw opening 0–36 mm over ~115° of servo swing, ~6.5 N per jaw
working, ~56 g, fingertips 71 mm past the TCP plane. Not self-locking:
the servo stays powered while gripping.

The forearm-tip plate provides the mount: outer face = TCP plane, Ø10.5
bore (registers the gripper's pilot boss, passes wiring via the bottom
notch), **4 × M3 heat-set inserts on a Ø16 BCD opening on the OUTER
face** (this rev relocated them — the old inner-face pockets and nut
recess sat unreachable under the beam). The twin's matching yaml:
`masses.gripper_g: 60` and `collision.gripper_extent_mm: 75`; if the
gripper changes, update those and re-check the torque budget
(`python python/validate_cycloid.py` reports the shoulder margins).

Gripper shopping list (details in its README):

| Item | Qty | Notes |
|---|---|---|
| SG-90 9 g micro servo | 1 | drives the lantern pinion directly; clones fine |
| TPU filament (pads) | — | printed jaw faces |

No lead screw, nut, thrust bearings, homing switch, or extra stepper
driver — the servo replaces all of it. Firmware note: the gripper axis is
one 50 Hz PWM channel (free GPIO 13 suggested) plus a 5 V rail good for
~650 mA stall transients — a `firmware.yaml` + firmware extension, not
just wiring; the old step/dir 13/14 + limit 27 reservation is obsolete.
