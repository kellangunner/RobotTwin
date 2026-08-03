
# Wiring & Bring-Up Guide

How to wire the ESP32 controller to the three joint motors and get the arm
moving for the first time. Companion to [firmware.md](firmware.md) (what the
firmware does) and [BOM.md](BOM.md) (what to buy). The single source of truth
for every pin below is [`config/firmware.yaml`](../config/firmware.yaml) —
if this document and that file ever disagree, the YAML wins, because it is
what actually gets flashed.

## What you need

From the [BOM](BOM.md) electronics section:

- ESP32 DevKit (WROOM-32, the common 30/38-pin board)
- 3 × TMC2209 stepper driver modules (StepStick/BTT-style carriers)
- 3 × NEMA 17 steppers (17HS4401 or similar, 4-wire bipolar)
- 3 × electrolytic capacitors, ≥100 µF ≥35 V (one across VM per driver)
- Motor PSU: 12–24 V DC, ≥5 A at 12 V (24 V preferred for speed headroom)
- USB cable (powers the ESP32 and carries the serial console)
- Hookup wire; 1 × 1 kΩ resistor for the TMC2209 UART bus (TX → PDN_UART)
- 1 × SG-90 hobby servo (the gripper's actuator) on 5 V

A breadboard works for the smoke test, but solder or use crimped dupont/JST
connections for anything that moves — a step pin that loses contact mid-move
means lost steps the firmware cannot detect. For a bench rig, there is a
board-agnostic breadboard layout (net list + per-module rules, works on
any boards) in [breadboard-wiring.md](breadboard-wiring.md) (drop `irun`
so phase current stays ≤ 0.8 A on breadboards), and a hole-by-hole
step-by-step build for the dovetailed two-board bench in
[breadboard-step-by-step.md](breadboard-step-by-step.md).

## Power architecture

Three independent supplies with a **common ground**:

- **Logic**: the ESP32 is powered over USB. Each TMC2209's VIO pin takes
  3.3 V from the ESP32's 3V3 pin.
- **Motor power**: the 12–24 V PSU feeds each driver's VM/GND. Put one
  ≥100 µF electrolytic directly across VM/GND at each driver — TMC2209s die
  from inductive voltage spikes without it.
- **Gripper**: a bench supply at **5.0 V with a 1 A current limit** feeds the
  servo, and the Arduino driving it runs off host USB. Nothing here is derived
  from the motor supply — see "Gripper node".
- **Tie PSU ground to ESP32 GND**, and the servo supply's − to both. Without
  the shared reference the step/dir signals float, the drivers behave
  erratically, and the relay link to the Arduino cannot be read at all.

Only grounds are shared. No two of the three rails meet anywhere else.

Rules that prevent dead drivers:

- **Never connect or disconnect a motor while VM is powered.** This is the
  classic way to kill a TMC2209.
- Power-up order doesn't otherwise matter (the shared EN line boots
  disabled), but wire the caps before first applying VM.

## Pin map

All values from `config/firmware.yaml`. Step/dir/enable must stay below
GPIO32 (the step ISR writes the low GPIO bank's registers).

This arm has **no limit switches** — it has none designed in, and no hard
stops to mount them against — so nothing is wired to a switch input and the
datum is set manually with `SETHOME` (see the bring-up procedure). GPIO32/33/25
are consequently free.

> The base joint uses GPIO4/GPIO27 rather than the GPIO16/GPIO17 you'll see
> on many WROOM-32 pinout diagrams: some 30-pin DevKit revisions don't break
> those two out (the ELEGOO DevKit V1 TypeC does, labeled RX2/TX2 — the
> config keeps 4/27 so either works). GPIO4/27 are the free, non-strapping,
> sub-GPIO32 pins that satisfy the same constraints; GPIO14 is the remaining
> spare. If your board *does* expose 16/17 and you'd rather use them, change
> `joints.base.step_pin`/`dir_pin` in `firmware.yaml` and reflash.

| Signal                 | ESP32 GPIO | Goes to                                          |
| ---------------------- | ---------- | ------------------------------------------------ |
| Base STEP              | 4          | base driver STEP                                 |
| Base DIR               | 27         | base driver DIR                                  |
| Shoulder STEP          | 18         | shoulder driver STEP                             |
| Shoulder DIR           | 19         | shoulder driver DIR                              |
| Elbow STEP             | 21         | elbow driver STEP                                |
| Elbow DIR              | 22         | elbow driver DIR                                 |
| Driver ENABLE (shared) | 23         | EN on**all three** drivers (active-low)    |
| TMC UART TX            | 26         | 1 kΩ → shared PDN_UART line, all three drivers |
| Gripper relay TX       | 13         | 1 kΩ → Arduino Uno D2 (gripper node)             |

**The servo does not connect to the ESP32.** It hangs off an Arduino Uno with
its own supply, and GPIO 13 — which used to carry the servo's PWM directly —
now carries commands to that Uno instead. The reasons are the ones this document
used to list as warnings: a stalled SG-90 draws ~650 mA, which is exactly the
transient that browns out the board generating step pulses, and an SG-90 wants
5 V logic rather than the ESP32's 3.3 V. See "Gripper node" below.
GPIO32/33/25 are unused (no limit switches).

```
                 ESP32 DevKit (WROOM-32)
                ┌───────────────────────┐
   USB ────────►│ 5V/USB          3V3   ├──► VIO on all 3 drivers
                │                 GND   ├──► common ground (PSU − too)
                │ GPIO4  ─ STEP ┐       │
                │ GPIO27 ─ DIR  ├─ base driver
                │ GPIO18 ─ STEP ┐       │
                │ GPIO19 ─ DIR  ├─ shoulder driver
                │ GPIO21 ─ STEP ┐       │
                │ GPIO22 ─ DIR  ├─ elbow driver
                │ GPIO23 ─ EN ──┴─┴─┴─ EN on all three (active-low)
                │ GPIO26 ─ 1kΩ ─ PDN_UART bus (all three)
                │ GPIO13 ─ 1kΩ ─ Uno D2 (gripper node, one-way)
                └───────────────────────┘

     12–24 V PSU ──┬── VM/GND driver 1 (+100 µF cap at the driver)
                   ├── VM/GND driver 2 (+100 µF)
                   └── VM/GND driver 3 (+100 µF)
```

## Gripper node (Arduino Uno)

The SG-90 lives on its own board. The ESP32 sends it commands over a single
wire and never hears back.

| Uno pin | Connect to                                                       |
| ------- | ---------------------------------------------------------------- |
| D9      | SG-90 signal wire (orange)                                       |
| A0      | Potentiometer wiper; the pot's two ends go to Uno 5 V and GND    |
| D2      | ESP32 GPIO 13 through 1 kΩ — the relay link                       |
| D3      | Nothing. Reserved for a return path                               |
| GND     | Servo brown wire, servo supply −, **and** ESP32 GND               |
| USB     | Host PC — power, sketch uploads, and the node's own console       |

Servo power comes from a bench supply set to **5.0 V with a 1 A limit**, wired
straight to the servo's red and brown wires. Fit a **470–1000 µF electrolytic
plus a 0.1 µF ceramic across V+/GND at the servo connector**: a bench supply's
regulation loop responds in milliseconds and the servo's inrush is faster than
that, so the capacitor — not the supply — is what actually covers the
transient.

```
        Arduino Uno                      bench PSU 5.0 V / 1 A limit
       ┌─────────────┐                        │        │
 USB ──┤ D9  ────────┼── SG-90 signal (orange)│        │
       │ A0  ────────┼── knob wiper           │        │
       │ D2  ◄─ 1kΩ ─┼── ESP32 GPIO13         ├── red ─┴─ SG-90 V+
       │ D13 (LED)   │                        │       (+470–1000 µF here)
       │ GND ────────┼────────────────────────┴── brown ── SG-90 GND
       └─────────────┘         └── star ground: also ESP32 GND, PSU −
```

Three things that will bite you:

- **Never feed 5 V into the Uno's barrel jack.** That input runs through the
  onboard AMS1117 regulator, which needs ~6.5 V; 5 V there leaves the board's
  5 V rail near 3.9 V and erratic. A 5 V source goes into the USB connector.
- **Never jumper the servo supply onto the Uno's 5V pin.** The rails stay
  separate; only the grounds meet.
- **The relay must not land on D0 or D1.** Those are the Uno's hardware UART,
  wired to its USB chip — using them would fight the console and break sketch
  uploads. That is why D2 is a `SoftwareSerial` receive pin.

The 3.3 V the ESP32 drives into D2 is above the Uno's 3.0 V logic-high
threshold, so it reads correctly, but only by 0.3 V. If bring-up shows garbled
bytes, drop `relay_baud` to 9600 before reaching for a level shifter.

Build and flash it from `arduino/gripper_node/` — see the README there.

## Per-driver wiring (TMC2209 carrier)

Each of the three drivers gets:

| Driver pin             | Connect to                                        |
| ---------------------- | ------------------------------------------------- |
| VM / GND (motor side)  | PSU + / PSU − , with the 100 µF cap across them |
| VIO / GND (logic side) | ESP32 3V3 / GND                                   |
| STEP, DIR              | the joint's GPIOs from the pin map                |
| EN                     | GPIO23 (shared by all three)                      |
| A1, A2, B1, B2         | one motor coil per pair (see below)               |
| MS1, MS2               | UART address straps — per the table below        |
| PDN_UART               | shared bus from GPIO26 (through the 1 kΩ)        |

**Motor coils:** a 4-wire bipolar stepper has two coils. Find the pairs with
a multimeter (a coil pair shows a few ohms; across coils is open). One pair
goes to A1/A2, the other to B1/B2. Which pair is "A" and which wire is which
within a pair only affects rotation direction — fixed later in software with
`invert_dir`, so don't agonize over it.

**Current limit (set over UART, not the pot):** this build runs
`tmc_uart.enabled: true`, so at boot the firmware programs each driver's
run/hold current from `firmware.yaml` (`irun: 16`, `ihold: 8`, on a 0–31
scale) and the onboard VREF potentiometer is **ignored** — don't chase it
with a screwdriver. With the 110 mΩ sense resistors on typical carriers,
`irun: 16` lands near **0.9–1.0 A RMS**, right for an uncooled 17HS4401
(rated 1.7 A); sense values differ between brands, so if a motor runs
notably weak or hot, check your carrier's sense resistors before blaming
the config. Changing current = edit `irun`/`ihold` and reflash (the YAML
is embedded in the image). If you settle well below rated current, derate
`robot.yaml motor.holding_torque_nm` proportionally so the twin and the
firmware torque governor plan against the torque you actually have.

## UART address straps (MS1/MS2) and the PDN bus

With the UART enabled, MS1/MS2 are **address pins**, not microstep
selects — the firmware pushes the microstep resolution (1/16, from
`robot.yaml motor.microstepping`) over the wire at boot via CHOPCONF.
Strap each driver to match its `uart_address` in `firmware.yaml`, and
note every driver needs a **unique** address:

| Joint    | Address | MS1 | MS2 |
| -------- | ------- | --- | --- |
| base     | 0       | GND | GND |
| shoulder | 1       | VIO | GND |
| elbow    | 2       | GND | VIO |

Wire GPIO26 through the single 1 kΩ resistor, then bus it to all three
PDN_UART pins tied together. The link is write-only — nothing is read
back, and step/dir remains the only motion path. Two drivers strapped to
the same address will both accept that address's settings and neither
will fault, so a strap mistake shows up as behavior, not an error (see
troubleshooting).

Fallback for reference: if `tmc_uart.enabled` is ever set back to false,
the straps revert to hardware microstep-select (both MS1 and MS2 high =
1/16) and the VREF pot rules current again.

## Flashing the firmware

The firmware builds with PlatformIO (espressif32 + ESP-IDF). Install it if
you don't have it (`pip install platformio`, or the VS Code extension),
then:

```
cd firmware
pio run              # build
pio run -t upload    # flash over USB
pio device monitor   # serial console, 115200 baud
```

Both YAML configs are embedded into the image at build time, so **any change
to `config/firmware.yaml` or `config/robot.yaml` requires a rebuild and
reflash** — but never a code change.

> Note: the firmware builds cleanly but has not yet been run on real
> hardware. Expect to iterate on the datum reference pose and current
> settings during bring-up.

## Bring-up procedure

Work through these stages in order; each one only risks what the previous
stage verified.

The arm has no limit switches, so the datum is set manually with `SETHOME`:
you place the arm at a known reference pose, `ENABLE` so the drivers hold it,
then declare that pose's angles. It stays valid until the next `DISABLE`.

### 1. Smoke test — no motors, no PSU

Flash the ESP32 with nothing but USB connected. In the monitor:

```
PING            → PONG rt-arm-fw 0.1.0
STATE           → current mode / pose / flags
```

If PING answers, the boot chain (config parse, step engine, console) is
alive.

### 2. First power — motors connected, arm NOT assembled

Wire everything, including the PDN_UART bus and the address straps (the
firmware sets motor current at boot — no pots to adjust), and clamp the
motors to the bench (or leave them loose) — do not couple them to the arm
yet.

```
ENABLE          → OK ENABLE      (motors should now hold — try turning a shaft by hand)
DISABLE         → OK DISABLE     (shafts turn freely again)
```

Confirm each motor holds under `ENABLE` and spins freely under `DISABLE`. If a
motor whines but won't hold, or a commanded direction is reversed, note it —
`invert_dir` in `firmware.yaml` fixes direction later.

### 3. Assembled arm — first datum with SETHOME

Mount the motors, couple the gearboxes, and place the arm at your **reference
pose** — a repeatable configuration you can hit accurately, ideally a printed
alignment jig (eyeballing works but the datum is only as good as the
placement). Keep a hand near the PSU switch.

```
ENABLE                       → drivers energize and hold the arm at the placed pose
SETHOME <θ1> <θ2> <θ3>       → EV HOMED manual datum   (angles of your reference pose, deg)
```

`SETHOME` zeroes the step counters to the angles you gave and marks the arm
homed. Re-run it after **every** `DISABLE` — unpowered steppers slip under
gravity, so the datum is dropped on disable by design. Verify it: command a
small move you can eyeball and measure the real angles; if they disagree, your
placement was off — re-place and `SETHOME` again.

### 4. First moves

Moves are refused with `ERR NOT_HOMED` until you `SETHOME` — that's the
firmware protecting you, not a bug.

```
MOVEJ 0 90 -90                → OK MOVEJ T=1.204 STRETCH=1.00 … EV MOVE_DONE
MOVEL 150 0 200               → straight-line move to x/y/z in mm
TELEM 10                      → streams ST pose lines at 10 Hz
STOP                          → halts immediately (also clears FAULT)
```

Every move runs the full twin pipeline on-board (IK, torque retiming,
collision check) — a command the simulator would refuse is refused on
hardware with the same `ERR TORQUE` / `ERR COLLISION` verdict. A
`STRETCH` greater than 1.00 means the torque governor slowed the move to
stay under the torque ceiling; that's normal for aggressive targets.

### 5. Gripper calibration

Do this once, with the arm parked and nothing between the jaws. **Do it against
the Uno alone** — the ESP32 does not need to be powered or even connected, which
is most of the reason the gripper has its own board. The whole model of the
gripper is two pulse widths and the opening they correspond to, all in the
`gripper:` block of `config/firmware.yaml`, and each edit needs a reflash of the
gripper node (`pio run -t upload` in `arduino/gripper_node/`). The shipped values
(1200 / 1800 µs over a nominal 30 mm) are deliberately narrow: a servo driven
past a mechanical stop stalls, draws ~650 mA and strips its own gears within
seconds, so you **widen toward the ends**, never start at them.

Set the bench supply to 5.0 V with a **1 A limit** and leave its current readout
where you can see it. That limit is the safety net for this whole procedure: if
it trips, the pulse span is driving the mechanism into a hard stop, and the fix
is to narrow the calibration rather than push on. Expect roughly 200 mA while
moving.

Open the jaws by hand to `boot_opening_mm` before powering up. The node drives
nothing until its first command, so the jaws will not jump on power-on — but the
first `G` will snap them from the assumed opening to the commanded one.

Talk to it directly over its USB console (`pio device monitor`, 115200):

```
G 0                          → drives to closed_pulse_us
G 30                         → the other end of the calibration
R                            → release; the servo goes slack
```

Or skip the keyboard entirely and sweep the knob — it commands the same axis at
the same slew rate, which is usually the faster way to find where the mechanism
binds. The onboard LED is solid while the jaws hold and blinks while they move.

1. **Find the closed end.** Command `G 0` and watch. If the pads stop short
   of touching, lower `closed_pulse_us` by ~50 µs and reflash; repeat until they
   just meet with no buzz. If the servo grinds, the mechanism visibly strains,
   or the supply's current climbs toward its limit, go back up 50 µs — that is a
   stall, not a grip.
2. **Find the open end.** Command `G <max_opening_mm>` and push
   `open_pulse_us` the same way in ~50 µs steps until the jaws reach their widest
   comfortable opening, again stopping before anything binds.
3. **Measure the stroke.** With the servo holding the open end, measure the
   actual pad-face-to-pad-face gap with calipers and put that number in
   `max_opening_mm`. Every `GRIP` distance is scaled from it.
4. **Check the middle.** Command a few intermediate openings and measure them.
   They should track within a millimetre or so; if they bow consistently, the
   linkage isn't linear over that range and the honest fix is to narrow the
   calibration to the range where it is.
5. **Set the boot opening** to wherever you intend to leave the jaws parked, and
   reflash a last time.

Once the calibration is settled and both boards are wired together, the bundled
script exercises the same openings through the ESP32 and so proves the relay
link end to end:

```bash
python python/run_script.py --port COM5 python/examples/gripper_checkout.txt
```

Keep the Uno's console open while it runs — every command the ESP32 relays is
echoed there, and that echo is the only confirmation the link is alive.

Three things the firmware cannot do for you: it never knows whether the jaws
actually reached the opening it commanded (there is no feedback of any kind), it
cannot tell whether the gripper node is even powered (the link is one-way), and
it does not tell the torque governor that you are now carrying something — send
`PAYLOAD <grams>` yourself after a grasp.

### Command reference

| Command                  | Arguments            | Effect                                                                                                                                |
| ------------------------ | -------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| `PING`                 | —                   | liveness check, returns firmware name/version                                                                                         |
| `STATE`                | —                   | current mode, pose, homed/enabled flags                                                                                               |
| `ENABLE` / `DISABLE` | —                   | energize / de-energize drivers.**DISABLE drops the homed flag** (unpowered steppers slip under gravity) — re-`SETHOME` after |
| `SETHOME`              | θ₁ θ₂ θ₃ (deg) | set the datum — declare the arm's current pose as home. The datum method for this build                                              |
| `HOME`                 | —                   | switch-seek homing (needs limit switches —**not fitted on this arm**; use `SETHOME`)                                         |
| `MOVEJ`                | θ₁ θ₂ θ₃ (deg) | joint-space move                                                                                                                      |
| `MOVEL`                | x y z (mm)           | straight-line Cartesian move via analytic IK                                                                                          |
| `GRIP`                 | opening mm           | jaw opening, pad face to pad face. Relayed to the gripper node; slews at`speed_mm_s`; `EV GRIP_DONE` when it lands. Needs `ENABLE`, not a datum |
| `STOP`                 | —                   | immediate halt; clears FAULT                                                                                                          |
| `PAYLOAD`              | grams                | tell the torque governor what the gripper is holding                                                                                  |
| `TELEM`                | Hz (0 = off)         | telemetry stream rate                                                                                                                 |

### Running a script of commands

To play back a saved sequence instead of typing commands one at a time, use the
host runner. It reads a `.txt` file of the commands above (one per line; `#`
comments and blank lines ignored; `SLEEP <s>` pauses the host), sends each and
waits for it to finish before the next:

```bash
python python/run_script.py --port COM5 python/examples/demo_moves.txt
```

Its one added behavior: a `MOVEJ` that moves **more than one joint is executed
one joint at a time** (holding the others at their current angle), so the shared
breadboard motor rail only ever powers one moving coil — the fix for the supply
sag when all three drive together. Pass `--no-split` to send coordinated moves
instead. `--dry-run --start θ₁,θ₂,θ₃` previews the exact wire lines (including
the per-joint split) with no hardware attached. `MOVEL` can't be split
host-side (it resolves to a pose on-chip via IK) and is sent whole. Live runs
need `pip install pyserial`.

## Troubleshooting

| Symptom                                                                  | Likely cause                                                                                                                              |
| ------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------- |
| Motor whines/holds but won't step                                        | STEP/DIR swapped, or STEP pin miswired                                                                                                    |
| Moves land at exactly ½ / 2× / 4× the commanded angle                 | the UART config never reached that driver (PDN bus wiring, wrong address strap) so it's running its pin-strap default resolution          |
| One motor weak/silent while others behave; or two joints act identically | duplicate or wrong MS1/MS2 address straps — each driver needs a unique address (base 0, shoulder 1, elbow 2)                             |
| Joint moves the wrong direction                                          | flip that joint's`invert_dir` in `firmware.yaml` and reflash                                                                          |
| `ERR NOT_HOMED` on every move                                          | run`SETHOME` first; also re-`SETHOME` after any `DISABLE`                                                                           |
| Datum drifts / moves land off by a constant offset                       | reference-pose placement was inaccurate — re-place the arm and`SETHOME` again (a jig helps)                                            |
| Motors stutter or drivers reset under load                               | missing VM capacitor, undersized PSU, or missing common ground                                                                            |
| Skipped steps on fast moves                                              | `irun` too low, or `motor.holding_torque_nm` in `robot.yaml` optimistic — the governor can only respect the torque it's told about |
| ESP32 resets when motors enable                                          | motor PSU current sagging into the USB ground path — check grounding and PSU capacity                                                    |
| Jaws jump the moment you send`ENABLE`                                  | the servo is snapping to`gripper.boot_opening_mm` — park the jaws there before power-up, or correct the value and reflash             |
| Servo buzzes hard, gets hot, or grinds at one end of the stroke           | it is stalled against a mechanical stop — pull`closed_pulse_us`/`open_pulse_us` back toward the middle by ~50 µs and reflash        |
| `GRIP 20` doesn't give 20 mm at the pads                               | calibration:`max_opening_mm` must be the gap you actually measure at `open_pulse_us`                                                  |
| Jaws creep open while holding a part                                     | the mechanism is not self-locking and the servo let go — set`release_after_s: 0` (the default) so the PWM never stops                 |
| Jaws don't move at all,`ERR DISABLED` on GRIP                          | commands are only relayed after`ENABLE`                                                                                                 |
| `GRIP` returns`OK` but nothing moves                                   | the link is one-way, so the ESP32 cannot detect a dead node. Check the Uno's console for the echo, then its power, then the GPIO 13 → D2 wire and the shared ground |
| Uno console shows garbled characters after`ERR BADCMD`                 | the 3.3 V relay signal has only 0.3 V of margin at the Uno's input — drop`relay_baud` to 9600 and reflash both boards                  |
| Jaws move on their own during a script                                   | the manual knob took over — it reclaims control whenever it moves past its deadband. Leave it alone, or send another`GRIP` to take control back |
| Bench supply hits its current limit during calibration                   | the servo is stalled against a stop; narrow`closed_pulse_us`/`open_pulse_us` rather than raising the limit                             |
