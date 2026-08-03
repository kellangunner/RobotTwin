# ESP32 Firmware

## Overview

The firmware makes the physical arm behave exactly like the digital twin: the same C++20 core
library (kinematics, drivetrain, dynamics, planning, retiming, collision) compiled unchanged for
the ESP32, fed by the same two YAML files, running the same plan-time pipeline before any move
reaches a motor. A command the twin would refuse is refused on hardware with the same verdict;
a command the twin would slow down is slowed by the same stretch factor.

Hardware: ESP32 DevKit (WROOM-32) + three TMC2209 drivers in step/dir mode, one shared enable
line, one limit switch per joint, and one SG-90 servo on the gripper. Board wiring lives in `config/firmware.yaml`; the robot itself
in `config/robot.yaml`. Both are baked into the flash image at build time (a generated header — `firmware/scripts/gen_embedded_configs.py`)
and parsed at boot by the same loaders the native tests exercise — a pin change is a config edit
plus reflash, never a code change.

## Architecture

```
firmware/
  components/robottwin_core/   the repository's src/ as an ESP-IDF component + embedded YAMLs
  src/
    main.cpp                   boot order: configs → step engine → TMC2209 → controller → console
    embedded_configs.cpp       flash-embedded YAML → RobotConfig / HardwareConfig
    step_engine.cpp            40 kHz GPTimer ISR, integer DDA per joint (step/dir generation)
    gripper_link.cpp           one-way UART2 relay to the gripper node's Arduino
    motion_controller.cpp      state machine + the twin's planning pipeline + homing
    serial_console.cpp         UART0 line protocol (rt::proto), telemetry, events
    tmc2209.cpp                optional write-only UART config of the drivers
```

Three execution contexts, strictly layered by math weight:

| Context                                     | Rate                      | Work                                                                                        |
| ------------------------------------------- | ------------------------- | ------------------------------------------------------------------------------------------- |
| GPTimer ISR (`StepEngine`)                | `step_tick_hz` (40 kHz) | integer DDA only: accumulate Q16.16 rate, emit one step toward target, write GPIO registers |
| control task (`MotionController`, core 1) | `loop_hz` (1 kHz)       | sample the active quintic, convert to microstep targets + rates, homing state machine       |
| console task (`SerialConsole`, core 0)    | on demand                 | parse commands, run the full planning pipeline (IK, metrics, audit, retime, collision)      |

Planning never runs in the control loop; the control loop never allocates. Shared state crosses
tasks under a short spinlock, and every control-tick write to the step engine re-checks the mode
under that lock so a STOP can never be overwritten by a tick that raced it. Completion and fault
notifications flow back through a FreeRTOS queue and appear on the wire as `EV` lines.

## Command pipeline (mirrors `web/src/state/store.ts`)

For `MOVEJ` / `MOVEL`:

1. guards — FAULT? busy? drivers enabled? homed?
2. `MOVEL` only: analytic IK, keep limit-respecting collision-free solutions, take the one
   closest to the current pose (same rule as the twin's `pickIkSolution`)
3. `computeMetrics` at the current pose → per-joint `vmax`/`amax`; plan with the twin's 0.8
   speed margin and `safety.min_move_duration_s`
4. `retimeForTorque` against `safety.torque_ceiling` / `safety.max_stretch` — the torque
   governor; a static overload rejects with `ERR TORQUE`
5. `checkPath` over the retimed plan — a colliding path rejects with `ERR COLLISION`
6. install the plan; reply `OK MOVEJ T=<s> STRETCH=<k>`; `EV MOVE_DONE` when settled

State machine: `IDLE → HOMING → IDLE`, `IDLE → MOVING → IDLE`, anything → `FAULT` (homing
timeout, stuck switch). `STOP` halts and clears FAULT; `DISABLE` de-energizes and drops `homed`
(unpowered steppers slip under gravity, so the datum is no longer trustworthy).

## Homing

Two ways to establish the datum; both set `homed` and unlock moves, and both are lost on
`DISABLE`.

**`SETHOME θ₁ θ₂ θ₃` (manual, no switches).** The operator places the arm at a known pose and
declares it: the firmware validates the angles against the joint limits, `setPosition`s each
step counter to them, and marks `homed`. This is the datum path for the arm as designed —
it has no limit switches and no hard stops. Requires `ENABLE` first (drivers must be holding
the placed pose).

**`HOME` (switch-seek, only if switches are fitted).** Joint-by-joint in `homing_order`,
per-joint watchdog `homing_timeout_s`: seek fast toward the switch → back off `backoff_deg` →
re-seek slow → the slow trip is the datum (`setPosition(home_angle_deg)`) → back off again to
release the switch.

Steps are the ground truth thereafter; before the first datum, position 0 is simply the boot
pose and moves are refused with `ERR NOT_HOMED`.

## Gripper axis

The gripper is not a joint. It is an SG-90 hobby servo — a closed-loop actuator in its own right,
told a position and left to hold it — driving the parallel-jaw mechanism at the forearm tip. So it
gets none of the machinery the three joints get: no step/dir, no driver, no trajectory, no torque
governor, no homing, no place in the state machine.

**It does not run on the ESP32.** An Arduino Uno drives the servo from its own 5 V supply and the
ESP32 only relays commands to it over one wire. Two reasons: a stalled SG-90 pulls ~650 mA, which
is precisely the transient that browns out a board generating step pulses, and a hobby servo wants
5 V logic rather than the ESP32's marginal 3.3 V. Because the gripper was already the least-coupled
subsystem here, the split cost one driver class and changed nothing about the protocol.

What it does get:

| Piece               | Where                             | Job                                                                                                            |
| ------------------- | --------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `gripper:` block  | `config/firmware.yaml`          | pins for both boards, PWM rate, the two calibration pulses, stroke, slew speed, boot opening, idle release     |
| `rt::GripperAxis` | `src/hardware/gripper.cpp`      | opening → pulse width, slew limiting, idle release — pure, shared, unit tested. Runs on**both** boards |
| `fw::GripperLink` | `firmware/src/gripper_link.cpp` | writes`G <mm>` / `R` to UART2; drops repeats so a 1 kHz loop stays off a 19200 baud wire                   |
| `tickGripper`     | `motion_controller.cpp`         | runs every control tick in*every* mode, outside the state machine                                            |
| gripper node        | `arduino/gripper_node/`         | the Uno sketch: servo, manual knob, both serial inputs                                                         |

**Both boards slew the same axis.** The ESP32 sends a destination, not a position, and the Uno
gets there on its own — they run identical `rt::GripperAxis` math from identical calibration (the
Uno's copy is baked in at compile time by `gen_gripper_config.py`, since an ATmega328P has no room
for a YAML parser). That agreement is what lets the ESP32 time `EV GRIP_DONE` and fill in `grip=`
without ever hearing back over the one-way link.

**The model is two points.** `closed_pulse_us` is 0 mm, `open_pulse_us` is `max_opening_mm`, and
opening is linear between them — which is what a rack-and-pinion (or an equivalent geared parallel
linkage) gives over its usable swing. The firmware knows nothing else about the mechanism, so a
`GRIP` distance is only as true as that calibration.

**Commands slew, they don't jump.** `GRIP <mm>` is rate-limited to `speed_mm_s`, so a full-stroke
command neither flings the mechanism nor browns out the servo's rail; `EV GRIP_DONE` reports the
landing. `GRIP` is accepted in `IDLE` *and* `MOVING` (the axes are mechanically independent, so a
script can approach and close in one breath) and needs no datum — but it does need `ENABLE`, which
is what starts relaying to the Uno, and `FAULT` blocks it like everything else.

**`STOP` and `FAULT` freeze the jaws where they are** rather than opening them: dropping the part
is the worse failure. `DISABLE` cuts the servo drive along with the steppers — nothing on the robot
stays energized after it.

**It is open-loop, and more blindly so than the steppers.** There is no step counter to reason
from: the firmware reports the opening it commanded. On boot (and after `DISABLE`) it simply
believes `boot_opening_mm`, so park the jaws there before powering up or the servo will snap to it
on `ENABLE`. `release_after_s` stops the drive after that many seconds of stillness to silence the
holding buzz — the mechanism is not self-locking, so it defaults to 0 (drive forever).

**Two things the split costs.** The link is one wire and a shared ground with nothing coming back,
so the ESP32 cannot tell whether the Uno is powered, listening, or even connected — a `GRIP` that
reports `OK` may have gone nowhere. During bring-up, watch the Uno's own USB console, which echoes
everything it receives. And turning the Uno's manual knob mid-script moves the jaws without the
ESP32 knowing, leaving `grip=` stale until the next `GRIP` re-synchronises it.

## Step generation

One GPTimer ISR services all three joints with an integer DDA: per tick a channel accumulates
its Q16.16 rate and on overflow emits one step toward its target — position can never overshoot,
and the max step rate equals `step_tick_hz`. The control loop feeds absolute microstep targets
plus a tracking rate (sampled |q̇| × 1.25 + 50 steps/s so quantization never outruns the DDA).
Conversion is `steps/rad = 1 / jointResolution(motor, gearbox)` from the shared drivetrain model.

## TMC2209 UART (enabled)

This build runs `tmc_uart.enabled: true`: boot pushes GCONF (UART current control, MRES from
register), CHOPCONF (microstep resolution from `robot.yaml motor.microstepping`) and IHOLD_IRUN
to each strap-addressed driver over the shared single-wire UART (GPIO26 through a 1 kΩ resistor
to the bussed PDN_UART pins; MS1/MS2 are the address straps — base 0, shoulder 1, elbow 2).
Motor current therefore comes from `firmware.yaml` `irun`/`ihold`, not the VREF pots, and a
current change is a config edit + reflash. Write-only — nothing is read back; step/dir remains
the only motion path. If disabled, the MS1/MS2 straps revert to microstep-select and must match
`robot.yaml` by wiring.

## Building

Wiring and first-power-on instructions live in [wiring-and-bringup.md](wiring-and-bringup.md).

```
cd firmware
pio run              # build (PlatformIO, espressif32 + ESP-IDF)
pio run -t upload    # flash
pio device monitor   # the protocol console at 115200 baud
```

Typical bring-up session:

```
PING            → PONG rt-arm-fw 0.1.0
ENABLE          → OK ENABLE
HOME            → OK HOME … EV HOMING elbow … EV HOMED
MOVEJ 0 90 -90  → OK MOVEJ T=1.204 STRETCH=1.00 … EV MOVE_DONE
GRIP 0          → OK GRIP … EV GRIP_DONE
TELEM 10        → OK TELEM + ST lines at 10 Hz
```

## Assumptions

- The arm is racked roughly upright at power-on (elbow homes first; see `firmware.yaml`).
- Limit switch angles/directions are bring-up placeholders until measured on the real arm.
- Open-loop steppers: the torque governor plus the audit's skipped-step prediction are the only
  defenses against lost steps; there is no encoder feedback.
- step/dir/enable pins < GPIO32 (the ISR writes the low GPIO bank's set/clear registers).
- The gripper's jaw opening is linear in servo pulse width between the two calibration points, and
  the jaws are physically at `boot_opening_mm` when the board powers up.

## Limitations / next steps

- No host-side `HardwareRobot` backend yet — the protocol codec (`src/hardware/serial_protocol`)
  is shared and round-trip tested, so the twin can grow a WebSerial/`IRobot` client next.
- Homing values and TMC current scales need tuning against real hardware.
- `EV` lines carry no timestamps; add them if trajectory tracking needs offline analysis.
- The gripper calibration in `firmware.yaml` ships deliberately conservative (1200–1800 µs over
  a nominal 30 mm) so a first power-on cannot stall the servo against a hard stop. Widen it and
  measure the real opening before trusting `GRIP` distances.
- The gripper is open-loop and unsensed: the firmware reports the opening it *commanded*, and a
  slipped, obstructed, or backdriven jaw looks identical to a successful grasp. Closing on a part
  is what `PAYLOAD` is for — the torque governor is not told about it automatically.
