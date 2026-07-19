# ESP32 Firmware

## Overview

The firmware makes the physical arm behave exactly like the digital twin: the same C++20 core
library (kinematics, drivetrain, dynamics, planning, retiming, collision) compiled unchanged for
the ESP32, fed by the same two YAML files, running the same plan-time pipeline before any move
reaches a motor. A command the twin would refuse is refused on hardware with the same verdict;
a command the twin would slow down is slowed by the same stretch factor.

Hardware: ESP32 DevKit (WROOM-32) + three TMC2209 drivers in step/dir mode, one shared enable
line, one limit switch per joint. Board wiring lives in `config/firmware.yaml`; the robot itself
in `config/robot.yaml`. Both are embedded into the flash image at build time (`EMBED_TXTFILES`)
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
    motion_controller.cpp      state machine + the twin's planning pipeline + homing
    serial_console.cpp         UART0 line protocol (rt::proto), telemetry, events
    tmc2209.cpp                optional write-only UART config of the drivers
```

Three execution contexts, strictly layered by math weight:

| Context | Rate | Work |
|---|---|---|
| GPTimer ISR (`StepEngine`) | `step_tick_hz` (40 kHz) | integer DDA only: accumulate Q16.16 rate, emit one step toward target, write GPIO registers |
| control task (`MotionController`, core 1) | `loop_hz` (1 kHz) | sample the active quintic, convert to microstep targets + rates, homing state machine |
| console task (`SerialConsole`, core 0) | on demand | parse commands, run the full planning pipeline (IK, metrics, audit, retime, collision) |

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

Joint-by-joint in `homing_order`, per-joint watchdog `homing_timeout_s`:

seek fast toward the switch → back off `backoff_deg` → re-seek slow → the slow trip is the
datum (`setPosition(home_angle_deg)`) → back off again to release the switch. Steps are the
ground truth thereafter; before the first HOME, position 0 is simply the boot pose and moves
are refused with `ERR NOT_HOMED`.

## Step generation

One GPTimer ISR services all three joints with an integer DDA: per tick a channel accumulates
its Q16.16 rate and on overflow emits one step toward its target — position can never overshoot,
and the max step rate equals `step_tick_hz`. The control loop feeds absolute microstep targets
plus a tracking rate (sampled |q̇| × 1.25 + 50 steps/s so quantization never outruns the DDA).
Conversion is `steps/rad = 1 / jointResolution(motor, gearbox)` from the shared drivetrain model.

## TMC2209 UART (optional)

When `tmc_uart.enabled`, boot pushes GCONF (UART current control, MRES from register), CHOPCONF
(microstep resolution from `robot.yaml motor.microstepping`) and IHOLD_IRUN to each strap-addressed
driver over the shared single-wire UART. Write-only — nothing is read back; step/dir remains the
only motion path. When disabled, the MS1/MS2 pin straps must match `robot.yaml` by wiring.

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
TELEM 10        → OK TELEM + ST lines at 10 Hz
```

## Assumptions

- The arm is racked roughly upright at power-on (elbow homes first; see `firmware.yaml`).
- Limit switch angles/directions are bring-up placeholders until measured on the real arm.
- Open-loop steppers: the torque governor plus the audit's skipped-step prediction are the only
  defenses against lost steps; there is no encoder feedback.
- step/dir/enable pins < GPIO32 (the ISR writes the low GPIO bank's set/clear registers).

## Limitations / next steps

- No host-side `HardwareRobot` backend yet — the protocol codec (`src/hardware/serial_protocol`)
  is shared and round-trip tested, so the twin can grow a WebSerial/`IRobot` client next.
- Homing values and TMC current scales need tuning against real hardware.
- `EV` lines carry no timestamps; add them if trajectory tracking needs offline analysis.
- No firmware-side gripper control yet (the protocol has no GRIP verb).
  The gripper is an SG-90 hobby servo: one LEDC PWM channel at 50 Hz
  (GPIO 13 reserved) mapping pulse width to jaw opening — no step engine,
  driver, or homing involved.
