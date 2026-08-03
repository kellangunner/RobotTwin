# RobotTwin — Bring-Up Handoff

**Date:** 2026-07-21
**Phase:** 6 (physical bring-up), in progress
**Scope of this doc:** the live work — getting firmware onto real hardware and
the arm moving. The software twin, C++/WASM core, and CAD are done; see
[CLAUDE.md](../CLAUDE.md) for the full project vision and the per-subsystem
docs for their details.

---

## TL;DR — start here

The firmware is finished and **proven on real hardware** — it built, flashed to
an ESP32, and answered `PING` → `PONG` over serial, with the embedded YAML
config parsing correctly on-chip. The blocker is **wiring the bench rig**. The
first dev board was killed by a wiring fault the instant a LiPo was connected
to a hand-wired breadboard. A new **hole-by-hole wiring guide with multimeter
verification gates** now exists specifically to prevent a repeat.

**To resume:** get a replacement ESP32 + a multimeter + an inline fuse, set
`irun: 12` for breadboard use, reflash, then build the rig by following
[breadboard-step-by-step.md](breadboard-step-by-step.md) top to bottom —
**do not skip Gate 1 and Gate 2.**

---

## Project state at a glance

| Layer | State |
|---|---|
| Web digital twin (`web/`) | ✅ Done — FK/IK, drivetrain, torque governor, collision, trajectories, 58 tests pass |
| C++20 core (`src/`) + WASM | ✅ Done — mirrors the twin, native tests pass, WASM drives the web app |
| Firmware (`firmware/`) | ✅ **Builds green + flash-proven on hardware** (this session) |
| CAD / mechanical | ✅ Frame, bearings, shafts ready to assemble (per owner) |
| **Physical bring-up** | 🔧 **IN PROGRESS — blocked on bench wiring** (dead board, rebuild pending) |

---

## What's proven (don't re-verify)

- **Firmware builds green** with PlatformIO (espressif32 6.13.0 / ESP-IDF
  5.5.3). Footprint: **Flash 52.5%, RAM 5.7%**. Full clean build from scratch
  produces `firmware.bin`.
- **First hardware flash succeeded.** `pio run -t upload` → board boots →
  serial console emits `EV BOOT rt-arm-fw` → `PING` returns
  `PONG rt-arm-fw 0.1.0`.
- **The embedded config parses on-target.** That clean `EV BOOT` (no panic, no
  reboot loop) is the runtime proof that `config/robot.yaml` + `firmware.yaml`
  are correctly baked into flash and parsed by the on-board loaders.

The whole software chain — build → flash → boot → config → console — is
validated on real silicon. **A replacement board reflashes to this exact state
in ~10 minutes.**

---

## This session's changes — ⚠️ UNCOMMITTED

The working tree has meaningful uncommitted changes. **Decide whether to commit
before continuing** (kept uncommitted so far by owner preference). All changes
are isolated to firmware + docs; nothing touched the twin, core, or CAD.

### Firmware build fix (the reason the firmware now builds under PlatformIO)

The original firmware embedded the YAML via ESP-IDF's `EMBED_TXTFILES` in a
component `CMakeLists.txt`. PlatformIO's SCons build mis-maps the generated
`robot.yaml.S` into a **doubled build path**
(`.pio\build\esp32dev\.pio\build\esp32dev\robot.yaml.S.o` → "Source not
found") — so the firmware had never actually built under PlatformIO despite
being assumed to. Replaced with a build-time code generator:

| File | Change |
|---|---|
| `firmware/scripts/gen_embedded_configs.py` | **New.** Pre-build hook: reads `config/*.yaml` → writes `src/embedded_yaml_gen.hpp` (`kRobotYaml`/`kFirmwareYaml` raw-string literals). |
| `firmware/platformio.ini` | Added `extra_scripts = pre:scripts/gen_embedded_configs.py`. |
| `firmware/src/embedded_configs.cpp` | Uses the generated header instead of `asm("_binary_*")` symbols. |
| `firmware/components/robottwin_core/CMakeLists.txt` | Removed `EMBED_TXTFILES`. |
| `firmware/src/embedded_configs.hpp` | Comment updated. |
| `firmware/.gitignore` | **New.** Ignores `.pio/` (was NOT ignored — the whole build tree was committable) and the generated header. |

Same single-source-of-truth: the twin's YAML is still baked into flash,
regenerated every build. A pin change is still a config edit + reflash, never a
code change.

### Documentation

| File | Change |
|---|---|
| `docs/breadboard-step-by-step.md` | **New.** Hole-by-hole guide for the owner's specific bench (see below). |
| `docs/breadboard-wiring.md` | Cross-link added; corrected the "two PDN pads are tied" line for split TX/RX carriers. |
| `docs/wiring-and-bringup.md` | Cross-link added; corrected a wrong claim that the 30-pin ELEGOO lacks GPIO16/17. |
| `docs/firmware.md` | `EMBED_TXTFILES` reference updated to the generated-header mechanism. |

---

## The hardware incident (what to learn from, not repeat)

- **What died:** an ELEGOO ESP32 DevKit V1 TypeC (CP2102 USB-serial). It still
  powers its LED but **enumerates zero COM ports** — the USB-serial chip is
  fried; the board is dead as a controller.
- **When:** the instant a **3S LiPo (~11.45 V, used as the motor supply)** was
  connected to a hand-wired breadboard rig.
- **Why (best hypothesis, never traced):** motor voltage reached the 3.3 V
  logic side — most likely LiPo onto a logic net / VIN, or a missing/faulty
  common ground. The rat's-nest wiring made an autopsy less reliable than a
  clean rebuild, so a rebuild was chosen.
- **The lesson, now baked into the guide:** the two most dangerous mistakes are
  (1) motor voltage bridging to the 3.3 V rail / VIN, and (2) reversed VM
  polarity. The new guide's **Gate 1** (cold multimeter check: *VM ↔ 3V3 must
  be open*) and **Gate 2** (staged power-up with drivers removed for first
  battery contact) exist to catch exactly these before anything expensive is at
  risk.

---

## Immediate next steps (in order)

1. **Acquire hardware:**
   - Replacement ESP32 — ELEGOO DevKit V1 TypeC, or any 30-pin WROOM-32 with
     the same labels (the guide's rows are locked to this board).
   - **Multimeter** (required — the verification gates depend on it).
   - **Inline fuse holder + 3–5 A fuse** for the LiPo + lead.
   - A **data** USB-C cable (not charge-only) — a brand-new board that shows no
     COM port is almost always the cable.
   - Confirm on hand: 3 × 100 µF ≥35 V caps, 1 × 1 kΩ resistor, 22 AWG
     solid-core jumpers.
2. **Set breadboard current:** edit `config/firmware.yaml` → `tmc_uart.irun:
   12` (down from 16; the breadboard ceiling is ≤ 0.8 A). Rebuild + reflash.
   **Restore `irun: 16` only when the rig graduates to soldered wiring.**
3. **Rebuild the rig** strictly per
   [breadboard-step-by-step.md](breadboard-step-by-step.md). Strip the boards
   bare first. **Do not skip Gate 1 (cold check) or Gate 2 (staged power).**
4. **Bring-up procedure** (from
   [wiring-and-bringup.md](wiring-and-bringup.md)): smoke test (`PING`) →
   `ENABLE` and feel each shaft lock → `SETHOME` manual datum (this arm has
   **no limit switches**) → first `MOVEJ` / `MOVEL`.
5. **Then** couple to the arm and tune (see watch-outs).

---

## Key commands

```bash
# All from the firmware/ directory, in a terminal where `pio` is on PATH
pio run                 # build (regenerates the embedded config header first)
pio run -t upload       # flash over USB
pio device monitor      # serial console, 115200 baud (--echo to see your typing)
```

Both YAML configs are embedded at build time, so **any change to
`config/firmware.yaml` or `config/robot.yaml` needs a rebuild + reflash** —
never a code change.

---

## File & doc map

| Path | What |
|---|---|
| [docs/breadboard-step-by-step.md](breadboard-step-by-step.md) | **Resume here** — hole-by-hole rig for the dovetailed 2-board bench |
| [docs/breadboard-wiring.md](breadboard-wiring.md) | Board-agnostic net list (electrical source of truth) |
| [docs/wiring-and-bringup.md](wiring-and-bringup.md) | Bring-up procedure, command reference, troubleshooting |
| [docs/firmware.md](firmware.md) | What the firmware does |
| [docs/BOM.md](BOM.md) | Bill of materials |
| `config/firmware.yaml` | Board wiring: pins, UART, current, addresses (**single source of truth for pins**) |
| `config/robot.yaml` | The robot: geometry, gearboxes, limits, motor, collision |
| `firmware/scripts/gen_embedded_configs.py` | Build-time config-embed generator |

**Pin summary** (from `firmware.yaml`): base STEP/DIR = D4/D27, shoulder =
D18/D19, elbow = D21/D22, shared EN = D23, UART TX = D26 (→ 1 kΩ → driver
**RX**, write-only), gripper relay TX = D13 (→ 1 kΩ → Arduino Uno D2; the
SG-90 runs off that separate board, not this one — reserved, not yet
wired on the bench rig). Driver UART
addresses: base 0, shoulder 1, elbow 2 (MS1/MS2 straps).

---

## Open items & watch-outs

- **Uncommitted working tree** — the firmware fix + all docs above. Commit
  decision pending.
- **Bench rig is bench-only.** The 2-board layout shares one motor rail pair
  across all three drivers (only two rail pairs exist), so it's for smoke test
  / hold test / slow unloaded moves. Solder or use screw terminals before
  running the assembled arm.
- **Split TX/RX PDN pads:** the owner's driver carriers break PDN_UART into
  separate TX/RX pads. This build's UART is write-only — wire the bus to **RX**
  only, leave TX empty.
- **Flash-size warning** during build (`Expected 4MB, found 2MB`) is cosmetic —
  app fits fine. Only worth reconciling (`sdkconfig`) if OTA/larger partitions
  are wanted later.
- **Motion-tuning risk (later phase):** the live-shaft pinch joints are
  friction joints on polished rod (~few Nm vs 5.4 Nm available) — see open
  issue 1 in `engineering/gearboxes/README`. Mitigations: degrease + Loctite
  638 + the firmware torque governor. Relevant once moving under load, not for
  electrical bring-up.
- **Doc drift flagged separately:** `docs/firmware.md` still references limit
  switches in one spot (the arm has none) — a background task was spun off for
  it.
