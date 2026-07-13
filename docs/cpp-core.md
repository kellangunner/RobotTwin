# C++ Core — Subsystem Documentation

## Overview

`src/` is the C++20 implementation of every computational module of the digital twin —
kinematics, drivetrain, gearbox derivation, dynamics, trajectory planning, metrics/audit,
collision detection, and CSV waypoint parsing. It is the **reference implementation** destined
to power all three targets from CLAUDE.md: native simulation, the WebAssembly module behind the
web twin, and (subset) embedded firmware.

Each file mirrors a `web/src/core/*.ts` module one-for-one, and the two test suites assert the
**same numeric fixtures** (gravity 1.166 / 0.294 N·m at full extension, 0.88² two-stage
planetary efficiency, identical IK round-trip grids and collision poses), so any divergence
between the implementations fails a test on one side or the other.

## Building (native)

Requires CMake ≥ 3.20 and a C++20 compiler (developed against MSVC 19.44 / VS 2022):

```
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
ctest --test-dir build -C Release        # or run build/Release/robottwin_tests.exe
```

The test binary loads the same `config/robot.yaml` the web twin uses (path injected via the
`ROBOTTWIN_CONFIG_PATH` compile definition).

## Building (WebAssembly)

`src/wasm/bindings.cpp` exposes the full API through embind. With the
[emsdk](https://emscripten.org/docs/getting_started/downloads.html) installed:

```
emcmake cmake -S . -B build-wasm
cmake --build build-wasm       # emits web/src/wasm/robottwin.{js,wasm} (ES6 module)
```

The JS side keeps YAML parsing (it already ships the config text) and passes a plain
`RobotConfig` object across the boundary; `src/config/yaml_lite.*` is native-only.

> Status: **integrated.** emsdk lives at `C:\Users\kgunner\emsdk` (activate with
> `emsdk_env.ps1`; pass `-DCMAKE_MAKE_PROGRAM=<winget ninja path>` on this machine). The web
> app's math calls go through `web/src/core/api.ts`, a thin adapter over the embind module
> (enum↔union-string conversion, joint records↔arrays; results cross as plain JS objects, no
> handles to delete). The TypeScript implementations in `web/src/core/` remain as the parity
> mirror exercised by vitest.

## Layout

| Path | Contents | Mirrors |
|---|---|---|
| `src/math/` | units, Vec3 helpers | `units.ts` |
| `src/config/` | typed config, YAML-subset loader (native only) | `config.ts` |
| `src/kinematics/` | FK, analytical IK (4 branches), Jacobian, workspace | `kinematics.ts` |
| `src/drivetrain/` | stepper curve, gearbox math, type→params derivation | `drivetrain.ts`, `gearboxModel.ts` |
| `src/dynamics/` | gravity torques, worst-case inertias | `dynamics.ts` |
| `src/planning/` | synchronized quintic point-to-point | `trajectory.ts` |
| `src/simulation/` | per-joint metrics, predictive trajectory audit | `metrics.ts` |
| `src/geometry/` | capsule/cylinder collision checks | `collision.ts` |
| `src/io/` | CSV waypoint parsing/validation | `waypoints.ts` |
| `src/wasm/` | embind bindings (Emscripten builds only) | — |
| `tests/` | self-registering micro-harness, 55 tests / ~1800 checks | `web/src/core/*.test.ts` |

## Design decisions

- **No external dependencies.** The YAML loader is a deliberate ~150-line subset parser scoped
  to exactly `robot.yaml`'s shape (2-space-indented maps, scalars, inline numeric lists,
  comments) and rejects everything else loudly. The test harness is ~70 lines. This keeps the
  core compilable offline on Windows, Linux, WASM, and eventually bare-metal.
- **Fixed-capacity results in hot paths** (`IkResult` holds ≤ 4 solutions inline, collision
  issues ≤ 5) — no heap allocation in kinematics/collision/dynamics, for firmware reuse.
  `std::vector`/`std::string` appear only in inherently variable-size I/O (waypoints, workspace
  boundary).
- **Collision issues are enums** with `describe()` for UI strings, so firmware can act on codes
  without string handling.

## Limitations / next steps

- Compile and smoke-test the embind module with emsdk; swap `web/src/state/store.ts` to import
  the WASM API and demote `web/src/core/` to type definitions.
- The YAML-subset parser intentionally fails on block lists and anchors; extend it only if the
  config schema ever needs them.
- CI should build both suites (`npm test` + `ctest`) so the mirrored fixtures actually gate
  divergence.
