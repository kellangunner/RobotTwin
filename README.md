# RobotTwin

Digital twin of a 3-DOF desktop robotic arm (base yaw · shoulder pitch · elbow pitch),
designed to be 3D-printed on a Bambu Lab A1 Mini and driven by NEMA 17 steppers through
printed planetary/cycloidal gearboxes.

**Design stance:** linkage geometry is fixed and validated for printability
([docs/linkage-geometry.md](docs/linkage-geometry.md)); the **gearboxes are the independent
variables**. The interactive twin exists to answer: *which reduction, at what efficiency and
backlash, makes this arm work?*

## Quick start

```
cd web
npm install
npm run dev     # interactive twin at http://localhost:5173
npm test        # kinematics / drivetrain / trajectory unit tests
```

Pick each joint's drive type (direct 1:1 / planetary / cycloidal) and reduction ratio in the
left panel — efficiency, backlash, and torque caps are derived from the type — and watch torque
budgets, TCP backlash error, speed limits, move durations, and skipped-step predictions react.
Command poses via Cartesian target (analytical IK, elbow-up/down) or joint sliders.

## Layout

| Path | Contents |
|---|---|
| `config/robot.yaml` | Single source of truth for all robot parameters (SI-converted on load) |
| `docs/` | Decision records & subsystem docs |
| `web/` | Interactive digital twin — React + TypeScript + React Three Fiber |
| `src/` *(planned)* | C++20 core library (kinematics/dynamics/planning) → WASM + firmware |
| `cad/` *(planned)* | Fusion 360 generation scripts driven by the same YAML |

See [CLAUDE.md](CLAUDE.md) for the full project charter and roadmap, and
[docs/digital-twin.md](docs/digital-twin.md) for the twin's architecture, equations, and
limitations.
