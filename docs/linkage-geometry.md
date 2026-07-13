# Linkage Geometry Decision Record

**Status:** Fixed for Phase 2–3. The gearbox parameters are the independent variables of the
digital twin; everything in this document is held constant so that gearbox trade-offs can be
studied in isolation.

## Decision

| Parameter | Value | Notes |
|---|---|---|
| Base height (table → shoulder pitch axis) | **90 mm** | Houses base NEMA 17 (vertical) + base gearbox + 608 thrust path |
| Upper arm (shoulder axis → elbow axis) | **120 mm** | |
| Forearm (elbow axis → TCP) | **120 mm** | TCP = gripper grasp point |
| Maximum horizontal reach | **240 mm** | From base yaw axis |
| Maximum working height | **330 mm** | Arm fully vertical |

### Joint conventions and limits

World frame: Z up, X forward. All angles zero at the reference pose (arm horizontal, straight, pointing along +X).

| Joint | Axis | Symbol | Limits | Notes |
|---|---|---|---|---|
| Base yaw | Z (vertical) | θ₁ | ±135° | Limited for cable routing |
| Shoulder pitch | horizontal, in-plane | θ₂ | 0° … 180° | Measured from horizontal, positive up |
| Elbow pitch | parallel to shoulder | θ₃ | ±150° | 0° = straight arm (workspace-boundary singularity); both elbow-up and elbow-down branches mechanically reachable |

Kinematics (arm plane at azimuth θ₁):

```
r   = L1·cos θ2 + L2·cos(θ2+θ3)
z   = h  + L1·sin θ2 + L2·sin(θ2+θ3)
tcp = (r·cos θ1, r·sin θ1, z)
```

## Why these numbers

**Printability on Bambu Lab A1 Mini (180 × 180 × 180 mm):**

| Part | Bounding size (est.) | Fits? |
|---|---|---|
| Upper arm (120 mm axis-to-axis + 2 × ~24 mm joint housings) | ~168 × 60 × 45 mm | ✔ printed flat |
| Forearm (120 mm + one housing + gripper mount) | ~160 × 55 × 40 mm | ✔ |
| Base housing | ⌀ ~140 × 70 mm | ✔ |
| Cycloidal / planetary gearbox | ⌀ 60–80 mm | ✔ |

Every structural part clears the build volume with ≥ 10 mm margin; no part needs to be split.

**Torque feasibility (worst case, arm straight and horizontal, 100 g payload):**

- Shoulder gravity torque ≈ **1.2 N·m** (link masses + elbow-mounted NEMA 17 + gripper + payload)
- Elbow gravity torque ≈ **0.30 N·m**
- A NEMA 17 (0.45 N·m holding) needs roughly a **4:1 ratio at ~80 % efficiency just to hold** the shoulder
  at full extension — which is exactly the regime where the gearbox trade study is interesting.
  Ratios from 1:1 (fails) to ~30:1 (slow but strong) all produce meaningfully different behavior.

**Equal link lengths (120/120):** maximizes dexterous annulus volume for a given reach, keeps
IK conditioning symmetric, and lets the upper arm and forearm share bearing/housing designs.

**Shoulder axis intersects base axis (no lateral offset):** keeps the analytical IK exact and O(1)
with no offset compensation term.

## Fixed mechanical assumptions baked into the mass model

- Motors: 3 × NEMA 17, 42.3 × 42.3 × 48 mm, ~350 g, 0.45 N·m holding torque
- Base and shoulder motors live in/near the base; **elbow motor is mounted at the elbow** (its 350 g
  appears at the end of the upper arm in the gravity model — deliberate, so gearbox ratio choices
  are stress-tested against a realistic mass distribution)
- Bearings: 608 (8 × 22 × 7 mm) at every joint; structural loads through bearings, gearboxes transmit torque only
- Printed material: PETG at ~30 % infill for link mass estimates

## Consequences

- All of the above lives in [config/robot.yaml](../config/robot.yaml); source code never hardcodes it.
- Changing geometry later means editing the YAML and re-validating torque margins in the twin —
  no source changes.
- CAD (Phase 4) must place joint axes exactly 120 mm apart and the shoulder axis 90 mm above the
  table datum.
