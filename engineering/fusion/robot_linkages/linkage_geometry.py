"""RobotTwin — parametric geometry for the printed linkage members.

Pure Python, importable anywhere (no Fusion dependency): this module IS the
parametric layer. It defines every dimension, derives the joint stacks, emits
each part as an ordered list of primitive booleans (add/cut of cylinders and
axis-aligned boxes, all in world mm coordinates: Z up, X forward, arm at the
reference pose along +X), and self-checks every mating clearance analytically.

Consumers:
    robot_linkages.py          Fusion wrapper: replays the primitive lists
                               through TemporaryBRepManager and exports.
    python/audit_linkages.py   Numeric CSG interference audit: sweeps the
                               joints through their configured ranges and
                               samples for overlaps between parts, purchased
                               hardware, and reserved keep-out envelopes.

Architecture (fixes the v1 interference audit — see git history):

  Base (direct drive, theta-1):
    The NEMA 17 hangs from a 4 mm top plate (motor body is ~40 mm, NOT the
    48 mm v1 assumed; the body floats above the pan floor so any 38..42 mm
    body fits). Stack above the plate, bottom to top:
      journal Ø25 (6805-2RS in a boss on the pan) -> clamp body Ø28 (D-bore
      + split pinch on the 5 mm motor shaft, cross bolt ABOVE the bearing so
      it can be tightened after insertion, bearing slides on from the journal
      end) -> column plate bolted down into inserts in the clamp top.
    Everything on the column side stays below z = shoulder_z - 15.5 near the
    yaw axis so the upper arm's Ø26 shoulder hub (+ pinch hardware) swings
    clear through the full 0..180 deg shoulder range.

  Pitch joints (shoulder, elbow — identical):
    608 bearings in the clevis ears carry structural loads; the 8 mm shaft
    transmits torque; the driven hub grips the shaft with a printed pinch
    clamp (vertical M3 + captured nut). No enclosed gearbox drum: the
    drive-side ear is a Ø76 disc whose OUTBOARD FACE is the gearbox mounting
    flange (6 x M3 heat-set inserts on a Ø66 bolt circle). The cycloidal
    cartridge (Ø62 x 38.4, engineering/fusion/cycloidal_gearbox) bolts on via
    a Ø76 front flange and carries the NEMA 17 on its own back plate, so the
    drive is assemblable and serviceable from outside the link.

  Upper arm (the ±150 deg elbow fix):
    A single mid-plane beam cannot coexist with the forearm folded 150 deg —
    the two would be coplanar. The upper arm is therefore a twin-beam clevis:
    a central beam leaves the shoulder hub but hands off, through a full-width
    web, to two side beams that flank the forearm's swing plane all the way to
    the elbow ears. The hand-off station is derived from the sweep-wedge
    formula below so the folded forearm always clears it.
"""

import math
import os
import re

# ---------------------------------------------------------------------------
# Purchased hardware (verified against engineering/standard parts/*.step)
# ---------------------------------------------------------------------------

# 608 deep-groove bearing (8 x 22 x 7) — one per clevis ear
BRG_OD = 22.0
BRG_W = 7.0
BRG_POCKET_D = BRG_OD + 0.2     # light press after printing
BRG_RETAIN_D = 16.0             # keeps the outer race captive, clears the inner

# 6805-2RS (25 x 37 x 7) — yaw support bearing
YAW_BRG_ID = 25.0
YAW_BRG_OD = 37.0
YAW_BRG_W = 7.0
YAW_POCKET_D = YAW_BRG_OD + 0.3
YAW_BRG_PRESS_R_MAX = 15.0      # max radius that presses the inner race only

# NEMA 17 (17HS4401: 42.3 sq x ~40 body, 5 mm D-shaft ~23.5 usable, Ø22 x 2
# pilot boss, 31 mm bolt square). v1 assumed a 48 mm body — measured 40.
NEMA_SQ = 42.3
NEMA_LEN = 40.0
NEMA_LEN_MAX = 42.0             # design must tolerate the longest variant
NEMA_HOLE_SPACING = 31.0
NEMA_PILOT_D = 22.5             # pilot boss + clearance
NEMA_PILOT_H = 2.0
NEMA_SHAFT_D = 5.0
NEMA_SHAFT_LEN = 23.5           # above the mounting face (measured)
NEMA_SHAFT_LEN_MAX = 25.0

# 8 mm hardened steel shaft — torque path gearbox output -> driven hub
SHAFT_D = 8.0
SHAFT_BORE_D = SHAFT_D + 0.4

# 8 mm shaft collar (set-screw, ~Ø18 x 9): axial keeper outboard of the +Y ear
COLLAR_OD = 18.0
COLLAR_W = 9.0

# T8 lead-screw flanged nut (gripper actuation, forearm tip)
T8_BORE_D = 10.5
T8_FLANGE_D = 22.5
T8_FLANGE_T = 3.6
T8_BCD = 16.0

# Fasteners: M3 throughout. Printed member receiving a bolted joint gets an
# M3 x 5.7 heat-set insert; screws into a motor's own threads get clearance.
M3_CLEAR_D = 3.4
M3_HEAD_D = 5.5                 # socket head
M3_CB_D = 6.5                   # counterbore / spot-face for that head
M3_HEAD_H = 3.0
M3_NUT_AF = 5.5                 # across flats
M3_NUT_T = 2.4
INSERT_D = 4.6
INSERT_POCKET = 6.5

# Cycloidal gearbox cartridge (envelope contract with cycloidal_gearbox.py):
# Ø62 x 38.4 body + Ø76 x 4 front mounting flange (bolts to the ear disc
# face), NEMA 17 on its own back plate. The linkage provides the mounting
# face only. Keep-out keeps the historical Ø66 (conservative on the real
# Ø62 body); the length grew in v2 to embed the Ø6 x 25 steel output pins.
GBX_BODY_D = 66.0
GBX_BODY_L = 38.4
GBX_FLANGE_D = 76.0
GBX_FLANGE_T = 4.0
GBX_BCD = 66.0                  # 6 x M3 into inserts in the ear disc face
GBX_BOLT_N = 6

# ---------------------------------------------------------------------------
# Frozen link geometry (defaults; overridden by config/robot.yaml at runtime)
# ---------------------------------------------------------------------------

DEFAULT_SHOULDER_Z = 90.0
DEFAULT_L_UPPER = 120.0
DEFAULT_L_FORE = 120.0
DEFAULT_THETA2_RANGE = (0.0, 180.0)     # shoulder, from horizontal
DEFAULT_THETA3_RANGE = (-150.0, 150.0)  # elbow, 0 = straight

BUILD_VOLUME = 180.0            # Bambu Lab A1 Mini

# ---------------------------------------------------------------------------
# Base pan
# ---------------------------------------------------------------------------

PAN_D = 140.0
PAN_H = 50.0                    # gearbox flange (r38 about the shoulder axis)
                                # sweeps 2 mm above this through full yaw
PAN_WALL = 4.0
PAN_FLOOR_T = 3.0
PAN_PLATE_T = 4.0               # top plate the motor hangs from
PAN_RING_T = 4.0                # stiffening ring under the plate, outside motor
PAN_RING_ID = 60.0              # clears the motor's 59.8 mm face diagonal
PAN_MOUNT_BCD = 120.0
WIRE_HOLE_D = 12.0

MOTOR_FACE_Z = PAN_H - PAN_PLATE_T          # 46: motor face on the plate's underside

# Yaw bearing boss: a ring on the pan top plate. Driver-access notches over
# the four motor screws double as head room for them; the 6805's outer race
# still seats on the four arcs left in between.
BOSS_OD = 44.0
BOSS_LEDGE_D = 30.0             # bore under the pocket: outer-race seat ledge
YAW_BRG_Z0 = PAN_H + 1.5        # 51.5: bearing bottom (outer race seat plane)
BOSS_TOP = YAW_BRG_Z0 + YAW_BRG_W + 0.9     # 59.4
NOTCH_D = 9.0

# ---------------------------------------------------------------------------
# Shaft coupling (printed): journal at the BOTTOM so the 6805 slides on from
# the journal end (v1 trapped the bearing between a Ø26 clamp and Ø30 flange)
# ---------------------------------------------------------------------------

JOURNAL_D = 25.0                # = 6805 bore
JOURNAL_Z0 = PAN_H + 0.5        # 50.5: spins 0.5 above the pan plate
JOURNAL_Z1 = YAW_BRG_Z0 + YAW_BRG_W         # 58.5: bearing top = body shoulder
CLAMP_D = 28.0                  # presses the inner race, clears the outer
COUPLING_TOP = JOURNAL_Z1 + 11.0            # 69.5
COUPLING_BORE_D = 5.2
COUPLING_FLAT_X = 2.05          # D-flat plane (NEMA 17 flat: 4.5 across)
COUPLING_BCD = 17.0             # 4 x M3 inserts, column bolts down into them
CLAMP_SLIT_Z0 = JOURNAL_Z1 + 1.0            # keep the journal round
CLAMP_BOLT_X = 10.5             # cross-bolt chord offset
CLAMP_BOLT_Z = 63.0

# ---------------------------------------------------------------------------
# Pitch joints (shared by shoulder and elbow)
# ---------------------------------------------------------------------------

HUB_D = 26.0                    # driven hub: small so the column plate +
HUB_W = 26.0                    # screw heads fit under its swing circle
HUB_KEEPOUT_R = 14.2            # hub + pinch hardware swing envelope + margin

EAR_T = 12.0
JOINT_GAP = 0.75
EAR_IN = HUB_W / 2 + JOINT_GAP              # 13.75
EAR_OUT = EAR_IN + EAR_T                    # 25.75
EAR_BOSS_D = 46.0               # plain (+Y) ear boss around the bearing
EAR_DISC_D = 76.0               # drive (-Y) ear disc = gearbox mounting face

# Hub pinch clamp: horizontal slit on the side away from the beam, vertical
# M3 + captured nut. All hardware stays inside the hub's swing envelope.
PINCH_BOLT_X = 8.5              # offset from the hub axis, on the slit side
PINCH_SLIT_HALF = 0.75

# Yaw-boss relief in the column: the shoulder's -Y ear disc (Ø76 about the
# shoulder axis) reaches down to z = shoulder_z - 38 and would orbit through
# the pan's bearing boss and motor screw heads. Two revolved clearance cuts
# fix that; the gearbox flange seat (the disc's outboard FACE at |y| = 25.75)
# is untouched because its radius about the yaw axis is always >= 25.75.
BOSS_RELIEF_R1 = BOSS_OD / 2 + 1.5          # 23.5, up to just above the boss
BOSS_RELIEF_TOP1 = BOSS_TOP + 1.0           # 60.4
BOSS_RELIEF_R2 = 26.0                       # wider, over the screw heads
BOSS_RELIEF_TOP2 = PAN_H + M3_HEAD_H + 1.5  # 54.5

# Column plate (theta-1 output) under the shoulder hub
COL_PLATE_D = 68.0
COL_PLATE_T = 5.0
COL_PLATE_Z0 = COUPLING_TOP                 # 69.5
COL_PLATE_TOP = COL_PLATE_Z0 + COL_PLATE_T  # 74.5
SHAFT_TIP_RECESS_D = 8.5
SHAFT_TIP_RECESS_TOP = COL_PLATE_Z0 + 2.5   # 72
EAR_SUPPORT_HALF_X = 23.0       # support boxes under the clevis bosses

# Beams (upper arm is a twin-beam clevis; see module docstring)
UA_BEAM_HALF_H = 14.0           # central beam z = shoulder_z -14 .. +14
UA_BEAM_HALF_W = 12.0
UA_SIDE_HALF_H = 13.0           # side beams / web z = shoulder_z -13 .. +13
UA_SIDE_Y_OUT = EAR_OUT - 1.0   # 24.75: 1 mm off the gearbox flange plane
UA_WEB_LEN = 10.0
FA_BEAM_HALF_H = 12.0
FA_BEAM_HALF_W = 12.0
FA_PLATE_T = 12.0
FA_PLATE_HALF = 20.0

CLEAR = 1.0                     # default working clearance between members
SWEEP_MARGIN_DEG = 3.0          # angular margin beyond the joint limits


# ---------------------------------------------------------------------------
# Config loading — link geometry and joint limits from config/robot.yaml
# ---------------------------------------------------------------------------

def read_config():
    """Returns (shoulder_z, l_upper, l_fore, theta2_range, theta3_range,
    warnings). Falls back to the frozen defaults if the file is absent."""
    warnings = []
    here = os.path.dirname(os.path.abspath(__file__))
    yaml_path = os.path.join(here, '..', '..', '..', 'config', 'robot.yaml')
    sz, lu, lf = DEFAULT_SHOULDER_Z, DEFAULT_L_UPPER, DEFAULT_L_FORE
    t2, t3 = DEFAULT_THETA2_RANGE, DEFAULT_THETA3_RANGE
    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            text = f.read()
        def scalar(key, default):
            m = re.search(r'^\s*' + key + r'\s*:\s*([-0-9.]+)', text, re.MULTILINE)
            if m:
                return float(m.group(1))
            warnings.append(f'{key} not found in robot.yaml; using {default}')
            return default
        def limits(joint, default):
            m = re.search(joint + r'\s*:\s*\n\s*limits_deg\s*:\s*\[\s*([-0-9.]+)\s*,\s*([-0-9.]+)\s*\]', text)
            if m:
                return (float(m.group(1)), float(m.group(2)))
            warnings.append(f'{joint} limits not found in robot.yaml; using {default}')
            return default
        sz = scalar('base_height_mm', sz)
        lu = scalar('upper_arm_mm', lu)
        lf = scalar('forearm_mm', lf)
        t2 = limits('shoulder', t2)
        t3 = limits('elbow', t3)
    except OSError:
        warnings.append('Could not read config/robot.yaml; using built-in defaults.')
    return sz, lu, lf, t2, t3, warnings


# ---------------------------------------------------------------------------
# Primitives: everything is an ordered list of add/cut of these two shapes.
# All in world mm. `contains` powers the numeric audit.
# ---------------------------------------------------------------------------

class Cyl:
    """Finite solid cylinder from p1 to p2 (any axis), diameter d."""

    __slots__ = ('p1', 'p2', 'd', '_ax', '_len2', '_r2')

    def __init__(self, p1, p2, d):
        self.p1, self.p2, self.d = tuple(p1), tuple(p2), d
        self._ax = tuple(b - a for a, b in zip(p1, p2))
        self._len2 = sum(c * c for c in self._ax)
        self._r2 = (d / 2.0) ** 2

    def contains(self, p):
        w = (p[0] - self.p1[0], p[1] - self.p1[1], p[2] - self.p1[2])
        t = w[0] * self._ax[0] + w[1] * self._ax[1] + w[2] * self._ax[2]
        if t < 0.0 or t > self._len2:
            return False
        ww = w[0] * w[0] + w[1] * w[1] + w[2] * w[2]
        return ww - t * t / self._len2 <= self._r2 + 1e-9

    def bbox(self):
        r = self.d / 2.0
        lo, hi = [], []
        for i in range(3):
            e = r * math.sqrt(max(0.0, 1.0 - self._ax[i] ** 2 / self._len2)) \
                if self._len2 > 0 else r
            lo.append(min(self.p1[i], self.p2[i]) - e)
            hi.append(max(self.p1[i], self.p2[i]) + e)
        return lo, hi


class Box:
    """Axis-aligned solid box."""

    __slots__ = ('lo', 'hi')

    def __init__(self, x0, x1, y0, y1, z0, z1):
        self.lo = (min(x0, x1), min(y0, y1), min(z0, z1))
        self.hi = (max(x0, x1), max(y0, y1), max(z0, z1))

    def contains(self, p):
        return all(self.lo[i] - 1e-9 <= p[i] <= self.hi[i] + 1e-9 for i in range(3))

    def bbox(self):
        return list(self.lo), list(self.hi)


class Part:
    """One printed member (or hardware/keep-out solid): ordered booleans."""

    def __init__(self, name):
        self.name = name
        self.steps = []             # list of ('add'|'cut', prim)

    # -- construction helpers (mirror the old Fusion builder API) --
    def add(self, prim):
        self.steps.append(('add', prim))

    def cut(self, prim):
        self.steps.append(('cut', prim))

    def cut_bolt_circle_z(self, cx, cy, bcd, n, dia, z0, z1, clock_deg=0.0):
        for i in range(n):
            a = math.radians(clock_deg) + 2 * math.pi * i / n
            x, y = cx + bcd / 2 * math.cos(a), cy + bcd / 2 * math.sin(a)
            self.cut(Cyl((x, y, z0), (x, y, z1), dia))

    def cut_bolt_circle_y(self, cx, cz, bcd, n, dia, y0, y1, clock_deg=0.0):
        for i in range(n):
            a = math.radians(clock_deg) + 2 * math.pi * i / n
            x, z = cx + bcd / 2 * math.cos(a), cz + bcd / 2 * math.sin(a)
            self.cut(Cyl((x, y0, z), (x, y1, z), dia))

    # -- evaluation --
    def contains(self, p):
        inside = False
        for op, prim in self.steps:
            if prim.contains(p):
                inside = op == 'add'
        return inside

    def bbox(self):
        lo = [float('inf')] * 3
        hi = [float('-inf')] * 3
        for op, prim in self.steps:
            if op != 'add':
                continue
            plo, phi = prim.bbox()
            for i in range(3):
                lo[i] = min(lo[i], plo[i])
                hi[i] = max(hi[i], phi[i])
        return lo, hi


# ---------------------------------------------------------------------------
# Sweep-clearance formulas (the math the v1 design got wrong)
# ---------------------------------------------------------------------------

def swing_free_halfheight(r, sweep_deg, sweeper_half_h):
    """The forearm beam (half-height sweeper_half_h) swings ±sweep_deg about
    the elbow. At radius r from the elbow axis, material in the forearm's
    swing plane survives only inside a wedge around the 180° direction; this
    returns that wedge's half-height in z (0 if the wedge is closed)."""
    if r <= sweeper_half_h:
        return 0.0
    ang = math.radians(180.0 - sweep_deg) - math.asin(sweeper_half_h / r)
    return r * math.sin(ang) if ang > 0 else 0.0


def wedge_angle_clear_deg(r, half_extent, sweep_deg, sweeper_half):
    """Angular margin (deg) between a target feature at radius r with z
    half-extent half_extent (about the joint plane, near the 180° direction)
    and the sweeper's coverage boundary. Positive = clear."""
    target = 180.0 - math.degrees(math.asin(min(1.0, half_extent / r)))
    boundary = sweep_deg + math.degrees(math.asin(min(1.0, sweeper_half / r)))
    return target - boundary


class Checks:
    """Named analytic clearance checks; all must be >= their minimum."""

    def __init__(self):
        self.rows = []              # (label, clearance, minimum)

    def ck(self, label, clearance, minimum):
        self.rows.append((label, clearance, minimum))

    def failures(self):
        return [r for r in self.rows if r[1] < r[2] - 1e-6]

    def report(self):
        lines = []
        for label, c, m in sorted(self.rows, key=lambda r: r[1] - r[2]):
            flag = 'FAIL' if c < m - 1e-6 else 'ok  '
            lines.append(f'{flag} {c:7.2f} >= {m:5.2f}  {label}')
        return '\n'.join(lines)


# ---------------------------------------------------------------------------
# Shared pitch-joint features
# ---------------------------------------------------------------------------

def add_clevis(part, jx, jz):
    """Ear bosses about the pitch axis at (jx, ., jz): plain Ø46 boss on +Y,
    Ø76 disc on -Y whose outboard face is the gearbox mounting flange."""
    part.add(Cyl((jx, EAR_IN, jz), (jx, EAR_OUT, jz), EAR_BOSS_D))
    part.add(Cyl((jx, -EAR_IN, jz), (jx, -EAR_OUT, jz), EAR_DISC_D))


def cut_bearing_bores(part, jx, jz):
    """608 pockets from each ear's outer face; Ø16 retention bore through."""
    part.cut(Cyl((jx, -EAR_OUT - 1, jz), (jx, EAR_OUT + 1, jz), BRG_RETAIN_D))
    for s in (+1, -1):
        part.cut(Cyl((jx, s * (EAR_OUT - BRG_W), jz), (jx, s * (EAR_OUT + 1), jz),
                     BRG_POCKET_D))


def cut_gearbox_mount(part, jx, jz):
    """6 x M3 heat-set insert pockets in the -Y ear disc's outboard face:
    the cycloidal cartridge's Ø76 front flange bolts here."""
    for i in range(GBX_BOLT_N):
        a = 2 * math.pi * i / GBX_BOLT_N
        x = jx + GBX_BCD / 2 * math.cos(a)
        z = jz + GBX_BCD / 2 * math.sin(a)
        part.cut(Cyl((x, -EAR_OUT - 1, z), (x, -(EAR_OUT - INSERT_POCKET), z),
                     INSERT_D))


def add_hub(part, jx, jz, slit_toward=-1):
    """Driven hub on the 8 mm shaft with a printed pinch clamp: horizontal
    slit on the side away from the beam (slit_toward = -1 -> -X), vertical
    M3 + captured nut. Hardware stays inside the hub swing envelope."""
    part.add(Cyl((jx, -HUB_W / 2, jz), (jx, HUB_W / 2, jz), HUB_D))
    part.cut(Cyl((jx, -HUB_W / 2 - 1, jz), (jx, HUB_W / 2 + 1, jz), SHAFT_BORE_D))
    s = slit_toward
    x_far = jx + s * (HUB_D / 2 + 1)
    x_near = jx + s * 2.0
    part.cut(Box(min(x_far, x_near), max(x_far, x_near), -HUB_W / 2 - 1,
                 HUB_W / 2 + 1, jz - PINCH_SLIT_HALF, jz + PINCH_SLIT_HALF))
    bx = jx + s * PINCH_BOLT_X
    part.cut(Cyl((bx, 0, jz - 8), (bx, 0, jz + 8), M3_CLEAR_D))
    part.cut(Cyl((bx, 0, jz + 4.5), (bx, 0, jz + 11), M3_CB_D))          # head
    part.cut(Box(bx - M3_NUT_AF / 2 - 0.1, bx + M3_NUT_AF / 2 + 0.1,     # nut
                 -M3_NUT_T / 2 - 0.5, M3_NUT_T / 2 + 0.5, jz - 11, jz - 4.6))


# ---------------------------------------------------------------------------
# The five printed members
# ---------------------------------------------------------------------------

def build_base_pan():
    """Stationary base: Ø140 x 50 drum. The NEMA 17 hangs from the 4 mm top
    plate (inserted through the floor opening; body floats above the floor),
    stiffening ring outside the motor. Yaw bearing boss on top with driver
    notches over the motor screws."""
    p = Part('rt_base_pan')
    p.add(Cyl((0, 0, 0), (0, 0, PAN_H), PAN_D))
    hollow_top = PAN_H - PAN_PLATE_T - PAN_RING_T                       # 42
    p.add(Cyl((0, 0, PAN_H), (0, 0, BOSS_TOP), BOSS_OD))                # boss
    p.cut(Cyl((0, 0, PAN_FLOOR_T), (0, 0, hollow_top), PAN_D - 2 * PAN_WALL))
    p.cut(Cyl((0, 0, hollow_top), (0, 0, MOTOR_FACE_Z), PAN_RING_ID))   # ring bore
    half = (NEMA_SQ + 1.0) / 2
    p.cut(Box(-half, half, -half, half, -1, PAN_FLOOR_T + 1))           # floor opening
    p.cut(Cyl((0, 0, MOTOR_FACE_Z - 1), (0, 0, PAN_H + 1), NEMA_PILOT_D))
    h = NEMA_HOLE_SPACING / 2
    for dx, dy in ((h, h), (h, -h), (-h, h), (-h, -h)):
        p.cut(Cyl((dx, dy, MOTOR_FACE_Z - 1), (dx, dy, PAN_H + 1), M3_CLEAR_D))
        p.cut(Cyl((dx, dy, PAN_H - 0.1), (dx, dy, BOSS_TOP + 1), NOTCH_D))
    p.cut_bolt_circle_z(0, 0, PAN_MOUNT_BCD, 4, M3_CLEAR_D, -1, PAN_FLOOR_T + 1,
                        clock_deg=45)
    wz = PAN_FLOOR_T + WIRE_HOLE_D / 2 + 2
    p.cut(Cyl((PAN_D / 2 - PAN_WALL - 2, 0, wz), (PAN_D / 2 + 2, 0, wz),
              WIRE_HOLE_D))
    # boss interior: outer-race seat ledge, then the 6805 pocket
    p.cut(Cyl((0, 0, PAN_H - 0.1), (0, 0, YAW_BRG_Z0), BOSS_LEDGE_D))
    p.cut(Cyl((0, 0, YAW_BRG_Z0), (0, 0, BOSS_TOP + 1), YAW_POCKET_D))
    return p


def build_shaft_coupling():
    """Printed coupling, bottom to top: Ø25 journal (the 6805 slides on from
    this end), Ø28 body with the D-bore + split pinch (cross bolt ABOVE the
    bearing: tighten after the coupling is on the motor shaft), 4 x M3
    insert pockets in the top face for the column plate."""
    p = Part('rt_shaft_coupling')
    p.add(Cyl((0, 0, JOURNAL_Z0), (0, 0, JOURNAL_Z1), JOURNAL_D))
    p.add(Cyl((0, 0, JOURNAL_Z1), (0, 0, COUPLING_TOP), CLAMP_D))
    # D-bore: cut the round bore, then restore the flat as a sliver (any part
    # of the sliver outside the bore is already solid, so this is exact)
    p.cut(Cyl((0, 0, JOURNAL_Z0 - 1), (0, 0, COUPLING_TOP + 1), COUPLING_BORE_D))
    p.add(Box(COUPLING_FLAT_X, COUPLING_BORE_D / 2 + 0.5,
              -COUPLING_BORE_D / 2, COUPLING_BORE_D / 2,
              JOURNAL_Z0, COUPLING_TOP))
    # pinch: slit through +X (journal left solid), one M3 cross bolt with a
    # captured nut (-Y pocket, breaks out the side for insertion) and a
    # +Y spot face. z chosen clear of the insert pockets above.
    p.cut(Box(2.0, CLAMP_D / 2 + 1, -PINCH_SLIT_HALF, PINCH_SLIT_HALF,
              CLAMP_SLIT_Z0, COUPLING_TOP + 1))
    p.cut(Cyl((CLAMP_BOLT_X, -CLAMP_D / 2 - 1, CLAMP_BOLT_Z),
              (CLAMP_BOLT_X, CLAMP_D / 2 + 1, CLAMP_BOLT_Z), M3_CLEAR_D))
    p.cut(Box(CLAMP_BOLT_X - M3_NUT_AF / 2 - 0.1, CLAMP_BOLT_X + M3_NUT_AF / 2 + 0.1,
              -12.5, -9.2, CLAMP_BOLT_Z - 2.9, CLAMP_BOLT_Z + 2.9))
    p.cut(Cyl((CLAMP_BOLT_X, 8.5, CLAMP_BOLT_Z),
              (CLAMP_BOLT_X, CLAMP_D / 2 + 1, CLAMP_BOLT_Z), M3_CB_D))
    p.cut_bolt_circle_z(0, 0, COUPLING_BCD, 4, INSERT_D,
                        COUPLING_TOP - INSERT_POCKET, COUPLING_TOP + 1,
                        clock_deg=45)
    return p


def build_yaw_column(shoulder_z):
    """Theta-1 output: a low plate bolted down onto the coupling (everything
    near the yaw axis stays below the shoulder hub's swing circle), ear
    support boxes rising to the clevis at the shoulder axis, gearbox mounting
    face on the -Y ear disc."""
    p = Part('rt_yaw_column')
    p.add(Cyl((0, 0, COL_PLATE_Z0), (0, 0, COL_PLATE_TOP), COL_PLATE_D))
    for s in (+1, -1):
        p.add(Box(-EAR_SUPPORT_HALF_X, EAR_SUPPORT_HALF_X,
                  s * EAR_IN, s * EAR_OUT, COL_PLATE_Z0 + 1.5, shoulder_z))
    add_clevis(p, 0, shoulder_z)
    # coupling interface: shaft-tip recess + 4 x M3 counterbored through holes
    p.cut(Cyl((0, 0, COL_PLATE_Z0 - 1), (0, 0, SHAFT_TIP_RECESS_TOP),
              SHAFT_TIP_RECESS_D))
    p.cut_bolt_circle_z(0, 0, COUPLING_BCD, 4, M3_CLEAR_D,
                        COL_PLATE_Z0 - 1, COL_PLATE_TOP + 1, clock_deg=45)
    p.cut_bolt_circle_z(0, 0, COUPLING_BCD, 4, M3_CB_D,
                        COL_PLATE_TOP - M3_HEAD_H, COL_PLATE_TOP + 1,
                        clock_deg=45)
    cut_bearing_bores(p, 0, shoulder_z)
    cut_gearbox_mount(p, 0, shoulder_z)
    # yaw-boss relief: keep the ear disc clear of the stationary boss and the
    # motor screw heads through full yaw (see BOSS_RELIEF_* above)
    p.cut(Cyl((0, 0, 40), (0, 0, BOSS_RELIEF_TOP1), 2 * BOSS_RELIEF_R1))
    p.cut(Cyl((0, 0, 40), (0, 0, BOSS_RELIEF_TOP2), 2 * BOSS_RELIEF_R2))
    return p


def ua_stations(l_upper, theta3_range):
    """Derive the upper arm's beam hand-off stations from the forearm's swing
    wedge: the central beam must end where the folded forearm still clears
    it; the web bridges to the side beams just behind that."""
    sweep = max(abs(theta3_range[0]), abs(theta3_range[1])) + SWEEP_MARGIN_DEG
    need_beam = UA_BEAM_HALF_H + CLEAR
    need_web = UA_SIDE_HALF_H + CLEAR
    r = FA_BEAM_HALF_H + 1.0
    while swing_free_halfheight(r, sweep, FA_BEAM_HALF_H) < need_beam:
        r += 0.5
        if r > l_upper:
            raise ValueError('no clear hand-off station: link too short')
    beam_end = l_upper - math.ceil(r)
    web_x0 = beam_end - UA_WEB_LEN
    r_web = l_upper - beam_end
    assert swing_free_halfheight(r_web, sweep, FA_BEAM_HALF_H) >= need_web
    return beam_end, web_x0, sweep


def build_upper_arm(shoulder_z, l_upper, theta3_range):
    """Shoulder hub -> central beam -> full-width web -> twin side beams
    flanking the forearm's swing plane -> elbow clevis with the gearbox
    mounting face. The hand-off station is derived, not guessed."""
    beam_end, web_x0, _ = ua_stations(l_upper, theta3_range)
    p = Part('rt_upper_arm')
    add_hub(p, 0, shoulder_z, slit_toward=-1)
    p.add(Box(0, beam_end, -UA_BEAM_HALF_W, UA_BEAM_HALF_W,
              shoulder_z - UA_BEAM_HALF_H, shoulder_z + UA_BEAM_HALF_H))
    p.add(Box(web_x0, beam_end, -UA_SIDE_Y_OUT, UA_SIDE_Y_OUT,
              shoulder_z - UA_SIDE_HALF_H, shoulder_z + UA_SIDE_HALF_H))
    for s in (+1, -1):
        p.add(Box(web_x0, l_upper, s * EAR_IN, s * UA_SIDE_Y_OUT,
                  shoulder_z - UA_SIDE_HALF_H, shoulder_z + UA_SIDE_HALF_H))
    add_clevis(p, l_upper, shoulder_z)
    # re-cut the hub bore/slit region shadowed by the beam root: the beam box
    # starts at x=0 over the bore; re-cut bore after all adds
    p.cut(Cyl((0, -HUB_W / 2 - 1, shoulder_z), (0, HUB_W / 2 + 1, shoulder_z),
              SHAFT_BORE_D))
    cut_bearing_bores(p, l_upper, shoulder_z)
    cut_gearbox_mount(p, l_upper, shoulder_z)
    return p


def build_forearm(shoulder_z, l_upper, l_fore):
    """Elbow hub -> beam -> gripper interface plate (T8 flanged-nut pattern);
    the plate's outer face is the TCP plane."""
    ex, tcp = l_upper, l_upper + l_fore
    plate_x0 = tcp - FA_PLATE_T
    p = Part('rt_forearm')
    add_hub(p, ex, shoulder_z, slit_toward=-1)
    p.add(Box(ex, plate_x0 + 1, -FA_BEAM_HALF_W, FA_BEAM_HALF_W,
              shoulder_z - FA_BEAM_HALF_H, shoulder_z + FA_BEAM_HALF_H))
    p.add(Box(plate_x0, tcp, -FA_PLATE_HALF, FA_PLATE_HALF,
              shoulder_z - FA_PLATE_HALF, shoulder_z + FA_PLATE_HALF))
    p.cut(Cyl((ex, -HUB_W / 2 - 1, shoulder_z), (ex, HUB_W / 2 + 1, shoulder_z),
              SHAFT_BORE_D))
    p.cut(Cyl((plate_x0 - 1, 0, shoulder_z), (tcp + 1, 0, shoulder_z), T8_BORE_D))
    p.cut(Cyl((plate_x0 - 1, 0, shoulder_z), (plate_x0 + T8_FLANGE_T, 0, shoulder_z),
              T8_FLANGE_D))
    for i in range(4):
        a = math.radians(45) + i * math.pi / 2
        y = T8_BCD / 2 * math.cos(a)
        z = shoulder_z + T8_BCD / 2 * math.sin(a)
        p.cut(Cyl((plate_x0 - 1, y, z),
                  (plate_x0 + T8_FLANGE_T + INSERT_POCKET, y, z), INSERT_D))
    return p


# ---------------------------------------------------------------------------
# Hardware / keep-out solids (audit only; never printed or exported)
# ---------------------------------------------------------------------------

def base_motor_solid():
    p = Part('hw_base_motor')
    h = NEMA_SQ / 2 + 0.15
    p.add(Box(-h, h, -h, h, MOTOR_FACE_Z - NEMA_LEN_MAX, MOTOR_FACE_Z))
    p.add(Cyl((0, 0, MOTOR_FACE_Z), (0, 0, MOTOR_FACE_Z + NEMA_PILOT_H), 22.0))
    return p


def pan_screwhead_solids():
    p = Part('hw_pan_screws')
    h = NEMA_HOLE_SPACING / 2
    for dx, dy in ((h, h), (h, -h), (-h, h), (-h, -h)):
        p.add(Cyl((dx, dy, PAN_H), (dx, dy, PAN_H + M3_HEAD_H), M3_HEAD_D))
    return p


def coupling_hardware_solid():
    """Clamp cross-bolt + nut + head, rotating with the coupling."""
    p = Part('hw_clamp_bolt')
    p.add(Cyl((CLAMP_BOLT_X, -11.5, CLAMP_BOLT_Z),
              (CLAMP_BOLT_X, 12.0, CLAMP_BOLT_Z), 3.0))
    p.add(Box(CLAMP_BOLT_X - M3_NUT_AF / 2, CLAMP_BOLT_X + M3_NUT_AF / 2,
              -12.0, -9.4, CLAMP_BOLT_Z - 2.75, CLAMP_BOLT_Z + 2.75))
    p.add(Cyl((CLAMP_BOLT_X, 8.7, CLAMP_BOLT_Z),
              (CLAMP_BOLT_X, 11.7, CLAMP_BOLT_Z), M3_HEAD_D + 0.5))
    return p


def column_screwhead_solids():
    p = Part('hw_column_screws')
    for i in range(4):
        a = math.radians(45) + i * math.pi / 2
        x = COUPLING_BCD / 2 * math.cos(a)
        y = COUPLING_BCD / 2 * math.sin(a)
        p.add(Cyl((x, y, COL_PLATE_TOP - M3_HEAD_H), (x, y, COL_PLATE_TOP),
                  M3_HEAD_D))
    return p


def gearbox_keepout(jx, jz, tag):
    """Cartridge + flange + its screw heads + NEMA 17 hanging outboard -Y."""
    p = Part(f'ko_gearbox_{tag}')
    p.add(Cyl((jx, -EAR_OUT, jz), (jx, -EAR_OUT - GBX_BODY_L, jz), GBX_BODY_D))
    p.add(Cyl((jx, -EAR_OUT, jz), (jx, -EAR_OUT - GBX_FLANGE_T, jz), GBX_FLANGE_D))
    for i in range(GBX_BOLT_N):
        a = 2 * math.pi * i / GBX_BOLT_N
        x = jx + GBX_BCD / 2 * math.cos(a)
        z = jz + GBX_BCD / 2 * math.sin(a)
        p.add(Cyl((x, -EAR_OUT - GBX_FLANGE_T, z),
                  (x, -EAR_OUT - GBX_FLANGE_T - M3_HEAD_H, z), M3_HEAD_D))
    h = NEMA_SQ / 2 + 0.15
    y0 = -EAR_OUT - GBX_BODY_L
    p.add(Box(jx - h, jx + h, y0 - NEMA_LEN_MAX, y0, jz - h, jz + h))
    return p


def hub_pinch_hardware(jx, jz, slit_toward=-1):
    p = Part(f'hw_pinch_{jx:.0f}')
    bx = jx + slit_toward * PINCH_BOLT_X
    p.add(Cyl((bx, 0, jz - 7), (bx, 0, jz + 7.6), M3_CB_D))
    return p


# ---------------------------------------------------------------------------
# Design assembly + analytic clearance verification
# ---------------------------------------------------------------------------

def design(shoulder_z=None, l_upper=None, l_fore=None,
           theta2_range=None, theta3_range=None):
    """Build every part and keep-out, run the analytic checks. Returns a dict:
    parts, hardware, keepouts, checks, stations, config warnings."""
    if shoulder_z is None:
        shoulder_z, l_upper, l_fore, t2, t3, warnings = read_config()
        theta2_range = theta2_range or t2
        theta3_range = theta3_range or t3
    else:
        warnings = []
        theta2_range = theta2_range or DEFAULT_THETA2_RANGE
        theta3_range = theta3_range or DEFAULT_THETA3_RANGE
    sz, ex = shoulder_z, l_upper

    parts = {
        'rt_base_pan': build_base_pan(),
        'rt_shaft_coupling': build_shaft_coupling(),
        'rt_yaw_column': build_yaw_column(sz),
        'rt_upper_arm': build_upper_arm(sz, l_upper, theta3_range),
        'rt_forearm': build_forearm(sz, l_upper, l_fore),
    }
    beam_end, web_x0, sweep = ua_stations(l_upper, theta3_range)

    c = Checks()
    # --- base stack ---
    c.ck('motor body bottom above table', MOTOR_FACE_Z - NEMA_LEN_MAX, 2.0)
    c.ck('motor pilot top vs pan top plate', PAN_H - (MOTOR_FACE_Z + NEMA_PILOT_H), 0.0)
    c.ck('journal bottom vs pan plate top', JOURNAL_Z0 - PAN_H, 0.4)
    c.ck('6805 seat ledge width', (YAW_POCKET_D - BOSS_LEDGE_D) / 2, 2.0)
    c.ck('ledge bore vs spinning journal', BOSS_LEDGE_D / 2 - JOURNAL_D / 2, 1.0)
    c.ck('clamp presses inner race only', YAW_BRG_PRESS_R_MAX - CLAMP_D / 2, 0.5)
    c.ck('clamp body vs 6805 pocket wall', YAW_POCKET_D / 2 - CLAMP_D / 2, 1.0)
    c.ck('pocket depth vs bearing width', (BOSS_TOP - YAW_BRG_Z0) - YAW_BRG_W, 0.5)
    c.ck('clamp bolt hardware above boss', (CLAMP_BOLT_Z - 2.9) - BOSS_TOP, 0.5)
    c.ck('coupling top = column plate seat', COL_PLATE_Z0 - COUPLING_TOP, 0.0)
    c.ck('shaft tip inside column recess',
         SHAFT_TIP_RECESS_TOP - 0.5 - (MOTOR_FACE_Z + NEMA_SHAFT_LEN_MAX), 0.0)
    c.ck('recess roof thickness', COL_PLATE_TOP - SHAFT_TIP_RECESS_TOP, 2.0)
    bolt_to_pocket = (CLAMP_BOLT_X - M3_CLEAR_D / 2) - \
        (COUPLING_BCD / 2 * math.cos(math.radians(45)) + INSERT_D / 2)
    c.ck('clamp bolt hole vs insert pockets', bolt_to_pocket, 0.3)
    c.ck('nut pocket vs insert pockets (y)',
         9.2 - (COUPLING_BCD / 2 * math.sin(math.radians(45)) + INSERT_D / 2), 0.5)
    c.ck('counterbore vs shaft-tip recess',
         (COUPLING_BCD / 2 - M3_CB_D / 2) - SHAFT_TIP_RECESS_D / 2, 0.5)
    c.ck('slit clears insert pockets (y)',
         COUPLING_BCD / 2 * math.sin(math.radians(45)) - INSERT_D / 2
         - PINCH_SLIT_HALF, 0.5)

    # --- shoulder swing keep-outs (theta-2 full range) ---
    # everything on the column side near the yaw axis must sit below the
    # shoulder hub's swing circle; swept-below rule: a point swept 0..180 deg
    # never dips below its own starting depth
    c.ck('hub swing circle vs column plate top',
         (sz - HUB_KEEPOUT_R) - COL_PLATE_TOP, 1.0)
    c.ck('hub swing circle vs column screw heads',
         (sz - HUB_KEEPOUT_R) - COL_PLATE_TOP, 1.0)
    c.ck('UA central beam sweep vs column plate',
         (sz - UA_BEAM_HALF_H) - COL_PLATE_TOP, 1.0)
    c.ck('UA web/side-beam sweep vs column plate',
         (sz - UA_SIDE_HALF_H) - COL_PLATE_TOP, 1.0)
    # UA side-band material orbits the shoulder axis: keep it radially clear
    # of the column ear boxes (reach: box corner) and the gearbox cartridge
    box_reach = math.hypot(EAR_SUPPORT_HALF_X, sz - (COL_PLATE_Z0 + 1.5))
    side_r_min = math.hypot(web_x0, UA_SIDE_HALF_H)
    c.ck('UA side band vs column ear boxes', side_r_min - box_reach, 1.0)
    c.ck('UA side band vs shoulder gearbox body', side_r_min - GBX_BODY_D / 2, 1.0)
    c.ck('UA outer face vs gearbox flange plane', EAR_OUT - UA_SIDE_Y_OUT, 0.9)
    c.ck('gearbox flange sweep vs pan top', (sz - GBX_FLANGE_D / 2) - PAN_H, 1.5)
    c.ck('gearbox cartridge vs boss (radial)', EAR_OUT - BOSS_OD / 2, 1.5)
    # the shoulder ear disc orbits the yaw axis: its relief cuts must clear
    # the boss and screw heads, yet spare the insert pockets and flange seat
    c.ck('disc relief vs boss (radial)', BOSS_RELIEF_R1 - BOSS_OD / 2, 1.0)
    c.ck('disc relief vs boss (top)', BOSS_RELIEF_TOP1 - BOSS_TOP, 0.5)
    head_orbit_r = math.hypot(NEMA_HOLE_SPACING / 2, NEMA_HOLE_SPACING / 2) \
        + M3_HEAD_D / 2
    c.ck('disc relief vs motor screw heads (radial)',
         BOSS_RELIEF_R2 - head_orbit_r, 1.0)
    c.ck('disc relief vs motor screw heads (top)',
         BOSS_RELIEF_TOP2 - (PAN_H + M3_HEAD_H), 1.0)
    pocket_r_xy = math.inf
    for i in range(GBX_BOLT_N):
        a = 2 * math.pi * i / GBX_BOLT_N
        xa, za = GBX_BCD / 2 * math.cos(a), sz + GBX_BCD / 2 * math.sin(a)
        if za - INSERT_D / 2 < BOSS_RELIEF_TOP1:    # pocket dips into the relief zone
            pocket_r_xy = min(pocket_r_xy,
                              math.hypot(abs(xa) - INSERT_D / 2,
                                         EAR_OUT - INSERT_POCKET))
    c.ck('relief spares gearbox insert pockets', pocket_r_xy - BOSS_RELIEF_R1, 0.3)
    c.ck('relief spares gearbox flange seat', EAR_OUT - BOSS_RELIEF_R1, 1.5)

    # --- elbow fold (theta-3 full range) ---
    r_beam = ex - beam_end
    c.ck('folded forearm vs UA central beam end',
         swing_free_halfheight(r_beam, sweep, FA_BEAM_HALF_H) - UA_BEAM_HALF_H,
         CLEAR - 0.01)
    c.ck('folded forearm vs UA web',
         swing_free_halfheight(r_beam, sweep, FA_BEAM_HALF_H) - UA_SIDE_HALF_H,
         CLEAR - 0.01)
    c.ck('FA beam vs UA ear inner faces', EAR_IN - FA_BEAM_HALF_H, 1.0)
    c.ck('FA hub vs UA ear inner faces', EAR_IN - HUB_W / 2, 0.5)
    c.ck('FA hub swing vs UA beam end', r_beam - HUB_KEEPOUT_R, 1.0)
    # gripper plate swinging past the shoulder hub / central beam far end
    plate_r_min = l_fore - FA_PLATE_T
    c.ck('folded gripper plate vs shoulder hub (deg)',
         wedge_angle_clear_deg(ex - HUB_KEEPOUT_R, HUB_KEEPOUT_R,
                               sweep - SWEEP_MARGIN_DEG, FA_PLATE_HALF)
         if plate_r_min < ex + HUB_KEEPOUT_R else 90.0, 3.0)
    c.ck('folded gripper plate vs UA beam far end (deg)',
         wedge_angle_clear_deg(math.hypot(ex, UA_BEAM_HALF_H), UA_BEAM_HALF_H,
                               sweep - SWEEP_MARGIN_DEG, FA_PLATE_HALF), 3.0)

    # --- pitch joint internals ---
    c.ck('hub running gap to ears', JOINT_GAP, 0.5)
    c.ck('ear wall behind bearing pocket', EAR_T - BRG_W, 4.0)
    c.ck('retention lip width', (BRG_POCKET_D - BRG_RETAIN_D) / 2, 2.0)
    c.ck('gearbox insert ring inside disc',
         EAR_DISC_D / 2 - (GBX_BCD / 2 + INSERT_D / 2), 1.5)
    c.ck('gearbox inserts clear bearing pocket',
         (GBX_BCD / 2 - INSERT_D / 2) - BRG_POCKET_D / 2, 2.0)
    c.ck('pinch nut slot wall to shaft bore',
         (PINCH_BOLT_X - M3_NUT_AF / 2 - 0.1) - SHAFT_BORE_D / 2, 1.0)
    c.ck('pinch bolt hole wall to shaft bore',
         (PINCH_BOLT_X - M3_CLEAR_D / 2) - SHAFT_BORE_D / 2, 2.0)

    # --- printability ---
    for name, part in parts.items():
        lo, hi = part.bbox()
        dims = [hi[i] - lo[i] for i in range(3)]
        c.ck(f'{name} fits build volume', BUILD_VOLUME - max(dims), 5.0)

    hardware = {
        'hw_base_motor': base_motor_solid(),
        'hw_pan_screws': pan_screwhead_solids(),
        'hw_clamp_bolt': coupling_hardware_solid(),
        'hw_column_screws': column_screwhead_solids(),
        'hw_pinch_shoulder': hub_pinch_hardware(0, sz),
        'hw_pinch_elbow': hub_pinch_hardware(ex, sz),
    }
    keepouts = {
        'ko_gearbox_shoulder': gearbox_keepout(0, sz, 'shoulder'),
        'ko_gearbox_elbow': gearbox_keepout(ex, sz, 'elbow'),
    }
    return {
        'parts': parts,
        'hardware': hardware,
        'keepouts': keepouts,
        'checks': c,
        'shoulder_z': sz, 'l_upper': l_upper, 'l_fore': l_fore,
        'theta2_range': theta2_range, 'theta3_range': theta3_range,
        'beam_end': beam_end, 'web_x0': web_x0,
        'warnings': warnings,
    }


if __name__ == '__main__':
    d = design()
    print(f"links {d['l_upper']:.0f}/{d['l_fore']:.0f}, shoulder z "
          f"{d['shoulder_z']:.0f}; UA hand-off at x={d['web_x0']:.0f}"
          f"..{d['beam_end']:.0f}")
    print(d['checks'].report())
    fails = d['checks'].failures()
    print(f"\n{len(d['checks'].rows)} checks, {len(fails)} failed")
    raise SystemExit(1 if fails else 0)
