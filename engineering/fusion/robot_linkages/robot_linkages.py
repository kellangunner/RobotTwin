"""RobotTwin — rigid-linkage generator for Autodesk Fusion.

Generates the four printed structural members of the 3-DOF arm and exports
them to engineering/f3d, engineering/step, and engineering/stl:

    rt_base_pan        — stationary base: NEMA 17 mount + yaw bearing boss
    rt_shaft_coupling  — printed split-clamp hub: 5 mm D-shaft -> 6805 journal
    rt_yaw_column      — theta-1 output: bolts onto the coupling, shoulder clevis
    rt_upper_arm       — shoulder hub -> elbow clevis + elbow gearbox drum
    rt_forearm         — elbow hub -> gripper interface plate (T8 nut pattern)

Each part exports its own STEP and STL; the f3d archive holds the assembly.

Link lengths come from config/robot.yaml (single source of truth); all
hardware dimensions are named constants below. Cycloidal gearboxes are OUT OF
SCOPE here — each pitch joint reserves a cylindrical envelope (drum) for one,
plus a NEMA 17 mount on the drum's back wall. The base is DIRECT-DRIVEN by a
vertical NEMA 17 (no gearbox): the yaw axis fights no gravity torque.

Fasteners are M3 throughout; wherever parts bolt to a printed member, the
member carries Ø4.6 pockets for M3 heat-set inserts (screws into a motor's
own threads get plain clearance).

Load path (per docs/linkage-geometry.md): 608 bearings in the clevis ears
carry all structural loads; the 8 mm hardened shaft transmits torque from the
future gearbox output to the driven link's hub; shaft collars provide axial
retention. Gearboxes transmit torque only.

Run from Fusion: UTILITIES > ADD-INS > Scripts and Add-Ins > "+" > select this
folder > Run. The script builds in Direct Modeling mode (no timeline): it is a
generator — change a parameter here or in robot.yaml and re-run.
"""

import math
import os
import re
import traceback

try:
    import adsk.core
    import adsk.fusion
except ImportError:  # running outside Fusion (e.g. by accident from a shell)
    raise SystemExit(
        'This script must be run from inside Autodesk Fusion:\n'
        '  UTILITIES > ADD-INS > Scripts and Add-Ins > "+" > select this folder > Run'
    )

# ---------------------------------------------------------------------------
# Parameters. Geometry (FIXED, see docs/linkage-geometry.md) is read from
# config/robot.yaml at runtime; these are the fallbacks if the file is absent.
# All values in mm; converted to Fusion's internal cm at the API boundary.
# ---------------------------------------------------------------------------

DEFAULT_SHOULDER_Z = 90.0   # table -> shoulder pitch axis
DEFAULT_L_UPPER = 120.0     # shoulder axis -> elbow axis
DEFAULT_L_FORE = 120.0      # elbow axis -> TCP (gripper grasp point)

# 608 deep-groove bearing (8 x 22 x 7) — one per clevis ear
BRG_OD = 22.0
BRG_W = 7.0
BRG_FIT = 0.2               # diametral print allowance for a light press fit
BRG_POCKET_D = BRG_OD + BRG_FIT
BRG_RETAIN_D = 16.0         # inner shoulder: retains outer race, clears inner

# 8 mm hardened steel shaft — torque path from gearbox output to driven hub
SHAFT_D = 8.0
SHAFT_BORE_D = SHAFT_D + 0.4  # printed clearance bore in hubs

# 8 mm shaft collars (set-screw type, ~18 OD x 9 W) retain the pitch-joint
# shafts; they sit exposed outboard of the ears (no printed recess needed).

# T8 trapezoidal lead-screw nut (gripper actuation, mounts on forearm tip)
T8_BORE_D = 10.5            # nut body 10.2 + clearance
T8_FLANGE_D = 22.5          # nut flange 22 + clearance
T8_FLANGE_T = 3.6
T8_BCD = 16.0               # 4 x M3 flange holes -> heat-set inserts

# Fasteners: M3 everywhere. Where two printed/purchased parts join, the
# receiving printed part gets an M3 heat-set insert (pocket Ø4.6 x 6.5);
# screws into a motor's own threaded holes just get clearance.
M3_CLEAR_D = 3.4
INSERT_D = 4.6              # M3 x 5.7 heat-set insert pocket diameter
INSERT_POCKET = 6.5         # pocket depth (insert length 5.7 + melt room)

# NEMA 17 (42.3 sq x 48, 31 mm bolt square, 5 mm D-shaft)
NEMA_SQ = 42.3
NEMA_LEN = 48.0
NEMA_HOLE_SPACING = 31.0
NEMA_PILOT_D = 22.5         # 22 mm pilot boss + clearance
NEMA_SHAFT_LEN = 22.0       # usable shaft above the mounting face

# Printed split-clamp coupling (rt_shaft_coupling): clamps the motor's 5 mm
# D-shaft, carries the yaw support bearing on its Ø25 journal, and bolts to
# the column hub through its top flange (M3 screws from above into heat-set
# inserts in the flange). A standard part with its own STEP export.
COUPLING_BCD = 16.0
COUPLING_BORE_D = 5.2
COUPLING_FLAT_X = 2.05       # D-flat plane offset (NEMA 17 flat: 4.5 across)
COUPLING_CLAMP_D = 26.0
COUPLING_CLAMP_H = 9.5
COUPLING_FLANGE_D = 30.0     # covers the bearing inner race (r<=15), clears outer
COUPLING_FLANGE_T = 3.0
COUPLING_TIP_CLEAR_D = 10.0  # bore over the shaft tip inside the column hub

# Yaw support bearing: 6805-2RS (25 x 37 x 7) between the coupling journal
# and a boss on the pan. Takes the arm's weight and, paired with the motor's
# front bearing ~17 mm below, the overturning moment off the bare shaft.
YAW_BRG_ID = 25.0            # = coupling journal diameter
YAW_BRG_OD = 37.0
YAW_BRG_W = 7.0
YAW_POCKET_D = YAW_BRG_OD + 0.3
BOSS_OD = 44.0               # stays inside the shoulder drum's r=25.75 sweep
BOSS_ID = 34.0               # clears the rotating clamp body (Ø26)
BOSS_LEG_H = 5.0             # legs straddle the motor screw heads

# Reserved cycloidal gearbox envelope at the pitch joints. Sized so the drum
# (envelope + wall) swings clear of the base pan: drum radius 38 => lowest
# point z = SHOULDER_Z - 38 = 52 > PAN_H + 2 mm sweep clearance.
GBX_ENV_D = 66.0
GBX_ENV_L = 24.0
GBX_CLEAR = 1.0             # radial slip fit for the future gearbox housing
DRUM_ID = GBX_ENV_D + 2 * GBX_CLEAR      # 68
DRUM_WALL = 4.0
DRUM_OD = DRUM_ID + 2 * DRUM_WALL        # 76
DRUM_BACK_T = 5.0           # back wall carrying the NEMA 17 mount

# Driven-link hub: rides the 8 mm shaft between the clevis ears
HUB_D = 40.0
HUB_W = 26.0
FLANGE_BCD = 30.0           # bolt circle for a future shaft-clamp/output disc
FLANGE_N = 6

# Clevis ears
EAR_T = 12.0
JOINT_GAP = 0.75            # running clearance hub <-> ear inner face
EAR_IN = HUB_W / 2 + JOINT_GAP           # 13.75 (ear inner face |y|)
EAR_OUT = EAR_IN + EAR_T                 # 25.75 (ear outer face |y|)
EAR_BOSS_D = 46.0           # plain ear boss around the bearing
EAR_DISC_D = DRUM_OD        # drive-side ear grows to a full disc under the drum

# Beams (rectangular, printed flat; PETG ~30 % infill per the mass model)
BEAM_W = 24.0               # y extent +/-12: passes between the clevis ears
UA_BEAM_H = 28.0
FA_BEAM_H = 24.0

# Base pan (stationary): direct-drive base — the NEMA 17 stands vertically
# through a floor opening (body 0..48 resting on the table), face bolted to a
# 2 mm web under the top plate; a stiffening ring keeps the plate rigid. The
# 50 mm height is a hard ceiling: the shoulder gearbox drum (radius 38 about
# the 90 mm shoulder axis) sweeps 2 mm above the pan through full yaw.
PAN_D = 140.0
PAN_H = 50.0
PAN_WALL = 4.0
PAN_FLOOR_T = 3.0
PAN_WEB_T = 2.0             # thin web the motor face bolts against
PAN_RING_T = 6.0            # stiffening ring thickness (outside the motor)
PAN_RING_ID = 60.0          # ring clears the motor's 59.8 mm face diagonal
PAN_MOUNT_BCD = 120.0       # 4 x M3 table mount, through the floor
WIRE_HOLE_D = 12.0
# The motor face must land exactly on the underside of the web: the body
# spans table (z=0) to face (z=48) through the floor opening.
assert PAN_H - PAN_WEB_T == NEMA_LEN, 'pan height must equal motor length + web'

# Derived z-planes of the base joint stack (bottom to top): motor face, then
# the coupling's clamp / bearing journal / flange, then the column hub.
MOTOR_FACE_Z = PAN_H - PAN_WEB_T
CLAMP_Z0 = MOTOR_FACE_Z + 0.5          # running clearance over the web
JOURNAL_Z0 = CLAMP_Z0 + COUPLING_CLAMP_H
FLANGE_Z0 = JOURNAL_Z0 + YAW_BRG_W     # flange presses the inner race top
COUPLING_TOP = FLANGE_Z0 + COUPLING_FLANGE_T
assert JOURNAL_Z0 == PAN_H + BOSS_LEG_H + 3.0, 'bearing seat must meet journal'

# Yaw column hub: bolts onto the flange shaft hub at the motor shaft tip
COL_HUB_D = 68.0
COL_HUB_T = 12.0

BUILD_VOLUME = 180.0        # Bambu Lab A1 Mini, per config/robot.yaml


# ---------------------------------------------------------------------------
# Config loading — CAD dimensions originate from config/robot.yaml
# ---------------------------------------------------------------------------

def read_config_links():
    """Read link geometry from config/robot.yaml (repo root is two levels up
    from engineering/fusion/robot_linkages). Returns (shoulder_z, l_upper,
    l_fore, warnings)."""
    warnings = []
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, '..', '..', '..'))
    yaml_path = os.path.join(repo_root, 'config', 'robot.yaml')
    values = {
        'base_height_mm': DEFAULT_SHOULDER_Z,
        'upper_arm_mm': DEFAULT_L_UPPER,
        'forearm_mm': DEFAULT_L_FORE,
    }
    try:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            text = f.read()
        for key in values:
            m = re.search(r'^\s*' + key + r'\s*:\s*([0-9.]+)', text, re.MULTILINE)
            if m:
                values[key] = float(m.group(1))
            else:
                warnings.append(f'{key} not found in robot.yaml; using default {values[key]}')
    except OSError:
        warnings.append(f'Could not read {yaml_path}; using built-in defaults (90/120/120).')
    return values['base_height_mm'], values['upper_arm_mm'], values['forearm_mm'], warnings


# ---------------------------------------------------------------------------
# Temporary-BRep helpers. Everything is modeled in world coordinates (mm,
# Z up, X forward, arm at the reference pose along +X) with exact primitive
# booleans — no sketch-plane sign ambiguity, robust to re-runs.
# ---------------------------------------------------------------------------

def _p(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


class Builder:
    """Accumulates one solid body from primitive unions and subtractions."""

    def __init__(self):
        self.tbm = adsk.fusion.TemporaryBRepManager.get()
        self.body = None

    def _boolean(self, tool, op):
        if self.body is None:
            self.body = tool
        else:
            self.tbm.booleanOperation(self.body, tool, op)

    def add(self, tool):
        self._boolean(tool, adsk.fusion.BooleanTypes.UnionBooleanType)

    def cut(self, tool):
        assert self.body is not None, 'cut before any add'
        self.tbm.booleanOperation(self.body, tool,
                                  adsk.fusion.BooleanTypes.DifferenceBooleanType)

    # ---- primitives (mm, world coordinates) ----

    def cyl(self, p1, p2, dia):
        return self.tbm.createCylinderOrCone(_p(*p1), dia / 20.0, _p(*p2), dia / 20.0)

    def box(self, x0, x1, y0, y1, z0, z1):
        center = _p((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
        obb = adsk.core.OrientedBoundingBox3D.create(
            center,
            adsk.core.Vector3D.create(1, 0, 0),
            adsk.core.Vector3D.create(0, 1, 0),
            abs(x1 - x0) / 10.0, abs(y1 - y0) / 10.0, abs(z1 - z0) / 10.0)
        return self.tbm.createBox(obb)

    def cut_bolt_circle_z(self, cx, cy, bcd, n, dia, z0, z1, clock_deg=0.0):
        """n holes on a bolt circle about a Z axis, spanning z0..z1."""
        for i in range(n):
            a = math.radians(clock_deg) + 2 * math.pi * i / n
            x = cx + bcd / 2 * math.cos(a)
            y = cy + bcd / 2 * math.sin(a)
            self.cut(self.cyl((x, y, z0), (x, y, z1), dia))

    def cut_bolt_circle_y(self, cx, cz, bcd, n, dia, y0, y1, clock_deg=0.0):
        """n holes on a bolt circle about a Y axis, spanning y0..y1."""
        for i in range(n):
            a = math.radians(clock_deg) + 2 * math.pi * i / n
            x = cx + bcd / 2 * math.cos(a)
            z = cz + bcd / 2 * math.sin(a)
            self.cut(self.cyl((x, y0, z), (x, y1, z), dia))


# ---------------------------------------------------------------------------
# Shared joint features (pitch joints are identical at shoulder and elbow)
# ---------------------------------------------------------------------------

def add_clevis_bosses(b, jx, jz, drive_side=-1):
    """Ear bosses around a pitch-joint axis at (jx, ., jz). The drive-side ear
    is a full Ø76 disc so the gearbox drum lands on solid material."""
    for s in (+1, -1):
        d = EAR_DISC_D if s == drive_side else EAR_BOSS_D
        b.add(b.cyl((jx, s * EAR_IN, jz), (jx, s * EAR_OUT, jz), d))


def cut_bearing_bores(b, jx, jz):
    """608 pockets from each ear's outer face; Ø16 retention bore through the
    remaining wall keeps the outer race captive and clears the inner race."""
    b.cut(b.cyl((jx, -EAR_OUT, jz), (jx, EAR_OUT, jz), BRG_RETAIN_D))
    for s in (+1, -1):
        b.cut(b.cyl((jx, s * (EAR_OUT - BRG_W), jz), (jx, s * (EAR_OUT + 1), jz),
                    BRG_POCKET_D))


def add_gearbox_drum(b, jx, jz):
    """Reserved cycloidal envelope on the -Y ear: Ø68 x 25 interior drum with
    a 5 mm back wall carrying the NEMA 17 mount. The 8 mm shaft enters the
    drum interior; the future gearbox output couples to it there."""
    y_open = -EAR_OUT
    y_back0 = y_open - (GBX_ENV_L + GBX_CLEAR)
    y_back1 = y_back0 - DRUM_BACK_T
    b.add(b.cyl((jx, y_open, jz), (jx, y_back1, jz), DRUM_OD))
    b.cut(b.cyl((jx, y_open + 1, jz), (jx, y_back0, jz), DRUM_ID))
    # NEMA 17 mount on the back wall (motor hangs outboard along -Y; screws
    # thread into the motor's own holes, so plain M3 clearance)
    b.cut(b.cyl((jx, y_back0 + 1, jz), (jx, y_back1 - 1, jz), NEMA_PILOT_D))
    h = NEMA_HOLE_SPACING / 2
    for dx, dz in ((h, h), (h, -h), (-h, h), (-h, -h)):
        b.cut(b.cyl((jx + dx, y_back0 + 1, jz + dz), (jx + dx, y_back1 - 1, jz + dz),
                    M3_CLEAR_D))


def add_hub(b, jx, jz):
    """Driven-link hub: rides the shaft between the ears. Torque arrives via
    the shaft; the Ø4.6 flange holes take M3 heat-set inserts (from either
    face) for a future shaft-clamp disc."""
    b.add(b.cyl((jx, -HUB_W / 2, jz), (jx, HUB_W / 2, jz), HUB_D))
    b.cut(b.cyl((jx, -HUB_W / 2 - 1, jz), (jx, HUB_W / 2 + 1, jz), SHAFT_BORE_D))
    b.cut_bolt_circle_y(jx, jz, FLANGE_BCD, FLANGE_N, INSERT_D,
                        -HUB_W / 2 - 1, HUB_W / 2 + 1)


# ---------------------------------------------------------------------------
# The four rigid members
# ---------------------------------------------------------------------------

def build_base_pan(pan_h):
    """Stationary base: Ø140 x 50 drum, direct-drive. The NEMA 17 stands
    vertically through a square floor opening (body spans the full 0..48,
    bottom resting on the table) with its face bolted up against a 2 mm web
    under the top plate; a Ø60..132 stiffening ring keeps the plate rigid.
    Screw heads sit above the plate inside the drum's sweep circle."""
    b = Builder()
    web_z = pan_h - PAN_WEB_T                 # motor mounting face plane
    b.add(b.cyl((0, 0, 0), (0, 0, pan_h), PAN_D))
    # hollow the interior up to the stiffening ring
    b.cut(b.cyl((0, 0, PAN_FLOOR_T), (0, 0, pan_h - PAN_RING_T),
                PAN_D - 2 * PAN_WALL))
    # relieve the ring's center so only the 2 mm web spans the motor face
    # (the ring's Ø60 inner bore clears the motor's 59.8 mm face diagonal)
    b.cut(b.cyl((0, 0, pan_h - PAN_RING_T), (0, 0, web_z), PAN_RING_ID))
    # square floor opening: motor slides in from below, sits on the table
    half = (NEMA_SQ + 1.0) / 2
    b.cut(b.box(-half, half, -half, half, -1, PAN_FLOOR_T + 1))
    # motor face mount through the web (screws into the motor's threads,
    # heads on top at r=21.9 — inside the drum's r=25.75 sweep)
    b.cut(b.cyl((0, 0, web_z - 1), (0, 0, pan_h + 1), NEMA_PILOT_D))
    h = NEMA_HOLE_SPACING / 2
    for dx, dy in ((h, h), (h, -h), (-h, h), (-h, -h)):
        b.cut(b.cyl((dx, dy, web_z - 1), (dx, dy, pan_h + 1), M3_CLEAR_D))
    # table mount through the floor (M3, accessible before the motor goes in)
    b.cut_bolt_circle_z(0, 0, PAN_MOUNT_BCD, 4, M3_CLEAR_D, -1, PAN_FLOOR_T + 1,
                        clock_deg=45)
    # wire exit through the side wall
    b.cut(b.cyl((PAN_D / 2 - PAN_WALL - 2, 0, PAN_FLOOR_T + WIRE_HOLE_D / 2 + 2),
                (PAN_D / 2 + 2, 0, PAN_FLOOR_T + WIRE_HOLE_D / 2 + 2), WIRE_HOLE_D))

    # --- yaw support bearing boss ---
    # Four legs at the cardinal azimuths (the motor screws sit at 45°) carry a
    # ring with the 6805 pocket. Bearing loads pass legs -> 2 mm web -> the
    # motor's steel face in compression, straight down the body to the table.
    ring_z0 = pan_h + BOSS_LEG_H
    pocket_top = JOURNAL_Z0 + YAW_BRG_W + 0.4
    for x0, x1, y0, y1 in ((16, 24, -5, 5), (-24, -16, -5, 5),
                           (-5, 5, 16, 24), (-5, 5, -24, -16)):
        b.add(b.box(x0, x1, y0, y1, pan_h, ring_z0))
    b.add(b.cyl((0, 0, ring_z0), (0, 0, pocket_top), BOSS_OD))
    # interior: clearance over the rotating clamp, then the bearing pocket
    # (the ID step at JOURNAL_Z0 is the shoulder the outer race sits on)
    b.cut(b.cyl((0, 0, ring_z0 - 0.1), (0, 0, JOURNAL_Z0), BOSS_ID))
    b.cut(b.cyl((0, 0, JOURNAL_Z0), (0, 0, pocket_top + 1), YAW_POCKET_D))
    # driver-access notches over the four motor screws (the bearing's outer
    # race still seats on the four arcs left between them)
    h = NEMA_HOLE_SPACING / 2
    for dx, dy in ((h, h), (h, -h), (-h, h), (-h, -h)):
        b.cut(b.cyl((dx, dy, pan_h - 0.1), (dx, dy, pocket_top + 1), 9.0))
    return b.body


def build_shaft_coupling():
    """rt_shaft_coupling — printable standard part replacing a set-screw
    flange hub. Bottom-up: split clamp on the motor's 5 mm D-shaft (slit +
    two M3 cross-bolts with captured nuts), Ø25 journal the 6805 rides on,
    Ø30 flange with 4 heat-set inserts the column bolts down into. The D-bore
    gives positive torque drive; the clamp carries none of the arm's weight —
    that goes flange -> bearing -> pan boss."""
    b = Builder()
    b.add(b.cyl((0, 0, CLAMP_Z0), (0, 0, JOURNAL_Z0), COUPLING_CLAMP_D))
    b.add(b.cyl((0, 0, JOURNAL_Z0), (0, 0, FLANGE_Z0), YAW_BRG_ID))
    b.add(b.cyl((0, 0, FLANGE_Z0), (0, 0, COUPLING_TOP), COUPLING_FLANGE_D))
    # D-bore: round bore minus the flat — shape the tool before cutting
    bore = b.cyl((0, 0, CLAMP_Z0 - 1), (0, 0, COUPLING_TOP + 1), COUPLING_BORE_D)
    flat = b.box(COUPLING_FLAT_X, COUPLING_BORE_D, -COUPLING_BORE_D,
                 COUPLING_BORE_D, CLAMP_Z0 - 1, COUPLING_TOP + 1)
    b.tbm.booleanOperation(bore, flat,
                           adsk.fusion.BooleanTypes.DifferenceBooleanType)
    b.cut(bore)
    # clamp slit through the +X side of the clamp section only
    b.cut(b.box(0, COUPLING_CLAMP_D, -1, 1, CLAMP_Z0 - 1, JOURNAL_Z0 - 0.5))
    # two M3 cross-bolts squeeze the slit: clearance hole through, captured
    # square nut pocket on -Y, spot-face for the head on +Y
    for dz in (3.0, 7.0):
        z = CLAMP_Z0 + dz
        b.cut(b.cyl((7.5, -14, z), (7.5, 14, z), M3_CLEAR_D))
        b.cut(b.box(7.5 - 3.2, 7.5 + 3.2, -11, -7, z - 2.9, z + 2.9))
        b.cut(b.cyl((7.5, 9.5, z), (7.5, 14, z), 6.5))
    # column-hub inserts in the flange top (pockets reach into the journal)
    b.cut_bolt_circle_z(0, 0, COUPLING_BCD, 4, INSERT_D,
                        COUPLING_TOP - INSERT_POCKET, COUPLING_TOP + 1,
                        clock_deg=45)
    return b.body


def build_yaw_column(pan_h, shoulder_z):  # noqa: ARG001 — stack derives from constants
    """Theta-1 output, direct-driven: the hub bolts down onto the printed
    split-clamp coupling (M3 screws from above, through counterbored holes,
    into the coupling's heat-set inserts). The 6805 under the flange carries
    the arm's weight and moment. Clevis ears rise to the shoulder axis; the
    reserved shoulder gearbox drum hangs on the -Y ear."""
    b = Builder()
    hub_z0 = COUPLING_TOP
    hub_z1 = hub_z0 + COL_HUB_T
    ear_z0 = hub_z1 - 6.5                      # ears root into the hub disc
    b.add(b.cyl((0, 0, hub_z0), (0, 0, hub_z1), COL_HUB_D))
    for s in (+1, -1):
        b.add(b.box(-EAR_BOSS_D / 2, EAR_BOSS_D / 2,
                    s * EAR_IN, s * EAR_OUT, ear_z0, shoulder_z))
    add_clevis_bosses(b, 0, shoulder_z, drive_side=-1)
    # coupling interface: shaft-tip clearance + 4 x M3 counterbored through
    # holes (heads accessible from above, between the ears)
    b.cut(b.cyl((0, 0, hub_z0 - 1), (0, 0, hub_z0 + 6), COUPLING_TIP_CLEAR_D))
    b.cut_bolt_circle_z(0, 0, COUPLING_BCD, 4, M3_CLEAR_D,
                        hub_z0 - 1, hub_z1 + 1, clock_deg=45)
    b.cut_bolt_circle_z(0, 0, COUPLING_BCD, 4, 6.5,
                        hub_z1 - 4, hub_z1 + 1, clock_deg=45)
    cut_bearing_bores(b, 0, shoulder_z)
    add_gearbox_drum(b, 0, shoulder_z)
    return b.body


def build_upper_arm(shoulder_z, l_upper):
    """Shoulder hub -> beam -> elbow clevis with the elbow gearbox drum.
    The beam stops 20.75 mm short of the elbow axis and bridges to the ears
    through a full-width block, keeping the forearm hub's swing volume clear
    through the full ±150° elbow range."""
    b = Builder()
    elbow_x = l_upper
    beam_end = elbow_x - HUB_D / 2 - JOINT_GAP          # 99.25
    add_hub(b, 0, shoulder_z)
    b.add(b.box(0, beam_end, -BEAM_W / 2, BEAM_W / 2,
                shoulder_z - UA_BEAM_H / 2, shoulder_z + UA_BEAM_H / 2))
    # bridge block: full clevis width; z-trimmed to ±10 so the folding forearm
    # beam clears it out to ±154° (limits are ±150°)
    b.add(b.box(beam_end - 12, beam_end, -EAR_OUT, EAR_OUT,
                shoulder_z - 10, shoulder_z + 10))
    add_clevis_bosses(b, elbow_x, shoulder_z, drive_side=-1)
    cut_bearing_bores(b, elbow_x, shoulder_z)
    add_gearbox_drum(b, elbow_x, shoulder_z)
    return b.body


def build_forearm(shoulder_z, l_upper, l_fore):
    """Elbow hub -> beam -> gripper interface plate. The plate's outer face is
    the TCP plane; a T8 lead-screw gripper bolts onto the flanged-nut pattern
    (nut inserted from the inner side, screw axis along +X)."""
    b = Builder()
    elbow_x = l_upper
    tcp_x = l_upper + l_fore
    plate_t = 12.0
    plate_x0 = tcp_x - plate_t
    add_hub(b, elbow_x, shoulder_z)
    b.add(b.box(elbow_x, plate_x0 + 1, -BEAM_W / 2, BEAM_W / 2,
                shoulder_z - FA_BEAM_H / 2, shoulder_z + FA_BEAM_H / 2))
    b.add(b.box(plate_x0, tcp_x, -20, 20, shoulder_z - 20, shoulder_z + 20))
    # T8 nut interface on the plate, screw axis along X at the TCP centerline.
    # The nut sits in the flange recess on the inner face; its M3 screws
    # thread into blind heat-set inserts behind the recess floor.
    b.cut(b.cyl((plate_x0 - 1, 0, shoulder_z), (tcp_x + 1, 0, shoulder_z), T8_BORE_D))
    b.cut(b.cyl((plate_x0 - 1, 0, shoulder_z), (plate_x0 + T8_FLANGE_T, 0, shoulder_z),
                T8_FLANGE_D))
    for i in range(4):
        a = math.radians(45) + i * math.pi / 2
        y = T8_BCD / 2 * math.cos(a)
        z = shoulder_z + T8_BCD / 2 * math.sin(a)
        b.cut(b.cyl((plate_x0 - 1, y, z),
                    (plate_x0 + T8_FLANGE_T + INSERT_POCKET, y, z), INSERT_D))
    return b.body


# ---------------------------------------------------------------------------
# Fusion document assembly + export
# ---------------------------------------------------------------------------

def run(context):  # noqa: ARG001 — Fusion entry point signature
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        shoulder_z, l_upper, l_fore, warnings = read_config_links()

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        # Direct modeling: this script is the parametric layer; re-run to rebuild.
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        # NB: the root component name follows the document name and cannot be
        # set on an unsaved document; part names live on the child components.
        root = design.rootComponent

        parts = {
            'rt_base_pan': build_base_pan(PAN_H),
            'rt_shaft_coupling': build_shaft_coupling(),
            'rt_yaw_column': build_yaw_column(PAN_H, shoulder_z),
            'rt_upper_arm': build_upper_arm(shoulder_z, l_upper),
            'rt_forearm': build_forearm(shoulder_z, l_upper, l_fore),
        }

        occurrences = {}
        for name, body in parts.items():
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = name
            occ.component.bRepBodies.add(body)
            occurrences[name] = occ

        # printability check against the A1 Mini build volume
        for name, occ in occurrences.items():
            bb = occ.component.bRepBodies.item(0).boundingBox
            dims = [(bb.maxPoint.x - bb.minPoint.x) * 10,
                    (bb.maxPoint.y - bb.minPoint.y) * 10,
                    (bb.maxPoint.z - bb.minPoint.z) * 10]
            if max(dims) > BUILD_VOLUME:
                warnings.append(
                    f'{name} bounding box {dims[0]:.0f} x {dims[1]:.0f} x '
                    f'{dims[2]:.0f} mm exceeds the {BUILD_VOLUME:.0f} mm build volume')

        # exports — engineering/{f3d,step,stl}
        script_dir = os.path.dirname(os.path.abspath(__file__))
        eng_dir = os.path.abspath(os.path.join(script_dir, '..', '..'))
        exported, failed = [], []
        mgr = design.exportManager

        def try_export(label, options):
            try:
                mgr.execute(options)
                exported.append(label)
            except Exception:
                failed.append(f'{label}: {traceback.format_exc(limit=1)}')

        f3d_path = os.path.join(eng_dir, 'f3d', 'rt-arm-3dof_linkages.f3d')
        try_export(os.path.basename(f3d_path),
                   mgr.createFusionArchiveExportOptions(f3d_path))

        # one STEP and one STL per part — no combined jumble
        for name, occ in occurrences.items():
            step_path = os.path.join(eng_dir, 'step', f'{name}.step')
            try_export(os.path.basename(step_path),
                       mgr.createSTEPExportOptions(step_path, occ.component))

            stl_path = os.path.join(eng_dir, 'stl', f'{name}.stl')
            opts = mgr.createSTLExportOptions(occ, stl_path)
            opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            try_export(os.path.basename(stl_path), opts)

        app.activeViewport.fit()

        lines = [
            f'Built 4 linkage members (shoulder axis z = {shoulder_z:.0f} mm, '
            f'links {l_upper:.0f} / {l_fore:.0f} mm).',
            '',
            'Exported: ' + (', '.join(exported) if exported else 'none'),
        ]
        if failed:
            lines += ['', 'FAILED exports:'] + failed
        if warnings:
            lines += ['', 'Warnings:'] + [f'- {w}' for w in warnings]
        ui.messageBox('\n'.join(lines), 'RobotTwin linkage generator')

    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
