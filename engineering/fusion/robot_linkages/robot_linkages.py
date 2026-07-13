"""RobotTwin — rigid-linkage generator for Autodesk Fusion.

Generates the four printed structural members of the 3-DOF arm and exports
them to engineering/f3d, engineering/step, and engineering/stl:

    rt_base_pan     — stationary base housing (reserves the base drive bay)
    rt_yaw_column   — theta-1 output: yaw hub + shoulder clevis + gearbox drum
    rt_upper_arm    — shoulder hub -> elbow clevis + elbow gearbox drum
    rt_forearm      — elbow hub -> gripper interface plate (T8 nut pattern)

Link lengths come from config/robot.yaml (single source of truth); all
hardware dimensions are named constants below. Cycloidal gearboxes are OUT OF
SCOPE here — each joint reserves a cylindrical envelope (drum) for one, plus a
NEMA 17 mount on the drum's back wall.

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

# 8 mm shaft collar (set-screw type, ~18 OD x 9 W) — axial retention
COLLAR_RECESS_D = 20.0
COLLAR_RECESS_DEPTH = 9.5

# T8 trapezoidal lead-screw nut (gripper actuation, mounts on forearm tip)
T8_BORE_D = 10.5            # nut body 10.2 + clearance
T8_FLANGE_D = 22.5          # nut flange 22 + clearance
T8_FLANGE_T = 3.6
T8_BCD = 16.0               # 4 x M3 flange holes
T8_HOLE_D = 3.5

# NEMA 17 (42.3 sq, 31 mm bolt square) — mounts on each gearbox drum back wall
NEMA_HOLE_SPACING = 31.0
NEMA_HOLE_D = 3.2           # M3 clearance
NEMA_PILOT_D = 22.5         # 22 mm pilot boss + clearance

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
FLANGE_HOLE_D = 3.2
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

# Base pan (stationary): reserves the base drive bay. The base motor +
# gearbox arrangement (belt/coax) is part of the gearbox design = out of
# scope; the pan provides the volume, a Ø68 output opening and a bolt circle.
PAN_D = 140.0
PAN_H = 50.0
PAN_WALL = 4.0
PAN_FLOOR_T = 3.0
PAN_TOP_T = 3.0
PAN_OPEN_D = DRUM_ID        # base drive cartridge output opening
PAN_TOP_BCD = 78.0          # 6 x M3: future cartridge hangs from the top plate
PAN_TOP_N = 6
PAN_MOUNT_BCD = 120.0       # 4 x M4 table mount, through the floor
PAN_MOUNT_D = 4.5
WIRE_HOLE_D = 12.0

# Yaw column hub (theta-1 output disc, sits 0.5 mm above the pan top plate)
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
    # NEMA 17 mount on the back wall (motor hangs outboard along -Y)
    b.cut(b.cyl((jx, y_back0 + 1, jz), (jx, y_back1 - 1, jz), NEMA_PILOT_D))
    h = NEMA_HOLE_SPACING / 2
    for dx, dz in ((h, h), (h, -h), (-h, h), (-h, -h)):
        b.cut(b.cyl((jx + dx, y_back0 + 1, jz + dz), (jx + dx, y_back1 - 1, jz + dz),
                    NEMA_HOLE_D))


def add_hub(b, jx, jz):
    """Driven-link hub: rides the shaft between the ears. Torque arrives via
    the shaft (collar-clamped, flange holes for a future clamp disc)."""
    b.add(b.cyl((jx, -HUB_W / 2, jz), (jx, HUB_W / 2, jz), HUB_D))
    b.cut(b.cyl((jx, -HUB_W / 2 - 1, jz), (jx, HUB_W / 2 + 1, jz), SHAFT_BORE_D))
    b.cut_bolt_circle_y(jx, jz, FLANGE_BCD, FLANGE_N, FLANGE_HOLE_D,
                        -HUB_W / 2 - 1, HUB_W / 2 + 1)


# ---------------------------------------------------------------------------
# The four rigid members
# ---------------------------------------------------------------------------

def build_base_pan(pan_h):
    """Stationary base: Ø140 x 50 drum. Hollow interior is the reserved base
    drive bay (fits a NEMA 17 lying flat plus a Ø66 cycloidal cartridge); the
    future cartridge hangs from the top plate on the Ø78 bolt circle and its
    output reaches the yaw column through the Ø68 opening."""
    b = Builder()
    b.add(b.cyl((0, 0, 0), (0, 0, pan_h), PAN_D))
    # hollow the drive bay between floor and top plate
    b.cut(b.cyl((0, 0, PAN_FLOOR_T), (0, 0, pan_h - PAN_TOP_T), PAN_D - 2 * PAN_WALL))
    # output opening + cartridge bolt circle in the top plate
    b.cut(b.cyl((0, 0, pan_h - PAN_TOP_T - 1), (0, 0, pan_h + 1), PAN_OPEN_D))
    b.cut_bolt_circle_z(0, 0, PAN_TOP_BCD, PAN_TOP_N, NEMA_HOLE_D,
                        pan_h - PAN_TOP_T - 1, pan_h + 1)
    # table mount through the floor (accessible before cartridge install)
    b.cut_bolt_circle_z(0, 0, PAN_MOUNT_BCD, 4, PAN_MOUNT_D, -1, PAN_FLOOR_T + 1,
                        clock_deg=45)
    # wire exit through the side wall
    b.cut(b.cyl((PAN_D / 2 - PAN_WALL - 2, 0, PAN_FLOOR_T + WIRE_HOLE_D / 2 + 2),
                (PAN_D / 2 + 2, 0, PAN_FLOOR_T + WIRE_HOLE_D / 2 + 2), WIRE_HOLE_D))
    return b.body


def build_yaw_column(pan_h, shoulder_z):
    """Theta-1 output: hub disc over the pan opening, clevis ears up to the
    shoulder axis, and the reserved shoulder gearbox drum on the -Y ear."""
    b = Builder()
    hub_z0 = pan_h + 0.5                       # running clearance over the pan
    hub_z1 = hub_z0 + COL_HUB_T
    ear_z0 = hub_z1 - 6.5                      # ears root into the hub disc
    b.add(b.cyl((0, 0, hub_z0), (0, 0, hub_z1), COL_HUB_D))
    for s in (+1, -1):
        b.add(b.box(-EAR_BOSS_D / 2, EAR_BOSS_D / 2,
                    s * EAR_IN, s * EAR_OUT, ear_z0, shoulder_z))
    add_clevis_bosses(b, 0, shoulder_z, drive_side=-1)
    # base coupling: center bore + collar recess (axial retention on an 8 mm
    # stub from the base cartridge) + output flange bolt circle
    b.cut(b.cyl((0, 0, hub_z0 - 1), (0, 0, hub_z1 + 1), SHAFT_BORE_D))
    b.cut(b.cyl((0, 0, hub_z1 - COLLAR_RECESS_DEPTH), (0, 0, hub_z1 + 1),
                COLLAR_RECESS_D))
    b.cut_bolt_circle_z(0, 0, FLANGE_BCD, FLANGE_N, FLANGE_HOLE_D,
                        hub_z0 - 1, hub_z1 + 1, clock_deg=30)
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
    # T8 nut interface on the plate, screw axis along X at the TCP centerline
    b.cut(b.cyl((plate_x0 - 1, 0, shoulder_z), (tcp_x + 1, 0, shoulder_z), T8_BORE_D))
    b.cut(b.cyl((plate_x0 - 1, 0, shoulder_z), (plate_x0 + T8_FLANGE_T, 0, shoulder_z),
                T8_FLANGE_D))
    for i in range(4):
        a = math.radians(45) + i * math.pi / 2
        y = T8_BCD / 2 * math.cos(a)
        z = shoulder_z + T8_BCD / 2 * math.sin(a)
        b.cut(b.cyl((plate_x0 - 1, y, z), (tcp_x + 1, y, z), T8_HOLE_D))
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
        root = design.rootComponent
        root.name = 'rt-arm-3dof linkages'

        parts = {
            'rt_base_pan': build_base_pan(PAN_H),
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

        step_path = os.path.join(eng_dir, 'step', 'rt-arm-3dof_linkages.step')
        try_export(os.path.basename(step_path),
                   mgr.createSTEPExportOptions(step_path))

        for name, occ in occurrences.items():
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
