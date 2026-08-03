"""RobotTwin — cycloidal gearbox generator for Autodesk Fusion (v3, pinned
two-part casing).

Generates two single-stage cycloidal gearbox cartridges — 15:1 and 20:1 —
driven entirely by engineering/gearboxes/gear_parameters.yaml, and exports
them per part to engineering/gearboxes/15_1/ and 20_1/:

    rt_cyc<N>_cup     — back casing half: motor plate + Ø76 outer wall.
                        The NEMA 17 bolts to its back (pilot recessed).
    rt_cyc<N>_lid     — front casing half: Ø76 annular flange whose face
                        seats on the linkage ear disc; the output's
                        extension passes out through its face bore.
    rt_cyc<N>_disc    — cycloid disc (x2 per gearbox, cam lobes 180° apart)
    rt_cyc<N>_cam     — eccentric input cam on the motor's 5 mm D-shaft
    rt_cyc<N>_output  — output flange: six Ø6 x 25 steel dowels reach back
                        through the discs; the Ø22 hub continues past the
                        lid face as the SHAFT-CLAMP EXTENSION (live-shaft
                        v5): a Ø25.05 seat for the drive ear's 6805, then
                        a Ø24.6 split pinch that clamps the 8 mm shaft.

v3 design intent (after the v2 print failed): SIMPLE. The casing is two
flat, support-free prints held together — and registered — by the Zp ring
dowels themselves: each Ø6 x 25 pin runs through the entire cup (press
holes, flush at the back face) and 6.5 mm into a blind hole in the lid, so
every pin is loaded in double shear and is fully captive once the halves
meet. Six M3 x 40 screws from the gearbox back clamp cup + lid + ear disc
in one stack (they are both the case closure and the mounting), which is
also why the casing can be a plain Ø76 cylinder: no driver has to sneak
past the Ø66 bolt circle any more, so the v2 necks, grooves, slits, and
pinch-access tunnels are all gone.

The discs get a Ø62.5 interior — over 4 mm of radial air to the box; the
mesh happens ONLY on the free-standing pins, and the hobbed profile now
carries 0.25 mm radial print clearance (yaml-tunable — the 0.15 of v2
printed too tight). python/validate_cycloid.py additionally sweeps the
assembled mesh through a full cam revolution and reports the running
clearance so a jam is caught numerically, not at the printer.

Interface to the linkage (live-shaft v5, matching the pinch-hub arms): Ø76
ear disc face, 6 x M3 heat-set inserts on a Ø66 bolt circle. The 8 mm
joint shaft IS the torque path: it slips into the output's Ø8.35 bore
(reaching inside the casing to z 28) and the extension's split pinch
clamps it; at the far end the arm hub's own pinch grabs it, so torque
flows output -> shaft -> arm. The extension carries a Ø25.05 seat for the
6805 bearing housed in the drive ear — that bearing, plus the 608 in the
+Y ear, carries the structural joint loads. The lid's face bore is pure
pass-through (Ø = seat + 0.8, so the extension assembles through it — the
old Ø23 hub journal could never pass the Ø25.05 seat and is gone). The
shaft stays smooth (no flat): both couplings are press bores closed by
pinch bolts.

Z axis = gearbox axis; z = 0 is the back face the NEMA 17 bolts to;
z = BODY_L is the lid face that seats on the linkage ear disc.

Run from Fusion: UTILITIES > ADD-INS > Scripts and Add-Ins > "+" > select
this folder > Run. Direct modeling, no timeline: edit the yaml and re-run.
The hobbing cuts take a minute or two — Fusion is not hung.

Run from a shell (no Fusion) for a dry run of the parameters and the
derived stack:  python cycloidal_gearbox.py
"""

import math
import os
import re
import traceback

try:
    import adsk.core
    import adsk.fusion
    IN_FUSION = True
except ImportError:
    IN_FUSION = False

# ---------------------------------------------------------------------------
# Standard purchased hardware and print constants (mm). Everything specific
# to THIS gearbox design lives in gear_parameters.yaml, not here.
# ---------------------------------------------------------------------------

# NEMA 17 (17HS4401), mounted directly on the cup's back plate
NEMA_HOLE_SPACING = 31.0
NEMA_PILOT_D = 22.5             # pilot boss + clearance
NEMA_PILOT_H = 2.2              # recess depth (boss is 2.0)
NEMA_BORE_D = 14.0              # shaft + boss shoulder clearance through the plate
MOTOR_SHAFT_D = 5.0             # D-shaft: Ø5, flat leaves 4.5 (plane at x = 2.0)
MOTOR_FLAT_X0 = 2.0             # cam D-bore fit is yaml (motor_*_clearance)
NEMA_SHAFT_LEN_MAX = 25.0       # above the mounting face (measured 23.5)

# 6802-2RS (15 x 24 x 5), one per disc, on the eccentric cam journals
ECC_BRG_BORE = 15.0
CAM_JOURNAL_D = ECC_BRG_BORE + 0.05     # light press after printing

# Linkage-side joint contract (mirror of the constants in
# engineering/fusion/robot_linkages/linkage_geometry.py — keep in sync).
# The output extension crosses the drive ear (through its 6805) and ends in
# the split pinch that clamps the live shaft, one running gap shy of the
# arm hub's face.
EAR_T = 12.0                    # drive-ear thickness (6805 pocket + shoulder)
JOINT_GAP = 0.75                # running gap each side of the pinch section

# 6805-2RS (25 x 37 x 7): drive-ear support bearing on the extension's seat
EAR_BRG_BORE = 25.0
EAR_BRG_W = 7.0

# M3 fasteners
M3_CLEAR_D = 3.4
M3_NUT_AF = 5.5                 # across flats
M3_NUT_T = 2.4
M3_CB_D = 6.0                   # socket-head counterbore (5.5 head): kept tight
                                # because the ring-pin holes pass close by at R 28
M3_CB_FLOOR = 1.5               # web left under the motor screw heads
M3_HEAD_D = 5.5
M3_HEAD_H = 3.0
MOUNT_CB_D = 6.5                # mount-screw counterbore in the cup back
MOUNT_CB_T = 2.0                # its depth (sets insert engagement ~6 mm)

GAP = 0.5                       # default axial running gap
MIN_WALL = 1.6                  # printable-wall floor (4 perimeters at 0.4 mm)
HOB_STEPS_PER_LOBE = 32         # cutter positions per lobe (envelope err < 0.01)
BUILD_VOLUME = 180.0            # Bambu Lab A1 Mini


# ---------------------------------------------------------------------------
# Parameters — engineering/gearboxes/gear_parameters.yaml is the only source.
# Minimal two-level 'Section: / key: number' parser (Fusion has no PyYAML).
# ---------------------------------------------------------------------------

def yaml_path():
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(
        here, '..', '..', 'gearboxes', 'gear_parameters.yaml'))


def load_params():
    """Returns ({ratio: {key: value}}, {hardware key: value})."""
    with open(yaml_path(), 'r', encoding='utf-8') as f:
        lines = f.read().splitlines()
    data, section = {}, None
    for ln in lines:
        ln = ln.split('#', 1)[0].rstrip()
        if not ln.strip():
            continue
        m = re.match(r'^(\S[^:]*):\s*$', ln)
        if m:
            section = m.group(1)
            data[section] = {}
            continue
        m = re.match(r'^\s+([A-Za-z_]\w*)\s*:\s*(-?[0-9.]+)\s*$', ln)
        if m and section is not None:
            data[section][m.group(1)] = float(m.group(2))
    gears = {}
    for name, vals in data.items():
        m = re.match(r'^(\d+)-1_Gear$', name)
        if m:
            gears[int(m.group(1))] = vals
    hw = data.get('hardware', {})
    if not gears or not hw:
        raise ValueError(f'no gear/hardware sections parsed from {yaml_path()}')
    return gears, hw


# ---------------------------------------------------------------------------
# Derived geometry. One Gearbox object per ratio; everything below is
# computed from the yaml, then self-checked analytically.
# ---------------------------------------------------------------------------

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


class Gearbox:
    """All derived dimensions for one ratio (mm, gearbox-local z)."""

    def __init__(self, ratio, g, hw):
        self.ratio = ratio
        self.zp = int(g['external_pins'])
        self.zc = self.zp - 1
        if self.zc != ratio:
            raise ValueError(f'{ratio}:1 section: external_pins {self.zp} '
                             f'implies ratio {self.zc}')
        self.ecc = g['eccentricity']
        self.disc_t = g['thickness']
        self.pin_circle_r = g['fixed_ring_diameter'] / 2.0
        self.ring_pin_d = g['external_pin_diamter']
        self.out_pin_d = g['output_pin_diamter']
        self.out_pin_n = int(g['output_pins'])
        self.disc_bore_d = g['camshaft_hole']
        self.yaml_tip_d = g.get('output_disk_diameter')

        # disc profile (hobbed): cutter = ring pin + radial print clearance
        self.profile_clear = hw['profile_print_clearance']
        self.cutter_d = self.ring_pin_d + 2 * self.profile_clear
        self.tip_r = self.pin_circle_r + self.ecc - self.cutter_d / 2
        self.root_r = self.pin_circle_r - self.ecc - self.cutter_d / 2
        self.reach_r = self.tip_r + self.ecc     # from the housing axis

        # hardware interface
        self.ring_pin_len = hw['external_pin_length']
        self.cup_hole_d = self.ring_pin_d + hw['external_pin_hole_clearance']
        self.lid_hole_d = self.ring_pin_d + hw['ring_pin_lid_clearance']
        self.hole_chamfer = hw['pin_hole_chamfer']
        self.out_pin_len = hw['output_pin_length']
        self.out_pin_bcd = hw['output_pin_bcd']
        self.pin_hole_d = self.out_pin_d + hw['output_pin_hole_clearance']
        self.motor_bore_d = MOTOR_SHAFT_D + hw['motor_bore_clearance']
        self.motor_flat_x = MOTOR_FLAT_X0 + hw['motor_flat_clearance']
        self.out_hole_d = (self.out_pin_d + 2 * self.ecc
                           + hw['output_hole_clearance'])
        # Live-shaft drive (v5): the 8 mm joint shaft IS the torque path.
        # The output hub continues past the lid face as the shaft-clamp
        # extension: 6805 seat (drive-ear support bearing), then the split
        # pinch that clamps the shaft. The shaft slips into the bore and
        # reaches inside the casing for alignment.
        self.shaft_d = hw['shaft_diameter']
        self.shaft_bore_d = self.shaft_d + hw['shaft_bore_clearance']
        self.hub_yaml_d = hw['hub_diameter']
        self.ext_od = hw['ext_body_od']
        self.seat_od = hw['ext_seat_od']
        self.seat_len = hw['ext_seat_len']
        self.pinch_len = hw['pinch_len']
        self.pinch_bolt_x = hw['pinch_bolt_offset']
        self.ext_len = hw['ext_extension']
        self.mount_bcd = hw['mount_bolt_circle']
        self.mount_n = int(hw['mount_bolt_count'])
        self.mount_hole_d = hw['mount_hole_diameter']
        self.casing_d = hw['casing_diameter']

        # radial: a plain Ø76 cylinder; the pins stand free of the wall
        self.wall_id = 2 * (self.pin_circle_r + self.ring_pin_d / 2) + 0.5
        self.fp_d = self.out_pin_bcd + self.out_pin_d + 4.0     # flange plate 46
        self.lid_id = self.fp_d + 2.0                           # plate bore 48
        self.hub_d = self.hub_yaml_d            # Ø22 inside the lid
        # Lid face bore: sized so the output ASSEMBLES — the lid drops over
        # the extension tip, so the seat and pinch body must pass through
        # this hole. Derive from the fattest extension section, never from
        # the hub (the v5 assembly bug: a Ø23 "hub journal" could not pass
        # the Ø25.05 seat). The journal function is gone; the drive-ear
        # 6805 supports the output at the same station.
        self.lid_hub_bore = max(self.seat_od, self.ext_od) + 0.8

        # axial stack, cup back face -> lid front (mounting) face
        self.plate_t = 5.0
        self.disc_a_z0 = self.plate_t + GAP
        self.disc_a_z1 = self.disc_a_z0 + self.disc_t
        self.disc_b_z0 = self.disc_a_z1 + GAP
        self.disc_b_z1 = self.disc_b_z0 + self.disc_t
        self.lid_z0 = self.disc_b_z1 + GAP          # wall top = lid seat
        self.ring_pin_z1 = self.ring_pin_len        # pins flush at z = 0
        self.lid_embed = self.ring_pin_z1 - self.lid_z0
        self.out_pin_z0 = self.disc_a_z0 + 0.2
        self.out_pin_z1 = self.out_pin_z0 + self.out_pin_len
        self.fp_z0 = self.disc_b_z1 + 1.0            # flange plate bottom
        self.pin_hole_top = self.out_pin_z1 + 0.2    # blind-hole ceiling
        self.fp_z1 = self.pin_hole_top + 0.5         # web over the pins
        self.step_z = self.fp_z1 + 0.5               # lid bore steps 48 -> 23
        self.pocket_top = NEMA_SHAFT_LEN_MAX + 1.0   # motor-shaft-tip room
        self.bore_z0 = self.pocket_top + 2.0         # web, then the shaft bore
        self.body_l = 36.0          # lid face — FROZEN: the printed casings stay
        self.seat_z1 = self.body_l + self.seat_len          # 6805 seat end
        self.ext_z1 = self.body_l + self.ext_len            # extension tip
        self.pinch_z0 = self.ext_z1 - self.pinch_len        # pinch section start
        self.bolt_z = self.pinch_z0 + 4.5                   # pinch cross-bolt
        self.cam_z0 = self.disc_a_z0 - 0.2
        self.cam_mid = self.disc_b_z0
        self.cam_z1 = self.lid_z0 - 0.2

        self.ring_clock_deg, self._ring_screw_dist = self._ring_clock()
        self.checks = self._check()

    def _ring_clock(self):
        """Clock the ring-pin pattern to maximize the distance between the
        cup's press holes (at R on the pin circle) and the four NEMA screw
        counterbores — both live in the back plate. Returns (deg, min mm)."""
        r_s = NEMA_HOLE_SPACING / 2 * math.sqrt(2)
        pitch = 360.0 / self.zp
        best_c, best_d = 0.0, -1.0
        steps = int(pitch / 0.05)
        for i in range(steps):
            c = i * 0.05
            d = min(math.hypot(
                self.pin_circle_r * math.cos(math.radians(c + j * pitch))
                - r_s * math.cos(math.radians(s)),
                self.pin_circle_r * math.sin(math.radians(c + j * pitch))
                - r_s * math.sin(math.radians(s)))
                for j in range(self.zp) for s in (45, 135, 225, 315))
            if d > best_d:
                best_d, best_c = d, c
        return best_c, best_d

    def _min_pin_to_mount_dist(self):
        """Closest approach between any ring pin and any mount screw hole
        (both patterns exist in the cup and the lid)."""
        return min(math.hypot(
            self.pin_circle_r
            * math.cos(math.radians(self.ring_clock_deg) + 2 * math.pi * j / self.zp)
            - self.mount_bcd / 2 * math.cos(2 * math.pi * s / self.mount_n),
            self.pin_circle_r
            * math.sin(math.radians(self.ring_clock_deg) + 2 * math.pi * j / self.zp)
            - self.mount_bcd / 2 * math.sin(2 * math.pi * s / self.mount_n))
            for j in range(self.zp) for s in range(self.mount_n))

    def _check(self):
        c = Checks()
        ck = c.ck
        # mesh + disc
        lam = self.ecc * self.zp / self.pin_circle_r
        ck('trochoid coefficient below 0.8 (undercut guard)', 0.8 - lam, 0.0)
        ck('radial mesh engagement 2e', 2 * self.ecc, 1.2)
        ck('disc clearance to the casing wall',
           self.wall_id / 2 - self.reach_r, 2.0)
        ck('ring pins stand free of the wall',
           self.wall_id / 2 - (self.pin_circle_r + self.ring_pin_d / 2), 0.2)
        ck('casing wall thickness', (self.casing_d - self.wall_id) / 2, 4.0)
        web = (2 * self.pin_circle_r * math.sin(math.pi / self.zp)
               - self.lid_hole_d)
        ck('web between ring-pin holes (lid)', web, MIN_WALL)
        ck('disc output-hole wall to bearing bore',
           self.out_pin_bcd / 2 - self.out_hole_d / 2 - self.disc_bore_d / 2,
           MIN_WALL)
        ck('disc output-hole wall to profile root',
           self.root_r - self.out_pin_bcd / 2 - self.out_hole_d / 2, MIN_WALL)
        if self.yaml_tip_d is not None:
            nominal_tip = 2 * (self.pin_circle_r + self.ecc
                               - self.ring_pin_d / 2)
            ck('yaml output_disk_diameter matches derived tip',
               0.5 - abs(self.yaml_tip_d - nominal_tip), 0.0)
        # ring pins tie the casing halves
        ck('ring pins engage the lid', self.lid_embed, 4.0)
        ck('lid material above the pin holes',
           self.body_l - self.ring_pin_z1 - 0.2, 3.0)
        ck('ring pins outlast disc B',
           self.ring_pin_z1 - self.disc_b_z1, 0.5)
        ck('lid bore keeps the pin holes enclosed',
           self.pin_circle_r - self.ring_pin_d / 2 - self.lid_id / 2, 0.5)
        ck('cam D-flat plane stays inside the bore',
           self.motor_bore_d / 2 - self.motor_flat_x, 0.2)
        ck('cam D-flat clears the shaft flat',
           self.motor_flat_x - MOTOR_FLAT_X0, 0.0)
        ck('ring-pin holes clear the NEMA counterbores',
           self._ring_screw_dist - (M3_CB_D + self.cup_hole_d) / 2, 0.0)
        ck('ring-hole back chamfer clears the NEMA clearance holes',
           self._ring_screw_dist
           - (M3_CLEAR_D + self.cup_hole_d + 2 * self.hole_chamfer) / 2, 0.0)
        ck('ring pins clear the mount screws',
           self._min_pin_to_mount_dist()
           - (self.lid_hole_d + self.mount_hole_d) / 2, 0.2)
        # mount screws (through-bolts, heads on the cup back)
        ck('mount holes inside the casing',
           (self.casing_d - self.mount_hole_d) / 2 - self.mount_bcd / 2, 1.0)
        ck('mount holes stay in the wall',
           self.mount_bcd / 2 - self.mount_hole_d / 2 - self.wall_id / 2, 0.1)
        ck('mount heads clear the motor body',
           (self.mount_bcd - M3_HEAD_D) / 2
           - NEMA_HOLE_SPACING / 2 * math.sqrt(2) - M3_HEAD_D / 2 - 2.0, 0.0)
        # axial stack
        head_top = M3_CB_FLOOR + M3_HEAD_H
        ck('disc A clears the motor screw heads', self.disc_a_z0 - head_top, 0.5)
        ck('output pins clear the motor screw heads',
           self.out_pin_z0 - head_top, 0.5)
        ck('output pins clear the back plate', self.out_pin_z0 - self.plate_t, 0.5)
        ck('flange plate clears disc B', self.fp_z0 - self.disc_b_z1, 0.75)
        ck('flange plate clears the cam', self.fp_z0 - self.cam_z1, 0.5)
        ck('pin embedment in the flange',
           self.pin_hole_top - 0.2 - self.fp_z0, 8.0)
        ck('web between shaft pocket and the shaft bore',
           self.bore_z0 - self.pocket_top, 1.5)
        ck('motor shaft tip room', self.pocket_top - NEMA_SHAFT_LEN_MAX, 0.5)
        ck('frozen casing length preserved', 0.01 - abs(self.body_l - 36.0), 0.0)
        ck('lid face above the face-bore start',
           self.body_l - self.step_z, 3.0)
        ck('lid bore step above the flange plate', self.step_z - self.fp_z1, 0.4)
        # radial interior
        ck('output pins clear the ring pins',
           self.pin_circle_r - self.ring_pin_d / 2
           - (self.out_pin_bcd + self.out_pin_d) / 2, 1.0)
        ck('flange plate spins inside the lid',
           (self.lid_id - self.fp_d) / 2, 0.75)
        # the make-or-break assembly check: the lid must drop over the
        # extension tip, so EVERY section beyond the hub passes this bore
        ck('extension assembles through the lid bore',
           self.lid_hub_bore - max(self.seat_od, self.ext_od), 0.6)
        ck('hub clears the lid bore', (self.lid_hub_bore - self.hub_d) / 2, 0.4)
        ck('flange plate holds the pin holes',
           (self.fp_d - self.pin_hole_d) / 2 - self.out_pin_bcd / 2, MIN_WALL)
        # shaft-clamp extension (live shaft)
        ck('shaft slips into the bore', self.shaft_bore_d - self.shaft_d, 0.05)
        # closing the bore by d diametral eats pi*d of slit width; 1.5 is the
        # slit cut in build_output, worst case a bore printed at modeled size
        ck('pinch slit travel closes the bore',
           1.5 - math.pi * (self.shaft_bore_d - self.shaft_d), 0.3)
        ck('bore wall inside the hub',
           (self.hub_d - self.shaft_bore_d) / 2, 3.0)
        ck('extension body passes the ear 6805',
           EAR_BRG_BORE - self.ext_od, 0.2)
        ck('6805 seat presses the inner race', self.seat_od - EAR_BRG_BORE, 0.0)
        ck('6805 seat press stays light', 0.15 - (self.seat_od - EAR_BRG_BORE), 0.0)
        ck('seat length fills the bearing', self.seat_len - EAR_BRG_W, 0.0)
        ck('extension = ear + gap + pinch',
           0.01 - abs(self.ext_len - (EAR_T + JOINT_GAP + self.pinch_len)), 0.0)
        ck('pinch section past the ear', self.pinch_z0
           - (self.body_l + EAR_T + JOINT_GAP), -0.01)
        ck('pinch bolt clears the shaft bore',
           self.pinch_bolt_x - M3_CLEAR_D / 2 - self.shaft_bore_d / 2, 1.5)
        ck('pinch bolt inside the pinch body',
           math.sqrt((self.ext_od / 2) ** 2 - self.pinch_bolt_x ** 2)
           - M3_CLEAR_D / 2, 1.0)
        ck('nut pocket fits the pinch length',
           self.ext_z1 - (self.bolt_z + M3_NUT_T / 2 + 0.5), 1.0)
        ck('shaft bore depth available', self.ext_z1 - self.bore_z0, 25.0)
        ck('fits the build volume',
           BUILD_VOLUME - max(self.casing_d, self.ext_z1), 5.0)
        return c


# ---------------------------------------------------------------------------
# Temporary-BRep helpers (same pattern as robot_linkages.py)
# ---------------------------------------------------------------------------

def _p(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


class Builder:
    """Accumulates one solid body from primitive unions and subtractions."""

    def __init__(self):
        self.tbm = adsk.fusion.TemporaryBRepManager.get()
        self.body = None

    def add(self, tool):
        if self.body is None:
            self.body = tool
        else:
            self.tbm.booleanOperation(self.body, tool,
                                      adsk.fusion.BooleanTypes.UnionBooleanType)

    def cut(self, tool):
        assert self.body is not None, 'cut before any add'
        self.tbm.booleanOperation(self.body, tool,
                                  adsk.fusion.BooleanTypes.DifferenceBooleanType)

    def cyl(self, p1, p2, dia):
        return self.tbm.createCylinderOrCone(_p(*p1), dia / 20.0, _p(*p2), dia / 20.0)

    def cone(self, p1, d1, p2, d2):
        return self.tbm.createCylinderOrCone(_p(*p1), d1 / 20.0, _p(*p2), d2 / 20.0)

    def box(self, x0, x1, y0, y1, z0, z1):
        center = _p((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
        obb = adsk.core.OrientedBoundingBox3D.create(
            center,
            adsk.core.Vector3D.create(1, 0, 0),
            adsk.core.Vector3D.create(0, 1, 0),
            abs(x1 - x0) / 10.0, abs(y1 - y0) / 10.0, abs(z1 - z0) / 10.0)
        return self.tbm.createBox(obb)

    def cut_bolt_circle_z(self, cx, cy, bcd, n, dia, z0, z1, clock_deg=0.0):
        for i in range(n):
            a = math.radians(clock_deg) + 2 * math.pi * i / n
            x = cx + bcd / 2 * math.cos(a)
            y = cy + bcd / 2 * math.sin(a)
            self.cut(self.cyl((x, y, z0), (x, y, z1), dia))


# ---------------------------------------------------------------------------
# The five parts (per ratio). All are modeled in gearbox-local coordinates;
# the disc is modeled at its cam-angle-0 pose (center offset +e in X).
# ---------------------------------------------------------------------------

def _ring_positions(gb):
    for j in range(gb.zp):
        a = math.radians(gb.ring_clock_deg) + 2 * math.pi * j / gb.zp
        yield gb.pin_circle_r * math.cos(a), gb.pin_circle_r * math.sin(a)


def build_cup(gb):
    """Back casing half: motor plate + plain Ø76 wall. The ring dowels press
    through the plate (flush at the back face) and stand free in the
    interior; the NEMA 17 bolts on from behind with its pilot boss recessed.
    Mount through-holes run the full height, counterbored at the back."""
    b = Builder()
    b.add(b.cyl((0, 0, 0), (0, 0, gb.lid_z0), gb.casing_d))
    b.cut(b.cyl((0, 0, gb.plate_t), (0, 0, gb.lid_z0 + 1), gb.wall_id))
    # ring-pin holes, clocked clear of the NEMA screws; a lead-in chamfer at
    # the BACK face (z 0) — the printed bottom, so it also clears the
    # elephant's-foot lip that would otherwise pinch the hole's first layer.
    # (Not at the interior mouth: there the NEMA counterbore is too close on
    # the 20:1.)
    ch = gb.hole_chamfer
    for x, y in _ring_positions(gb):
        b.cut(b.cyl((x, y, -1), (x, y, gb.plate_t + 0.1), gb.cup_hole_d))
        b.cut(b.cone((x, y, -0.01), gb.cup_hole_d + 2 * ch,
                     (x, y, ch), gb.cup_hole_d))
    # motor interface: pilot recess from the back, shaft bore, screws from
    # inside into the motor's own threads (heads sunk below the plate top)
    b.cut(b.cyl((0, 0, -1), (0, 0, NEMA_PILOT_H), NEMA_PILOT_D))
    b.cut(b.cyl((0, 0, -1), (0, 0, gb.plate_t + 0.1), NEMA_BORE_D))
    h = NEMA_HOLE_SPACING / 2
    for dx, dy in ((h, h), (h, -h), (-h, h), (-h, -h)):
        b.cut(b.cyl((dx, dy, -1), (dx, dy, gb.plate_t + 0.1), M3_CLEAR_D))
        b.cut(b.cyl((dx, dy, M3_CB_FLOOR), (dx, dy, gb.plate_t + 0.1), M3_CB_D))
    # mount through-bolts: snug holes + head counterbores at the back
    b.cut_bolt_circle_z(0, 0, gb.mount_bcd, gb.mount_n, gb.mount_hole_d,
                        -1, gb.lid_z0 + 1)
    b.cut_bolt_circle_z(0, 0, gb.mount_bcd, gb.mount_n, MOUNT_CB_D,
                        -1, MOUNT_CB_T)
    return b.body


def build_lid(gb):
    """Front casing half: Ø76 annular flange, its front face seats flush on
    the linkage ear disc. Blind holes locate the ring dowel tips (keeping
    them captive); the stepped center bore lets the output flange plate spin
    below, then opens to the face bore the extension assembles through;
    mount through-holes match the cup's. Live-shaft v5 REPRINTS this part:
    the dead-axle lid's Ø23 journal could not pass the Ø25.05 seat."""
    b = Builder()
    b.add(b.cyl((0, 0, gb.lid_z0), (0, 0, gb.body_l), gb.casing_d))
    b.cut(b.cyl((0, 0, gb.lid_z0 - 1), (0, 0, gb.step_z), gb.lid_id))
    b.cut(b.cyl((0, 0, gb.step_z), (0, 0, gb.body_l + 1), gb.lid_hub_bore))
    for x, y in _ring_positions(gb):
        b.cut(b.cyl((x, y, gb.lid_z0 - 0.1), (x, y, gb.ring_pin_z1 + 0.2),
                    gb.lid_hole_d))
    b.cut_bolt_circle_z(0, 0, gb.mount_bcd, gb.mount_n, gb.mount_hole_d,
                        gb.lid_z0 - 1, gb.body_l + 1)
    return b.body


def build_disc(gb):
    """Cycloid disc at the cam-angle-0 pose (center at +e). Profile by
    simulated hobbing: subtract the pin-radius cutter along the pin-center
    trochoid p(t) = R*u(t) - e*u(Zp*t) seen from the disc frame."""
    e, r = gb.ecc, gb.pin_circle_r
    b = Builder()
    b.add(b.cyl((e, 0, gb.disc_a_z0), (e, 0, gb.disc_a_z1), 2 * gb.tip_r))
    n = HOB_STEPS_PER_LOBE * gb.zc
    for k in range(n):
        t = 2 * math.pi * k / n
        x = e + r * math.cos(t) - e * math.cos(gb.zp * t)
        y = r * math.sin(t) - e * math.sin(gb.zp * t)
        b.cut(b.cyl((x, y, gb.disc_a_z0 - 0.1), (x, y, gb.disc_a_z1 + 0.1),
                    gb.cutter_d))
    b.cut(b.cyl((e, 0, gb.disc_a_z0 - 1), (e, 0, gb.disc_a_z1 + 1),
                gb.disc_bore_d))
    b.cut_bolt_circle_z(e, 0, gb.out_pin_bcd, gb.out_pin_n, gb.out_hole_d,
                        gb.disc_a_z0 - 1, gb.disc_a_z1 + 1)
    return b.body


def build_cam(gb):
    """Eccentric input cam: 5 mm D-bore, two journals 180° apart for the
    6802 bearings (they overlap 0.5 mm axially so the body is one solid).
    Identical journals + identical discs = balanced eccentric masses."""
    e = gb.ecc
    b = Builder()
    b.add(b.cyl((e, 0, gb.cam_z0), (e, 0, gb.cam_mid), CAM_JOURNAL_D))
    b.add(b.cyl((-e, 0, gb.cam_mid - 0.5), (-e, 0, gb.cam_z1), CAM_JOURNAL_D))
    bore = b.cyl((0, 0, gb.cam_z0 - 1), (0, 0, gb.cam_z1 + 1), gb.motor_bore_d)
    flat = b.box(gb.motor_flat_x, gb.motor_bore_d, -gb.motor_bore_d,
                 gb.motor_bore_d, gb.cam_z0 - 1, gb.cam_z1 + 1)
    b.tbm.booleanOperation(bore, flat,
                           adsk.fusion.BooleanTypes.DifferenceBooleanType)
    b.cut(bore)
    return b.body


def build_output(gb):
    """Output flange (live-shaft v5): six blind press-fit holes take the
    Ø6 x 25 steel dowel pins, which reach back through both discs. The Ø22
    hub spins inside the lid's face bore and continues past the lid face
    as the SHAFT-CLAMP EXTENSION: a Ø25.05 seat carries the drive ear's
    6805 (the joint's -Y support bearing), then the Ø24.6 body ends in a
    split pinch — a radial slit, open at the tip, closed by one M3
    cross-bolt with a captured nut — that clamps the smooth 8 mm shaft.
    The whole extension assembles through the lid's face bore (which is
    why that bore is derived from the seat OD, not the hub).
    The shaft slips into the Ø8.35 bore down to z 28 inside the casing;
    torque leaves through the shaft into the arm hub's own pinch."""
    b = Builder()
    b.add(b.cyl((0, 0, gb.fp_z0), (0, 0, gb.fp_z1), gb.fp_d))
    b.add(b.cyl((0, 0, gb.fp_z1), (0, 0, gb.body_l), gb.hub_d))
    b.add(b.cyl((0, 0, gb.body_l), (0, 0, gb.seat_z1), gb.seat_od))
    b.add(b.cyl((0, 0, gb.seat_z1), (0, 0, gb.ext_z1), gb.ext_od))
    b.cut_bolt_circle_z(0, 0, gb.out_pin_bcd, gb.out_pin_n, gb.pin_hole_d,
                        gb.fp_z0 - 1, gb.pin_hole_top)
    ch = gb.hole_chamfer                        # lead-in at the plate underside
    for i in range(gb.out_pin_n):
        a = 2 * math.pi * i / gb.out_pin_n
        x, y = gb.out_pin_bcd / 2 * math.cos(a), gb.out_pin_bcd / 2 * math.sin(a)
        b.cut(b.cone((x, y, gb.fp_z0 - 0.01), gb.pin_hole_d + 2 * ch,
                     (x, y, gb.fp_z0 + ch), gb.pin_hole_d))
    b.cut(b.cyl((0, 0, gb.fp_z0 - 1), (0, 0, gb.pocket_top),
                gb.motor_bore_d + 3.4))         # motor shaft tip spins here
    # shaft bore: slip fit, from the web over the motor pocket to the tip
    b.cut(b.cyl((0, 0, gb.bore_z0), (0, 0, gb.ext_z1 + 1), gb.shaft_bore_d))
    # pinch slit: radial slot on +X, open at the tip so the halves can flex
    b.cut(b.box(gb.shaft_bore_d / 2 - 0.2, gb.ext_od / 2 + 1,
                -0.75, 0.75, gb.pinch_z0 - 0.5, gb.ext_z1 + 1))
    # cross-bolt: M3 through, captured-nut pocket (-Y, breaks out the side
    # for insertion), spot face (+Y)
    b.cut(b.cyl((gb.pinch_bolt_x, -gb.ext_od / 2 - 1, gb.bolt_z),
                (gb.pinch_bolt_x, gb.ext_od / 2 + 1, gb.bolt_z), M3_CLEAR_D))
    b.cut(b.box(gb.pinch_bolt_x - M3_NUT_AF / 2 - 0.1,
                gb.pinch_bolt_x + M3_NUT_AF / 2 + 0.1,
                -gb.ext_od / 2 - 1, -(gb.shaft_bore_d / 2 + 3.0),
                gb.bolt_z - M3_NUT_T / 2 - 0.5, gb.bolt_z + M3_NUT_T / 2 + 0.5))
    b.cut(b.cyl((gb.pinch_bolt_x, gb.shaft_bore_d / 2 + 3.0, gb.bolt_z),
                (gb.pinch_bolt_x, gb.ext_od / 2 + 1, gb.bolt_z), M3_HEAD_D + 0.5))
    return b.body


def disc_b_matrix(gb, x_off):
    """Occurrence transform mapping the disc-A body to the disc-B pose:
    opposite eccentric (cam at 180°) needs the identical disc clocked half a
    lobe: p -> Rz(-pi/Zc)*(p - (e,0,0)) + (-e,0,dz)."""
    e = gb.ecc
    ang = -math.pi / gb.zc
    m = adsk.core.Matrix3D.create()
    m.setToRotation(ang, adsk.core.Vector3D.create(0, 0, 1),
                    adsk.core.Point3D.create(0, 0, 0))
    tx = -e * math.cos(ang) - e + x_off
    ty = -e * math.sin(ang)
    tz = gb.disc_b_z0 - gb.disc_a_z0
    m.translation = adsk.core.Vector3D.create(tx / 10.0, ty / 10.0, tz / 10.0)
    return m


# ---------------------------------------------------------------------------
# Fusion document assembly + export
# ---------------------------------------------------------------------------

def run(context):  # noqa: ARG001 — Fusion entry point signature
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        gears, hw = load_params()
        gearboxes = {r: Gearbox(r, g, hw) for r, g in sorted(gears.items())}
        failures = []
        for gb in gearboxes.values():
            failures += [f'{gb.ratio}:1  {label}: {c:.2f} < {m:.2f}'
                         for label, c, m in gb.checks.failures()]
        if failures:
            ui.messageBox('Derived geometry FAILED self-checks — nothing '
                          'built:\n\n' + '\n'.join(failures),
                          'RobotTwin cycloidal gearbox generator')
            return

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(app.activeProduct)
        design.designType = adsk.fusion.DesignTypes.DirectDesignType
        root = design.rootComponent

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

        for gi, gb in enumerate(gearboxes.values()):
            x_off = gi * 120.0          # place the assemblies side by side
            parts = {
                f'rt_cyc{gb.zc}_cup': build_cup(gb),
                f'rt_cyc{gb.zc}_lid': build_lid(gb),
                f'rt_cyc{gb.zc}_disc': build_disc(gb),
                f'rt_cyc{gb.zc}_cam': build_cam(gb),
                f'rt_cyc{gb.zc}_output': build_output(gb),
            }
            shift = adsk.core.Matrix3D.create()
            shift.translation = adsk.core.Vector3D.create(x_off / 10.0, 0, 0)

            occurrences = {}
            for name, body in parts.items():
                occ = root.occurrences.addNewComponent(shift)
                occ.component.name = name
                occ.component.bRepBodies.add(body)
                occurrences[name] = occ
            # second disc: same component, opposite eccentric, half a lobe over
            root.occurrences.addExistingComponent(
                occurrences[f'rt_cyc{gb.zc}_disc'].component,
                disc_b_matrix(gb, x_off))

            out_dir = os.path.join(eng_dir, 'gearboxes', f'{gb.zc}_1')
            for name, occ in occurrences.items():
                step_path = os.path.join(out_dir, f'{name}.step')
                try_export(f'{gb.zc}_1/{name}.step',
                           mgr.createSTEPExportOptions(step_path, occ.component))
                stl_path = os.path.join(out_dir, f'{name}.stl')
                opts = mgr.createSTLExportOptions(occ, stl_path)
                opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                try_export(f'{gb.zc}_1/{name}.stl', opts)

        f3d_path = os.path.join(eng_dir, 'f3d', 'rt-gearboxes_cycloidal.f3d')
        try_export(os.path.basename(f3d_path),
                   mgr.createFusionArchiveExportOptions(f3d_path))

        app.activeViewport.fit()

        first = next(iter(gearboxes.values()))
        lines = [
            'Built cycloidal gearbox cartridges from gear_parameters.yaml: '
            + ', '.join(f'{gb.zc}:1 (Zp={gb.zp}, e={gb.ecc} mm)'
                        for gb in gearboxes.values()),
            f'Casing: Ø{first.casing_d:.0f} x {first.body_l:.0f} mm, two '
            f'halves pinned by the Ø6 x 25 ring dowels, clamped by the six '
            f'M3 x 40 mount screws.',
            '',
            'Exported: ' + (', '.join(exported) if exported else 'none'),
        ]
        if failed:
            lines += ['', 'FAILED exports:'] + failed
        lines += ['', 'Verify the assembled mesh with python/validate_cycloid.py.']
        ui.messageBox('\n'.join(lines), 'RobotTwin cycloidal gearbox generator')

    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# ---------------------------------------------------------------------------
# Dry run (no Fusion): parse the yaml, derive, and report the self-checks.
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    import sys
    gears, hw = load_params()
    any_fail = False
    for ratio, g in sorted(gears.items()):
        gb = Gearbox(ratio, g, hw)
        print(f'\n=== {gb.zc}:1  (Zp={gb.zp}, R={gb.pin_circle_r}, '
              f'e={gb.ecc}, ring dowels Ø{gb.ring_pin_d:.0f} x '
              f'{gb.ring_pin_len:.0f} clocked {gb.ring_clock_deg:.2f} deg) ===')
        print(f'  casing Ø{gb.casing_d:.0f} x {gb.body_l:.1f}: cup '
              f'0..{gb.lid_z0} (wall bore Ø{gb.wall_id:.1f}), lid '
              f'{gb.lid_z0}..{gb.body_l} (bores Ø{gb.lid_id:.0f} -> '
              f'Ø{gb.lid_hub_bore:.0f} at z {gb.step_z})')
        print(f'  ring dowels z 0..{gb.ring_pin_z1:.0f}: through the cup, '
              f'{gb.lid_embed:.1f} into the lid; discs {gb.disc_a_z0}..'
              f'{gb.disc_a_z1} and {gb.disc_b_z0}..{gb.disc_b_z1}')
        print(f'  output: plate {gb.fp_z0}..{gb.fp_z1} Ø{gb.fp_d:.0f}, pins '
              f'z {gb.out_pin_z0}..{gb.out_pin_z1} (embed '
              f'{gb.out_pin_z1 - gb.fp_z0:.1f}); disc holes Ø{gb.out_hole_d:.2f}; '
              f'extension: 6805 seat Ø{gb.seat_od:.2f} z {gb.body_l}..'
              f'{gb.seat_z1}, pinch Ø{gb.ext_od:.1f} z {gb.pinch_z0:.2f}..'
              f'{gb.ext_z1:.2f} (bolt z {gb.bolt_z:.2f}); shaft bore '
              f'Ø{gb.shaft_bore_d:.2f} z {gb.bore_z0}..{gb.ext_z1:.2f}')
        print(gb.checks.report())
        fails = gb.checks.failures()
        any_fail |= bool(fails)
        print(f'  {len(gb.checks.rows)} checks, {len(fails)} failed')
    if not IN_FUSION:
        print('\n(dry run only — run from Fusion to build and export; '
              'run python/validate_cycloid.py for the mesh sweep)')
    sys.exit(1 if any_fail else 0)
