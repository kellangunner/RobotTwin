"""RobotTwin — numeric interference audit of the printed linkage assembly.

Loads the parametric geometry (engineering/fusion/robot_linkages/
linkage_geometry.py), poses the moving groups through their configured joint
ranges, and grid-samples for solid/solid overlap between:

    - the five printed members,
    - purchased hardware (base NEMA 17, screw heads, clamp bolt),
    - reserved keep-out envelopes (cycloidal cartridge + flange + motor
      at each pitch joint).

Adjacent-link mechanical clearance is what CAD must guarantee; whole-arm
world collisions (folded forearm vs the base at extreme poses, table
strikes) are the twin's runtime collision layer per config/robot.yaml
`collision:` and are intentionally out of scope here.

Designed contacts (bearing races on journals/pockets, shaft in bores,
mating bolted faces) are excluded by construction: bearings and shafts are
not modeled as solids, and bolted interfaces meet at exact planes which the
strict-interior sampling does not flag.

Usage:  python python/audit_linkages.py [--res MM]
Exit status 0 = no interference found.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'engineering', 'fusion', 'robot_linkages'))
import linkage_geometry as lg


# ---------------------------------------------------------------------------
# Posing: rotate a moving group about a world axis; membership tests happen
# in the group's local frame (sample points come from the group's own
# primitives, then transform to world to test the static group).
# ---------------------------------------------------------------------------

class Pose:
    """Rotation by theta about a line through `origin` along `axis` (unit,
    one of x/y/z here). local -> world."""

    def __init__(self, axis, origin, theta_deg):
        self.axis = axis
        self.o = origin
        t = math.radians(theta_deg)
        self.c, self.s = math.cos(t), math.sin(t)
        self.theta_deg = theta_deg

    def to_world(self, p):
        ox, oy, oz = self.o
        x, y, z = p[0] - ox, p[1] - oy, p[2] - oz
        c, s = self.c, self.s
        if self.axis == 'z':                    # yaw
            return (ox + c * x - s * y, oy + s * x + c * y, oz + z)
        if self.axis == 'y':                    # pitch: +theta lifts +X to +Z
            return (ox + c * x - s * z, oy + y, oz + s * x + c * z)
        raise ValueError(self.axis)


IDENTITY = Pose('z', (0, 0, 0), 0.0)


class Group:
    """A rigid set of Parts sharing one pose."""

    def __init__(self, name, parts):
        self.name = name
        self.parts = parts

    def contains(self, p_local):
        for part in self.parts:
            if part.contains(p_local):
                return part.name
        return None

    def sample_prims(self):
        """(part_name, add-primitive) pairs to source sample points from."""
        for part in self.parts:
            for op, prim in part.steps:
                if op == 'add':
                    yield part.name, prim


def check_pair(static, moving, pose, res, hits, max_hits=5):
    """Sample the moving group's add-primitives (local frame), transform to
    world, and flag points inside both groups. Points are sampled strictly
    inside primitives (grid offset by res/2) so exact mating faces between
    designed contacts do not false-positive."""
    for part_name, prim in moving.sample_prims():
        lo, hi = prim.bbox()
        nx = max(1, int((hi[0] - lo[0]) / res))
        ny = max(1, int((hi[1] - lo[1]) / res))
        nz = max(1, int((hi[2] - lo[2]) / res))
        for i in range(nx):
            x = lo[0] + (i + 0.5) * (hi[0] - lo[0]) / nx
            for j in range(ny):
                y = lo[1] + (j + 0.5) * (hi[1] - lo[1]) / ny
                for k in range(nz):
                    z = lo[2] + (k + 0.5) * (hi[2] - lo[2]) / nz
                    p = (x, y, z)
                    if not prim.contains(p):
                        continue
                    if moving.contains(p) is None:   # inside a cut void
                        continue
                    w = pose.to_world(p)
                    other = static.contains(w)
                    if other:
                        hits.append((static.name, other, moving.name, part_name,
                                     pose.theta_deg, w))
                        if len(hits) >= max_hits:
                            return


def frange(a, b, step):
    vals, v = [], a
    while v <= b + 1e-9:
        vals.append(round(v, 3))
        v += step
    return vals


def self_test():
    """The detector must flag a known overlap and pass a known gap."""
    a = lg.Part('A'); a.add(lg.Box(0, 10, 0, 10, 0, 10))
    b = lg.Part('B'); b.add(lg.Box(9, 19, 0, 10, 0, 10))     # 1 mm overlap
    c = lg.Part('C'); c.add(lg.Box(10.5, 20, 0, 10, 0, 10))  # 0.5 mm gap
    hits = []
    check_pair(Group('GA', [a]), Group('GB', [b]), IDENTITY, 0.4, hits)
    assert hits, 'self-test: failed to detect a 1 mm overlap'
    hits = []
    check_pair(Group('GA', [a]), Group('GC', [c]), IDENTITY, 0.4, hits)
    assert not hits, 'self-test: false positive across a 0.5 mm gap'


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--res', type=float, default=1.2,
                    help='sample grid resolution in mm (default 1.2)')
    args = ap.parse_args()

    self_test()

    d = lg.design()
    fails = d['checks'].failures()
    print(f"analytic checks: {len(d['checks'].rows)} run, {len(fails)} failed")
    if fails:
        for label, c, m in fails:
            print(f'  FAIL {label}: {c:.2f} < {m:.2f}')

    P, H, K = d['parts'], d['hardware'], d['keepouts']
    sz, ex = d['shoulder_z'], d['l_upper']
    t2lo, t2hi = d['theta2_range']
    t3lo, t3hi = d['theta3_range']

    g_base = Group('base', [P['rt_base_pan'], H['hw_base_motor'],
                            H['hw_pan_screws']])
    g_column = Group('column', [P['rt_yaw_column'], P['rt_shaft_coupling'],
                                H['hw_clamp_bolt'], H['hw_column_screws'],
                                K['ko_gearbox_shoulder']])
    g_upper = Group('upper_arm', [P['rt_upper_arm'], H['hw_pinch_shoulder'],
                                  K['ko_gearbox_elbow']])
    g_forearm = Group('forearm', [P['rt_forearm'], H['hw_pinch_elbow']])
    g_col_static = Group('base+column', g_base.parts + g_column.parts)

    hits = []
    t0 = time.time()

    # 1. column group spinning over the stationary base (yaw)
    for theta in frange(0, 90, 15):            # boss/pan are 4-fold symmetric
        check_pair(g_base, g_column, Pose('z', (0, 0, 0), theta), args.res, hits)

    # 2. upper arm swinging over base + column (shoulder pitch, full range)
    for theta in frange(t2lo, t2hi, 10):
        check_pair(g_col_static, g_upper, Pose('y', (0, 0, sz), theta),
                   args.res, hits)

    # 3. forearm folding on the upper arm (elbow, full range, fine near limits)
    thetas = set(frange(t3lo, t3hi, 10)) | {t3lo, t3lo + 2, t3lo + 5,
                                            t3hi - 5, t3hi - 2, t3hi}
    for theta in sorted(thetas):
        check_pair(g_upper, g_forearm, Pose('y', (ex, 0, sz), theta),
                   args.res, hits)

    dt = time.time() - t0
    if hits:
        print(f'\nINTERFERENCE: {len(hits)} hit(s) found ({dt:.1f} s):')
        for sgroup, spart, mgroup, mpart, theta, w in hits:
            print(f'  {mgroup}/{mpart} vs {sgroup}/{spart} at theta='
                  f'{theta:g} deg, point ({w[0]:.1f}, {w[1]:.1f}, {w[2]:.1f})')
    else:
        print(f'\nno interference at {args.res} mm resolution across the '
              f'swept ranges ({dt:.1f} s)')
    return 1 if (hits or fails) else 0


if __name__ == '__main__':
    raise SystemExit(main())
