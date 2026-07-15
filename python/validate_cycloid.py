"""Validation for the RobotTwin printed cycloidal gearboxes (15:1 and 20:1).

Checks the cycloid disc geometry BEFORE anything is printed (simulation
first): undercut, meshing depth, printable wall thicknesses, and the static
torque budget against config/robot.yaml.

There is one source of truth: engineering/gearboxes/gear_parameters.yaml.
This script imports the generator module
(engineering/fusion/cycloidal_gearbox/cycloidal_gearbox.py — importable
outside Fusion) so the derived dimensions here are BY CONSTRUCTION the ones
Fusion will build; it then re-runs the generator's analytic clearance checks
and adds the numeric profile checks the generator cannot do cheaply.

The disc profile is the inner offset (by the pin radius) of the pin-center
locus seen from the disc frame:

    p(phi) = Rot(phi/Zc) . ( R*[1,0] - e*[cos(phi), sin(phi)] ),
    phi in [0, 2*pi*Zc)

with Zp = Zc + 1 ring pins on radius R and cam eccentricity e. The profile is
free of cusps/undercut iff the pin radius stays below the locus' minimum
radius of curvature (checked numerically).

Outputs a PASS/FAIL report per ratio and writes an SVG of each disc profile
meshed with its ring pins to engineering/gearboxes/<ratio>_1/.

Usage:  python python/validate_cycloid.py
"""

import math
import os
import re
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(REPO, 'engineering', 'fusion',
                                'cycloidal_gearbox'))
from cycloidal_gearbox import Gearbox, load_params  # noqa: E402

CYC_EFF = 0.60          # printed-drive efficiency assumption (see README)
N_DISCS = 2


def read_yaml_numbers(path, keys):
    """Minimal flat 'key: number' reader (config/robot.yaml)."""
    with open(path, 'r', encoding='utf-8') as f:
        text = f.read()
    out = {}
    for key in keys:
        m = re.search(r'^\s*' + key + r'\s*:\s*([0-9.]+)', text, re.MULTILINE)
        if m:
            out[key] = float(m.group(1))
    return out


def min_convex_curvature_radius(gb, n=8192):
    """Minimum radius of curvature of the pin-center locus, taken only where
    the center of curvature lies on the disc-interior side (the offset side).

    Offsetting a curve inward by the pin radius forms a cusp (undercut) only
    where the curvature center sits on the offset side closer than the offset
    distance; the locus' concave valleys have tiny radii but curve away from
    the disc and are safe. Locus (exact): p(t) = R*u(t) - e*u(Zp*t)."""
    r, e, zp = gb.pin_circle_r, gb.ecc, gb.zp
    rmin = float('inf')
    for k in range(n):
        t = 2 * math.pi * k / n
        x = r * math.cos(t) - e * math.cos(zp * t)
        y = r * math.sin(t) - e * math.sin(zp * t)
        dx = -r * math.sin(t) + e * zp * math.sin(zp * t)
        dy = r * math.cos(t) - e * zp * math.cos(zp * t)
        ddx = -r * math.cos(t) + e * zp * zp * math.cos(zp * t)
        ddy = -r * math.sin(t) + e * zp * zp * math.sin(zp * t)
        speed2 = dx * dx + dy * dy
        num = dx * ddy - dy * ddx           # signed curvature numerator
        if abs(num) < 1e-12:
            continue
        # left normal; the interior (offset) side points toward the origin
        nlx, nly = -dy, dx
        inward = 1.0 if (nlx * x + nly * y) < 0 else -1.0
        # curvature center is at p + n_left / kappa_signed; it lies on the
        # interior side iff inward and the curvature share sign
        if inward * num > 0:
            rmin = min(rmin, speed2 ** 1.5 / abs(num))
    return rmin


def profile_points(gb, n=2048):
    """Disc profile: inner offset of the locus by the cutter radius."""
    r, e, zc = gb.pin_circle_r, gb.ecc, gb.zc
    cutter = gb.cutter_d / 2
    pts = []
    for k in range(n):
        phi = 2 * math.pi * zc * k / n
        # locus point and (numerical) inward normal toward the disc center
        eps = 1e-5
        def locus(p):
            a = p / zc
            px = r - e * math.cos(p)
            py = -e * math.sin(p)
            return (px * math.cos(a) - py * math.sin(a),
                    px * math.sin(a) + py * math.cos(a))
        x0, y0 = locus(phi)
        x1, y1 = locus(phi + eps)
        tx, ty = x1 - x0, y1 - y0
        t = math.hypot(tx, ty)
        if t < 1e-12:
            continue
        # normal pointing toward origin-side of the tangent
        nx, ny = -ty / t, tx / t
        if nx * x0 + ny * y0 > 0:
            nx, ny = -nx, -ny
        pts.append((x0 + cutter * nx, y0 + cutter * ny))
    return pts


def write_svg(path, gb):
    """Disc profile meshed with the ring pins (disc drawn at cam angle 0)."""
    prof = profile_points(gb)
    r, e, pin_r = gb.pin_circle_r, gb.ecc, gb.ring_pin_d / 2
    s = 6.0  # px per mm
    size = int(2 * (gb.ring_id / 2 + 4) * s)
    cx = cy = size / 2

    def pt(x, y):
        return f'{cx + x * s:.1f},{cy - y * s:.1f}'

    path_d = 'M ' + ' L '.join(pt(x + e, y) for x, y in prof) + ' Z'
    pins = []
    for j in range(gb.zp):
        a = 2 * math.pi * j / gb.zp
        pins.append(
            f'<circle cx="{cx + r * math.cos(a) * s:.1f}" '
            f'cy="{cy - r * math.sin(a) * s:.1f}" '
            f'r="{pin_r * s:.1f}" fill="#888"/>')
    holes = []
    for j in range(gb.out_pin_n):
        a = 2 * math.pi * j / gb.out_pin_n
        hx = e + gb.out_pin_bcd / 2 * math.cos(a)
        hy = gb.out_pin_bcd / 2 * math.sin(a)
        holes.append(
            f'<circle cx="{cx + hx * s:.1f}" cy="{cy - hy * s:.1f}" '
            f'r="{gb.out_hole_d / 2 * s:.1f}" fill="#fff" stroke="#26c"/>')
        holes.append(
            f'<circle cx="{cx + hx * s:.1f}" cy="{cy - hy * s:.1f}" '
            f'r="{gb.out_pin_d / 2 * s:.1f}" fill="#bcd"/>')
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}" viewBox="0 0 {size} {size}">
<rect width="100%" height="100%" fill="white"/>
<circle cx="{cx}" cy="{cy}" r="{gb.ring_id / 2 * s:.1f}" fill="none" stroke="#aaa" stroke-dasharray="4 3"/>
{''.join(pins)}
<path d="{path_d}" fill="#e8eefc" stroke="#26c" stroke-width="1.5"/>
<circle cx="{cx + e * s:.1f}" cy="{cy}" r="{gb.disc_bore_d / 2 * s:.1f}" fill="#fff" stroke="#26c"/>
{''.join(holes)}
<text x="8" y="{size - 10}" font-family="monospace" font-size="14">rt_cyc{gb.zc} disc — Zc={gb.zc} Zp={gb.zp} R={r} e={e} (cam at 0°)</text>
</svg>'''
    with open(path, 'w', encoding='utf-8') as f:
        f.write(svg)


def main():
    cfg = read_yaml_numbers(
        os.path.join(REPO, 'config', 'robot.yaml'),
        ['holding_torque_nm', 'max_speed_rpm', 'upper_arm_g', 'forearm_g',
         'elbow_motor_g', 'gripper_g', 'payload_default_g', 'payload_max_g',
         'upper_arm_mm', 'forearm_mm'])
    t_motor = cfg['holding_torque_nm']
    l_ua = cfg['upper_arm_mm'] / 1000
    l_fa = cfg['forearm_mm'] / 1000
    g = 9.81

    def shoulder_gravity(payload_kg):
        """Worst case: arm straight and horizontal, payload at the TCP."""
        return g * (cfg['upper_arm_g'] / 1000 * l_ua / 2
                    + cfg['elbow_motor_g'] / 1000 * l_ua
                    + cfg['forearm_g'] / 1000 * (l_ua + l_fa / 2)
                    + cfg['gripper_g'] / 1000 * (l_ua + l_fa)
                    + payload_kg * (l_ua + l_fa))

    tau_nom = shoulder_gravity(cfg['payload_default_g'] / 1000)
    tau_max = shoulder_gravity(cfg['payload_max_g'] / 1000)

    gears, hw = load_params()
    all_ok = True
    for ratio, gp in sorted(gears.items()):
        gb = Gearbox(ratio, gp, hw)
        checks = []

        def check(name, ok, detail):
            checks.append((name, ok, detail))

        lam = gb.ecc * gb.zp / gb.pin_circle_r
        check('trochoid coefficient', 0.4 <= lam <= 0.8,
              f'lambda = e*Zp/R = {lam:.3f} (want 0.4..0.8)')

        rho = min_convex_curvature_radius(gb)
        check('no undercut', gb.cutter_d / 2 < rho,
              f'min convex-side curvature radius {rho:.2f} > cutter '
              f'{gb.cutter_d / 2:.2f}')

        # the generator's own analytic clearance checks (walls, stack,
        # mount interface, pinch clamp) roll up into one line here
        gen_fails = gb.checks.failures()
        check('generator self-checks', not gen_fails,
              f'{len(gb.checks.rows)} checks'
              + (': ' + '; '.join(f[0] for f in gen_fails) if gen_fails
                 else ' all pass'))

        t_out = t_motor * gb.zc * CYC_EFF
        f_cam = t_out / (gb.zc * gb.ecc / 1000) / N_DISCS
        f_pin = 4 * t_out / (gb.zp * gb.pin_circle_r / 1000)
        f_out = t_out / (gb.out_pin_n / 2 * gb.out_pin_bcd / 2000)

        check('holds nominal payload', t_out >= tau_nom,
              f'{t_out:.2f} Nm vs {tau_nom:.2f} Nm (100 g at full reach)')

        status = 'PASS' if all(ok for _, ok, _ in checks) else 'FAIL'
        all_ok &= status == 'PASS'
        print(f'\n=== {gb.zc}:1  (Zc={gb.zc}, Zp={gb.zp}, '
              f'R={gb.pin_circle_r}, e={gb.ecc}, '
              f'pins Ø{gb.ring_pin_d:.0f})  ->  {status} ===')
        for name, ok, detail in checks:
            print(f'  [{"ok" if ok else "XX"}] {name:<26} {detail}')
        print(f'  disc radii: root {gb.root_r:.2f} / tip {gb.tip_r:.2f} mm; '
              f'output speed {cfg["max_speed_rpm"] / gb.zc:.0f} rpm '
              f'({cfg["max_speed_rpm"] / gb.zc * 6:.0f} deg/s)')
        print(f'  envelope: Ø{gb.body_d:.0f} x {gb.body_l:.1f} body + '
              f'Ø{gb.flange_d:.0f} x {gb.flange_t:.0f} flange; steel output '
              f'pins Ø{gb.out_pin_d:.0f} x {gb.out_pin_len:.0f} '
              f'(embed {gb.out_pin_z1 - gb.fp_z0:.1f})')
        print(f'  torque: out {t_out:.2f} Nm '
              f'(motor {t_motor} Nm x {gb.zc} x {CYC_EFF:.2f}); '
              f'gravity nominal {tau_nom:.2f} / max-payload {tau_max:.2f} Nm '
              f'-> margin x{t_out / tau_nom:.1f} / x{t_out / tau_max:.2f}')
        print(f'  loads: cam bearing {f_cam:.0f} N per 6802; '
              f'peak ring pin ~{f_pin:.0f} N; '
              f'output pin ~{f_out:.0f} N each (half engaged)')

        out_dir = os.path.join(REPO, 'engineering', 'gearboxes', f'{gb.zc}_1')
        os.makedirs(out_dir, exist_ok=True)
        svg = os.path.join(out_dir, f'rt_cyc{gb.zc}_disc_profile.svg')
        write_svg(svg, gb)
        print(f'  profile SVG -> {os.path.relpath(svg, REPO)}')

    print()
    sys.exit(0 if all_ok else 1)


if __name__ == '__main__':
    main()
