"""RobotTwin — rigid-linkage generator for Autodesk Fusion.

Thin Fusion wrapper: ALL geometry lives in linkage_geometry.py (pure Python,
runs and self-checks outside Fusion — `python linkage_geometry.py` prints the
analytic clearance report, and python/audit_linkages.py sweeps the joints
numerically for interference). This script replays each part's primitive list
through the TemporaryBRepManager and exports:

    rt_base_pan        — stationary base: hanging NEMA 17 mount + yaw bearing boss
    rt_shaft_coupling  — printed coupling: Ø25 journal / D-bore pinch / bolt face
    rt_yaw_column      — theta-1 output: low plate, clevis to the shoulder axis
    rt_upper_arm       — twin-beam clevis link, shoulder hub -> elbow ears
    rt_forearm         — elbow hub -> gripper interface plate (T8 nut pattern)

Exports go to engineering/{f3d,step,stl}; the coupling STEP is also filed
with the purchased hardware in engineering/standard parts/.

The cycloidal gearboxes (out of scope here) bolt onto the drive-side ear
disc faces: Ø66 bolt circle, 6 x M3 heat-set inserts. See
engineering/fusion/cycloidal_gearbox for the cartridge that fits this face.

Run from Fusion: UTILITIES > ADD-INS > Scripts and Add-Ins > "+" > select this
folder > Run. Direct Modeling (no timeline): the Python is the parametric
layer — change a parameter and re-run.
"""

import importlib
import os
import sys
import traceback

try:
    import adsk.core
    import adsk.fusion
except ImportError:  # running outside Fusion (e.g. by accident from a shell)
    raise SystemExit(
        'This script must be run from inside Autodesk Fusion.\n'
        'For the geometry self-checks, run:  python linkage_geometry.py\n'
        'For the interference audit, run:    python ../../../python/audit_linkages.py'
    )

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import linkage_geometry
importlib.reload(linkage_geometry)  # Fusion caches modules between runs
from linkage_geometry import Box, Cyl, design


def _p(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


def realize(part):
    """Replay a Part's primitive list into one temporary BRep body."""
    tbm = adsk.fusion.TemporaryBRepManager.get()
    body = None
    for op, prim in part.steps:
        if isinstance(prim, Cyl):
            tool = tbm.createCylinderOrCone(_p(*prim.p1), prim.d / 20.0,
                                            _p(*prim.p2), prim.d / 20.0)
        elif isinstance(prim, Box):
            lo, hi = prim.lo, prim.hi
            center = _p((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2,
                        (lo[2] + hi[2]) / 2)
            obb = adsk.core.OrientedBoundingBox3D.create(
                center,
                adsk.core.Vector3D.create(1, 0, 0),
                adsk.core.Vector3D.create(0, 1, 0),
                (hi[0] - lo[0]) / 10.0, (hi[1] - lo[1]) / 10.0,
                (hi[2] - lo[2]) / 10.0)
            tool = tbm.createBox(obb)
        else:
            raise TypeError(f'unknown primitive {type(prim)!r}')
        if body is None:
            assert op == 'add', f'{part.name}: first step must be an add'
            body = tool
        else:
            boolean = (adsk.fusion.BooleanTypes.UnionBooleanType if op == 'add'
                       else adsk.fusion.BooleanTypes.DifferenceBooleanType)
            tbm.booleanOperation(body, tool, boolean)
    return body


def run(context):  # noqa: ARG001 — Fusion entry point signature
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        d = design()
        warnings = list(d['warnings'])

        # refuse to export geometry that fails its own clearance checks
        fails = d['checks'].failures()
        if fails:
            ui.messageBox(
                'Geometry self-checks FAILED — nothing was built:\n\n'
                + '\n'.join(f'{label}: {c:.2f} < {m:.2f}' for label, c, m in fails),
                'RobotTwin linkage generator')
            return

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        fusion_design = adsk.fusion.Design.cast(app.activeProduct)
        fusion_design.designType = adsk.fusion.DesignTypes.DirectDesignType
        # NB: the root component name follows the document name and cannot be
        # set on an unsaved document; part names live on the child components.
        root = fusion_design.rootComponent

        occurrences = {}
        for name, part in d['parts'].items():
            occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            occ.component.name = name
            occ.component.bRepBodies.add(realize(part))
            occurrences[name] = occ

        # exports — engineering/{f3d,step,stl}
        script_dir = os.path.dirname(os.path.abspath(__file__))
        eng_dir = os.path.abspath(os.path.join(script_dir, '..', '..'))
        exported, failed = [], []
        mgr = fusion_design.exportManager

        def try_export(label, options):
            try:
                mgr.execute(options)
                exported.append(label)
            except Exception:
                failed.append(f'{label}: {traceback.format_exc(limit=1)}')

        f3d_path = os.path.join(eng_dir, 'f3d', 'rt-arm-3dof_linkages.f3d')
        try_export(os.path.basename(f3d_path),
                   mgr.createFusionArchiveExportOptions(f3d_path))

        for name, occ in occurrences.items():
            step_path = os.path.join(eng_dir, 'step', f'{name}.step')
            try_export(os.path.basename(step_path),
                       mgr.createSTEPExportOptions(step_path, occ.component))
            stl_path = os.path.join(eng_dir, 'stl', f'{name}.stl')
            opts = mgr.createSTLExportOptions(occ, stl_path)
            opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            try_export(os.path.basename(stl_path), opts)

        # the coupling is a reusable standard part — also file it with the
        # purchased-hardware STEPs in engineering/standard parts/
        std_dir = os.path.join(eng_dir, 'standard parts')
        if os.path.isdir(std_dir):
            path = os.path.join(std_dir, 'rt_shaft_coupling.step')
            try_export('standard parts/rt_shaft_coupling.step',
                       mgr.createSTEPExportOptions(
                           path, occurrences['rt_shaft_coupling'].component))

        app.activeViewport.fit()

        lines = [
            f"Built 5 linkage members (shoulder z = {d['shoulder_z']:.0f} mm, "
            f"links {d['l_upper']:.0f} / {d['l_fore']:.0f} mm).",
            f"{len(d['checks'].rows)} clearance self-checks passed; "
            f"UA beam hand-off at x = {d['web_x0']:.0f}..{d['beam_end']:.0f}.",
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
