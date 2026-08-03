"""RobotTwin — parallel-jaw gripper generator for Autodesk Fusion (v2, servo).

Thin Fusion wrapper: ALL geometry lives in gripper_geometry.py (pure Python,
runs and self-checks outside Fusion — `python gripper_geometry.py` prints the
analytic clearance report and the robot.yaml numbers the design implies).
This script replays each part's primitive list through the
TemporaryBRepManager and exports:

    rt_grip_body     — housing: back plate (bolts to the forearm tip), servo
                       bay, bulkhead-plate grooves, rack window, jaw head
    rt_grip_lid      — top plate + the upper rack's back-up rail rib,
                       6 x M3 into inserts
    rt_grip_bulkhead — servo cartridge plate: SG-90 screws to it on the
                       bench, the assembly slides down the wall grooves
    rt_grip_pinion   — lantern pinion: flange pressed on the servo spline,
                       eight printed pins are the teeth
    rt_grip_jaw_p /
    rt_grip_jaw_n    — jaw sliders with integrated hobbed racks (above and
                       below the pinion)
    rt_grip_pad      — TPU finger pad (print 2, flexible filament)

Exports go to engineering/f3d/rt-gripper.f3d plus one STEP and one STL per
part into engineering/gripper/ (the same per-assembly layout as
engineering/gearboxes/).

Run from Fusion: UTILITIES > ADD-INS > Scripts and Add-Ins > "+" > select
this folder > Run. Direct Modeling (no timeline): gripper_geometry.py +
gripper_parameters.yaml are the parametric layer — edit and re-run.
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
        'For the geometry self-checks, run:  python gripper_geometry.py'
    )

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_HERE, '..', 'fusion', 'robot_linkages'))
import linkage_geometry
importlib.reload(linkage_geometry)  # Fusion caches modules between runs
import gripper_geometry
importlib.reload(gripper_geometry)
from gripper_geometry import design
from linkage_geometry import Box, Cyl


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

        # refuse to export geometry that fails its own clearance checks
        fails = d['checks'].failures()
        if fails:
            ui.messageBox(
                'Geometry self-checks FAILED — nothing was built:\n\n'
                + '\n'.join(f'{label}: {c:.2f} < {m:.2f}' for label, c, m in fails),
                'RobotTwin gripper generator')
            return

        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        fusion_design = adsk.fusion.Design.cast(app.activeProduct)
        fusion_design.designType = adsk.fusion.DesignTypes.DirectDesignType
        # NB: the root component name follows the document name and cannot be
        # set on an unsaved document; part names live on the child components.
        root = fusion_design.rootComponent

        occurrences = {}
        for group in (d['parts'], d['hardware']):
            for name, part in group.items():
                occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                occ.component.name = name
                occ.component.bRepBodies.add(realize(part))
                occurrences[name] = occ

        # exports — engineering/f3d + engineering/gripper (printed parts only)
        eng_dir = os.path.abspath(os.path.join(_HERE, '..'))
        exported, failed = [], []
        mgr = fusion_design.exportManager

        def try_export(label, options):
            try:
                mgr.execute(options)
                exported.append(label)
            except Exception:
                failed.append(f'{label}: {traceback.format_exc(limit=1)}')

        f3d_path = os.path.join(eng_dir, 'f3d', 'rt-gripper.f3d')
        try_export(os.path.basename(f3d_path),
                   mgr.createFusionArchiveExportOptions(f3d_path))

        for name in d['parts']:
            occ = occurrences[name]
            step_path = os.path.join(_HERE, f'{name}.step')
            try_export(os.path.basename(step_path),
                       mgr.createSTEPExportOptions(step_path, occ.component))
            stl_path = os.path.join(_HERE, f'{name}.stl')
            opts = mgr.createSTLExportOptions(occ, stl_path)
            opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
            try_export(os.path.basename(stl_path), opts)

        app.activeViewport.fit()

        st = d['st']
        lines = [
            f"Built the gripper: opening 0..{2 * st['travel']:.0f} mm over "
            f"{st['rot_deg']:.0f} deg of SG-90 swing, extent "
            f"{st['extent']:.0f} mm beyond the TCP, ~{st['f_jaw']:.1f} N grip "
            f"per jaw.",
            f"{len(d['checks'].rows)} clearance self-checks passed. "
            "NOT self-locking: keep the servo powered while gripping.",
            'Update robot.yaml (collision.gripper_extent_mm, masses.gripper_g)'
            ' and re-run python/validate_cycloid.py — see the README.',
            '',
            'Exported: ' + (', '.join(exported) if exported else 'none'),
        ]
        if failed:
            lines += ['', 'FAILED exports:'] + failed
        ui.messageBox('\n'.join(lines), 'RobotTwin gripper generator')

    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
