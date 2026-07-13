// Pose commanding: Cartesian target (IK) or direct joint angles (FK).

import { JOINT_NAMES } from '../core/config';
import type { Vec3 } from '../core/kinematics';
import { deg2rad, m2mm, mm2m, rad2deg } from '../core/units';
import { config, useTwinStore } from '../state/store';
import { Chip, Panel, Slider } from './controls';

const REACH_MM = m2mm(config.links.upperArm + config.links.forearm);

function TargetControls() {
  const target = useTwinStore((s) => s.target);
  const setTarget = useTwinStore((s) => s.setTarget);
  const branch = useTwinStore((s) => s.branch);
  const setBranch = useTwinStore((s) => s.setBranch);
  const status = useTwinStore((s) => s.ikStatus);

  const axis = (i: 0 | 1 | 2) => (vMm: number) => {
    const next = [...target] as Vec3;
    next[i] = mm2m(vMm);
    setTarget(next);
  };

  return (
    <>
      <Slider label="Target X" value={m2mm(target[0])} min={-REACH_MM} max={REACH_MM} step={2} unit="mm" format={(v) => v.toFixed(0)} onChange={axis(0)} />
      <Slider label="Target Y" value={m2mm(target[1])} min={-REACH_MM} max={REACH_MM} step={2} unit="mm" format={(v) => v.toFixed(0)} onChange={axis(1)} />
      <Slider label="Target Z" value={m2mm(target[2])} min={-20} max={m2mm(config.links.baseHeight) + REACH_MM} step={2} unit="mm" format={(v) => v.toFixed(0)} onChange={axis(2)} />
      <div className="mt-1 flex items-center gap-1">
        <span className="mr-1 text-[10px] uppercase tracking-wide text-slate-500">Branch</span>
        <Chip active={branch === 'elbow-up'} onClick={() => setBranch('elbow-up')}>elbow-up</Chip>
        <Chip active={branch === 'elbow-down'} onClick={() => setBranch('elbow-down')}>elbow-down</Chip>
      </div>
      <div className="mt-2 text-xs">
        {status.kind === 'ok' && !status.nearSingularity && (
          <span className="text-emerald-400">● target reachable</span>
        )}
        {status.kind === 'ok' && status.nearSingularity && (
          <span className="text-amber-400">● reachable — near singularity</span>
        )}
        {status.kind === 'unreachable' && <span className="text-red-400">● out of reach</span>}
        {status.kind === 'limits' && (
          <span className="text-red-400">● reachable only outside joint limits</span>
        )}
      </div>
    </>
  );
}

function JointControls() {
  const motionTo = useTwinStore((s) => (s.motion ? s.motion.plan.to : s.q));
  const setJointTarget = useTwinStore((s) => s.setJointTarget);

  return (
    <>
      {JOINT_NAMES.map((name, i) => {
        const lim = config.limits[name];
        return (
          <Slider
            key={name}
            label={`θ${i + 1} ${name}`}
            value={rad2deg(motionTo[i])}
            min={rad2deg(lim.min)}
            max={rad2deg(lim.max)}
            step={1}
            unit="°"
            format={(v) => v.toFixed(0)}
            onChange={(v) => setJointTarget(i, deg2rad(v))}
          />
        );
      })}
    </>
  );
}

export function PosePanel() {
  const mode = useTwinStore((s) => s.controlMode);
  const setMode = useTwinStore((s) => s.setControlMode);
  const goHome = useTwinStore((s) => s.goHome);
  const showWorkspace = useTwinStore((s) => s.showWorkspace);
  const toggleWorkspace = useTwinStore((s) => s.toggleWorkspace);

  return (
    <Panel title="Pose command">
      <div className="mb-2 flex items-center gap-1">
        <Chip active={mode === 'target'} onClick={() => setMode('target')}>Cartesian (IK)</Chip>
        <Chip active={mode === 'joints'} onClick={() => setMode('joints')}>Joint space</Chip>
        <div className="grow" />
        <Chip onClick={goHome}>⌂ Home</Chip>
        <Chip active={showWorkspace} onClick={toggleWorkspace}>workspace</Chip>
      </div>
      {mode === 'target' ? <TargetControls /> : <JointControls />}
    </Panel>
  );
}
