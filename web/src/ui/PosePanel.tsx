// Pose commanding: Cartesian target (IK) or direct joint angles (FK), plus
// CSV waypoint sequences interpreted per the active tab.

import { useRef, useState } from 'react';
import { JOINT_NAMES } from '../core/config';
import type { Vec3 } from '../core/kinematics';
import { deg2rad, m2mm, mm2m, rad2deg } from '../core/units';
import { parseWaypointCsv } from '../core/api';
import { config, useTwinStore } from '../state/store';
import { Chip, Panel, Slider } from './controls';

const REACH_MM = m2mm(config.links.upperArm + config.links.forearm);

function TargetControls() {
  const target = useTwinStore((s) => s.target);
  const setTarget = useTwinStore((s) => s.setTarget);
  const branch = useTwinStore((s) => s.branch);
  const setBranch = useTwinStore((s) => s.setBranch);

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

function StatusLine() {
  const status = useTwinStore((s) => s.ikStatus);
  const sequence = useTwinStore((s) => s.sequence);

  if (sequence) {
    const at = Math.min(sequence.index, sequence.targets.length);
    return (
      <div className="mt-2 text-xs text-sky-300">
        ▶ waypoint {at}/{sequence.targets.length}…
      </div>
    );
  }
  return (
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
      {status.kind === 'collision' && (
        <span className="text-red-400">● blocked — {status.issues[0]}</span>
      )}
      {status.kind === 'path-collision' && (
        <span className="text-red-400">● move blocked — path collision: {status.issues[0]}</span>
      )}
    </div>
  );
}

function CsvLoader() {
  const mode = useTwinStore((s) => s.controlMode);
  const fileRef = useRef<HTMLInputElement>(null);
  const [message, setMessage] = useState<string | null>(null);

  const onFile: React.ChangeEventHandler<HTMLInputElement> = (e) => {
    const file = e.target.files?.[0];
    e.target.value = '';
    if (!file) return;
    file.text().then((text) => {
      const s = useTwinStore.getState();
      const result = parseWaypointCsv(
        text,
        s.controlMode === 'target' ? 'cartesian' : 'joints',
        config,
        s.branch,
        s.motion ? s.motion.plan.to : s.q,
      );
      if (result.targets.length === 0) {
        setMessage(`no valid waypoints${result.firstIssue ? ` — ${result.firstIssue}` : ''}`);
        return;
      }
      setMessage(
        `${result.targets.length} waypoints loaded` +
          (result.skipped > 0 ? ` · ${result.skipped} skipped (${result.firstIssue})` : ''),
      );
      s.startSequence(result.targets);
    });
  };

  return (
    <div className="mt-2 border-t border-slate-800 pt-2">
      <div className="flex items-center gap-1">
        <Chip onClick={() => fileRef.current?.click()}>⭱ Load CSV</Chip>
        <span className="text-[10px] text-slate-500">
          rows: {mode === 'target' ? 'x, y, z (mm)' : 'θ1, θ2, θ3 (deg)'}
        </span>
      </div>
      <input ref={fileRef} type="file" accept=".csv,text/csv,text/plain" className="hidden" onChange={onFile} />
      {message && <p className="mt-1 text-[10px] text-slate-400">{message}</p>}
    </div>
  );
}

export function PosePanel() {
  const mode = useTwinStore((s) => s.controlMode);
  const setMode = useTwinStore((s) => s.setControlMode);
  const goHome = useTwinStore((s) => s.goHome);
  const showWorkspace = useTwinStore((s) => s.showWorkspace);
  const toggleWorkspace = useTwinStore((s) => s.toggleWorkspace);
  const clearTrace = useTwinStore((s) => s.clearTrace);
  const traceLen = useTwinStore((s) => s.trace.length);

  return (
    <Panel title="Pose command">
      <div className="mb-2 flex items-center gap-1">
        <Chip active={mode === 'target'} onClick={() => setMode('target')}>Cartesian (IK)</Chip>
        <Chip active={mode === 'joints'} onClick={() => setMode('joints')}>Joint space</Chip>
      </div>
      <div className="mb-2 flex flex-wrap items-center gap-1">
        <Chip onClick={goHome}>⌂ Home</Chip>
        <Chip onClick={clearTrace}>✕ Clear path{traceLen > 0 ? ` (${traceLen})` : ''}</Chip>
        <Chip active={showWorkspace} onClick={toggleWorkspace}>Workspace</Chip>
      </div>
      {mode === 'target' ? <TargetControls /> : <JointControls />}
      <StatusLine />
      <CsvLoader />
    </Panel>
  );
}
