// THE control surface of the twin: per-joint drive type + reduction ratio.
// Efficiency, backlash, torque cap, and inertia are DERIVED from the chosen
// type via config gearbox_models and shown read-only — the point is dialing
// in ratios, not hand-tuning gearbox internals.

import type { GearboxType, JointName } from '../core/config';
import { JOINT_NAMES } from '../core/config';
import { deriveGearbox, ratioRange } from '../core/api';
import { rad2deg } from '../core/units';
import { config, useTwinStore } from '../state/store';
import { Chip, Panel, Slider } from './controls';

const JOINT_LABEL: Record<JointName, string> = {
  base: 'J1 · Base yaw',
  shoulder: 'J2 · Shoulder pitch',
  elbow: 'J3 · Elbow pitch',
};

const TYPE_LABEL: Record<GearboxType, string> = {
  direct: 'Direct 1:1',
  planetary: 'Planetary',
  cycloidal: 'Cycloidal',
};

const TYPES: GearboxType[] = ['direct', 'planetary', 'cycloidal'];

function GearboxCard({ joint }: { joint: JointName }) {
  const drive = useTwinStore((s) => s.drives[joint]);
  const gb = useTwinStore((s) => s.gearboxes[joint]);
  const setDrive = useTwinStore((s) => s.setDrive);

  const [minRatio, maxRatio] = ratioRange(config.gearboxModels, drive.type);
  const { stages } = deriveGearbox(config.gearboxModels, drive.type, drive.ratio);

  return (
    <div className="mb-3 border border-zinc-300 bg-zinc-50 p-2 last:mb-0">
      <div className="mb-1.5 flex items-center justify-between">
        <span className="text-xs font-semibold text-zinc-800">{JOINT_LABEL[joint]}</span>
        <span className="font-mono text-[10px] uppercase tracking-wide text-orange-700">
          {gb.ratio.toFixed(gb.ratio % 1 ? 1 : 0)}:1
          {drive.type === 'planetary' && stages > 1 && ` · ${stages}-stage`}
        </span>
      </div>

      <div className="mb-2 flex flex-wrap gap-1">
        {TYPES.map((t) => (
          <Chip key={t} active={drive.type === t} onClick={() => setDrive(joint, { type: t })}>
            {TYPE_LABEL[t]}
          </Chip>
        ))}
      </div>

      {drive.type !== 'direct' && (
        <Slider
          label="Reduction ratio"
          value={drive.ratio}
          min={minRatio}
          max={maxRatio}
          step={0.5}
          unit=":1"
          onChange={(v) => setDrive(joint, { ratio: v })}
        />
      )}

      <dl className="mt-1 grid grid-cols-2 gap-x-2 gap-y-0.5 text-[10px]">
        <dt className="text-zinc-500">Efficiency</dt>
        <dd className="text-right font-mono tabular-nums text-zinc-700">
          {(gb.efficiency * 100).toFixed(0)}%
        </dd>
        <dt className="text-zinc-500">Backlash</dt>
        <dd className="text-right font-mono tabular-nums text-zinc-700">
          {gb.backlash === 0 ? '~0 (cycloidal)' : `${rad2deg(gb.backlash).toFixed(2)}°`}
        </dd>
        <dt className="text-zinc-500">Torque cap</dt>
        <dd className="text-right font-mono tabular-nums text-zinc-700">{gb.maxTorque.toFixed(1)} N·m</dd>
      </dl>
    </div>
  );
}

export function GearboxPanel() {
  const payload = useTwinStore((s) => s.payload);
  const setPayload = useTwinStore((s) => s.setPayload);

  return (
    <div className="flex flex-col gap-3">
      <Panel title="Drivetrain — independent variables">
        {JOINT_NAMES.map((j) => (
          <GearboxCard key={j} joint={j} />
        ))}
      </Panel>
      <Panel title="Payload">
        <Slider
          label="Mass at gripper"
          value={payload * 1000}
          min={0}
          max={config.masses.payloadMax * 1000}
          step={10}
          unit="g"
          format={(v) => v.toFixed(0)}
          onChange={(v) => setPayload(v / 1000)}
        />
      </Panel>
    </div>
  );
}
