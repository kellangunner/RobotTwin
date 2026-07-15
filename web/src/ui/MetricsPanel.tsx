// Live twin metrics — every value here is a consequence of the gearbox
// choices on the left. Recomputed from core/ on every relevant state change.

import { useMemo } from 'react';
import { computeMetrics } from '../core/api';
import { m2mm, rad2deg } from '../core/units';
import { config, useTwinStore } from '../state/store';
import { Panel } from './controls';

function TorqueBar({ required, available }: { required: number; available: number }) {
  const scale = Math.max(available, required, 0.01);
  const availPct = (available / scale) * 100;
  const reqPct = (required / scale) * 100;
  const overloaded = required > available;
  return (
    <div className="relative h-2.5 w-full overflow-hidden border border-zinc-400 bg-zinc-200">
      <div className="absolute h-full bg-sky-600/25" style={{ width: `${availPct}%` }} />
      <div
        className={`absolute h-full ${overloaded ? 'bg-red-600' : 'bg-emerald-600'}`}
        style={{ width: `${reqPct}%` }}
      />
    </div>
  );
}

export function MetricsPanel() {
  const gearboxes = useTwinStore((s) => s.gearboxes);
  const payload = useTwinStore((s) => s.payload);
  const q = useTwinStore((s) => s.q);
  const lastMove = useTwinStore((s) => s.lastMove);
  const motion = useTwinStore((s) => s.motion);
  const sequence = useTwinStore((s) => s.sequence);

  const metrics = useMemo(
    () => computeMetrics(config, gearboxes, q, payload),
    [gearboxes, q, payload],
  );

  return (
    <div className="flex flex-col gap-3">
      <Panel title="Joint torque budget (static hold)">
        {metrics.joints.map((j) => (
          <div key={j.name} className="mb-2.5 last:mb-0">
            <div className="mb-0.5 flex justify-between text-xs">
              <span className="capitalize text-zinc-700">{j.name}</span>
              <span className="font-mono tabular-nums text-zinc-900">
                {j.requiredTorque.toFixed(2)} / {j.availableTorque.toFixed(2)} N·m
                {j.holdFails && <span className="ml-1 font-semibold text-red-600">COLLAPSES</span>}
              </span>
            </div>
            <TorqueBar required={j.requiredTorque} available={j.availableTorque} />
            <div className="mt-0.5 flex justify-between font-mono text-[10px] text-zinc-500">
              <span>max {rad2deg(j.maxSpeed).toFixed(0)}°/s</span>
              <span>res {(rad2deg(j.resolution) * 1000).toFixed(1)} m°</span>
              <span>refl. inertia ×{(j.reflectedInertia / Math.max(j.linkInertia, 1e-12)).toFixed(2)}</span>
            </div>
          </div>
        ))}
      </Panel>

      <Panel title="End effector — at current pose">
        <dl className="grid grid-cols-2 gap-x-2 gap-y-1 text-xs">
          <dt className="text-zinc-500">Backlash error</dt>
          <dd className="text-right font-mono tabular-nums text-zinc-900">
            {(m2mm(metrics.backlashErrorTcp)).toFixed(2)} mm
          </dd>
          <dt className="text-zinc-500">Max TCP speed</dt>
          <dd className="text-right font-mono tabular-nums text-zinc-900">
            {(m2mm(metrics.maxTcpSpeed)).toFixed(0)} mm/s
          </dd>
          <dt className="text-zinc-500">Manipulability</dt>
          <dd
            className={`text-right font-mono tabular-nums ${
              metrics.singularity < 0.05 ? 'text-amber-600' : 'text-zinc-900'
            }`}
          >
            {metrics.singularity.toFixed(3)}
            {metrics.singularity < 0.05 && ' ⚠ singular'}
          </dd>
        </dl>
      </Panel>

      <Panel title="Last move">
        {sequence && (
          <p className="mb-1 font-mono text-xs text-orange-700">
            waypoint {Math.min(sequence.index, sequence.targets.length)}/{sequence.targets.length}
            {motion ? '' : ' — holding'}
          </p>
        )}
        {motion && (
          <p className="font-mono text-xs text-orange-700">
            moving… {(motion.elapsed).toFixed(1)} / {motion.plan.duration.toFixed(1)} s
          </p>
        )}
        {!motion && !sequence && lastMove && (
          <dl className="grid grid-cols-2 gap-x-2 gap-y-1 text-xs">
            <dt className="text-zinc-500">Duration</dt>
            <dd className="text-right font-mono tabular-nums text-zinc-900">
              {lastMove.duration.toFixed(2)} s
            </dd>
            <dt className="text-zinc-500">Peak torque util.</dt>
            <dd
              className={`text-right font-mono tabular-nums ${
                lastMove.skippedSteps ? 'font-semibold text-red-600' : 'text-zinc-900'
              }`}
            >
              {(lastMove.peakUtilization * 100).toFixed(0)}% ({lastMove.peakJoint})
            </dd>
            <dt className="text-zinc-500">Torque governor</dt>
            <dd
              className={`text-right font-mono tabular-nums ${
                lastMove.stretch > 1.001 ? 'text-orange-700' : 'text-zinc-500'
              }`}
            >
              {lastMove.stretch > 1.001
                ? `slowed ×${lastMove.stretch.toFixed(2)}`
                : 'not needed'}
            </dd>
            {lastMove.skippedSteps && (
              <dd className="col-span-2 text-red-600">
                ⚠ torque exceeded at any speed — gravity load beats the torque budget;
                open-loop steppers would skip steps here
              </dd>
            )}
            {lastMove.infeasible && (
              <dd className="col-span-2 text-amber-600">
                ⚠ a joint had no torque budget to accelerate at plan time
              </dd>
            )}
          </dl>
        )}
        {!motion && !sequence && !lastMove && (
          <p className="text-xs text-zinc-500">no moves yet</p>
        )}
      </Panel>
    </div>
  );
}
