// Live twin metrics — every value here is a consequence of the gearbox
// choices on the left. Recomputed from core/ on every relevant state change.

import { useMemo } from 'react';
import { computeMetrics } from '../core/metrics';
import { m2mm, rad2deg } from '../core/units';
import { config, useTwinStore } from '../state/store';
import { Panel } from './controls';

function TorqueBar({ required, available }: { required: number; available: number }) {
  const scale = Math.max(available, required, 0.01);
  const availPct = (available / scale) * 100;
  const reqPct = (required / scale) * 100;
  const overloaded = required > available;
  return (
    <div className="relative h-2.5 w-full overflow-hidden rounded bg-slate-800">
      <div className="absolute h-full rounded bg-sky-500/30" style={{ width: `${availPct}%` }} />
      <div
        className={`absolute h-full rounded ${overloaded ? 'bg-red-500' : 'bg-emerald-500/80'}`}
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
              <span className="capitalize text-slate-300">{j.name}</span>
              <span className="tabular-nums text-slate-100">
                {j.requiredTorque.toFixed(2)} / {j.availableTorque.toFixed(2)} N·m
                {j.holdFails && <span className="ml-1 font-semibold text-red-400">COLLAPSES</span>}
              </span>
            </div>
            <TorqueBar required={j.requiredTorque} available={j.availableTorque} />
            <div className="mt-0.5 flex justify-between text-[10px] text-slate-500">
              <span>max {rad2deg(j.maxSpeed).toFixed(0)}°/s</span>
              <span>res {(rad2deg(j.resolution) * 1000).toFixed(1)} m°</span>
              <span>refl. inertia ×{(j.reflectedInertia / Math.max(j.linkInertia, 1e-12)).toFixed(2)}</span>
            </div>
          </div>
        ))}
      </Panel>

      <Panel title="End effector — at current pose">
        <dl className="grid grid-cols-2 gap-x-2 gap-y-1 text-xs">
          <dt className="text-slate-400">Backlash error</dt>
          <dd className="text-right tabular-nums text-slate-100">
            {(m2mm(metrics.backlashErrorTcp)).toFixed(2)} mm
          </dd>
          <dt className="text-slate-400">Max TCP speed</dt>
          <dd className="text-right tabular-nums text-slate-100">
            {(m2mm(metrics.maxTcpSpeed)).toFixed(0)} mm/s
          </dd>
          <dt className="text-slate-400">Manipulability</dt>
          <dd
            className={`text-right tabular-nums ${
              metrics.singularity < 0.05 ? 'text-amber-400' : 'text-slate-100'
            }`}
          >
            {metrics.singularity.toFixed(3)}
            {metrics.singularity < 0.05 && ' ⚠ singular'}
          </dd>
        </dl>
      </Panel>

      <Panel title="Last move">
        {motion && (
          <p className="text-xs text-sky-300">
            moving… {(motion.elapsed).toFixed(1)} / {motion.plan.duration.toFixed(1)} s
          </p>
        )}
        {!motion && lastMove && (
          <dl className="grid grid-cols-2 gap-x-2 gap-y-1 text-xs">
            <dt className="text-slate-400">Duration</dt>
            <dd className="text-right tabular-nums text-slate-100">
              {lastMove.duration.toFixed(2)} s
            </dd>
            <dt className="text-slate-400">Peak torque util.</dt>
            <dd
              className={`text-right tabular-nums ${
                lastMove.skippedSteps ? 'font-semibold text-red-400' : 'text-slate-100'
              }`}
            >
              {(lastMove.peakUtilization * 100).toFixed(0)}% ({lastMove.peakJoint})
            </dd>
            {lastMove.skippedSteps && (
              <dd className="col-span-2 text-red-400">
                ⚠ torque exceeded — open-loop steppers would skip steps here
              </dd>
            )}
            {lastMove.infeasible && (
              <dd className="col-span-2 text-amber-400">
                ⚠ a joint had no torque budget to accelerate at plan time
              </dd>
            )}
          </dl>
        )}
        {!motion && !lastMove && <p className="text-xs text-slate-500">no moves yet</p>}
      </Panel>
    </div>
  );
}
