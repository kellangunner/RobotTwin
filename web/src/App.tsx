import { Scene } from './three/Scene';
import { GearboxPanel } from './ui/GearboxPanel';
import { MetricsPanel } from './ui/MetricsPanel';
import { PosePanel } from './ui/PosePanel';
import { config } from './state/store';

export default function App() {
  return (
    <div className="flex h-screen flex-col bg-slate-950 text-slate-100">
      <header className="flex items-baseline gap-3 border-b border-slate-800 px-4 py-2">
        <h1 className="text-sm font-bold tracking-wide">RobotTwin</h1>
        <span className="text-xs text-slate-400">
          {config.name} · 3-DOF digital twin · gearbox trade-study
        </span>
        <span className="ml-auto text-[10px] text-slate-500">
          geometry fixed: 90 / 120 / 120 mm · NEMA 17 · A1 Mini printable
        </span>
      </header>
      <div className="flex min-h-0 flex-1">
        <aside className="w-72 shrink-0 overflow-y-auto border-r border-slate-800 p-3">
          <GearboxPanel />
        </aside>
        <main className="relative min-w-0 flex-1">
          <Scene />
        </main>
        <aside className="w-72 shrink-0 overflow-y-auto border-l border-slate-800 p-3">
          <div className="flex flex-col gap-3">
            <PosePanel />
            <MetricsPanel />
          </div>
        </aside>
      </div>
    </div>
  );
}
