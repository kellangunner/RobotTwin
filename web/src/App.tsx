import React, { useEffect, useState, ReactNode } from 'react';
import { Scene } from './three/Scene';
import { GearboxPanel } from './ui/GearboxPanel';
import { MetricsPanel } from './ui/MetricsPanel';
import { PosePanel } from './ui/PosePanel';
import { config } from './state/store';
import { wasmReady } from './core/api';

class ErrorBoundary extends React.Component<{ children: ReactNode }, { error?: Error }> {
  constructor(props: { children: ReactNode }) {
    super(props);
    this.state = {};
  }

  static getDerivedStateFromError(error: Error) {
    return { error };
  }

  render() {
    if (this.state.error) {
      return (
        <div className="flex h-screen flex-col gap-4 bg-slate-950 p-4 text-slate-100">
          <h2 className="text-lg font-bold text-red-400">Error</h2>
          <pre className="overflow-auto rounded bg-slate-900 p-2 text-sm text-red-200">
            {this.state.error.message}
            {'\n\n'}
            {this.state.error.stack}
          </pre>
        </div>
      );
    }
    return this.props.children;
  }
}

function AppContent() {
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

function AppLoader() {
  return (
    <div className="flex h-screen items-center justify-center bg-slate-950">
      <div className="text-center">
        <div className="mb-4 text-xs text-slate-400">Initializing...</div>
        <div className="h-1 w-32 bg-slate-800">
          <div className="h-full animate-pulse bg-sky-500" />
        </div>
      </div>
    </div>
  );
}

export default function App() {
  const [ready, setReady] = useState(false);

  useEffect(() => {
    console.log('App: waiting for WASM...');
    wasmReady.then(() => {
      console.log('App: WASM ready!');
      setReady(true);
    }).catch((err) => {
      console.error('App: WASM failed', err);
    });
  }, []);

  console.log('App rendering, ready=' + ready);
  return (
    <ErrorBoundary>
      {ready ? <AppContent /> : <AppLoader />}
    </ErrorBoundary>
  );
}
