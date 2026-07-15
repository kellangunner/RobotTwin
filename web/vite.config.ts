/// <reference types="vitest/config" />
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';

export default defineConfig({
  plugins: [react(), tailwindcss()],
  // relative asset URLs so the built twin can be served standalone or
  // embedded under any subpath (e.g. the portfolio site's /robot-twin/sim/)
  base: './',
  build: {
    // top-level await in core/api.ts (WASM module init)
    target: 'es2022',
  },
  server: {
    port: parseInt(process.env.PORT || '5173', 10),
    fs: {
      // allow importing config/robot.yaml from the repo root
      allow: ['..'],
    },
  },
  test: {
    environment: 'node',
    include: ['src/**/*.test.ts'],
  },
});
