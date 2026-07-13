/// <reference types="vitest/config" />
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';
import tailwindcss from '@tailwindcss/vite';

export default defineConfig({
  plugins: [react(), tailwindcss()],
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
