import { defineConfig } from 'vite';

// Relative base so the static build works from any sub-path (e.g. /linkage-sim/).
export default defineConfig({
  base: './',
  // Local tool: one bundle with three.js and CodeMirror is fine.
  build: { chunkSizeWarningLimit: 2000 },
});
