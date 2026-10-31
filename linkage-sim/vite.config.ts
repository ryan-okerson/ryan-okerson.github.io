import { defineConfig } from 'vite';

// Relative base so the static build works from any sub-path (e.g. /linkage-sim/).
export default defineConfig({
  base: './',
});
