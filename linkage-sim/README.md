# linkage-sim

Browser-based 3D kinematic simulator for overconstrained spatial linkages
(scissor / pantograph deployables for spacecraft solar arrays).

The full spec lives in [PROMPT.md](PROMPT.md). This folder is currently a
scaffold only: Vite + TypeScript + Vitest, with no simulator code yet.

## Commands

```sh
npm install
npm run dev     # local dev server
npm test        # vitest
npm run build   # static output in dist/
```

The build uses a relative base path, so `dist/` can be served from any
sub-path, e.g. `ryan-okerson.github.io/linkage-sim/`.
