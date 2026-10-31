# linkage-sim

Browser-based 3D kinematic simulator for overconstrained spatial linkages, mainly
scissor (pantograph) deployable structures. You describe bodies, joints, gear
synchronizers and one driven motion in a short script; the app assembles the
constraint equations and Jacobian, solves the motion, checks mobility from the
numerical rank of the Jacobian, and animates the result.

## Coding principles
- **Think before coding.** Don't assume. If something here is ambiguous, state the ambiguity and the choice you made.
- **Simplicity first.** No features this prompt doesn't ask for. No abstraction layers "for later."
- **Surgical changes.** Don't refactor working code. Match the existing style.
- **Goal-driven.** Write the test first, then the code that passes it.
- **Easy to consume.** Code should be skimmable, with no cleverness and early returns.
- **One thing per function.**
- **Obvious variable names.**
- **One-line comments** above non-obvious blocks.
- **Simple error handling.** Use try/catch, log the error, then return null or continue.

## Running it

```sh
npm install
npm run dev     # http://localhost:5173
npm test        # the 9 spec tests + a Jacobian check
npm run build   # static site in dist/ (open with: npx vite preview)
```

Everything runs in the browser; the solver runs in a Web Worker. The original
specification is in [PROMPT.md](PROMPT.md), and every ambiguity and the choice
made is listed in [NOTES.md](NOTES.md), together with the test results.

## Using the app

- **Left:** the model script and an editable `library.js` tab. `Ctrl/Cmd+Enter` or
  **Run** re-runs. Every `param` gets a slider and every group gets a length-scale
  slider; moving one re-runs the script and re-solves.
- **Center:** 3D view (drag to orbit, scroll to zoom). Bars are colored by their first
  tag. Joint symbols: pin = grey cylinder along its axis, ball = yellow sphere,
  universal joint = blue ring, gear = pink ring. Hover a bar to see its name;
  click it to plot the coordinates of its nearest named point. The scrubber and
  Play button move through the solved steps.
- **Right:** mobility at the current step (Kutzbach count next to the rank result,
  smallest singular values, a "wiggle" button per free-motion shape), then plots
  versus driven angle or time: free motions, smallest singular values (log scale,
  dashed line = rank threshold), picked quantities (points, or pin angles from the
  menu), and bounding-box extent. Red vertical lines mark steps where Newton
  failed, the mechanism locked, or a branch jump was suspected.
- **Export:** model JSON, a CSV of every plotted quantity, and a PNG of the view.
  **Load JSON** solves a saved model directly (sliders need the script).

## How to write a model

A model is plain JavaScript. It calls the functions below; `library.js` is
prepended, so its macros (`lazyTong`, `gearedRoot`, `doubleTong`, ...) are
available too. Units are meters and radians; profiles and rotations accept degrees
with `deg: true`. A minimal model:

```js
const L = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const units = lazyTong(3, L);
ground(units[0].A);
drive(units[0].A, units[0].B, { profile: ramp({ from: 85, to: 8, deg: true, time: 10 }) });
```

You never place bodies by hand. The solver walks the joint graph outward from
ground, places each body to satisfy the joint to its parent, picks the remaining
free angle to best close the loops, then runs Newton. Bodies made by `copy` or
`mirror` start at their source's pose moved by that transform. Use `hint` to pick
a different branch (for example, which way a scissor opens).

Points are referenced as `'Body.point'`. Body names may contain dots (`U3.A`);
the point name is whatever follows the last dot (`U3.A.mid`).

## API reference

### `param(name, value, opts)` → number
A named number with a slider. Options: `min`, `max`, `unit`, `int`, `group`.
Giving a `group` creates that group's length-scale slider.
```js
const n = param('units', 3, { int: true, min: 1, max: 40 });
```

### `bar(name, opts)` → full body name
A straight rigid body along its local x axis. Options: `length` (required), `tags`,
`points`, `hint`. Points `a: 0`, `mid: 0.5`, `b: 1` exist by default. A number is a
fraction of the length (moves with group scaling); `[x, y, z]` is an absolute offset
in meters. The length is multiplied by the scale of every group in `tags`.
```js
bar('A0', { length: L, tags: ['tong'], points: { a: 0, mid: 0.5, b: 1, lug: [0.2, 0.03, 0] } });
```

### `plate(name, opts)` → full body name
A general rigid body given by named points (`[x, y, z]` in meters).
```js
plate('root', { points: { p1: [0, 0.05, 0], p2: [0, -0.05, 0] } });
```

### `ground(name, opts)`
Fixes a body (6 rows). Optional pose: `pos: [x, y, z]`, `rot: { axis, deg }`.
```js
ground('root', { rot: { axis: 'z', deg: -42.5 } });
```

### `pin(pointA, pointB, opts)`
Revolute joint, 5 rows. `axis` is in the first body's frame (`'x'`, `'y'`, `'z'`,
`'-z'`, or a vector), default `'z'`. The second body uses the same local axis unless
`axisB` is given.
```js
pin('A0.mid', 'B0.mid', { axis: 'z' });
```

### `ball(pointA, pointB)`
Spherical joint (rod end), 3 rows.
```js
ball('W0.a', 'A0.mid');
```

### `ujoint(pointA, pointB, opts)`
Universal joint, 4 rows: the points coincide and `axes[0]` (in body A's frame)
stays perpendicular to `axes[1]` (in body B's frame). Default `['z', 'z']`.
```js
ujoint('W0.b', 'B1.a', { axes: ['z', 'y'] });
```

### `gear(bodyA, bodyB, opts)`
Synchronizer, 1 row: `angle(A) = ratio × angle(B) + offset`. Each angle is the
signed angle of the body's local x axis about the reference body's `axis`
(default `'z'`), measured from the reference body's local x axis. The reference
is `ref`, else the body of the first `about` point, else the first grounded body.
`offset` is in radians (or degrees with `deg: true`), default 0.
```js
gear('A0', 'B0', { ratio: -1, about: ['root.p1', 'root.p2'] });
```

### `fix(bodyA, bodyB, opts)`
Weld, 6 rows. B's frame equals A's frame moved by `offset` and rotated by `rot`
(both in A's frame); by default the frames coincide.
```js
fix('A0', 'C0', { offset: [0.5, 0, 0], rot: { axis: 'z', deg: 90 } });
```

### `custom(fn, opts)`
Escape hatch for new joint types: `fn(q)` returns residual rows (zero when
satisfied). `q.point('Body.p')` and `q.axis('Body', 'z')` give world vectors,
`q.pose('Body')` gives `{ pos, quat }`, `q.t` is the time. The Jacobian is taken
by finite differences. Optional `label`.
```js
custom((q) => [q.point('U2.A.b')[1]], { label: 'tip on y = 0' });
```

### `repeat(n, fn)` → array
Builds n copies; `fn(i)` gets the copy index.
```js
const units = repeat(n, (i) => scissorUnit(`U${i}`, { L }));
```

### `chain(items, fn)`
Calls `fn(item, next, i)` for each consecutive pair.
```js
chain(units, (u, next) => {
  pin(u.A + '.b', next.B + '.a', { axis: 'z' });
  pin(u.B + '.b', next.A + '.a', { axis: 'z' });
});
```

### `copy(fn, opts)` → whatever fn returns
Runs `fn` with every body it creates renamed with `prefix`. Names used inside `fn`
refer to its own bodies first. `translate` and `rotate: { axis, deg }` move its
starting poses, grounds and hints.
```js
const lower = copy(() => lazyTong(3, L), { prefix: 'bot_', translate: [0, 0, -0.04] });
```

### `mirror(fn, opts)` → whatever fn returns
Like `copy`, reflected through `plane` (`'xy'`, `'xz'` or `'yz'`). Bars keep their
local +x direction; local y is reflected so frames stay right-handed.
```js
mirror(() => lazyTong(3, L), { plane: 'xz', prefix: 'R_' });
```

### `drive(bodyA, bodyB, opts)`
The one driver, 1 row: the signed angle of B's local x about A's `axis` (default
`'z'`, measured from A's local x) follows `profile`. A second `drive` is an error.
```js
drive('A0', 'B0', { profile: ramp({ from: 85, to: 8, deg: true, time: 10 }) });
```

### `ramp(opts)`, `sCurve(opts)`, `table(rows, opts)` → profile
`ramp` is linear from `from` to `to` over `time` seconds. `sCurve` is a half-cosine
blend with zero rate at start and stop. `table([[t, value], ...])` interpolates
linearly. Add `deg: true` for degrees.
```js
sCurve({ from: 85, to: 8, deg: true, time: 10 });
table([[0, 85], [4, 40], [10, 8]], { deg: true });
```

### Body `hint`
Optional start orientation (and position, for a body not connected to anything)
used to break ties in the initial guess.
```js
bar('U1.A', { length: L, hint: { rot: { axis: 'z', deg: 180 } } });
```

## Example models (`library.js`)

`library.js` holds ordinary functions; edit them in the app's second tab.

- `scissorUnit(name, { L, tags })`: two bars `name.A`, `name.B` pinned at their mids.
- `lazyTong(n, L, { prefix, tags })`: n units chained into a planar lazy tong.
- `gearedRoot(units, { spacing, ratio })`: a grounded root plate, the first unit's two
  root ends on fixed pins `spacing` apart, and a ratio −1 gear between the root bars.
- `webCross(...)` and `doubleTong(n, L, Lweb, { ends, ujointAxes, crossingPin })`: two
  lazy tongs (the lower one a `copy`) joined in each unit by two crossed web pairs.
  Each web runs from one tong's crossing point to the other tong's outer node, with
  ball (or universal) ends and a pin where the pair crosses.

They appear in the app's **Examples** menu:

**Lazy tong**
```js
const L = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const n = param('units', 3, { int: true, min: 1, max: 40 });
const units = lazyTong(n, L);
ground(units[0].A);
drive(units[0].A, units[0].B, { profile: sCurve({ from: 85, to: 8, deg: true, time: 10 }) });
```

**Geared root**
```js
const L = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const n = param('units', 3, { int: true, min: 1, max: 40 });
const start = param('startAngle', 85, { min: 10, max: 120, unit: 'deg' });
const units = lazyTong(n, L);
gearedRoot(units, { spacing: L * Math.sin((start / 2) * Math.PI / 180) });
drive(units[0].A, units[0].B, { profile: ramp({ from: start, to: 8, deg: true, time: 10 }) });
```

**Double tong**
```js
const L    = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const Lweb = param('Lweb', 0.6, { min: 0.51, max: 2, unit: 'm', group: 'web' });
const n    = param('units', 2, { int: true, min: 1, max: 20 });
doubleTong(n, L, Lweb, { ends: 'ball', crossingPin: true });
drive('T0.A', 'T0.B', { profile: ramp({ from: 60, to: 7.5, deg: true, time: 7 }) });
```

## Solver

- **Coordinates:** each body has a position and a unit quaternion; Newton increments
  are a translation and a world-frame rotation vector (6 per body).
- **Rows:** pin 5, ball 3, universal 4, weld 6, ground 6, gear 1, driver 1. The
  Jacobian is analytic (custom rows use finite differences) and is checked against
  finite differences in `test/jacobian.test.ts`.
- **Position:** Gauss-Newton with backtracking. Each step solves the least-squares
  problem through slightly damped normal equations, so redundant rows are fine.
  Converged when every residual is below 1e-10; 50 iterations per step (200 for
  the first assembly). On failure the sweep stops and the step is marked.
- **Mobility:** free motions = 6 × bodies − rank(J without the driver row). Rank
  counts singular values above 1e-8 × the largest. The smallest singular values and
  their vectors (the free-motion shapes) come from block inverse iteration plus a
  Rayleigh-Ritz step on J itself, so values near zero keep full precision. The
  Kutzbach count is shown for comparison only.
- **Velocity and acceleration:** least-squares solves with the same Jacobian; the
  acceleration right-hand side uses a central-difference second derivative.
- **Flags:** locked (0 free motions), Newton failure, and branch jump (any body
  rotating more than 20° between steps).

**Size and speed.** The Jacobian is stored sparse, and the normal equations are
factored with a profile (skyline) Cholesky after a reverse Cuthill-McKee ordering
(`src/solver/sparse.ts`). Measured in Node: a 100-bar lazy tong runs 41 steps in
about 2.7 s, and a 96-body double tong in about 2.8 s. Toward 500 bars, the place for a
true sparse solver is `NormalFactor` in `src/solver/sparse.ts`: swap the skyline
factor for a supernodal sparse Cholesky or a sparse rank-revealing QR (for
example SuiteSparse compiled to WebAssembly). Nothing else needs to change.

## Layout

```
src/math/vec.ts          vectors, matrices, quaternions
src/model/builder.ts     authoring API -> JSON model
src/model/library.js     editable macros (lazyTong, gearedRoot, doubleTong)
src/model/profiles.ts    ramp, sCurve, table
src/solver/system.ts     constraint rows and analytic Jacobian
src/solver/guess.ts      spanning-tree initial guess
src/solver/newton.ts     position solve
src/solver/sparse.ts     sparse least squares and smallest singular values
src/solver/sweep.ts      time stepping, mobility, velocity, acceleration, flags
src/worker.ts            Web Worker entry
src/ui/                  app wiring, three.js view, plots
test/                    spec tests 1-9 and the Jacobian check
```
