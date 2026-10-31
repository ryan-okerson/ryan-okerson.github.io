# Prompt for Claude Code: Web-Based 3D Linkage Motion Simulator

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

---

## Goal

Build a browser-based 3D kinematic simulator for overconstrained spatial linkages, mainly scissor (pantograph) deployable structures for spacecraft solar arrays.

The user describes bodies, joints, gear synchronizers and one driven motion profile. The software assembles the constraint equations and Jacobian, solves the motion, checks mobility, and animates the result.

**The top priority is input ergonomics:**
1. Linkage configurations must be very fast to type.
2. Repeating patterns of links (units of a scissor, mirrored or offset copies) must take one or two lines.
3. Lengths of whole groups of links must be changeable in one place, both in code and from a slider.

## Hard constraints
- Runs fully in the browser as a static site, with no backend. Use Vite + TypeScript.
- **No sliding (prismatic) joints in the first version.** The target hardware has none. Leave room to add one later.
- Units internally: meters and radians. The user interface shows degrees, with the unit labeled.
- Must handle overconstrained mechanisms. The Kutzbach count is often negative for these, so never use it to decide anything. Use the numerical rank of the constraint Jacobian.
- Don't look up or copy reference designs from pantograph deployer patents or papers. Build from first principles.

---

## 1. Model input: a small JavaScript authoring API

The model is a short script typed in an in-browser code editor (CodeMirror is fine). A script beats YAML here because loops, functions and arithmetic come for free. The script calls a small API and produces a plain JSON model, which the solver consumes. Also allow loading and saving the JSON.

### Parameters and groups
```js
const L    = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
const Lweb = param('Lweb', 0.6, { min: 0.1, max: 2, unit: 'm', group: 'web' });
const n    = param('units', 3, { int: true, min: 1, max: 40 });
```
- Every `param` automatically gets a slider in a side panel.
- Every `group` automatically gets a **scale slider** (default 1.0) that multiplies the lengths of all bars tagged with that group.
- Changing a slider re-runs the script and re-solves. It should feel live for models up to about 100 bars.

### Bodies
```js
bar('A0', { length: L, tags: ['tong'], points: { a: 0, mid: 0.5, b: 1 } });
plate('root', { points: { p1: [0, 0.05, 0], p2: [0, -0.05, 0] } });
```
- A `bar` is a straight rigid body along its local x axis.
- A point given as a single number is a **fraction of the bar length**. That way pivots move proportionally when a group is rescaled.
- A point given as `[x, y, z]` is an absolute offset in meters, for out-of-axis or offset pivots.
- `plate` is a general rigid body defined by named points.
- Tags are free text. The tags drive group scaling and color in the 3D view.

### Joints
Points are referenced as `'Body.point'`.
```js
ground('root');
pin('A0.mid', 'B0.mid', { axis: 'z' });              // revolute joint
ball('W0.a', 'A0.mid');                              // spherical joint (rod end)
ujoint('W0.b', 'B1.a', { axes: ['z', 'y'] });         // two-axis universal joint
gear('A0', 'B0', { ratio: -1, about: ['root.p1', 'root.p2'] });  // synchronizer
fix('A0', 'C0');                                      // weld two bodies together
custom((q) => [/* residual rows */]);                 // escape hatch for new joint types
```
- Pin axes are given in the first body's local frame, as `'x'`, `'y'`, `'z'` or a vector.
- **Gear synchronizer:** rotation of body A about its pivot = ratio × rotation of body B about its pivot. Both rotations are measured relative to a named reference body (default: ground) about each pin's axis. This is one constraint row. It must work at the root and tip of a scissor with fixed pins and no slider.

### Repeating patterns
Patterns are plain functions. Ship these helpers:
```js
// Build n copies; i is the copy index. Returns an array of whatever the callback returns.
const units = repeat(n, (i) => scissorUnit(`U${i}`, { L }));

// Link consecutive copies: callback receives (unit i, unit i+1).
chain(units, (u, next) => {
  pin(u.A + '.b', next.B + '.a', { axis: 'z' });
  pin(u.B + '.b', next.A + '.a', { axis: 'z' });
});

// Copy a whole sub-model with a rigid transform, renaming bodies with a prefix.
const lower = copy(() => buildTong('top'), { prefix: 'bot_', translate: [0, 0, -0.04] });
mirror(() => buildTong('L'), { plane: 'xz', prefix: 'R_' });
```
- `repeat`, `chain`, `copy` and `mirror` are all that's needed. The user writes macros like `scissorUnit` and `lazyTong` as ordinary functions.
- Ship a `library.js` with `scissorUnit`, `lazyTong(n, L)` and `gearedRoot(...)` as worked examples. The user can edit these.
- Indexed names like `U3.A` must show in the 3D view on hover, so the user can find a body by clicking it.

### Driver and motion profile
```js
drive('A0', 'B0', {                       // relative angle between two bodies
  profile: ramp({ from: 85, to: 8, deg: true, time: 10 }),
});
```
- Profiles: `ramp`, `sCurve` (smooth start and stop), and `table([[t, value], ...])` with linear interpolation.
- One driver for now. Two drivers on one model is an error.

### Initial guess
The user must **not** have to place every body by hand.
- Build the first guess by walking the joint graph outward from ground (a spanning tree). Place each body so it satisfies the joint that connects it to its parent. Use the driver's starting angle for the driven joint.
- The user can give an optional `hint` pose on any body to break ties, for example flipping which way a scissor opens.
- Then run the full Newton solve.

---

## 2. Solver

**Coordinates:** each body has a position (3 numbers) and an orientation stored as a quaternion. Newton increments use a small rotation vector (3 numbers), so each body contributes 6 increments.

**Constraint rows per joint:**
- Pin: 3 rows (points coincide) + 2 rows (axes stay parallel) = 5 rows
- Ball: 3 rows
- Universal joint: 3 rows + 1 row (the two axes stay perpendicular) = 4 rows
- Fixed weld: 6 rows
- Ground: 6 rows
- Gear: 1 row
- Driver: 1 row

**Each time step:**
1. **Position:** Newton iteration on all constraint rows. Use a **least-squares** step through a singular value decomposition, or a QR decomposition with column pivoting, because redundant rows make the Jacobian rank-deficient. Plain inversion will fail. Converge when the largest residual is below 1e-10 meters, with a 50-iteration cap. On failure, log it, stop the sweep, and mark the step in the plots.
2. **Mobility:** free motions = (6 × number of bodies) − (rank of the Jacobian, with the driver row removed). Compute rank with a singular value threshold, and show the smallest few singular values. A value near the threshold means near-singular.
3. **Velocity and acceleration:** linear least-squares solves with the same Jacobian.
4. **Lock and branch detection:** flag any step where free motions drop to 0, Newton fails, or the solution jumps branch (any body rotates more than a set limit between steps).

**Size:** dense linear algebra is fine for the first version. Use `ml-matrix` or an equivalent. The target is 100 bars, which is 600 coordinates. Run the solver in a Web Worker so the user interface never freezes. Note in the README where a sparse solver would go when models reach 500 bars.

---

## 3. Output and user interface
- **3D view** using three.js: orbit controls, bars as lines or thin boxes colored by tag, joint symbols by type (pin, ball, universal, gear). Include a time scrubber and play/pause.
- **Plots** versus the driven angle and versus time:
  - Free motions
  - Smallest singular values
  - Any user-selected joint angle or point coordinate (click a point in the 3D view to add it)
  - Overall extent: length, width and depth of the bounding box
- **Mobility panel** at the current step: Kutzbach count shown next to the rank result. Also show the free-motion shapes (null-space vectors), animated as small wiggles, so the user can see *which* motion is free. This is how unwanted sway or a harmless link spin gets found.
- **Export:** model JSON, a CSV of every plotted quantity, and a PNG of the current view.

---

## 4. Tests (write these first)

Each test builds a model with the authoring API, solves it, and checks the result.

| # | Model | Expected result |
|---|---|---|
| 1 | Single pin between ground and one bar | 1 free motion |
| 2 | Planar four-bar linkage | 1 free motion; matches the closed-form output angle within 1e-9 radians |
| 3 | Planar lazy tong, 3 units, 6 bars, 7 pins, one bar grounded | Kutzbach count = 6 × (6 − 1) − 5 × 7 = −5. Rank-based free motions = 1 |
| 4 | Test 3, with the two root bars on fixed separate pins and a gear (ratio −1) instead of a free end | Report whether it moves or locks, and the residual through the stroke. **Do not assume the answer**; print it |
| 5 | Two-unit spatial "double tong": two offset lazy tongs joined by crossed web links with ball ends and web crossing pins | 1 free motion at driven angles 60°, 30° and 7.5° |
| 6 | Test 5 with the web crossing pins removed | More than 1 free motion; the extra motions include sideways sway of the lower tong |
| 7 | Test 5 with web ends as universal joints instead of ball joints | Expected to lock (0 free motions) at some angle; report where |
| 8 | Group scaling: scale group `tong` by 1.2 in test 3 | Every tagged bar length × 1.2; fractional pivot points stay at the same fraction |
| 9 | `repeat` + `chain` with n = 1, 3 and 20 | Correct body count (2 × n for a lazy tong), 1 free motion every time |

Tests 5 to 7 reproduce earlier hand analysis of a 3D scissor. If the solver disagrees, report the disagreement with numbers rather than adjusting the test.

---

## 5. Phases

**Phase 1 (build now):** authoring API, solver, mobility, 3D view, plots, all tests passing.

**Phase 2 (only after Phase 1 is reviewed):** quasi-static loads.
- Give every joint a stiffness (translational and rotational springs) instead of an exact constraint. This makes the joint reactions of an overconstrained mechanism unique.
- Applied loads: torsion springs in joints (preload + rate), a damper, and a constant tension force between two points (models the solar blanket).
- Outputs versus stroke: required driver torque, torque margin, every joint reaction force, and the forced mismatch (strain) in nominally compatible geometry. Test 4 should then report a mismatch number in millimeters and newtons.

**Out of scope:** forward dynamics (time-stepping deployment transients), contact, pin clearance, flexible links, collision between bodies.

---

## 6. Deliverables
- Repository with `npm run dev`, `npm test` and `npm run build` (static output).
- `README.md` with the coding principles block above, then: how to write a model, the full API reference with one example per function, and the three example models from `library.js` (lazy tong, geared root, double tong).
- A short note listing every ambiguity you hit and the choice you made.
