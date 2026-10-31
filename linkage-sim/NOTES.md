# Notes: test results, disagreements, and every ambiguity resolved

## Test results (`npm test`)

| # | Model | Result |
|---|---|---|
| 1 | Pin between ground and one bar | 1 free motion. **Pass.** |
| 2 | Planar four-bar (crank 0.4, coupler 1.0, rocker 0.8, ground 1.0 m), crank 30° → 150° | 1 free motion at every step; rocker angle matches the closed form (Freudenstein) within 6.9e-11 rad. **Pass.** |
| 3 | Planar lazy tong, 3 units, 6 bars, 7 pins, one bar grounded | Kutzbach = −5, rank-based free motions = 1 at all 21 steps from 85° to 8°. **Pass.** |
| 4 | Test 3 with both root bars on fixed, separate ground pins plus a −1 gear | **Locks.** 0 free motions from the first step. The start (85°) assembles exactly because the pin spacing was chosen to match it. Every later step fails, and the least-squares residual grows steadily: 1.47e-2 at 78°, 6.22e-2 at 57°, 1.14e-1 at 36°, 1.87e-1 at 8° (meters, the largest single row). Kutzbach −10. |
| 5 | Two-unit spatial double tong, ball-ended crossed webs with crossing pins | 1 free motion at 60°, 30° and 7.5° (and every step in between). **Pass.** |
| 6 | Test 5 without the web crossing pins | More than 1 free motion: 9. **Pass.** But the extra motions do **not** include sideways sway (see below). |
| 7 | Test 5 with universal-joint web ends | **Disagrees:** 1 free motion at every one of 36 steps from 60° to 7.5°, no lock (see below). |
| 8 | Group `tong` scaled by 1.2 in test 3 | Every tagged bar length × 1.2; fractional pivots keep their fraction. **Pass.** |
| 9 | `repeat` + `chain`, n = 1, 3, 20 | 2n bodies and 1 free motion at every step. **Pass.** |

Plus `test/jacobian.test.ts`: the analytic Jacobian of every joint type matches
central finite differences (worst error below 1e-6).

### Where the solver disagrees with the hand analysis

As the spec asked, the expected results for tests 6 and 7 were not changed. Each
one runs as an expected failure (`it.fails`): it prints the numbers, keeps
`npm test` green, and will fail loudly if the solver ever starts to agree.

The geometry of the double tong was not given, so the disagreements may come from
my geometry rather than from the hand analysis. Here is exactly what I built and
what I tried.

**Chosen double tong (in `library.js`).** Two identical lazy tongs. The lower one is
a `copy` of the upper one, a gap below it. In every unit there are two crossed web
pairs. Each web runs from one tong's crossing point (`A.mid`) to the other tong's
outer node of that unit (`A.b` or `B.b`). I took this from the spec's own example,
`ball('W0.a', 'A0.mid')` / `B1.a`, with `Lweb = 0.6` and `L = 1`. In a lazy tong, the
distance from a crossing point to its unit's outer nodes is always L/2, so a web of
that kind keeps a constant length, and the gap between the tongs stays constant:
sqrt(0.6² − 0.5²) = 0.332 m. The two webs of a pair cross at their midpoints, where
the crossing pin goes.

- **Test 6 (no crossing pins):** 9 free motions = the driven motion + 8 others. All 8
  are pure spins of a web bar about its own axis (one per web bar; a bar with ball
  joints at both ends can always spin). The lower tong's largest sideways component
  in those motions is 5e-17, which is zero. So in this geometry the crossing pins only
  remove the harmless web spins.
- **Test 7 (universal-joint ends):** with `axes: ['z', 'z']` (the web's crossing-pin
  axis perpendicular to the tong bar's pin axis), the smallest nonzero singular value
  falls smoothly from 0.108 at 60° to 0.0187 at 7.5°. It never reaches the threshold,
  so there is no lock. With the spec's example axes `['z', 'y']`, the model cannot even
  be assembled: the residual stays at 0.05, because the web's pin axis cannot be
  perpendicular to the tong bar's in-plane axis in this geometry.

**Other web layouts I tried** (2 units, 60° → 7.5°, free motions):

| Layout | Ball + crossing pins | Ball, no pins | U-joints + pins |
|---|---|---|---|
| X across each station (+y node to the other tong's −y node) | 2: the lower tong swings fore-and-aft like a parallelogram | — | — |
| X in each side face, between stations k and k+1 (Lweb 1.2) | 2: both side faces lean together, so the lower tong sways sideways | — | — |
| Box diagonals of each unit, crossing at the unit center (Lweb 1.2) | 2 | 5 (did not assemble) | 1 |
| Crossing point → 2 outer nodes (**chosen**) | 1 | 10 | 1 |
| Crossing point → all 4 nodes | 1 | 17 | 1 |

No layout I tried locks with universal-joint ends. The side-face layout is the only
one with sideways sway, and it sways even *with* crossing pins. If you send the
geometry used in the hand analysis, it can be modeled directly with the same API.

## Ambiguities and the choice made

### Repository
- "Sub repo" was read as a folder (`linkage-sim/`) inside the site repository, not a
  separate repository or a git submodule. It is meant to be run locally.

### Authoring API
- **Second body's pin axis.** The spec gives the axis in the first body's frame only.
  The second body uses the same local vector unless `axisB` is given.
- **Gear angle.** "Rotation of A about its pivot" is the signed angle of A's local x
  axis about the reference body's axis (default `'z'`), measured from the reference
  body's local x axis. `ratio` and an `offset` (default 0) relate the two angles.
  Angles do not depend on where the pivot is, so `about` is used only to pick the
  default reference body (the body of the first `about` point) and to place the
  gear symbol. With no `about` or `ref`, the reference is the first grounded body.
  The row wraps angles to (−π, π], so gear angles beyond ±180° are not supported.
- **Driver angle.** The signed angle of B's local x axis about A's axis (default z),
  measured from A's local x. A model without a driver is solved once, at t = 0.
- **`fix`.** B's frame equals A's frame, optionally moved by `offset` and rotated by
  `rot` (both in A's frame).
- **`ground`.** Default pose is the origin with identity rotation; `pos` and `rot`
  are optional.
- **`ujoint` axes.** Default `['z', 'z']`; the spec always passes them.
- **Default bar points.** Every bar has `a: 0`, `mid: 0.5`, `b: 1` unless overridden.
- **Groups.** A group exists when some `param` names it. A bar is scaled by every
  group in its tags. Absolute `[x, y, z]` points are not scaled (they are offsets in
  meters), and `param` values themselves are not scaled.
- **Names with dots.** `'U3.A.mid'` splits at the last dot: body `U3.A`, point `mid`.
- **`bar`/`plate` return the full (prefixed) body name**, so macros can build
  references from return values.
- **`copy`.** The transform moves the copy's *starting pose* (each copied body starts
  at its source body's pose moved by the transform), its grounds and its hints.
  The solver then finds the real pose. For nested copies, the source is the same
  name in the enclosing scope, and only the innermost transform is used for the
  start pose.
- **`mirror`.** A reflection is not a rigid motion, so mirrored bodies keep their local
  +x direction and reflect local y. Their frames stay right-handed, and
  measured angles change sign. Gear offsets are negated to match.
- **`hint`.** A pose `{ pos, rot }`. It fixes the orientation of that body's tree joint
  (as closely as the joint allows). The position is used only for a body with no
  joints to anything already placed.
- **`custom`.** `q` gives `point`, `axis`, `pose` and `t`. The Jacobian is taken by
  central differences. In saved JSON the function is stored as source text and
  revived on load, so variables it captured from the script are lost.
- **Profiles.** `sCurve` is a half-cosine blend (zero rate at start and end). Time is
  in seconds. The time steps are evenly spaced, and their number is set in the
  header (default 41). That control is needed but not in the spec.

### Solver
- **Least squares without SVD.** The spec suggests SVD or QR with column pivoting.
  A dense SVD of a 100-bar Jacobian (747 × 600) took about 3 s (measured in
  Node), so a sweep took minutes and was nowhere near live. The current version
  stores J sparse and does this instead:
  - **Newton step:** the least-squares problem is solved through the normal
    equations with a small damping term (1e-12 relative), factored by a
    profile Cholesky. Redundant rows are handled because the damping keeps the
    factorization positive definite.
  - **Rank:** still a singular-value threshold. The smallest singular values of J
    come from block inverse iteration followed by a Rayleigh-Ritz step on J itself
    (a small dense SVD of J·V), so a zero singular value comes out near 1e-16, not
    near 1e-8 as it would through JᵀJ. The block grows until it reaches past the
    null space.

  The result is 100 bars × 41 steps in about 3 s.
- **Rank threshold.** 1e-8 × max(largest singular value, 1). The rows mix meters
  (point coincidence) and dimensionless or radian rows (axes, angles), so for bars far
  from about 1 m long the scale matters. The threshold is `rankTolRel` in
  `DEFAULT_SWEEP`.
- **Convergence test.** Every residual must be below 1e-10, whatever its unit, not
  just the meter rows. The first assembly is allowed 200 iterations instead of 50,
  because it starts from the tree guess rather than the previous step.
- **Initial guess.** Beyond the spec's spanning tree, three choices: (1) the tree
  prefers stiff joints (welds, then pins, then universals, then balls); (2) the
  free parameter of each tree joint is picked by a scan of its angle (or, for ball
  joints, over 96 directions plus a local search) to best satisfy the other
  constraints on that body; (3) the tree is grown 6 bodies at a time, and every
  closed loop is solved with Newton before more bodies are added. Without (3), a
  50-unit tong did not assemble. If the copy-based start fails, the plain tree
  guess is tried too.
- **Branch-jump limit.** 20° of rotation of any body between consecutive steps. With
  very few steps, normal motion can trip it; use more steps.
- **Velocity and acceleration.** Minimum-norm least-squares solutions. When there are
  extra free motions (e.g. web spins), those motions get zero velocity. The
  acceleration's curvature term is a central difference along the velocity.
- **Kutzbach count.** 6 × (bodies − grounded bodies) − the rows of every joint except
  ground and the driver. It is shown for comparison only and never used.
- **Mobility with the driver removed** is computed at the solved pose. If Newton
  failed, the mobility shown is at the best least-squares pose, which is what test 4
  reports.

### Test models
- **Test 3.** `U0.A` is grounded with identity pose; the driver is the angle between
  `U0.A` and `U0.B`, 85° → 8°.
- **Test 4.** The root plate is grounded. `U0.A.a` and `U0.B.a` are pinned to plate
  points 0.6756 m apart (L·sin(42.5°), which fits the 85° start). The gear is
  `ratio −1` between `U0.A` and `U0.B`, measured on the root plate. The bars cross at
  their mids as in test 3, so the root triangle (two half-bars and the fixed pin
  spacing) is rigid. That is why it locks.
- **Test 5–7 sampling.** The driver runs 60° → 7.5° in 7.5° steps, so 60°, 30° and
  7.5° are exact samples. Test 7 uses 1.5° steps.
- **Test 6 "sideways".** In the tong plane, perpendicular to the line from the root of
  `T0.A` to the root of `T1.A`. The check is the largest sideways motion of the
  lower tong's centroid over all motions with the driver held.

### UI
- Plots can use driven angle (in degrees) or time on the x axis. Picked points plot
  x, y, z in meters; pin angles are picked from a menu and plotted in degrees.
- The model script and `library.js` are saved in the browser's local storage, for
  convenience only.
- Free-motion shapes are animated at about 6% of the model size.
