// Initial guess: walk a spanning tree of the joint graph outward from ground.
// Each child is placed so it satisfies the joint to its parent; the one free
// parameter of that joint is chosen to best satisfy every other constraint
// touching the child (a few Gauss-Seidel passes). Newton then finishes the job.
import {
  type Mat3, type Quat, type Vec3,
  EX, add, matMul, matMulVec, quatFromMat, quatAngleBetween, quatConj, quatFromAxisAngle, quatFromTwoVectors, quatMul, quatToMat, sub,
  QUAT_IDENTITY,
} from '../math/vec';
import type { JointDef, Model } from '../model/types';
import { type Constraint, type Frame, type Pose, type System, evaluateConstraint } from './system';
import { sumSquares } from './sparse';
import { solvePositions } from './newton';

type TreeJoint = Extract<JointDef, { type: 'pin' | 'ball' | 'ujoint' | 'fix' }>;

interface TreeEdge {
  parent: number;
  child: number;
  joint: TreeJoint;
  // True when the child is the joint's "a" side.
  childIsA: boolean;
  // Orientation of the tree's root, used as an alternative base for ball joints.
  root: number;
  locked: boolean;
  angle: number;
  quat: Quat | null;
}

const JOINT_WEIGHT: Record<TreeJoint['type'], number> = { fix: 0, pin: 1, ujoint: 2, ball: 3 };
const PASSES = 4;
const GROW_CHUNK = 6;
const COARSE_SAMPLES = 72;
const BALL_DIRECTIONS = fibonacciSphere(96);

// Candidate starting poses, best first: with copies placed from their sources, then without.
export function initialGuesses(system: System, t0: number): Pose[][] {
  const withCopies = initialGuess(system, t0, true);
  const hasCopies = system.model.bodies.some((b) => b.copyOf);
  return hasCopies ? [withCopies, initialGuess(system, t0, false)] : [withCopies];
}

export function initialGuess(system: System, t0: number, useCopies = true): Pose[] {
  const model = system.model;
  const n = system.bodyNames.length;
  const frames: Frame[] = Array.from({ length: n }, () => toFrame({ r: [0, 0, 0], q: QUAT_IDENTITY }));
  const { edges, roots } = spanningTree(system);
  for (const [body, pose] of roots) frames[body] = toFrame(pose);

  const touching = constraintsByBody(system);
  const placed = new Set(roots.keys());

  // Stiff joints first (pins, welds, gears, driver), so ball-jointed links do not drag them.
  const stiff = touching.map((list) => list.filter((c) => c.kind !== 'ball' && c.kind !== 'ujoint'));
  growAssembly(system, edges, frames, stiff, placed, t0);
  if (!useCopies) {
    relax(model, edges, frames, touching, placed, t0, new Set());
  } else {
    // Copies start at their source's pose moved by the copy transform; place the rest around them.
    const held = placeCopies(system, frames);
    relax(model, edges, frames, touching, placed, t0, held);
  }
  return frames.map((f) => ({ r: f.r, q: f.q }));
}

// Place the tree a few bodies at a time, closing every loop among the placed
// bodies with Newton before adding more. Long chains then never drift far.
function growAssembly(system: System, edges: TreeEdge[], frames: Frame[], touching: Constraint[][], placed: Set<number>, t0: number): void {
  const model = system.model;
  for (let start = 0; start < edges.length; start += GROW_CHUNK) {
    const chunk = edges.slice(start, start + GROW_CHUNK);
    for (let pass = 0; pass < 2; pass++) {
      for (const edge of chunk) {
        placed.add(edge.child);
        const hint = model.bodies[edge.child].hint;
        if (hint) lockToHint(model, edge, frames, hint.quat);
        if (!edge.locked) chooseParameter(model, edge, frames, touching[edge.child], placed, t0);
        frames[edge.child] = placeChild(model, edge, frames);
      }
    }
    solveAmong(system, frames, placed, t0);
  }
}

// Newton on the stiff constraints whose bodies are all placed; other bodies stay put.
function solveAmong(system: System, frames: Frame[], placed: Set<number>, t0: number): void {
  const constraints = system.constraints.filter((c) =>
    c.kind !== 'ball' && c.kind !== 'ujoint' && c.bodies.every((b) => b < 0 || placed.has(b)));
  if (constraints.length === 0) return;
  const rowCount = constraints.reduce((sum, c) => sum + c.rows, 0);
  const subsystem: System = { ...system, constraints, rowCount, driverRow: -1 };
  const poses = frames.map((f) => ({ r: f.r, q: f.q }));
  const result = solvePositions(subsystem, poses, t0, { tolerance: 1e-12, maxIterations: 50 });
  result.poses.forEach((p, i) => (frames[i] = toFrame(p)));
}

// Gauss-Seidel passes over the tree edges. Bodies in `fixed` are never moved.
function relax(model: Model, edges: TreeEdge[], frames: Frame[], touching: Constraint[][], placed: Set<number>, t0: number, fixed: Set<number>): void {
  const hintOf = (i: number) => model.bodies[i].hint;
  for (let pass = 0; pass < PASSES; pass++) {
    for (const edge of edges) {
      if (fixed.has(edge.child)) continue;
      placed.add(edge.child);
      const hint = hintOf(edge.child);
      if (hint) lockToHint(model, edge, frames, hint.quat);
      if (!edge.locked) chooseParameter(model, edge, frames, touching[edge.child], placed, t0);
      frames[edge.child] = placeChild(model, edge, frames);
    }
  }
}

// Returns the copies and their sources, which are then held still.
function placeCopies(system: System, frames: Frame[]): Set<number> {
  const held = new Set<number>();
  system.model.bodies.forEach((body, i) => {
    const c = body.copyOf;
    if (!c) return;
    const sourceIndex = system.bodyIndex.get(c.source)!;
    const source = frames[sourceIndex];
    const linear = c.linear as Mat3;
    let R = matMul(linear, source.R);
    if (c.improper) R = matMul(R, LOCAL_MIRROR);
    const q = quatFromMat(R);
    frames[i] = { r: add(matMulVec(linear, source.r), c.shift), q, R: quatToMat(q) };
    held.add(i);
    held.add(sourceIndex);
  });
  return held;
}

const LOCAL_MIRROR: Mat3 = [1, 0, 0, 0, -1, 0, 0, 0, 1];

function toFrame(p: Pose): Frame {
  return { r: p.r, q: p.q, R: quatToMat(p.q) };
}

// Prim-style tree: prefer stiff joints (fix, pin) as tree edges.
function spanningTree(system: System): { edges: TreeEdge[]; roots: Map<number, Pose> } {
  const model = system.model;
  const idx = (name: string) => system.bodyIndex.get(name)!;
  const roots = new Map<number, Pose>();
  const rootOf = new Map<number, number>();
  for (const j of model.joints) {
    if (j.type !== 'ground') continue;
    const b = idx(j.body);
    roots.set(b, { r: j.pose.pos, q: j.pose.quat });
    rootOf.set(b, b);
  }
  const links = model.joints.filter((j): j is TreeJoint => j.type === 'pin' || j.type === 'ball' || j.type === 'ujoint' || j.type === 'fix');
  const bodiesOf = (j: TreeJoint): [number, number] => (j.type === 'fix' ? [idx(j.a), idx(j.b)] : [idx(j.a.body), idx(j.b.body)]);
  const edges: TreeEdge[] = [];
  const n = system.bodyNames.length;

  while (rootOf.size < n) {
    let best: TreeEdge | null = null;
    let bestWeight = Infinity;
    for (const j of links) {
      const [a, b] = bodiesOf(j);
      if (a === b) continue;
      const aIn = rootOf.has(a);
      const bIn = rootOf.has(b);
      if (aIn === bIn) continue;
      const w = JOINT_WEIGHT[j.type];
      if (w >= bestWeight) continue;
      bestWeight = w;
      const parent = aIn ? a : b;
      best = { parent, child: aIn ? b : a, joint: j, childIsA: !aIn, root: rootOf.get(parent)!, locked: j.type === 'fix', angle: 0, quat: null };
    }
    if (best) {
      edges.push(best);
      rootOf.set(best.child, best.root);
      continue;
    }
    // Nothing reachable: start a new tree at the first unplaced body.
    const free = Array.from({ length: n }, (_, i) => i).find((i) => !rootOf.has(i))!;
    const hint = model.bodies[free].hint;
    roots.set(free, hint ? { r: hint.pos, q: hint.quat } : { r: [0, 0, 0], q: QUAT_IDENTITY });
    rootOf.set(free, free);
  }
  return { edges, roots };
}

function constraintsByBody(system: System): Constraint[][] {
  const out: Constraint[][] = system.bodyNames.map(() => []);
  for (const c of system.constraints) {
    for (const b of new Set(c.bodies)) if (b >= 0) out[b].push(c);
  }
  return out;
}

// Child frame given its parent's frame and the edge's free parameter.
function placeChild(model: Model, edge: TreeEdge, frames: Frame[]): Frame {
  const parent = frames[edge.parent];
  const j = edge.joint;
  if (j.type === 'fix') {
    if (!edge.childIsA) {
      const q = quatMul(parent.q, j.quat);
      return toFrame({ r: add(parent.r, matMulVec(parent.R, j.offset)), q });
    }
    const q = quatMul(parent.q, quatConj(j.quat));
    const R = quatToMat(q);
    return { r: sub(parent.r, matMulVec(R, j.offset)), q, R };
  }
  const parentRef = edge.childIsA ? j.b : j.a;
  const childRef = edge.childIsA ? j.a : j.b;
  const parentPoint = model.bodies[edge.parent].points[parentRef.point];
  const childPoint = model.bodies[edge.child].points[childRef.point];
  const q = j.type === 'pin' ? pinOrientation(edge, parent) : (edge.quat ?? parent.q);
  const R = quatToMat(q);
  const world = add(parent.r, matMulVec(parent.R, parentPoint));
  return { r: sub(world, matMulVec(R, childPoint)), q, R };
}

function pinOrientation(edge: TreeEdge, parent: Frame): Quat {
  const j = edge.joint as Extract<TreeJoint, { type: 'pin' }>;
  const parentAxis = edge.childIsA ? j.axisB : j.axisA;
  const childAxis = edge.childIsA ? j.axisA : j.axisB;
  const axisWorld = matMulVec(parent.R, parentAxis);
  const base = quatMul(quatFromTwoVectors(matMulVec(parent.R, childAxis), axisWorld), parent.q);
  return quatMul(quatFromAxisAngle(axisWorld, edge.angle), base);
}

// Candidate orientations for a ball or universal joint child.
function ballCandidates(parent: Frame, root: Frame): Quat[] {
  const out: Quat[] = [parent.q, root.q];
  for (const base of [parent, root]) {
    const x = matMulVec(base.R, EX);
    for (const d of BALL_DIRECTIONS) out.push(quatMul(quatFromTwoVectors(x, d), base.q));
  }
  return out;
}

// Pick the edge parameter that minimizes the residual of constraints touching the child.
function chooseParameter(model: Model, edge: TreeEdge, frames: Frame[], touching: Constraint[], placed: Set<number>, t: number): void {
  const active = touching.filter((c) => c.bodies.every((b) => b < 0 || placed.has(b)));
  if (active.length === 0) return;
  const cost = () => {
    frames[edge.child] = placeChild(model, edge, frames);
    let s = 0;
    for (const c of active) s += sumSquares(evaluateConstraint(c, frames, t));
    return s;
  };
  if (edge.joint.type === 'pin') {
    edge.angle = bestAngle(edge, cost);
    return;
  }
  let bestCost = Infinity;
  let bestQuat: Quat | null = edge.quat;
  for (const q of ballCandidates(frames[edge.parent], frames[edge.root])) {
    edge.quat = q;
    const c = cost();
    if (c < bestCost) {
      bestCost = c;
      bestQuat = q;
    }
  }
  edge.quat = refineOrientation(edge, bestQuat!, bestCost, cost);
}

// Pattern search: small world-axis rotations, shrinking step, keep improvements.
function refineOrientation(edge: TreeEdge, start: Quat, startCost: number, cost: () => number): Quat {
  let best = start;
  let bestCost = startCost;
  for (const step of [0.2, 0.1, 0.05, 0.02, 0.01, 0.005]) {
    let improved = true;
    while (improved) {
      improved = false;
      for (const axis of [[1, 0, 0], [0, 1, 0], [0, 0, 1]] as Vec3[]) {
        for (const sign of [1, -1]) {
          edge.quat = quatMul(quatFromAxisAngle(axis, sign * step), best);
          const c = cost();
          if (c >= bestCost - 1e-15) continue;
          bestCost = c;
          best = edge.quat;
          improved = true;
        }
      }
    }
  }
  return best;
}

// Coarse scan over the full turn, then a fine scan around the best sample.
function bestAngle(edge: TreeEdge, cost: () => number): number {
  let best = edge.angle;
  let bestCost = cost();
  const tryAngle = (a: number) => {
    edge.angle = a;
    const c = cost();
    if (c < bestCost - 1e-14) {
      bestCost = c;
      best = a;
    }
  };
  for (let k = 0; k < COARSE_SAMPLES; k++) tryAngle(-Math.PI + (2 * Math.PI * k) / COARSE_SAMPLES);
  const center = best;
  const span = (2 * Math.PI) / COARSE_SAMPLES;
  for (let k = -10; k <= 10; k++) tryAngle(center + (span * k) / 10);
  edge.angle = best;
  return best;
}

// A hint fixes the child's orientation (as closely as the parent joint allows).
function lockToHint(model: Model, edge: TreeEdge, frames: Frame[], target: Quat): void {
  if (edge.joint.type === 'fix') return;
  edge.locked = true;
  if (edge.joint.type !== 'pin') {
    edge.quat = target;
    return;
  }
  let best = 0;
  let bestDist = Infinity;
  for (let k = 0; k < 720; k++) {
    edge.angle = -Math.PI + (2 * Math.PI * k) / 720;
    const d = quatAngleBetween(placeChild(model, edge, frames).q, target);
    if (d < bestDist) {
      bestDist = d;
      best = edge.angle;
    }
  }
  edge.angle = best;
}

function fibonacciSphere(count: number): Vec3[] {
  const out: Vec3[] = [];
  const golden = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < count; i++) {
    const y = 1 - (2 * (i + 0.5)) / count;
    const r = Math.sqrt(1 - y * y);
    out.push([Math.cos(golden * i) * r, y, Math.sin(golden * i) * r]);
  }
  return out;
}
