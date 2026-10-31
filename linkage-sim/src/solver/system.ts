// Constraint equations and their analytic Jacobian.
// Each body has 6 increment columns: [dr (3), dtheta (3)], dtheta a world-frame rotation vector.
import { SparseJacobian } from './sparse';
import {
  type Mat3, type Quat, type Vec3,
  EX, add, cross, dot, matMulVec, normalize, perpendicular, quatConj, quatFromRotVec, quatMul, quatNormalize,
  quatToMat, scale, sub, QUAT_IDENTITY,
} from '../math/vec';
import { sampleProfile } from '../model/profiles';
import type { CustomAccess, JointDef, Model, PointRef } from '../model/types';
import { parseAxis } from '../model/builder';

export interface Pose {
  r: Vec3;
  q: Quat;
}

export interface Frame extends Pose {
  R: Mat3;
}

// Adds value to entry (row, col) of the Jacobian.
export type JacobianWriter = (row: number, col: number, value: number) => void;

export interface Constraint {
  kind: string;
  label: string;
  rows: number;
  // Body indices whose poses this constraint reads (-1 = world).
  bodies: number[];
  isDriver: boolean;
  evaluate(frames: Frame[], t: number, res: Float64Array, jac: JacobianWriter | null, row: number): void;
}

export interface System {
  model: Model;
  bodyNames: string[];
  bodyIndex: Map<string, number>;
  constraints: Constraint[];
  rowCount: number;
  // Row of the driver constraint, or -1 without a driver.
  driverRow: number;
  columnCount: number;
}

const WORLD = -1;
const WORLD_FRAME: Frame = { r: [0, 0, 0], q: QUAT_IDENTITY, R: quatToMat(QUAT_IDENTITY) };

export function compileSystem(model: Model): System {
  const bodyNames = model.bodies.map((b) => b.name);
  const bodyIndex = new Map(bodyNames.map((name, i) => [name, i]));
  const constraints: Constraint[] = [];
  const firstGround = model.joints.find((j) => j.type === 'ground');
  const defaultRef = firstGround && firstGround.type === 'ground' ? bodyIndex.get(firstGround.body)! : WORLD;
  const localPoint = (ref: PointRef) => model.bodies[bodyIndex.get(ref.body)!].points[ref.point];
  const points = Object.fromEntries(model.bodies.map((b) => [b.name, b.points]));

  for (const joint of model.joints) {
    const c = compileJoint(joint, bodyIndex, localPoint, defaultRef, points);
    if (c) constraints.push(c);
  }
  let driverRow = -1;
  if (model.driver) {
    const d = model.driver;
    const a = bodyIndex.get(d.a)!;
    const b = bodyIndex.get(d.b)!;
    constraints.push(angleConstraint('driver', `drive ${d.a} / ${d.b}`, a, b, d.axis, (t) => sampleProfile(d.profile, t).value, true));
  }
  let rowCount = 0;
  for (const c of constraints) {
    if (c.isDriver) driverRow = rowCount;
    rowCount += c.rows;
  }
  return { model, bodyNames, bodyIndex, constraints, rowCount, driverRow, columnCount: 6 * bodyNames.length };
}

function compileJoint(
  joint: JointDef,
  bodyIndex: Map<string, number>,
  localPoint: (ref: PointRef) => Vec3,
  defaultRef: number,
  points: PointTable,
): Constraint | null {
  const idx = (name: string) => bodyIndex.get(name)!;
  if (joint.type === 'ground') return groundConstraint(idx(joint.body), joint.pose.pos, joint.pose.quat, joint.body);
  if (joint.type === 'pin') {
    return pinConstraint(idx(joint.a.body), localPoint(joint.a), joint.axisA, idx(joint.b.body), localPoint(joint.b), joint.axisB, `pin ${refName(joint.a)} - ${refName(joint.b)}`);
  }
  if (joint.type === 'ball') {
    return pointConstraint('ball', `ball ${refName(joint.a)} - ${refName(joint.b)}`, idx(joint.a.body), localPoint(joint.a), idx(joint.b.body), localPoint(joint.b));
  }
  if (joint.type === 'ujoint') {
    return ujointConstraint(idx(joint.a.body), localPoint(joint.a), joint.axisA, idx(joint.b.body), localPoint(joint.b), joint.axisB, `ujoint ${refName(joint.a)} - ${refName(joint.b)}`);
  }
  if (joint.type === 'fix') return fixConstraint(idx(joint.a), idx(joint.b), joint.offset, joint.quat, `fix ${joint.a} - ${joint.b}`);
  if (joint.type === 'gear') {
    const ref = joint.ref ? idx(joint.ref) : defaultRef;
    return gearConstraint(idx(joint.a), idx(joint.b), ref, joint.axis, joint.ratio, joint.offset, `gear ${joint.a} / ${joint.b}`);
  }
  if (joint.type === 'custom') return customConstraint(joint.label, joint.fn, bodyIndex, points);
  return null;
}

function refName(ref: PointRef): string {
  return `${ref.body}.${ref.point}`;
}

export function framesFromPoses(poses: Pose[]): Frame[] {
  return poses.map((p) => ({ r: p.r, q: p.q, R: quatToMat(p.q) }));
}

function frameOf(frames: Frame[], i: number): Frame {
  return i === WORLD ? WORLD_FRAME : frames[i];
}

// Write a 3-vector gradient into the translation or rotation columns of a body.
function writeGrad(jac: JacobianWriter, row: number, body: number, block: 0 | 3, g: Vec3, sign = 1): void {
  if (body === WORLD) return;
  const col = 6 * body + block;
  if (g[0] !== 0) jac(row, col, sign * g[0]);
  if (g[1] !== 0) jac(row, col + 1, sign * g[1]);
  if (g[2] !== 0) jac(row, col + 2, sign * g[2]);
}

const UNIT: Vec3[] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];

// Rows: world point of a minus world point of b.
function writePointRows(frames: Frame[], a: number, sa: Vec3, b: number, sb: Vec3, res: Float64Array, jac: JacobianWriter | null, row: number): void {
  const fa = frameOf(frames, a);
  const fb = frameOf(frames, b);
  const wa = matMulVec(fa.R, sa);
  const wb = matMulVec(fb.R, sb);
  const gap = sub(add(fa.r, wa), add(fb.r, wb));
  for (let k = 0; k < 3; k++) {
    res[row + k] = gap[k];
    if (!jac) continue;
    // d(p)/d(dtheta) for p = r + w: dp = dtheta x w, so the gradient of component k is w x e_k.
    writeGrad(jac, row + k, a, 0, UNIT[k]);
    writeGrad(jac, row + k, a, 3, cross(wa, UNIT[k]));
    writeGrad(jac, row + k, b, 0, UNIT[k], -1);
    writeGrad(jac, row + k, b, 3, cross(wb, UNIT[k]), -1);
  }
}

function pointConstraint(kind: string, label: string, a: number, sa: Vec3, b: number, sb: Vec3): Constraint {
  return {
    kind, label, rows: 3, bodies: [a, b], isDriver: false,
    evaluate(frames, _t, res, jac, row) {
      writePointRows(frames, a, sa, b, sb, res, jac, row);
    },
  };
}

// Pin: points coincide (3 rows) and the axes stay parallel (2 rows).
function pinConstraint(a: number, sa: Vec3, ua: Vec3, b: number, sb: Vec3, ub: Vec3, label: string): Constraint {
  const t1 = perpendicular(ua);
  const t2 = normalize(cross(ua, t1));
  return {
    kind: 'pin', label, rows: 5, bodies: [a, b], isDriver: false,
    evaluate(frames, _t, res, jac, row) {
      writePointRows(frames, a, sa, b, sb, res, jac, row);
      const fa = frameOf(frames, a);
      const axisB = matMulVec(frameOf(frames, b).R, ub);
      const tangents = [matMulVec(fa.R, t1), matMulVec(fa.R, t2)];
      for (let k = 0; k < 2; k++) {
        const tk = tangents[k];
        res[row + 3 + k] = dot(tk, axisB);
        if (!jac) continue;
        const g = cross(tk, axisB);
        writeGrad(jac, row + 3 + k, a, 3, g);
        writeGrad(jac, row + 3 + k, b, 3, g, -1);
      }
    },
  };
}

// Universal joint: points coincide (3 rows) and the two axes stay perpendicular (1 row).
function ujointConstraint(a: number, sa: Vec3, ua: Vec3, b: number, sb: Vec3, ub: Vec3, label: string): Constraint {
  return {
    kind: 'ujoint', label, rows: 4, bodies: [a, b], isDriver: false,
    evaluate(frames, _t, res, jac, row) {
      writePointRows(frames, a, sa, b, sb, res, jac, row);
      const axisA = matMulVec(frameOf(frames, a).R, ua);
      const axisB = matMulVec(frameOf(frames, b).R, ub);
      res[row + 3] = dot(axisA, axisB);
      if (!jac) return;
      const g = cross(axisA, axisB);
      writeGrad(jac, row + 3, a, 3, g);
      writeGrad(jac, row + 3, b, 3, g, -1);
    },
  };
}

// Orientation error 2*vec(delta) and the matrix w*I - sign*[v]x used by its gradient.
function orientationError(delta: Quat): { err: Vec3; w: number; v: Vec3 } {
  const d = delta[0] < 0 ? (delta.map((x) => -x) as Quat) : delta;
  const v: Vec3 = [d[1], d[2], d[3]];
  return { err: scale(v, 2), w: d[0], v };
}

// Rows of (w*I + s*[v]x) applied as gradients; s = -1 for left perturbation, +1 for right.
function orientationGradRow(w: number, v: Vec3, k: number, s: number): Vec3 {
  // Row k of [v]x: [v]x e = v x e, so row k is (e_k x ... ) written out explicitly.
  const skewRows: Vec3[] = [
    [0, -v[2], v[1]],
    [v[2], 0, -v[0]],
    [-v[1], v[0], 0],
  ];
  const r = skewRows[k];
  const out: Vec3 = [s * r[0], s * r[1], s * r[2]];
  out[k] += w;
  return out;
}

function groundConstraint(body: number, pos: Vec3, quat: Quat, label: string): Constraint {
  const target = quatConj(quat);
  return {
    kind: 'ground', label: `ground ${label}`, rows: 6, bodies: [body], isDriver: false,
    evaluate(frames, _t, res, jac, row) {
      const f = frames[body];
      const gap = sub(f.r, pos);
      const { err, w, v } = orientationError(quatMul(f.q, target));
      for (let k = 0; k < 3; k++) {
        res[row + k] = gap[k];
        res[row + 3 + k] = err[k];
        if (!jac) continue;
        writeGrad(jac, row + k, body, 0, UNIT[k]);
        writeGrad(jac, row + 3 + k, body, 3, orientationGradRow(w, v, k, -1));
      }
    },
  };
}

// Weld: b's frame equals a's frame moved by offset and rotated by quat (both in a's frame).
function fixConstraint(a: number, b: number, offset: Vec3, quat: Quat, label: string): Constraint {
  return {
    kind: 'fix', label, rows: 6, bodies: [a, b], isDriver: false,
    evaluate(frames, _t, res, jac, row) {
      const fa = frames[a];
      const fb = frames[b];
      const wo = matMulVec(fa.R, offset);
      const gap = sub(fb.r, add(fa.r, wo));
      const { err, w, v } = orientationError(quatMul(fb.q, quatConj(quatMul(fa.q, quat))));
      for (let k = 0; k < 3; k++) {
        res[row + k] = gap[k];
        res[row + 3 + k] = err[k];
        if (!jac) continue;
        writeGrad(jac, row + k, b, 0, UNIT[k]);
        writeGrad(jac, row + k, a, 0, UNIT[k], -1);
        writeGrad(jac, row + k, a, 3, cross(wo, UNIT[k]), -1);
        writeGrad(jac, row + 3 + k, b, 3, orientationGradRow(w, v, k, -1));
        writeGrad(jac, row + 3 + k, a, 3, orientationGradRow(w, v, k, 1), -1);
      }
    },
  };
}

// Signed angle of body's local x axis about ref's local axis, measured from a fixed ref direction.
export function measureAngle(ref: Frame, axisLocal: Vec3, body: Frame): { angle: number; grad: Vec3 } {
  const refDir = perpendicularInPlane(axisLocal);
  const xr = matMulVec(ref.R, refDir);
  const yr = matMulVec(ref.R, cross(normalize(axisLocal), refDir));
  const d = matMulVec(body.R, EX);
  const x = dot(xr, d);
  const y = dot(yr, d);
  const r2 = Math.max(x * x + y * y, 1e-30);
  const grad = scale(sub(scale(cross(d, yr), x), scale(cross(d, xr), y)), 1 / r2);
  return { angle: Math.atan2(y, x), grad };
}

// Reference direction in the plane normal to the axis: local x projected, or y if x is parallel.
function perpendicularInPlane(axis: Vec3): Vec3 {
  const n = normalize(axis);
  const base: Vec3 = Math.abs(n[0]) < 0.9 ? EX : [0, 1, 0];
  return normalize(sub(base, scale(n, dot(base, n))));
}

// One row: angle of b about a's axis minus target(t).
function angleConstraint(kind: string, label: string, a: number, b: number, axis: Vec3, target: (t: number) => number, isDriver: boolean): Constraint {
  return {
    kind, label, rows: 1, bodies: [a, b], isDriver,
    evaluate(frames, t, res, jac, row) {
      const m = measureAngle(frameOf(frames, a), axis, frameOf(frames, b));
      res[row] = wrapAngle(m.angle - target(t));
      if (!jac) return;
      writeGrad(jac, row, b, 3, m.grad);
      writeGrad(jac, row, a, 3, m.grad, -1);
    },
  };
}

// Gear: angle(a) - ratio * angle(b) - offset = 0, both measured relative to ref.
function gearConstraint(a: number, b: number, ref: number, axis: Vec3, ratio: number, offset: number, label: string): Constraint {
  return {
    kind: 'gear', label, rows: 1, bodies: [a, b, ref], isDriver: false,
    evaluate(frames, _t, res, jac, row) {
      const fr = frameOf(frames, ref);
      const ma = measureAngle(fr, axis, frameOf(frames, a));
      const mb = measureAngle(fr, axis, frameOf(frames, b));
      res[row] = wrapAngle(ma.angle - ratio * mb.angle - offset);
      if (!jac) return;
      writeGrad(jac, row, a, 3, ma.grad);
      writeGrad(jac, row, b, 3, mb.grad, -ratio);
      writeGrad(jac, row, ref, 3, sub(scale(mb.grad, ratio), ma.grad));
    },
  };
}

function wrapAngle(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

// Escape hatch: residual rows from a user function, Jacobian by central differences.
function customConstraint(label: string, fn: CustomJointFn | undefined, bodyIndex: Map<string, number>, points: PointTable): Constraint | null {
  if (!fn) {
    console.error(`Custom joint '${label}' has no function; skipped.`);
    return null;
  }
  const bodyCount = bodyIndex.size;
  const identity = Array.from({ length: bodyCount }, () => ({ r: [0, 0, 0] as Vec3, q: QUAT_IDENTITY, R: quatToMat(QUAT_IDENTITY) }));
  const rows = fn(customAccess(identity, 0, bodyIndex, points)).length;
  const allBodies = Array.from({ length: bodyCount }, (_, i) => i);
  return {
    kind: 'custom', label, rows, bodies: allBodies, isDriver: false,
    evaluate(frames, t, res, jac, row) {
      const base = fn(customAccess(frames, t, bodyIndex, points));
      for (let k = 0; k < rows; k++) res[row + k] = base[k] ?? 0;
      if (!jac) return;
      const h = 1e-6;
      for (let body = 0; body < bodyCount; body++) {
        for (let c = 0; c < 6; c++) {
          const step = new Float64Array(6 * bodyCount);
          step[6 * body + c] = h;
          const plus = fn(customAccess(framesFromPoses(applyIncrement(frames, step, 1)), t, bodyIndex, points));
          const minus = fn(customAccess(framesFromPoses(applyIncrement(frames, step, -1)), t, bodyIndex, points));
          for (let k = 0; k < rows; k++) {
            const g = ((plus[k] ?? 0) - (minus[k] ?? 0)) / (2 * h);
            if (g !== 0) jac(row + k, 6 * body + c, g);
          }
        }
      }
    },
  };
}

type CustomJointFn = (q: CustomAccess) => number[];
type PointTable = Record<string, Record<string, Vec3>>;

function customAccess(frames: Frame[], t: number, bodyIndex: Map<string, number>, points: PointTable): CustomAccess {
  const frameNamed = (name: string) => {
    const i = bodyIndex.get(name);
    if (i === undefined) throw new Error(`custom(): unknown body '${name}'.`);
    return frames[i];
  };
  return {
    t,
    point(ref) {
      const dotAt = ref.lastIndexOf('.');
      const f = frameNamed(ref.slice(0, dotAt));
      const local = points[ref.slice(0, dotAt)]?.[ref.slice(dotAt + 1)];
      if (!local) throw new Error(`custom(): unknown point '${ref}'.`);
      return add(f.r, matMulVec(f.R, local));
    },
    axis(body, localAxis) {
      const v = typeof localAxis === 'string' ? parseAxis(localAxis as 'x') : localAxis;
      return matMulVec(frameNamed(body).R, v);
    },
    pose(body) {
      const f = frameNamed(body);
      return { pos: f.r, quat: f.q };
    },
  };
}

// Residual vector and (optionally) Jacobian at the given poses.
export function evaluateSystem(system: System, poses: Pose[], t: number, withJacobian: boolean): { res: Float64Array; J: SparseJacobian | null } {
  const frames = framesFromPoses(poses);
  const res = new Float64Array(system.rowCount);
  const J = withJacobian ? new SparseJacobian(system.rowCount, system.columnCount) : null;
  const jac: JacobianWriter | null = J ? (r, c, v) => J.add(r, c, v) : null;
  let row = 0;
  for (const c of system.constraints) {
    try {
      c.evaluate(frames, t, res, jac, row);
    } catch (err) {
      console.error(`Constraint '${c.label}' failed to evaluate:`, err);
    }
    row += c.rows;
  }
  return { res, J };
}

// Residual rows of a single constraint (used by the initial-guess search).
export function evaluateConstraint(c: Constraint, frames: Frame[], t: number): Float64Array {
  const res = new Float64Array(c.rows);
  try {
    c.evaluate(frames, t, res, null, 0);
  } catch (err) {
    console.error(`Constraint '${c.label}' failed to evaluate:`, err);
    res.fill(1e3);
  }
  return res;
}

// Apply increments dx (6 per body) scaled by s. Rotation increments multiply on the left.
export function applyIncrement(poses: Pose[], dx: ArrayLike<number>, s: number): Pose[] {
  return poses.map((p, i) => {
    const o = 6 * i;
    const r: Vec3 = [p.r[0] + s * dx[o], p.r[1] + s * dx[o + 1], p.r[2] + s * dx[o + 2]];
    const dq = quatFromRotVec([s * dx[o + 3], s * dx[o + 4], s * dx[o + 5]]);
    return { r, q: quatNormalize(quatMul(dq, p.q)) };
  });
}
