// Authoring API: runs a model script and produces a plain JSON Model.
import {
  type Mat3, type Quat, type Vec3,
  add, matMul, matMulVec, normalize, quatFromAxisAngle, quatFromMat, quatToMat, QUAT_IDENTITY,
} from '../math/vec';
import type {
  BodyDef, CustomAccess, DriverDef, GroupDef, JointDef, Model, ParamDef, PointRef, PoseDef, ProfileDef,
} from './types';

export interface BuildOptions {
  paramValues?: Record<string, number>;
  groupScales?: Record<string, number>;
}

type AxisInput = 'x' | 'y' | 'z' | '-x' | '-y' | '-z' | Vec3;
type RotInput = { axis: AxisInput; deg?: number; rad?: number };
type PoseInput = { pos?: Vec3; rot?: RotInput };

// A copy/mirror scope: renames bodies and transforms world poses.
interface Scope {
  prefix: string;
  // This scope's own transform (relative to the enclosing scope).
  ownLinear: Mat3;
  ownShift: Vec3;
  ownImproper: boolean;
  localToFull: Map<string, string>;
  // World transform x -> linear * x + shift, accumulated with outer scopes.
  linear: Mat3;
  shift: Vec3;
  improper: boolean;
}

const IDENTITY_MAT: Mat3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
// Local reflection used for mirrored bodies; keeps a bar's local x axis.
const LOCAL_MIRROR: Mat3 = [1, 0, 0, 0, -1, 0, 0, 0, 1];

export const API_NAMES = [
  'param', 'bar', 'plate', 'ground', 'pin', 'ball', 'ujoint', 'gear', 'fix', 'custom',
  'repeat', 'chain', 'copy', 'mirror', 'drive', 'ramp', 'sCurve', 'table',
] as const;

export function buildModel(source: string, options: BuildOptions = {}): Model {
  const builder = createBuilder(options);
  const run = new Function(...API_NAMES, source);
  run(...API_NAMES.map((name) => builder.api[name]));
  return builder.finish();
}

export function parseAxis(axis: AxisInput): Vec3 {
  if (Array.isArray(axis)) return normalize(axis as Vec3);
  const table: Record<string, Vec3> = {
    x: [1, 0, 0], y: [0, 1, 0], z: [0, 0, 1], '-x': [-1, 0, 0], '-y': [0, -1, 0], '-z': [0, 0, -1],
  };
  const v = table[axis];
  if (!v) throw new Error(`Unknown axis '${axis}'. Use 'x', 'y', 'z' or a vector.`);
  return v;
}

export function parsePointRef(ref: string): PointRef {
  const dot = ref.lastIndexOf('.');
  if (dot <= 0) throw new Error(`Point reference '${ref}' must look like 'Body.point'.`);
  return { body: ref.slice(0, dot), point: ref.slice(dot + 1) };
}

function createBuilder(options: BuildOptions) {
  const paramValues = options.paramValues ?? {};
  const groupScales = options.groupScales ?? {};
  const params = new Map<string, ParamDef>();
  const groups = new Map<string, GroupDef>();
  const bodies = new Map<string, BodyDef>();
  const joints: JointDef[] = [];
  let driver: DriverDef | null = null;
  const scopes: Scope[] = [{
    prefix: '', ownLinear: IDENTITY_MAT, ownShift: [0, 0, 0], ownImproper: false, localToFull: new Map(), linear: IDENTITY_MAT, shift: [0, 0, 0], improper: false,
  }];

  const top = () => scopes[scopes.length - 1];

  // Map a name used inside the current scope to the full body name.
  function resolveBody(name: string): string {
    for (let i = scopes.length - 1; i >= 0; i--) {
      const full = scopes[i].localToFull.get(name);
      if (full) return full;
    }
    return name;
  }

  function resolvePoint(ref: string): PointRef {
    const parsed = parsePointRef(ref);
    return { body: resolveBody(parsed.body), point: parsed.point };
  }

  // Local vectors of mirrored bodies are reflected so their frames stay right-handed.
  function localVec(v: Vec3): Vec3 {
    return top().improper ? matMulVec(LOCAL_MIRROR, v) : v;
  }

  function localAxis(axis: AxisInput): Vec3 {
    return localVec(parseAxis(axis));
  }

  function localQuat(q: Quat): Quat {
    if (!top().improper) return q;
    return quatFromMat(matMul(matMul(LOCAL_MIRROR, quatToMat(q)), LOCAL_MIRROR));
  }

  function parseRot(rot?: RotInput): Quat {
    if (!rot) return QUAT_IDENTITY;
    const angle = rot.rad ?? ((rot.deg ?? 0) * Math.PI) / 180;
    return quatFromAxisAngle(parseAxis(rot.axis), angle);
  }

  // Apply the scope's world transform to a pose given in script coordinates.
  function worldPose(input?: PoseInput): PoseDef {
    const scope = top();
    const pos = add(matMulVec(scope.linear, input?.pos ?? [0, 0, 0]), scope.shift);
    let rotation = matMul(scope.linear, quatToMat(parseRot(input?.rot)));
    if (scope.improper) rotation = matMul(rotation, LOCAL_MIRROR);
    return { pos, quat: quatFromMat(rotation) };
  }

  function addBody(name: string, body: Omit<BodyDef, 'name'>): string {
    const scope = top();
    const full = scope.prefix + name;
    if (bodies.has(full)) throw new Error(`Body '${full}' is defined twice.`);
    const copyOf = scopes.length > 1 ? {
      source: scopes[scopes.length - 2].prefix + name,
      linear: [...scope.ownLinear], shift: scope.ownShift, improper: scope.ownImproper,
    } : undefined;
    bodies.set(full, { name: full, ...body, ...(copyOf ? { copyOf } : {}) });
    scope.localToFull.set(name, full);
    return full;
  }

  function groupScaleFor(tags: string[]): number {
    let s = 1;
    for (const tag of tags) s *= groups.get(tag)?.scale ?? 1;
    return s;
  }

  function param(name: string, value: number, opts: Partial<ParamDef> = {}): number {
    const existing = params.get(name);
    if (existing) return existing.value;
    let chosen = paramValues[name] ?? value;
    if (opts.int) chosen = Math.round(chosen);
    params.set(name, {
      name, value: chosen, min: opts.min ?? Math.min(0, value), max: opts.max ?? Math.max(1, 2 * value),
      unit: opts.unit, group: opts.group, int: opts.int,
    });
    if (opts.group && !groups.has(opts.group)) {
      groups.set(opts.group, { name: opts.group, scale: groupScales[opts.group] ?? 1 });
    }
    return chosen;
  }

  function bar(name: string, opts: { length: number; tags?: string[]; points?: Record<string, number | Vec3>; hint?: PoseInput }): string {
    if (!(opts.length > 0)) throw new Error(`Bar '${name}' needs a positive length.`);
    const tags = opts.tags ?? [];
    const length = opts.length * groupScaleFor(tags);
    const given = { a: 0, mid: 0.5, b: 1, ...(opts.points ?? {}) };
    const points: Record<string, Vec3> = {};
    const fractions: Record<string, number> = {};
    for (const [key, value] of Object.entries(given)) {
      if (typeof value === 'number') {
        points[key] = [value * length, 0, 0];
        fractions[key] = value;
      } else {
        points[key] = localVec(value);
      }
    }
    return addBody(name, { kind: 'bar', length, tags, points, fractions, hint: opts.hint && worldPose(opts.hint) });
  }

  function plate(name: string, opts: { points: Record<string, Vec3>; tags?: string[]; hint?: PoseInput }): string {
    const points: Record<string, Vec3> = {};
    for (const [key, value] of Object.entries(opts.points ?? {})) {
      if (!Array.isArray(value)) throw new Error(`Plate '${name}' point '${key}' must be [x, y, z].`);
      points[key] = localVec(value);
    }
    return addBody(name, { kind: 'plate', length: 0, tags: opts.tags ?? [], points, fractions: {}, hint: opts.hint && worldPose(opts.hint) });
  }

  function ground(name: string, opts: PoseInput = {}): void {
    joints.push({ type: 'ground', body: resolveBody(name), pose: worldPose(opts) });
  }

  function pin(a: string, b: string, opts: { axis?: AxisInput; axisB?: AxisInput } = {}): void {
    const axisA = localAxis(opts.axis ?? 'z');
    const axisB = opts.axisB ? localAxis(opts.axisB) : axisA;
    joints.push({ type: 'pin', a: resolvePoint(a), b: resolvePoint(b), axisA, axisB });
  }

  function ball(a: string, b: string): void {
    joints.push({ type: 'ball', a: resolvePoint(a), b: resolvePoint(b) });
  }

  function ujoint(a: string, b: string, opts: { axes?: [AxisInput, AxisInput] } = {}): void {
    const [axisA, axisB] = opts.axes ?? ['z', 'z'];
    joints.push({ type: 'ujoint', a: resolvePoint(a), b: resolvePoint(b), axisA: localAxis(axisA), axisB: localAxis(axisB) });
  }

  function gear(a: string, b: string, opts: { ratio: number; about?: [string, string]; ref?: string; axis?: AxisInput; offset?: number; deg?: boolean }): void {
    if (typeof opts?.ratio !== 'number') throw new Error(`gear('${a}', '${b}') needs a numeric ratio.`);
    const about = opts.about ? [resolvePoint(opts.about[0]), resolvePoint(opts.about[1])] as [PointRef, PointRef] : undefined;
    const ref = opts.ref ? resolveBody(opts.ref) : about ? about[0].body : '';
    let offset = (opts.offset ?? 0) * (opts.deg ? Math.PI / 180 : 1);
    if (top().improper) offset = -offset;
    joints.push({ type: 'gear', a: resolveBody(a), b: resolveBody(b), ratio: opts.ratio, ref, axis: localAxis(opts.axis ?? 'z'), offset, about });
  }

  function fix(a: string, b: string, opts: { offset?: Vec3; rot?: RotInput } = {}): void {
    joints.push({
      type: 'fix', a: resolveBody(a), b: resolveBody(b),
      offset: localVec(opts.offset ?? [0, 0, 0]), quat: localQuat(parseRot(opts.rot)),
    });
  }

  function custom(fn: (q: CustomAccess) => number[], opts: { label?: string } = {}): void {
    if (typeof fn !== 'function') throw new Error('custom() needs a function (q) => [residuals].');
    joints.push({ type: 'custom', label: opts.label ?? `custom${joints.length}`, source: fn.toString(), fn });
  }

  function repeat<T>(n: number, fn: (i: number) => T): T[] {
    return Array.from({ length: n }, (_, i) => fn(i));
  }

  function chain<T>(items: T[], fn: (item: T, next: T, i: number) => void): void {
    for (let i = 0; i + 1 < items.length; i++) fn(items[i], items[i + 1], i);
  }

  // Run fn inside a new naming and transform scope.
  function withScope<T>(fn: () => T, prefix: string, linear: Mat3, shift: Vec3, improper: boolean): T {
    const outer = top();
    scopes.push({
      prefix: outer.prefix + prefix,
      ownLinear: linear,
      ownShift: shift,
      ownImproper: improper,
      localToFull: new Map(),
      linear: matMul(outer.linear, linear),
      shift: add(matMulVec(outer.linear, shift), outer.shift),
      improper: outer.improper !== improper,
    });
    try {
      return fn();
    } finally {
      scopes.pop();
    }
  }

  function copy<T>(fn: () => T, opts: { prefix: string; translate?: Vec3; rotate?: RotInput }): T {
    if (!opts?.prefix) throw new Error('copy() needs a prefix.');
    return withScope(fn, opts.prefix, quatToMat(parseRot(opts.rotate)), opts.translate ?? [0, 0, 0], false);
  }

  function mirror<T>(fn: () => T, opts: { plane: 'xy' | 'xz' | 'yz'; prefix: string }): T {
    if (!opts?.prefix) throw new Error('mirror() needs a prefix.');
    const reflections: Record<string, Mat3> = {
      xy: [1, 0, 0, 0, 1, 0, 0, 0, -1],
      xz: [1, 0, 0, 0, -1, 0, 0, 0, 1],
      yz: [-1, 0, 0, 0, 1, 0, 0, 0, 1],
    };
    const linear = reflections[opts.plane];
    if (!linear) throw new Error(`mirror() plane must be 'xy', 'xz' or 'yz'.`);
    return withScope(fn, opts.prefix, linear, [0, 0, 0], true);
  }

  function drive(a: string, b: string, opts: { profile: ProfileDef; axis?: AxisInput }): void {
    if (driver) throw new Error('Only one driver is allowed. Remove the second drive() call.');
    if (!opts?.profile) throw new Error('drive() needs a profile.');
    driver = { a: resolveBody(a), b: resolveBody(b), axis: localAxis(opts.axis ?? 'z'), profile: opts.profile };
  }

  const toRad = (deg?: boolean) => (deg ? Math.PI / 180 : 1);

  function ramp(o: { from: number; to: number; time: number; deg?: boolean }): ProfileDef {
    return { kind: 'ramp', from: o.from * toRad(o.deg), to: o.to * toRad(o.deg), time: o.time };
  }

  function sCurve(o: { from: number; to: number; time: number; deg?: boolean }): ProfileDef {
    return { kind: 'sCurve', from: o.from * toRad(o.deg), to: o.to * toRad(o.deg), time: o.time };
  }

  function table(points: [number, number][], o: { deg?: boolean } = {}): ProfileDef {
    if (!Array.isArray(points) || points.length < 2) throw new Error('table() needs at least two [t, value] rows.');
    return { kind: 'table', points: points.map(([t, v]) => [t, v * toRad(o.deg)] as [number, number]) };
  }

  function finish(): Model {
    // A copy whose source body does not exist has nothing to start from.
    for (const body of bodies.values()) {
      if (body.copyOf && !bodies.has(body.copyOf.source)) delete body.copyOf;
    }
    const model: Model = {
      params: [...params.values()],
      groups: [...groups.values()],
      bodies: [...bodies.values()],
      joints,
      driver,
    };
    validateModel(model);
    return model;
  }

  const api = { param, bar, plate, ground, pin, ball, ujoint, gear, fix, custom, repeat, chain, copy, mirror, drive, ramp, sCurve, table };
  return { api: api as Record<(typeof API_NAMES)[number], unknown>, finish };
}

// Check that every reference names a real body and point.
export function validateModel(model: Model): void {
  const bodies = new Map(model.bodies.map((b) => [b.name, b]));
  const needBody = (name: string, where: string) => {
    if (!bodies.has(name)) throw new Error(`${where}: unknown body '${name}'.`);
  };
  const needPoint = (ref: PointRef, where: string) => {
    needBody(ref.body, where);
    if (!bodies.get(ref.body)!.points[ref.point]) throw new Error(`${where}: body '${ref.body}' has no point '${ref.point}'.`);
  };
  for (const j of model.joints) {
    if (j.type === 'ground') needBody(j.body, 'ground');
    if (j.type === 'pin' || j.type === 'ball' || j.type === 'ujoint') {
      needPoint(j.a, j.type);
      needPoint(j.b, j.type);
    }
    if (j.type === 'fix') {
      needBody(j.a, 'fix');
      needBody(j.b, 'fix');
    }
    if (j.type === 'gear') {
      needBody(j.a, 'gear');
      needBody(j.b, 'gear');
      if (j.ref) needBody(j.ref, 'gear ref');
    }
  }
  if (model.driver) {
    needBody(model.driver.a, 'drive');
    needBody(model.driver.b, 'drive');
  }
}

// Strip live functions so the model can be saved or posted between threads.
export function serializeModel(model: Model): Model {
  return JSON.parse(JSON.stringify(model));
}

// Restore custom() functions from their source text after loading JSON.
export function reviveModel(model: Model): Model {
  for (const j of model.joints) {
    if (j.type !== 'custom' || j.fn) continue;
    try {
      j.fn = new Function(`return (${j.source});`)();
    } catch (err) {
      console.error(`Could not revive custom joint '${j.label}':`, err);
    }
  }
  validateModel(model);
  return model;
}

