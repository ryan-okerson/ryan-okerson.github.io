// Time sweep: position solve, mobility, velocity and acceleration at each step.
import { type Vec3, add, matMulVec, quatAngleBetween, quatToMat } from '../math/vec';
import { profileDuration, sampleProfile } from '../model/profiles';
import type { Model } from '../model/types';
import { type Pose, type System, applyIncrement, compileSystem, evaluateSystem } from './system';
import { initialGuesses } from './guess';
import { type NewtonResult, solvePositions } from './newton';
import { NormalFactor, smallestSingular, solveLeastSquares } from './sparse';

export interface SweepOptions {
  steps: number;
  tolerance: number;
  maxIterations: number;
  // Iteration cap for the very first assembly from the tree guess.
  assemblyIterations: number;
  rankTolRel: number;
  branchLimitDeg: number;
  stopOnFailure: boolean;
  onProgress?: (done: number, total: number) => void;
}

export const DEFAULT_SWEEP: SweepOptions = {
  steps: 41,
  tolerance: 1e-10,
  maxIterations: 50,
  assemblyIterations: 200,
  rankTolRel: 1e-8,
  branchLimitDeg: 20,
  stopOnFailure: true,
};

export interface StepResult {
  index: number;
  t: number;
  // Driver target angle at this step (radians).
  drivenAngle: number;
  converged: boolean;
  iterations: number;
  maxResidual: number;
  rank: number;
  freeMotions: number;
  // Smallest singular values of the Jacobian without the driver row, ascending.
  smallestSingular: number[];
  tolerance: number;
  nullVectors: number[][];
  poses: Pose[];
  velocity: number[];
  acceleration: number[];
  extent: Vec3;
  locked: boolean;
  branchJump: boolean;
}

export interface SweepResult {
  bodyNames: string[];
  steps: StepResult[];
  kutzbach: number;
  log: string[];
}

export function runSweep(model: Model, options: Partial<SweepOptions> = {}): SweepResult {
  const opts = { ...DEFAULT_SWEEP, ...options };
  const system = compileSystem(model);
  const log: string[] = [];
  const times = sampleTimes(model, opts.steps);
  const steps: StepResult[] = [];
  let previous: StepResult | null = null;

  for (let k = 0; k < times.length; k++) {
    const t = times[k];
    const newton = previous
      ? solvePositions(system, applyIncrement(previous.poses, previous.velocity, t - previous.t), t, opts)
      : assemble(system, t, opts, log);
    const step = analyzeStep(system, newton.poses, t, k, opts);
    step.converged = newton.converged;
    step.iterations = newton.iterations;
    step.maxResidual = newton.maxResidual;
    step.branchJump = previous ? maxRotation(previous.poses, step.poses) > (opts.branchLimitDeg * Math.PI) / 180 : false;
    steps.push(step);
    opts.onProgress?.(k + 1, times.length);
    logStep(log, step);
    previous = step;
    if (!newton.converged && opts.stopOnFailure) {
      log.push(`Stopped sweep at step ${k}: Newton did not converge (max residual ${newton.maxResidual.toExponential(3)}).`);
      break;
    }
  }
  return { bodyNames: system.bodyNames, steps, kutzbach: kutzbachCount(system), log };
}

// First solve: try each candidate guess until one converges; keep the best otherwise.
function assemble(system: System, t: number, opts: SweepOptions, log: string[]): NewtonResult {
  let best: NewtonResult | null = null;
  for (const guess of initialGuesses(system, t)) {
    const result = solvePositions(system, guess, t, { ...opts, maxIterations: opts.assemblyIterations });
    if (result.converged) return result;
    if (!best || result.maxResidual < best.maxResidual) best = result;
    log.push(`Assembly attempt failed (max residual ${result.maxResidual.toExponential(3)}); trying next guess.`);
  }
  return best!;
}

function sampleTimes(model: Model, steps: number): number[] {
  if (!model.driver) return [0];
  const duration = profileDuration(model.driver.profile);
  const n = Math.max(2, Math.round(steps));
  return Array.from({ length: n }, (_, k) => (duration * k) / (n - 1));
}

function logStep(log: string[], s: StepResult): void {
  if (!s.converged) log.push(`Step ${s.index} (t=${s.t.toFixed(3)}): Newton failed, max residual ${s.maxResidual.toExponential(3)}.`);
  if (s.locked) log.push(`Step ${s.index} (t=${s.t.toFixed(3)}): locked, 0 free motions.`);
  if (s.branchJump) log.push(`Step ${s.index} (t=${s.t.toFixed(3)}): branch jump suspected.`);
}

// Mobility, velocity and acceleration at a solved configuration.
function analyzeStep(system: System, poses: Pose[], t: number, index: number, opts: SweepOptions): StepResult {
  const { J } = evaluateSystem(system, poses, t, true);
  // Mobility uses the Jacobian with the driver row removed.
  const mobility = smallestSingular(J!, opts.rankTolRel, system.driverRow);
  const freeMotions = mobility.nullity;
  const rank = system.columnCount - freeMotions;
  const full = new NormalFactor(J!, 1e-13);
  const drive = system.model.driver ? sampleProfile(system.model.driver.profile, t) : { value: 0, rate: 0, accel: 0 };

  const rhs = new Float64Array(system.rowCount);
  if (system.driverRow >= 0) rhs[system.driverRow] = drive.rate;
  const velocity = solveLeastSquares(J!, rhs, 0, full);

  const curvature = secondDerivative(system, poses, velocity, t);
  const accelRhs = curvature.map((c) => -c);
  if (system.driverRow >= 0) accelRhs[system.driverRow] += drive.accel;
  const acceleration = solveLeastSquares(J!, accelRhs, 0, full);

  return {
    index, t, drivenAngle: drive.value, converged: false, iterations: 0, maxResidual: 0,
    rank, freeMotions,
    smallestSingular: mobility.sigma.slice(0, 8),
    tolerance: mobility.tolerance,
    nullVectors: mobility.vectors.slice(0, freeMotions).map((v) => Array.from(v)),
    poses,
    velocity: Array.from(velocity),
    acceleration: Array.from(acceleration),
    extent: boundingExtent(system, poses),
    locked: freeMotions === 0,
    branchJump: false,
  };
}

// Second derivative of the constraints along the velocity (central difference).
function secondDerivative(system: System, poses: Pose[], velocity: Float64Array, t: number): Float64Array {
  let speed = 0;
  for (const v of velocity) speed = Math.max(speed, Math.abs(v));
  if (speed === 0) return new Float64Array(system.rowCount);
  const h = 1e-4 / speed;
  const plus = evaluateSystem(system, applyIncrement(poses, velocity, h), t, false).res;
  const minus = evaluateSystem(system, applyIncrement(poses, velocity, -h), t, false).res;
  const mid = evaluateSystem(system, poses, t, false).res;
  return plus.map((p, i) => (p - 2 * mid[i] + minus[i]) / (h * h));
}

function maxRotation(a: Pose[], b: Pose[]): number {
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, quatAngleBetween(a[i].q, b[i].q));
  return m;
}

// Size of the axis-aligned box around every named point.
export function boundingExtent(system: System, poses: Pose[]): Vec3 {
  const lo: Vec3 = [Infinity, Infinity, Infinity];
  const hi: Vec3 = [-Infinity, -Infinity, -Infinity];
  system.model.bodies.forEach((body, i) => {
    const R = quatToMat(poses[i].q);
    for (const local of Object.values(body.points)) {
      const p = add(poses[i].r, matMulVec(R, local));
      for (let k = 0; k < 3; k++) {
        lo[k] = Math.min(lo[k], p[k]);
        hi[k] = Math.max(hi[k], p[k]);
      }
    }
  });
  if (lo[0] === Infinity) return [0, 0, 0];
  return [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
}

// Kutzbach count: 6 per non-ground body minus rows of every joint. For display only.
export function kutzbachCount(system: System): number {
  const grounded = new Set(system.model.joints.flatMap((j) => (j.type === 'ground' ? [j.body] : [])));
  let removed = 0;
  for (const c of system.constraints) {
    if (c.kind !== 'ground' && !c.isDriver) removed += c.rows;
  }
  return 6 * (system.bodyNames.length - grounded.size) - removed;
}
