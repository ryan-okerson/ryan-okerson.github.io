// Shared test helpers: build a model with the shipped library, then solve it.
import library from '../src/model/library.js?raw';
import { buildModel, type BuildOptions } from '../src/model/builder';
import { runSweep, type SweepOptions } from '../src/solver/sweep';

export function build(script: string, options: BuildOptions = {}) {
  return buildModel(library + '\n' + script, options);
}

export function solve(script: string, sweep: Partial<SweepOptions> = {}, options: BuildOptions = {}) {
  const model = build(script, options);
  return { model, result: runSweep(model, sweep) };
}

export const deg = (rad: number) => (rad * 180) / Math.PI;
