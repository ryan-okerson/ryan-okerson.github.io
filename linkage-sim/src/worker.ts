// Solver worker: builds the model from the script (or JSON) and runs the sweep off the UI thread.
import { buildModel, reviveModel, serializeModel } from './model/builder';
import type { Model } from './model/types';
import { runSweep, type SweepOptions, type SweepResult } from './solver/sweep';

export interface SolveRequest {
  id: number;
  script?: string;
  library?: string;
  model?: Model;
  paramValues: Record<string, number>;
  groupScales: Record<string, number>;
  options: Partial<SweepOptions>;
}

export type SolveResponse =
  | { id: number; type: 'progress'; done: number; total: number }
  | { id: number; type: 'result'; model: Model; result: SweepResult }
  | { id: number; type: 'error'; message: string };

const post = (msg: SolveResponse) => self.postMessage(msg);

self.onmessage = (event: MessageEvent<SolveRequest>) => {
  const req = event.data;
  try {
    const model = req.model
      ? reviveModel(req.model)
      : buildModel(`${req.library ?? ''}\n${req.script ?? ''}`, { paramValues: req.paramValues, groupScales: req.groupScales });
    const result = runSweep(model, {
      ...req.options,
      onProgress: (done, total) => post({ id: req.id, type: 'progress', done, total }),
    });
    post({ id: req.id, type: 'result', model: serializeModel(model), result });
  } catch (err) {
    console.error(err);
    post({ id: req.id, type: 'error', message: err instanceof Error ? err.message : String(err) });
  }
};
