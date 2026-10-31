// Wires editor, sliders, worker, 3D view, plots, mobility panel and exports together.
import { EditorView, basicSetup } from 'codemirror';
import { javascript } from '@codemirror/lang-javascript';
import { keymap } from '@codemirror/view';
import { oneDark } from '@codemirror/theme-one-dark';
import library from '../model/library.js?raw';
import { EXAMPLES } from '../examples';
import type { Model, PinJoint } from '../model/types';
import type { SweepResult, StepResult } from '../solver/sweep';
import { add, matMulVec, quatToMat } from '../math/vec';
import { applyIncrement, framesFromPoses, measureAngle, type Pose } from '../solver/system';
import type { SolveRequest, SolveResponse } from '../worker';
import { Viewer } from './viewer';
import { SERIES_COLORS, drawPlot, type Series } from './plot';

type Pick = { kind: 'point'; ref: string } | { kind: 'joint'; index: number; label: string };

const STORAGE_KEY = 'linkage-sim-v1';
const DEG = 180 / Math.PI;

interface State {
  model: Model | null;
  result: SweepResult | null;
  // Model loaded from JSON (no script); sliders cannot rebuild it.
  jsonModel: Model | null;
  paramValues: Record<string, number>;
  groupScales: Record<string, number>;
  step: number;
  playing: boolean;
  wiggle: number;
  picks: Pick[];
}

const $ = <T extends HTMLElement>(id: string) => document.getElementById(id) as T;

export function startApp(): void {
  const saved = loadSaved();
  const state: State = {
    model: null, result: null, jsonModel: null,
    paramValues: {}, groupScales: {}, step: 0, playing: false, wiggle: -1, picks: [],
  };

  const runKey = keymap.of([{ key: 'Mod-Enter', run: () => (solve(true), true) }]);
  const modelEditor = new EditorView({
    doc: saved?.script ?? EXAMPLES['Lazy tong'],
    extensions: [basicSetup, javascript(), oneDark, runKey, EditorView.updateListener.of((u) => u.docChanged && persist())],
    parent: $('editor-model'),
  });
  const libraryEditor = new EditorView({
    doc: saved?.library ?? library,
    extensions: [basicSetup, javascript(), oneDark, runKey, EditorView.updateListener.of((u) => u.docChanged && persist())],
    parent: $('editor-library'),
  });

  const viewer = new Viewer($('view'));
  viewer.onPickPoint = (ref) => {
    state.picks.push({ kind: 'point', ref });
    renderPlots();
  };

  // Examples menu.
  const exampleSelect = $<HTMLSelectElement>('example');
  exampleSelect.innerHTML = '<option value="">Examples…</option>' + Object.keys(EXAMPLES).map((k) => `<option>${k}</option>`).join('');
  exampleSelect.onchange = () => {
    const script = EXAMPLES[exampleSelect.value];
    if (!script) return;
    setDoc(modelEditor, script);
    state.paramValues = {};
    state.groupScales = {};
    state.picks = [];
    exampleSelect.value = '';
    solve(true);
  };

  // Model / library tabs.
  document.querySelectorAll<HTMLButtonElement>('.tabs button').forEach((b) => {
    b.onclick = () => {
      document.querySelectorAll('.tabs button').forEach((x) => x.classList.toggle('active', x === b));
      $('editor-model').hidden = b.dataset.tab !== 'model';
      $('editor-library').hidden = b.dataset.tab !== 'library';
    };
  });

  // Worker lifecycle: a new request replaces a running one.
  let worker: Worker | null = null;
  let busy = false;
  let requestId = 0;
  let fitNext = true;

  function startWorker(): Worker {
    const w = new Worker(new URL('../worker.ts', import.meta.url), { type: 'module' });
    w.onmessage = (e: MessageEvent<SolveResponse>) => handleResponse(e.data);
    w.onerror = (e) => {
      busy = false;
      showLog([`Worker error: ${e.message}`], true);
    };
    return w;
  }

  function solve(fit = false): void {
    fitNext = fitNext || fit;
    if (worker && busy) {
      worker.terminate();
      worker = null;
    }
    worker ??= startWorker();
    busy = true;
    requestId += 1;
    const req: SolveRequest = {
      id: requestId,
      paramValues: state.paramValues,
      groupScales: state.groupScales,
      options: { steps: Number($<HTMLInputElement>('steps').value) || 41 },
      ...(state.jsonModel ? { model: state.jsonModel } : { script: modelEditor.state.doc.toString(), library: libraryEditor.state.doc.toString() }),
    };
    $('step-label').textContent = 'solving…';
    worker.postMessage(req);
  }

  function handleResponse(msg: SolveResponse): void {
    if (msg.id !== requestId) return;
    if (msg.type === 'progress') {
      $('step-label').textContent = `solving… step ${msg.done} / ${msg.total}`;
      return;
    }
    busy = false;
    if (msg.type === 'error') {
      showLog([msg.message], true);
      $('step-label').textContent = 'error';
      return;
    }
    state.model = msg.model;
    state.result = msg.result;
    state.step = Math.min(state.step, msg.result.steps.length - 1);
    state.wiggle = -1;
    buildSliders();
    const scrub = $<HTMLInputElement>('scrub');
    scrub.max = String(Math.max(0, msg.result.steps.length - 1));
    scrub.value = String(state.step);
    viewer.setModel(msg.model, currentStep()?.poses ?? [], fitNext);
    fitNext = false;
    fillJointMenu();
    showLog(msg.result.log.length ? msg.result.log : ['Sweep finished without warnings.'], false);
    showStep();
  }

  const currentStep = (): StepResult | undefined => state.result?.steps[state.step];

  // Sliders: one per param, one scale slider per group.
  function buildSliders(): void {
    const box = $('sliders');
    box.innerHTML = '';
    const model = state.model;
    if (!model) return;
    if (state.jsonModel) {
      box.innerHTML = '<div class="slider-heading">Loaded from JSON: edit the script to use sliders.</div>';
      return;
    }
    if (model.params.length) box.insertAdjacentHTML('beforeend', '<div class="slider-heading">Parameters</div>');
    for (const p of model.params) {
      const step = p.int ? 1 : (p.max - p.min) / 400;
      addSlider(box, `${p.name}${p.unit ? ` (${p.unit})` : ''}`, p.min, p.max, step, p.value, (v) => {
        state.paramValues[p.name] = v;
        return p.int ? String(v) : v.toPrecision(4);
      });
    }
    if (model.groups.length) box.insertAdjacentHTML('beforeend', '<div class="slider-heading">Group length scale</div>');
    for (const g of model.groups) {
      addSlider(box, `group ${g.name} ×`, 0.25, 3, 0.01, g.scale, (v) => {
        state.groupScales[g.name] = v;
        return v.toFixed(2);
      });
    }
  }

  let debounce = 0;
  function addSlider(box: HTMLElement, label: string, min: number, max: number, step: number, value: number, set: (v: number) => string): void {
    const row = document.createElement('div');
    row.className = 'slider-row';
    row.innerHTML = `<span>${label}</span><input type="range" min="${min}" max="${max}" step="${step}" value="${value}"><span></span>`;
    const input = row.querySelector('input')!;
    const out = row.querySelectorAll('span')[1];
    out.textContent = set(value);
    input.oninput = () => {
      out.textContent = set(Number(input.value));
      clearTimeout(debounce);
      debounce = window.setTimeout(() => solve(false), 120);
    };
    box.appendChild(row);
  }

  // Joint angle picks: every pin can be plotted.
  function fillJointMenu(): void {
    const select = $<HTMLSelectElement>('add-joint');
    const pins = (state.model?.joints ?? []).map((j, i) => ({ j, i })).filter(({ j }) => j.type === 'pin');
    select.innerHTML = '<option value="">Plot pin angle…</option>' +
      pins.map(({ j, i }) => `<option value="${i}">${pinLabel(j as PinJoint)}</option>`).join('');
    select.onchange = () => {
      const index = Number(select.value);
      if (select.value === '') return;
      state.picks.push({ kind: 'joint', index, label: pinLabel(state.model!.joints[index] as PinJoint) });
      select.value = '';
      renderPlots();
    };
  }

  // Current step: view, label, mobility panel and plot cursor.
  function showStep(): void {
    const s = currentStep();
    if (!s || !state.result) return;
    if (state.wiggle < 0) viewer.setPoses(s.poses);
    const flags = [!s.converged && 'NEWTON FAILED', s.locked && 'LOCKED', s.branchJump && 'BRANCH JUMP'].filter(Boolean).join(' ');
    $('step-label').textContent = `step ${s.index}  t=${s.t.toFixed(2)} s  angle=${(s.drivenAngle * DEG).toFixed(2)}°  ${flags}`;
    renderMobility(s, state.result);
    renderPlots();
  }

  function renderMobility(s: StepResult, result: SweepResult): void {
    const box = $('mobility');
    const nonzero = s.smallestSingular.find((v) => v > s.tolerance);
    box.innerHTML = `
      <div><b>Mobility at step ${s.index}</b></div>
      <div>Kutzbach count: ${result.kutzbach} <span style="color:var(--muted)">(display only)</span></div>
      <div>Rank result: <b class="${s.freeMotions === 0 ? 'bad' : ''}">${s.freeMotions} free motion(s)</b>
        = 6×${result.bodyNames.length} − rank ${s.rank}</div>
      <div>Smallest singular values: ${s.smallestSingular.slice(0, 6).map((v) => v.toExponential(2)).join(', ')}
        <br><span style="color:var(--muted)">threshold ${s.tolerance.toExponential(1)}; smallest above it ${nonzero?.toExponential(2) ?? '—'}</span></div>
      <div>Max residual ${s.maxResidual.toExponential(2)}, Newton iterations ${s.iterations}</div>
      <div>Free-motion shapes: </div>`;
    s.nullVectors.forEach((_, k) => {
      const b = document.createElement('button');
      b.textContent = `wiggle ${k + 1}`;
      b.className = state.wiggle === k ? 'active' : '';
      b.onclick = () => {
        state.wiggle = state.wiggle === k ? -1 : k;
        if (state.wiggle < 0) viewer.setPoses(s.poses);
        renderMobility(s, result);
      };
      box.appendChild(b);
    });
  }

  // Plots against driven angle or time.
  function renderPlots(): void {
    const result = state.result;
    const box = $('plots');
    if (!result || !state.model) return;
    const steps = result.steps;
    const byAngle = $<HTMLSelectElement>('x-axis').value === 'angle';
    const x = steps.map((s) => (byAngle ? s.drivenAngle * DEG : s.t));
    const xLabel = byAngle ? 'driven angle (deg)' : 'time (s)';
    const marks = steps.filter((s) => !s.converged || s.locked || s.branchJump).map((s) => s.index);
    const specs = [
      { title: 'Free motions', series: [series('free motions', steps.map((s) => s.freeMotions), 0)] },
      {
        title: 'Smallest singular values (log)', logY: true, reference: steps[0]?.tolerance,
        series: [0, 1, 2, 3].map((k) => series(`σ${k + 1}`, steps.map((s) => s.smallestSingular[k] ?? 0), k)),
      },
      { title: 'Picked quantities', series: pickedSeries(state.model, steps, state.picks) },
      { title: 'Bounding box extent (m)', series: ['x', 'y', 'z'].map((axis, k) => series(`L${axis}`, steps.map((s) => s.extent[k]), k)) },
    ];
    while (box.children.length < specs.length) box.appendChild(document.createElement('canvas'));
    specs.forEach((spec, i) => drawPlot(box.children[i] as HTMLCanvasElement, { ...spec, x, xLabel, marks, cursor: state.step }));
  }

  // Scrubber and playback.
  const scrub = $<HTMLInputElement>('scrub');
  scrub.oninput = () => {
    state.step = Number(scrub.value);
    showStep();
  };
  $('play').onclick = () => {
    state.playing = !state.playing;
    $('play').textContent = state.playing ? 'Pause' : 'Play';
  };
  let lastFrame = performance.now();
  let stepClock = 0;
  const animate = (now: number) => {
    const dt = (now - lastFrame) / 1000;
    lastFrame = now;
    const steps = state.result?.steps ?? [];
    if (state.playing && steps.length > 1) {
      const stepTime = (steps[steps.length - 1].t - steps[0].t) / (steps.length - 1) || 0.05;
      stepClock += dt;
      if (stepClock >= stepTime) {
        stepClock = 0;
        state.step = (state.step + 1) % steps.length;
        scrub.value = String(state.step);
        showStep();
      }
    }
    const s = currentStep();
    if (s && state.wiggle >= 0 && s.nullVectors[state.wiggle]) {
      viewer.setPoses(wigglePoses(s.poses, s.nullVectors[state.wiggle], now / 1000, s.extent));
    }
    requestAnimationFrame(animate);
  };
  requestAnimationFrame(animate);

  // Buttons.
  $('run').onclick = () => {
    state.jsonModel = null;
    solve(true);
  };
  $('x-axis').onchange = () => renderPlots();
  $('clear-selection').onclick = () => {
    state.picks = [];
    renderPlots();
  };
  $('save-json').onclick = () => state.model && download('model.json', JSON.stringify(state.model, null, 2), 'application/json');
  $('save-csv').onclick = () => state.result && state.model && download('linkage.csv', toCSV(state.model, state.result, state.picks), 'text/csv');
  $('save-png').onclick = () => downloadURL('view.png', viewer.toPNG());
  $('load-json').onclick = () => $('json-file').click();
  $<HTMLInputElement>('json-file').onchange = async (e) => {
    const file = (e.target as HTMLInputElement).files?.[0];
    if (!file) return;
    try {
      state.jsonModel = JSON.parse(await file.text());
      state.picks = [];
      solve(true);
    } catch (err) {
      console.error(err);
      showLog([`Could not read JSON: ${err}`], true);
    }
  };
  window.addEventListener('resize', () => renderPlots());

  function persist(): void {
    try {
      localStorage.setItem(STORAGE_KEY, JSON.stringify({ script: modelEditor.state.doc.toString(), library: libraryEditor.state.doc.toString() }));
    } catch {
      // Storage unavailable: nothing to keep.
    }
  }

  solve(true);
}

function loadSaved(): { script: string; library: string } | null {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    return raw ? JSON.parse(raw) : null;
  } catch {
    return null;
  }
}

function setDoc(view: EditorView, text: string): void {
  view.dispatch({ changes: { from: 0, to: view.state.doc.length, insert: text } });
}

function series(name: string, values: number[], k: number): Series {
  return { name, values, color: SERIES_COLORS[k % SERIES_COLORS.length] };
}

function pinLabel(j: PinJoint): string {
  return `pin ${j.a.body}.${j.a.point} / ${j.b.body}.${j.b.point}`;
}

// Picked point coordinates (m) and pin angles (deg) per step.
function pickedSeries(model: Model, steps: StepResult[], picks: Pick[]): Series[] {
  const index = new Map(model.bodies.map((b, i) => [b.name, i]));
  const out: Series[] = [];
  for (const pick of picks) {
    if (pick.kind === 'point') {
      const dot = pick.ref.lastIndexOf('.');
      const bodyIndex = index.get(pick.ref.slice(0, dot));
      if (bodyIndex === undefined) continue;
      const local = model.bodies[bodyIndex].points[pick.ref.slice(dot + 1)];
      const coords = steps.map((s) => add(s.poses[bodyIndex].r, matMulVec(quatToMat(s.poses[bodyIndex].q), local)));
      ['x', 'y', 'z'].forEach((axis, k) => out.push(series(`${pick.ref} ${axis} (m)`, coords.map((c) => c[k]), out.length)));
    } else {
      const j = model.joints[pick.index] as PinJoint;
      const a = index.get(j.a.body)!;
      const b = index.get(j.b.body)!;
      const values = steps.map((s) => {
        const frames = framesFromPoses(s.poses);
        return measureAngle(frames[a], j.axisA, frames[b]).angle * DEG;
      });
      out.push(series(`${pick.label} (deg)`, values, out.length));
    }
  }
  return out;
}

// Free-motion shape as a small oscillation about the solved pose.
function wigglePoses(poses: Pose[], mode: number[], time: number, extent: [number, number, number]): Pose[] {
  const size = Math.max(...extent, 0.1);
  let biggest = 1e-12;
  for (let i = 0; i < mode.length; i += 6) {
    biggest = Math.max(biggest, Math.hypot(mode[i], mode[i + 1], mode[i + 2]), Math.hypot(mode[i + 3], mode[i + 4], mode[i + 5]) * size);
  }
  const amplitude = (0.06 * size) / biggest;
  return applyIncrement(poses, mode, amplitude * Math.sin(time * 4));
}

function toCSV(model: Model, result: SweepResult, picks: Pick[]): string {
  const steps = result.steps;
  const picked = pickedSeries(model, steps, picks);
  const header = ['step', 't_s', 'driven_angle_deg', 'converged', 'max_residual', 'free_motions', 'sigma1', 'sigma2', 'sigma3', 'sigma4',
    'extent_x_m', 'extent_y_m', 'extent_z_m', 'locked', 'branch_jump', ...picked.map((s) => s.name)];
  const rows = steps.map((s, i) => [
    s.index, s.t, s.drivenAngle * DEG, s.converged, s.maxResidual, s.freeMotions,
    ...[0, 1, 2, 3].map((k) => s.smallestSingular[k] ?? ''), ...s.extent, s.locked, s.branchJump,
    ...picked.map((p) => p.values[i]),
  ].join(','));
  return [header.map((h) => `"${h}"`).join(','), ...rows].join('\n');
}

function download(name: string, text: string, type: string): void {
  const url = URL.createObjectURL(new Blob([text], { type }));
  downloadURL(name, url);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
}

function downloadURL(name: string, url: string): void {
  const a = document.createElement('a');
  a.href = url;
  a.download = name;
  a.click();
}

function showLog(lines: string[], isError: boolean): void {
  const log = $('log');
  log.textContent = lines.join('\n');
  log.className = isError ? 'error' : '';
}
