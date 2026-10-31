// Minimal canvas line plot: one x array, several y series, optional log axis, flagged steps.

export interface Series {
  name: string;
  values: number[];
  color: string;
}

export interface PlotSpec {
  title: string;
  xLabel: string;
  x: number[];
  series: Series[];
  logY?: boolean;
  // Indices of steps to mark (failed, locked or branch jump).
  marks: number[];
  // Index of the current step (cursor line).
  cursor: number;
  // Optional horizontal reference line (e.g. rank tolerance).
  reference?: number;
}

export const SERIES_COLORS = ['#4e79a7', '#f28e2b', '#59a14f', '#e15759', '#76b7b2', '#edc948', '#b07aa1', '#ff9da7'];

export function drawPlot(canvas: HTMLCanvasElement, spec: PlotSpec): void {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth;
  const h = canvas.clientHeight;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, w, h);
  const pad = { left: 52, right: 10, top: 18, bottom: 26 };
  const plotW = w - pad.left - pad.right;
  const plotH = h - pad.top - pad.bottom;

  const tf = (v: number) => (spec.logY ? Math.log10(Math.max(Math.abs(v), 1e-18)) : v);
  const ys = spec.series.flatMap((s) => s.values.map(tf)).filter(Number.isFinite);
  if (spec.reference !== undefined) ys.push(tf(spec.reference));
  if (spec.x.length === 0 || ys.length === 0) {
    ctx.fillStyle = '#888';
    ctx.fillText(`${spec.title}: no data`, pad.left, pad.top + 12);
    return;
  }
  let [yMin, yMax] = [Math.min(...ys), Math.max(...ys)];
  if (yMax - yMin < 1e-12) [yMin, yMax] = [yMin - 1, yMax + 1];
  const [xMin, xMax] = [Math.min(...spec.x), Math.max(...spec.x)];
  const sx = (x: number) => pad.left + (xMax === xMin ? plotW / 2 : ((x - xMin) / (xMax - xMin)) * plotW);
  const sy = (y: number) => pad.top + plotH - ((tf(y) - yMin) / (yMax - yMin)) * plotH;

  ctx.strokeStyle = '#444';
  ctx.strokeRect(pad.left, pad.top, plotW, plotH);
  ctx.fillStyle = '#aaa';
  ctx.font = '11px system-ui, sans-serif';
  ctx.fillText(spec.title, pad.left, 12);
  ctx.fillText(spec.xLabel, pad.left + plotW / 2 - 30, h - 6);
  const fmt = (v: number) => (spec.logY ? `1e${v.toFixed(0)}` : Math.abs(v) >= 100 ? v.toFixed(0) : v.toPrecision(3));
  ctx.fillText(fmt(yMax), 4, pad.top + 8);
  ctx.fillText(fmt(yMin), 4, pad.top + plotH);
  ctx.fillText(xMin.toPrecision(3), pad.left, h - 14);
  ctx.fillText(xMax.toPrecision(3), pad.left + plotW - 24, h - 14);

  ctx.strokeStyle = 'rgba(225, 87, 89, 0.7)';
  for (const i of spec.marks) line(ctx, sx(spec.x[i]), pad.top, sx(spec.x[i]), pad.top + plotH);
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.35)';
  if (spec.x[spec.cursor] !== undefined) line(ctx, sx(spec.x[spec.cursor]), pad.top, sx(spec.x[spec.cursor]), pad.top + plotH);
  if (spec.reference !== undefined) {
    ctx.setLineDash([4, 4]);
    line(ctx, pad.left, sy(spec.reference), pad.left + plotW, sy(spec.reference));
    ctx.setLineDash([]);
  }

  spec.series.forEach((s, k) => {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    s.values.forEach((v, i) => (i === 0 ? ctx.moveTo(sx(spec.x[i]), sy(v)) : ctx.lineTo(sx(spec.x[i]), sy(v))));
    ctx.stroke();
    ctx.fillStyle = s.color;
    ctx.fillText(s.name, pad.left + 6 + (k % 4) * Math.min(130, plotW / 4), pad.top + 12 + Math.floor(k / 4) * 12);
  });
  ctx.lineWidth = 1;
}

function line(ctx: CanvasRenderingContext2D, x0: number, y0: number, x1: number, y1: number): void {
  ctx.beginPath();
  ctx.moveTo(x0, y0);
  ctx.lineTo(x1, y1);
  ctx.stroke();
}
