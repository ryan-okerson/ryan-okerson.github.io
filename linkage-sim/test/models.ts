// Model scripts shared by the tests. They use the macros in src/model/library.js.

export const LAZY_TONG = `
  const L = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
  const units = lazyTong(3, L);
  ground(units[0].A);
  drive(units[0].A, units[0].B, { profile: ramp({ from: 85, to: 8, deg: true, time: 10 }) });
`;

export const GEARED_ROOT = `
  const L = param('L', 1.0, { min: 0.2, max: 4, unit: 'm', group: 'tong' });
  const units = lazyTong(3, L);
  // Root pin spacing matches the 85 deg starting angle, so the start can assemble.
  gearedRoot(units, { spacing: L * Math.sin((85 / 2) * Math.PI / 180) });
  drive(units[0].A, units[0].B, { profile: ramp({ from: 85, to: 8, deg: true, time: 10 }) });
`;

// Driven 60 -> 7.5 deg in 7.5 deg steps, so 60, 30 and 7.5 are all sampled.
export function doubleTong(options: string) {
  return `
    doubleTong(2, 1.0, 0.6, ${options});
    drive('T0.A', 'T0.B', { profile: ramp({ from: 60, to: 7.5, deg: true, time: 7 }) });
  `;
}

export function tongOfUnits(n: number) {
  return `
    const units = lazyTong(${n}, 1.0);
    ground(units[0].A);
    drive(units[0].A, units[0].B, { profile: ramp({ from: 80, to: 10, deg: true, time: 1 }) });
  `;
}
