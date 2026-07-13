// Unit conversion helpers. Core code works in SI (m, rad, kg, N·m, s);
// config files and UI use human units (mm, deg, g, RPM).

export const DEG = Math.PI / 180;

export const deg2rad = (d: number): number => d * DEG;
export const rad2deg = (r: number): number => r / DEG;
export const mm2m = (mm: number): number => mm / 1000;
export const m2mm = (m: number): number => m * 1000;
export const g2kg = (g: number): number => g / 1000;
export const rpm2radps = (rpm: number): number => (rpm * 2 * Math.PI) / 60;
export const gcm2_to_kgm2 = (gcm2: number): number => gcm2 * 1e-7;

export const GRAVITY = 9.81; // m/s²

export const clamp = (v: number, lo: number, hi: number): number =>
  Math.min(hi, Math.max(lo, v));
