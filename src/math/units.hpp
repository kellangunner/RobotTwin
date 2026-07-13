// Unit conversion helpers. Core code works in SI (m, rad, kg, N·m, s);
// config files and UIs use human units (mm, deg, g, RPM).
// Mirrors web/src/core/units.ts.
#pragma once

#include <algorithm>
#include <numbers>

namespace rt {

inline constexpr double kPi = std::numbers::pi;
inline constexpr double kGravity = 9.81; // m/s²

constexpr double deg2rad(double d) { return d * kPi / 180.0; }
constexpr double rad2deg(double r) { return r * 180.0 / kPi; }
constexpr double mm2m(double mm) { return mm / 1000.0; }
constexpr double m2mm(double m) { return m * 1000.0; }
constexpr double g2kg(double g) { return g / 1000.0; }
constexpr double rpm2radps(double rpm) { return rpm * 2.0 * kPi / 60.0; }
constexpr double gcm2ToKgm2(double gcm2) { return gcm2 * 1e-7; }

constexpr double clamp(double v, double lo, double hi) { return std::min(hi, std::max(lo, v)); }

} // namespace rt
