// Minimal 3-vector helpers over std::array — no external math dependency.
#pragma once

#include <array>
#include <cmath>

namespace rt {

using Vec3 = std::array<double, 3>;
using JointAngles = std::array<double, 3>; // [θ1, θ2, θ3] rad

constexpr Vec3 sub(const Vec3& a, const Vec3& b) {
  return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}
constexpr Vec3 add(const Vec3& a, const Vec3& b) {
  return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}
constexpr Vec3 scale(const Vec3& a, double k) { return {a[0] * k, a[1] * k, a[2] * k}; }
constexpr double dot(const Vec3& a, const Vec3& b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
inline double norm(const Vec3& a) { return std::hypot(a[0], a[1], a[2]); }

} // namespace rt
