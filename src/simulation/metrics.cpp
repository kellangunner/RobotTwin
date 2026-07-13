#include "metrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../drivetrain/drivetrain.hpp"
#include "../dynamics/dynamics.hpp"
#include "../kinematics/kinematics.hpp"

namespace rt {

namespace {
/** Fraction of the static torque budget reserved for acceleration. */
constexpr double kAccelTorqueFraction = 0.7;
} // namespace

TwinMetrics computeMetrics(const RobotConfig& config,
                           const std::array<GearboxParams, kNumJoints>& gearboxes,
                           const JointAngles& q, double payload) {
  const MotorParams& motor = config.motor;
  const Jacobian J = jacobian(q, config.links);
  const auto tauGrav = gravityTorques(q, config.links, config.masses, payload);
  const auto linkInertias = worstCaseLinkInertia(config.links, config.masses, payload);

  TwinMetrics out{};
  std::array<double, kNumJoints> backlash{};
  for (int i = 0; i < kNumJoints; ++i) {
    const GearboxParams& gb = gearboxes[i];
    const double available = availableJointTorque(motor, gb, 0.0);
    const double required = std::abs(tauGrav[i]);
    const double refl = reflectedInertia(motor, gb);
    const double totalInertia = refl + linkInertias[i];
    const double budget = kAccelTorqueFraction * (available - required);
    out.joints[i] = JointMetrics{
        static_cast<Joint>(i),
        available,
        required,
        available > 1e-9 ? required / available : std::numeric_limits<double>::infinity(),
        maxJointSpeed(motor, gb),
        std::max(0.0, budget / totalInertia),
        refl,
        linkInertias[i],
        jointResolution(motor, gb),
        required > available,
    };
    backlash[i] = gb.backlash;
    out.vmax[i] = out.joints[i].maxSpeed;
    out.amax[i] = out.joints[i].maxAccel;
  }

  out.backlashErrorTcp = backlashTcpError(J, backlash);
  out.maxTcpSpeed = maxTcpSpeed(J, out.vmax);
  out.singularity = singularityMeasure(q, config.links);
  return out;
}

TrajectoryAudit auditTrajectory(const RobotConfig& config,
                                const std::array<GearboxParams, kNumJoints>& gearboxes,
                                const TrajectoryPlan& plan, double payload,
                                int samples) {
  const auto linkInertias = worstCaseLinkInertia(config.links, config.masses, payload);
  double peakUtilization = 0.0;
  Joint peakJoint = Joint::Base;
  for (int k = 0; k <= samples; ++k) {
    const TrajectorySample s = sampleTrajectory(plan, plan.duration * k / samples);
    const auto tauGrav = gravityTorques(s.q, config.links, config.masses, payload);
    for (int i = 0; i < kNumJoints; ++i) {
      const GearboxParams& gb = gearboxes[i];
      const double inertia = linkInertias[i] + reflectedInertia(config.motor, gb);
      const double required = std::abs(tauGrav[i]) + inertia * std::abs(s.qdd[i]);
      const double available = availableJointTorque(config.motor, gb, s.qd[i]);
      const double util = available > 1e-9 ? required / available
                                           : std::numeric_limits<double>::infinity();
      if (util > peakUtilization) {
        peakUtilization = util;
        peakJoint = static_cast<Joint>(i);
      }
    }
  }
  return {peakUtilization, peakJoint, peakUtilization > 1.0};
}

} // namespace rt
