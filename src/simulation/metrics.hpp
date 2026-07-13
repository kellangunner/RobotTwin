// Aggregates kinematics + drivetrain + dynamics into the live metrics a twin
// frontend displays, plus the predictive trajectory torque audit.
// Mirrors web/src/core/metrics.ts.
#pragma once

#include "../config/config.hpp"
#include "../planning/trajectory.hpp"

namespace rt {

struct JointMetrics {
  Joint joint;
  double availableTorque;  // N·m at standstill
  double requiredTorque;   // N·m gravity load at current pose
  double utilization;      // required / available (huge when available ~ 0)
  double maxSpeed;         // rad/s
  double maxAccel;         // rad/s², torque budget / total inertia
  double reflectedInertia; // kg·m²
  double linkInertia;      // kg·m² (worst-case pose)
  double resolution;       // rad per microstep
  bool holdFails;          // cannot even hold this pose statically
};

struct TwinMetrics {
  std::array<JointMetrics, kNumJoints> joints;
  double backlashErrorTcp; // m, worst case at current pose
  double maxTcpSpeed;      // m/s bound at current pose
  double singularity;      // 0 (singular) .. 1
  std::array<double, kNumJoints> vmax;
  std::array<double, kNumJoints> amax;
};

TwinMetrics computeMetrics(const RobotConfig& config,
                           const std::array<GearboxParams, kNumJoints>& gearboxes,
                           const JointAngles& q, double payload);

struct TrajectoryAudit {
  double peakUtilization; // worst required/available torque ratio over the move
  Joint peakJoint;
  bool skippedSteps; // >1 → open-loop steppers would lose position
};

/**
 * Predictive torque audit of a planned move: at each sample, gravity plus
 * inertial torque vs what the drivetrain delivers at that joint speed.
 * Deterministic — evaluated at plan time, never during playback.
 */
TrajectoryAudit auditTrajectory(const RobotConfig& config,
                                const std::array<GearboxParams, kNumJoints>& gearboxes,
                                const TrajectoryPlan& plan, double payload,
                                int samples = 120);

} // namespace rt
