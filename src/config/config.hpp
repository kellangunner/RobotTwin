// Typed robot configuration (SI units). Loaded from config/robot.yaml by
// config/load.cpp on native builds; WASM builds receive the same values from
// the JavaScript side. Mirrors web/src/core/config.ts.
#pragma once

#include <array>
#include <string>

#include "../math/vec3.hpp"

namespace rt {

inline constexpr int kNumJoints = 3;

enum class Joint : int { Base = 0, Shoulder = 1, Elbow = 2 };

inline const char* jointName(Joint j) {
  switch (j) {
    case Joint::Base: return "base";
    case Joint::Shoulder: return "shoulder";
    case Joint::Elbow: return "elbow";
  }
  return "?";
}

struct LinkGeometry {
  double baseHeight; // m — table to shoulder axis
  double upperArm;   // m — shoulder axis to elbow axis
  double forearm;    // m — elbow axis to TCP
};

struct JointLimits {
  double min; // rad
  double max; // rad
};

struct MassModel {
  double upperArm;       // kg, CoM at mid-link
  double forearm;        // kg, CoM at mid-link
  double elbowMotor;     // kg, at the elbow joint
  double gripper;        // kg, at TCP
  double payloadDefault; // kg
  double payloadMax;     // kg
};

struct MotorParams {
  double holdingTorque; // N·m at standstill
  double maxSpeed;      // rad/s (usable ceiling, torque → 0 here)
  double rotorInertia;  // kg·m²
  double stepAngle;     // rad
  double microstepping;
};

enum class GearboxType : int { Direct = 0, Planetary = 1, Cycloidal = 2 };

struct GearboxParams {
  GearboxType type;
  double ratio;      // output:input, >= 1
  double efficiency; // 0..1
  double backlash;   // rad, at the output
  double maxTorque;  // N·m — printed-gear strength cap
  double inertia;    // kg·m² — input-side gearbox inertia
};

struct FixedGearboxModel {
  std::array<double, 2> ratioRange;
  double efficiency;
  double backlash;  // rad at output
  double maxTorque; // N·m
  double inertia;   // kg·m²
};

struct StagedGearboxModel {
  std::array<double, 2> ratioRange;
  double maxStageRatio;
  double stageEfficiency;
  double stageBacklash; // rad at output, per stage
  double maxTorque;
  double stageInertia; // kg·m² per stage
};

struct GearboxModels {
  FixedGearboxModel direct;
  StagedGearboxModel planetary;
  FixedGearboxModel cycloidal;
};

struct DriveSelection {
  GearboxType type;
  double ratio;
};

struct CollisionModel {
  double groundClearance; // m
  double columnRadius;
  double columnTop; // column cylinder spans z = 0 .. columnTop
  double baseRadius;
  double baseTop;
  double shoulderRadius;
  double upperArmRadius;
  double forearmRadius;
  double gripperExtent; // m past the TCP along the forearm axis
  double elbowTrim;     // fraction of the forearm ignored next to the elbow
};

struct RobotConfig {
  std::string name;
  LinkGeometry links;
  std::array<JointLimits, kNumJoints> limits;
  MassModel masses;
  MotorParams motor;
  GearboxModels gearboxModels;
  std::array<DriveSelection, kNumJoints> drives;   // the independent variables
  std::array<GearboxParams, kNumJoints> gearboxes; // derived from drives
  CollisionModel collision;
};

} // namespace rt
