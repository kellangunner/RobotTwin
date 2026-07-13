// Embind bindings exposing the C++ core to the web twin. Built only under
// Emscripten (see CMakeLists.txt); output lands in web/src/wasm/.
//
// The JS side parses config/robot.yaml (it already ships the text) and passes
// a plain RobotConfig object here — the YAML subset parser is native-only.
//
// Variable-length results are returned as plain JS arrays (val::array), never
// register_vector handles, so the frontend has no embind objects to delete.

#include <emscripten/bind.h>
#include <emscripten/val.h>

#include "../config/config.hpp"
#include "../drivetrain/drivetrain.hpp"
#include "../drivetrain/gearbox_model.hpp"
#include "../dynamics/dynamics.hpp"
#include "../geometry/collision.hpp"
#include "../io/waypoints.hpp"
#include "../kinematics/kinematics.hpp"
#include "../planning/trajectory.hpp"
#include "../simulation/metrics.hpp"

using namespace emscripten;
using namespace rt;

namespace {

val collisionIssueArray(const CollisionCheck& c) {
  val arr = val::array();
  for (int i = 0; i < c.issueCount; ++i) arr.call<void>("push", std::string(describe(c.issues[i])));
  return arr;
}

val collisionCheckToVal(const CollisionCheck& c) {
  val out = val::object();
  out.set("colliding", c.colliding);
  out.set("issues", collisionIssueArray(c));
  return out;
}

} // namespace

EMSCRIPTEN_BINDINGS(robottwin) {
  // ------------------------------------------------------------- value types
  value_array<Vec3>("Vec3").element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());
  value_array<std::array<double, 2>>("Range").element(emscripten::index<0>()).element(emscripten::index<1>());
  value_array<std::array<bool, 3>>("Bool3")
      .element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());

  enum_<Joint>("Joint")
      .value("Base", Joint::Base)
      .value("Shoulder", Joint::Shoulder)
      .value("Elbow", Joint::Elbow);
  enum_<GearboxType>("GearboxType")
      .value("Direct", GearboxType::Direct)
      .value("Planetary", GearboxType::Planetary)
      .value("Cycloidal", GearboxType::Cycloidal);
  enum_<IkBranch>("IkBranch")
      .value("ElbowUp", IkBranch::ElbowUp)
      .value("ElbowDown", IkBranch::ElbowDown);
  enum_<WaypointMode>("WaypointMode")
      .value("Cartesian", WaypointMode::Cartesian)
      .value("Joints", WaypointMode::Joints);

  value_object<LinkGeometry>("LinkGeometry")
      .field("baseHeight", &LinkGeometry::baseHeight)
      .field("upperArm", &LinkGeometry::upperArm)
      .field("forearm", &LinkGeometry::forearm);
  value_object<JointLimits>("JointLimits")
      .field("min", &JointLimits::min)
      .field("max", &JointLimits::max);
  value_array<std::array<JointLimits, 3>>("JointLimits3")
      .element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());
  value_object<MassModel>("MassModel")
      .field("upperArm", &MassModel::upperArm)
      .field("forearm", &MassModel::forearm)
      .field("elbowMotor", &MassModel::elbowMotor)
      .field("gripper", &MassModel::gripper)
      .field("payloadDefault", &MassModel::payloadDefault)
      .field("payloadMax", &MassModel::payloadMax);
  value_object<MotorParams>("MotorParams")
      .field("holdingTorque", &MotorParams::holdingTorque)
      .field("maxSpeed", &MotorParams::maxSpeed)
      .field("rotorInertia", &MotorParams::rotorInertia)
      .field("stepAngle", &MotorParams::stepAngle)
      .field("microstepping", &MotorParams::microstepping);
  value_object<GearboxParams>("GearboxParams")
      .field("type", &GearboxParams::type)
      .field("ratio", &GearboxParams::ratio)
      .field("efficiency", &GearboxParams::efficiency)
      .field("backlash", &GearboxParams::backlash)
      .field("maxTorque", &GearboxParams::maxTorque)
      .field("inertia", &GearboxParams::inertia);
  value_array<std::array<GearboxParams, 3>>("GearboxParams3")
      .element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());
  value_object<FixedGearboxModel>("FixedGearboxModel")
      .field("ratioRange", &FixedGearboxModel::ratioRange)
      .field("efficiency", &FixedGearboxModel::efficiency)
      .field("backlash", &FixedGearboxModel::backlash)
      .field("maxTorque", &FixedGearboxModel::maxTorque)
      .field("inertia", &FixedGearboxModel::inertia);
  value_object<StagedGearboxModel>("StagedGearboxModel")
      .field("ratioRange", &StagedGearboxModel::ratioRange)
      .field("maxStageRatio", &StagedGearboxModel::maxStageRatio)
      .field("stageEfficiency", &StagedGearboxModel::stageEfficiency)
      .field("stageBacklash", &StagedGearboxModel::stageBacklash)
      .field("maxTorque", &StagedGearboxModel::maxTorque)
      .field("stageInertia", &StagedGearboxModel::stageInertia);
  value_object<GearboxModels>("GearboxModels")
      .field("direct", &GearboxModels::direct)
      .field("planetary", &GearboxModels::planetary)
      .field("cycloidal", &GearboxModels::cycloidal);
  value_object<DriveSelection>("DriveSelection")
      .field("type", &DriveSelection::type)
      .field("ratio", &DriveSelection::ratio);
  value_array<std::array<DriveSelection, 3>>("DriveSelection3")
      .element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());
  value_object<CollisionModel>("CollisionModel")
      .field("groundClearance", &CollisionModel::groundClearance)
      .field("columnRadius", &CollisionModel::columnRadius)
      .field("columnTop", &CollisionModel::columnTop)
      .field("baseRadius", &CollisionModel::baseRadius)
      .field("baseTop", &CollisionModel::baseTop)
      .field("shoulderRadius", &CollisionModel::shoulderRadius)
      .field("upperArmRadius", &CollisionModel::upperArmRadius)
      .field("forearmRadius", &CollisionModel::forearmRadius)
      .field("gripperExtent", &CollisionModel::gripperExtent)
      .field("elbowTrim", &CollisionModel::elbowTrim);
  value_object<RobotConfig>("RobotConfig")
      .field("name", &RobotConfig::name)
      .field("links", &RobotConfig::links)
      .field("limits", &RobotConfig::limits)
      .field("masses", &RobotConfig::masses)
      .field("motor", &RobotConfig::motor)
      .field("gearboxModels", &RobotConfig::gearboxModels)
      .field("drives", &RobotConfig::drives)
      .field("gearboxes", &RobotConfig::gearboxes)
      .field("collision", &RobotConfig::collision);

  value_object<FkResult>("FkResult")
      .field("shoulder", &FkResult::shoulder)
      .field("elbow", &FkResult::elbow)
      .field("tcp", &FkResult::tcp);
  value_object<IkSolution>("IkSolution")
      .field("q", &IkSolution::q)
      .field("branch", &IkSolution::branch)
      .field("baseFlipped", &IkSolution::baseFlipped)
      .field("withinLimits", &IkSolution::withinLimits)
      .field("violated", &IkSolution::violated)
      .field("nearSingularity", &IkSolution::nearSingularity);
  value_object<TrajectoryPlan>("TrajectoryPlan")
      .field("from", &TrajectoryPlan::from)
      .field("to", &TrajectoryPlan::to)
      .field("duration", &TrajectoryPlan::duration)
      .field("infeasible", &TrajectoryPlan::infeasible);
  value_object<TrajectorySample>("TrajectorySample")
      .field("q", &TrajectorySample::q)
      .field("qd", &TrajectorySample::qd)
      .field("qdd", &TrajectorySample::qdd);
  value_object<JointMetrics>("JointMetrics")
      .field("joint", &JointMetrics::joint)
      .field("availableTorque", &JointMetrics::availableTorque)
      .field("requiredTorque", &JointMetrics::requiredTorque)
      .field("utilization", &JointMetrics::utilization)
      .field("maxSpeed", &JointMetrics::maxSpeed)
      .field("maxAccel", &JointMetrics::maxAccel)
      .field("reflectedInertia", &JointMetrics::reflectedInertia)
      .field("linkInertia", &JointMetrics::linkInertia)
      .field("resolution", &JointMetrics::resolution)
      .field("holdFails", &JointMetrics::holdFails);
  value_array<std::array<JointMetrics, 3>>("JointMetrics3")
      .element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());
  value_object<TwinMetrics>("TwinMetrics")
      .field("joints", &TwinMetrics::joints)
      .field("backlashErrorTcp", &TwinMetrics::backlashErrorTcp)
      .field("maxTcpSpeed", &TwinMetrics::maxTcpSpeed)
      .field("singularity", &TwinMetrics::singularity)
      .field("vmax", &TwinMetrics::vmax)
      .field("amax", &TwinMetrics::amax);
  value_object<TrajectoryAudit>("TrajectoryAudit")
      .field("peakUtilization", &TrajectoryAudit::peakUtilization)
      .field("peakJoint", &TrajectoryAudit::peakJoint)
      .field("skippedSteps", &TrajectoryAudit::skippedSteps);
  value_object<DerivedGearbox>("DerivedGearbox")
      .field("params", &DerivedGearbox::params)
      .field("stages", &DerivedGearbox::stages);

  value_array<Jacobian>("Jacobian")
      .element(emscripten::index<0>()).element(emscripten::index<1>()).element(emscripten::index<2>());

  // -------------------------------------------------------------- functions
  function("forwardKinematics", &forwardKinematics);
  function("jacobian", &jacobian);
  function("singularityMeasure", &singularityMeasure);
  function("inverseKinematics",
           +[](const Vec3& target, const LinkGeometry& geom,
               const std::array<JointLimits, 3>& limits) {
             const IkResult r = inverseKinematics(target, geom, limits);
             val solutions = val::array();
             for (int i = 0; i < r.solutionCount; ++i) {
               solutions.call<void>("push", val(r.solutions[i]));
             }
             val out = val::object();
             out.set("reachable", r.reachable);
             out.set("baseSingular", r.baseSingular);
             out.set("solutions", solutions);
             return out;
           });
  function("workspaceBoundary",
           +[](const LinkGeometry& geom, const std::array<JointLimits, 3>& limits,
               int samples) {
             val arr = val::array();
             for (const auto& p : workspaceBoundary(geom, limits, samples)) {
               arr.call<void>("push", val(p));
             }
             return arr;
           });

  function("motorTorqueAtSpeed", &motorTorqueAtSpeed);
  function("availableJointTorque", &availableJointTorque);
  function("maxJointSpeed", &maxJointSpeed);
  function("torqueLimitedSpeed", &torqueLimitedSpeed);
  function("reflectedInertia", &reflectedInertia);
  function("jointResolution", &jointResolution);
  function("deriveGearbox", &deriveGearbox);
  function("ratioRange", &ratioRange);

  function("gravityTorques", &gravityTorques);
  function("planTrajectory", &planTrajectory);
  function("sampleTrajectory", &sampleTrajectory);
  function("computeMetrics", &computeMetrics);
  function("auditTrajectory", &auditTrajectory);

  function("checkPose", +[](const JointAngles& q, const LinkGeometry& geom,
                            const CollisionModel& model) {
    return collisionCheckToVal(checkPose(q, geom, model));
  });
  function("checkPath", +[](const TrajectoryPlan& plan, const LinkGeometry& geom,
                            const CollisionModel& model) {
    return collisionCheckToVal(checkPath(plan, geom, model));
  });

  function("parseWaypointCsv", +[](const std::string& text, WaypointMode mode,
                                   const RobotConfig& config, IkBranch branch,
                                   const JointAngles& fromQ) {
    const WaypointParseResult r = parseWaypointCsv(text, mode, config, branch, fromQ);
    val targets = val::array();
    for (const JointAngles& q : r.targets) targets.call<void>("push", val(q));
    val out = val::object();
    out.set("targets", targets);
    out.set("skipped", r.skipped);
    out.set("firstIssue", r.firstIssue);
    return out;
  });
}
