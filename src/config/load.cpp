#include "load.hpp"

#include <stdexcept>

#include "../drivetrain/gearbox_model.hpp"
#include "../math/units.hpp"
#include "yaml_lite.hpp"

namespace rt {

namespace {

GearboxType typeFromString(const std::string& s) {
  if (s == "direct") return GearboxType::Direct;
  if (s == "planetary") return GearboxType::Planetary;
  if (s == "cycloidal") return GearboxType::Cycloidal;
  throw std::runtime_error("unknown gearbox type: " + s);
}

std::array<double, 2> pair(const yaml::Node& n) {
  const auto v = n.asDoubleList();
  if (v.size() != 2) throw std::runtime_error("expected a 2-element list");
  return {v[0], v[1]};
}

} // namespace

RobotConfig parseRobotConfig(const std::string& yamlText) {
  const yaml::Node root = yaml::parse(yamlText);
  RobotConfig cfg;

  cfg.name = root.at("robot").at("name").asString();

  const yaml::Node& links = root.at("links");
  cfg.links = {
      mm2m(links.at("base_height_mm").asDouble()),
      mm2m(links.at("upper_arm_mm").asDouble()),
      mm2m(links.at("forearm_mm").asDouble()),
  };

  const yaml::Node& joints = root.at("joints");
  for (int i = 0; i < kNumJoints; ++i) {
    const auto lim = pair(joints.at(jointName(static_cast<Joint>(i))).at("limits_deg"));
    cfg.limits[i] = {deg2rad(lim[0]), deg2rad(lim[1])};
  }

  const yaml::Node& masses = root.at("masses");
  cfg.masses = {
      g2kg(masses.at("upper_arm_g").asDouble()),
      g2kg(masses.at("forearm_g").asDouble()),
      g2kg(masses.at("elbow_motor_g").asDouble()),
      g2kg(masses.at("gripper_g").asDouble()),
      g2kg(masses.at("payload_default_g").asDouble()),
      g2kg(masses.at("payload_max_g").asDouble()),
  };

  const yaml::Node& motor = root.at("motor");
  cfg.motor = {
      motor.at("holding_torque_nm").asDouble(),
      rpm2radps(motor.at("max_speed_rpm").asDouble()),
      gcm2ToKgm2(motor.at("rotor_inertia_g_cm2").asDouble()),
      deg2rad(motor.at("step_angle_deg").asDouble()),
      motor.at("microstepping").asDouble(),
  };

  const yaml::Node& models = root.at("gearbox_models");
  const yaml::Node& d = models.at("direct");
  cfg.gearboxModels.direct = {
      pair(d.at("ratio_range")),        d.at("efficiency").asDouble(),
      deg2rad(d.at("backlash_deg").asDouble()), d.at("max_torque_nm").asDouble(),
      gcm2ToKgm2(d.at("inertia_g_cm2").asDouble()),
  };
  const yaml::Node& p = models.at("planetary");
  cfg.gearboxModels.planetary = {
      pair(p.at("ratio_range")),
      p.at("max_stage_ratio").asDouble(),
      p.at("stage_efficiency").asDouble(),
      deg2rad(p.at("stage_backlash_deg").asDouble()),
      p.at("max_torque_nm").asDouble(),
      gcm2ToKgm2(p.at("stage_inertia_g_cm2").asDouble()),
  };
  const yaml::Node& c = models.at("cycloidal");
  cfg.gearboxModels.cycloidal = {
      pair(c.at("ratio_range")),        c.at("efficiency").asDouble(),
      deg2rad(c.at("backlash_deg").asDouble()), c.at("max_torque_nm").asDouble(),
      gcm2ToKgm2(c.at("inertia_g_cm2").asDouble()),
  };

  const yaml::Node& gearboxes = root.at("gearboxes");
  for (int i = 0; i < kNumJoints; ++i) {
    const yaml::Node& g = gearboxes.at(jointName(static_cast<Joint>(i)));
    const GearboxType type = typeFromString(g.at("type").asString());
    const double ratio = g.at("ratio").asDouble();
    const DerivedGearbox derived = deriveGearbox(cfg.gearboxModels, type, ratio);
    cfg.drives[i] = {type, derived.params.ratio};
    cfg.gearboxes[i] = derived.params;
  }

  const yaml::Node& col = root.at("collision");
  cfg.collision = {
      mm2m(col.at("ground_clearance_mm").asDouble()),
      mm2m(col.at("column_radius_mm").asDouble()),
      mm2m(col.at("column_top_mm").asDouble()),
      mm2m(col.at("base_radius_mm").asDouble()),
      mm2m(col.at("base_top_mm").asDouble()),
      mm2m(col.at("shoulder_radius_mm").asDouble()),
      mm2m(col.at("upper_arm_radius_mm").asDouble()),
      mm2m(col.at("forearm_radius_mm").asDouble()),
      mm2m(col.at("gripper_extent_mm").asDouble()),
      col.at("elbow_trim").asDouble(),
  };

  return cfg;
}

RobotConfig loadRobotConfig(const std::string& path) {
  return parseRobotConfig(yaml::readFile(path));
}

} // namespace rt
