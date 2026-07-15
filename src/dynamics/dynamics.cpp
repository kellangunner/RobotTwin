#include "dynamics.hpp"

#include <cmath>

#include "../math/units.hpp"

namespace rt {

std::array<double, kNumJoints> gravityTorques(const JointAngles& q,
                                              const LinkGeometry& geom,
                                              const MassModel& masses,
                                              double payload) {
  const double L1 = geom.upperArm, L2 = geom.forearm;
  const double c2 = std::cos(q[1]);
  const double c23 = std::cos(q[1] + q[2]);
  const double mTip = masses.gripper + payload;
  const double tauElbow = kGravity * (masses.forearm * (L2 / 2.0) * c23 + mTip * L2 * c23);
  const double tauShoulder =
      kGravity * ((masses.upperArm * (L1 / 2.0) + masses.elbowMotor * L1) * c2 +
                  masses.forearm * (L1 * c2 + (L2 / 2.0) * c23) +
                  mTip * (L1 * c2 + L2 * c23));

  return {0.0, tauShoulder, tauElbow}; // base axis is vertical so no gravity load
}

std::array<double, kNumJoints> worstCaseLinkInertia(const LinkGeometry& geom,
                                                    const MassModel& masses,
                                                    double payload) {
  const double L1 = geom.upperArm, L2 = geom.forearm;
  const double mTip = masses.gripper + payload;
  const double reach = L1 + L2;

  const double aboutElbow = masses.forearm * L2 * L2 / 3.0 + mTip * L2 * L2;
  const double armMid = L1 + L2 / 2.0;
  const double aboutShoulder = masses.upperArm * L1 * L1 / 3.0 +
                               masses.elbowMotor * L1 * L1 +
                               masses.forearm * armMid * armMid + mTip * reach * reach;
  // base spins the whole extended arm about the vertical axis
  const double aboutBase = aboutShoulder;

  return {aboutBase, aboutShoulder, aboutElbow};
}

}
