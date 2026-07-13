// Gravity loading and effective inertia. Rod/point mass model:
//   upper arm     — rod, CoM at L1/2 from the shoulder
//   elbow motor   — point mass at the elbow (NEMA 17 mounted there)
//   forearm       — rod, CoM at L2/2 from the elbow
//   gripper+load  — point mass at the TCP
// Mirrors web/src/core/dynamics.ts.
#pragma once

#include "../config/config.hpp"

namespace rt {

/** Static torque (N·m) each joint must hold against gravity at pose q. */
std::array<double, kNumJoints> gravityTorques(const JointAngles& q,
                                              const LinkGeometry& geom,
                                              const MassModel& masses,
                                              double payload);

/**
 * Conservative (fully-extended pose) link-side inertia about each joint axis,
 * used for trajectory acceleration limits. Rods as m·L²/3, point masses m·L².
 */
std::array<double, kNumJoints> worstCaseLinkInertia(const LinkGeometry& geom,
                                                    const MassModel& masses,
                                                    double payload);

} // namespace rt
