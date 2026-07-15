#include "retime.hpp"

#include <algorithm>

namespace rt {

namespace {
TrajectoryPlan stretched(const TrajectoryPlan& plan, double k) {
  TrajectoryPlan out = plan;
  out.duration = plan.duration * k;
  return out;
}
} // namespace

RetimeResult retimeForTorque(const TrajectoryPlan& plan,
                              const std::function<TrajectoryAudit(const TrajectoryPlan&)>& audit,
                              double ceiling, double maxStretch) {
  const TrajectoryAudit first = audit(plan);
  if (first.peakUtilization <= ceiling) {
    return {plan, first, 1.0, false};
  }

  // Exponential search for any passing stretch; `lo` stays known-failing.
  double lo = 1.0;
  double hi = 1.0;
  TrajectoryAudit hiAudit = first;
  do {
    lo = hi;
    hi = std::min(maxStretch, hi * 2.0);
    hiAudit = audit(stretched(plan, hi));
  } while (hiAudit.peakUtilization > ceiling && hi < maxStretch);

  if (hiAudit.peakUtilization > ceiling) {
    // Static overload: gravity alone beats the budget, so no speed is slow
    // enough. Keep the planner's timing and let the caller flag the failure.
    return {plan, first, 1.0, true};
  }

  // Bisect down to (near) the smallest passing stretch.
  double pass = hi;
  TrajectoryAudit passAudit = hiAudit;
  for (int i = 0; i < 8; ++i) {
    const double mid = (lo + pass) / 2.0;
    const TrajectoryAudit midAudit = audit(stretched(plan, mid));
    if (midAudit.peakUtilization <= ceiling) {
      pass = mid;
      passAudit = midAudit;
    } else {
      lo = mid;
    }
  }
  return {stretched(plan, pass), passAudit, pass, false};
}

} // namespace rt
