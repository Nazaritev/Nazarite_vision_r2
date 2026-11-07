#pragma once

#include <atomic>
#include <cmath>

namespace rc_cloud_localization
{
// No reset method: a TF ownership conflict can only be cleared by restarting.
// Keep this independent of ROS so failure paths can be checked without a ROS graph.
class OutputGate
{
public:
  bool latch_conflict() {return !conflict_.exchange(true);}
  bool conflict() const {return conflict_.load();}
  bool permits(int good, int required, bool lost, bool startup_check_complete,
    double result_age, double max_age) const
  {
    return !conflict() && !lost && startup_check_complete && required > 0 &&
           good >= required && std::isfinite(result_age) && result_age >= -0.05 &&
           result_age <= max_age;
  }
private:
  std::atomic<bool> conflict_{false};
};
}  // namespace rc_cloud_localization
