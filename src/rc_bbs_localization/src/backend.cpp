#include "rc_bbs_localization/core.hpp"
#include <cpu_bbs3d/bbs3d.hpp>
#include <stdexcept>

namespace rc_bbs {
namespace {
std::vector<Eigen::Vector3d> vectors(const Cloud::ConstPtr& cloud) {
  std::vector<Eigen::Vector3d> result;
  result.reserve(cloud->size());
  for (const auto& p : *cloud) result.emplace_back(p.x, p.y, p.z);
  return result;
}
class OfficialCpu final : public GlobalBackend {
  Config config_;
  cpu::BBS3D bbs_;
public:
  OfficialCpu(const Config& c, const Cloud::ConstPtr& map) : config_(c) {
    bbs_.set_num_threads(c.bbs_threads);
    bbs_.set_max_queue_size(c.max_queue_size);
    bbs_.set_tar_points(vectors(map), c.bbs_resolution, c.bbs_levels);
    bbs_.set_trans_search_range(Eigen::Map<const Eigen::Vector3d>(c.search_min.data()),
                                Eigen::Map<const Eigen::Vector3d>(c.search_max.data()));
    const double rp = c.roll_pitch_window_rad;
    bbs_.set_angular_search_range({-rp, -rp, c.yaw_min_rad}, {rp, rp, c.yaw_max_rad});
    bbs_.set_score_threshold_percentage(c.bbs_min_hit_fraction);
    bbs_.enable_timeout();
  }
  std::string name() const override { return "official_cpu+distinct_leaf_extension"; }
  CoarseResult search(const Cloud::ConstPtr& source, Clock::time_point deadline,bool partial_overlap) override {
    CoarseResult out;
    bbs_.set_src_points(vectors(source));
    // Keep the official absolute hit-count objective (and its admissible
    // branch upper bound). Unmapped returns cannot be identified before pose
    // search; allow an explicit bounded outlier fraction, never erase them
    // using the unknown pose or silently renormalize a branch score.
    bbs_.set_score_threshold_percentage(config_.bbs_min_hit_fraction);
    if(config_.unmapped_obstacle_filter && partial_overlap) {
      const double minimum=std::max(config_.bbs_min_hit_fraction*(1.-config_.bbs_max_unmapped_fraction),
        double(config_.bbs_min_matched_points)/std::max<size_t>(1,source->size()));
      if(minimum>=1.) {out.complete=true;out.reason="insufficient_partial_map_bbs_support";return out;}
      bbs_.set_score_threshold_percentage(minimum);
    }
    std::vector<Matrix> exclusions;
    // Find the best remaining distinct mode, then search outside all returned modes.
    // A final empty complete search is required. Reaching the candidate cap is uncertain.
    for (int i = 0; i <= config_.max_candidates; ++i) {
      const int left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
      if (left <= 0) { out.reason = "search_timeout"; return out; }
      bbs_.set_timeout_duration_in_msec(left);
      bbs_.set_exclusions(exclusions, config_.candidate_separation_m, config_.candidate_separation_rad);
      bbs_.localize();
      if (bbs_.has_timed_out() || Clock::now() >= deadline) {
        out.reason = "search_timeout_or_queue_limit"; return out;
      }
      if (!bbs_.has_localized()) {
        out.complete = true;
        out.reason = out.poses.empty() ? "no_bbs_candidate" : "complete";
        return out;
      }
      if (i == config_.max_candidates) { out.reason = "candidate_cap_ambiguity_unresolved"; return out; }
      exclusions.push_back(bbs_.get_global_pose());
      out.poses.emplace_back(bbs_.get_global_pose(), bbs_.get_best_score_percentage());
    }
    return out;
  }
};
}
std::unique_ptr<GlobalBackend> make_backend(const Config& c, const Cloud::ConstPtr& map) {
  if (c.backend != "official_cpu") {
    throw std::invalid_argument("Requested backend '"+c.backend+"' is unavailable. This build supports official_cpu only; no fallback.");
  }
  return std::make_unique<OfficialCpu>(c, map);
}
}  // namespace rc_bbs
