#pragma once
#include "motion/core/clock_time.hpp"
#include "motion/kinematics/differential_kinematics.hpp"
#include <map>
#include <set>
namespace motion {
struct TrackingFrameEdge {
  std::string child, parent;
  Pose parent_from_child;
  double position_error_m{}, orientation_error_rad{};
};
struct ResolvedTrackingFrame {
  Pose root_from_frame;
  double position_error_m{}, orientation_error_rad{};
};
class TrackingFrameSnapshot {
public:
  TrackingFrameSnapshot(std::string root, std::vector<TrackingFrameEdge> edges, std::uint64_t revision,
                        ClockTime from, ClockTime until)
      : root_(std::move(root)), revision_(revision), from_(from), until_(until) {
    if (root_.empty() || !revision || !from.clock || from.clock != until.clock ||
        !std::isfinite(from.seconds) || !std::isfinite(until.seconds) || from.seconds >= until.seconds ||
        from.uncertainty_s < 0 || until.uncertainty_s < 0 || edges.size() > 128)
      throw std::invalid_argument("Invalid calibration frame snapshot");
    for (const auto &e : edges) {
      if (e.child.empty() || e.child == root_ || e.parent.empty() ||
          !kinematic_detail::finite_pose(e.parent_from_child) || !std::isfinite(e.position_error_m) ||
          e.position_error_m < 0 || !std::isfinite(e.orientation_error_rad) || e.orientation_error_rad < 0 ||
          !edges_.emplace(e.child, e).second)
        throw std::invalid_argument("Invalid calibration edge");
    }
    for (const auto &[name, _] : edges_)
      (void)resolve(name);
  }
  const std::string &root() const { return root_; }
  std::uint64_t revision() const { return revision_; }
  ClockId clock() const { return from_.clock; }
  bool valid(ClockTime t) const {
    return t.clock == clock() && std::isfinite(t.seconds) && std::isfinite(t.uncertainty_s) &&
           t.uncertainty_s >= 0 && t.seconds - t.uncertainty_s >= from_.seconds + from_.uncertainty_s &&
           t.seconds + t.uncertainty_s <= until_.seconds - until_.uncertainty_s;
  }
  ResolvedTrackingFrame resolve(std::string frame) const {
    std::vector<const TrackingFrameEdge *> chain;
    std::set<std::string> seen;
    while (frame != root_) {
      if (!seen.insert(frame).second || !edges_.count(frame))
        throw std::invalid_argument("Disconnected/cyclic calibration frame");
      const auto &e = edges_.at(frame);
      chain.push_back(&e);
      frame = e.parent;
    }
    ResolvedTrackingFrame r;
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
      const auto &e = **it;
      r.position_error_m +=
          e.position_error_m + 2 * std::sin(std::min(kinematic_detail::pi, r.orientation_error_rad) / 2) *
                                   kinematic_detail::norm(e.parent_from_child.position);
      r.orientation_error_rad =
          std::min(kinematic_detail::pi, r.orientation_error_rad + e.orientation_error_rad);
      r.root_from_frame = tracking_detail::compose(r.root_from_frame, e.parent_from_child);
    }
    return r;
  }

private:
  std::string root_;
  std::uint64_t revision_;
  ClockTime from_, until_;
  std::map<std::string, TrackingFrameEdge> edges_;
};
} // namespace motion
