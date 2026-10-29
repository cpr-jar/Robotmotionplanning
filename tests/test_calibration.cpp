#include "motion/calibration.hpp"
#include "motion/pose_path.hpp"
#include <cassert>
#include <iostream>
using namespace motion;
int main() {
  const Pose truth{{.1, -.2, .3}, {std::cos(.2), 0, 0, std::sin(.2)}};
  std::vector<PointCorrespondence> points;
  for (auto p :
       std::vector<kinematic_detail::V3>{{0, 0, 0}, {.2, 0, 0}, {0, .3, 0}, {0, 0, .4}, {.1, .1, .1}})
    points.push_back(
        {p, kinematic_detail::add(truth.position, tracking_detail::rotate(truth.orientation, p)), 1});
  auto fit = fit_rigid_calibration(points, 1e-8);
  assert(fit && fit.rms_m < 1e-12);
  assert(rotation_distance(fit.target_from_source->orientation, truth.orientation) < 1e-12);
  const kinematic_detail::V3 held{.13, -.04, .27};
  const auto expected =
                 kinematic_detail::add(truth.position, tracking_detail::rotate(truth.orientation, held)),
             actual =
                 kinematic_detail::add(fit.target_from_source->position,
                                       tracking_detail::rotate(fit.target_from_source->orientation, held));
  assert(kinematic_detail::norm(kinematic_detail::sub(expected, actual)) < 1e-12);
  points.back().target[0] += .05;
  assert(fit_rigid_calibration(points, .001).status == CalibrationStatus::ResidualRejected);
  assert(fit_rigid_calibration(
             {{{0, 0, 0}, {0, 0, 0}, 1}, {{1, 0, 0}, {1, 0, 0}, 1}, {{2, 0, 0}, {2, 0, 0}, 1}}, .01)
             .status == CalibrationStatus::Degenerate);
  std::cout << "Weighted rigid fit, held-out transform direction and degenerate/outlier rejection passed\n";
}
