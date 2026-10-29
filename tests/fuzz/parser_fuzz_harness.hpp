#pragma once
#include "motion/program/robot_program.hpp"
#include <stdexcept>
namespace motion_test {
inline void require_parser_invariant(bool condition) {
  if (!condition)
    throw std::logic_error("Accepted parser output violates its invariant");
}
// Parse only: arbitrary profile/URDF filenames never trigger filesystem access.
inline bool fuzz_parse(std::string_view bytes) {
  using namespace motion;
  try {
    const auto source = parse_robot_program_yaml(bytes, "fuzz-input.yaml");
    require_parser_invariant(!source.robot_profile().empty() && !source.start().empty());
    require_parser_invariant(source.start().size() <= 12 && !source.instructions().empty() &&
                             source.instructions().size() <= 10000);
    for (double x : source.start())
      require_parser_invariant(std::isfinite(x));
    require_parser_invariant(std::isfinite(source.epoch_s()) && source.cycle_s() >= .0001 &&
                             source.cycle_s() <= 1);
    for (const auto &step : source.instructions()) {
      require_parser_invariant(step.speed > 0 && step.speed <= 1 && std::isfinite(step.speed));
      require_parser_invariant(step.location.line && step.location.column && step.location.step);
      if (const auto *joints = std::get_if<Vector>(&step.target)) {
        for (double x : *joints)
          require_parser_invariant(std::isfinite(x));
      } else
        require_parser_invariant(kinematic_detail::finite_pose(std::get<Pose>(step.target)));
      if (step.operation == ProgramOperation::Wait)
        require_parser_invariant(std::isfinite(step.seconds) && step.seconds > 0);
      if (step.operation == ProgramOperation::Circle) {
        require_parser_invariant(std::isfinite(step.sweep_rad) && step.sweep_rad != 0 &&
                                 std::abs(step.sweep_rad) <= 2 * kinematic_detail::pi);
        for (double x : step.center)
          require_parser_invariant(std::isfinite(x));
        for (double x : step.normal)
          require_parser_invariant(std::isfinite(x));
        require_parser_invariant(
            std::any_of(step.normal.begin(), step.normal.end(), [](double x) { return x != 0; }));
      }
      if (step.operation == ProgramOperation::Dxf) {
        require_parser_invariant(step.dxf && !step.dxf->file.empty());
        require_parser_invariant(kinematic_detail::finite_pose(step.dxf->placement));
        require_parser_invariant(step.path_tolerance_m >= 1e-6 && step.path_tolerance_m <= .001);
      }
    }
    return true;
  } catch (const RobotProgramError &e) {
    require_parser_invariant(!e.detail().empty() && e.location().file == "fuzz-input.yaml");
    return false;
  }
}
} // namespace motion_test
