#include "motion/program/robot_program.hpp"
namespace motion {
void PreparedRobotProgram::validate() const {
  if (!reference || !schedule)
    throw std::invalid_argument("Invalid prepared program: missing reference or schedule");
  if (!schedule->references(*reference))
    throw std::invalid_argument("Execution schedule does not belong to program reference");
}
PreparedRobotProgram prepare_robot_program(std::shared_ptr<const RobotProgram> reference, double cycle) {
  return prepare_robot_program(std::move(reference), cycle, nullptr);
}
PreparedRobotProgram prepare_robot_program(std::shared_ptr<const RobotProgram> reference, double cycle,
                                           PreparationContext *context) {
  if (!reference)
    throw std::invalid_argument("Invalid program reference");
  const auto grid = align_program_cycles(reference, cycle, context);
  if (!grid)
    throw std::invalid_argument(grid.diagnostic);
  const double epoch = reference->initial_state().timestamp_s,
               magnitude = std::max(std::abs(epoch), std::abs(epoch + grid.program->duration())),
               ulp = std::nextafter(magnitude, INFINITY) - magnitude;
  if (cycle <= 64 * ulp)
    throw std::invalid_argument("Insufficient clock precision for dispatch");
  return {std::move(reference), grid.program};
}
PreparedRobotProgram retime_robot_program(const PreparedRobotProgram &prepared, double cycle) {
  return retime_robot_program(prepared, cycle, nullptr);
}
PreparedRobotProgram retime_robot_program(const PreparedRobotProgram &prepared, double cycle,
                                          PreparationContext *context) {
  prepared.validate();
  return prepare_robot_program(prepared.reference, cycle, context);
}
} // namespace motion
