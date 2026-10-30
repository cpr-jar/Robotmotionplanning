#pragma once
#include "motion/core/preparation_context.hpp"
#include "motion/execution/cycle_aligned_program.hpp"
#include "motion/io/dxf_path_import.hpp"
#include "motion/io/robot_model_import.hpp"
#include "motion/planning/path_time_parameterization.hpp"
#include <functional>
#include <iosfwd>
#include <variant>
namespace motion {
struct ProgramLocation {
  std::string file, block{"steps"};
  std::size_t line{}, column{}, step{};
};
class RobotProgramError : public std::runtime_error {
public:
  RobotProgramError(ProgramLocation, const std::string &);
  const ProgramLocation &location() const noexcept { return location_; }
  const std::string &detail() const noexcept { return detail_; }

private:
  ProgramLocation location_;
  std::string detail_;
};
using ProgramPoint = std::variant<Vector, Pose>;
enum class ProgramOperation { Ptp, Line, Circle, Wait, Dxf };
enum class DxfTravel { Reject, Line, Ptp };
/// LIN keeps the actual start TCP orientation in world coordinates by default.
enum class LinearOrientationMode { Keep, Interpolate };
struct DxfProgramCommand {
  std::string file;
  DxfImportOptions import;
  bool relative_to_tcp{true};
  Pose placement;
  DxfTravel travel{DxfTravel::Reject};
};
struct ProgramInstruction {
  ProgramOperation operation{};
  ProgramPoint target{Vector{}};
  bool relative_center{};
  PathVector3 center{}, normal{};
  double sweep_rad{}, seconds{}, speed{1};
  double path_tolerance_m{.0002};
  std::optional<std::array<double, 4>> orientation;
  ProgramLocation location;
  std::optional<DxfProgramCommand> dxf;
  std::optional<DxfLocation> drawing_location;
  std::optional<PathVector3> drawing_start; // Nominal CAD start; preparation checks start/end within 1 um.
  LinearOrientationMode linear_orientation_mode{LinearOrientationMode::Keep};
};
class RobotProgramSource {
public:
  const std::string &robot_profile() const { return profile_; }
  const std::string &robot_urdf() const { return urdf_; }
  const std::string &filename() const { return filename_; }
  const Vector &start() const { return start_; }
  const auto &instructions() const { return steps_; }
  double cycle_s() const { return cycle_; }
  double epoch_s() const { return epoch_; }
  const ProgramLocation &start_location() const { return start_location_; }
  const ProgramLocation &robot_location() const { return robot_location_; }

private:
  friend RobotProgramSource parse_robot_program_yaml(std::string_view, const std::string &,
                                                     PreparationContext *);
  RobotProgramSource() = default;
  std::string profile_, urdf_, filename_;
  Vector start_;
  std::vector<ProgramInstruction> steps_;
  double cycle_{.01}, epoch_{};
  ProgramLocation start_location_, robot_location_;
};
RobotProgramSource parse_robot_program_yaml(std::string_view, const std::string &filename = "<memory>");
RobotProgramSource parse_robot_program_yaml(std::string_view, const std::string &, PreparationContext *);
RobotProgramSource load_robot_program_source(const std::string &);
RobotProgramSource load_robot_program_source(const std::string &, PreparationContext *);
class RobotProgram;
struct PreparedRobotProgram {
  // Public fields are retained for source compatibility. Prefer the checked
  // prepare/retime factories; consumers must validate legacy assembled pairs.
  std::shared_ptr<const RobotProgram> reference;
  std::shared_ptr<const CycleAlignedProgram> schedule;
  void validate() const;
};
/// Preparation-thread operations; throw invalid_argument without publishing a partial result.
PreparedRobotProgram prepare_robot_program(std::shared_ptr<const RobotProgram>, double cycle_s);
PreparedRobotProgram retime_robot_program(const PreparedRobotProgram &, double cycle_s);
PreparedRobotProgram prepare_robot_program(std::shared_ptr<const RobotProgram>, double, PreparationContext *);
PreparedRobotProgram retime_robot_program(const PreparedRobotProgram &, double, PreparationContext *);
PreparedRobotProgram compile_robot_program(const RobotProgramSource &, ImportedSerialRobot);
// Optional asset loader snapshots external DXF text during preparation. No IO
// happens when the finished program is sampled or executed.
using ProgramAssetLoader = std::function<std::string(const std::string &)>;
PreparedRobotProgram compile_robot_program(const RobotProgramSource &, ImportedSerialRobot,
                                           const ProgramAssetLoader &);
PreparedRobotProgram compile_robot_program(const RobotProgramSource &, ImportedSerialRobot,
                                           const ProgramAssetLoader &, PreparationContext *);
PreparedRobotProgram load_robot_program(const std::string &);
PreparedRobotProgram load_robot_program(const std::string &, PreparationContext *);
class RobotProgram final : public IJointProgram {
public:
  double duration() const noexcept override { return ends_.back(); }
  std::size_t segment_count() const noexcept override { return segments_.size(); }
  double segment_end(std::size_t i) const override { return ends_.at(i); }
  double segment_duration(std::size_t i) const override { return segments_.at(i)->duration(); }
  JointState sample_segment(std::size_t, double) const override;
  JointState sample(double) const override;
  const JointLimits &joint_limits() const noexcept override { return robot_.joint_limits; }
  const JointState &initial_state() const noexcept override { return initial_; }
  const ImportedSerialRobot &robot() const { return robot_; }
  const ProgramInstruction &instruction(std::size_t i) const { return instructions_.at(i); }
  TimedPathSample sample_motion(std::size_t, double) const;

private:
  friend PreparedRobotProgram compile_robot_program(const RobotProgramSource &, ImportedSerialRobot);
  friend PreparedRobotProgram compile_robot_program(const RobotProgramSource &, ImportedSerialRobot,
                                                    const ProgramAssetLoader &, PreparationContext *);
  explicit RobotProgram(ImportedSerialRobot robot) : robot_(std::move(robot)) {}
  ImportedSerialRobot robot_;
  JointState initial_;
  std::vector<TrajectoryPtr> segments_;
  Vector ends_;
  std::vector<ProgramInstruction> instructions_;
};
struct ProgramSimulationOptions {
  std::uint64_t max_ticks{1000000};
  PreparationContext *preparation{};
};
struct ProgramSimulationReport {
  double duration_s{};
  std::uint64_t commands{}, rest_boundaries{};
  Vector peak_joint_velocity, peak_joint_acceleration, peak_joint_jerk;
  std::array<double, 3> peak_tcp_linear{}, peak_tcp_angular{};
  JointState final_state;
};
ProgramSimulationReport simulate_robot_program(const PreparedRobotProgram &, std::ostream *csv = nullptr,
                                               ProgramSimulationOptions = {});
} // namespace motion
