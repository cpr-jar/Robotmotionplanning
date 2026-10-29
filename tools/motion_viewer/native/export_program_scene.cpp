// Viewer-only exporter: use the admitted model and the existing virtual dispatcher.
// No library implementation is duplicated and no hardware is contacted.
#include "motion/program/robot_program.hpp"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <streambuf>
#include <string_view>

namespace {
class BudgetedTraceBuffer final : public std::streambuf {
public:
  const std::string &data() const { return data_; }
  bool exceeded() const { return exceeded_; }

protected:
  std::streamsize xsputn(const char *text, std::streamsize count) override {
    if (count < 0 || std::size_t(count) > byte_limit - data_.size()) {
      exceeded_ = true;
      return 0;
    }
    data_.append(text, std::size_t(count));
    return count;
  }
  int_type overflow(int_type value) override {
    if (traits_type::eq_int_type(value, traits_type::eof()))
      return traits_type::not_eof(value);
    const char character = traits_type::to_char_type(value);
    return xsputn(&character, 1) == 1 ? value : traits_type::eof();
  }

private:
  static constexpr std::size_t byte_limit = 64 * 1024 * 1024;
  std::string data_;
  bool exceeded_{};
};

void text(std::ostream &output, std::string_view value) {
  constexpr char hex[] = "0123456789abcdef";
  output << '"';
  for (const unsigned char character : value) {
    if (character == '"' || character == '\\')
      output << '\\' << char(character);
    else if (character < 32)
      output << "\\u00" << hex[character >> 4] << hex[character & 15];
    else
      output << char(character);
  }
  output << '"';
}
template <class Range> void numbers(std::ostream &output, const Range &values) {
  output << '[';
  bool first = true;
  for (double value : values) {
    if (!std::isfinite(value))
      throw std::runtime_error("Nonfinite visualization metadata");
    if (!first)
      output << ',';
    first = false;
    output << value;
  }
  output << ']';
}
void pose(std::ostream &output, const motion::Pose &value) {
  output << "{\"position\":";
  numbers(output, value.position);
  output << ",\"orientation\":";
  numbers(output, value.orientation);
  output << '}';
}
const char *joint_type(motion::SerialJointType type) {
  using motion::SerialJointType;
  switch (type) {
  case SerialJointType::Fixed:
    return "fixed";
  case SerialJointType::Revolute:
    return "revolute";
  case SerialJointType::Continuous:
    return "continuous";
  case SerialJointType::Prismatic:
    return "prismatic";
  }
  throw std::logic_error("Unknown joint type");
}
const char *operation(motion::ProgramOperation value) {
  using motion::ProgramOperation;
  switch (value) {
  case ProgramOperation::Ptp:
    return "PTP";
  case ProgramOperation::Line:
    return "LIN";
  case ProgramOperation::Circle:
    return "CIRC";
  case ProgramOperation::Wait:
    return "WAIT";
  }
  throw std::logic_error("Unknown program operation");
}
void write_scene(const motion::PreparedRobotProgram &prepared, std::ostream &output, const std::string &csv,
                 const motion::ProgramSimulationReport &report) {
  const auto &program = *prepared.reference;
  const auto &schedule = *prepared.schedule;
  const auto &robot = program.robot();
  const auto &model = *robot.kinematics;
  output.imbue(std::locale::classic());
  output << std::setprecision(17);
  output << "{\"format\":\"robot-motion-viewer\",\"schema_version\":1,\"robot\":{\"name\":";
  text(output, robot.name);
  output << ",\"base_link\":";
  text(output, robot.base_link);
  output << ",\"tip_link\":";
  text(output, robot.tip_link);
  output << ",\"base_pose\":";
  pose(output, model.base_pose());
  output << ",\"tool_offset\":";
  pose(output, model.tool_offset());
  output << ",\"joint_names\":[";
  bool first = true;
  for (const auto &name : model.joint_names()) {
    if (!first)
      output << ',';
    first = false;
    text(output, name);
  }
  output << "],\"joints\":[";
  first = true;
  for (const auto &joint : model.joints()) {
    if (!first)
      output << ',';
    first = false;
    output << "{\"name\":";
    text(output, joint.name);
    output << ",\"parent_link\":";
    text(output, joint.parent_link);
    output << ",\"child_link\":";
    text(output, joint.child_link);
    output << ",\"type\":";
    text(output, joint_type(joint.type));
    output << ",\"origin\":";
    pose(output, joint.origin);
    output << ",\"axis\":";
    numbers(output, joint.axis);
    output << '}';
  }
  output << "],\"limits\":{\"lower\":";
  numbers(output, robot.joint_limits.min_position);
  output << ",\"upper\":";
  numbers(output, robot.joint_limits.max_position);
  output << ",\"velocity\":";
  numbers(output, robot.joint_limits.max_velocity);
  output << ",\"acceleration\":";
  numbers(output, robot.joint_limits.max_acceleration);
  output << ",\"jerk\":";
  numbers(output, robot.joint_limits.max_jerk);
  const auto &caps = robot.tcp_limits;
  output << "},\"tcp_limits\":{\"linear\":";
  numbers(output, std::array{caps.max_linear_velocity, caps.max_linear_acceleration, caps.max_linear_jerk});
  output << ",\"angular\":";
  numbers(output,
          std::array{caps.max_angular_velocity, caps.max_angular_acceleration, caps.max_angular_jerk});
  output << "}},\"program\":{\"cycle_s\":" << schedule.cycle_s() << ",\"duration_s\":" << schedule.duration()
         << ",\"total_ticks\":" << schedule.total_ticks() << ",\"segments\":[";
  for (std::size_t index = 0; index < program.segment_count(); ++index) {
    if (index)
      output << ',';
    const auto &instruction = program.instruction(index);
    output << "{\"index\":" << index + 1 << ",\"operation\":";
    text(output, operation(instruction.operation));
    output << ",\"source_file\":";
    text(output, std::filesystem::path(instruction.location.file).filename().string());
    output << ",\"source_block\":";
    text(output, instruction.location.block);
    output << ",\"source_step\":" << instruction.location.step
           << ",\"source_line\":" << instruction.location.line
           << ",\"start_s\":" << (index ? schedule.segment_end(index - 1) : 0)
           << ",\"end_s\":" << schedule.segment_end(index) << ",\"speed\":" << instruction.speed << '}';
  }
  output << "],\"commands\":" << report.commands << "},\"trace_csv\":";
  text(output, csv);
  output << "}\n";
  if (!output)
    throw std::runtime_error("Cannot finish visualization export");
}

// Local editor programs resolve paths from the project root, never from a
// temporary run folder. Only actual files below models/ may be imported.
motion::PreparedRobotProgram prepare_editor_program(const std::filesystem::path &filename,
                                                    const std::filesystem::path &workspace) {
  const auto source = motion::load_robot_program_source(filename.string());
  const auto root = std::filesystem::canonical(workspace);
  const auto models = std::filesystem::canonical(root / "models");
  const auto model_file = [&](const std::filesystem::path &relative) {
    if (relative.is_absolute())
      throw motion::RobotProgramError(
          source.robot_location(),
          "Editor model paths must be relative to the project root: models/<robot>/profile.yaml");
    const auto file = std::filesystem::weakly_canonical(root / relative);
    const auto prefix = std::mismatch(models.begin(), models.end(), file.begin(), file.end());
    if (prefix.first != models.end() || !std::filesystem::is_regular_file(file))
      throw motion::RobotProgramError(
          source.robot_location(),
          "Editor model file must exist inside models/; paths are relative to the project root");
    return file;
  };
  const auto profile = model_file(source.robot_profile());
  const auto urdf = source.robot_urdf().empty()
                        ? model_file(std::filesystem::relative(profile.parent_path() / "robot.urdf", root))
                        : model_file(source.robot_urdf());
  auto robot = [&]() {
    try {
      return motion::load_serial_robot_files(urdf.string(), profile.string());
    } catch (const std::exception &error) {
      throw motion::RobotProgramError(source.robot_location(), error.what());
    }
  }();
  return motion::compile_robot_program(source, std::move(robot));
}

void editor_diagnostic(const std::exception &error, const motion::ProgramLocation *location = nullptr) {
  std::cerr << "{\"message\":";
  text(std::cerr, error.what());
  if (location) {
    std::cerr << ",\"filename\":";
    text(std::cerr, std::filesystem::path(location->file).filename().string());
    std::cerr << ",\"line\":" << location->line << ",\"column\":" << location->column
              << ",\"step\":" << location->step << ",\"block\":";
    text(std::cerr, location->block);
  }
  std::cerr << "}\n";
}
} // namespace

int main(int argc, char **argv) {
  std::filesystem::path temporary;
  const bool editor_mode = argc == 7 && std::string_view(argv[5]) == "--workspace";
  try {
    if ((argc < 3 || argc > 5) && !editor_mode)
      throw std::invalid_argument("Usage: motion_viewer_exporter PROGRAM.yaml OUTPUT.json [cycle_ms:1..20] "
                                  "[max_ticks:1..99999] [--workspace PROJECT_ROOT]");
    const std::filesystem::path destination = argv[2];
    if (destination.extension() != ".json")
      throw std::invalid_argument("Output needs .json extension");
    const std::filesystem::path source = argv[1];
    std::error_code equivalence_error;
    if (std::filesystem::absolute(source).lexically_normal() ==
            std::filesystem::absolute(destination).lexically_normal() ||
        std::filesystem::equivalent(source, destination, equivalence_error))
      throw std::invalid_argument("Input and output must differ");
    auto prepared =
        editor_mode ? prepare_editor_program(source, argv[6]) : motion::load_robot_program(argv[1]);
    const auto integer = [](const char *value, unsigned lower, unsigned upper) {
      const std::string_view input(value);
      unsigned result = 0;
      const auto parsed = std::from_chars(input.data(), input.data() + input.size(), result);
      if (parsed.ec != std::errc{} || parsed.ptr != input.data() + input.size() || result < lower ||
          result > upper)
        throw std::invalid_argument("Integer argument outside permitted range");
      return result;
    };
    if (argc >= 4) {
      const auto aligned = motion::align_program_cycles(prepared.reference, integer(argv[3], 1, 20) * .001);
      if (!aligned)
        throw std::invalid_argument(aligned.diagnostic);
      prepared.schedule = aligned.program;
    }
    const unsigned max_ticks = argc >= 5 ? integer(argv[4], 1, 99999) : 99999;
    BudgetedTraceBuffer trace_buffer;
    std::ostream trace_stream(&trace_buffer);
    motion::ProgramSimulationReport report;
    try {
      report = motion::simulate_robot_program(prepared, &trace_stream, {max_ticks});
    } catch (const std::exception &) {
      if (trace_buffer.exceeded())
        throw std::runtime_error("Viewer CSV budget of 64 MiB exceeded");
      throw;
    }
    temporary = destination;
    temporary += ".tmp-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    if (std::filesystem::exists(temporary))
      throw std::runtime_error("Temporary filename exists");
    std::ofstream file(temporary, std::ios::binary);
    if (!file)
      throw std::runtime_error("Cannot create visualization output");
    write_scene(prepared, file, trace_buffer.data(), report);
    file.close();
    if (!file)
      throw std::runtime_error("Cannot close visualization output");
    std::filesystem::rename(temporary, destination);
    temporary.clear();
    std::cout << report.commands << " virtually dispatched commands exported; simulation only\n";
  } catch (const std::exception &error) {
    if (!temporary.empty()) {
      std::error_code ignored;
      std::filesystem::remove(temporary, ignored);
    }
    if (editor_mode) {
      const auto *program_error = dynamic_cast<const motion::RobotProgramError *>(&error);
      if (program_error) {
        const std::runtime_error detail(program_error->detail());
        editor_diagnostic(detail, &program_error->location());
      } else
        editor_diagnostic(error);
    } else
      std::cerr << error.what() << '\n';
    return 1;
  }
}
