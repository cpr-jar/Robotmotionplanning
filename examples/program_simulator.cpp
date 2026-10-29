#include "motion/robot_program.hpp"
#include <charconv>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
namespace {
const char *name(motion::ProgramOperation op) {
  switch (op) {
  case motion::ProgramOperation::Ptp:
    return "PTP";
  case motion::ProgramOperation::Line:
    return "LIN";
  case motion::ProgramOperation::Circle:
    return "CIRC";
  case motion::ProgramOperation::Wait:
    return "WAIT";
  }
  return "?";
}
} // namespace
int main(int argc, char **argv) {
  const char *usage =
      "Usage: program_simulator program.yaml [--check] [--csv trace.csv] [--max-ticks count]\n";
  std::filesystem::path temporary;
  try {
    std::string input, output;
    bool check = false;
    motion::ProgramSimulationOptions options;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") {
        std::cout << usage;
        return 0;
      }
      if (arg == "--check") {
        if (check)
          throw std::invalid_argument("Duplicate --check");
        check = true;
      } else if (arg == "--csv") {
        if (!output.empty() || i + 1 >= argc)
          throw std::invalid_argument("--csv requires output filename");
        output = argv[++i];
        if (std::filesystem::path(output).extension() != ".csv")
          throw std::invalid_argument("CSV output needs .csv extension");
      } else if (arg == "--max-ticks") {
        if (i + 1 >= argc)
          throw std::invalid_argument("--max-ticks requires count");
        std::string count = argv[++i];
        auto p = std::from_chars(count.data(), count.data() + count.size(), options.max_ticks);
        if (p.ec != std::errc{} || p.ptr != count.data() + count.size() || options.max_ticks == 0 ||
            options.max_ticks > 10000000)
          throw std::invalid_argument("Tick budget must be 1..10000000");
      } else if (arg.starts_with('-') || !input.empty())
        throw std::invalid_argument("Unknown/extra argument: " + arg);
      else
        input = arg;
    }
    if (input.empty() || (check && !output.empty()))
      throw std::invalid_argument("Supply a program; --check cannot combine with --csv");
    const auto prepared = motion::load_robot_program(input);
    const auto &program = *prepared.reference;
    const auto &grid = *prepared.schedule;
    std::cout << program.robot().name << ": " << grid.segment_count() << " admitted exact-stop steps, "
              << grid.duration() << " s, cycle=" << grid.cycle_s() << " s\n";
    for (std::size_t i = 0; i < program.segment_count(); ++i) {
      const auto &op = program.instruction(i);
      std::cout << i + 1 << ": " << name(op.operation) << " " << grid.segment_duration(i) << " s ["
                << op.location.block << " step " << op.location.step << ", line " << op.location.line
                << "]\n";
    }
    if (check) {
      std::cout << "Complete program admitted; simulation not requested\n";
      return 0;
    }
    if (grid.total_ticks() > options.max_ticks)
      throw std::invalid_argument("Simulation tick budget exceeded");
    std::ofstream file;
    if (!output.empty()) {
      temporary = output;
      temporary += ".tmp-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
      if (std::filesystem::exists(temporary))
        throw std::runtime_error("Temporary filename exists");
      file.open(temporary, std::ios::binary);
      if (!file)
        throw std::runtime_error("Cannot create CSV");
    }
    const auto report = motion::simulate_robot_program(prepared, output.empty() ? nullptr : &file, options);
    if (file.is_open()) {
      file.close();
      if (!file)
        throw std::runtime_error("Cannot finish CSV");
      std::filesystem::rename(temporary, output);
      temporary.clear();
    }
    std::cout << report.commands << " commands, " << report.rest_boundaries
              << " rest boundaries, terminal rest; no dispatch fault\nSampled TCP linear v/a/j: "
              << report.peak_tcp_linear[0] << " / " << report.peak_tcp_linear[1] << " / "
              << report.peak_tcp_linear[2] << "\nSampled TCP angular v/a/j: " << report.peak_tcp_angular[0]
              << " / " << report.peak_tcp_angular[1] << " / " << report.peak_tcp_angular[2] << '\n';
    if (!output.empty())
      std::cout << "CSV: " << output << '\n';
  } catch (const std::exception &e) {
    if (!temporary.empty()) {
      std::error_code ec;
      std::filesystem::remove(temporary, ec);
    }
    std::cerr << e.what() << '\n' << usage;
    return 1;
  }
}
