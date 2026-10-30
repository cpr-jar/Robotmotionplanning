#include "motion/program/robot_program.hpp"
#include <cassert>
#include <filesystem>
#include <iostream>
#include <sstream>
using namespace motion;
namespace {
ImportedSerialRobot xyz() {
  auto model = std::make_shared<SerialChainKinematics>(
      std::vector<SerialJoint>{{"x", "base", "x", SerialJointType::Prismatic, {}, {1, 0, 0}, -2, 2, 1},
                               {"y", "x", "y", SerialJointType::Prismatic, {}, {0, 1, 0}, -2, 2, 1},
                               {"z", "y", "tool", SerialJointType::Prismatic, {}, {0, 0, 1}, -2, 2, 1}});
  return {"xyz",
          "base",
          "tool",
          model,
          {Vector(3, -2), Vector(3, 2), Vector(3, 1), Vector(3, 2), Vector(3, 10)},
          {.2, .5, 2, .5, 1, 5}};
}
const std::string prefix =
    "schema_version: 1\nunits: SI\nrobot_profile: profile.yaml\nstart: {joints: [0.1,0.2,0.3]}\n";
const std::string line =
    "0\nSECTION\n2\nENTITIES\n0\nLINE\n8\nPATH\n10\n0\n20\n0\n11\n1\n21\n0\n0\nENDSEC\n0\nEOF\n";
template <class F> void exhausted(F action, PreparationResource resource) {
  bool caught = false;
  try {
    action();
  } catch (const PreparationError &e) {
    caught = true;
    assert(e.code() == PreparationFailure::BudgetExceeded && e.resource() == resource);
  }
  assert(caught);
}
} // namespace
int main() {
  const auto root = std::filesystem::path(MOTION_SOURCE_DIR);
  const auto urdf = root / "models/ur5e/robot.urdf", profile = root / "models/ur5e/profile.yaml";
  PreparationContext model_context;
  const auto imported = load_serial_robot_files(urdf.string(), profile.string(), &model_context);
  assert(imported.kinematics && model_context.used(PreparationResource::InputBytes) ==
                                    std::filesystem::file_size(urdf) + std::filesystem::file_size(profile));
  PreparationBudget model_budget;
  model_budget.input_bytes = std::filesystem::file_size(profile);
  PreparationContext model_limit(model_budget);
  exhausted([&] { load_serial_robot_files(urdf.string(), profile.string(), &model_limit); },
            PreparationResource::InputBytes);
  PreparationContext context;
  const auto text = prefix + "steps: [{type: DXF, file: shape.dxf, units: mm}, {type: DXF, file: shape.dxf, "
                             "units: mm}, {type: DXF, file: shape.dxf, units: cm}]";
  const auto source = parse_robot_program_yaml(text, "budget.yaml", &context);
  int loads = 0;
  const auto prepared = compile_robot_program(
      source, xyz(),
      [&](const auto &) {
        ++loads;
        return line;
      },
      &context);
  assert(loads == 1 && context.used(PreparationResource::Assets) == 1 &&
         context.used(PreparationResource::Drawings) == 2);
  assert(context.used(PreparationResource::InputBytes) == text.size() + line.size());
  assert(context.used(PreparationResource::Instructions) == 3 &&
         context.used(PreparationResource::Work) > 100);
  assert(std::abs(prepared.reference->sample(prepared.reference->duration()).q[0] - .112) < 1e-8);
  assert(simulate_robot_program(prepared, nullptr, {1000000, &context}).commands ==
         context.used(PreparationResource::SimulationTicks));
  PreparationBudget budget;
  budget.drawings = 1;
  PreparationContext drawings(budget);
  exhausted([&] { compile_robot_program(source, xyz(), [&](const auto &) { return line; }, &drawings); },
            PreparationResource::Drawings);
  budget = {};
  budget.instructions = 2;
  PreparationContext steps(budget);
  exhausted([&] { compile_robot_program(source, xyz(), [&](const auto &) { return line; }, &steps); },
            PreparationResource::Instructions);
  budget = {};
  budget.work = 1;
  PreparationContext work(budget);
  exhausted([&] { compile_robot_program(source, xyz(), [&](const auto &) { return line; }, &work); },
            PreparationResource::Work);
  budget = {};
  budget.input_bytes = text.size() - 1;
  PreparationContext bytes(budget);
  exhausted([&] { parse_robot_program_yaml(text, "budget.yaml", &bytes); }, PreparationResource::InputBytes);
  std::stop_source stop;
  PreparationContext cancelled({}, stop.get_token());
  bool caught = false;
  try {
    compile_robot_program(
        source, xyz(),
        [&](const auto &) {
          stop.request_stop();
          return line;
        },
        &cancelled);
  } catch (const PreparationError &e) {
    caught = true;
    assert(e.code() == PreparationFailure::Cancelled);
  }
  assert(caught && cancelled.used(PreparationResource::Drawings) == 0);
  budget = {};
  budget.simulation_ticks = 2;
  PreparationContext ticks(budget);
  std::ostringstream partial;
  exhausted([&] { simulate_robot_program(prepared, &partial, {1000000, &ticks}); },
            PreparationResource::SimulationTicks);
  // Failed preparation never mutates a previously returned plan.
  assert(simulate_robot_program(prepared).final_state.q[0] > .111);
  budget = {};
  budget.work = 0;
  PreparationContext ik(budget);
  exhausted(
      [&] {
        auto model = xyz().kinematics;
        model->ik_detailed(model->fk({.11, .2, .3}), {.1, .2, .3}, &ik);
      },
      PreparationResource::Work);
  std::cout << "Shared parse/compile/simulate budgets, immutable DXF variants, cancellation and retained "
               "prior plan passed\n";
}
