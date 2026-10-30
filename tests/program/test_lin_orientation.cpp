#include "motion/planning/pose_path.hpp"
#include "motion/program/robot_program.hpp"
#include <cassert>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
using namespace motion;
namespace {
const Vector home{2, -.6, .7, .3, .8, -.2};
const PathVector3 nearby{-.47390467854451826, .62398337528833302, .24366518271764495};
const std::array<double, 4> old_orientation{.1404804311152778, .25362367811777614, .7585188460561569,
                                            .5836004099975534};
ImportedSerialRobot robot() {
  const auto folder = std::filesystem::path(MOTION_SOURCE_DIR) / "models/ur5e";
  return load_serial_robot_files((folder / "robot.urdf").string(),
                                 (folder / "profile_pick_place_simulation.yaml").string());
}
std::string prefix() {
  return "schema_version: 1\nunits: SI\nrobot_profile: profile.yaml\nstart: {joints: "
         "[2,-0.6,0.7,0.3,0.8,-0.2]}\n";
}
std::string pose_target(const PathVector3 &position, const std::array<double, 4> &orientation) {
  std::ostringstream text;
  text << std::setprecision(17) << "{position: [" << position[0] << ',' << position[1] << ',' << position[2]
       << "], orientation: [";
  for (std::size_t i = 0; i < orientation.size(); ++i)
    text << (i ? "," : "") << orientation[i];
  return text.str() + "]}";
}
PreparedRobotProgram compile(const std::string &body) {
  return compile_robot_program(parse_robot_program_yaml(prefix() + body, "lin-orientation.yaml"), robot());
}
void check_line(const PreparedRobotProgram &program, std::size_t segment, const PathVector3 &goal) {
  const auto &model = *program.reference->robot().kinematics;
  const auto begin = model.fk(program.reference->sample_segment(segment, 0).q);
  for (unsigned i = 0; i <= 200; ++i) {
    const auto state =
        program.reference->sample_segment(segment, program.reference->segment_duration(segment) * (i / 200.));
    const auto actual = model.fk(state.q);
    assert(rotation_distance(actual.orientation, begin.orientation) < 5e-4);
    const auto direction = kinematic_detail::sub(goal, begin.position);
    const auto displacement = kinematic_detail::sub(actual.position, begin.position);
    assert(kinematic_detail::norm(kinematic_detail::cross(direction, displacement)) /
               kinematic_detail::norm(direction) <
           2e-4);
    assert(within_joint_limits(state, program.reference->joint_limits()));
  }
  const auto last =
      model.fk(program.reference->sample_segment(segment, program.reference->segment_duration(segment)).q);
  assert(kinematic_detail::norm(kinematic_detail::sub(last.position, goal)) < 1e-6);
}
void modes() {
  const auto command = "steps: [{type: LIN, target: " + pose_target(nearby, old_orientation);
  const auto default_keep = compile(command + "}]\n");
  const auto explicit_keep = compile(command + ", orientation_mode: keep}]\n");
  assert(default_keep.reference->instruction(0).linear_orientation_mode == LinearOrientationMode::Keep);
  check_line(default_keep, 0, nearby);
  for (unsigned i = 0; i <= 100; ++i)
    assert(default_keep.reference->sample(default_keep.reference->duration() * (i / 100.)).q ==
           explicit_keep.reference->sample(explicit_keep.reference->duration() * (i / 100.)).q);
  // Intentionally commanded rotation remains possible, independently of LIN's default.
  auto model = robot().kinematics;
  const auto initial = model->fk(home);
  const auto rotated = tracking_detail::multiply(std::array<double, 4>{std::cos(.01), 0, 0, std::sin(.01)},
                                                 initial.orientation);
  const auto turn = compile("steps: [{type: LIN, target: " + pose_target(nearby, rotated) +
                            ", orientation_mode: interpolate}]\n");
  const auto final = model->fk(simulate_robot_program(turn).final_state.q);
  assert(rotation_distance(final.orientation, rotated) < 1e-6);
  assert(rotation_distance(final.orientation, initial.orientation) > .0199);
  // A named joint point supplies the Cartesian POSITION; its FK orientation is
  // not an implicit rotation request in the default mode.
  Vector joint_goal = home;
  joint_goal[0] += .01;
  const auto joint_point =
      compile("points: {end: {joints: [2.01,-0.6,0.7,0.3,0.8,-0.2]}}\nsteps: [{type: LIN, target: end}]\n");
  check_line(joint_point, 0, model->fk(joint_goal).position);
  for (const std::string &invalid : std::vector<std::string>{
           "steps: [{type: LIN, target: " + pose_target(nearby, old_orientation) +
               ", orientation_mode: fly}]",
           "steps: [{type: PTP, joints: [2,-0.6,0.7,0.3,0.8,-0.2], orientation_mode: keep}]"}) {
    bool rejected = false;
    try {
      compile(invalid);
    } catch (const RobotProgramError &e) {
      rejected = true;
      assert(e.location().step == 1);
    }
    assert(rejected);
  }
}
void after_ptp_and_grids() {
  const auto sequence =
      compile("points: {wide: {joints: [0.7,-0.9,1,-0.3,1.1,0.2]}, home: {joints: "
              "[2,-0.6,0.7,0.3,0.8,-0.2]}}\nsteps:\n"
              " - {type: PTP, target: wide}\n - {type: PTP, target: home}\n - {type: LIN, target: " +
              pose_target(nearby, old_orientation) +
              "}\n"
              " - {type: CIRC, center_offset: [-0.015,0,0], normal: [0,0,1], sweep_rad: 6.283185307179586}\n "
              "- {type: PTP, target: home}\n");
  check_line(sequence, 2, nearby);
  const auto &model = *sequence.reference->robot().kinematics;
  const auto expected = model.fk(home).orientation;
  for (double cycle : {.001, .01, .02}) {
    const auto retimed = retime_robot_program(sequence, cycle);
    const auto &grid = *retimed.schedule;
    for (auto tick = grid.boundary_ticks()[1]; tick <= grid.boundary_ticks()[3]; ++tick) {
      const auto state = grid.sample_tick(tick);
      assert(rotation_distance(model.fk(state.q).orientation, expected) < 5e-4);
      assert(within_joint_limits(state, sequence.reference->joint_limits()));
    }
    assert(simulate_robot_program(retimed).final_state.q == home);
  }
  // The supplied far target must be rejected if a fixed-orientation path cannot
  // be admitted. The planner may not fall back to rotation to make it reachable.
  bool rejected = false;
  try {
    compile("steps: [{type: LIN, target: " +
            pose_target({.7444144539141021, .3622280684167766, .24366518271764495}, old_orientation) +
            "}]\n");
  } catch (const RobotProgramError &e) {
    rejected = true;
    assert(e.location().step == 1);
  }
  assert(rejected);
}
void dxf_entry() {
  const auto model = robot().kinematics;
  const auto first = model->fk(home);
  const auto desired =
      tracking_detail::multiply(std::array<double, 4>{std::cos(.01), 0, 0, std::sin(.01)}, first.orientation);
  const std::string drawing =
      "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n0\n20\n0\n11\n1\n21\n0\n0\nENDSEC\n0\nEOF\n";
  std::ostringstream q;
  q << std::setprecision(17) << '[';
  for (std::size_t i = 0; i < 4; ++i)
    q << (i ? "," : "") << desired[i];
  q << ']';
  const auto source = parse_robot_program_yaml(
      prefix() + "steps: [{type: DXF, file: line.dxf, units: mm, travel: LIN, orientation: " + q.str() +
      "}]\n");
  const auto program = compile_robot_program(source, robot(), [&](const auto &) { return drawing; });
  assert(program.reference->segment_count() == 2);
  assert(program.reference->instruction(0).linear_orientation_mode == LinearOrientationMode::Interpolate);
  assert(program.reference->instruction(1).linear_orientation_mode == LinearOrientationMode::Keep);
  const auto entered = model->fk(program.reference->sample_segment(1, 0).q);
  assert(rotation_distance(entered.orientation, desired) < 1e-6);
  const auto last = model->fk(program.reference->sample(program.reference->duration()).q);
  check_line(program, 1, last.position);
}
} // namespace
int main() {
  modes();
  after_ptp_and_grids();
  dxf_entry();
  std::cout << "LIN retains the actual start orientation; explicit rotation, joint targets, DXF entry and "
               "1/10/20 ms grids passed\n";
}
