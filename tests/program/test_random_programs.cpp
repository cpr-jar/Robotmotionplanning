#include "support/program_test_fixture.hpp"
#include <cassert>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
using namespace motion;
int main() {
  constexpr unsigned seed = 0xC0DE2026;
  std::mt19937 random(seed);
  std::uniform_real_distribution<double> delta(-.025, .025), speed(.4, 1);
  std::uint64_t commands = 0;
  constexpr int count = 120;
  for (int trial = 0; trial < count; ++trial) {
    std::ostringstream source;
    source << std::setprecision(17) << motion_test::prefix << "epoch_s: " << trial * 10.
           << "\ncycle_s: 0.01\n"
           << "subprograms: {dwell: [{type: WAIT, seconds: 0.013}]}\nsteps:\n";
    Vector end{.1, 0, 0};
    for (int step = 0; step < 6; ++step) {
      const auto operation = random() % 4;
      const double factor = speed(random);
      if (operation < 2) {
        for (auto &x : end)
          x += delta(random);
        if (operation == 0)
          source << " - {type: PTP, joints: [" << end[0] << ',' << end[1] << ',' << end[2];
        else
          source << " - {type: LIN, target: {position: [" << end[0] << ',' << end[1] << ',' << end[2];
        source << (operation == 0 ? "]" : "], orientation: [1,0,0,0]}") << ", speed: " << factor << "}\n";
      } else if (operation == 2 && trial % 20 == 0) {
        // Signed complete circles must return to the independently tracked start.
        source << " - {type: CIRC, center_offset: [-0.01,0,0], normal: [0,0,1], sweep_rad: "
               << (random() % 2 ? 1 : -1) * 2 * kinematic_detail::pi << ", speed: " << factor << "}\n";
      } else
        source << " - {type: CALL, program: dwell}\n";
    }
    PreparedRobotProgram published;
    try {
      published = compile_robot_program(parse_robot_program_yaml(source.str()), motion_test::xyz());
    } catch (const std::exception &e) {
      std::cerr << "seed=" << seed << " trial=" << trial << '\n' << source.str() << e.what() << '\n';
      return 1;
    }
    const auto report = simulate_robot_program(published);
    assert(report.rest_boundaries == 6 && report.commands == published.schedule->total_ticks() + 1);
    assert(is_rest(report.final_state, 0));
    for (int i = 0; i < 3; ++i) {
      assert(std::abs(report.final_state.q[i] - end[i]) < 1e-7);
      assert(report.peak_joint_velocity[i] <= 1 + 1e-8);
      assert(report.peak_joint_acceleration[i] <= 2 + 1e-8);
      assert(report.peak_joint_jerk[i] <= 10 + 1e-8);
    }
    const double caps[3] = {.2, .5, 2};
    for (int order = 0; order < 3; ++order) {
      assert(report.peak_tcp_linear[order] <= caps[order] + 1e-8);
      assert(report.peak_tcp_angular[order] < 1e-10);
    }
    for (std::size_t step = 0; step < published.reference->segment_count(); ++step)
      for (int k = 0; k <= 10; ++k) {
        const auto p =
            published.reference->sample_motion(step, published.reference->segment_duration(step) * (k / 10.));
        assert(within_joint_limits(p.joints, published.reference->joint_limits()));
        for (int i = 0; i < 3; ++i)
          assert(std::abs(p.tcp.position[i] - p.joints.q[i]) < 1e-12); // independent XYZ FK
      }
    // Both syntax and a late semantic failure leave the published object untouched.
    const auto prior_reference = published.reference;
    const auto prior_schedule = published.schedule;
    for (const auto &suffix :
         {std::string(" - {type: UNKNOWN}\n"), std::string(" - {type: PTP, joints: [3,0,0]}\n")}) {
      bool rejected = false;
      try {
        auto candidate = compile_robot_program(
            parse_robot_program_yaml(motion_test::prefix + "steps:\n - {type: PTP, joints: [0.2,0,0]}\n" +
                                     suffix),
            motion_test::xyz());
        published = std::move(candidate);
      } catch (const RobotProgramError &e) {
        rejected = true;
        assert(!e.detail().empty());
      }
      assert(rejected && published.reference == prior_reference && published.schedule == prior_schedule);
    }
    assert(simulate_robot_program(published).final_state.q == report.final_state.q);
    commands += report.commands;
  }
  std::cout << "Random programs seed=0xC0DE2026: " << count << " accepted, " << 2 * count
            << " transactional rejections, " << commands << " uniquely dispatched commands\n";
}
