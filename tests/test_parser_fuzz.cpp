#include "parser_fuzz_harness.hpp"
#include "program_test_fixture.hpp"
#include <cassert>
#include <fstream>
#include <iostream>
#include <random>
int main(int argc, char **argv) {
  if (argc > 1) {
    for (int i = 1; i < argc; ++i) {
      std::ifstream input(argv[i], std::ios::binary);
      if (!input)
        return 2;
      motion_test::fuzz_parse(std::string(std::istreambuf_iterator<char>(input), {}));
    }
    return 0;
  }
  constexpr unsigned seed = 0xF0222026;
  std::mt19937 random(seed);
  const std::vector<std::string> corpus{
      motion_test::prefix + "steps: [{type: WAIT, seconds: 0.01}]\n",
      motion_test::prefix + "points: {a: {joints: [0.2,0,0]}}\nsteps: [{type: PTP, target: a}]\n",
      motion_test::prefix + "steps: [{type: LIN, target: {position: [0.2,0,0], orientation: [1,0,0,0]}}]\n",
      motion_test::prefix +
          "subprograms: {a: [{type: WAIT, seconds: 0.01}]}\nsteps: [{type: CALL, program: a}]\n",
      motion_test::prefix + "steps: [{type: CIRC, center_offset: [-0.01,0,0], normal: [0,0,1], sweep_rad: "
                            "6.283185307179586}]\n"};
  for (const auto &source : corpus)
    assert(motion_test::fuzz_parse(source));
  const std::vector<std::string> tokens{
      ".nan",     ".inf",    "-1e9999", "!tag", "&a", "*a", "---\n", "null",    "[]",    "{}",
      "[",        "]",       "{",       "}",    "\n", ":",  "CALL",  "seconds", "start", std::string(1, '\0'),
      "\xff\xfe", "\xc0\xaf"};
  std::size_t accepted = 0, rejected = 0;
  for (unsigned trial = 0; trial < 12000; ++trial) {
    auto input = corpus[random() % corpus.size()];
    for (unsigned edits = 1 + random() % 8; edits; --edits) {
      const auto at = random() % (input.size() + 1);
      switch (random() % 5) {
      case 0:
        input.insert(at, tokens[random() % tokens.size()]);
        break;
      case 1:
        input.erase(at, random() % 32);
        break;
      case 2:
        input.insert(at, 1, char(random() % 256));
        break;
      case 3:
        if (at < input.size())
          input[at] = char(random() % 256);
        break;
      case 4:
        input.insert(at, input.substr(random() % (input.size() + 1), random() % 64));
        break;
      }
    }
    // Unexpected C++ exceptions retain an exact replay file; libFuzzer also records crashes.
    try {
      if (motion_test::fuzz_parse(input))
        ++accepted;
      else
        ++rejected;
    } catch (const std::exception &e) {
      std::ofstream("parser-fuzz-failure.yaml", std::ios::binary).write(input.data(), input.size());
      std::cerr << "seed=" << seed << " case=" << trial << ": " << e.what() << '\n';
      return 1;
    }
  }
  // Hard resource limits: bytes, nesting, node count, expansion and CALL depth.
  assert(!motion_test::fuzz_parse(std::string(1024 * 1024 + 1, ' ')));
  assert(!motion_test::fuzz_parse(motion_test::prefix + "steps: " + std::string(80, '[') + "0" +
                                  std::string(80, ']')));
  std::string nodes = motion_test::prefix + "steps: [";
  for (int i = 0; i < 51000; ++i)
    nodes += "0,";
  assert(!motion_test::fuzz_parse(nodes + "]"));
  for (int depth : {16, 17, 255}) {
    std::string calls = motion_test::prefix + "subprograms:\n";
    for (int i = 0; i < depth; ++i)
      calls += "  p" + std::to_string(i) + ": [{type: CALL, program: p" + std::to_string(i + 1) + "}]\n";
    calls +=
        "  p" + std::to_string(depth) + ": [{type: WAIT, seconds: 0.01}]\nsteps: [{type: CALL, program: p0}]";
    assert(!motion_test::fuzz_parse(calls));
  }
  assert(accepted && rejected);
  std::cout << "Parser mutations seed=0xF0222026: accepted=" << accepted << ", rejected=" << rejected
            << "; replay: parser_fuzz_tests INPUT.yaml\n";
}
