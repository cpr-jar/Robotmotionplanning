#include "parser_fuzz_harness.hpp"
#include <cstddef>
#include <cstdint>
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, std::size_t size) {
  motion_test::fuzz_parse({reinterpret_cast<const char *>(data), size});
  return 0;
}
