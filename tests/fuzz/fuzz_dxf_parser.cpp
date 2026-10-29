#include "motion/io/dxf_path_import.hpp"
#include <cstddef>
#include <cstdint>
extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, std::size_t size) {
  try {
    const auto drawing = motion::parse_dxf_path(std::string_view(reinterpret_cast<const char *>(data), size),
                                                {motion::DxfUnit::Millimetre, {}}, "fuzz.dxf");
    if (drawing.strokes.empty() || drawing.curve_count > motion::dxf_max_segments)
      __builtin_trap();
  } catch (const motion::DxfImportError &) {
  }
  return 0;
}
