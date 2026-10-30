#include "motion/io/dxf_path_import.hpp"
#include <cassert>
int main() {
  const auto drawing =
      motion::parse_dxf_path("0\nSECTION\n2\nENTITIES\n0\nPOINT\n10\n25\n20\n50\n0\nENDSEC\n0\nEOF\n",
                             {motion::DxfUnit::Millimetre, {}});
  assert(drawing.strokes.size() == 1 && drawing.strokes[0].start[0] == .025 &&
         drawing.strokes[0].start[1] == .05);
}
