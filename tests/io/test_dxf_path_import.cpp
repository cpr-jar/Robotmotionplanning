#include "motion/io/dxf_path_import.hpp"
#include "motion/math/rigid_transform.hpp"
#include <cassert>
#include <iostream>
#include <random>
using namespace motion;
namespace {
std::string document(const std::string &body, int units = 4) {
  return "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n" + std::to_string(units) +
         "\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n" + body + "0\nENDSEC\n0\nEOF\n";
}
template <class F> void rejected(F run, const std::string &part) {
  bool caught = false;
  try {
    run();
  } catch (const DxfImportError &error) {
    caught = true;
    if (std::string(error.what()).find(part) == std::string::npos) {
      std::cerr << error.what() << " expected: " << part << '\n';
      std::abort();
    }
  }
  assert(caught);
}
void near(double a, double b, double tolerance = 1e-12) {
  assert(std::abs(a - b) <= tolerance);
}
const std::string line = "0\nLINE\n8\nPATH\n10\n-2\n20\n3\n30\n4\n11\n8\n21\n9\n31\n5\n";
void primitives_and_units() {
  const auto drawing = parse_dxf_path(document(line + "0\nPOINT\n10\n2\n20\n1\n30\n3\n"));
  assert(drawing.strokes.size() == 2 && drawing.curve_count == 1 && drawing.strokes[1].curves.empty());
  near(drawing.strokes[0].start[0], -.002);
  near(drawing.strokes[0].curves[0].end[2], .005);
  near(drawing.strokes[1].start[2], .003);
  near(parse_dxf_path(document(line, 1)).strokes[0].start[0], -2 * .0254);
  near(parse_dxf_path(document(line, 2)).strokes[0].start[0], -2 * .3048);
  for (const auto &[unit, scale] : std::vector<std::pair<DxfUnit, double>>{{DxfUnit::Millimetre, .001},
                                                                           {DxfUnit::Centimetre, .01},
                                                                           {DxfUnit::Metre, 1},
                                                                           {DxfUnit::Inch, .0254},
                                                                           {DxfUnit::Foot, .3048}})
    near(parse_dxf_path(document(line, 0), {unit, {}}).strokes[0].start[0], -2 * scale);
  rejected([&] { parse_dxf_path(document(line, 0)); }, "units explicitly");
  const auto curves = parse_dxf_path(
      document("0\nARC\n10\n0\n20\n0\n40\n10\n50\n350\n51\n10\n0\nCIRCLE\n10\n2\n20\n3\n30\n4\n40\n5\n"));
  near(curves.strokes[0].curves[0].sweep_rad, 20 * kinematic_detail::pi / 180);
  near(curves.strokes[0].start[1], .01 * std::sin(350 * kinematic_detail::pi / 180));
  near(curves.strokes[1].curves[0].sweep_rad, 2 * kinematic_detail::pi);
  near(curves.strokes[1].start[0], .007);
}
std::string polyline(double bulge, bool closed = false) {
  return "0\nLWPOLYLINE\n90\n2\n70\n" + std::to_string(closed ? 1 : 0) + "\n10\n0\n20\n0\n42\n" +
         std::to_string(bulge) + "\n10\n10\n20\n0\n";
}
void polylines() {
  for (double bulge : {-2., -1., -.414214, .414214, 1., 2.}) {
    const auto drawing = parse_dxf_path(document(polyline(bulge)));
    const auto &curve = drawing.strokes[0].curves[0];
    assert(curve.type == DxfCurveType::Arc);
    near(curve.sweep_rad, 4 * std::atan(bulge), 1e-6);
    const auto x = curve.start[0] - curve.center[0], y = curve.start[1] - curve.center[1];
    near(curve.center[0] + x * std::cos(curve.sweep_rad) - y * std::sin(curve.sweep_rad), curve.end[0]);
    near(curve.center[1] + x * std::sin(curve.sweep_rad) + y * std::cos(curve.sweep_rad), curve.end[1]);
  }
  const auto closed = parse_dxf_path(document(polyline(0, true)));
  assert(closed.curve_count == 2 && closed.strokes[0].curves.back().end == closed.strokes[0].start);
  const auto old = parse_dxf_path(
      document("0\nPOLYLINE\n70\n0\n30\n2\n0\nVERTEX\n10\n0\n20\n0\n0\nVERTEX\n10\n10\n20\n0\n0\nSEQEND\n"));
  near(old.strokes[0].start[2], .002);
  assert(old.curve_count == 1);
  const auto spatial = parse_dxf_path(document("0\nPOLYLINE\n70\n8\n0\nVERTEX\n70\n32\n10\n0\n20\n0\n30\n1\n0"
                                               "\nVERTEX\n70\n32\n10\n10\n20\n0\n30\n5\n0\nSEQEND\n"));
  near(spatial.strokes[0].curves[0].end[2], .005);
}
void errors_and_layers() {
  const auto mixed = document(line + "0\nSPLINE\n8\nLABELS\n");
  rejected([&] { parse_dxf_path(mixed); }, "SPLINE");
  assert(parse_dxf_path(mixed, {DxfUnit::Header, std::string("PATH")}).curve_count == 1);
  rejected([&] { parse_dxf_path(mixed, {DxfUnit::Header, std::string("missing")}); }, "No supported");
  for (const auto &[body, error] : std::vector<std::pair<std::string, std::string>>{
           {line + "0\nINSERT\n2\nBLOCK\n", "Unsupported"},
           {line + "10\n2\n", "Duplicate"},
           {"0\nLINE\n10\n.nan\n11\n2\n", "finite"},
           {"0\nLINE\n10\n1e999\n11\n2\n", "finite"},
           {"0\nLINE\n10\n0\n11\n0\n", "Zero-length"},
           {line + "210\n1\n220\n0\n230\n0\n", "extrusion"},
           {line + "67\n1\n", "Paper-space"},
           {"0\nARC\n10\n0\n40\n1\n50\n20\n51\n20\n", "Zero-sweep"},
           {"0\nCIRCLE\n10\n0\n40\n-1\n", "radius"},
           {"0\nLWPOLYLINE\n90\n3\n10\n0\n10\n1\n", "count"},
           {"0\nPOLYLINE\n0\nVERTEX\n10\n0\n", "SEQEND"},
           {"0\nPOLYLINE\n70\n64\n0\nSEQEND\n", "flags"}})
    rejected([&] { parse_dxf_path(document(body)); }, error);
  rejected([&] { parse_dxf_path("AutoCAD Binary DXF"); }, "binary");
  rejected([&] { parse_dxf_path(std::string(dxf_max_bytes + 1, 'x')); }, "maximum");
  rejected([&] { parse_dxf_path(document(line) + "0\nLINE\n"); }, "after");
  rejected([&] { parse_dxf_path(document(line).substr(0, document(line).size() - 6)); }, "EOF");
  rejected([&] { parse_dxf_path("0\nSECTION\n2\nENTITIES\n0\nEOF\n"); }, "ENDSEC");
  const auto parsed = parse_dxf_path("\xef\xbb\xbf" + document(line) + " \n\r\n");
  assert(parsed.curve_count == 1);
}
void mutation_checks() {
  std::mt19937 rng(0xDFF2026);
  const auto seed = document(line + polyline(.414214));
  std::size_t accepted = 0, rejected_count = 0;
  for (int attempt = 0; attempt < 4000; ++attempt) {
    auto text = seed;
    for (unsigned mutation = 0; mutation < 1 + rng() % 5; ++mutation)
      text[rng() % text.size()] = char(rng() % 128);
    if (attempt % 5 == 0)
      text.resize(rng() % text.size());
    try {
      const auto drawing = parse_dxf_path(text);
      ++accepted;
      assert(!drawing.strokes.empty() && drawing.curve_count <= dxf_max_segments);
      for (const auto &stroke : drawing.strokes) {
        for (double x : stroke.start)
          assert(std::isfinite(x));
        for (const auto &curve : stroke.curves) {
          for (double x : curve.end)
            assert(std::isfinite(x));
          for (double x : curve.center)
            assert(std::isfinite(x));
          if (curve.type == DxfCurveType::Arc)
            assert(std::isfinite(curve.sweep_rad) && curve.sweep_rad != 0 &&
                   std::abs(curve.sweep_rad) <= 2 * kinematic_detail::pi);
        }
      }
    } catch (const DxfImportError &) {
      ++rejected_count;
    }
  }
  assert(accepted > 0 && rejected_count > 0);
  std::cout << "DXF seeded mutations: " << accepted << " accepted, " << rejected_count << " rejected\n";
}
} // namespace
int main() {
  primitives_and_units();
  polylines();
  errors_and_layers();
  mutation_checks();
  const auto reference =
      load_dxf_path_file(std::string(MOTION_SOURCE_DIR) + "/programs/dxf/rounded_path.dxf");
  assert(reference.strokes.size() == 1 && reference.curve_count == 5);
  const auto &arc = reference.strokes[0].curves[1];
  assert(arc.type == DxfCurveType::Arc);
  near(arc.center[0], .025);
  near(arc.center[1], .005);
  near(arc.sweep_rad, kinematic_detail::pi / 2);
}
