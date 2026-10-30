#pragma once
#include "motion/core/geometry_types.hpp"
#include "motion/core/preparation_context.hpp"
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace motion {
// Text DXF import is preparation-time work, never part of cyclic execution.
inline constexpr std::size_t dxf_max_bytes = 4 * 1024 * 1024;
inline constexpr std::size_t dxf_max_segments = 10000;
enum class DxfUnit { Header, Millimetre, Centimetre, Metre, Inch, Foot };
struct DxfImportOptions {
  DxfUnit units{DxfUnit::Header};
  std::optional<std::string> layer;
  bool operator==(const DxfImportOptions &) const = default;
};
struct DxfLocation {
  std::string file, layer, entity_type;
  std::size_t line{}, entity{};
};
class DxfImportError : public std::runtime_error {
public:
  DxfImportError(DxfLocation, const std::string &);
  const DxfLocation &location() const noexcept { return location_; }

private:
  DxfLocation location_;
};
enum class DxfCurveType { Line, Arc };
struct DxfCurve {
  DxfCurveType type{DxfCurveType::Line};
  PathVector3 start{}, end{}, center{};
  double sweep_rad{};
  DxfLocation location;
};
struct DxfStroke {
  PathVector3 start{};
  std::vector<DxfCurve> curves; // Empty for POINT: the stroke is a waypoint.
  DxfLocation location;
};
struct DxfDrawing {
  std::vector<DxfStroke> strokes;
  double metres_per_unit{};
  std::size_t curve_count{};
};
DxfDrawing parse_dxf_path(std::string_view, const DxfImportOptions & = {},
                          const std::string &filename = "<memory>");
DxfDrawing parse_dxf_path(std::string_view, const DxfImportOptions &, const std::string &,
                          PreparationContext *);
std::string read_dxf_text_file(const std::string &);
DxfDrawing load_dxf_path_file(const std::string &, const DxfImportOptions & = {});
} // namespace motion
