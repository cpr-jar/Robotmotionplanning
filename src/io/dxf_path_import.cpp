#include "motion/io/dxf_path_import.hpp"
#include "motion/math/rigid_transform.hpp"
#include <charconv>
#include <fstream>
#include <set>
#include <sstream>

namespace motion {
namespace {
struct Group {
  int code;
  std::string value;
  std::size_t line;
};
struct Entity {
  std::string type;
  std::vector<Group> groups;
  std::size_t line;
};
std::string_view trim(std::string_view value) {
  const auto first = value.find_first_not_of(" \t\r");
  if (first == value.npos)
    return {};
  return value.substr(first, value.find_last_not_of(" \t\r") - first + 1);
}
[[noreturn]] void fail(const DxfLocation &where, const std::string &message) {
  throw DxfImportError(where, message);
}
int integer(std::string_view value, const DxfLocation &where) {
  value = trim(value);
  if (!value.empty() && value.front() == '+')
    value.remove_prefix(1);
  if (value.empty())
    fail(where, "Expected integer DXF group value");
  int number{};
  const auto result = std::from_chars(value.data(), value.data() + value.size(), number);
  if (value.empty() || result.ec != std::errc{} || result.ptr != value.data() + value.size())
    fail(where, "Expected integer DXF group value");
  return number;
}
double number(std::string_view value, const DxfLocation &where) {
  value = trim(value);
  if (!value.empty() && value.front() == '+')
    value.remove_prefix(1);
  if (value.empty())
    fail(where, "Expected finite DXF number");
  double result{};
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
  if (value.empty() || parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
      !std::isfinite(result))
    fail(where, "Expected finite DXF number");
  return result;
}
const Group *single(const Entity &entity, int code, const DxfLocation &where) {
  const Group *found = nullptr;
  for (const auto &group : entity.groups)
    if (group.code == code) {
      if (found)
        fail(where, "Duplicate DXF group " + std::to_string(code));
      found = &group;
    }
  return found;
}
double scalar(const Entity &entity, int code, const DxfLocation &where, std::optional<double> fallback = {}) {
  if (const auto *group = single(entity, code, where)) {
    auto location = where;
    location.line = group->line;
    return number(group->value, location);
  }
  if (fallback)
    return *fallback;
  fail(where, "Missing DXF group " + std::to_string(code));
}
int flag(const Entity &entity, int code, const DxfLocation &where, int fallback = 0) {
  if (const auto *group = single(entity, code, where))
    return integer(group->value, where);
  return fallback;
}
PathVector3 point(const Entity &entity, int code, const DxfLocation &where) {
  return {scalar(entity, code, where), scalar(entity, code + 10, where, 0),
          scalar(entity, code + 20, where, 0)};
}
void extrusion(const Entity &entity, const DxfLocation &where) {
  const PathVector3 normal{scalar(entity, 210, where, 0), scalar(entity, 220, where, 0),
                           scalar(entity, 230, where, 1)};
  if (normal != PathVector3{0, 0, 1})
    fail(where, "Only +Z DXF extrusion is supported; place the drawing with the DXF command transform");
}
double unit_scale(DxfUnit units, std::optional<int> header, const DxfLocation &where) {
  if (units == DxfUnit::Header) {
    if (!header || *header == 0)
      fail(where, "DXF has no defined $INSUNITS; set units explicitly (mm, cm, m, inch or ft)");
    switch (*header) {
    case 1:
      units = DxfUnit::Inch;
      break;
    case 2:
      units = DxfUnit::Foot;
      break;
    case 4:
      units = DxfUnit::Millimetre;
      break;
    case 5:
      units = DxfUnit::Centimetre;
      break;
    case 6:
      units = DxfUnit::Metre;
      break;
    default:
      fail(where, "Unsupported $INSUNITS; set units explicitly");
    }
  }
  switch (units) {
  case DxfUnit::Millimetre:
    return .001;
  case DxfUnit::Centimetre:
    return .01;
  case DxfUnit::Metre:
    return 1;
  case DxfUnit::Inch:
    return .0254;
  case DxfUnit::Foot:
    return .3048;
  default:
    fail(where, "Invalid DXF unit selection");
  }
}
std::vector<Group> groups(std::string_view input, const std::string &filename) {
  const DxfLocation root{filename, "", "", 0, 0};
  if (input.empty() || input.size() > dxf_max_bytes || input.find('\0') != input.npos ||
      input.starts_with("AutoCAD Binary DXF"))
    fail(root, "Expected nonempty text DXF, maximum 4 MiB; binary DXF is unsupported");
  if (input.starts_with("\xef\xbb\xbf"))
    input.remove_prefix(3);
  std::vector<Group> result;
  std::size_t line = 0;
  const auto next_line = [&]() mutable {
    ++line;
    const auto end = input.find('\n');
    const auto value = input.substr(0, end);
    input = end == input.npos ? std::string_view{} : input.substr(end + 1);
    if (value.size() > 4096)
      fail({filename, "", "", line, 0}, "DXF line length budget exceeded");
    return std::string(trim(value));
  };
  // A terminal newline is consumed with its value; blank group lines are invalid.
  auto read_line = next_line;
  while (!input.empty()) {
    if (input.find_first_not_of(" \t\r\n") == input.npos)
      break;
    const auto code_line = line + 1;
    const auto code = integer(read_line(), {filename, "", "", code_line, 0});
    if (input.empty())
      fail({filename, "", "", code_line, 0}, "Truncated DXF code/value pair");
    const auto value = read_line();
    if (code < 0 || code > 1071)
      fail({filename, "", "", code_line, 0}, "Invalid DXF group code");
    if (result.size() >= 250000)
      fail(root, "DXF group budget exceeded");
    result.push_back({code, value, code_line});
  }
  return result;
}
} // namespace

DxfImportError::DxfImportError(DxfLocation location, const std::string &message)
    : std::runtime_error(location.file + ":" + std::to_string(location.line) + " [DXF " +
                         location.entity_type + " #" + std::to_string(location.entity) + " layer '" +
                         location.layer + "']: " + message),
      location_(std::move(location)) {}

DxfDrawing parse_dxf_path(std::string_view input, const DxfImportOptions &options,
                          const std::string &filename) {
  const DxfLocation root{filename, "", "", 0, 0};
  if (options.layer && (options.layer->empty() || options.layer->size() > 4096))
    fail(root, "Invalid DXF layer selection");
  const auto data = groups(input, filename);
  std::optional<int> header;
  std::vector<Entity> entities;
  bool ended = false, have_entities = false, have_header = false;
  for (std::size_t i = 0; i < data.size();) {
    if (data[i].code == 999) {
      ++i;
      continue;
    }
    if (data[i].code == 0 && data[i].value == "EOF") {
      ended = true;
      ++i;
      for (; i < data.size(); ++i)
        if (data[i].code != 999)
          fail(root, "Data after DXF EOF");
      break;
    }
    if (data[i].code != 0 || data[i].value != "SECTION" || i + 1 >= data.size() || data[i + 1].code != 2)
      fail({filename, "", "", data[i].line, 0}, "Expected DXF SECTION or EOF");
    const auto name = data[i + 1].value;
    const auto begin = i + 2;
    i = begin;
    while (i < data.size() && !(data[i].code == 0 && data[i].value == "ENDSEC")) {
      if (data[i].code == 0 && (data[i].value == "EOF" || data[i].value == "SECTION"))
        fail(root, "Missing DXF ENDSEC");
      ++i;
    }
    if (i == data.size())
      fail(root, "Missing DXF ENDSEC");
    if (name == "HEADER") {
      if (have_header)
        fail(root, "Duplicate DXF HEADER section");
      have_header = true;
      for (auto j = begin; j < i; ++j)
        if (data[j].code == 9 && data[j].value == "$INSUNITS") {
          if (header || j + 1 >= i || data[j + 1].code != 70)
            fail(root, "Invalid or duplicate $INSUNITS");
          header = integer(data[j + 1].value, {filename, "", "", data[j + 1].line, 0});
        }
    } else if (name == "ENTITIES") {
      if (have_entities)
        fail(root, "Duplicate DXF ENTITIES section");
      have_entities = true;
      for (auto j = begin; j < i;) {
        if (data[j].code == 999) {
          ++j;
          continue;
        }
        if (data[j].code != 0)
          fail({filename, "", "", data[j].line, 0}, "Expected DXF entity");
        Entity entity{data[j].value, {}, data[j].line};
        ++j;
        while (j < i && data[j].code != 0)
          entity.groups.push_back(data[j++]);
        if (entities.size() >= 20000)
          fail(root, "DXF entity record budget exceeded");
        entities.push_back(std::move(entity));
      }
    }
    ++i;
  }
  if (!ended || !have_entities)
    fail(root, "DXF requires a complete ENTITIES section and EOF");
  DxfDrawing drawing;
  drawing.metres_per_unit = unit_scale(options.units, header, root);
  std::size_t entity_index = 0, point_count = 0;
  const auto scaled_point = [&](PathVector3 value, const DxfLocation &where) {
    for (auto &coordinate : value) {
      coordinate *= drawing.metres_per_unit;
      if (!std::isfinite(coordinate) || std::abs(coordinate) > 1e6)
        fail(where, "DXF coordinate budget exceeded (1e6 m)");
    }
    return value;
  };
  const auto append = [&](DxfStroke &stroke, DxfCurve curve) {
    if (++drawing.curve_count > dxf_max_segments)
      fail(curve.location, "DXF segment budget exceeded");
    stroke.curves.push_back(std::move(curve));
  };
  for (std::size_t index = 0; index < entities.size(); ++index) {
    const auto &entity = entities[index];
    ++entity_index;
    DxfLocation where{filename, "0", entity.type, entity.line, entity_index};
    if (const auto *layer = single(entity, 8, where))
      where.layer = layer->value;
    const bool selected = !options.layer || *options.layer == where.layer;
    std::vector<Entity> vertices;
    if (entity.type == "POLYLINE") {
      while (index + 1 < entities.size() && entities[index + 1].type == "VERTEX")
        vertices.push_back(entities[++index]);
      if (index + 1 >= entities.size() || entities[index + 1].type != "SEQEND")
        fail(where, "POLYLINE is missing SEQEND");
      ++index;
    }
    if (!selected)
      continue;
    if (flag(entity, 67, where) != 0)
      fail(where, "Paper-space entities are unsupported; export the model-space geometry layer");
    if (++point_count > 5000)
      fail(where, "DXF selected entity budget exceeded");
    extrusion(entity, where);
    DxfStroke stroke;
    stroke.location = where;
    if (entity.type == "POINT") {
      stroke.start = scaled_point(point(entity, 10, where), where);
    } else if (entity.type == "LINE") {
      const auto start = scaled_point(point(entity, 10, where), where),
                 end = scaled_point(point(entity, 11, where), where);
      if (kinematic_detail::norm(kinematic_detail::sub(end, start)) <= 1e-12)
        fail(where, "Zero-length DXF LINE");
      stroke.start = start;
      append(stroke, {DxfCurveType::Line, start, end, {}, 0, where});
    } else if (entity.type == "ARC" || entity.type == "CIRCLE") {
      const auto center = scaled_point(point(entity, 10, where), where);
      const auto radius = scalar(entity, 40, where) * drawing.metres_per_unit;
      if (!std::isfinite(radius) || radius <= 1e-10 || radius > 1e6)
        fail(where, "Invalid DXF radius");
      const auto start_angle = entity.type == "CIRCLE" ? 0 : scalar(entity, 50, where);
      const auto end_angle = entity.type == "CIRCLE" ? 360 : scalar(entity, 51, where);
      if (std::abs(start_angle) > 360 || std::abs(end_angle) > 360)
        fail(where, "DXF ARC angles must be within -360..360 degrees");
      double sweep = entity.type == "CIRCLE" ? 360 : std::fmod(end_angle - start_angle + 720, 360);
      if (sweep <= 1e-12)
        fail(where, "Zero-sweep DXF ARC; use CIRCLE for a full circle");
      const auto angle = start_angle * kinematic_detail::pi / 180;
      const auto radians = sweep * kinematic_detail::pi / 180;
      const PathVector3 start{center[0] + radius * std::cos(angle), center[1] + radius * std::sin(angle),
                              center[2]},
          end{center[0] + radius * std::cos(angle + radians), center[1] + radius * std::sin(angle + radians),
              center[2]};
      stroke.start = start;
      append(stroke, {DxfCurveType::Arc, start, end, center, radians, where});
    } else if (entity.type == "LWPOLYLINE" || entity.type == "POLYLINE") {
      struct Vertex {
        PathVector3 position;
        double bulge{};
        DxfLocation location;
        bool have_y{}, have_bulge{};
      };
      std::vector<Vertex> points;
      const int flags = flag(entity, 70, where);
      if (flags < 0 || (flags & ~(1 | 8 | 128)) || (entity.type == "LWPOLYLINE" && (flags & 8)))
        fail(where, "Unsupported fitted/mesh DXF POLYLINE flags");
      if (entity.type == "LWPOLYLINE") {
        const auto elevation = scalar(entity, 38, where, 0);
        for (const auto &group : entity.groups) {
          auto location = where;
          location.line = group.line;
          if (group.code == 30)
            fail(location, "LWPOLYLINE uses elevation group 38, not per-vertex Z");
          if (group.code == 10) {
            if (points.size() >= dxf_max_segments + 1)
              fail(where, "DXF vertex budget exceeded");
            points.push_back({{number(group.value, location), 0, elevation}, 0, location});
          } else if (group.code == 20 || group.code == 42) {
            if (points.empty())
              fail(location, "DXF vertex component before X");
            auto &vertex = points.back();
            if (group.code == 20) {
              if (vertex.have_y)
                fail(location, "Duplicate DXF vertex Y");
              vertex.position[1] = number(group.value, location);
              vertex.have_y = true;
            } else {
              if (vertex.have_bulge)
                fail(location, "Duplicate DXF bulge");
              vertex.bulge = number(group.value, location);
              vertex.have_bulge = true;
            }
          }
        }
        if (flag(entity, 90, where, -1) != int(points.size()))
          fail(where, "LWPOLYLINE vertex count mismatch");
      } else {
        const auto elevation = scalar(entity, 30, where, 0);
        for (const auto &vertex : vertices) {
          auto location = where;
          location.line = vertex.line;
          const auto vertex_flags = flag(vertex, 70, location);
          if (vertex_flags < 0 || (vertex_flags & ~32) || (!(flags & 8) && vertex_flags != 0))
            fail(location, "Unsupported fitted/mesh POLYLINE vertex flags");
          auto position = point(vertex, 10, location);
          if (!(flags & 8)) {
            if (position[2] != 0)
              fail(location,
                   "2D POLYLINE vertices require zero local Z; use header elevation or 3D POLYLINE");
            position[2] = elevation;
          }
          const auto bulge = scalar(vertex, 42, location, 0);
          if ((flags & 8) && bulge != 0)
            fail(location, "3D POLYLINE bulges are unsupported");
          points.push_back({position, bulge, location});
        }
      }
      if (points.size() < 2)
        fail(where, "POLYLINE requires at least two vertices");
      for (auto &vertex : points)
        vertex.position = scaled_point(vertex.position, vertex.location);
      stroke.start = points.front().position;
      const auto edges = (flags & 1) ? points.size() : points.size() - 1;
      for (std::size_t j = 0; j < edges; ++j) {
        const auto &first = points[j], &last = points[(j + 1) % points.size()];
        const auto delta = kinematic_detail::sub(last.position, first.position);
        const auto length = kinematic_detail::norm(delta);
        if (length <= 1e-12)
          fail(first.location, "Repeated/zero-length POLYLINE edge");
        DxfCurve curve{DxfCurveType::Line, first.position, last.position, {}, 0, first.location};
        if (first.bulge != 0) {
          if (std::abs(delta[2]) > 1e-12)
            fail(first.location, "Nonplanar bulged POLYLINE");
          const auto offset = (1 - first.bulge * first.bulge) / (4 * first.bulge);
          curve.type = DxfCurveType::Arc;
          curve.center = {(first.position[0] + last.position[0]) / 2 - delta[1] * offset,
                          (first.position[1] + last.position[1]) / 2 + delta[0] * offset, first.position[2]};
          curve.sweep_rad = 4 * std::atan(first.bulge);
          if (!kinematic_detail::finite_pose({curve.center, {1, 0, 0, 0}}) ||
              kinematic_detail::norm(kinematic_detail::sub(first.position, curve.center)) > 1e6 ||
              std::abs(curve.sweep_rad) >= 2 * kinematic_detail::pi)
            fail(first.location, "Invalid or numerically degenerate DXF bulge");
        }
        append(stroke, std::move(curve));
      }
    } else
      fail(where, "Unsupported DXF entity '" + entity.type +
                      "'; select a geometry layer or convert it to supported primitives");
    drawing.strokes.push_back(std::move(stroke));
  }
  if (drawing.strokes.empty())
    fail(root, "No supported entities in selected DXF layer");
  return drawing;
}

std::string read_dxf_text_file(const std::string &filename) {
  std::ifstream file(filename, std::ios::binary | std::ios::ate);
  if (!file)
    fail({filename, "", "", 0, 0}, "Cannot open DXF file");
  const auto size = file.tellg();
  if (size <= 0 || size > std::streamoff(dxf_max_bytes))
    fail({filename, "", "", 0, 0}, "DXF file is empty or exceeds 4 MiB");
  std::string text(std::size_t(size), '\0');
  file.seekg(0);
  if (!file.read(text.data(), std::streamsize(text.size())))
    fail({filename, "", "", 0, 0}, "Cannot read DXF file");
  return text;
}
DxfDrawing load_dxf_path_file(const std::string &filename, const DxfImportOptions &options) {
  return parse_dxf_path(read_dxf_text_file(filename), options, filename);
}
} // namespace motion
