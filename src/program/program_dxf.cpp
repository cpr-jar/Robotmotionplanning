#include "program_dxf.hpp"

namespace motion::program_detail {
std::vector<ProgramInstruction> expand_dxf(const ProgramInstruction &source, const DxfDrawing &drawing,
                                           const Pose &first) {
  if (!source.dxf)
    throw std::invalid_argument("DXF command is missing configuration");
  const auto &command = *source.dxf;
  const auto frame =
      command.relative_to_tcp ? tracking_detail::compose(first, command.placement) : command.placement;
  const auto orientation = source.orientation.value_or(first.orientation);
  const auto transform = [&](PathVector3 point) {
    return kinematic_detail::add(frame.position, tracking_detail::rotate(frame.orientation, point));
  };
  std::vector<ProgramInstruction> steps;
  Pose expected = first;
  const auto append = [&](ProgramInstruction step) {
    if (steps.size() >= 10000)
      throw std::invalid_argument("Expanded DXF instruction budget exceeded");
    steps.push_back(std::move(step));
  };
  for (const auto &stroke : drawing.strokes) {
    const Pose start{transform(stroke.start), orientation};
    double dot = 0;
    for (std::size_t i = 0; i < 4; ++i)
      dot += expected.orientation[i] * orientation[i];
    if (kinematic_detail::norm(kinematic_detail::sub(start.position, expected.position)) > 1e-7 ||
        std::abs(std::abs(dot) - 1) > 1e-10) {
      if (command.travel == DxfTravel::Reject)
        throw DxfImportError(stroke.location, "DXF entry/disconnected stroke requires an explicit travel: "
                                              "LIN or PTP (or a matching start pose)");
      ProgramInstruction travel;
      travel.operation = command.travel == DxfTravel::Line ? ProgramOperation::Line : ProgramOperation::Ptp;
      travel.target = start;
      travel.speed = source.speed;
      travel.location = source.location;
      travel.drawing_location = stroke.location;
      travel.path_tolerance_m = source.path_tolerance_m;
      append(std::move(travel));
    }
    expected = start;
    for (const auto &curve : stroke.curves) {
      ProgramInstruction step;
      step.operation = curve.type == DxfCurveType::Line ? ProgramOperation::Line : ProgramOperation::Circle;
      step.target = Pose{transform(curve.end), orientation};
      step.drawing_start = transform(curve.start);
      step.speed = source.speed;
      step.location = source.location;
      step.drawing_location = curve.location;
      step.path_tolerance_m = source.path_tolerance_m;
      if (curve.type == DxfCurveType::Arc) {
        // A mapped IK endpoint may be ~1e-7 m off the nominal CAD point.
        // Use the exact CAD radius vector from that actual endpoint, retaining
        // the arc plane/radius without weakening CartesianArcPath's invariants.
        step.relative_center = true;
        step.center =
            tracking_detail::rotate(frame.orientation, kinematic_detail::sub(curve.center, curve.start));
        step.normal = tracking_detail::rotate(frame.orientation, {0, 0, 1});
        step.sweep_rad = curve.sweep_rad;
        step.orientation = orientation;
      }
      append(std::move(step));
      expected = {transform(curve.end), orientation};
    }
  }
  if (steps.empty())
    throw std::invalid_argument("DXF command produces no motion; all waypoints already match the TCP");
  return steps;
}
} // namespace motion::program_detail
