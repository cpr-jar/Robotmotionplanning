#pragma once
#include "tracking_frames.hpp"
namespace motion {
struct ObjectObservation {
  std::string object_id, frame;
  Pose pose;
  TrackingTime measured_at, received_at;
  double position_error_m{}, orientation_error_rad{};
};
struct EncoderObservation {
  double position_m{}, velocity_m_s{}, acceleration_m_s2{};
  TrackingTime measured_at, received_at;
  double velocity_error_m_s{}, acceleration_error_m_s2{};
};
struct ConveyorModel {
  std::string frame;
  kinematic_detail::V3 axis{1, 0, 0};
  double scale{1}, direction_error_rad{}, scale_error{};
};
struct TrackingPolicy {
  double max_object_age_s{.25}, max_encoder_age_s{.15}, max_transport_latency_s{.1}, max_prediction_s{1},
      max_history_s{1}, max_position_error_m{.01}, max_velocity_error_m_s{.1}, max_acceleration_error_m_s2{1},
      max_clock_uncertainty_s{.001};
};
struct PredictedConveyorTarget {
  Pose pose;
  kinematic_detail::V3 velocity{}, acceleration{};
  double position_error_m{}, orientation_error_rad{}, velocity_error_m_s{}, acceleration_error_m_s2{};
  TrackingTime at;
};
enum class PredictionStatus {
  Ready,
  InvalidInput,
  ClockMismatch,
  StaleObservation,
  LatencyExceeded,
  HorizonExceeded,
  CalibrationUnavailable,
  ExcessUncertainty
};
struct PredictionResult {
  PredictionStatus status{PredictionStatus::InvalidInput};
  std::optional<PredictedConveyorTarget> target;
  std::string diagnostic;
  explicit operator bool() const { return status == PredictionStatus::Ready && bool(target); }
};
class ConveyorTargetPredictor {
public:
  ConveyorTargetPredictor(ConveyorModel model, TrackingPolicy policy = {})
      : model_(std::move(model)), policy_(policy) {
    const double n = kinematic_detail::norm(model_.axis);
    if (model_.frame.empty() || !std::isfinite(n) || n <= 0 || !std::isfinite(model_.scale) ||
        model_.scale == 0 || !std::isfinite(model_.direction_error_rad) || model_.direction_error_rad < 0 ||
        !std::isfinite(model_.scale_error) || model_.scale_error < 0)
      throw std::invalid_argument("Invalid conveyor model");
    model_.axis = kinematic_detail::scale(model_.axis, 1 / n);
    for (double x :
         {policy.max_object_age_s, policy.max_encoder_age_s, policy.max_transport_latency_s,
          policy.max_prediction_s, policy.max_history_s, policy.max_position_error_m,
          policy.max_velocity_error_m_s, policy.max_acceleration_error_m_s2, policy.max_clock_uncertainty_s})
      if (!std::isfinite(x) || x < 0)
        throw std::invalid_argument("Invalid tracking policy");
  }
  PredictionResult predict(const ObjectObservation &o, const EncoderObservation &e,
                           const TrackingFrameSnapshot &frames, TrackingTime now, TrackingTime at) const {
    const auto fail = [](PredictionStatus s, const char *d) { return PredictionResult{s, {}, d}; };
    if (o.object_id.empty() || !kinematic_detail::finite_pose(o.pose) || !std::isfinite(e.position_m) ||
        !std::isfinite(e.velocity_m_s) || !std::isfinite(e.acceleration_m_s2))
      return fail(PredictionStatus::InvalidInput, "Invalid observation");
    for (double x :
         {o.position_error_m, o.orientation_error_rad, e.velocity_error_m_s, e.acceleration_error_m_s2})
      if (!std::isfinite(x) || x < 0)
        return fail(PredictionStatus::InvalidInput, "Invalid observation uncertainty");
    for (auto t : {o.measured_at, o.received_at, e.measured_at, e.received_at, now, at}) {
      if (t.clock != frames.clock())
        return fail(PredictionStatus::ClockMismatch, "Tracking clock domain mismatch");
      if (!std::isfinite(t.seconds) || !std::isfinite(t.uncertainty_s) || t.uncertainty_s < 0 ||
          t.uncertainty_s > policy_.max_clock_uncertainty_s)
        return fail(PredictionStatus::InvalidInput, "Invalid tracking clock uncertainty");
      if (!frames.valid(t))
        return fail(PredictionStatus::CalibrationUnavailable, "Calibration not valid at requested epoch");
    }
    const auto causal = [](TrackingTime a, TrackingTime b) { return a.seconds <= b.seconds; };
    if (!causal(o.measured_at, o.received_at) || !causal(e.measured_at, e.received_at) ||
        !causal(o.received_at, now) || !causal(e.received_at, now) || !causal(now, at))
      return fail(PredictionStatus::InvalidInput, "Noncausal observation/request");
    if (now.seconds - o.measured_at.seconds + now.uncertainty_s + o.measured_at.uncertainty_s >
            policy_.max_object_age_s ||
        now.seconds - e.measured_at.seconds + now.uncertainty_s + e.measured_at.uncertainty_s >
            policy_.max_encoder_age_s)
      return fail(PredictionStatus::StaleObservation, "Stale tracking observations");
    if (o.received_at.seconds - o.measured_at.seconds + o.received_at.uncertainty_s +
                o.measured_at.uncertainty_s >
            policy_.max_transport_latency_s ||
        e.received_at.seconds - e.measured_at.seconds + e.received_at.uncertainty_s +
                e.measured_at.uncertainty_s >
            policy_.max_transport_latency_s)
      return fail(PredictionStatus::LatencyExceeded, "Observation transport latency exceeded");
    if (at.seconds - now.seconds + at.uncertainty_s + now.uncertainty_s > policy_.max_prediction_s ||
        std::abs(o.measured_at.seconds - e.measured_at.seconds) + o.measured_at.uncertainty_s +
                e.measured_at.uncertainty_s >
            policy_.max_history_s)
      return fail(PredictionStatus::HorizonExceeded, "Prediction/history horizon exceeded");
    try {
      const auto object_frame = frames.resolve(o.frame), belt_frame = frames.resolve(model_.frame);
      const auto world = tracking_detail::compose(object_frame.root_from_frame, o.pose);
      const auto direction = tracking_detail::rotate(belt_frame.root_from_frame.orientation, model_.axis);
      const long double df = static_cast<long double>(at.seconds) - e.measured_at.seconds,
                        d0 = static_cast<long double>(o.measured_at.seconds) - e.measured_at.seconds;
      const double displacement = double(model_.scale * (e.velocity_m_s * (df - d0) +
                                                         .5L * e.acceleration_m_s2 * (df * df - d0 * d0))),
                   velocity = model_.scale * double(e.velocity_m_s + e.acceleration_m_s2 * df),
                   acceleration = model_.scale * e.acceleration_m_s2;
      PredictedConveyorTarget target;
      target.pose = {kinematic_detail::add(world.position, kinematic_detail::scale(direction, displacement)),
                     world.orientation};
      target.velocity = kinematic_detail::scale(direction, velocity);
      target.acceleration = kinematic_detail::scale(direction, acceleration);
      const double dt = at.uncertainty_s + o.measured_at.uncertainty_s + 2 * e.measured_at.uncertainty_s;
      const double angle = std::min(kinematic_detail::pi,
                                    belt_frame.orientation_error_rad + model_.direction_error_rad),
                   axis_error = 2 * std::sin(angle / 2);
      const double abs_scale = std::abs(model_.scale);
      target.position_error_m =
          o.position_error_m + object_frame.position_error_m +
          2 * std::sin(std::min(kinematic_detail::pi, object_frame.orientation_error_rad) / 2) *
              kinematic_detail::norm(o.pose.position) +
          abs_scale * (e.velocity_error_m_s * double(std::abs(df - d0)) +
                       .5 * e.acceleration_error_m_s2 * double(std::abs(df * df - d0 * d0))) +
          std::abs(displacement) * (axis_error + model_.scale_error / abs_scale) +
          (std::abs(velocity) + std::abs(acceleration) * double(std::abs(d0))) * dt;
      target.orientation_error_rad =
          std::min(kinematic_detail::pi, o.orientation_error_rad + object_frame.orientation_error_rad);
      target.velocity_error_m_s =
          abs_scale * (e.velocity_error_m_s + e.acceleration_error_m_s2 * double(std::abs(df))) +
          std::abs(acceleration) * dt + std::abs(velocity) * (axis_error + model_.scale_error / abs_scale);
      target.acceleration_error_m_s2 = abs_scale * e.acceleration_error_m_s2 +
                                       std::abs(acceleration) * (axis_error + model_.scale_error / abs_scale);
      target.at = at;
      if (!kinematic_detail::finite_pose(target.pose) || !std::isfinite(target.position_error_m) ||
          !std::isfinite(target.velocity_error_m_s) || !std::isfinite(target.acceleration_error_m_s2) ||
          target.position_error_m > policy_.max_position_error_m ||
          target.velocity_error_m_s > policy_.max_velocity_error_m_s ||
          target.acceleration_error_m_s2 > policy_.max_acceleration_error_m_s2)
        return fail(PredictionStatus::ExcessUncertainty, "Prediction uncertainty exceeded");
      return {PredictionStatus::Ready, target, "Calibrated straight-conveyor prediction"};
    } catch (const std::exception &e) {
      return {PredictionStatus::CalibrationUnavailable, {}, e.what()};
    }
  }

private:
  ConveyorModel model_;
  TrackingPolicy policy_;
};
} // namespace motion
