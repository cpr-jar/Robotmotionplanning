#pragma once
#include "motion/execution/live_motion_executor.hpp"
#include "motion/execution/stop_schedules.hpp"
#include "motion/planning/pose_path.hpp"
#include "motion/tracking/conveyor_target_predictor.hpp"
#include <deque>
#if defined(MOTION_WITH_SERIAL) || defined(MOTION_WITH_DIFFERENTIAL)
#include "motion/kinematics/inverse_differential.hpp"
#include "motion/planning/envelope_constraints.hpp"
#endif
namespace motion {
struct ObservationHistoryPolicy {
  std::size_t capacity{64};
  double object_innovation_m{.003}, encoder_innovation_m{.005}, max_encoder_jerk_m_s3{.1},
      velocity_innovation_m_s{.005}, orientation_innovation_rad{.02};
};
enum class ObservationUpdateStatus {
  Accepted,
  Invalid,
  AssociationMismatch,
  CalibrationChanged,
  OutOfOrder,
  Outlier
};
struct ObservationUpdateResult {
  ObservationUpdateStatus status{};
  std::string diagnostic;
  explicit operator bool() const { return status == ObservationUpdateStatus::Accepted; }
};
class ConveyorObservationHistory {
public:
  ConveyorObservationHistory(std::string object, std::uint64_t revision, ConveyorModel model,
                             TrackingPolicy prediction = {}, ObservationHistoryPolicy history = {})
      : object_(std::move(object)), revision_(revision), predictor_(std::move(model), prediction),
        prediction_(prediction), policy_(history) {
    if (object_.empty() || !revision || !history.capacity || history.capacity > 4096)
      throw std::invalid_argument("Invalid observation history identity/capacity");
    for (double x : {history.object_innovation_m, history.encoder_innovation_m, history.max_encoder_jerk_m_s3,
                     history.velocity_innovation_m_s, history.orientation_innovation_rad})
      if (!std::isfinite(x) || x < 0)
        throw std::invalid_argument("Invalid history gates");
  }
  void reset(std::string object, std::uint64_t revision) {
    if (object.empty() || !revision)
      throw std::invalid_argument("Invalid history reset");
    object_ = std::move(object);
    revision_ = revision;
    history_.clear();
  }
  std::size_t size() const { return history_.size(); }
  ObservationUpdateResult ingest(const ObjectObservation &o, const EncoderObservation &e,
                                 const TrackingFrameSnapshot &frames, ClockTime now) {
    if (o.object_id != object_)
      return {ObservationUpdateStatus::AssociationMismatch, "Different object association"};
    if (frames.revision() != revision_)
      return {ObservationUpdateStatus::CalibrationChanged, "Calibration reset required"};
    auto fresh = predictor_.predict(o, e, frames, now, now);
    if (!fresh)
      return {ObservationUpdateStatus::Invalid, fresh.diagnostic};
    if (!history_.empty()) {
      const auto &old = history_.back();
      if (o.measured_at.seconds <= old.object.measured_at.seconds ||
          e.measured_at.seconds <= old.encoder.measured_at.seconds ||
          o.received_at.seconds < old.object.received_at.seconds ||
          e.received_at.seconds < old.encoder.received_at.seconds)
        return {ObservationUpdateStatus::OutOfOrder, "Duplicate/out-of-order observations"};
      const double dt = e.measured_at.seconds - old.encoder.measured_at.seconds;
      if (dt <= prediction_.max_history_s) {
        const double predicted = old.encoder.position_m + old.encoder.velocity_m_s * dt +
                                 .5 * old.encoder.acceleration_m_s2 * dt * dt;
        const double allowance = policy_.encoder_innovation_m + old.encoder.velocity_error_m_s * dt +
                                 .5 * old.encoder.acceleration_error_m_s2 * dt * dt +
                                 policy_.max_encoder_jerk_m_s3 * dt * dt * dt / 6;
        if (std::abs(e.position_m - predicted) > allowance ||
            std::abs(e.acceleration_m_s2 - old.encoder.acceleration_m_s2) >
                policy_.max_encoder_jerk_m_s3 * dt + e.acceleration_error_m_s2 +
                    old.encoder.acceleration_error_m_s2)
          return {ObservationUpdateStatus::Outlier, "Encoder reset/acceleration outlier"};
        if (std::abs(e.velocity_m_s - old.encoder.velocity_m_s - old.encoder.acceleration_m_s2 * dt) >
            policy_.velocity_innovation_m_s + old.encoder.velocity_error_m_s + e.velocity_error_m_s +
                old.encoder.acceleration_error_m_s2 * dt + .5 * policy_.max_encoder_jerk_m_s3 * dt * dt)
          return {ObservationUpdateStatus::Outlier, "Encoder velocity outlier"};
        const auto past = predictor_.predict(old.object, old.encoder, frames, now, now);
        if (past) {
          const double age = now.seconds - old.encoder.measured_at.seconds;
          const double residual = kinematic_detail::norm(
              kinematic_detail::sub(past.target->pose.position, fresh.target->pose.position));
          if (residual >
              policy_.object_innovation_m + past.target->position_error_m + fresh.target->position_error_m +
                  policy_.max_encoder_jerk_m_s3 * predictor_.absolute_scale() * age * age * age / 6)
            return {ObservationUpdateStatus::Outlier, "Object position innovation"};
          if (rotation_distance(past.target->pose.orientation, fresh.target->pose.orientation) >
              policy_.orientation_innovation_rad + past.target->orientation_error_rad +
                  fresh.target->orientation_error_rad)
            return {ObservationUpdateStatus::Outlier, "Object orientation innovation"};
        }
      }
    }
    history_.push_back({o, e});
    while (history_.size() > policy_.capacity ||
           (history_.size() > 1 &&
            o.measured_at.seconds - history_.front().object.measured_at.seconds > prediction_.max_history_s))
      history_.pop_front();
    return {ObservationUpdateStatus::Accepted, "History update admitted"};
  }
  PredictionResult predict(const TrackingFrameSnapshot &frames, ClockTime now, ClockTime at) const {
    if (history_.empty())
      return {PredictionStatus::InvalidInput, {}, "No accepted observations"};
    if (frames.revision() != revision_)
      return {PredictionStatus::CalibrationUnavailable, {}, "Calibration reset required"};
    const auto &last = history_.back();
    auto r = predictor_.predict(last.object, last.encoder, frames, now, at);
    if (!r)
      return r;
    const long double df = std::abs(static_cast<long double>(at.seconds) - last.encoder.measured_at.seconds) +
                           at.uncertainty_s + last.encoder.measured_at.uncertainty_s,
                      d0 = std::abs(static_cast<long double>(last.object.measured_at.seconds) -
                                    last.encoder.measured_at.seconds) +
                           last.object.measured_at.uncertainty_s + last.encoder.measured_at.uncertainty_s,
                      j = policy_.max_encoder_jerk_m_s3 * predictor_.absolute_scale();
    r.target->position_error_m += double(j * (df * df * df + d0 * d0 * d0) / 6);
    r.target->velocity_error_m_s += double(j * df * df / 2);
    r.target->acceleration_error_m_s2 += double(j * df);
    if (!std::isfinite(r.target->position_error_m) ||
        r.target->position_error_m > prediction_.max_position_error_m ||
        r.target->velocity_error_m_s > prediction_.max_velocity_error_m_s ||
        r.target->acceleration_error_m_s2 > prediction_.max_acceleration_error_m_s2)
      return {PredictionStatus::ExcessUncertainty, {}, "Changing-acceleration uncertainty exceeded"};
    return r;
  }

private:
  struct Entry {
    ObjectObservation object;
    EncoderObservation encoder;
  };
  std::string object_;
  std::uint64_t revision_;
  ConveyorTargetPredictor predictor_;
  TrackingPolicy prediction_;
  ObservationHistoryPolicy policy_;
  std::deque<Entry> history_;
};
template <std::size_t N> struct KinematicFollowingResult {
  std::shared_ptr<const PreparedLivePlan<N>> plan;
  PredictionResult prediction;
  std::uint64_t tracking_end_tick{};
  std::string diagnostic;
  explicit operator bool() const { return bool(plan); }
};
using FollowingPlanResult = KinematicFollowingResult<3>;
class GantryConveyorFollower {
public:
  GantryConveyorFollower(JointLimits limits, CartesianLimits tcp, std::string frame = "world")
      : limits_(std::move(limits)), frame_(std::move(frame)) {
    check_limits(limits_, 3);
    if (frame_.empty())
      throw std::invalid_argument("Missing gantry frame");
    for (double x : {tcp.max_linear_velocity, tcp.max_linear_acceleration, tcp.max_linear_jerk,
                     tcp.max_angular_velocity, tcp.max_angular_acceleration, tcp.max_angular_jerk})
      if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("Invalid gantry TCP caps");
    for (int i = 0; i < 3; ++i) {
      limits_.max_velocity[i] = std::min(limits_.max_velocity[i], tcp.max_linear_velocity / std::sqrt(3.));
      limits_.max_acceleration[i] =
          std::min(limits_.max_acceleration[i], tcp.max_linear_acceleration / std::sqrt(3.));
      limits_.max_jerk[i] = std::min(limits_.max_jerk[i], tcp.max_linear_jerk / std::sqrt(3.));
    }
  }
  const JointLimits &joint_limits() const { return limits_; }
  FollowingPlanResult plan(const JointState &activation, double epoch, double cycle, LivePlanStamp stamp,
                           std::uint64_t horizon, const ConveyorObservationHistory &history,
                           const TrackingFrameSnapshot &frames, ClockTime now) const {
    FollowingPlanResult r;
    if (frames.root() != frame_ || stamp.clock != now.clock || stamp.clock != frames.clock() || !horizon ||
        horizon > 100000 || stamp.start_tick > schedule_detail::max_ticks - horizon ||
        !std::isfinite(cycle) || cycle <= 0 || !std::isfinite(activation.timestamp_s) ||
        activation.timestamp_s < now.seconds + now.uncertainty_s) {
      r.diagnostic = "Invalid following frame/clock/horizon/activation";
      return r;
    }
    const double arrival = epoch + schedule_detail::tick_time(stamp.start_tick + horizon, cycle);
    r.prediction = history.predict(frames, now, {arrival, stamp.clock, now.uncertainty_s});
    if (!r.prediction) {
      r.diagnostic = r.prediction.diagnostic;
      return r;
    }
    const auto &t = *r.prediction.target;
    if (!kinematic_detail::fixed_orientation(t.pose)) {
      r.diagnostic = "XYZ following needs identity orientation";
      return r;
    }
    auto polynomial = plan_boundary_quintic(limits_, activation,
                                            {Vector(t.pose.position.begin(), t.pose.position.end()),
                                             Vector(t.velocity.begin(), t.velocity.end()),
                                             Vector(t.acceleration.begin(), t.acceleration.end())},
                                            arrival - activation.timestamp_s);
    if (!polynomial) {
      r.diagnostic = polynomial.diagnostic;
      return r;
    }
    auto schedule = make_polynomial_stop_schedule(polynomial.trajectory, cycle);
    if (!schedule) {
      r.diagnostic = schedule.diagnostic;
      return r;
    }
    auto prepared = prepare_live_plan<3>(schedule.program, epoch, stamp);
    r.plan = prepared.plan;
    r.diagnostic = prepared.diagnostic;
    r.tracking_end_tick = stamp.start_tick + horizon;
    return r;
  }

private:
  JointLimits limits_;
  std::string frame_;
};
#if defined(MOTION_WITH_SERIAL) || defined(MOTION_WITH_DIFFERENTIAL)
template <std::size_t N> class KinematicConveyorFollower {
public:
  KinematicConveyorFollower(std::shared_ptr<const IDifferentialKinematics> model, JointLimits limits,
                            CartesianLimits tcp, std::string frame = "world",
                            std::vector<std::shared_ptr<const IEnvelopeConstraint>> constraints = {})
      : model_(std::move(model)), limits_(std::move(limits)), tcp_(tcp), frame_(std::move(frame)),
        constraints_(std::move(constraints)) {
    if (!model_ || model_->dof() != N || frame_.empty())
      throw std::invalid_argument("Invalid kinematic following model/frame");
    check_limits(limits_, N);
    for (double x : {tcp.max_linear_velocity, tcp.max_linear_acceleration, tcp.max_linear_jerk,
                     tcp.max_angular_velocity, tcp.max_angular_acceleration, tcp.max_angular_jerk})
      if (!std::isfinite(x) || x <= 0)
        throw std::invalid_argument("Invalid following TCP caps");
    for (auto c : constraints_)
      if (!c)
        throw std::invalid_argument("Missing following constraint");
  }
  KinematicFollowingResult<N> plan(const JointState &activation, double epoch, double cycle,
                                   LivePlanStamp stamp, std::uint64_t horizon,
                                   const ConveyorObservationHistory &history,
                                   const TrackingFrameSnapshot &frames, ClockTime now) const {
    KinematicFollowingResult<N> r;
    if (frames.root() != frame_ || stamp.clock != now.clock || stamp.clock != frames.clock() || !horizon ||
        horizon > 100000 || stamp.start_tick > schedule_detail::max_ticks - horizon ||
        !std::isfinite(cycle) || cycle <= 0 || !std::isfinite(activation.timestamp_s) ||
        activation.timestamp_s < now.seconds + now.uncertainty_s) {
      r.diagnostic = "Invalid following frame/clock/horizon/activation";
      return r;
    }
    try {
      const double arrival = epoch + schedule_detail::tick_time(stamp.start_tick + horizon, cycle);
      r.prediction = history.predict(frames, now, {arrival, stamp.clock, now.uncertainty_s});
      if (!r.prediction) {
        r.diagnostic = r.prediction.diagnostic;
        return r;
      }
      const auto &t = *r.prediction.target;
      auto q = model_->ik(t.pose, activation.q);
      if (!q) {
        r.diagnostic = "No seed-local IK for following goal";
        return r;
      }
      Twist v{}, a{};
      for (int i = 0; i < 3; ++i) {
        v.data[i] = t.velocity[i];
        a.data[i] = t.acceleration[i];
      }
      auto boundary = inverse_differential(*model_, *q, v, a);
      if (!boundary) {
        r.diagnostic = boundary.diagnostic;
        return r;
      }
      auto polynomial =
          plan_boundary_quintic(limits_, activation, *boundary.boundary, arrival - activation.timestamp_s);
      if (!polynomial) {
        r.diagnostic = polynomial.diagnostic;
        return r;
      }
      auto moving = check_polynomial_constraints(*model_, *polynomial.trajectory, tcp_, constraints_);
      if (!moving) {
        r.diagnostic = "Following horizon: " + moving.diagnostic;
        return r;
      }
      auto schedule = make_polynomial_stop_schedule(polynomial.trajectory, cycle);
      if (!schedule) {
        r.diagnostic = schedule.diagnostic;
        return r;
      }
      auto stopped = check_stop_constraints(*model_, *schedule.program->fallback(), tcp_, constraints_);
      if (!stopped) {
        r.diagnostic = "Following fallback: " + stopped.diagnostic;
        return r;
      }
      auto prepared = prepare_live_plan<N>(schedule.program, epoch, stamp);
      r.plan = prepared.plan;
      r.diagnostic = prepared.diagnostic;
      r.tracking_end_tick = stamp.start_tick + horizon;
    } catch (const std::exception &e) {
      r.diagnostic = e.what();
    }
    return r;
  }

private:
  std::shared_ptr<const IDifferentialKinematics> model_;
  JointLimits limits_;
  CartesianLimits tcp_;
  std::string frame_;
  std::vector<std::shared_ptr<const IEnvelopeConstraint>> constraints_;
};
#endif
} // namespace motion
