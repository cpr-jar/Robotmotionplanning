#pragma once
#ifndef MOTION_WITH_RUCKIG
#error "Build with -DMOTION_WITH_RUCKIG=ON to use OnlineJointPlanner"
#endif
#include "core.hpp"
#include <ruckig/ruckig.hpp>
namespace motion {
// Joint-only ruckig planner. Does NOT guarantee Cartesian path or jerk bounds.
class OnlineJointPlanner {
public:
  OnlineJointPlanner(const JointLimits& limits, double cycle_s)
    : limits_(limits), cycle_s_(cycle_s), generator_(limits.max_velocity.size(),cycle_s),
      input_(limits.max_velocity.size()), output_(limits.max_velocity.size()) {
    check_limits(limits_,limits_.max_velocity.size());
    if(cycle_s<=0) throw std::invalid_argument("cycle_s must be positive");
    input_.max_velocity=limits_.max_velocity;
    input_.max_acceleration=limits_.max_acceleration;
    input_.max_jerk=limits_.max_jerk;
    input_.synchronization=ruckig::Synchronization::Time;
  }
  void reset(const JointState& current) {
    check_state(current,limits_.max_velocity.size());
    if(!within_joint_limits(current,limits_)) throw std::invalid_argument("Initial state violates limits");
    input_.current_position=current.q;
    input_.current_velocity=current.dq;
    input_.current_acceleration=current.ddq;
    initialized_=true;
  }
  void set_target(const JointTarget& target) {
    const auto n=limits_.max_velocity.size();
    if(target.q.size()!=n || target.dq.size()!=n || target.ddq.size()!=n)
      throw std::invalid_argument("Invalid target dimension");
    for(std::size_t i=0;i<n;++i)
      if(!std::isfinite(target.q[i]) || !std::isfinite(target.dq[i]) || !std::isfinite(target.ddq[i]) ||
          target.q[i]<limits_.min_position[i] || target.q[i]>limits_.max_position[i] ||
          std::abs(target.dq[i])>limits_.max_velocity[i] || std::abs(target.ddq[i])>limits_.max_acceleration[i])
        throw std::invalid_argument("Target exceeds limits");
    input_.target_position=target.q;
    input_.target_velocity=target.dq;
    input_.target_acceleration=target.ddq;
    has_target_=true;
  }
  StepStatus step(JointState& commanded) {
    if(!initialized_ || !has_target_) return StepStatus::InvalidInput;
    const auto result=generator_.update(input_,output_);
    if(static_cast<int>(result)<0) return StepStatus::Error;
    commanded={output_.new_position,output_.new_velocity,output_.new_acceleration,
               commanded.timestamp_s+cycle_s_};
    // Position limits need checking as Ruckig Community does not constrain extrema by default.
    // Rejecting a completed step is not a guaranteed safe stopping policy.
    if(!within_joint_limits(commanded,limits_)) return StepStatus::Error;
    output_.pass_to_input(input_);
    return result==ruckig::Result::Finished ? StepStatus::Finished : StepStatus::Running;
  }
private:
  JointLimits limits_;
  double cycle_s_;
  ruckig::Ruckig<ruckig::DynamicDOFs> generator_;
  ruckig::InputParameter<ruckig::DynamicDOFs> input_;
  ruckig::OutputParameter<ruckig::DynamicDOFs> output_;
  bool initialized_{false}, has_target_{false};
};
}
