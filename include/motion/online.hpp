#pragma once
#include "trajectory.hpp"
namespace motion {
// Interface for future streaming online replanners (Ruckig, custom, Pro).
// step() must never return an unverified setpoint on failure.
class IOnlineJointGenerator {
public:
  virtual ~IOnlineJointGenerator()=default;
  virtual StepStatus reset(const JointState& measured_or_commanded_state)=0;
  virtual StepStatus retarget(const JointTarget& new_target)=0;
  virtual StepStatus step(JointState& next_command)=0;
};
// A conservative fallback: plan a rest-to-rest move and reject retargeting
// while moving. Real online retargeting from nonzero v/a is NOT implemented.
class RestOnlyOnlineGenerator final : public IOnlineJointGenerator {
public:
  RestOnlyOnlineGenerator(JointLimits limits,double dt)
    : planner_(std::move(limits)),dt_(dt) {if(!(std::isfinite(dt_)&&dt_>0)) throw std::invalid_argument("dt");}
  StepStatus reset(const JointState& current) override {
    if(!is_rest(current)) return StepStatus::InvalidInput;
    state_=current; trajectory_.reset(); t_=0; return StepStatus::Finished;
  }
  StepStatus retarget(const JointTarget& target) override {
    if(!is_rest(state_)) return StepStatus::InvalidInput;
    auto result=planner_.plan(state_,target);
    if(!result) return StepStatus::InvalidInput;
    trajectory_=std::move(result.trajectory); t_=0; return StepStatus::Running;
  }
  StepStatus step(JointState& next_command) override {
    if(!trajectory_) return StepStatus::InvalidInput;
    t_=std::min(t_+dt_,trajectory_->duration());
    state_=trajectory_->sample(t_);
    next_command=state_;
    return t_>=trajectory_->duration()?StepStatus::Finished:StepStatus::Running;
  }
private:
  QuinticJointPlanner planner_;
  double dt_,t_{};
  JointState state_;
  TrajectoryPtr trajectory_;
};
}
