#include "motion/online_joint.hpp"
#include <iostream>
int main() {
  using namespace motion;
  JointLimits limits{{-2,-2,-2},{2,2,2},{1,1,1},{2,2,2},{10,10,10}};
  OnlineJointPlanner planner(limits,0.001);
  planner.reset({{0,0,0},{0,0,0},{0,0,0},0});
  planner.set_target({{1,0.5,-0.3},{0,0,0},{0,0,0}});
  JointState sample{{0,0,0},{0,0,0},{0,0,0},0};
  for(int i=0;i<10000;++i) {
    const auto status=planner.step(sample);
    if(status==StepStatus::Error || status==StepStatus::InvalidInput) {std::cerr<<"Planning failed\n";return 1;}
    if(i%100==0) std::cout<<sample.timestamp_s<<','<<sample.q[0]<<','<<sample.dq[0]<<','<<sample.ddq[0]<<'\n';
    if(status==StepStatus::Finished) return 0;
  }
  return 2;
}
