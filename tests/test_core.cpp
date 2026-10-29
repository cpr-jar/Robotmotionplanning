#include "motion/core.hpp"
#include "motion/tracking.hpp"
#include <cassert>
#include <cmath>
int main(){
  using namespace motion;
  CartesianGantry k;
  auto q=k.ik({{1,2,3},{1,0,0,0}}, {0,0,0});
  assert(q && q->size()==3);
  assert(k.fk(*q).position[1]==2);
  JointLimits limits{{-2,-2,-2},{2,2,2},{1,1,1},{2,2,2},{10,10,10}};
  assert(within_joint_limits({{0,0,0},{0,0,0},{0,0,0},0},limits));
  assert(!within_joint_limits({{0,0,0},{2,0,0},{0,0,0},0},limits));
  ConveyorPredictor tracker(2.0);
  ConveyorState belt;
  belt.velocity_m_s=0.5;
  TrackedTarget target{{{0.2,0.1,0.0},{1,0,0,0}},0.0};
  auto predicted=tracker.predict(target,belt,1.0);
  assert(predicted && std::abs(predicted->position[0]-0.7)<1e-10);
  assert(!tracker.predict(target,belt,3.0));
}
