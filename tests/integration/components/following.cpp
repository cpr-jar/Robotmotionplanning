#include "motion/kinematics/reference_kinematics.hpp"
#include "motion/tracking/conveyor_following.hpp"
#include <cassert>
#ifdef MOTION_WITH_SERIAL
#error "Generic following fixture must not depend on the serial solver"
#endif
int main() {
  using namespace motion;
  auto model = std::make_shared<ScaraKinematics>(ScaraGeometry{.4, .3},
                                                 KinematicJointLimits{{-3, -3, -1, -3}, {3, 3, 1, 3}});
  const Vector q{.3, .5, .1, -.8};
  TrackingFrameSnapshot frames("world", {}, 1, {0, 7, 0}, {100, 7, 0});
  ConveyorObservationHistory history("part", 1, {"world"});
  assert(history.ingest({"part", "world", model->fk(q), {10, 7, 0}, {10, 7, 0}, 0, 0},
                        {0, 0, 0, {10, 7, 0}, {10, 7, 0}, 0, 0}, frames, {10, 7, 0}));
  const auto bounds = model->position_limits();
  KinematicConveyorFollower<4> follower(
      model, {bounds.lower, bounds.upper, Vector(4, 2), Vector(4, 5), Vector(4, 50)},
      {2, 10, 100, 2, 10, 100});
  const auto prepared =
      follower.plan({q, Vector(4), Vector(4), 10}, 10, .01, {1, 0, 0, 7}, 20, history, frames, {10, 7, 0});
  assert(prepared && buffer_detail::rest(prepared.plan->command(prepared.plan->end_tick())));
}
