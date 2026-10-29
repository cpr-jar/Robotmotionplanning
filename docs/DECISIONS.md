# Architecture decisions and agreements

Status: agreed project direction, 2026-10-28. This is the source of truth for design intent, not a claim of production readiness.

## Scope
- Robot-controller-side motion planning only; **exclude** servo firmware, power electronics and fieldbus control.
- C++20, CMake; SI units internally (m, rad, s).
- Support XYZ gantries, Delta robots, and serial kinematics through robot-specific kinematics plugins.
- Jerk limits **must** be respected in both joint and Cartesian domains for appropriate movement types. A joint jerk bound does **not** imply a Cartesian TCP jerk bound for nonlinear kinematics.
- Design for online replanning and later **conveyor tracking** (moving workpieces): eventually allow tracking a moving target over time, not merely a one-time intercept.
- Separate geometry/path planning, kinematics/constraints, time parameterization, online replanning, and deterministic realtime interpolation.
- Use Ruckig **Community initially** for joint-space online motion where appropriate, behind an interchangeable adapter; avoid dependence on its cloud intermediate-waypoint service. Do **not** presume Ruckig Pro is necessary; evaluate if proven needed, and obtain a quote.
- Own geometric Cartesian planning and constraint/time-parameterization design. Do not assume Ruckig, including Pro, alone guarantees simultaneous Cartesian and joint kinematic limits.
- Never present a per-cycle setpoint check as proof that the whole planned trajectory respects position extrema or as a safe stop policy.

## Implemented in v0.2.0
- `ITrajectory`, `IJointTrajectoryPlanner`, `ICartesianPathPlanner`, `IOnlineJointGenerator` contracts.
- An immutable rest-to-rest quintic trajectory, with analytic **continuous-time** velocity/acceleration/jerk bounds along straight joint-space segments.
- `GantryLinearPlanner`: fixed-orientation XYZ straight-line motion with analytically proven continuous-time joint and Cartesian **linear** v/a/j bounds; start/target constraints checked before publication.
- A deliberately restricted rest-only streaming example; it rejects mid-motion retargets rather than claiming unsafe replanning.
- Existing Ruckig joint-space prototype remains feature-gated by CMake, not validated in this environment.
- Existing conveyor position predictor remains a simple constant-acceleration, no-rotation model, NOT real tracking/replanning.

## Intentionally not implemented / acceptance gates
1. General nonzero initial/target velocity or acceleration, jerk-bounded online replanning, and feasible constrained stopping.
2. Delta and serial FK/IK with continuity, singularity analysis, and derivatives through third order.
3. Continuous-time LIN/CIRC proof with simultaneous Cartesian *angular* and linear v/a/j, joint v/a/j for nonlinear kinematics and blending.
4. Look-ahead, motion queue, high-confidence online target updates and conveyor tracking with timestamps, frame transformation and latency compensation.
5. Obstacle/collision detection, payload-dependent torque constraints, independently assured safety functions, runtime timing guarantees and fault-handling validation.
6. Third-party Ruckig integration build/test and full trajectory extrema enforcement for that prototype.

These missing features are blockers for deployment on a physical robot.

## Design guardrails
- Generated trajectory is validated before it is published to an interpolator.
- Reject unsupported cases; don't silently degrade a LIN to joint PTP.
- Explicitly distinguish proof of limits (fixed-orientation gantry quintic) from numerical sampling or best effort.
- Retargeting while moving needs a feasibility-aware planner starting from q/dq/ddq, not a restarted rest-to-rest curve.
