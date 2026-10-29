# Changelog

## 0.2.0 — 2026-10-28
- Documented agreed scope and engineering decisions in `docs/DECISIONS.md`.
- Added versioned architecture and explicit implemented vs. planned capability list.
- Added `ITrajectory`, `IJointTrajectoryPlanner`, `ICartesianPathPlanner`, and `IOnlineJointGenerator`.
- Added analytic, jerk-bounded quintic rest-to-rest joint planner and XYZ gantry Cartesian LIN planner.
- Added tests for limits, rejecting unsupported moving-state retargets and invalid goals.
- Ruckig joint prototype retained; it has not been rebuilt against upstream library in this environment.

## 0.1.0 — 2026-10-28
- Initial C++20 starter, XYZ kinematics, optional Ruckig joint prototype, basic conveyor predictor.
