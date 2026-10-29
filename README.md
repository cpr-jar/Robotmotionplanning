# Robot Motion Core — v0.2.0 (C++20 prototype)

Scope: robot-controller-side motion planning only. No servo control or fieldbus.

**Read first:** [Decisions and agreements](docs/DECISIONS.md), [Architecture and proof boundaries](docs/ARCHITECTURE.md), [Changelog](CHANGELOG.md).

## Current implementation
- Robot-independent state and limits with units **SI** (meters, radians, seconds).
- Kinematics contract with FK, seed-dependent IK and Jacobian; implemented XYZ gantry sample.
- Pluggable `IKinematics` for serial manipulators and Delta robots (actual solvers **NOT implemented**).
- Optional Ruckig-based online **joint-space** prototype (not verified against downloaded Ruckig in this environment).
- `ITrajectory` and separate joint/Cartesian/online planner interfaces.
- A fully analytic **rest-to-rest** quintic joint trajectory with continuous-time v/a/jerk bounds and checked joint position limits.
- XYZ gantry fixed-orientation LIN with continuous-time TCP **linear** and joint v/a/jerk bounds.
- An explicitly rest-only online sample that refuses retargeting during motion.
- Minimal linear-conveyor target predictor with staleness check (does not perform online intercept planning).
- CMake project and smoke tests for implemented core.

## Build
```
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

For online joint planner (needs internet during first CMake configuration):
```
cmake -S . -B build-ruckig -DMOTION_WITH_RUCKIG=ON
cmake --build build-ruckig
./build-ruckig/online_ptp
```

Ruckig dependency version is pinned by CMake. Review upstream license before commercial use.

## Essential limitations / next milestones
1. `OnlineJointPlanner` is not a LIN/CIRC or Cartesian motion planner. It does not guarantee TCP path, TCP acceleration or TCP jerk limits.
2. Ruckig Community does not impose position extrema throughout the planned trajectory. Current per-step limit check is not a substitute for ahead-of-time validation and safe stopping. Add full trajectory extrema checking before machine deployment.
3. Add kinematics plugins for Delta and serial arm, including branch tracking, singularity handling, and tests against reference models.
4. Add geometric Cartesian path representation, path derivatives `q_s`, `q_ss`, `q_sss`, constraint evaluation and a **jerk-constrained** path time-parameterizer with certified continuous interval bounds.
5. Add trajectory look-ahead, motion queue, realtime buffering, online replanning from q/dq/ddq and feedforward target velocity for moving conveyors.
6. Track moving workpieces in an explicitly timestamped frame tree. Account for belt encoder timestamp, latency, acceleration, calibration, rotational transforms, detection uncertainty and interception time.
7. Add independent real-time watchdog, guaranteed feasible deceleration/stop, executable deadline monitoring, and safety-function separation.
8. Test worst-case kinematic singularities, limits, missed deadlines, wrong belt measurements, varying targets, and CI sanitizers.

This is an **engineering starter**, not a certified or production-safe robot motion controller.
