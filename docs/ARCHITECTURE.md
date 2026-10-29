# Robot motion software architecture (v0.2.0)

```
Program → Motion Queue / Look-Ahead [planned]
              ↓
Geometric Path (PTP/LIN/CIRC/blending) [only basic straight XYZ implemented]
              ↓
Kinematics (FK/IK/J and derivatives) [XYZ implemented; Delta/serial plugins planned]
              ↓
Constraint Evaluation (joint + Cartesian + future dynamic limits)
              ↓
Time Parameterization (jerk constrained) [analytic quintic restricted case]
              ↓
Immutable validated trajectory → Real-time sampling [reference implementation]
              ↓
Joint setpoints (q, dq, ddq) → drive interface [outside current scope]

Tracked object + belt encoder timestamps → target predictor [simple prototype]
                                      ↘ online replanning [planned]
```

## API boundaries
- `motion/core.hpp`: robot states, poses, kinematics and limits.
- `motion/trajectory.hpp`: planned-trajectory types, planners, currently supported rest-to-rest joint and XYZ-LIN implementations.
- `motion/online.hpp`: online-generator interface and conservative rest-only example.
- `motion/online_joint.hpp`: optional legacy Ruckig Community prototype (separate implementation, needs validation and adapter alignment).
- `motion/tracking.hpp`: simplified belt prediction; not a conveyor tracking solver.

`QuinticRestTrajectory::sample(t)` produces position, velocity and acceleration for a continuous polynomial. The third derivative (jerk) is bounded analytically by its construction, rather than by sampling the points. Sampling returns vectors and thus allocates; a later hard real-time implementation must use preallocated output buffers.

## Timing and coordinate frames
All internal values use SI units. A `JointState` carries an absolute timestamp in seconds, while trajectory `sample(t)` uses **relative trajectory seconds**. Future tracking must add explicitly identified monotonic-clock epoch, uncertainty, transform snapshots and encoder measurement age.

## Limits and proof obligations
For `q_i(t)=q_i(0)+D_i s(t)` with `s(u)=10u³−15u⁴+6u⁵`, `u=t/T`:
- `max |ds/dt| = 15/(8T)`
- `max |d²s/dt²| = 10/(sqrt(3) T²)`
- `max |d³s/dt³| = 60/T³`
- s is monotone on [0,1], thus endpoints imply position bounds over whole segment.
For XYZ gantry linear TCP motion with fixed orientation, `||v_TCP|| = ||D|| |ds/dt|`; similarly for acceleration and jerk. The actual Cartesian norm is compared with the linear limits to choose T. No angular movement supported. For nonlinear robot kinematics these arguments do NOT apply.

## Next implementation increments
1. Integrate a Ruckig adapter under a unified online API, validate full continuous trajectory bounds and failure behaviour.
2. Implement branch-aware serial and Delta kinematics, derivative/limit interfaces and reference tests.
3. Separate immutable geometric path representation and jerk-aware global path time-parameterization with conservative or certified interval bounds.
4. Implement time-varying targets, look-ahead, online replanning from moving initial conditions, feasibility-aware stop.
5. Conveyor-following benchmark with encoder latency, workpiece uncertainty and dynamic target pose, including tests at all relevant boundaries.
