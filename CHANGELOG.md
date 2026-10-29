# Changelog

## 0.12.2 — Robustness verification, 2026-10-29

- Added kinematic edge cases: near singularities/limits, angle wrap and quaternion
  signs, half-turn targets, zero/tiny lines and signed full circles; explicit
  rejection diagnostics and admitted-path limit/tube checks.
- Added an offline, independently constructed Orocos KDL 1.5.1 FK/J reference:
  1,866 configurations across three public robot models and 24 random mixed chains.
- Added clock/data fault injection, late/fenced live swaps, mailbox/ring backpressure,
  duplicate-command rejection, stale-measurement and calibration-change fallback tests.
- Added 120 reproducible mixed programs, transactional publication failures,
  12,000 parser mutations, resource-limit regressions and a Clang libFuzzer target/CI job.
- Fixed XYZ gantry rejection of equivalent negative identity quaternions and
  acceptance of nonfinite pose/joint/seed inputs.
- KDL is test-only; the production motion libraries retain their existing dependencies.

## 0.12.1 — Rekonstruktion, 2026-10-29

- Retained the uploaded v0.2.0 as the initial Git commit.
- Reconstructed serial FK/J/analytic derivatives/local IK, strict model import,
  public nominal robot fixtures and offline pinned dependencies.
- Reconstructed numerical boundary-polynomial/path envelopes, Cartesian splines,
  rotation/blending, SCARA/Delta reference kinematics and cycle-aligned execution.
- Reconstructed SPSC dispatch, clock/fault gates, immutable live cache replacement,
  delivery fences, coherent snapshots and nonallocating dispatch tests.
- Reconstructed optional Ruckig joint retargeting, independently integrated stops,
  prepared transitions and continuous XYZ/six-axis conveyor following.
- Reconstructed calibration, TCP obstacle checks and Linux timing facilities.
- Reconstructed YAML programs, PTP/LIN/CIRC, speed factors, WAIT/CALL, virtual-clock
  simulation, CSV and optional plots; all tested anew.
- CANopen intentionally excluded. Full link/self-collision and payload/torque
  modules and selected legacy convenience APIs remain unreconstructed.
- The lost v0.12.0 is not claimed byte-identical or test-identical. See recovery report.

## 0.2.0 — Uploaded baseline

The original uploaded source is preserved by the first repository commit; later
recovery commits implement the documented continuation. Historical versions
0.3–0.12 were described in the conversation but are not accessible as original files.
