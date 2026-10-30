# Changelog

## 0.14.0 — Managed plan ownership and delivery lifecycle, 2026-10-30

- Added LiveMotionSession and its non-RT LivePlanOwner: retain active/offered
  caches, collect retirement replies, cancel pending offers, and release only
  after cyclic quiescence. Cache destruction stays outside the cyclic thread.
- Added synchronous, noexcept sink acceptance followed by delivery commit;
  rejection and execution faults latch a structured diagnostic and stop further
  normal output. The terminal command is accepted exactly once.
- Added explicit cyclic shutdown and cancellation to the existing low-level
  executor, retaining its prior APIs and enum values. Snapshots report quiescence.
- Preserve BufferedMotionProducer's original exception and failed tick, and latch
  reference failures instead of silently retrying on a later refill.
- Migrated the controller contract example and external consumer to the managed
  API; added lifecycle/concurrency/allocation regressions, diagrams and a detailed
  integration contract. CANopen and physical stop integration remain deferred.

## 0.13.1 — Architecture review corrections, 2026-10-30

- Reject malformed schedule/path dimensions and nonfinite provider values before
  fixed-array copies or indexing, including IK spline outputs and position limits.
- Bind simulation metadata to the actual cycle-grid source identity; add checked
  preparation/retiming factories and validate legacy reference/schedule pairs.
  The editor exporter and controller example use the checked retiming operation.
- Reject nonfinite inputs and overflowing predictions in the legacy ConveyorPredictor.
- Separate the editor's successful publication commit from expired-result cleanup;
  failed cleanup is logged and retried after a later successful run. Tool: 0.4.1.
- Add regression coverage and retain the original architecture review with a
  separate implementation status and validation record.

## 0.13.0 — DXF program command and test-tool import, 2026-10-29

- Added a bounded preparation-time text DXF importer for POINT, LINE,
  LWPOLYLINE, 2D/3D POLYLINE, ARC and CIRCLE, including signed bulges.
- Added a real `DXF` YAML instruction, including CALL expansion, units/layer,
  TCP/world placement, explicit entry/connection policy, path tolerance and CAD provenance.
  Geometry uses the existing LIN/CIRC/PTP planning, limits and virtual execution.
- Added editor DXF attachments, command insertion, stale-result tracking for CAD
  edits, exact YAML+DXF bundles and path-focus controls. Tool version is 0.4.0.
- Added native geometric/program/fuzz tests, editor HTTP/browser regressions,
  an audited ezdxf reference fixture and separate DXF libFuzzer/CI coverage.
  Native/library work is under io/program; editor and visual work stay in tools.

## 0.12.3 — Integration groundwork and project structure, 2026-10-29

- Grouped public headers, sources, tests, examples, programs, tools and documentation
  by task; retained all 34 flat public headers as compatibility forwarders.
- Added `ClockTime` in core (retaining `TrackingTime` as an alias), separated execution
  value types and clarified private timing/dispatch names without changing motion laws.
- Split library, test and example CMake configuration; added `RobotMotion::motion`
  and `RobotMotion::serial` aliases and verified a standalone CMake consumer.
- Added an executable virtual controller-contract example for 1/10/20 ms plus
  latched deadline/axis-refusal scenarios; concrete controller integration remains later work.
- Added a detailed German integration/data/model-format handbook, naming/migration rules,
  API/module catalog, seven SVG UML diagrams and sequence/state views.
- Added a self-contained offline HTML handbook, diagram/declaration/link checks,
  isolated-header compilation and documentation/consumer CI verification.

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
