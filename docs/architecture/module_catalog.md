# Ordner und Modulzuständigkeiten

Das Verzeichnis `include/motion/` enthält die öffentliche C++-API. Die direkten
Header darin sind ausschließlich Weiterleitungen für bestehenden Quellcode.
Neue Anwendungen verwenden die aufgabenbezogenen Unterverzeichnisse.
`src/` folgt derselben Aufgabenverteilung; generische Algorithmen bleiben
teilweise headerbasiert. Es werden keine neuen C++-Namespaces erzwungen:
die öffentliche API liegt weiterhin in `motion`.

```text
robot-motion-cpp/
├── CMakeLists.txt                 Bibliotheksoptionen und Zusammenschaltung
├── cmake/                        Abhängigkeiten und gemeinsame Targetregeln
├── include/motion/
│   ├── core/                     Zustände, Grenzen, Uhrdomänen
│   ├── math/                     Ableitungs-Jets für Kinematik
│   ├── kinematics/               FK, IK, Jacobian, Referenzmodelle und Hüllen
│   ├── planning/                 Geometrie, Trajektorien, Zeitplanung und Halteplanung
│   ├── io/                       Roboter-Modellimport
│   ├── program/                  Anwenderprogramme und segmentierte Referenzen
│   ├── execution/                Zyklusraster, Puffer, Dispatcher und Live-Wechsel
│   ├── tracking/                 Frames, Beobachtungen, Prognose und Following
│   ├── calibration/              Starrer Punktkalibrierungs-Fit
│   └── platform/                 Optionale Linux-Uhr und Thread-Konfiguration
├── src/{kinematics,io,program,calibration}/
├── examples/
│   ├── integration/              Vertragstest und separates CMake-Anwenderprojekt
│   ├── programs/                 Programmimport, Simulation und CSV
│   ├── planning/                 Optionales Online-PTP-Beispiel
│   └── platform/                 Linux-Zeitdiagnose
├── tests/                        Nach denselben Aufgaben gegliederte Tests
│   ├── integration/              Alte/neue öffentliche Header
│   ├── fuzz/corpus/              Parser-Fuzz-Harness und Startkorpus
│   └── support/                  Gemeinsame Testmodelle
├── test_support/kinematics/kdl/   Nur die unabhängige KDL-Testreferenz
├── models/                       Profile, URDFs und belegte Upstream-Quellen
├── programs/examples/            Mitgelieferte Anwenderprogramme
├── tools/{architecture,models,visualization}/
├── docs/{integration,architecture,formats,development,testing,planning,history}/
├── third_party/                  Unveränderte Quellarchive und Lizenzen
└── backups/                      Historische Rekonstruktionssicherung
```

## Öffentliche Header

Die Spalte „zyklisch“ bewertet nur die aufgeführten Tätigkeiten. Konstruktion,
Destruktion und beliebige Benutzer-Callbacks gehören dadurch nicht automatisch
in den zyklischen Thread.

| Aufgabe / Header unter `include/motion/` | Wichtige Typen / Funktionen | Zuständigkeit; zyklisch |
| --- | --- | --- |
| `core/motion_types.hpp` | `JointState`, `JointLimits`, `Pose`, `Twist`, `IKinematics`, `CartesianGantry` | Grunddaten; Vektoren/Modellmethoden außerhalb RT |
| `core/clock_time.hpp` | `ClockTime`, `ClockId`, Alias `TrackingTime` | Sekunden, Uhrdomäne und Unsicherheit; feste Werttypen |
| `math/derivative_jets.hpp` | `differential_math` | Interne numerische Ableitungshilfen; kein Controller-Einstieg |
| `kinematics/differential_kinematics.hpp` | `IDifferentialKinematics`, `KinematicDerivatives` | Ableitungen bis Ruck; Planung/Prüfung |
| `kinematics/serial_chain_kinematics.hpp` | `SerialChainKinematics`, `SerialIKResult` | Serielle 1–12-Achs-Kette; Planung |
| `kinematics/reference_kinematics.hpp` | `ScaraKinematics`, `DeltaKinematics` | Referenzkinematiken; Planung |
| `kinematics/inverse_differential.hpp` | `inverse_differential()` | Räumliche Geschwindigkeit/Beschleunigung in dq/ddq; Planung |
| `kinematics/interval_kinematics.hpp` | `enclosure` | Numerische Hüllen zur Zulassung; Planung |
| `planning/joint_trajectory.hpp` | `ITrajectory`, `QuinticJointPlanner`, `QuinticRestTrajectory`, `GantryLinearPlanner` | Gelenk-/Gantry-Planung, Sampling außerhalb RT |
| `planning/boundary_quintic.hpp` | `BoundaryQuinticTrajectory`, `plan_boundary_quintic()` | Quintik mit q/v/a-Randzuständen; Planung |
| `planning/cartesian_path.hpp` | `CartesianLinePath`, `CartesianArcPath`, `IBoundedJointPath` | Normierte Weggeometrie s∈[0,1]; Planung |
| `planning/pose_path.hpp` | `PoseSweepPath`, `BlendedPosePath` | Orientierungssweep und C2-Blends; Planung |
| `planning/cartesian_spline.hpp` | `map_cartesian_spline()` | IK-Abbildung mit Pose-Schlauch; Planung |
| `planning/path_time_parameterization.hpp` | `PathTimeParameterizer`, `TimedPathTrajectory` | Konservative Zeitplanung mit Gelenk-/TCP-Grenzen |
| `planning/envelope_constraints.hpp` | `IEnvelopeConstraint`, `TcpObstacleConstraint` | Zusätzliche TCP-Hindernishüllen; Zulassung |
| `planning/constant_jerk_phase.hpp` | `phase_detail` | Unabhängige Integration konstanter Ruckphasen; Planung |
| `planning/joint_stop.hpp` | `JointStopPlanner`, `JointStopTrajectory` | Optionaler Ruckig-Haltekandidat plus unabhängige Prüfung |
| `planning/online_joint_generator.hpp` | `IOnlineJointGenerator`, `RestOnlyOnlineGenerator` | Allgemeiner Generatorvertrag; `step()` ist kein Dispatcher |
| `planning/ruckig_joint_generator.hpp` | `OnlineJointPlanner` | Optionaler geprüfter Ruckig-Generator; außerhalb RT |
| `io/robot_model_import.hpp` | `ImportedSerialRobot`, `load_serial_robot_files()` | URDF/SI-YAML; Datei- und Parserarbeit außerhalb RT |
| `program/robot_program.hpp` | `RobotProgramSource`, `RobotProgram`, `PreparedRobotProgram` | Strikter Programmimport, Gesamtplanung und Simulation |
| `program/joint_program.hpp` | `IJointProgram` | Segmentierte unveränderliche Referenz; Sampling außerhalb RT |
| `execution/execution_types.hpp` | `ExecutionStatus`, `ExecutionFault`, `ExecutionTimingPolicy` | Werttypen und Timingvertrag |
| `execution/execution_schedule.hpp` | `IExecutionSchedule` | Zeit-/Tickvertrag; Sampling außerhalb RT |
| `execution/cycle_aligned_program.hpp` | `CycleAlignedProgram`, `align_program_cycles()` | Stillstandsgrenzen auf ganzzahlige Ticks; Vorbereitung |
| `execution/boundary_polynomial_schedule.hpp` | `BoundaryPolynomialSchedule` | Unveränderte Quintik plus Stillstandspadding |
| `execution/setpoint_buffer.hpp` | `FixedJointCommand`, `SpscSetpointBuffer`, `BufferedMotionProducer` | `fill()` außerhalb RT; feste Push/Pop-Übergaben zyklisch |
| `execution/deadline_motion_executor.hpp` | `DeadlineMotionExecutor` | Gepufferte Ausgabe mit Daten-/Zeitprüfung; `poll[_clock]()` zyklisch |
| `execution/live_motion_executor.hpp` | `PreparedLivePlan`, `LivePlanMailbox`, `LiveMotionExecutor` | Cachebau außerhalb RT; feste Befehlsausgabe, Wechsel und Snapshot zyklisch |
| `execution/stop_schedules.hpp` | `JointStopSchedule`, `PolynomialStopSchedule`, `PreparedStopTransition` | Optional vorbereitete Halteverläufe; Sampling außerhalb RT |
| `tracking/constant_velocity_predictor.hpp` | `ConveyorPredictor` | Einfacher ursprünglicher Prädiktor; kein Ausführungs-Dispatcher |
| `tracking/frame_snapshot.hpp` | `TrackingFrameSnapshot` | Statische kalibrierte Framegraphen; außerhalb RT |
| `tracking/conveyor_target_predictor.hpp` | `ConveyorTargetPredictor` | Kausale/freshness-/Unsicherheitsprüfung; außerhalb RT |
| `tracking/conveyor_following.hpp` | `ConveyorObservationHistory`, `KinematicConveyorFollower` | Optionaler bewegter Horizont mit Halt; Planung außerhalb RT |
| `calibration/rigid_calibration.hpp` | `PointCorrespondence`, `fit_rigid_calibration()` | Gewichteter starrer Fit; außerhalb RT |
| `platform/linux_realtime.hpp` | `LinuxMonotonicClock`, `configure_realtime_thread()` | Uhr/absolutes Warten im Thread; FIFO/Affinität/mlock nur im Setup |

`MOTION_WITH_SERIAL` aktiviert die kompilierten Modell-, Programm-, seriellen
Differential- und Kalibrierfunktionen. `MOTION_WITH_RUCKIG` aktiviert die vier
optionalen Header `joint_stop`, `ruckig_joint_generator`, `stop_schedules` und
`conveyor_following`; sie verlangen ausdrücklich diese Build-Option. Die
KDL-Bibliothek erscheint ausschließlich bei aktivierten Tests und KDL-Referenz.
Details zur Quellkompatibilität: [Migration](../development/naming_and_migration.md).
