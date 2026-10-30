# Prüfnachweise: Komponenten 0.15.0

Lokal ausgeführt am 30.10.2026, Basis 0.14.0 / d5494c1.
[Umfang und Grenzen](../../components_validation.md). Nachgestellte Leerzeichen
in den Laufprotokollen wurden entfernt. Kein behaupteter abgeschlossener GitHub-CI-Lauf.

- layers.json: gemessener Include-/Komponentengraph und Runtime-Abschlüsse.
- component-consumers.json: elf getrennt gebaute Verbraucher, tatsächliche Targets,
  CPP-Quellen und Archive aus der CMake File API.
- component-consumers-asan.json: zwei direkte Komponenten-Sanitizer-Consumer.
- release-tests.log, asan-tests.log, serial-default-tests.log, minimal-tests.log,
  ruckig-only-tests.log: Gesamt-CTests der Legacy-Matrix.
- headers.log, architecture-tests.log: Header und sechs Guard-Fixtures.
- legacy-consumer*.log: bestehende externe Fassade weiter verwendbar.
- tool-tests.log: 32 Python-Tests mit selektiv neu gebautem nativen Exporter.
- tsan-tests.log: fokussierter finaler Session-Lifecycle unter Clang 19.
- fuzz-program.log, fuzz-dxf.log: je 1000 libFuzzer-Läufe, temporäre Korpuskopien.
- trace-comparison.json: sechs vollständige Exporte unverändert zu 0.14.0.

Große Buildverzeichnisse, Binaries und Spuren werden nicht eingecheckt.
