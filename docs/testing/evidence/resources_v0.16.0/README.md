# Prüfnachweise: Ressourcen 0.16.0

Lokal am 30.10.2026, Basis 0.15.0 / f9ed167. Keine Behauptung abgeschlossener
GitHub-Remote-Jobs. [Umfang und Grenzen](../../resources_validation.md).

- release-tests.log, asan-tests.log: 34 CTests, komplette Serial/Ruckig/KDL-Konfiguration.
- tsan-tests.log: drei fokussierte Clang-19-Protokollprüfungen.
- tool-tests.log: 38 Python-Tests, echte Worker und 100.000 Zeilen; der gedruckte
  tracemalloc-Wert schließt bereits vorhandene Eingabedaten/Meshes/HTML aus.
- browser-tests.log: drei echte Browser-Smokes einschließlich Editorabbruch.
- components.log, component-consumers.json: vier unabhängig gebaute Verbraucher.
- headers.log, architecture-tests.log, layers.json: Header und Architekturregeln.
- trace-comparison.json: sechs vollständige Exporte bytegleich zu 0.15.0.

Nachgestellte Leerzeichen in Protokollen wurden entfernt. Binaries, große
Spuren, Parser-/Build-Scratch und lokale Testserver werden nicht eingecheckt.
