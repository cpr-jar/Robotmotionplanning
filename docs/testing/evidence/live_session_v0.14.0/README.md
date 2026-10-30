# Prüfnachweise: Live-Sitzung 0.14.0

Lokal ausgeführt am 30.10.2026. Basis: 0.13.1 / cb41cb2.
[Umfang und Grenzen](../../live_session_validation.md). Nachgestellte Leerzeichen
in den Laufprotokollen wurden entfernt. Diese Dateien behaupten keinen bereits
abgeschlossenen GitHub-CI-Lauf.

- `release-tests.log`, `asan-tests.log`, `minimal-tests.log`: gesamte CTest-Suiten.
- `final-races-tests.log`, `final-races-asan-tests.log` und ggf. Minimal:
  zusätzlich ergänzte Angebot-/Fehlerrennen nach den gesamten Suiten.
- `tsan-tests.log`: gesamter finaler Lifecycle-Test unter Clang-19-ThreadSanitizer,
  einschließlich 200 Planwechseln und 200 Angebot-/Fehlerrennen.
- `controller-tests.log`: virtuelle sechsachsige Ausgabe bei 1/10/20 ms sowie
  Zeitfehler, Annahmefehler und Shutdown.
- `tool-tests.log`: 32 unittest-Fälle mit echtem neu gebauten C++-Exporter.
- `consumer.log`, `consumer-minimal-asan.log`: externe CMake-Einbindung.
- `headers.log`: 39 einzeln kompilierte öffentliche Header.
- `trace-comparison.json`: Export-SHA-256 für sechs vollständige Beispielspuren
  im Vergleich zu 0.13.1; keine zusätzlichen großen Spurdaten im Repository.

Build-/Reproduktionskommandos stehen im verlinkten Prüfnachweis. Der fokussierte
TSan-Test benötigt weder Serial noch Ruckig oder eine laufende HTTP-Oberfläche.
