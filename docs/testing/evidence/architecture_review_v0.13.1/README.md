# Prüfnachweise: Architekturkorrekturen 0.13.1

Lokal ausgeführt am 30.10.2026. Umfang und Grenzen stehen im
[Umsetzungsnachweis](../../architecture_review_fixes.md). Dies sind lokale
Laufprotokolle (nachgestellte Leerzeichen entfernt), keine Behauptung eines bereits abgeschlossenen GitHub-CI-Laufs.

- `release-tests.log`, `asan-tests.log`, `minimal-tests.log`: gesamte CTest-Suiten.
- `tool-tests.log`, `tool-asan-tests.log`: unittest-Ausgabe des separaten Tools.
- `provider-final-tests.log` und weitere `provider-final-*.log`: ergänzte
  Negativfälle nach Abschluss der Gesamtsuiten.
- `editor-browser.log`, `dxf-browser.log`: echte Chromium-Smokes.
- `consumer.log`, `headers.log`: externer Build und isolierte Headerprüfung.
- `trace-comparison.json`: sechs bytegleiche vollständige Exporte samt SHA-256,
  verglichen mit dem vorhandenen Exporter von 0.13.0. Die großen JSON-Spuren
  selbst werden nicht zusätzlich im Repository gespeichert.

Die negativen Adapter stehen in `tests/integration/test_provider_contracts.cpp`;
sie sind Tests auf abzuweisende Eingaben, keine Anwendungsbeispiele.
