# Architektur-Review: Korrekturpaket 1 — 0.13.1

Stand: **30.10.2026**, Bibliothek **0.13.1**, separates Testwerkzeug **0.4.1**.
Basis ist das [historische Review für `ecf4259`](../architecture/architecture_review_v0.13.0.md).
Dieser Nachweis ergänzt es; die ursprünglichen Befunde und Bewertungen bleiben
als Beschreibung des damaligen Stands erhalten.

## Umgesetzte Befunde

| Befund | Änderung | Nachweis |
| --- | --- | --- |
| **F01: ungeprüfte Providerformen** | Live-Vorbereitung prüft Limits, alle drei Zustandsvektoren und jedes Sample gegen N, außerdem Anfangszustand und Rastermetadaten. Zeitplanung prüft Bounds/Punkte einschließlich Ableitungen vor jedem indizierten Zugriff; IK-Spline-Aufbau prüft gelieferte Gelenkgrenzen, IK-Vektoren und Posen. | `provider_contracts_tests`: leere, zu kurze und zu lange Rückgaben, 64-Achs-Reproducer bei N=1, NaN-Werte, falsche Zeiten, ungültige Ableitungen und Fehler erst im Pfadinneren. |
| **F02: vertauschte Referenz und Raster** | `CycleAlignedProgram::references()` macht die tatsächliche Quellidentität prüfbar. `PreparedRobotProgram::validate()` und der Simulator weisen falsche Paare ab. Geprüfte Factories erzeugen passende Referenz-/Rasterpaare; Editor und Steuerungsbeispiel nutzen geprüftes Retiming. | `robot_program_tests`: WAIT-Referenz mit PTP-Raster sowie umgekehrter Fall werden vor CSV-Ausgabe abgewiesen; auch separat kompilierte identische Programme gelten nicht als dieselbe Referenz. Retiming für 1/10/20 ms erhält Referenz und Endzustand. |
| **F06: NaN-Erfolg im älteren Tracking** | `ConveyorPredictor` verlangt einen endlichen positiven Horizont, endliche Zeit-/Pose-/Banddaten und endliche Rechenergebnisse. | `tracking_tests`: NaN und beide Unendlichkeiten in den Eingabefeldern, Nullachse und Überlauf werden abgewiesen; der bisherige gültige Vorhersagefall bleibt erhalten. |
| **F09: widersprüchliche Ergebnisveröffentlichung** | Registrierung und Begrenzung der verfügbaren Ergebnisse sind der Commit. Nachfolgende Dateibereinigung läuft außerhalb der Ergebnissperre; ein OSError wird protokolliert und später erneut versucht. | Echte native Simulationen mit injiziertem Löschfehler liefern weiterhin das neue abrufbare Ergebnis. Alte URLs laufen korrekt ab; der nächste erfolgreiche Lauf räumt die zurückgestellten Dateien auf. Ein Schreibfehler vor Commit lässt vorherige Ergebnisse unverändert. |

Die neuen C++-Regressionen für die drei ersten Befunde und der Editor-Test für
F09 scheiterten am vorherigen Verhalten. Die Korrekturen werden nicht nur anhand
erfolgreicher Beispielprogramme beurteilt.

## Kompatibilität und Grenzen

- YAML-Schema 1, DXF-Umfang, bestehende Headerpfade und `RobotMotion::motion`
  bleiben erhalten. Es wird keine neue Laufzeitbibliothek eingeführt.
- `PreparedRobotProgram` bleibt als öffentliches Paar verfügbar, damit bestehende
  Aufrufer kompilieren. Es ist **kein unveränderlicher Werttyp**. Eigene Verbraucher
  rufen `validate()` vor der gemeinsamen Verwendung auf; der Bibliothekssimulator
  tut dies selbst. Bevorzugte Erzeugung: `prepare_robot_program()` bzw.
  `retime_robot_program()`. Beide gehören in die Vorbereitung außerhalb RT.
- Die Identitätsprüfung bindet das Raster an das konkrete Referenzobjekt. Sie
  ersetzt keine spätere Modell-/Kalibrierrevision oder Host-Commit-Semantik.
- Provider dürfen sich während ihrer Verwendung nicht nebenläufig ändern.
  Form-/Endlichkeitsprüfungen können keine mathematisch falschen fremden
  Ableitungshüllen oder falschen Kinematikmodelle beweisen.
- Bei anhaltenden Dateisystemfehlern können abgelaufene Ergebnisdateien bis zu
  einer erfolgreichen Bereinigung oder dem Sitzungsende liegen bleiben. Der
  öffentliche Ergebnisbestand bleibt auf drei Stände begrenzt; ein globales
  Speicher-/Plattenbudget ist weiterhin eine eigene Maßnahme.
- Reservierter Live-Cache-Speicher vermeidet wiederholtes Vektorwachstum. Die
  vollständigen Caches und Python-Spuren bestehen weiterhin; F08 ist nicht erledigt.

## Bewegungsdaten unverändert

Der Vergleich verwendet den vorher vorhandenen nativen Exporter von 0.13.0 und
den neu gebauten Exporter von 0.13.1. Die vollständigen JSON-Exporte einschließlich
CSV-Sollwertspur und Diagnosedaten sind für diese sechs Fälle **bytegleich**:

| Programm | Raster | Sollwerte |
| --- | --- | ---: |
| `ur5e_demo.yaml` | 1 ms | 9829 |
| `ur5e_demo.yaml` | 10 ms | 987 |
| `ur5e_demo.yaml` | 20 ms | 495 |
| `abb_irb120_demo.yaml` | 10 ms | 1039 |
| `fanuc_lrmate200id_demo.yaml` | 10 ms | 998 |
| `ur5e_dxf_demo.yaml` | 10 ms | 1181 |

Das ist ein konkreter Regressionsnachweis für die genannten Beispiele, kein
Beweis unveränderter numerischer Ergebnisse für alle möglichen Programme.

## Prüfungen reproduzieren

Tatsächlich ausgeführt unter Linux/x86-64 mit GCC 14.2.0, C++20 und CMake 4.4.4,
mit aktiven Assertions und strikten Warnungen. Bestehende Buildverzeichnisse
wurden neu konfiguriert und betroffene Targets neu kompiliert; dies ist kein
behaupteter Clean-Build jeder möglichen Plattformkonfiguration.

| Konfiguration | Ergebnis |
| --- | --- |
| Core Release, Serial/Ruckig/KDL an | **30/30 CTests bestanden** |
| Core Debug `-O1 -g1`, ASan/UBSan, Serial/Ruckig/KDL an | **30/30 CTests bestanden** |
| Minimal Release, Serial/Ruckig aus | **11/11 CTests bestanden** |
| Separater Viewer/Editor Release | **32/32 Python-Tests bestanden** |
| Separater Viewer/Editor mit ASan/UBSan-Exporter | **32/32 Python-Tests bestanden** |
| Browser: Editor und DXF | Beide bestanden; keine Browserfehler oder externen Anfragen |
| Externer CMake-Consumer | Gebaut und ausgeführt |
| Öffentliche Header einzeln, Serial/Ruckig an | **37/37 kompiliert** |

Die Provider-Testfälle wurden anschließend um weitere Pose- und
Innenpunktfehler ergänzt; der betreffende Test wird zusätzlich in Release,
ASan/UBSan und Minimal geprüft. Laufprotokolle und Spurhashes stehen unter
[Prüfnachweise](evidence/architecture_review_v0.13.1/README.md).

Die Python-Tests werden als ein CTest-Eintrag ausgeführt; die Zahl 32 bezeichnet
die darin ausgeführten unittest-Fälle. Der Sanitizer instrumentiert den nativen
C++-Exporter, nicht den Python-Interpreter. LeakSanitizer war deaktiviert;
UBSan lief mit `halt_on_error=1`.

```sh
cmake -S . -B build-review -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build-review --parallel 2
ctest --test-dir build-review --output-on-failure

cmake -S . -B build-review-asan -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS_DEBUG="-O1 -g1" -DMOTION_WITH_RUCKIG=ON -DMOTION_SANITIZE=ON
cmake --build build-review-asan --parallel 2
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build-review-asan --output-on-failure

cmake -S tools/motion_viewer -B build-motion-viewer -DCMAKE_BUILD_TYPE=Release
cmake --build build-motion-viewer --parallel 2
ctest --test-dir build-motion-viewer --output-on-failure
```

Browserprüfungen benötigen optional Playwright und lokales Chromium:

```sh
python3 tools/motion_viewer/tests/editor_browser_smoke.py
python3 tools/motion_viewer/tests/dxf_browser_smoke.py
```

Es gibt weiterhin keinen Raspberry-Pi-WCET-, TSan- oder Hardwareabnahmenachweis.

## Nächste Arbeitspakete

Das erste Paket behebt F01/F02/F06/F09. Offen bleiben insbesondere Lebensdauer
und Host-Übergabe, Modulzyklen/Targets (F03), Erweiterungspunkte und Programmmodell
(F04/F05), IK-Skalierung (F07), Gesamtbudgets (F08), zusätzliche CI-Nachweise
(F10), Diagnosen (F11) und optionale Optimierungen (F12).
Reihenfolge und Abhängigkeiten: [Roadmap](../planning/roadmap.md).
