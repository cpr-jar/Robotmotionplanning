# Prüfnachweis: Ressourcen und Abbruch — 0.16.0

Stand: **30.10.2026**, Basis **0.15.0 / f9ed167**. Dieses Paket behandelt F08
und die CI-Erweiterungen aus F10 des [Architektur-Reviews](../architecture/architecture_review_v0.13.0.md).
[API, Budgetzählweise und Grenzen](../integration/preparation_resources.md).

## Fachliche Gates

- Ein gemeinsamer Kontext zählt über Parse/Modellimport, Kompilierung, Rasterung,
  Simulation und Cachebau; Limits werden nicht pro Segment oder Phase neu gestartet.
- Ein zu kleiner Live-Byteetat wird **vor dem ersten Sample** abgelehnt. Exakte
  Zulassungsgrenze, zweite Generation und Überlauf um UINT64_MAX sind geprüft.
- Ein anderes Thread darf einen Stop anfordern; der Worker behält seine Zähler.
  Ein Stop beim letzten Cache-Sample verhindert ebenfalls die Rückgabe eines Plans.
- Fehlende Eingabebytes, Drawings, Work, Instructions und Simulationsticks werden
  mit unterscheidbaren Fehlercodes abgelehnt. Vorherige Programme bleiben nutzbar.
- Wiederholte gleiche DXF-Verweise laden einmal und parsen einmal. Einheitenwechsel
  erzeugen einen eigenen Parse; Platzierungen liefern weiterhin korrekte Endpunkte.
- Ein realer Editor-Worker wird bei Abbruch/Timeout beendet und reap't. Temporäre
  Dateien verschwinden; ein späterer Auftrag kann wieder starten.
- Ein Abbruch in Python und eine nach dem Rendering abgelaufene Deadline dürfen
  keine neue Ansicht freigeben. Die Browserprüfung benutzt den echten Abbruchknopf
  und HTTP-Dienst und bestätigt den Erhalt der vorherigen Vorschau.
- Ausdünnung erhält Pflichtpunkte und prüft auch die nicht angezeigten Zeilen.
  LF/CRLF/CR und mehrzeilige CSV-Quoting-Konventionen bleiben erhalten.

## Ausgeführte Prüfungen

Lokal Linux/x86-64, GCC 14.2.0, C++20, CMake 4.4.4, aktive Assertions,
Clang 19.1.7 für ThreadSanitizer. ASan: detect_leaks=0; UBSan/TSan: halt_on_error=1.

| Prüfung | Umfang |
| --- | --- |
| Release, Serial/Ruckig/KDL | 34/34 CTests |
| Debug -O1 -g1, Serial/Ruckig/KDL, ASan/UBSan | 34/34 CTests |
| Fokussierte ThreadSanitizer-Prüfungen | 3/3: Vorbereitung, Session-Lifecycle und Ausführungsfehler |
| Unabhängige Komponenten-Verbraucher | Execution, DXF, Program und Following: 4/4 |
| Selbstständige öffentliche Header | 47/47 einschließlich Serial/Ruckig |
| Architektur-Guard-Fixtures | 6/6; Datei-/Modul-/Komponentengraph bleibt azyklisch |
| Werkzeugtests | 38/38 Python-Tests, einschließlich 100.000 Zeilen und echtem Workerabbruch |
| Echter Browser | 3/3: Offline-Viewer, Editor samt Abbruch, DXF-Editor; keine Browserfehler/externe Anfragen |
| Komplette Bewegungsdaten | Sechs JSON/CSV-Dateien bytegleich zu 0.15.0: UR5e 1/10/20 ms, ABB/FANUC/DXF 10 ms |

[Protokolle, Graph und Hashes](evidence/resources_v0.16.0/README.md).
Das sind lokale Nachweise. Neue GitHub-CI-Jobs sind eingecheckt, ihr vollständiger
erfolgreicher Remote-Lauf wird hier nicht behauptet.

## Definierter Speicherlastfall

`test_streaming_validation_has_bounded_numeric_storage_and_checks_hidden_rows`
erzeugt aus einem echten sechsachsigen WAIT-Export eine konstante Spur mit
**100.000 Zeilen**, rund **35,9 MB CSV** und maximal **100 Anzeigepunkten**.
Die Prüfung ist vollständig; 99 Zeilen werden für die Darstellung behalten.

Der zusätzliche von `tracemalloc` erfasste Spitzenbedarf der Spuraufbereitung
liegt bei rund **0,34 MB** (Testgrenze 4 MiB). Der bereits existierende Eingabestring,
Metadaten, native Speicherbelegungen, Meshes und HTML sind **nicht** in dieser
Messung enthalten. Die Messung ist deshalb kein Prozess-RSS-Wert. Die alte
vollständige Float-Listen-Materialisierung und der StringIO-Puffer sind entfernt.
Ein Limitverstoß in Tick 1 wird erkannt, obwohl dieser Tick nicht angezeigt wird.

Bei sechs Achsen benötigt ein Live-Cache mit 200.000 Kommandos weiterhin **35,2 MB**
reine Befehlsdaten. Die Vorbereitung hat dafür eine explizite, vorab geprüfte
Bytezulassung. Eine andere Planrepräsentation oder ein kleinerer Onlinehorizont
wird mit diesem Paket nicht eingeführt.

## Reproduktion

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
python3 tools/architecture/check_layers.py
python3 tools/architecture/check_components.py --case execution --case dxf_import --case program --case following
cmake -S tools/motion_viewer -B build-motion-viewer -DCMAKE_BUILD_TYPE=Release
cmake --build build-motion-viewer --parallel 2
ctest --test-dir build-motion-viewer --output-on-failure
python3 -m venv build-browser-venv
build-browser-venv/bin/python -m pip install -r tools/motion_viewer/requirements-browser.txt
build-browser-venv/bin/python -m playwright install --with-deps chromium
build-browser-venv/bin/python tools/motion_viewer/tests/run_browser_checks.py --exporter build-motion-viewer/motion_viewer_exporter
```

Die Werkzeugtests haben wegen der instrumentierten 100k-Messung 600 s Timeout.
Das ist ein Testzeitbudget, kein zugesagtes Simulationstempo. CI besitzt zusätzlich
native `ubuntu-24.04-arm`- und TSan-Jobs. Letzterer stellt auf seinem isolierten
Runner die ASLR-Adressbreite für TSan ein; lokal wurde Clang 19 ohne Änderung der
Host-ASLR verwendet.

## Offen und ausdrücklich nicht gemessen

Pi 4B/ARM-Hardware-WCET, Antriebe und CANopen bleiben zurückgestellt. Native
ARM64-CI ist kein Pi-Lastnachweis. Es gibt keine harte Präemptions-/Abbruchlatenz
für Dateisystem-, Drittparser- oder fremde Loader-/Kinematikaufrufe, keine globale
RSS-/Heapgarantie über mehrere Kontexte und keine allgemeine vollständige TSan-
Abdeckung aller Solver/Parser. Eine lange Fuzzkampagne wurde für dieses Paket
nicht neu durchgeführt; die bestehenden Fuzz- und Zufallstests bleiben erhalten.
Die typisierte Programm-IR, Enclosure-Erweiterung und IK-Skalierung bleiben die
separaten nächsten Architekturthemen.
