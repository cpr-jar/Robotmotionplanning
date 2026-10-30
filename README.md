# Robot Motion C++ — 0.13.1

C++20-Bibliothek für Roboterkinematik, Bewegungsplanung, Anwenderprogramme und
zeitlich geprüfte Sollwertausgabe. Der Quellcode ist nach Aufgaben gegliedert;
die Einbindung in eine eigene Steuerung ist für einen Taktbereich von 1–20 ms
beschrieben. Die konkrete Host-/Achsanbindung wird später festgelegt.

**[Dokumentationsübersicht](docs/README.md)** ·
[Einsteiger-Tutorial mit Ablaufdiagrammen](docs/tutorials/motion_planning_basics.md) ·
[Integrationshandbuch](docs/integration/controller_integration.md) ·
[Klassendiagramme](docs/architecture/class_diagrams.md) ·
[Offline-Handbuch](docs/handbook.html) (lokal im Browser öffnen, auch als PDF druckbar).

Die Basis wurde aus der hochgeladenen ZIP **v0.2.0** und dem Gespräch
rekonstruiert; sie ist keine byteidentische Wiederherstellung der verlorenen v0.12.0.
[Wiederherstellungsbericht](docs/history/recovery_status.md) · [Programmformat](docs/formats/robot_program_yaml.md)
· [Architektur](docs/architecture/overview.md) · [Prüfbericht](docs/testing/validation.md).
Die anschließenden vier Robustheitspakete sind in [Robustheitstests](docs/testing/robustness.md) beschrieben.

**Architektur-Review:** [Befunde für 0.13.0](docs/architecture/architecture_review_v0.13.0.md)
und [Umsetzung des ersten Korrekturpakets in 0.13.1](docs/testing/architecture_review_fixes.md).

## Starten

C++20, CMake ab 3.20 und ein C++-Compiler werden benötigt. Die benötigten
C++-Abhängigkeiten liegen als versions- und hashgebundene Quellarchive bei;
**auch der optionale Ruckig-Build benötigt keinen Internetzugang**.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/program_simulator programs/examples/ur5e_demo.yaml --check
./build/program_simulator programs/examples/ur5e_demo.yaml --csv trace.csv
```

Weitere vollständige Beispiele:
`programs/examples/abb_irb120_demo.yaml` und `programs/examples/fanuc_lrmate200id_demo.yaml`.
Kopiere eine passende Beispieldatei und passe Punkte und Schritte an.
Der Simulator verbindet sich mit keiner Hardware.

Neu: **DXF als Programmbefehl**. `programs/examples/ur5e_dxf_demo.yaml` fährt
eine CAD-Kontur über die vorhandene LIN-/CIRC-Planung ab. Text-DXF mit Punkten,
Linien, Polylinien und Kreisen/Bögen; Einheiten, Layer und starre Platzierung
sind einstellbar. [DXF-Anleitung](docs/formats/dxf_program_command.md).
Der separate [Programmeditor](tools/motion_viewer/EDITOR.md) kann DXF-Dateien
öffnen, den Befehl einfügen und das Ergebnis mit dem vollständigen UR5e darstellen:

```sh
python3 tools/motion_viewer/run_editor.py --open
```

Der Vertragstest für eine spätere Eigensteuerung verwendet eine ideale virtuelle
Uhr und simulierte Achsen:

```sh
./build/controller_contract_example programs/examples/ur5e_demo.yaml 1 normal
./build/controller_contract_example programs/examples/ur5e_demo.yaml 20 normal
./build/controller_contract_example programs/examples/ur5e_demo.yaml 10 late-clock
./build/controller_contract_example programs/examples/ur5e_demo.yaml 10 sink-refusal
```

Für einen Quellcodeeinbau wird `RobotMotion::motion` mit `add_subdirectory`
verlinkt. Tests und Beispiele sind dabei standardmäßig ausgeschaltet.
[CMake-Beispiel](examples/integration/consumer/CMakeLists.txt),
[Benennung und Migration](docs/development/naming_and_migration.md).

Optionales Plotten der Gelenk- und TCP-Verläufe:

```sh
python -m pip install -r tools/requirements-optional.txt
python tools/visualization/plot_program_trace.py trace.csv --output trace.png
```

Der Plotter unterstützt PNG/SVG/PDF. Python ist für C++-Build, Import und
CSV-Simulation nicht erforderlich.

## Rekonstruierte Funktionen

- SI-Zustände und Gelenk-/TCP-Grenzen; analytische rest-to-rest PTP-Quintiken.
- Serielle Ketten mit 1–12 Achsen, FK/J, analytischen Ableitungen bis zum Ruck und
  lokalem, begrenztem numerischem IK-Solver; striktes URDF/SI-YAML-Modellformat.
- Nominale UR5e-, ABB IRB120- und FANUC LR Mate 200iD-Modelle aus belegten
  öffentlichen Quellen, mit Lizenzen, Commit-/Dateihashes und unabhängigen FK-Fixtures.
- SCARA- und Rotary-Delta-Referenzkinematik, Delta LIN/CIRC, rotierende serielle
  Posepfade, C2-Wegpunktblends und numerische Intervallhüllen vor Zeitplanung.
- Gleichzeitige Gelenk- und lineare/angulare TCP-v/a/Ruck-Grenzen; explizite
  TCP-Hindernishüllen. Konservative Zeitplanung, keine globale Zeitoptimierung.
- Zeitraster mit erhaltenen Stillstandspunkten, SPSC-Puffer, Zeit-/Datenprüfungen,
  unveränderliche Live-Pläne, Generationswechsel, Übergabegrenzen und atomare Snapshots.
- Optional Ruckig Community v0.15.3: bewegtes Retargeting und unabhängig integrierte,
  vollständig auf Gelenkextrema geprüfte Haltephasen sowie vorbereitete Übergänge.
- Kalibrierte statische Frames, Conveyor-Prognose, begrenzte Messhistorie und
  kontinuierliches XYZ-/Sechsachs-Tracking mit vorab geprüftem Halt bei Update-Ausfall.
- Starre Punktkalibrierung, Linux-Zeitdiagnostik und ein eigener YAML-Programmimport
  mit PTP/LIN/CIRC, benannten Punkten, Geschwindigkeitsfaktoren, WAIT und CALL.

## Build-Varianten

```sh
# Vollständiger Funktionsumfang mit Ruckig, ohne Hardwareverbindung:
cmake -S . -B build-all -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build-all --parallel 2
ctest --test-dir build-all --output-on-failure

# Minimaler Header-Kern ohne Modell-/Programmimport:
cmake -S . -B build-minimal -DMOTION_WITH_SERIAL=OFF

# Sanitizer:
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG="-O1 -g1" -DMOTION_WITH_RUCKIG=ON -DMOTION_SANITIZE=ON
```

`MOTION_WITH_SERIAL=ON` ist der Standard und aktiviert den Programmimport. Ruckig
ist für die normalen PTP/LIN/CIRC-Programme nicht erforderlich. Nur Stopps,
Online-Retargeting und Conveyor-Following benötigen `MOTION_WITH_RUCKIG=ON`.
Linux bietet zusätzlich `platform_timing_probe [Anzahl]` für Host-Zeitdiagnostik.
Der standardmäßig gebaute KDL-FK/J-Vergleich benötigt ebenfalls kein Netzwerk und
verlinkt KDL nur in den Test. `MOTION_WITH_KDL_REFERENCE=OFF` lässt ihn aus.
Parser-Fuzzing mit Clang, Replay und Seeds: [Anleitung](docs/testing/robustness.md).

## Grenzen der Wiederherstellung

CANopen bleibt auf ausdrücklichen Nutzerwunsch ausgeklammert. Die früher
beschriebenen vollständigen Link-/Selbstkollisions- und Payload-/Drehmomentmodule
sind hier noch nicht wiederhergestellt. Kein Programm behauptet diese Prüfungen
stillschweigend. Digitale Hardware-I/O, eingangsabhängige Wartebefehle,
Hersteller-/G-Code-Übersetzer und sensorabhängige Programmverzweigung waren auch
im zuletzt beschriebenen Programmformat nicht implementiert.

Die Intervall-/Polynomhüllen sind numerische Engineering-Prüfungen, keine formal
zertifizierte Intervallarithmetik. IK ist lokal. Die öffentlichen Modelle und
Beispielgrenzen ersetzen keine Robotermessung, Achs-/Werkzeugkalibrierung oder
unabhängige Maschinensicherheit. Eine geprüfte Sollbahn und ein virtueller
Dispatch-Test sind kein Nachweis physischer Ausführung oder harter Echtzeit.

## Dauerhafte Sicherung

Zielrepository: https://github.com/cpr-jar/Robotmotionplanning

Der fortgeführte Quellstand liegt auf `recovery/reconstructed-v0.12.1`.
[Quell-ZIP v0.12.3](https://github.com/cpr-jar/Robotmotionplanning/archive/refs/tags/v0.12.3.zip).
Die frühere Rekonstruktionssicherung v0.12.1 bleibt zusätzlich als Git-Bundle in
`backups/` erhalten; sie wird durch neue Versionen nicht überschrieben.
