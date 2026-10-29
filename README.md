# Robot Motion C++ — 0.12.1, rekonstruierter Stand

Dieses Projekt wurde am 29.10.2026 aus der erneut hochgeladenen ZIP **v0.2.0** und
dem Gespräch rekonstruiert. Es ist eine neu geprüfte Implementierung, keine
byteidentische Wiederherstellung der verlorenen v0.12.0.
[Wiederherstellungsbericht](docs/RECOVERY_STATUS.md) · [Programmformat](docs/ROBOT_PROGRAMS.md)
· [Architektur](docs/ARCHITECTURE.md) · [Prüfbericht](docs/VALIDATION.md).

## Starten

C++20, CMake ab 3.20 und ein C++-Compiler werden benötigt. Die benötigten
C++-Abhängigkeiten liegen als versions- und hashgebundene Quellarchive bei;
**auch der optionale Ruckig-Build benötigt keinen Internetzugang**.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/program_simulator programs/ur5e_demo.yaml --check
./build/program_simulator programs/ur5e_demo.yaml --csv trace.csv
```

Weitere vollständige Beispiele:
`programs/abb_irb120_demo.yaml` und `programs/fanuc_lrmate200id_demo.yaml`.
Kopiere eine passende Beispieldatei und passe Punkte und Schritte an.
Der Simulator verbindet sich mit keiner Hardware.

Optionales Plotten der Gelenk- und TCP-Verläufe:

```sh
python -m pip install -r tools/requirements-optional.txt
python tools/plot_program_trace.py trace.csv --output trace.png
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
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DMOTION_WITH_RUCKIG=ON -DMOTION_SANITIZE=ON
```

`MOTION_WITH_SERIAL=ON` ist der Standard und aktiviert den Programmimport. Ruckig
ist für die normalen PTP/LIN/CIRC-Programme nicht erforderlich. Nur Stopps,
Online-Retargeting und Conveyor-Following benötigen `MOTION_WITH_RUCKIG=ON`.
Linux bietet zusätzlich `platform_timing_probe [Anzahl]` für Host-Zeitdiagnostik.

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

Der rekonstruierte Quellstand liegt auf `recovery/reconstructed-v0.12.1`.
Ein Quell-ZIP und eine vollständige Git-Bundle-Sicherung werden zusätzlich
bereitgestellt. Nur bestätigte Pushes/Deployments werden als erfolgreich angegeben.
