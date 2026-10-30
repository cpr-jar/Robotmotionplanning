# Prüfnachweis: Komponenten und Schichten — 0.15.0

Stand: **30.10.2026**, Basis **0.14.0 / d5494c1**. Dieses Paket setzt F03 des
[Architektur-Reviews](../architecture/architecture_review_v0.13.0.md) um.
Auswahl und Migration: [Komponentenhandbuch](../architecture/components.md).

## Architektur und unabhängige Verbraucher

Die bisherigen Include-Modulzyklen `math ↔ kinematics ↔ planning` und
`execution ↔ program` sind aufgelöst. Gemeinsame Schnittstellen, Werte und
unveränderte Rechenhilfen liegen in niedrigeren Schichten.

Der konservative Check zählt bedingte Includes auf beiden Seiten: **11 Schichten,
17 CMake-Komponenten, 55 kanonische Bibliotheksdateien** einschließlich CPP-
Implementierungen und privatem Programmheader. Keine Datei-, Modul- oder
Komponentenzyklen. Die drei Runtime-Einstiege ziehen nur `core`, `contracts`
und `execution` ein. Produktionscode darf die zwei Vertragsfassaden und die
flachen Kompatibilitätsheader nicht zur Umgehung der Regeln verwenden.

Jeder Verbraucher ist ein separates CMake-Projekt mit expliziter Auswahl,
eigenem Buildverzeichnis und ausgeführtem C++-Programm. Die Prüfung misst die
**tatsächlich erzeugten Targets und CPP-Dateien über die CMake File API**,
die Archive und nicht angeforderte Drittbibliotheken.

| Fall | Motion-CPP | Motion-Archive | Fachliche Prüfung |
| --- | ---: | ---: | --- |
| execution | 0 | 0 | Eigener Schedule, Session, Tickfolge und Shutdown |
| planning | 0 | 0 | Kartesischer Gantry-Spline und Zeitplanung |
| serial_kinematics | 1 | 1 | FK, Jacobian und lokale IK ohne Parser |
| serial_planning | 1 | 1 | Planung kombiniert mit explizitem Serial-Ziel |
| differential_kinematics | 1 | 1 | Inverse SCARA-Differentialrechnung ohne Serial |
| calibration | 1 | 1 | Starrer Fit ohne Parser/seriellen Solver |
| dxf_import | 1 | 1 | POINT-DXF, Millimeter nach SI |
| model_import | 2 | 2 | Reales UR5e-URDF/Profil und FK |
| program | 8 | 4 | UR5e-Programm, Simulation, Stillstandsgrenzen |
| serial | 10 | 7 | Legacy-Ziel und echtes Sammelarchiv mit zehn Implementierungsobjekten |
| following | 1 | 1 | SCARA-Following samt Halt ohne Serial und Parser |

Alle **11/11** Fälle bestanden. Die Zahlen betreffen eigene Motion-Teile,
nicht die Standardbibliothek oder vendortes Ruckig/YAML/XML. Im Legacy-Fall
zählt das wiederverwendende Sammelarchiv zusätzlich zu sechs Komponentenarchiven.
Execution, Planning und DXF konfigurieren keine Eigen-/YAML-/XML-/Ruckig-Quellen.
Die vorhandene Core-Gantryreferenz bleibt aus Kompatibilitätsgründen erhalten;
ein neues generisches Modell-/Enclosure-Pluginmodell aus F04 wird nicht behauptet.

## Ausgeführte Matrix

Linux/x86-64, GCC 14.2.0, CMake 4.4.4, C++20, strikte Warnungen und aktive
Assertions. Gesamtbuilds wurden neu konfiguriert/gebaut, die unabhängigen
Verbraucher separat konfiguriert.

| Prüfung | Ergebnis |
| --- | --- |
| Legacy Serial/Ruckig/KDL, Release | **32/32 CTests** |
| Dieselbe Vollkonfiguration, Debug -O1 -g1, ASan/UBSan | **32/32 CTests** |
| Serial ohne Ruckig, Release | **30/30 CTests** |
| Minimal ohne Serial/Ruckig, Release | **12/12 CTests** |
| Ruckig ohne Serial, Release | **14/14 CTests** |
| Unabhängige Komponenten-Verbraucher | **11/11** |
| Direkte externe Execution-/DXF-Targets mit ASan/UBSan | **2/2**, einschließlich Sanitizerweitergabe an Parent-Consumer |
| Bisheriger Fassade-Consumer | Serial-Standard und Minimal-ASan/UBSan bestanden |
| Öffentliche Header einzeln | **46/46**, davon 44 kanonische Aufgabenheader und zwei Vertragsfassaden; 34 flache Header zusätzlich im Kompatibilitätstest |
| Schichtenschutz-Fixtures | **6/6**, einschließlich inaktiver Rückwärtskante, Datei-/CMake-Zyklus, Fassadenumgehung und transitiver Runtime-Abhängigkeit |
| Session unter Clang-19-ThreadSanitizer | Lifecycle, 200 Wechsel und 200 Angebot-/Fehlerrennen bestanden |
| Getrennter Viewer/Editor | **32/32 Python-Tests** mit selektivem Program-Build |
| Clang-19-libFuzzer nach neuer Objekt-/Archivaufteilung | Beide Harnesses gebaut, je **1000 Läufe** in temporären Korpuskopien bestanden |

ASan verwendet detect_leaks=0, UBSan/TSan halt_on_error=1. Der fokussierte TSan-
Test ist keine Abdeckung aller Solver/Parser. Die 1000 Fuzz-Läufe sind ein Build-/
Instrumentierungs-Smoke, keine lange Kampagne. CI behält die bestehenden
50.000-Lauf-Jobs und ergänzt elf Komponenten-Verbraucher und Schichtenchecks.
Lokale Ergebnisse sind kein behaupteter bereits abgeschlossener GitHub-CI-Lauf.

## Bewegungsdaten und Reproduktion

Alle sechs vollständigen Exporte haben identische SHA-256 wie 0.14.0:
UR5e bei 1/10/20 ms, ABB/FANUC/DXF bei 10 ms. Das umfasst JSON einschließlich
CSV-Spur und Diagnosedaten. KDL-Vergleiche und Grenzfälle bleiben in den Gesamtsuiten.
[Graph, Komponentenmessungen, Exporthashes und Laufprotokolle](evidence/components_v0.15.0/README.md).

```sh
python3 tools/architecture/check_layers.py
python3 -m unittest discover -s tools/architecture/tests -v
python3 tools/architecture/check_components.py
python3 tools/architecture/check_components.py --case execution --case dxf_import --sanitize --build-root build-components-asan
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

Mit --cmake /pfad/zu/cmake lässt sich ein eigener CMake-Pfad wählen, mit
--case serial_planning ein einzelner Fall. Consumerquellen liegen unter
`tests/integration/components/`, Prüfwerkzeuge getrennt unter `tools/architecture/`.

Pi-/ARM-/Bus-/Drive-Abnahme, installierbare Exportpakete, Gesamt-RAM-/Abbruchbudgets
und eine Anlagenpolicy für Modell-/Kalibrierwechsel sind weiterhin eigene Aufgaben.
Die Komponentenwahl ist kein Dateisystem-Sandboxing gegen manuell falsche Includes.
