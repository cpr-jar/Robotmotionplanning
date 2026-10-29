# Eigene Roboterprogramme importieren

Programmformat v1 aus dem Gespräch, neu implementiert und geprüft. Der Import
benötigt den Standard-Build mit `MOTION_WITH_SERIAL=ON`, keinen Ruckig- oder
CANopen-Anschluss.

Ab 0.13.0 kann ein einzelner **`DXF`-Befehl** zusätzlich eine externe Zeichnung
einlesen und mit der vorhandenen Bewegungsplanung abfahren. Beispiele, Einheiten,
Platzierung und Grenzen stehen unter [DXF als Programmbefehl](dxf_program_command.md).

## Vollständiges kleines Beispiel

Speichere diese Datei unter `programs/mein_programm.yaml`:

```yaml
schema_version: 1
units: SI
frame: world
robot_profile: ../models/ur5e/profile.yaml
cycle_s: 0.01
speed: 0.75
start: home
points:
  home: {joints: [0.2, -0.6, 0.7, 0.3, 0.8, -0.2]}
  approach: {joints: [0.21, -0.6, 0.7, 0.3, 0.8, -0.2]}
subprograms:
  pause:
    - {type: WAIT, seconds: 0.05}
steps:
  - {type: PTP, target: approach}
  - {type: CALL, program: pause}
  - {type: PTP, target: home, speed: 0.5}
```

```sh
./build/program_simulator programs/mein_programm.yaml --check
./build/program_simulator programs/mein_programm.yaml --csv trace.csv
python tools/visualization/plot_program_trace.py trace.csv --output trace.png
```

Die drei gelieferten `*_demo.yaml`-Dateien enthalten zusätzlich LIN und CIRC.
Bei `--check` wird bereits **das gesamte Programm geplant und zugelassen**;
lediglich die zyklische Wiedergabe entfällt. Ohne `--check` folgt die vollständige
Simulation mit virtueller Uhr durch Producer und Dispatcher.

## Koordinaten, Modelle und Punkte

- `units: SI` bedeutet Meter, Radiant und Sekunden; keine implizite mm-/Grad-Umrechnung.
- Posen stehen im Modell-Weltkoordinatensystem. Nur `frame: world` ist zulässig.
- Quaternionen haben die Reihenfolge **[w,x,y,z]** und müssen normiert sein.
- `robot_profile` und optional `robot_urdf` werden relativ zur **Programmdatei**
  aufgelöst, unabhängig vom Arbeitsverzeichnis. Ohne `robot_urdf` wird `robot.urdf`
  neben dem Profil geladen. Es wird kein Xacro ausgeführt oder Fremdcode geladen.
- `start` ist ein benannter Gelenkpunkt oder `{joints: [...]}`. Diese Konfiguration
  wird bei Simulationsbeginn als Stillstand vorausgesetzt; sie ist kein Fahrbefehl.
- Ein Punkt ist entweder `{joints: [...]}` oder
  `{position: [x,y,z], orientation: [w,x,y,z]}`. Mischformen sind ungültig.
- Gelenkwerte stehen in der importierten Kettenreihenfolge. Prismatic: m; Rotary: rad.

`target` akzeptiert den Namen eines Punkts oder die jeweilige Punktbeschreibung
als Inline-Mapping. PTP erlaubt alternativ die Kurzform `joints: [...]` direkt im
Schritt. LIN verlangt `target`.

## Befehle

| Typ | Felder | Bedeutung |
| --- | --- | --- |
| PTP | `target` **oder** `joints`, optional `speed` | Rest-to-rest-Gelenkquintik. Posen werden vorher über lokalen IK gelöst. Gelenk- und nichtlineare TCP-v/a/Ruck-Grenzen werden über die ganze Bahn geprüft; keine TCP-Geraden-Garantie. |
| LIN | `target`, optional `speed` | Kartesische Linie mit glatter kürzester Quaternion-Drehung. Ein Gelenkpunkt liefert seine FK-Pose, keine vorgeschriebene terminale Gelenkbranche. |
| CIRC | `center` **oder** `center_offset`, `normal`, `sweep_rad`, optional `orientation`, `speed` | Vorzeichenbehafteter Rechtshandbogen im Weltkoordinatensystem ab der aktuellen TCP-Pose. Betrag des Winkels höchstens 2pi. |
| WAIT | positive `seconds` | Stillstand mindestens für die angegebene Zeit. |
| CALL | `program` | Vorab-Auflösen eines internen benannten Unterprogramms. Kein externer Skriptaufruf. |

Beispiel eines CIRC-Schritts:

```yaml
- type: CIRC
  center_offset: [-0.002, 0, 0]
  normal: [0, 0, 1]
  sweep_rad: 0.5
  speed: 0.5
```

`center` ist ein absoluter Weltpunkt. `center_offset` wird als weltorientierter
Vektor zur tatsächlichen Modell-FK am Anfang des Schritts addiert, **nicht** mit
dem Werkzeug gedreht. Genau eine Darstellung ist erforderlich. Radius muss
positiv und Normale senkrecht zum Startradius sein. Ein absoluter Mittelpunkt
kann wegen verbleibender IK-/Spline-Residuen am vorausgehenden Schritt diese
strenge Bedingung verletzen; Geometrie wird nicht stillschweigend projiziert.
Ohne `orientation` bleibt die aktuelle Orientierung erhalten.

Globales und schrittweises `speed` werden multipliziert. Der effektive Wert muss
in [0.001,1] liegen. Er reduziert Geschwindigkeitsgrenzen mit s, Beschleunigung
mit s² und Ruck mit s³, in Gelenk- und TCP-Domäne. Er kann keine Profilgrenze erhöhen.
WAIT-Zeiten ändern sich dadurch nicht. Das ist keine Laufzeit-Geschwindigkeitsänderung.

## Vorabprüfung und Ausführung

Alle Befehle werden vor Veröffentlichung des Programms geprüft. PTP verwendet
Gelenkpolynomhüllen plus nichtlineare FK-Intervalljets; bei zu hohen TCP-Werten
werden innerhalb eines festen Budgets längere Dauern versucht. LIN/CIRC werden
über C2-Gelenksplines im Pose-Toleranzschlauch abgebildet und konservativ zeitgeplant.
Standardtoleranzen: 0.2 mm und 0.5 mrad relativ zum nominalen Modell. Dies sind
numerisch begrenzte Approximationen, keine exakten analytischen Sechsachs-LIN/CIRC.

Alle Schrittgrenzen sind Stillstandspunkte. Segmente werden nur verlängert auf
ganze Zyklen gelegt; WAIT wird nie verkürzt. Binäre Rundung kann an einer Grenze
einen zusätzlichen Zyklus verursachen. Es wird kein implizites Blending aktiviert.
Die Bibliothek bietet BlendedPosePath gesondert an.

Fehler nennen Datei, Zeile/Spalte und Block/Schritt, soweit vorhanden. Die
CLI listet die Zuordnung expandierter Segmente zu ihren Definitionen auf.
Ein Fehler in einem späten Schritt verhindert die Veröffentlichung des ganzen
Programms. Eine vorhandene CSV bleibt bei Ablehnung/fehlgeschlagener Simulation
erhalten: Die CLI schreibt zunächst eine temporäre Datei und benennt erst nach
Erfolg um.

## Formate und Budgets

Genau ein YAML-Dokument, maximal 1 MiB; 50.000 YAML-Ereignisse, Tiefe 32;
4.096 Punkte, 256 Unterprogramme, 10.000 Quellbefehle und 10.000 expandierte Befehle;
Unterprogrammtiefe 16. YAML-Aliase, explizite Tags, doppelte/unbekannte Felder,
rekursive Aufrufe, nichtendliche Werte und nicht unterstützte Instruktionen
werden abgelehnt. Auch unbenutzte Definitionen werden syntaktisch und bezüglich
Aufrufreferenzen geprüft. Kinematische Zulassung betrifft das expandierte Hauptprogramm.

WAIT höchstens eine Stunde; Referenzprogramm höchstens 24 Stunden.
PTP-Zeitverlängerung: maximal 16 Versuche, jeweils höchstens eine Stunde.
`cycle_s`: 0.0001..1 s, Standard 0.01; `epoch_s`: Standard 0.
Das Zeitraster muss ausreichende numerische Auflösung besitzen.
Simulation: Standard maximal 1.000.000 Ticks, mit `--max-ticks Anzahl` explizit
bis zu 10.000.000. `--check` ist auch für längere geprüfte Programme möglich.
Eine konservative Ablehnung beweist keine globale Unlösbarkeit.

## C++-Schnittstelle und CSV

```cpp
#include "motion/robot_program.hpp"
const auto prepared = motion::load_robot_program("programs/examples/ur5e_demo.yaml");
const auto report = motion::simulate_robot_program(prepared);
```

Alternativ: `parse_robot_program_yaml(text, name)` und
`compile_robot_program(source, imported_robot)`. Der Programmquellwert ist über
seine öffentliche Schnittstelle unveränderlich. Das Ergebnis enthält einen
IJointProgram-kompatiblen Referenzplan und das CycleAlignedProgram.
Planen, Samplen, CSV und Simulation allozieren; sie gehören nicht in den RT-Thread.

CSV: Tick, relative `time_s`, absolute `timestamp_s`, expandiertes Segment (ab 1),
lokaler Quellschritt, q/dq/ddq/Ruck je Gelenk, tatsächliche Modell-FK mit
TCP-Position/Quaternion, lineare/angulare TCP-v/a/Ruck-Komponenten. Dezimalpunkt
und 17-stellige Präzision sind locale-unabhängig. Der C++-Simulator stellt die
Streamformatierung wieder her. Berichts-/Plot-Maxima sind Sample-Diagnostik;
die kontinuierliche Zulassung erfolgt vorher.

Der optionale Matplotlib-Plotter erzeugt PNG/SVG/PDF und akzeptiert bis zu
100.000 endliche, streng zeitlich geordnete Samples. Es handelt sich um
Sollreferenzen, keine Hardwaremessungen. Digitale I/O, eingangsabhängige WAITs,
Hersteller-/G-Code-Import und sensorabhängige Programmverzweigungen sind nicht
Bestandteil dieses Formats.
