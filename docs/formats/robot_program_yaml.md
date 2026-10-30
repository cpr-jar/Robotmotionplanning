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

### Pick-and-Place-Beispiel für UR5e

`programs/examples/ur5e_pick_place.yaml` verwendet das vorhandene nominale
UR5e-Modell, nicht den älteren UR5. Vorgesehen sind zwei Stationen mit
senkrechtem Absenken und Anheben, kurzen WAITs als Greifer-Platzhalter und
weitem PTP-Transfer mit 180 Grad Werkzeugdrehung bei globalem `speed: 1.0`.
Alle Schrittgrenzen bleiben
Stillstandspunkte. Es werden weder Greifer-I/O noch Werkstücke simuliert.

Die Datei muss mit dem nativen Exporter vollständig planbar sein und darf
keine Sample-Profilverletzungen zeigen. Mindestens eine Sample-Spitze für
Geschwindigkeit, Beschleunigung oder Ruck soll mindestens 90 % einer
Profilgrenze erreichen. Das ist ein Belastungsbeispiel für Simulationsannahmen,
keine Herstellerfreigabe, Nutzlastprüfung oder Kollisionsprüfung.

Das Beispiel nutzt das separate `models/ur5e/profile_pick_place_simulation.yaml`.
Das konservative `models/ur5e/profile.yaml` bleibt unverändert. Für das neue Profil
gelten folgende vor der Simulation festgelegte Annahmen:

| Bereich | Geschwindigkeit | Beschleunigung | Ruck |
| --- | ---: | ---: | ---: |
| Jede Gelenkachse | 3,141592653589793 rad/s | 12 rad/s² | 60 rad/s³ |
| TCP linear | 1,0 m/s | 2 m/s² | 10 m/s³ |
| TCP rotatorisch | 3,141592653589793 rad/s | 12 rad/s² | 60 rad/s³ |

Die Gelenkgeschwindigkeit entspricht den 180 Grad/s aus
[UR-Forumsbeitrag 4](https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338/4).
Die Beschleunigungen liegen unter den Empfehlungen für moveJ und moveL aus
[Beitrag 2](https://forum.universal-robots.com/t/maximum-axis-speed-acceleration/13338/2),
sind aber keine für jede Pose und Nutzlast garantierten Grenzen. Der angenommene
Ruck ist Beschleunigung geteilt durch 0,2 s Anstiegszeit, kein Herstellerwert.
Der [Ruck-Thread](https://forum.universal-robots.com/t/how-to-apply-a-forth-order-polynomial-to-limit-jerk/3708/4)
liefert dafür keine numerischen Herstellergrenzen. Die lineare TCP-Geschwindigkeit
von 1,0 m/s ist eine zusätzliche Simulationsannahme, kein durch diese Beiträge
belegter Herstellergrenzwert. Auch die rotatorischen TCP-Grenzen sind zusätzliche
Simulationsannahmen: 3,141592653589793 rad/s, 12 rad/s² und 60 rad/s³.
Die rotatorische Ruckannahme folgt ebenfalls aus 0,2 s Beschleunigungsanstieg.
Sie sind keine abgeleiteten oder belegten Herstellergrenzen für die TCP-Drehung.
Nur dieses separate Profil wird angepasst; Bahn, Wartezeiten, Gelenkgrenzen
und lineare TCP-Grenzen bleiben unverändert. Die Änderung muss nativ und über
den Editor ohne Profilverletzungen planbar sein; Zykluszeit und Sample-Spitzen
werden erneut gemessen statt aus den Grenzwerten vorhergesagt.

CLI-Modellpfade sind relativ zur Programmdatei. Beim Import in den Editor
`robot_profile` auf `models/ur5e/profile_pick_place_simulation.yaml` setzen und
erneut simulieren.

Aktueller Prüfstand mit den erhöhten rotatorischen TCP-Grenzen vom 30.10.2026:
zwei Zyklen, 16 Segmente und 1.581 Sollwerte bei 10 ms ergeben 15,80 s
Gesamtdauer statt 46,28 s. Jeder der vier PTP-Transfers dauert 1,88 s statt
9,5 s. Das isolierte Anheben der rotatorischen TCP-Grenzen verkürzt damit
die Transferzeit; Bahn und übrige Profilwerte wurden nicht verändert.
Der PTP-Planer verlängert abgelehnte Zeitkandidaten jeweils um Faktor 1,5;
Sample-Spitzen müssen daher nicht genau an jeder kontinuierlich geprüften
TCP-Grenze liegen.

| Größe | Anteil an der aktuellen Profilgrenze |
| --- | ---: |
| Gelenkgeschwindigkeit | 99,7 % |
| Gelenkbeschleunigung | 42,8 % |
| Gelenkruck | 47,3 % |
| Lineare TCP-Geschwindigkeit | 81,6 % (0,8158 m/s) |
| Lineare TCP-Beschleunigung | 67,8 % |
| Linearer TCP-Ruck | 89,5 % |

Native Planung und Editor-HTML-Veröffentlichung waren erfolgreich, ohne
Sample-Profilverletzungen. Die acht senkrechten 120-mm-Bewegungen,
Rückkehr zur Startkonfiguration und terminaler Stillstand sind erneut geprüft.
Das Belastungsziel von mindestens 90 % ist mit der Gelenkgeschwindigkeit erfüllt.
Dies sind weiterhin Simulationsergebnisse ohne Kollisions- oder Nutzlastfreigabe.

Vergleichsstand mit 1,0 m/s linearer TCP-Geschwindigkeitsgrenze und noch
konservativen rotatorischen TCP-Grenzen vom
30.10.2026: native Planung und Editor-HTML-Veröffentlichung erfolgreich,
keine Sample-Profilverletzungen. Die acht senkrechten 120-mm-Bewegungen,
Rückkehr zur Startkonfiguration und terminaler Stillstand sind geprüft.
Die Gesamtdauer bleibt bei 46,28 s und 4.629 Sollwerten bei 10 ms.
Die tatsächliche lineare TCP-Spitzengeschwindigkeit bleibt 0,2419 m/s,
nun 24,2 % der erhöhten Grenze. Die übrigen Sample-Spitzen bleiben gegenüber
dem folgenden Vergleichsstand unverändert. Die vier PTP-Transfers dauern
weiterhin jeweils 9,5 s; eine höhere Geschwindigkeitsgrenze allein erzwingt
keine kürzere Planung. Beschleunigung, Ruck, rotatorische TCP-Grenzen und
die Bahn bleiben unverändert. Das ursprüngliche Belastungsziel von mindestens
90 % wird mit maximal 89,5 % Sample-Grenzausnutzung in diesem Vergleichsstand knapp verfehlt;
dafür werden keine weiteren Grenzen ohne neue Annahmen verändert.

Vergleichsstand mit dem Annahmenprofil und noch 0,25 m/s TCP-Geschwindigkeit
vom 30.10.2026: dieselben zwei
Zyklen, 16 Segmente und acht senkrechte 120-mm-Bewegungen ergeben 4.629 Sollwerte
bei 10 ms und 46,28 s Gesamtdauer (vorher 60,72 s). Der native Export und die
HTML-Veröffentlichung im Editor waren erfolgreich. Alle Samples bleiben innerhalb
der Profilgrenzen; Start- und Endkonfiguration stimmen überein, der Endzustand
ist in Ruhe. Die Sample-Spitzen relativ zum neuen Profil sind:

| Größe | Anteil an der Profilgrenze |
| --- | ---: |
| Gelenkgeschwindigkeit | 19,7 % |
| Gelenkbeschleunigung | 15,8 % |
| Gelenkruck | 35,5 % |
| Lineare TCP-Geschwindigkeit | 96,8 % |
| Lineare TCP-Beschleunigung | 40,1 % |
| Linearer TCP-Ruck | 89,5 % |

Die unveränderten TCP-Grenzen begrenzen diese Bahn weiterhin. Die höheren
Gelenkgrenzen bedeuten daher nicht, dass die Bahn diese auch ausschöpft.

Vergleichsstand mit dem bisherigen konservativen Profil vom 30.10.2026:
zwei Zyklen mit je 120 mm vertikalem
Hub, 16 expandierte Segmente, 6.073 Sollwerte bei 10 ms und 60,72 s Gesamtdauer.
Der native Export und die HTML-Veröffentlichung durch den Editor waren erfolgreich.
Die vollständige Sample-Spur enthält keine Profilverletzungen. Ihre Maxima sind:

| Größe | Sample-Spitze | Anteil an der Profilgrenze |
| --- | ---: | ---: |
| Gelenkgeschwindigkeit | 0,4996 rad/s | 99,9 % |
| Gelenkbeschleunigung | 0,6469 rad/s² | 64,7 % |
| Gelenkruck | 4,2586 rad/s³ | 85,2 % |
| Lineare TCP-Beschleunigung | 0,2741 m/s² | 54,8 % |
| Linearer TCP-Ruck | 1,7914 m/s³ | 89,6 % |

Sample-Spitzen ersetzen nicht die kontinuierliche Zulassung des nativen Planers
und sind keine Messungen an einem physischen Roboter.

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
