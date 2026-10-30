# LIN mit konstanter TCP-Orientierung — 0.16.1

Stand: **30.10.2026**. Bibliothek 0.16.1, Werkzeug 0.5.1.
[Vertrag und Migration](../formats/robot_program_yaml.md#lin-und-tcp-orientierung).

## Ursache und Korrektur

Der Programmcompiler hat bei LIN bisher automatisch die Orientierung des
Zielpunkts in einen PoseSweepPath übernommen. Im gemeldeten UR5e-Programm wurde
die erste Achse von home auf **2 rad** geändert; das Quaternion von work stammt
weiterhin aus der ursprünglichen **0,2-rad**-Konfiguration. Der bisherige
Interpolationsmodus dreht das TCP auf der Linie deshalb um rund **103,13°**.
Die Orientierung stammt aus der nativen FK-Spur; der Browser stellt diese Daten
korrekt dar. Eine Korrektur nur der Zeichnung würde die Ursache verdecken.

Jetzt gilt für normale LIN-Programmschritte `orientation_mode: keep` als Default.
Der Compiler verwendet die tatsächliche TCP-Orientierung am Anfang des jeweiligen
Segments. `orientation_mode: interpolate` erhält bewusst angeforderte Drehungen.
Das gilt auch für benannte Gelenkpunkte, deren FK-Zielposition angefahren wird.

Explizite DXF-Orientierungen werden weiterhin beim Eintritt über die dafür
markierte Travel-Bewegung eingestellt; die eigentlichen LIN-Konturen halten sie.
PTP, CIRC, Low-Level-PoseSweepPath und der zyklische Dispatcher bleiben in ihrem
bisherigen Vertrag. Der Export nennt den LIN-Modus, der Viewer zeigt ihn an.

## Gemeldeter Zielpunkt und erreichbares Beispiel

Mit unverändertem Ziel `[0.7444144539141021, 0.3622280684167766,
0.24366518271764495]` findet der aktuelle Solver ab home mit Achse 1 bei 2 rad
**keine zugelassene LIN-Bahn bei gehaltener Orientierung**. Der Compiler meldet
dies am LIN-Schritt. Es wird keine Drehung als Ersatzweg verwendet. Das ist kein
Beweis globaler geometrischer Unerreichbarkeit über alle möglichen IK-Verzweigungen.

Ein freigegebenes Beispiel mit demselben home, den beiden PTP-Anfahrbewegungen,
einem nahen LIN-Ziel, Vollkreis, WAIT und Rückfahrt steht in
[`ur5e_lin_keep_orientation.yaml`](../../programs/examples/ur5e_lin_keep_orientation.yaml).
Sein Arbeitspunkt ist:

```yaml
position: [-0.47390467854451826, 0.62398337528833302, 0.24366518271764495]
```

Das entspricht vom neuen home aus 30 mm in Welt-X und 10 mm in Welt-Y.
Das Profil `profile_pick_place_simulation.yaml` wurde aus dem Projekt übernommen;
für diese Korrektur wurden dessen Grenzen nicht verändert.

Im Editor: neu starten, **„UR5e · LIN mit konstanter Orientierung“** wählen und
simulieren. Bestehende HTML-Dateien sind unveränderliche alte Schnappschüsse;
für den geänderten Vertrag ist eine neue Simulation erforderlich.

## Regressionen und Grenzen

Die dedizierte C++-Prüfung deckt ab:

- Default und explizites keep liefern dieselben Gelenkverläufe, obwohl das
  Zielquaternion eine stark abweichende Orientierung beschreibt.
- Die nominale TCP-Gerade und die FK-Orientierung werden entlang der Bahn geprüft.
- LIN nach PTP und benannte Gelenkpunkte beziehen sich auf die tatsächliche
  Segmentanfangspose.
- Bei 1/10/20 ms werden alle LIN-/CIRC-Sollwerte auf Orientierung und Gelenkgrenzen
  geprüft und mit der echten virtuellen Ausführung simuliert.
- Explizite Interpolation erreicht die gewünschte Drehung; unbekannte Modi und
  orientation_mode an PTP werden mit Quellort abgelehnt.
- Der gemeldete weit entfernte Zielpunkt wird im konstanten Modus abgelehnt.
- DXF-Eintritt mit vorgegebener Orientierung und anschließende konstante Kontur
  bleiben zulässig.

Werkzeugprüfungen lesen jedes LIN-Quaternion aus der nativen CSV-Spur, prüfen die
Modusmetadaten und den Editor-Fehlerpfad. Der echte Browsertest lädt das neue
Beispiel, prüft die Beschriftung und berechnet die TCP-Rotationsmatrizen erneut
mit der JavaScript-FK aus den tatsächlich dargestellten Gelenkwerten.

Gemessen am nahen Beispiel, 10 ms: maximale LIN-Orientierungsänderung rund
**0,00000033°** bei 98 LIN-Ticks. Das ist ein numerischer Messwert dieses Beispiels,
keine allgemeine Exaktheitsgarantie. Die vorhandene Zulassungstoleranz beträgt
weiterhin 0,2 mm Position und 0,5 mrad Orientierung. Physische Genauigkeit,
Kalibrierung, Kollisionen und Antriebsausgabe wurden nicht geprüft.

## Ausgeführte Prüfungen

Linux/x86-64, GCC 14.2.0, CMake 4.4.4, aktive Assertions. Die Kernkonfiguration
enthält Serial, Ruckig und den unabhängigen KDL-Vergleich.

| Prüfung | Ergebnis |
| --- | --- |
| Release | 35/35 CTests |
| Debug -O1 -g1, ASan/UBSan | 35/35 CTests, detect_leaks=0 / halt_on_error=1 |
| Gesamte Werkzeugtests | 41/41 Python-Tests einschließlich 100k-Spurprüfung |
| Abschließende Editor-/LIN-Prüfungen | 15/15 und 2/2 nach Ergänzung der neuen Beispielauswahl |
| Echte Chromium-Abläufe | 3/3; Browser-FK-Matrixabweichung auf LIN höchstens 5,61·10⁻⁹ im 2-rad-Beispiel |
| Bestehendes Pick-and-place-Programm | 16 Segmente, 4.629 Befehle, 46,28 s, keine Sample-Grenzverletzungen |
| Öffentliche Header | 47/47 einzeln kompiliert, Serial/Ruckig |
| Schichtengraph | 11 Schichten, 17 Komponenten, 56 kanonische Dateien; azyklisch |

Der Viewer prüft weiterhin die tatsächliche FK gegen die exportierten Samples.
Die neue Beschriftung ersetzt keine Bewegung durch eine kosmetische Darstellung.

[Ausführungsprotokolle](evidence/lin_orientation_v0.16.1/README.md).

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/program_simulator programs/examples/ur5e_lin_keep_orientation.yaml --check
python3 tools/motion_viewer/run_editor.py --open
```
