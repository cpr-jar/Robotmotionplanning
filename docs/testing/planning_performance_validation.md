# Beschleunigte Vorbereitung — 0.16.2

Stand: **30.10.2026**. Bibliothek 0.16.2, Testwerkzeug 0.5.2.
[Verhalten und Online-Ausbau](../integration/planning_performance.md).

## Messung und Ergebnis

Untersucht wurde das vorhandene UR5e-Pick-and-Place-Programm einschließlich
`profile_pick_place_simulation.yaml`. Vergleichsbasis ist der unveränderte
Commit `0dd8e02` / **v0.16.1**. Zwei statisch gebundene Release-Probes mit derselben
Messlogik wurden auf demselben Linux-x86_64-Host abwechselnd je dreimal ausgeführt.
Zeitmessung über `steady_clock`, ohne Sanitizer und nach Ende der nebenläufigen
Build-/Browsertests. Das ist eine Hostmessung, kein Nachweis auf dem Anwender-PC
oder Raspberry Pi.

| Programm, 10 ms | Bewegungsdauer | Planung vorher, Median | Planung jetzt, Median | Beschleunigung |
| --- | ---: | ---: | ---: | ---: |
| UR5e Pick-and-Place | 46,28 s | 7,269 s | 1,109 s | 6,55× |
| UR5e LIN mit konstanter Orientierung + Vollkreis | 25,46 s | 1,218 s | 0,188 s | 6,47× |

Die Vergleichsprobe trennt Import, Planung/Zulassung mit Rasterung und
Simulation/CSV. Der Import liegt hier bei etwa 1 ms. Die Simulation/CSV des
Pick-and-Place-Beispiels braucht bei 10 ms ungefähr 0,13 s; sie ist nicht die
Ursache der ursprünglichen mehrsekündigen Planungszeit.

Bei Pick-and-Place bleiben **18.900 Arbeitszähler-Einheiten und 4.629 Sollwerte**
erhalten, beim LIN/Kreis-Beispiel **3.297 Einheiten und 2.547 Sollwerte**. Die
Beschleunigung entsteht somit durch weniger Rechenaufwand innerhalb der
Prüfschritte, nicht durch das Weglassen dieser Schritte. Toleranzen und Limits
wurden nicht verändert.

Die CSV-Spuren sind innerhalb einer Version über alle Wiederholungen bytegleich.
Zwischen den Versionen sind sie wegen anderer Rundungsreihenfolge nicht bytegleich;
der Vergleich jeder numerischen Spalte findet höchstens etwa **2,31e-14**
absolute Abweichung beim Pick-and-Place-Beispiel beziehungsweise **1,20e-13**
beim LIN/Kreis-Beispiel, jeweils in der SI-Einheit des Kanals. Ticknummern,
Segment-/Quellzuordnung, Dauer und Sollwertzahl stimmen überein.

## Produktmessung bei 1/10/20 ms

Anschließend wurde der neue, unveränderte Exporter mit dem mitgelieferten
`benchmark.py` je Fall dreimal ausgeführt. Die Phasenmetadaten im Produkt zählen
`planning_s` einschließlich Import und gegebenenfalls Retiming.

| Programm | Raster | Planung, Median | Simulation/CSV, Median | Bewegungsdauer |
| --- | ---: | ---: | ---: | ---: |
| Pick-and-Place | 1 ms | 1,107 s | 1,273 s | 46,176 s |
| Pick-and-Place | 10 ms | 1,111 s | 0,128 s | 46,280 s |
| Pick-and-Place | 20 ms | 1,109 s | 0,064 s | 46,400 s |
| LIN + Vollkreis | 1 ms | 0,189 s | 0,753 s | 25,438 s |
| LIN + Vollkreis | 10 ms | 0,188 s | 0,077 s | 25,460 s |
| LIN + Vollkreis | 20 ms | 0,187 s | 0,038 s | 25,480 s |

Der geometrische Planungsaufwand ändert sich hier kaum mit dem Ausgaberaster.
Die Simulation erzeugt bei 1 ms etwa zehnmal so viele CSV-Zeilen wie bei 10 ms
und braucht entsprechend mehr Zeit. Unterschiedliche Bewegungsdauern entstehen
durch das segmentweise Aufrunden auf ganze Zyklusticks.

## Regressionsnachweise

- **36/36 Release-C++-Tests** einschließlich KDL-FK/J-Vergleich, Singularitäten,
  Gelenkgrenzen, LIN-Orientierung, Vollkreisen, DXF, Ausführungsfehlern und Budgets.
- **36/36 ASan-/UBSan-C++-Tests** im bestehenden Debug/-O1-Build, Leakprüfung
  deaktiviert wie beim vorherigen Nachweis; keine Sanitizer-Meldung.
- Neuer `interval_enclosures_tests`: Punktarithmetik einschließlich ±0, ±1,
  Subnormalzahlen und großen Werten gegen Long-Double-Rechnung; 2.000
  Intervallpaare; 800 Zustände aus 32 gemischten Ketten mit festen, rotierenden,
  kontinuierlichen und prismatischen Gelenken sowie versetzten Basis-/TCP-Frames.
  Optimierte Intervall-FK umschließt die separate Eigen-FK und die unveränderten
  dichten Double-Jet-Ableitungen bis zur dritten Ordnung.
- **44 Python-Prüfungen abgedeckt:** 42 Einzeltests im ersten Lauf erfolgreich;
  dessen HTTP-Klassenaufbau war zunächst durch die lokale Netz-Sandbox blockiert.
  Die zwei HTTP-Tests wurden danach mit Loopback-Netzfreigabe separat ausgeführt
  und bestanden. Der erste unveränderte Log und der HTTP-Nachlauf liegen als
  Belege bei. Dazu gehören Benchmark-Determinismus, optionale Zeitmetadaten,
  Generatorauswahl, alle CSV-Zeilen, Abbruch und unveränderte Vorresultate.
- **Drei echte Chromium-Prüfungen bestanden**, einschließlich nativer Simulation,
  Timing-Anzeige, vollständigem UR-Modell, Editor-/DXF-Abläufen und LIN-Orientierung.

Multi-Config-Nachweis und abschließende Strukturprüfungen sind im beigefügten
Prüfmanifest mit den tatsächlich ausgeführten Befehlen erfasst. Windows wurde
nicht physisch auf dem Anwender-PC getestet. Der Generatorpfad wird zusätzlich
mit Ninja Multi-Config geprüft; Unit-Tests verhindern den Rückfall auf eine alte
Root-/Debug-Binärdatei.

CI zeichnet die Phasenmessungen künftig als Artefakt auf und prüft die
Release-Auswahl auch mit Ninja Multi-Config. Es gibt keine flüchtige
Timing-Pass/Fail-Schwelle. Ein neu konfigurierter CI-Job ist noch kein erfolgreich
abgeschlossener Lauf dieser Revision.

## Reproduktion und Belege

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_KDL_REFERENCE=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
python tools/motion_viewer/benchmark.py --cycle-ms 1 10 20 --repeats 3 --output benchmark.json
```

[Einzelmessungen und Vergleich](evidence/preparation_v0.16.2/comparison.json),
[Produkt-Benchmark](evidence/preparation_v0.16.2/benchmark.json),
[Prüfmanifest](evidence/preparation_v0.16.2/manifest.json).
Die zusätzlich archivierte Messprobe dokumentiert exakt die Vergleichsmessung;
für Anwender ist `tools/motion_viewer/benchmark.py` der unterstützte Einstieg.

## Verbleibende Grenzen

Die vollständige Programmvorbereitung ist weiter synchron und hat keine
bewiesene obere Laufzeitgrenze. Schwierige IK, ungültige Ziele oder enge
Toleranzen können erheblich andere Zeiten erzeugen. Bestehende Arbeitsbudgets
und Abbruch bleiben wirksam; die mathematische Intervallimplementierung erhält
durch die Optimierung keine formale Zertifizierung.

Für Onlinebetrieb fehlen der inkrementelle Programmcompiler, begrenzte Fenster-
Produktion, definierte Vorlauf-/Unterlaufbehandlung und Lastnachweise auf dem Pi.
[Der konkrete Ausbauplan](../integration/planning_performance.md#was-für-echte-vorausschauende-planung-noch-fehlt)
führt diese Schritte auf. CANopen und Antriebshardware wurden nicht untersucht.
