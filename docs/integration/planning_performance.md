# Planungszeit und vorausschauende Onlineplanung

Stand: **0.16.2**, Testwerkzeug **0.5.2**. Ausgangspunkt ist die lange
Vorbereitung des UR5e-Pick-and-Place-Programms auf dem Anwender-PC.

## Was aktuell während der Wartezeit passiert

Der Editor lädt das Modell, plant und prüft **das vollständige Programm**,
rastert es auf den gewählten Takt, simuliert jeden Sollwert und erzeugt danach
die Ansicht. Erst nach erfolgreichem Abschluss wird ein neues Ergebnis sichtbar.
Die Abspielzeit ist die simulierte Bewegungsdauer; die Vorbereitung findet davor
statt. Das Bibliotheks-API `load_robot_program` macht ebenfalls eine vollständige
Vorbereitung. Ein schneller Executor allein macht diesen Import nicht onlinefähig.

```mermaid
flowchart LR
  A[YAML und Modell] --> B[Alle Segmente planen und zulassen]
  B --> C[Zyklusraster]
  C --> D[Alle Sollwerte virtuell ausgeben und prüfen]
  D --> E[HTML und Robotergeometrie]
  E --> F[Abspielen]
```

Für eine spätere Steuerung gehören Planung und Prüfung auf einen Worker. Der
zyklische Ausführungspfad übernimmt bereits vorbereitete Befehle. Der gewünschte
Takt von **1–20 ms** ist kein Zeitbudget, in dem derzeit ein beliebiger LIN- oder
CIRC-Auftrag vollständig berechnet werden könnte.

## Konkrete Beschleunigung

Die numerische Intervall-FK war beim untersuchten Programm der Hauptaufwand:
Sie multiplizierte auch zeitlich konstante Frames als dichte 4×4-Matrizen mit
Ableitungen bis zur dritten Ordnung. Damit wurden zahlreiche bekannte Nullterme
wiederholt multipliziert, addiert und nach außen gerundet.

Ab 0.16.2 verwendet diese Rechnung die Struktur starrer Transformationen:
konstante Frames werden ordnungsweise angewendet, die homogene letzte Zeile
bleibt exakt `[0,0,0,1]`, Drehgelenke verändern nur den Rotationsblock und
Schubgelenke nur die Translation. Exakte Intervallidentitäten für 0 und ±1
vermeiden weitere Arbeit. Alle verbleibenden Intervalloperationen verwenden die
bisherige Rundung nach außen. Die unveränderte dichte Double-Rechnung bleibt ein
separater Vergleich für die Ableitungen.

Bei der Spline-Zulassung wird am Mittelpunkt außerdem nur die benötigte FK-Pose
berechnet. Die dort früher zusätzlich ausgewerteten TCP-Ableitungen wurden
verworfen; die Hüllenrechnung stellt die benötigten Schranken bereits bereit.
Positionstoleranz, Orientierungstoleranz, Gelenk- und TCP-Grenzen sowie
Unterteilungsbudgets werden für die Beschleunigung nicht gelockert.

Unter Windows wählen die Python-Starter nun auch bei Visual Studio und anderen
Multi-Config-Generatoren ausdrücklich `--config Release` und dessen ausführbare
Datei. `CMAKE_BUILD_TYPE=Release` allein hatte diese Konfiguration nicht festgelegt.
Die Konfiguration des tatsächlich gestarteten Exporters wird im Ergebnis genannt.
Das kann zusätzliche Unterschiede auf einem Anwender-PC erklären; dessen
bisheriger Build wurde hier nicht untersucht.

## Auf dem eigenen Rechner messen

Editor neu starten und das Programm erneut simulieren:

```sh
python tools/motion_viewer/run_editor.py
```

Die Ergebniszeile nennt nun Planung, Simulation, Gesamtzeit und Build-Konfiguration.
Vorhandene HTML-Dateien und mit `--exporter` übergebene ältere Binärdateien werden
nicht automatisch aktualisiert. Für letztere den aktuellen Exporter neu bauen.

Eine wiederholbare Messung ohne Browser liefert:

```sh
python tools/motion_viewer/benchmark.py --cycle-ms 1 10 20 --repeats 3
```

Standardmäßig werden Pick-and-Place und das Beispiel mit konstanter
LIN-Orientierung gemessen. Eigenes Programm:

```sh
python tools/motion_viewer/benchmark.py --program mein_programm.yaml --cycle-ms 10 --repeats 3 --output benchmark.json
```

Buildzeit und JSON-Einlesen im Benchmark liegen außerhalb der gemessenen nativen
Prozesszeit. Wiederholte Läufe müssen identische Bewegungsdauer, Sollwertzahl,
Arbeitszähler und CSV-SHA256 liefern; sonst bricht der Vergleich ab. Der Bericht
enthält alle Einzelwerte, Median und größte beobachtete Planungszeit. Ein Maximum
aus wenigen Läufen ist **keine obere Laufzeitschranke**.

| Messwert | Enthalten |
| --- | --- |
| `planning_s` | Modell-/Programmimport, vollständige Planung/Zulassung, Zyklusraster und gegebenenfalls erneutes Rastern |
| `simulation_s` | Virtuelle Ausführung jedes Ticks einschließlich CSV-Spooling |
| `native_process_s` | Aufruf des nativen Werkzeugs einschließlich JSON-Ausgabe und Prozessstart/-ende |
| `postprocess_s` | Im Editor: JSON-/CSV-Prüfung, Modellansicht, HTML und Programm-Bündel |
| `total_s` | Im Editor: Bearbeitung nach Request-Validierung bis unmittelbar vor Veröffentlichung, ohne nachgelagerte Altdatei-Bereinigung |
| `work_units` | Vorbereitungszähler; weder CPU-Instruktionen noch Echtzeitnachweis |

Bericht und Grenzen der lokalen Messung: [Prüfnachweis](../testing/planning_performance_validation.md).

## Was für echte vorausschauende Planung noch fehlt

Diese Optimierung reduziert die Vorbereitung. Der Programmcompiler erstellt
weiterhin den ganzen Auftrag. Die vorhandenen Live-Planwechsel und bewegten
Horizonte sind Bausteine; ein inkrementeller YAML-Programmplaner ist damit noch
nicht implementiert.

Die nächsten Pakete sind in dieser Reihenfolge sinnvoll:

1. **Begrenzte Segmentvorbereitung:** Compiler-Cursor mit definiertem Eingangs-
   und Endzustand; höchstens eine begrenzte Zahl an Bewegungen beziehungsweise
   DXF-Teilstücken pro Job. Gemeinsame Arbeits-/Speicherbudgets, Abbruch und
   Quellorte gelten auch für dieses Fenster. Die vorhandenen Ruhegrenzen
   ermöglichen zunächst Übergaben ohne zusätzliche Blend-Semantik.
2. **Lookahead-Worker und begrenzte Übergabe:** Zugelassene Fenster im Hintergrund
   erzeugen, Besitz über die bestehende Live-Session verwalten und ausreichend
   Vorlauf vor dem Start füllen. Modell-, Kalibrierungs-, Programm- und
   Grenzversion gehören zur Fensteridentität. Veraltete Ergebnisse verwerfen.
   Simulation, CSV und Rendering liegen außerhalb dieses Produktionspfads.
3. **Definiertes Verhalten bei zu wenig Vorlauf:** Vorab zugelassene Haltstrategie,
   keine Fortsetzung mit ungeprüften Daten und kein Warten im zyklischen Thread.
   Verzögerung, Abbruch, Pufferfüllstand, Grenzfall-IK und Versionswechsel unter
   Last prüfen. Die Reserve muss die Laufzeit des Halts berücksichtigen.
4. **Abnahme auf dem Pi 4B:** Laufzeiten pro Fenster, schlechteste beobachtete
   Verzögerung, Speicherspitzen und Reserve unter realistischer Konkurrenzlast
   messen; daraus die zulässigen Fenster-/Startbedingungen ableiten. Erst das
   erlaubt eine belastbare Aussage zur Zielplattform. CANopen bleibt vorerst
   zurückgestellt.

Ein Verhältnis Planungszeit/Bewegungsdauer unter 1 ist eine notwendige
Durchsatzbetrachtung für einen dauerhaften Betrieb, aber allein unzureichend:
Ein einzelnes schwieriges Fenster kann seinen Vorlauf trotzdem aufbrauchen.
Gemessene PC-Durchschnittswerte werden deshalb nicht als garantierte Online-
oder Echtzeitfähigkeit ausgewiesen.
