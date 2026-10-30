# Integrationshandbuch für die eigene Steuerung

## Zweck und Geltungsbereich

Dieses Handbuch beschreibt **Robot Motion C++ 0.12.3** als einbettbare C++20-
Bibliothek. Eure Steuerung ist selbst entwickelt; konkrete Adapter, Prozesse,
Feldbuszuordnung und Freigabelogik werden später festgelegt. Der betrachtete
Zyklustakt liegt zwischen **1 und 20 ms**. Die Bibliothek plant Sollbewegungen
und liefert zeitlich geprüfte Gelenkbefehle. Antriebsregelung, Istwertüberwachung,
Maschinenzustände und die tatsächliche Befehlsübertragung bleiben Aufgaben der
Steuerung. Das Referenzbeispiel benutzt eine ideale virtuelle Uhr und simulierte
Achsen. Es schaltet keine Hardware frei.

Für den Einstieg sind drei Ebenen relevant:

1. **Import und Planung:** Modelle, Profile, Anwenderprogramme vollständig lesen
   und zulassen. Hier sind Dateizugriffe, dynamische Speicherverwaltung, IK,
   Hüllenprüfung und Ausnahmen zulässig.
2. **Ausführungsvorbereitung:** Zyklusraster bestimmen, Uhrbezug festlegen und
   entweder einen vollständigen Befehlscache oder einen Producer-Puffer aufbauen.
3. **Zyklische Übergabe:** Genau einen fälligen festen Befehl lesen, Rückgabestatus
   prüfen und die Achsschnittstelle bedienen. Kein neues Planen oder Samplen.

Die [Schnittstellenverträge](interface_contracts.md) sind die zugehörige
Feldreferenz; [Klassendiagramme](../architecture/class_diagrams.md) und
[Abläufe](../architecture/runtime_flows.md) zeigen die Beziehungen.

## 1. Bibliothek bauen

Voraussetzungen: C++20-Compiler und CMake ≥3.20. Eigen, yaml-cpp, TinyXML2 und
optional Ruckig sowie die testweise KDL-Referenz liegen versions-/hashgebunden
als lokale Quellarchive bei. Der C++-Build braucht keinen Netzwerkzugriff.
Python und Graphviz sind nur für ergänzende Werkzeuge bzw. Dokumentation nötig.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/program_simulator programs/examples/ur5e_demo.yaml --check
```

| CMake-Option | Standard im eigenständigen Projekt | Wirkung |
| --- | --- | --- |
| `MOTION_WITH_SERIAL` | ON | Kompilierte serielle Kinematik, Modell-/Programmimport, Kalibrierung |
| `MOTION_WITH_RUCKIG` | OFF | Online-Gelenkgenerator, Halteplanung, vorbereitete Halte-Schedules und Following |
| `MOTION_BUILD_TESTS` | ON | Eigene CTests; bei `add_subdirectory` standardmäßig OFF |
| `MOTION_BUILD_EXAMPLES` | ON | Standalone-Beispiele; bei `add_subdirectory` standardmäßig OFF |
| `MOTION_WITH_KDL_REFERENCE` | ON | Nur bei aktiven Tests: unabhängiger FK/J-Vergleich |
| `MOTION_SANITIZE` | OFF | ASan/UBSan für Entwicklung |
| `MOTION_BUILD_FUZZER` | OFF | Clang/libFuzzer, benötigt Tests, Serial und Sanitizer |

`BUILD_TESTING=OFF` setzt den Standard von `MOTION_BUILD_TESTS` auf OFF, sofern
keine eigene Auswahl gespeichert ist. Bereits gesetzte Cache-Optionen bleiben
CMake-üblich erhalten. Produkt- und Testbuilds deshalb getrennt konfigurieren.

### Als Unterprojekt übernehmen

Im späteren Steuerungsprojekt beispielsweise:

```cmake
set(MOTION_BUILD_TESTS OFF CACHE BOOL "")
set(MOTION_BUILD_EXAMPLES OFF CACHE BOOL "")
set(MOTION_WITH_SERIAL ON CACHE BOOL "")
set(MOTION_WITH_RUCKIG OFF CACHE BOOL "")
add_subdirectory(external/robot-motion-cpp)
target_link_libraries(eure_steuerung PRIVATE RobotMotion::motion)
```

`RobotMotion::motion` liefert Include-Pfade, C++20 und die ausgewählten
Bibliotheksabhängigkeiten transitiv. Eigene Include-Pfade für Eigen/yaml-cpp sind
nicht erforderlich. Bestehende Targets `robot_motion` und `robot_motion_serial`
bleiben vorhanden; `RobotMotion::serial` benennt die kompilierte Teilbibliothek.
Ein installierbares `find_package(RobotMotion)`-Paket und eine C-ABI sind derzeit
nicht enthalten. Für eine andere Host-Sprache muss später ein Adapter festgelegt
werden; C++-Objektlayout oder `std::vector` nicht als Drahtformat verwenden.

Das getrennte Anwenderprojekt überprüft diese Einbettung bereits:

```sh
cmake -S examples/integration/consumer -B build-consumer \
  -DROBOT_MOTION_SOURCE_DIR="$PWD" -DCMAKE_BUILD_TYPE=Release
cmake --build build-consumer --parallel 2
./build-consumer/controller_consumer
```

## 2. Modell, Profil und Programm zuordnen

Für eine serielle Kette beschreibt URDF Gelenktyp, Achse, Reihenfolge und
Geometrie. Das SI-YAML-Profil ergänzt zulässige Beschleunigung/Ruck sowie TCP-
Grenzen, Basis und Werkzeug. Die Bewegungsgrenzen sind von den geometrischen
Kinematikgrenzen zu unterscheiden. Das Programm verweist auf sein Profil;
relative Dateipfade beziehen sich auf die **Programmdatei**, nicht auf den
Prozess-Arbeitsordner. Ohne das optionale Programmfeld `robot_urdf` wird
`robot.urdf` neben der Profildatei geladen. Bei gesetztem `robot_urdf` wird auch
dieser Pfad relativ zur Programmdatei aufgelöst; das Profil enthält keinen URDF-Pfad.

Vor einem Steuerungstest festlegen und außerhalb des Zyklus prüfen:

- `joint_names()` und `dof()` gegen die tatsächliche Host-Achsreihenfolge;
  ein sechsachsiger Roboter bedeutet nicht automatisch dieselbe Achszuordnung.
- Winkel in rad, prismatische Wege in m, Ableitungen pro Sekunde. Encoder-
  Skalierung, Vorzeichen, Übersetzung und Nullpunkt gehören in die spätere
  Achsabbildung. Gelenkposition und Motorposition sind nicht austauschbar.
- Welt-/Basis-/Werkzeugkoordinaten und Quaternionreihenfolge `[w,x,y,z]`.
- Konkrete Softwaregrenzen, gültige Kalibrierungsrevision und Profilversion.
  Öffentliche Beispielprofile sind nominale Testdaten.
- Der `start`-Zustand des Programms muss zum freigegebenen Anfangszustand der
  Steuerung passen. Die Bibliothek liest keine Encoder und fährt nicht
  automatisch auf diesen Startpunkt.

Siehe [Programmformat](../formats/robot_program_yaml.md) und
[Datenvertrag](interface_contracts.md).

## 3. Vollständigen Plan vorbereiten

```cpp
#include <motion/program/robot_program.hpp>

const auto prepared_program = motion::load_robot_program(program_filename);
// Ausnahmen hier abfangen, bevor ein Programm veröffentlicht wird.
// prepared_program.reference: fachliche Referenz und Quellorte
// prepared_program.schedule: vollständig zugelassenes Zyklusraster
```

Das Programm wird komplett geprüft. Ein später ungültiger Schritt verhindert die
Rückgabe eines neuen `PreparedRobotProgram`; ein zuvor gespeichertes Programm
bleibt unverändert. Erst nach erfolgreicher Rückgabe darf die Anwendung ihre
aktive Programmauswahl ändern. Fehler als `RobotProgramError` samt Datei, Block,
Schrittnummer, Zeile, Spalte und Diagnose anzeigen.

### Zykluszeit festlegen

Im Programm bedeutet `cycle_s: 0.001` einen Millisekundentakt. Eine andere
Taktzeit wird auf Basis der ursprünglichen Referenz neu ausgerichtet:

```cpp
const double cycle_period_s = 0.020;
const auto prepared_for_cycle = motion::retime_robot_program(
    prepared_program, cycle_period_s);
// prepared_for_cycle.reference und .schedule gehören nachweislich zusammen.
// Fehler als std::invalid_argument außerhalb des zyklischen Threads behandeln.
```

Die Ausrichtung verkürzt kein Segment und erhält die Stillstandsgrenzen.
`retime_robot_program()` prüft zuerst die Herkunft des bisherigen Schedule und
erzeugt anschließend ein neues Paar aus derselben Referenz und dem neuen Raster.
Das ursprüngliche Paar bleibt auch bei einem Fehler unverändert. Für eine einzelne
Programmreferenz gibt es `prepare_robot_program(reference, cycle_period_s)`.
Die öffentliche Paarstruktur bleibt aus Kompatibilitätsgründen verfügbar;
selbst zusammengesetzte Paare vor eigener Verarbeitung mit `validate()` prüfen.
Der Simulator lehnt nicht zusammengehörige Paare vor der ersten CSV-Ausgabe ab.
Eine Bewegung bekommt bei gröberem Raster gegebenenfalls eine längere Dauer.
Verringerung von v/a/Ruck erfolgt über die Zeitstreckung. Es wird nicht einfach
jeder zehnte Sollwert verwendet. Ein Wechsel der Host-Zykluszeit während einer
laufenden Ausführung ist kein unterstützter Live-Wechsel: Schedule und Cache
müssen passend neu vorbereitet werden.

### Uhr und Epoche verbindlich machen

`ClockTime.seconds` und alle `timestamp_s` müssen zu derselben monotonen Uhrdomäne
gehören. `clock` ist deren vereinbarte ID; die Bibliothek konvertiert keine
Uhren. Unix-Zeit oder Kamera-Zeit nicht ungeprüft einsetzen.

Der Ausführungsstart steht in `schedule->initial_state().timestamp_s`. Für globale
Ticks gilt `timestamp_s = execution_epoch_s + tick_time(tick)`.
Für Zeitpunkte die bereitgestellte Methode `schedule->tick_time(tick)` verwenden;
nicht durch wiederholtes Addieren der Zyklusdauer Rundungsdrift aufbauen.
`prepare_live_plan()` verlangt diese Übereinstimmung **exakt**; der Parameter
`epoch` verschiebt einen bestehenden Schedule nicht automatisch.

Im Referenzbeispiel wird die Programmepoche unverändert als virtuelle Epoche
benutzt. Für die spätere echte Steuerung muss die Host-Schicht einen stabilen
Zeitbezug bereitstellen: entweder die gewünschte Epoche bei der Erstellung des
Programms setzen und ausreichend Startvorlauf lassen oder einen expliziten
Schedule-Adapter erstellen, der ausschließlich Zeitstempel konsistent neu bindet
(`initial_state`, `sample`, `sample_tick`). Ein solcher Startadapter gehört zum
noch zu planenden Integrationspaket. Relative Trajektorienzeiten, Tickdauer,
Befehlsinhalt und aktuelle Zeit dürfen dabei nicht vermischt werden.

## 4. Ausführungsmodell auswählen

| Modell | Geeignet für | Vorbereitung | Zyklisch |
| --- | --- | --- | --- |
| Vollständiger Cache | Erster Integrationstest, begrenzte Programmlänge | Alle festen Befehle mit `prepare_live_plan<N>()` erzeugen | `LiveMotionExecutor<N>::poll_clock()` |
| SPSC-Puffer | Längere Programme mit dauerhaft verfügbarem Producer | `BufferedMotionProducer<N>` und festen Ring anlegen/vorfüllen | `DeadlineMotionExecutor<N>::poll_clock()` |

Für den ersten Test ist der vollständige Cache leichter nachzuvollziehen.
`N` ist die statisch gewählte Achsanzahl. Der sechsachsige Beispieladapter
verwendet `N=6`; ein anderes Modell braucht eine passende Instanziierung und
Achsabbildung, keine stille Abschneidung oder Auffüllung.

### Cachepfad

```cpp
#include <motion/execution/live_motion_executor.hpp>

constexpr std::size_t axis_count = 6;
constexpr motion::ClockId controller_clock_id = 1;
const auto execution_schedule = prepared_program.schedule;
const double execution_epoch_s = execution_schedule->initial_state().timestamp_s;
const auto prepared_cache = motion::prepare_live_plan<axis_count>(
    execution_schedule, execution_epoch_s,
    {1, 0, 0, controller_clock_id}); // generation, parent, global start tick, clock
if (!prepared_cache) { /* Start verhindern; diagnostic ausgeben */ }

motion::LivePlanMailbox<axis_count> plan_mailbox;
const double cycle_period_s = execution_schedule->cycle_s();
const motion::ExecutionTimingPolicy timing_policy{
    0, cycle_period_s * .1, 0, 0}; // Nur ideales virtuelles Beispiel!
motion::LiveMotionExecutor<axis_count> motion_executor(
    *prepared_cache.plan, plan_mailbox, timing_policy);
```

Die Host-Anwendung hält `prepared_cache.plan` so lange am Leben, wie der Executor
oder ein ausstehender Wechsel ihn referenziert. Der Executor übernimmt keinen
`shared_ptr`. Ein Cache darf nach Beginn der Benutzung nicht verändert werden.
Die Freigabe und damit verbundene Speicherarbeit findet außerhalb RT statt.

### Gepufferter Pfad

`BufferedMotionProducer<N>::fill(ring, budget)` gehört in einen eigenen
Nicht-RT-Producer. Der Ring ist SPSC: exakt ein Producer und ein Consumer.
Ein voller Ring erzeugt Backpressure, ohne ungelesene Werte zu überschreiben.
Der Consumer wartet nicht auf den Producer. Ein Unterlauf zur fälligen Ausgabe
verriegelt `ExecutionFault::Underflow`; späteres Nachfüllen setzt die Ausführung
nicht fort. Den Worst-Case-Vorlauf des Producers gegen die Ringkapazität planen.
Die Konstruktoren und das finale Freigeben der geteilten Schedules gehören in
die Vor-/Nachbereitungsphase.

## 5. Zyklische Befehlsübergabe

Die Steuerungsuhr braucht `ClockTime now() noexcept`; sie muss eigene Fehler über
nichtendliche Werte oder einen expliziten Host-Fehlerpfad melden. Alternativ kann
unter Linux `LinuxMonotonicClock` benutzt werden. Der Dispatcher schreibt seinen
Ausgabewert nur bei `Running` und `Finished`.

```cpp
// Nur der zyklische Eigentümer greift direkt auf den Executor zu.
motion::LiveJointCommand<axis_count> next_command;
const auto execution_status = motion_executor.poll_clock(clock, next_command);
if (execution_status == motion::ExecutionStatus::Running ||
    execution_status == motion::ExecutionStatus::Finished) {
  // Inhalt jetzt genau einmal an die Host-Achsschnittstelle übergeben.
  // Finished enthält den letzten Stillstandsbefehl und muss ebenfalls raus.
} else if (execution_status == motion::ExecutionStatus::Error) {
  // Anwendungsfehler verriegeln; vereinbarte Stop-/Fehlerreaktion auslösen.
}
// Inactive: nichts neu senden. Den alten Inhalt von next_command nicht wiederholen.
```

`poll_clock()` liest die Uhr vor und nach der Prüfung. Damit kann es auch eine
Unterbrechung während der Verarbeitung erkennen. `poll(now, output)` hat nur die
eine vom Aufrufer bereitgestellte Zeitbeobachtung. Beide Varianten können
zukünftige Verzögerungen der nachfolgenden Achsschnittstelle nicht messen.
Deren garantierte maximale Annahmelatenz muss in `dispatch_latency_bound_s`
eingerechnet und später auf der Zielsteuerung überprüft werden.

Die spätere Achsschnittstelle benötigt einen klaren Vertrag: Annahme aller
Achswerte für genau diesen Tick oder gemeldeter Fehler, begrenzte Ausführungszeit,
kein Behalten eines Zeigers auf die wiederverwendete Ausgabestruktur und
geklärter Umgang mit partieller Übertragung. Bei Ablehnung ist der Dispatcher
bereits weitergeschaltet: nicht denselben Befehl erneut senden und nicht
ungeprüft weiterpollen. Die Anwendung verriegelt diesen Übergabefehler und
führt die vereinbarte Fehlerreaktion aus. Das Referenzbeispiel zeigt diese
zusätzliche Anwendungssperre, ohne einen bestimmten Hardware-Stopp vorzugeben.

## 6. Speicher und Threadgrenzen

| Tätigkeit | Kontext |
| --- | --- |
| URDF/YAML, IK, Planung, Hüllen, Kalibrierungs-Fit | Vorbereitungs-/Planerthread |
| `sample()`, `sample_tick()`, Cachebau, `fill()` | Vorbereitungs-/Producerthread |
| `poll[_clock]()`, feste Push/Pop, Live-Aktivierung, Fence | Ein zyklischer Eigentümer |
| `snapshot()` | Auch parallele Beobachter; begrenzte Wiederholversuche, optionales Ergebnis |
| `generation()`, `fault()`, `commit_delivery()`, `service_requests()` | Zyklischer Eigentümer, keine beliebigen parallelen Zugriffe |
| Logs, Dateien, UI-Meldungen, `shared_ptr`-Freigaben | Außerhalb des zyklischen Pfads |

Feste Ausgabevektoren sind `std::array`; die Trajektorienschnittstellen liefern
häufig `std::vector` und können allokieren. `const` und unveränderlicher Besitz
allein bedeuten keine Allokationsfreiheit. Die vorhandenen Tests beobachten
C++-`new/delete` im Dispatcherpfad; das ist kein Beweis für beliebige Host-Callbacks.

Cachebedarf ungefähr `(total_ticks+1) * sizeof(FixedJointCommand<N>)`, zusätzlich
Containerkapazität und Referenzpläne. Bei sechs Achsen beträgt die Struktur im
geprüften 64-bit-Linux-Build 176 Byte: eine Minute mit 1 ms benötigt rund
10,1 MiB reine Befehlsdaten. Die Vektorkapazität kann größer sein; bei Live-
Wechseln leben alter und neuer Cache zeitweise gleichzeitig. Die Standardgrenze
von `prepare_live_plan()` beträgt 200.000 Befehle einschließlich Endbefehl.
Sie ist explizit änderbar und kein allgemeines RAM-Budget. Tatsächliches `sizeof`
und Spitzen-RAM auf eurem Build messen. 64-bit-Atomics müssen auf der gewählten
Plattform lock-free sein; die betreffenden Templates verlangen das zur Compilezeit.

## 7. Live-Wechsel erst nach stabilem Grundbetrieb

Ein neuer Cache hat höhere Generation, exakte Vorgängergeneration, gleiche
Uhrdomäne, Epoche, Takt und Grenzen. Sein erster globaler Tick muss mit q/dq/ddq
und Zeitstempel **exakt** zum aktiven Cache passen. Anfangszustände deshalb direkt
aus dessen Befehl übernehmen, nicht erneut mit Rundung rekonstruieren.

Der Planer bietet über `try_offer()` maximal einen ausstehenden Plan an.
Der Executor prüft Identität, Mindestvorlauf, Zeitpunkt und Übergabegrenze.
Aktivierung erfolgt am vereinbarten Tick; die Antwort benennt die freigegebene
Altgeneration. Nach einer Ablehnung bleibt der alte Cache aktiv. Ein belegtes
Postfach nicht im zyklischen Thread mit einer Warteschleife bedienen.

Wenn die Host-Achsschnittstelle bereits Befehle angenommen hat, wird die
unwiderruflich übergebene Grenze mit `commit_delivery(generation, through_tick)`
geführt. Kein Wechsel darf bereits angenommene Befehle rückwirkend ersetzen.
Besitzdauer, Acknowledgement und Freigabe sind in den
[Live-Sequenzen](../architecture/runtime_flows.md) beschrieben.

## 8. Timing für 1–20 ms

Zulässig sind nur nicht überlappende Zeitfenster:

```text
0 < max_early_s + max_late_s < cycle_s
2 * max_clock_uncertainty_s + dispatch_latency_bound_s
    <= max_early_s + max_late_s
```

Für jeden Befehl muss die gesamte beobachtete Zeitunsicherheit inklusive
nachfolgender Dispatchlatenz in dessen Fenster passen. Beispielbudgets,
**keine gemessenen Zielplattformwerte**:

| Zyklus | maximal früh | maximal spät | Uhrunsicherheit | Dispatchlatenz |
| --- | --- | --- | --- | --- |
| 1 ms | 20 µs | 180 µs | 5 µs | 40 µs |
| 10 ms | 200 µs | 1.800 µs | 50 µs | 400 µs |
| 20 ms | 400 µs | 3.600 µs | 100 µs | 800 µs |

Ein gröberer Takt erleichtert nicht automatisch jede Regelungsaufgabe; ein
feinerer Takt erhöht Befehlszahl, Speicherbedarf und Übertragungsanforderungen.
Die Bibliothek kann alle drei Beispielraster planen; dadurch ist ihre physische
Einhaltung auf eurer Steuerung noch nicht bestimmt. Für den realen Aufbau müssen
Uhrfehler, Aufwachlatenz, Dispatch, Achsannahme und Lastfälle gemeinsam gemessen
werden. Ein Fehler wird verriegelt, es gibt kein automatisches Aufholen oder
Überspringen von Ticks.

## 9. Ausführbares Referenzbeispiel

```sh
./build/controller_contract_example programs/examples/ur5e_demo.yaml 1 normal
./build/controller_contract_example programs/examples/ur5e_demo.yaml 10 normal
./build/controller_contract_example programs/examples/ur5e_demo.yaml 20 normal
./build/controller_contract_example programs/examples/ur5e_demo.yaml 10 late-clock
./build/controller_contract_example programs/examples/ur5e_demo.yaml 10 sink-refusal
```

Quelle: [controller_contract_example.cpp](../../examples/integration/controller_contract_example.cpp).
Bei den Fehlerszenarien bedeutet `CONTRACT PASS`, dass die **erwartete
Fehlerreaktion** geprüft wurde: einmalige Stopanforderung und kein nachfolgender
Befehl. Der normale Fall liefert den terminalen Stillstandsbefehl genau einmal.
Alle Zeitpunkte sind virtuell; die Laufzeit des Beispiels ist kein 1-ms-Benchmark.
Die eigene Steuerungssoftware wird durch dieses Beispiel nicht verändert.

Nächster Schritt für die spätere konkrete Integration ist die Auswahl der noch
offenen Verträge im [Abnahme- und Integrationsplan](acceptance_plan.md).
