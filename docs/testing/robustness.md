# Vier Robustheitspakete — 0.12.2

Die Prüfungen arbeiten ohne Hardware. Neue Tests verwenden dieselben öffentlichen
APIs wie Anwenderprogramme und Dispatch; KDL berechnet FK/J unabhängig.
Assertions bleiben auch in Release aktiv. Zufalls-Seeds sind im Quelltext fest;
die unten genannten Zählwerte stammen vom GCC-14.2-Linux-Lauf. Für identische
Zufallsfolgen Toolchain und Standardbibliothek beibehalten.

## Kinematische Grenzfälle

`kinematic_edges_tests` prüft drei reale Referenzmodelle an 360 lokalen IK-Zielen:
reguläre Zustände, exakt Null, Annäherungen an Null/Wrist-Singularitäten mit
±10⁻³ bis ±10⁻¹², Zustände 10⁻¹⁰ innerhalb jeder Gelenkgrenze und Orientierungen
mit 0 sowie π/π±10⁻⁹ rad zusätzlicher Drehung. Akzeptierte Lösungen müssen
Positions-/Orientierungsfehler ≤10⁻⁷ und die exakten Profilgrenzen einhalten.
Ablehnungen brauchen eine Diagnose und eine begrenzte Iterationszahl.
Im gemessenen Lauf: **137 akzeptiert, 223 begründet abgelehnt**. Diese Anteile
sind kein Anspruch, dass alle geometrisch erreichbaren Ziele gefunden werden;
der Solver sucht lokal und lehnt singularitätsnahe Lösungen konservativ ab.

Ein eigener Rotationsfall prüft das Überschreiten von π ohne Gelenksprung sowie
beide Quaternion-Vorzeichen. Vier LIN-Fälle mit 0/10⁻¹²/10⁻⁹/10⁻⁶ m und zwei
vollständige Kreise mit ±2π müssen tatsächlich zugelassen werden. Je 301 Samples
prüfen den Positionsschlauch und Gelenk-/TCP-Grenzen bis zum Ruck, einschließlich
ruhender Endpunkte. Ein Pfad über die Gelenkgrenze muss abgelehnt werden.
Die zugrundeliegende Planung prüft kontinuierliche Hüllen; die Samples ergänzen
diese Zulassung und ersetzen sie nicht.

Der neue Test im hardwareunabhängigen Kern deckte zwei XYZ-Gantry-Fehler auf:
negative Identitätsquaternionen wurden abgelehnt, NaN/Inf in Pose oder Gelenken
teilweise akzeptiert. Beide sind behoben. Der Quaternion-Regressionsfall wurde
vor der Korrektur ausgeführt und schlug wie erwartet fehl.

## Unabhängige FK-/Jacobian-Rechnung

`kdl_reference_tests` verwendet **Orocos KDL 1.5.1**, Commit und Archivhash in
`third_party/kdl-provenance.json`. KDL ist aus dem vollständigen beigefügten,
unveränderten Quellarchiv offline baubar und nur in dieses Testprogramm gelinkt.
`MOTION_WITH_KDL_REFERENCE=OFF` lässt es aus. Es beeinflusst keinen Motion-Solver.

Der Test liest die URDF separat mit TinyXML2, baut die Gelenkkette selbst auf
und benutzt KDLs rekursiven FK- und geometrischen Jacobian-Solver. Er ruft weder
unseren URDF-Importer noch unsere Transformations-/Jacobian-Helfer für die
Referenzrechnung auf. Verglichen werden TCP-Position, alle neun
Rotationsmatrixkoeffizienten und alle sechs Jacobian-Zeilen in Weltkoordinaten.

Seed **0x4BDF01**, insgesamt **1.866 Konfigurationen**:

- Je 300 zufällige Zustände plus beide Gelenkgrenzen für UR5e, ABB IRB120 und
  FANUC LR Mate 200iD: 906 Vergleiche.
- 24 unabhängig erzeugte Ketten mit festen, rotatorischen und prismatischen
  Gelenken, beliebigen Achsen, nichttrivialen Origins sowie Basis-/Werkzeug-
  Transformationen; je 40 Zustände: weitere 960 Vergleiche.

Größte absolute Differenz: **3,56·10⁻¹⁵** bei Positions-/Rotationskoeffizienten,
**3,11·10⁻¹⁵** beim Jacobian; Zulassungsschwelle jeweils 2·10⁻¹¹.
Die Modellparameter sind bewusst dieselben. Damit prüft der Vergleich
Implementierung und Interpretation, nicht die Übereinstimmung mit einem
physischen Roboter oder die Richtigkeit der Herstellerbeschreibung.

## Fehler während der Ausführung

`execution_faults_tests` ergänzt die bisherigen Parallelitäts-/Snapshottests:

- Doppelte Ticks, veraltete Zeitstempel, NaN und Grenzüberschreitungen sowie
  ein falsches Terminalbit dürfen keinen neuen Ausgabebefehl liefern.
- Falsche Uhrdomäne, NaN-Zeit, negative/zu große Uhrunsicherheit,
  Zeitrücksprung und Terminüberschreitung verriegeln den Fehler.
- Ein verspäteter oder bereits von der Übergabegrenze überholter Live-Wechsel
  wird zurückgewiesen. Ein belegtes Postfach nimmt keinen zweiten Plan an.
- **100** zufällige Füll-/Backpressure-Abläufe mit Seed **0xFA017** und einem
  Zweierpuffer liefern jeden der 201 Ticks genau einmal. Frühe Polls verbrauchen
  keinen Befehl, voller Puffer überschreibt nichts. Unterlauf darf auch nach
  erneutem Füllen nicht stillschweigend weiterlaufen.

`following_tests` injiziert während der Ausführung veraltete Beobachtungen und
Kalibrierungsrevision 2 statt 1. Die Historie nimmt diese Kombination nicht an;
daraus entsteht kein neuer Folgeplan. Der bereits freigegebene, unveränderliche
Plan bleibt Generation 1 und läuft über seinen vorbereiteten Halt bis zum
Stillstand, mit eindeutig fortlaufenden Ticks. Ein explizites Historien-Reset
löscht alte Beobachtungen, bevor neue Daten unter Revision 2 angenommen werden.
Ein Revisionswechsel ersetzt hier keinen unabhängigen Sofortstopp.

## Zufallsprogramme und Parser-Fuzzing

`random_programs_tests`, Seed **0xC0DE2026**, erzeugt **120** zulässige Programme
mit jeweils sechs gemischten PTP-/LIN-/WAIT-/CALL-Schritten und gelegentlichen
vollständigen CIRC-Kreisen. Ziele, Geschwindigkeiten und Start-Epochen variieren.
Alle werden geplant und vollständig virtuell ausgeführt: **59.506 Befehle**,
Stillstand an jeder Segmentgrenze und Gelenk-/TCP-Grenzen bis zum Ruck. Die
XYZ-Zielrechnung geschieht direkt, unabhängig von FK und Programmsimulation.
Ein zweiter Replay muss denselben Endzustand liefern. Je ein Syntaxfehler und
ein Fehler nach einem gültigen ersten Schritt ergeben **240** abgelehnte
Übernahmen; Referenz und Schedule des vorher veröffentlichten Programms bleiben
identisch. Diese Programme testen den Import-/Planungsablauf für eine einfache
Kinematik; die allgemeinen Sechsachsketten prüft zusätzlich der KDL-Vergleich.

`parser_fuzz_tests` mutiert gültige PTP/LIN/CIRC/WAIT/CALL-Eingaben **12.000** Mal,
Seed **0xF0222026**: Einfügen/Löschen/Kopieren, beliebige Bytes, ungültiges UTF-8,
Nullbytes, Tags/Aliase, Trennzeichen, NaN/Inf und Exponenten. Akzeptierte
Ergebnisse müssen ihre AST-Invarianten einhalten, Ablehnungen einen
`RobotProgramError` mit Diagnose liefern. Der Lauf ergab **132 akzeptierte und
11.868 abgelehnte** Mutationen. Zusätzliche Fälle erzwingen Byte-, Knoten-,
Verschachtelungs- und CALL-Tiefenbudgets; der bestehende Programmtest prüft auch
die Expansionsgrenze. CTest begrenzt die Laufzeit der Testprozesse.

Unerwartete C++-Ausnahmen speichern `parser-fuzz-failure.yaml`. Replay:

```sh
./build/parser_fuzz_tests parser-fuzz-failure.yaml
```

Zusätzlich gibt es einen echten, instrumentierten **Clang libFuzzer**-Einstieg.
Er instrumentiert auch Parser und Bibliotheksabhängigkeiten und führt dieselben
AST-Invarianten aus. Beliebige Dateinamen im YAML werden dabei nur geparst;
Fuzz-Eingaben lösen keine Modell-Dateizugriffe aus.

```sh
cmake -S . -B build-fuzz -DCMAKE_CXX_COMPILER=clang++ \
  -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS_DEBUG="-O1 -g1" \
  -DMOTION_SANITIZE=ON -DMOTION_BUILD_FUZZER=ON -DMOTION_WITH_KDL_REFERENCE=OFF
cmake --build build-fuzz --target program_parser_fuzzer --parallel 2
mkdir -p build-fuzz/corpus build-fuzz/findings
cp tests/fuzz/corpus/*.yaml build-fuzz/corpus/
UBSAN_OPTIONS=halt_on_error=1 ./build-fuzz/program_parser_fuzzer build-fuzz/corpus \
  -runs=50000 -max_len=8192 -timeout=5 -rss_limit_mb=1024 \
  -seed=4028751910 -artifact_prefix=build-fuzz/findings/
# Einen gefundenen Crash exakt wiedergeben:
./build-fuzz/program_parser_fuzzer build-fuzz/findings/crash-HASH
```

Der erste lokale Clang-19.1.7-Lauf absolvierte 50.000 Eingaben ohne Crash,
Timeout oder ASan-/UBSan-Befund. Weitere 100.000 Eingaben auf fortgesetztem/ergänztem Korpus bestanden ebenfalls
ohne Befund. Die vollständige Build-Matrix steht im [Prüfbericht](validation.md). CI führt bei
Push/PR die Variantenmatrix und einen begrenzten 50.000er-Fuzzlauf aus.
Fuzzing ist eine wiederholbare Fehlersuche, kein Beweis für jede mögliche Eingabe.
