# Prüfnachweis: Besitz und Übergabe — 0.14.0

Stand: **30.10.2026**, Grundlage **0.13.1 / `cb41cb2`**.
Das zweite Paket aus dem [Architektur-Review](../architecture/architecture_review_v0.13.0.md)
ergänzt den [verwalteten Ausführungsvertrag](../integration/live_motion_session.md).
Die erste Korrekturrunde bleibt als [eigener Versionsnachweis](architecture_review_fixes.md) erhalten.

## Implementierung

- `execution/live_plan_owner.hpp`: genau ein Management-Endpunkt hält aktive und
  angebotene Cache-Referenzen, sammelt Antworten, fordert Rücknahme/Shutdown an
  und gibt erst nach bestätigter Quieszenz frei.
- `execution/live_motion_session.hpp`: ein zyklischer Endpunkt koppelt feste
  Ausgabe und synchrone Host-Annahme an den Delivery-Fence. Er verriegelt
  Ausführungs-/Annahmefehler und veröffentlicht deren ursprünglichen Code,
  Generation und Tick. Er erzeugt keinen zusätzlichen Thread.
- Der rohe Executor erhält additive Rücknahme-/Shutdown-Operationen. Alte
  Include-Pfade und bisherige Enumwerte bleiben erhalten. Nach Shutdown werden
  geliehene Cache-Adressen nicht mehr gelesen.
- `BufferFillResult` erhält strukturierten Fehlergrund, Tick und ursprüngliche
  `exception_ptr`. Ein Referenzfehler bleibt verriegelt; weitere Fills dürfen
  nicht still erneut samplen.
- Das sechsachsige Steuerungsbeispiel und der externe CMake-Consumer verwenden
  die neue Sitzung. Es wurden keine Bewegungsgesetze oder YAML-/DXF-Formate geändert.

## Geprüfte Eigenschaften

| Eigenschaft | Konkreter Test |
| --- | --- |
| Lebensdauer | Im Test besitzt nur der Owner die letzten Referenzen. Ein kontrollierter Deleter prüft, dass die tatsächliche Cache-Freigabe auf dem Managementthread erfolgt. |
| Aktivierung | Alte und neue Referenz bleiben bis zur passenden Antwort erhalten; `Activated` gibt nur den alten Cache frei. Tickfolge und Generation bleiben korrekt. |
| Backpressure / Identität | Zweites Angebot wird bei offener Anfrage/ungelesener Antwort abgewiesen; Generationsnummern werden nicht wiederverwendet. |
| Rücknahme | Vor der Aktivierung `Cancelled`; eine bereits erfolgte Aktivierung bleibt `Activated`. Der aktive Plan läuft bei Rücknahme weiter. |
| Host-Annahme | Jeder übernommene Tick wird genau einmal bestätigt. Eine Ablehnung erhält Generation/Tick und führt ohne Wiederholung zum verriegelten Fehler. |
| Terminaler Befehl | `Finished` erst nach Annahme; terminale Ablehnung ergibt `Error`; spätere Aufrufe erzeugen keinen doppelten letzten Befehl. |
| Shutdown | Vor/nach Admission und bei ungelesener Aktivierungsantwort; frühzeitige Freigabe wird abgewiesen. Quieszenz gibt alle Referenzen frei, auch ohne einzeln gelesene Antwort. |
| Laufende Übergabe | Ein gezielt angehaltener Test-Sink erlaubt eine Shutdown-Anforderung während `try_write`. Der laufende Befehl darf abschließen; erst der nächste Dispatch bestätigt Quieszenz. |
| Freigabe bei noch lebendem RT-Thread | Nach Quieszenz werden Caches freigegeben; der Thread ruft weiter Dispatch auf und erhält ohne Cachezugriff `Inactive` bzw. `Error`. Die Sitzung selbst wird erst nach Join zerstört. |
| Zwei Threads | 200 aufeinanderfolgende Planwechsel mit echten gleichzeitigen Owner-/Dispatch-/Snapshot-Zugriffen und 200 Angebot-/Fehlerrennen; kontrollierte virtuelle Ticks. |
| Diagnose-Publikation | Gleichzeitiges Lesen der einmalig geschriebenen Fehlerdiagnose; kein teilweise gelesener Ursachen-/Tickdatensatz. |
| Producer-Ursache | Eigenes Nicht-std-Fehlerobjekt bleibt per exception_ptr erhalten, einschließlich Tick und schon erzeugtem Präfix. Wiederholter Fill sampelt nicht erneut. |
| C++-Speicherarbeit | Globales new/delete wird im zyklischen Testpfad gezählt: einschließlich Aktivierung, Rücknahme, Snapshot, Annahme, Fehler und Quieszenz **0 / 0**. Management-Freigaben liegen ausdrücklich außerhalb dieses Messfensters. |

Die Tests stehen in `tests/execution/test_live_motion_session.cpp`,
`test_live_execution.cpp`, `test_execution_faults.cpp` und
`test_dispatch_allocations.cpp`. Der blockierende Test-Sink dient ausschließlich
der gesteuerten Interleaving-Prüfung, nicht als Vorbild für einen Produktions-Sink.

## Ausgeführte Matrix

Linux/x86-64, GCC 14.2.0, C++20, CMake 4.4.4. Eigene Targets mit strikten
Warnungen und aktiven Assertions, auch in Release. Vorhandene Buildverzeichnisse
wurden neu konfiguriert und betroffene Targets neu gebaut.

| Konfiguration | Ergebnis |
| --- | --- |
| Core Release, Serial/Ruckig/KDL an | **32/32 CTests bestanden** |
| Core Debug `-O1 -g1`, ASan/UBSan, Serial/Ruckig/KDL an | **32/32 CTests bestanden** |
| Minimal Release, Serial/Ruckig aus | **12/12 CTests bestanden** |
| Zusätzliche Angebot-/Fehlerrennen | Lifecycle-Test nach Erweiterung separat in Release, Minimal und ASan/UBSan bestanden; auch als header-only Test ohne Serial unter TSan |
| ThreadSanitizer, Clang 19.1.7, `-O1 -g1` | Gesamter Session-Lifecycle-Test mit 200 Wechseln und 200 Angebot-/Fehlerrennen bestanden, `halt_on_error=1` |
| Sechsachsiges Integrationsbeispiel | 1/10/20 ms, spätes Zeitfenster, Sink-Ablehnung und angeforderter Shutdown bestanden |
| Getrenntes Viewer-/Editor-Werkzeug, Release | **32/32 Python-Tests bestanden** mit neu gebautem nativem Exporter |
| Externer CMake-Consumer | Serial-Standard und Minimal mit ASan/UBSan gebaut und ausgeführt |
| Öffentliche Header einzeln | **39/39** mit Serial/Ruckig kompiliert |

Der zusätzliche fokussierte TSan-Lauf instrumentiert die Session und deren
headerbasierten Ausführungspfad, keine vollständige serielle Solver-/Parser-Matrix.
Im ASan/UBSan-Lauf war LeakSanitizer deaktiviert, UBSan lief mit
`halt_on_error=1`. Die 32 Werkzeugtests laufen innerhalb eines CTest-Eintrags;
die Zahl bezeichnet dessen unittest-Fälle. Browser-Smokes und der separate
ASan-Exporter wurden in diesem Paket nicht erneut geprüft; die Browserquellen
wurden nicht geändert.

Vorbereitete Spuren von UR5e bei 1/10/20 ms, ABB und FANUC bei 10 ms sowie
des DXF-Beispiels werden gegen die vollständigen Exporthashes von 0.13.1
verglichen; alle sechs SHA-256 sind identisch. Die Zielprüfung betrifft JSON einschließlich CSV-Spur und
Diagnosedaten, nicht bloß Endpositionen. Laufprotokolle, Hashvergleich und
Reproduktionskommandos: [Prüfnachweise](evidence/live_session_v0.14.0/README.md).

## Reproduzieren

```sh
cmake -S . -B build-session -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build-session --parallel 2
ctest --test-dir build-session --output-on-failure

cmake -S . -B build-session-asan -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS_DEBUG="-O1 -g1" -DMOTION_WITH_RUCKIG=ON -DMOTION_SANITIZE=ON
cmake --build build-session-asan --parallel 2
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir build-session-asan --output-on-failure
```

Fokussierter ThreadSanitizer-Lauf mit installiertem Clang 19 und dessen Runtime:

```sh
clang++ -std=c++20 -O1 -g1 -fsanitize=thread -fno-omit-frame-pointer -pthread \
  -I include tests/execution/test_live_motion_session.cpp -o /tmp/motion-session-tsan
TSAN_OPTIONS=halt_on_error=1 /tmp/motion-session-tsan
```

## Verbleibende Grenzen

Dieses Paket konkretisiert Besitz, synchronen Annahmevertrag und
Software-Shutdown. Es bestätigt keinen physischen Stillstand und plant beim
Shutdown keine Bremsbahn. Der Host muss weiterhin Maschinenfreigabe, Stopreaktion,
Folgefehler, partielle Übertragung und tatsächliche Antriebsbestätigung behandeln.
Asynchrone Treiber-Acks, Modell-/Kalibrierwechsel und deren Anlagenpolicy bleiben
Integrationsentscheidungen. Es gibt keine automatische Wiederfreigabe nach Fehler.

Die virtuellen, gesteuerten Zwei-Thread-Tests sind kein Scheduler-/Pi-WCET-Nachweis.
Kein ARM64-/Hardwarelauf und keine TSan-Abdeckung aller Bibliothekskomponenten
werden daraus abgeleitet. Speicherfreigabe ist nach bestätigter Quieszenz erlaubt;
das Sitzungsobjekt selbst darf erst nach Ende aller Aufrufe zerstört werden.

F11 aus dem Review ist für Session und Producer verbessert; andere Planer-
Diagnosen und verstreute Zulassungspolicies sind damit noch nicht vereinheitlicht.
Die nächsten Pakete sind Modulgrenzen und Ressourcen/CI; siehe [Roadmap](../planning/roadmap.md).
