# Aktuelle Ergänzung — Architekturkorrekturen 0.13.1, 30.10.2026

F01/F02/F06/F09 des Architektur-Reviews sind mit Regressionen bearbeitet.
Die Vollkonfiguration besteht **30/30** CTests in Release und ASan/UBSan;
der getrennte Viewer/Editor besteht **32/32** Python-Tests in beiden nativen
Konfigurationen. Minimal ohne Serial/Ruckig: **11/11**. Editor-/DXF-Browserabläufe,
externer Consumer und 37 einzeln kompilierte öffentliche Header sind geprüft.
Sechs vollständige Beispielspuren sind gegenüber dem vorherigen Exporter bytegleich.
Verträge, Einschränkungen und Nachweise: [Korrekturpaket](architecture_review_fixes.md).

## DXF-Befehl 0.13.0, 29.10.2026

Die Vollkonfiguration mit Serial/Ruckig/KDL besteht **29/29** CTests im Release-
und ASan/UBSan-Build. Die getrennte Werkzeugprüfung umfasst zusätzlich **30**
Python-Tests für Viewer und lokalen Editor. DXF-Geometrie, Programmbefehl,
Quelldateien und Browserablauf: [DXF-Prüfbericht](dxf_validation.md).
Die bisherige Struktur-/Integrationsabnahme bleibt nachstehend als eigener
Versionsnachweis erhalten.

## Struktur- und Integrationsgrundlage — 0.12.3, 29.10.2026

Aufgabenbezogene Projektstruktur, Benennungsregeln und Integrationsverträge für
eine selbst entwickelte Steuerung mit **1–20 ms**. Die konkrete Anbindung wird
später geplant. Einstieg: [Dokumentation](../README.md),
[Handbuch](../integration/controller_integration.md),
[UML-Klassendiagramme](../architecture/class_diagrams.md).

Linux, GCC 14.2.0, CMake 4.4.4, C++20; eigene Targets mit strikten Warnungen,
aktive Test-Assertions auch in Release. Alle C++-Abhängigkeiten aus lokalen
hashgebundenen Quellarchiven.

| Serial | Ruckig | Konfiguration | Ergebnis |
| --- | --- | --- | --- |
| ON | ON | Release | **27/27** |
| ON | ON | Debug ASan/UBSan, -O1 -g1 | **27/27** |
| ON | OFF | Standard-Release | **25/25** |
| OFF | OFF | Minimal-Release | **10/10** |
| OFF | ON | Ruckig ohne Serial | **12/12** |

Zusätzliche Nachweise dieses Pakets:

- Alle **34 bisherigen flachen Header** bleiben Forwarder; alte und neue Pfade
  sowie der Alias `TrackingTime = ClockTime` werden gemeinsam kompiliert.
- **36 kanonische Header einzeln** kompiliert, einschließlich optionaler Ruckig-
  Header; zusätzlich 32 im normalen Serial-Build ohne Ruckig. Damit sind
  versteckte Abhängigkeiten durch zufällige Include-Reihenfolge geprüft.
- **Separates CMake-Anwenderprojekt** mit Serial-Standard sowie minimal ohne Serial
  gebaut und ausgeführt. Tests, Beispiele und KDL erscheinen darin nicht als
  ungewollte Targets. Minimaler externer Consumer auch mit ASan/UBSan erfolgreich;
  die Sanitizer-Compiler-/Linkoptionen werden über das öffentliche Target vererbt.
- Neuer ausführbarer Vertragstest: vollständige Cacheausgabe bei **1/10/20 ms**,
  Fehler durch verspäteten Takt und verweigerte Achsannahme. Fehlerpfad fordert
  genau einmal einen Host-Stopp an und gibt danach keine weiteren normalen
  Befehle aus. Ideale virtuelle Uhr und simulierte Achsen, kein Timingbenchmark.
- **Sieben UML-Sichten**: Deklarations-/Methodennamen gegen Header geprüft,
  DOT/SVG lokal erzeugt. **80 Migrationsziele** und lokale Dokumentationslinks
  geprüft. Offline-HTML wird aus den Texten und SVGs erstellt und prüft seine
  internen Anker; es lädt keine externen Ressourcen.
- Das vollständige URDF-/Profilbeispiel aus dem neuen Modellhandbuch mit einem
  PTP-Programm importiert, geplant und virtuell ausgeführt.
- Verschobener Referenzmodellgenerator reproduziert Modelle, Profile, FK-Fixtures
  und Beispielprogramme **byteidentisch**.
- Clang 19.1.7/libFuzzer: neuer Pfadaufbau gebaut, neun Korpuseingaben direkt
  wiedergegeben und **10.000 weitere Mutationen** mit ASan/UBSan ohne Crash,
  Timeout oder Befund ausgeführt. Die früheren 150.000 Eingaben bleiben ein
  versionierter Nachweis von 0.12.2, kein erneut behaupteter Lauf dieses Pakets.
- C++-Formatprüfung, Python-Syntax und CI-Konfiguration geprüft. Die CI enthält
  zusätzlich Struktur-/Dokumentationsprüfung und externes Anwenderprojekt.

Lokal: `ASAN_OPTIONS=detect_leaks=0`, `UBSAN_OPTIONS=halt_on_error=1`;
LeakSanitizer ist in dieser Prozess-Trace-Umgebung nicht als geprüft angegeben.
Die Ergebnisse betreffen Software und Simulation. Die Handbücher benennen
Startfreigabe, Zeitbezug, tatsächliche Achsannahme, Istwert-/Fehlerreaktion und
Zielplattformmessung als noch zu vereinbarende Host-Verträge.

---

# Robustheitsprüfung — 0.12.2, 29.10.2026

Alle vier beauftragten Software-Prüfpakete sind umgesetzt. Details und Replay:
[ROBUSTNESS.md](robustness.md). Linux, GCC 14.2.0, CMake 4.4.4, C++20;
`-Wall -Wextra -Wpedantic -Werror`, aktive Assertions auch in Release.
Die Abhängigkeiten einschließlich der testweisen KDL-Referenz wurden aus
lokalen, hashgebundenen Quellarchiven gebaut.

| Serial | Ruckig | Konfiguration | Ergebnis |
| --- | --- | --- | --- |
| ON | ON | Release, KDL-Referenz ON | **21/21** |
| ON | ON | Debug, ASan/UBSan, -O1 -g1 | **21/21** |
| ON | OFF | Standard-Release, KDL-Referenz ON | **19/19** |
| OFF | OFF | Minimal-Release | **9/9** |
| OFF | ON | Ruckig ohne Serial | **11/11** |

- 360 lokale IK-Grenzfallziele: 137 zugelassen, 223 mit Diagnose abgelehnt;
  zusätzliche kurze/Null-Linien, volle Kreise, Quaternion-Vorzeichen und Grenzverletzungen.
- Unabhängiges Orocos KDL 1.5.1: **1.866 FK/J-Vergleiche**, größte absolute
  Abweichungen 3,56·10⁻¹⁵ / 3,11·10⁻¹⁵.
- Fehler während der Ausführung, verspätete/fenced Wechsel, belegte Übergaben,
  doppelte/ungültige Befehle, 100 zufällige Pufferabläufe;
  stale-/Kalibrierungswechsel mit Auslauf in den vorgeprüften Halt.
- **120 Zufallsprogramme**, 59.506 ausgegebene Befehle, 240 transaktional
  abgelehnte Übernahmen; **12.000 Parser-Mutationen** (132 akzeptiert,
  11.868 abgelehnt), explizite Ressourcenbudgets und Replay.
- Clang **19.1.7**, libFuzzer plus ASan/UBSan: **50.000 + 100.000** Eingaben,
  zweiter Lauf auf fortgesetztem/ergänztem Korpus; keine Crashes, Timeouts oder
  Sanitizer-Befunde. Zweiter Lauf: 71 s, 5.338 Coverage-Punkte (`cov`) laut libFuzzer,
  912 Korpuseinträge / 267 KiB, RSS 459 MiB. Diese Coverage-Zahl ist kein Prozentwert.
- Zusätzliche CLI-Prüfung: bereits vorhandene CSV bleibt sowohl bei überschrittenem
  Tick-Budget als auch bei einem ungültigen späten Programmschritt byteidentisch;
  keine übrig gebliebenen temporären Dateien.
- Gefundene und behobene Fehler: XYZ-Gantry muss beide Identitätsquaternion-
  Vorzeichen akzeptieren und nichtendliche Posen/Gelenke/Seeds ablehnen.

Lokal: `ASAN_OPTIONS=detect_leaks=0`, `UBSAN_OPTIONS=halt_on_error=1`.
LeakSanitizer ist wegen der Prozess-Trace-Umgebung nicht als geprüft angegeben.
Die CI-Konfiguration enthält die fünf Build-Varianten und zusätzlich einen
50.000er-libFuzzer-Job; diese Tabelle berichtet die tatsächlich lokal ausgeführten
Cloud-Prüfungen, keinen noch unbestätigten GitHub-Actions-Lauf.

Die nominalen Robotermodelle, gemeinsame Modellparameter beim KDL-Vergleich,
Engineering-Intervallhüllen und lokale IK behalten ihre dokumentierten Grenzen.
Hardware, Raspberry-Pi-Echtzeit und CANopen wurden nicht geprüft; CANopen bleibt
wie beauftragt ausgeklammert. Die folgenden Ergebnisse sind historisch.

---

# Erneute Prüfung der Rekonstruktion — 29.10.2026

Linux, GCC 14.2.0, CMake 4.4.4, C++20; eigene Targets mit
`-Wall -Wextra -Wpedantic -Werror`. Assertions sind auch in Release aktiv.
Fremdheader sind System-Includes; ihre Quellen bleiben unverändert.
Diese Ergebnisse gehören zum **rekonstruierten** Stand 0.12.1.

| Serial | Ruckig | Konfiguration | Ergebnis |
| --- | --- | --- | --- |
| ON | ON | Release | 16/16 |
| ON | ON | Debug ASan/UBSan, -O0 -g1 | 16/16 |
| ON | OFF | Standard-Release | 14/14 |
| OFF | OFF | Minimal-Release | 8/8 |
| OFF | ON | Ruckig ohne Serial | 10/10 |

Alle Konfigurationen wurden mit gebündelten Quellarchiven gebaut. Ruckig wurde
in der finalen Matrix ebenfalls aus dem lokalen versions-/hashgebundenen Archiv
gebaut, nicht aus einem versteckten alten Build-/Netzwerkcache. Sanitizer:
`ASAN_OPTIONS=detect_leaks=0`, `UBSAN_OPTIONS=halt_on_error=1`.
LeakSanitizer wird unter der lokalen Prozess-Trace-Umgebung nicht als geprüft behauptet.

## Nachweise

- **Seriell:** unabhängige homogene FK-Fixtures für drei Modelle und drei
  Konfigurationen; 45 lokale IK-Rücklösungen; J*dq-Identität sowie zentrale
  Differenzen für v/a/Ruck auf zeitabhängigen q/dq/ddq. Ein analytischer
  Einachs-Pendel-/Rotationsfall prüft die Ableitungskette unabhängig.
- **Import:** verpflichtende SI-Limits, Duplicate-/Unknown-/Tag-Rejection,
  Unsupported-Mimic- und ungültige Dimensionen. Modell-Kinematik und Motion-
  Grenzen bleiben getrennt; kein automatisches Makro-/Skript- oder Mesh-Laden.
- **Polynome/Hüllen:** geschlossene Quintik-/Ruck-Referenz, endliche Bereiche,
  zu schnelle Kandidaten, Intervallarithmetik, klar freie Endpunkte mit
  Hindernis im Inneren. Zulassung prüft kontinuierliche Hüllen; Samples
  sind ergänzende Regression, kein Beweisersatz.
- **SCARA/Delta/Blends:** FK/IK und zeitliche Ableitungsketten, unabhängige
  symmetrische Delta-Kugelgeometrie; Delta LIN/CIRC-Splines im Pose-Schlauch,
  konservative Zeitplanung und ein Blend mit nichtverschwindender Tangente
  am inneren Wegpunkt.
- **Programme:** strikte Schema-/Werte-/Quellortprüfung, Alias-/Tag-/Mehrdokument-
  Rejection, fehlende/rekursive CALLs, Größen-/Nesting-/Expansionsbudgets,
  unabhängige XYZ-LIN/CIRC-Geometrie, WAIT, Geschwindigkeitsreduktion und PTP-
  TCP-Grenzen. Ein Fehler in Schritt 2 verhindert ein neues Gesamtprogramm;
  der vorher gültige Plan bleibt unverändert.
- **Ausführung:** exakter Live-Grenzzustand, Generation, Aktivierung einmalig,
  Delivery-Fence-Rejection, atomarer paralleler Snapshot-Leser; 10.000
  Zwei-Thread-Postfachanfragen und 100.000 SPSC-Einträge. Deadline-/Zweiuhr-
  Preemption-Fehler verriegeln und schreiben keinen neuen Ausgabebefehl.
  Kein ThreadSanitizer-/WCET-Zertifikat wird behauptet.
- **Allokationen:** abgefangene C++-new/delete-Aufrufe zeigen null Allokation
  oder Freigabe während Cache-Dispatch, Live-Aktivierung, Snapshot, Fence
  und Acknowledgement. Konstruktion, Planung, Samplen und Freigabe bleiben
  außerhalb des beobachteten zyklischen Bereichs. Kein allgemeiner malloc-
  oder Benutzer-Callback-Nachweis.
- **Stopps:** geschlossene Dauer-/Wegformeln für beide Vorzeichen sowie
  Dreiecks-/Beschleunigungsplateau-Fälle; 300 bewegte Zufallszustände;
  Bernstein-Phasenhüllen gegen Samples, innere Positions-/Geschwindigkeits-
  Verletzungen, Rest-Padding, vorbereiteter Prefix-Stop und Online-Retargeting
  mit bewegten Endpunkten. Kandidaten werden unabhängig integriert/geprüft.
- **Tracking:** Frame-Rotation/Komposition, positive Uhrunsicherheit, absolute
  Encoder-Offset-Kompensation, stale/clock/causal/horizon Rejection, Historien-
  Ausreißer. Beschleunigungsänderungsfehler werden mit Encoder-Skalierung
  berücksichtigt. Unmögliche TCP-Horizonte und ein Halt mit überhöhter
  Winkelgeschwindigkeit werden abgelehnt.
- **Following:** UR5e, ABB und FANUC erreichen jeweils 29 überlappende
  Aktivierungen ohne Zwischenhalt und danach den vorbereiteten Halt.
  Maximaler nominaler Simulationsfehler ungefähr 0.230/0.231/0.231 µm.
  Das ist Modellübereinstimmung, keine physische Genauigkeit.
- **Kalibrierung:** gewichteter starrer Fit, unabhängiger zurückgehaltener Punkt,
  Richtung des Transforms, kollineare Daten und Ausreißer-Residuen.
- **Linux:** monotone Uhr, absoluter Termin in der Vergangenheit,
  Uhrdomänen-/Setup-Fehler und Statistik. Ein separater 20-Sample-/10-ms-
  Host-Probe beobachtete maximal etwa 0.098 ms Verspätung und null vollständig
  verpasste Perioden. Das ist kein Raspberry-Pi-/CAN-/Drive-Nachweis.

## Beispiele und Export

| Rekonstruiertes Programm | Ausgerichtete Dauer | Befehle |
| --- | --- | --- |
| UR5e | 9.86 s | 987 |
| ABB IRB120 | 10.38 s | 1039 |
| FANUC LR Mate 200iD | 9.97 s | 998 |

Alle Programme haben fünf Stillstandsgrenzen und terminal dq/ddq=0 ohne
Dispatch-Fehler. Laufzeiten unterscheiden sich von den verlorenen Originalen,
weil Hüllen/Implementierung neu rekonstruiert wurden.

Zusätzliche CLI-Prüfungen: anderes Arbeitsverzeichnis, `--check` ohne Playback,
ungültige Argumente, begrenzte Simulations-Ticks und unveränderte bestehende CSV
bei Ablehnung. Export enthält alle geordneten Ticks, absolute Zeit und terminalen
Stillstand. Der optionale Matplotlib-Plot wurde als PNG erzeugt und visuell
geprüft; er ist keine C++-Build-Abhängigkeit.

Alle zehn neu gepinnten öffentlichen Quelldateien bestehen ihre SHA256-Prüfung;
die Offline-Regeneration der Modelle, FK-Fixtures und Beispielprogramme ist
byteidentisch. Lizenzen und neue Quellprovenienz liegen bei.

## Grenzen

Quelltext-/ABI-/Testergebnisidentität mit der verlorenen v0.12.0 wird nicht
behauptet. CANopen ist bewusst nicht rekonstruiert. Vollständige Payload-/Torque-
und Link-/Selbstkollisionsmodule sowie einige frühere Komfort-/Benchmark-APIs
fehlen. Dafür werden keine stillen Ersatzgarantien ausgegeben.

Keine reale Hardware war verbunden. Physische Kalibrierung, Bus-/Antriebs-
Interpolation, unabhängige Sicherheit und harte Echtzeit bleiben externe
Abnahmearbeit. Numerische Hüllen sind keine zertifizierte Intervallarithmetik;
IK und Stopps lösen lokale Kandidaten, keine globale Feasibility.

Die GitHub-CI ist eingerichtet; diese Tabelle beschreibt lokale Prüfungen.
Ein Remote-CI-Ergebnis wird nur nach tatsächlicher Beobachtung behauptet.
