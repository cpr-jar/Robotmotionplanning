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
