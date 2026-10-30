# Implementierungs- und Integrationsroadmap

## Architektur-Review: nächste Schritte

Basis ist das [Review von 0.13.0](../architecture/architecture_review_v0.13.0.md).
Das [erste Korrekturpaket in 0.13.1](../testing/architecture_review_fixes.md) behandelt
die Befunde **F01, F02, F06 und F09**: Providerformen, Programm-/Rasteridentität,
älteres Tracking und Ergebnisveröffentlichung. Größere Strukturänderungen sind
damit noch nicht abgeschlossen.

Mit **0.14.0** ist das zweite Paket [verwaltete Planübergabe](../integration/live_motion_session.md)
umgesetzt: Cachebesitz außerhalb RT, Annahme/Commit, Rücknahme von Angeboten,
strukturierte Session-/Producer-Diagnosen und bestätigter Shutdown. Nachweise:
[Lifecycle-Prüfung](../testing/live_session_validation.md). Ein physischer Halt,
asynchrone Antriebs-Acks und konkrete Host-Freigaben werden damit nicht vorweggenommen.

Mit **0.15.0** ist Paket 3 [Modulgrenzen und Komponenten](../architecture/components.md)
umgesetzt: niedrigere Verträge und Mathematik, keine Datei-/Modul-/CMake-Zyklen,
separate Bibliotheken und unabhängig geprüfte Verbraucher. Legacy-Fassade,
Headerpfade und vollständiges Serial-Archiv bleiben erhalten.
Nachweise: [Komponentenprüfung](../testing/components_validation.md).

Mit **0.16.0** ist Paket 4 [Ressourcen und Abbruch](../integration/preparation_resources.md)
umgesetzt: gemeinsamer Vorbereitungskontext, DXF-Parsecache, Cache-Zulassung nach
Bytes, CSV-Spooling und zweipassige Spurprüfung. Der Editor kann einen laufenden
Auftrag abbrechen und teilt ein Gesamtzeitbudget bis zur Veröffentlichung.
Browser-/TSan-/ARM64-Jobs sind in CI ergänzt. Lokale Tests und verbleibende
Grenzen: [Ressourcenprüfung](../testing/resources_validation.md).

Die nächsten Schritte:

1. **Zielplattform und Betriebsbudgets konkretisieren:** ARM64-CI-Läufe beobachten,
   Pi-/Host-Lastmessungen mit festgelegten Programmen durchführen sowie globale
   RAM-/Scratch-/Parallelitätslimits und maximale Abbruchlatenz für die Anlage
   festlegen. Der Vorbereitungskontext liefert Zulassungszähler, keine Prozess-
   RSS-Garantie oder WCET. CANopen bleibt zurückgestellt.
2. **Bedarfsgerechte Erweiterungen:** typisierte Quell-/Bewegungsrepräsentation,
   Enclosure-Fähigkeit und das separat offene IK-Skalierungsproblem F07.
   F11 ist für Session/Producer verbessert; durchgehende Planerdiagnosen und
   benannte Zulassungspolicies bleiben offen.

CANopen und die konkrete Steuerungsintegration bleiben zurückgestellt.

## Funktionsstand

Mit **0.13.0** ist DXF-Geometrie als YAML-Programmbefehl ergänzt: Import,
Einheiten/Layer, TCP-/Weltplatzierung und Planung über LIN/CIRC/PTP. Der getrennte
Testeditor unterstützt Dateiimport, Simulation und YAML+DXF-Bündel.
Details und bewusst begrenzter Formatumfang: [DXF-Befehl](../formats/dxf_program_command.md).

Mit 0.12.3 sind die aufgabenbezogene Struktur, Benennungs-/Migrationsregeln,
sieben UML-Klassendiagramme, das Integrations-/Datenhandbuch und ausführbare
Vertragstests für 1/10/20 ms ergänzt. Die konkrete Anbindung an die selbst
entwickelte Steuerung folgt später. Offene Host-Schnittstellen und gestufte
Prüfungen stehen im [Integrationsplan](../integration/acceptance_plan.md).

## Jetzt rekonstruierte Software

- Modell-/Programmimport, Sechsachs-Solver und drei öffentlich belegte Referenzmodelle.
- PTP/LIN/CIRC, rotierende Posen, SCARA/Delta-Referenzmodelle und numerische
  Spline-Abbildung, C2-Wegpunktblends und konservative Zeitplanung.
- Ganzzahlige Zyklusraster, gepufferte Ausführung, Zeit-/Datenfehler, Live-Planwechsel,
  Übergabegrenzen und coherent atomare Fortschrittssnapshots.
- Optionale geprüfte Gelenkhaltephasen, vorbereitete Übergänge und bewegte Horizonte.
- Statische Frame-Kalibrierung, Unsicherheits-/Historien-/Ausreißergrenzen und
  kontinuierliches XYZ-/Sechsachs-Tracking mit vorher geprüftem Halt.
- YAML-Anwenderprogramme mit benannten Punkten, Geschwindigkeit, WAIT/CALL;
  Simulation, CSV und optionale wissenschaftliche Plots.
- Kalibrier-Fit, TCP-Hindernishüllen, Linux-Uhr-/Zeitdiagnostik, Git-Sicherung.

## Noch nicht wiederhergestellt oder weiterhin offen

Die vier zusätzlich beauftragten Software-Prüfpakete sind mit 0.12.2 umgesetzt:
kinematische Grenzfälle, unabhängiger KDL-FK/J-Vergleich, Ausführungsfehler und
reproduzierbare Zufallsprogramme/Parser-Fuzzing. Umfang, Seeds und Grenzen:
[ROBUSTNESS.md](../testing/robustness.md). Diese Tests laufen künftig bei Änderungen in CI;
längere Fuzz-Kampagnen können mit demselben Harness fortgesetzt werden.

1. Frühere zusätzliche Payload-/Drehmoment- und vollständige Link-/Selbstkollisions-
   Module sowie einzelne ältere Komfort-/Benchmark-APIs. Aktuelle Programme
   behaupten diese Prüfungen nicht. Die zugrundeliegenden inertialen/Geometriedaten
   müssen ohnehin explizit geliefert und validiert werden.
2. Digitale Hardware-I/O, eingangsabhängige WAITs, Bedingungen/Schleifen, Hersteller-
   und G-Code-Übersetzer, sensorabhängige Programmflüsse und Live-Speed-Override.
   Das waren auch im zuletzt beschriebenen Programmformat weitere Erweiterungen.
3. Physische Zielplattform-/Antriebsprüfung und CANopen sind vorerst zurückgestellt.
   Ein Host-Timing-Probe ist kein Pi-/Bus-/Drive-WCET-Nachweis.
4. Formale Intervallzertifizierung, globale Zeitoptimierung, analytische roboter-
   spezifische IK-Branch-Enumeration, vollständige constrained-stop Feasibility
   und robuste Schlupf-/Kontakt-/dynamische Frame-Fusion bleiben Spezialpakete.
5. Reale Kalibrierung, Payload/Obstacles, Antriebsskalierung und unabhängige
   Maschinensicherheit sind echte Installations-/Abnahmearbeit.

Diese Liste trennt rekonstruierte Software, nicht rekonstruierte Zusatzmodule
und grundsätzlich hardwareabhängige Nachweise. Keine physische Prüfung gilt
wegen einer erfolgreichen Simulation als bestanden.
