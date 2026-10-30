# Test- und Abnahmeplan für die spätere Integration

Derzeit wird die Bibliotheksbasis vorbereitet. Eure selbst entwickelte Steuerung
und der Taktbereich **1–20 ms** sind bekannt; konkrete Scheduler-, Achs- und
Transportadapter werden erst später entworfen. Die folgende Liste trennt
vorhandene Softwareprüfungen von noch zu vereinbarenden Host-Verträgen.

## Schon ausführbar

| Prüfgebiet | Vorhandener Nachweis |
| --- | --- |
| Kinematische Grenzfälle | Singularitäts-/Grenzannäherungen, Winkelumschläge, π-Orientierungen, kurze Linien, volle Kreise |
| Unabhängige Kinematik | 1.866 FK/J-Vergleiche mit KDL sowie analytische und Differenz-Fixtures |
| Programmzulassung | Gesamtplan, Quellorte, Limits, transaktionale Ablehnung, CSV-Veröffentlichung |
| Ausführung | Zeit-/Datenfehler, Live-Aktivierung, Generationen, Fence, Snapshot, SPSC-Stress |
| Speicher | Beobachtete null C++-new/delete im Dispatcher-/Aktivierungspfad |
| Fehlerhafte Beobachtungen | Alter, Kausalität, Objektzuordnung, Revision, Ausreißer, vorbereiteter Halt |
| Zufall/Fuzzing | Reproduzierbare Programme, Parsermutationen, Ressourcenbudgets, libFuzzer |
| Neue Struktur | Kompatibilitätsheader, Diagrammdeklarationen, Links, separates CMake-Anwenderprojekt |
| Host-Vertrag als Simulation | Cacheausgabe bei 1/10/20 ms, spätes Zeitfenster, abgelehnte Achsübergabe, bestätigter Shutdown |
| Verwaltete Lebensdauer | Freigabe außerhalb RT, Rücknahme, ungelesene Antworten, laufende Übergabe beim Shutdown, 200 Zwei-Thread-Wechsel |

Die konkreten ausgeführten Konfigurationen und Sanitizer-Ergebnisse stehen im
[Prüfbericht](../testing/validation.md). Diese Tests decken Softwareverträge ab,
keine reale Maschinenabnahme.

## Vor einem echten Steuerungstest verbindlich festlegen

| Thema | Benötigte Entscheidung / Schnittstelle | Warum noch offen |
| --- | --- | --- |
| Host-API | C++-Prozess, Threadmodell, Compiler/ABI oder notwendiger Sprachadapter | Eigenentwicklung noch nicht konkret angebunden |
| Takt | Fester gewählter Wert in 1–20 ms, Startphase, Scheduling und Fehlerschwellen | Bereich bekannt, Betriebspunkt und Lastfall nicht gewählt |
| Uhrbezug | Monotone Domäne, Unsicherheitsgrenze, Epochbindung und Startvorlauf | Kein automatischer Uhren-/Startadapter vorhanden |
| Achsabbildung | Namen/Reihenfolge, Skalierung, Vorzeichen, Nullpunkte, m/rad ↔ Steuerungseinheiten | Nominalmodelle enthalten keine Host-Encoderzuordnung |
| Startfreigabe | Frische Istwerte, Grenz-/Folgefehler, Ruhe, Referenzierung, Kalibrierung | Dispatcher ist Sollwertausführung, kein Maschinen-Freigabeautomat |
| Achsannahme | Konkrete Abbildung des synchronen Session-Sink-Vertrags, maximale Latenz und partielle Übertragung | Softwarevertrag vorhanden; Treiber-/Hardwareadapter bewusst später |
| Fehlerreaktion | Verhalten bei Zeitfehler, Unterlauf, Treiberfehler, Abbruch, Verlust von Feedback | Neue normale Befehle zu sperren definiert noch keinen physischen Halt |
| Rücksetzung | Wer quittiert; wie neuer Startzustand, neue Epoche und Generation entstehen | Kein automatischer Reset/Resume im Executor |
| Beobachtung | Nichtblockierende Status-/Fehlerübergabe, Logformat und Zeitkorrelation | UI-/Diagnosekanal gehört zur Host-Steuerung |
| Kalibrierwechsel | Aktive Programme halten/beenden; Revisionen an Programmauswahl binden | Cache wird durch neue Frame-Revision nicht automatisch gestoppt |
| Speicher | Maximale Programmlänge, Cache- oder Ringmodus, gleichzeitig lebende Generationen | Abhängig von gewähltem Takt und Host-RAM |

C-ABI, Shared-Memory-/Netzwerkprotokoll, automatischer Epochadapter und Hardware-
Freigabe werden hier **nicht als fertig implementiert** dargestellt. Ein
installierbares CMake-Paket, formale API-/ABI-Stabilität und plattformspezifische
Build-/Deploymentvorgaben wären weitere Integrationsentscheidungen. Für den
aktuellen Quellcodeeinbau ist `add_subdirectory` nachweislich nutzbar.

## Gestufter Testablauf

1. **Host ohne Hardware:** Bibliothek einbinden, Achsnamen/Dimensionen prüfen,
   mit virtueller Uhr bei 1, 10 und 20 ms ausführen. Jeder Tick genau einmal;
   letzter Befehl terminal und ruhig. Fehlerszenarien dürfen keine weiteren
   normalen Befehle erzeugen. Eigene Programme zuerst mit `--check` und CSV prüfen.
2. **Zielplattform ohne Bewegung:** Gleicher Binary-/Compilerstand, reale monotone
   Uhr, Ziel-Lastfälle, Scheduler, Speicher und Achsübergabe messen. Kein Catch-up,
   auch bei Verzögerung/Preemption nicht. Timestampadapter und Startvorlauf testen.
3. **Signalpfad mit deaktivierten Achsen:** Einheiten/Mapping an unabhängigen
   Testwerten prüfen. Treiberablehnung, partielle Übertragung, alte Rückmeldungen,
   Queuefüllung und Abbruch gezielt injizieren. Commit-Fence und Diagnose abgleichen.
4. **Begrenzter Bewegungsversuch:** Erst nach Host-Freigabeprüfung mit für den
   Aufbau festgelegten Grenzen und Stopreaktionen. Soll-/Istwerte und tatsächliche
   Stillstandsbestätigung erfassen. Die Freigabe des Versuchs gehört zur Anlage.
5. **Live-/Following-Funktionen:** Erst nach stabilem Grundpfad überlappende
   Generationen, Verlust von Updates und Kalibrierwechsel erproben. Speicher
   erst nach korrekter Retirement-Antwort freigeben.

## Messprotokoll pro Versuch

Mindestens Software-Commit, Modell-/Profilhash, Kalibrierungsrevision, Achsabbildung,
Build-Optionen, Zielsystem, gewählter Takt und Timingpolicy dokumentieren. Pro
Befehl: globale Ticknummer, Generation, Sollzeit, beobachtete Ausgabezeit,
Uhrunsicherheit, Annahmezeit/-ergebnis, relevante Istwerte und Fehlerzustand.
Logs über einen separaten Kanal abführen; Dateischreiben/Formatieren nicht in den
beobachteten Zyklus mischen. Maximale und verteilte Latenzen unter definierten
Lastfällen berichten, keine bloßen Mittelwerte als obere Grenzen ausgeben.

Ein erfolgreicher Test muss für seinen konkreten Vertrag zeigen: kein alter oder
doppelter neuer Befehl, kein Weiterlaufen nach verriegeltem Fehler, passende
Anfangs-/Endzustände und vollständige Diagnose bei jeder Ablehnung. Timing und
Istbewegung sind getrennte Messgrößen.
