# Werkzeugprüfung 0.5.1 · 30.10.2026

**41/41 Python-Tests** und drei echte Chromium-Abläufe bestanden. LIN hält jetzt
standardmäßig die tatsächliche TCP-Orientierung am Segmentanfang. Die neue
Editorvorlage startet mit Achse 1 bei 2 rad; Browser-FK und sämtliche nativen
LIN-Samples bestätigen den konstanten Verlauf. Explizite Interpolation und
Ablehnung des nicht zugelassenen konstanten Fernziels sind geprüft.
[LIN-Prüfbericht](../../docs/testing/lin_orientation_validation.md).

## Vorheriger Nachweis 0.5.0 · 30.10.2026


**38/38 Python-Tests** und drei echte Chromium-Abläufe lokal bestanden.
Neue Prüfungen: nativen Worker abbrechen/reapen, Gesamtdeadline nach Rendering,
Abbruch während der Python-Spuraufbereitung, falsche/alte Jobkennung, Erhalt der
vorherigen Ansicht sowie 100.000 Spurzeilen ohne vollständige numerische Listen.
Browser-Smokes und Jobprotokolle sind jetzt in CI eingebunden.
[Ressourcenprüfung und Grenzen](../../docs/testing/resources_validation.md).

## Vorheriger Nachweis 0.4.1 · 30.10.2026


**32/32** Python-Tests mit Release- sowie ASan/UBSan-Exporter bestanden.
Zwei neue Tests prüfen IO-Fehler vor der Veröffentlichung und beim späteren
Aufräumen abgelaufener Ergebnisse. Editor- und DXF-Browserprüfungen mit echtem
Chromium bestanden ebenfalls; keine Browserfehler oder externen Anfragen.
Die geprüfte Retiming-API der Bibliothek ersetzt den freien Schedule-Austausch.
Details: [Architekturkorrekturen](../../docs/testing/architecture_review_fixes.md).

## Vorheriger Nachweis 0.4.0 · 29.10.2026

DXF-Erweiterung: drei zusätzliche Python-Tests (insgesamt **30**), native
DXF-Befehle, bytegleiche YAML+DXF-ZIP-Bündel, Asset-/Pfadgrenzen und Erhalt der
vorherigen Ansicht bei späteren CAD-Fehlern. Release und ASan/UBSan bestehen
jeweils alle 30 Tests. Die Bibliothek besteht zusätzlich 29/29 CTests in beiden
Konfigurationen. Details: [DXF-Prüfbericht](../../docs/testing/dxf_validation.md).

Der zusätzliche echte DXF-Browsertest prüft Dateiimport, Einfügen eines einzigen
YAML-Befehls vor nachfolgenden Root-Feldern, LIN/CIRC-Planung, CAD-Quellorte,
Mesh-Wiedergabe, Bahnfokus, Bundledownload, Wiederherstellung samt Zeichnung und
Stale-Markierung bei CAD-Wechsel. Fehlerhafte beziehungsweise entfernte Anhänge
ersetzen keine gültige Ansicht. Die unabhängige Dateiprüfung erfolgt mit
ezdxf 1.4.2; die Anwendung braucht dieses Paket nicht.

Die nachstehenden bisherigen Prüfungen bleiben enthalten; die Tabelle zeigt
die aktualisierte Werkzeug-Testzahl.

Der gesamte neue Viewer liegt unter `tools/motion_viewer/`. Die Implementierung,
öffentlichen Header und CMake-Konfiguration der Bewegungsbibliothek wurden für
dieses Paket nicht verändert. Native Daten entstehen durch deren vorhandene
Import-/Planungs-/Simulations-APIs, nicht durch eine zweite Bewegungsplanung.

| Prüfung | Tatsächlich ausgeführt |
| --- | --- |
| Standalone-CMake, GCC 14.2, Release | Gebaut; 30 Werkzeugtests bestanden |
| Standalone-CMake, Debug -O1 -g1, ASan/UBSan | Gebaut; dieselben 30 Werkzeugtests bestanden |
| Ein-Befehl-Einstieg `run_demo.py` | Build und fertige HTML erfolgreich erzeugt |
| Sollwertraster | 1 und 20 ms mit Referenzprogramm; Visual-Demo mit 10 ms und zusätzlich 1 ms (30.018 Befehle, 3.762 angezeigte Samples) |
| Visual-Demo | 3.007 ausgegebene Befehle, sechs Schritte, 30,06 s, vollständiger Kreis und terminale Ruhe |
| Renderer-FK gegen C++-Export | 6.139 Konfigurationen über UR5e, ABB IRB120, FANUC LR Mate sowie gemischte Kette |
| Größter Positionsfehler | 4,45·10⁻¹⁶ m (aufgerundet) |
| Größte Rotationsmatrix-Abweichung | 1,34·10⁻¹⁵ (aufgerundet) |
| Fehlerhafte Inputs / Exportbudget | Diagnosen; frühere JSON-/HTML-Ausgaben bleiben unverändert |
| Eingebettete Textfelder | Schließende Script-Tags, Unicode-Trennzeichen und Sonderzeichen bleiben inerten JSON-Texten zugeordnet |
| Browser: Chromium / Playwright | Keine JavaScript-Fehler und keine Netzwerkanfragen; Abspielen/Pause, Einzelschritt, Achs-/Einheitenwechsel, Diagramme, Segmentsprung, Kamera, terminaler Frame und Neustart geprüft |
| Mobiles Layout | 420 px, kein horizontaler Seitenüberlauf |
| Vollständiges UR5e-Visual-Modell | Sieben Links, 31 Primitive-/Materialgruppen, 121.789 Dreiecke; Materialien und zusätzliche Original-Linien erhalten |
| Herkunft und Integrität | Alle zwölf offiziellen Originaldateien, vendertes Three.js und abgeleitetes Mesh-Paket gegen SHA-256-Prüfsummen geprüft |
| Geometriepuffer | Endlichkeit, Indizes, Dreieckszahlen, normalisierte Normalen und physische Größenordnung geprüft |
| Konverter | Handberechnete Fixture mit hierarchischem Matrixversatz, nichtuniformer Skalierung, cm→m und invers transponierten Normalen; falsche Hochachse abgelehnt |
| Modellzuordnung | Veränderte Gelenkachsen, Namen, Typen, Linknamen und Montageversätze fallen auf Kinematik zurück; explizit verlangtes unpassendes UR5e wird abgelehnt |
| Visual-URDF | Alle sieben Mesh-Pfade lokal auflösbar; Gelenkdefinitionen gegenüber Bibliotheksmodell unverändert |
| Tatsächliche Three.js-Linkmatrizen | 42 Fälle gegen getrennte Three.js-Transformationen: drei Posen × sieben Visual-Links × originale/geänderte Basis- und Werkzeugpose; maximale Abweichung 0 |
| Fehlendes WebGL / Kontextverlust | Verweigerte WebGL-Kontexterzeugung und eingespeistes Kontextverlust-Ereignis: verständliche Meldung, Kinematik und Einzelschritt weiter bedienbar |
| Einzeldatei und Lizenzen | Keine externen Laufzeitanfragen; vollständige BSD-/MIT-Bedingungen und ursprüngliche Autorennennungen im HTML enthalten |
| Kinematik ohne Mesh-Renderer | UR5e mit `--visuals none` und ABB im Auto-Modus im Browser bedienbar; Three.js nicht eingebettet, keine JavaScript-Fehler oder Netzwerkanfragen |
| Reproduzierbare Assets | Erneute Konvertierung erzeugt `visuals.json`, `manifest.json` und `ur5e_visual.urdf` bytegleich; regulärer `run_demo.py`-Einstieg erzeugt dieselbe HTML-Datei wie die mitgelieferte Demo |
| Editor-Startbefehl | `run_editor.py` mit Build gestartet, tatsächlich ausgegebenen Loopback-Port per HTTP abgefragt und mit Strg+C sauber beendet |
| Editor: reale native Ausführung | Erfolgreiche Simulationen mit 1/20-ms-Raster, exakt erhaltene YAML-Schnappschüsse und unveränderliche HTML-Downloads geprüft |
| Editor: native Fehler | Später ungültiger PTP liefert Datei, Zeile und Schritt; vorige gültige HTML bleibt bytegleich, fehlgeschlagene Läufe hinterlassen keine Teilresultate |
| Editor: Importgrenzen | Fehlende/absolute/nach außen führende Modellpfade und Symlinks abgelehnt; Parser-Duplikate, ungültige Unicode-Eingaben, falsche Raster und Größenbudgets geprüft |
| Editor: Auftragsgrenzen | Belegter Platz liefert 409; injizierter nativer Timeout publiziert nichts und gibt den Platz frei; spätere echte Simulation erfolgreich |
| Editor: Ergebnisspeicher | Maximal drei erfolgreiche Ergebnisse; ältere URLs liefern 404; Vorschau und Download enthalten denselben Stand |
| Editor: HTTP | Echte Requests/Downloads; Host, Origin, Token einschließlich Nicht-ASCII-Token, JSON-Duplikate/NaN, Requestgröße und feste Dateirouten geprüft |
| Editor: Browser / Chromium | Echte UR5e-Simulation und Mesh-Wiedergabe, YAML-Import, Entwurf-/Schnappschuss-/HTML-Download, Fehlerzeilensprung, Entwurfswiederherstellung und Änderungen während einer echten Simulation geprüft |
| Editor: zweites Modell und Layout | ABB-Vorlage mit 20 ms simuliert; Kinematikfallback; 420 px ohne horizontalen Seitenüberlauf |

Die zehn neuen Editor-/HTTP-Tests ergänzen die 17 vorhandenen Werkzeugtests.
Der Editor-Browsertest lief mit einem echten temporären Loopback-Dienst und
dem realen Release-Exporter; externe Anfragen wurden blockiert und es entstanden
keine JavaScript-Fehler oder externen Netzwerkanfragen. Der Timeout selbst wurde
gezielt injiziert, um den Fehlerpfad ohne langes Warten reproduzierbar zu prüfen;
anschließend wurde derselbe Auftragsplatz mit dem echten Exporter verwendet.
Die Editorvorschau wurde aus diesem getesteten Browser aufgenommen.

Der Mesh-Browsertest lief mit Chromium und dessen Software-WebGL-Renderer
SwiftShader. Die Vorschau wurde aus dem tatsächlich gerenderten Viewer aufgenommen.
Mesh/Kinematik-Umschaltung, die sieben Baugruppen und ihre Befestigung an den
bewegten Gelenkrahmen wurden zusätzlich im Bild kontrolliert. Unterschiedliche
GPU-/Treiberkombinationen werden dadurch nicht als geprüft behauptet.

Die gemischte Kette verwendet feste, rotatorische und prismatische Gelenke,
beliebige Achsen, nichttriviale Origins sowie Basis- und Werkzeugtransformationen.
Der Abgleich überprüft die **Anzeige-FK**, keine physische Roboterkalibrierung.
Vor dem Anzeigen wird auch im Browser jede dargestellte Pose gegen ihre
exportierte TCP-Position und -Orientierung geprüft.

Alle exportierten Samples werden vor einer eventuellen Anzeigeausdünnung
auf Endlichkeit, Tick-/Zeitfolge, Vollständigkeit und globale Profilgrenzen
geprüft. Durchgehende Grenzzulassung bleibt Aufgabe des vorhandenen C++-Planers;
Samples und Browserplots sind kein Beweisersatz. Gleiche Ausgangsmodelle sind
keine unabhängige Vermessung der Maschine.

Die Visual-Geometrie ist das nominale offizielle UR5e-Arm-Modell ohne angebautes
Werkzeug. Eine sichtbare Durchdringung wird nicht automatisch als Kollision
erkannt. Geänderte Kinematik erhält dieses Modell nur mit einer passenden neuen
Visual-Referenz. Die neue Implementierung bleibt vollständig im Tool-Verzeichnis;
der Native-Exporter ergänzt lediglich die Linknamen seiner Diagnoseausgabe.

Lokal liefen Sanitizer mit `ASAN_OPTIONS=detect_leaks=0` und
`UBSAN_OPTIONS=halt_on_error=1`; LeakSanitizer wird in der Prozess-Trace-Umgebung
nicht als geprüft behauptet. Für den Browsertest wurde der fertige HTML-Inhalt in
eine leere Testseite geladen und jeder Netzwerkzugriff gesperrt. Eine Änderung
von Browserrichtlinien war nicht erforderlich.

Die Bibliotheks-Tests des vorherigen Stands wurden für unveränderten Code nicht
erneut als neu ausgeführt angegeben. Sie bleiben unter
[docs/testing/validation.md](../../docs/testing/validation.md) dokumentiert.
