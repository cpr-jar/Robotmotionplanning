# Werkzeugprüfung 0.1.0 · 29.10.2026

Der gesamte neue Viewer liegt unter `tools/motion_viewer/`. Die Implementierung,
öffentlichen Header und CMake-Konfiguration der Bewegungsbibliothek wurden für
dieses Paket nicht verändert. Native Daten entstehen durch deren vorhandene
Import-/Planungs-/Simulations-APIs, nicht durch eine zweite Bewegungsplanung.

| Prüfung | Tatsächlich ausgeführt |
| --- | --- |
| Standalone-CMake, GCC 14.2, Release | Gebaut; zehn Werkzeugtests bestanden |
| Standalone-CMake, Debug -O1 -g1, ASan/UBSan | Gebaut; dieselben zehn Werkzeugtests bestanden |
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

Lokal liefen Sanitizer mit `ASAN_OPTIONS=detect_leaks=0` und
`UBSAN_OPTIONS=halt_on_error=1`; LeakSanitizer wird in der Prozess-Trace-Umgebung
nicht als geprüft behauptet. Für den Browsertest wurde der fertige HTML-Inhalt in
eine leere Testseite geladen und jeder Netzwerkzugriff gesperrt. Eine Änderung
von Browserrichtlinien war nicht erforderlich.

Die Bibliotheks-Tests des vorherigen Stands wurden für unveränderten Code nicht
erneut als neu ausgeführt angegeben. Sie bleiben unter
[docs/testing/validation.md](../../docs/testing/validation.md) dokumentiert.
