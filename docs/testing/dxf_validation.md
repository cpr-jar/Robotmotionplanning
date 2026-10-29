# DXF-Implementierung und Prüfung — 0.13.0 / Tool 0.4.0

Stand: 29.10.2026, Linux/GCC 14.2, C++20, strikte Warnungen und aktive Assertions.
DXF-Einlesen und Expansion liegen in `io/` und `program/`; Bedienung und Viewer
bleiben unter `tools/motion_viewer/`.

| Prüfung | Ergebnis / Umfang |
| --- | --- |
| Volle Bibliothek mit Serial, Ruckig und unabhängiger KDL-Referenz | Release 29/29; ASan/UBSan 29/29 |
| Native DXF-Importer-/Programmtests | Einheiten/Header/Overrides, LINE/POINT/ARC/CIRCLE, positive/negative Bulge bis >180°, alte/3D-Polylinien, geschlossene Kanten |
| Geometrie unabhängig kontrolliert | ezdxf 1.4.2 liest/auditiert die R2000-Beispielzeichnung ohne Fehler/Reparaturen; vier LINE und ein 90°-ARC, Radius 5 mm, Zentrum (25,5,0) mm; native Assertions gegen diese Werte |
| Befehlsvertrag | TCP-/Weltbezug mit Rotation, CALL-Wiederholung, einmaliges Laden gleicher Dateinamen, POINT-Verbindungsfahrten LIN/PTP, Standardablehnung von Sprüngen |
| Fehler und Ressourcen | Binär/NaN/Overflow/Duplikate, fehlendes EOF/ENDSEC/SEQEND, falsche Vertexzahl, unsupported Entities, Extrusion/Papierbereich, Eingabe-/Quellbudgets |
| Deterministische Importmutationen | 4.000 Eingaben mit Seed 0xDFF2026; Fehler oder gültige endliche Geometrie, keine Abstürze/Hänger |
| Clang-19-libFuzzer unter ASan/UBSan | Je 50.000 Durchläufe für DXF und YAML, Seed 4028751910; ohne Befund; eigene DXF-CI-Kampagne ergänzt |
| Tatsächliche UR5e-Ausführung | Beispiel mit einem DXF-Befehl → fünf Bewegungen + WAIT; vollständige Trace-Prüfung, dynamische Zulassung, terminaler Stillstand |
| Werkzeugdaten und Editor | 30 Python-Tests; YAML/DXF im Resultatbundle bytegleich, fehlende/nach außen führende Anhänge abgelehnt, vorige Ansicht bei späterem Geometriefehler unverändert |
| Realer DXF-Browsertest | Import, ein eingefügter Befehl, indentloser steps-Block vor points, echte native LIN/CIRC, Mesh-Wiedergabe, CAD-Quellorte, Bahn-Fokus, exaktes ZIP |
| Browserzustände | Entwurf mit DXF nach Neuladen wiederhergestellt; CAD-Ersetzen markiert Altstand; unsupported/missing DXF behält vorige Ansicht; 420 px ohne Überlauf |

Der Browsertest verwendet einen echten lokalen Python-Dienst und den echten
C++-Exporter. Externe Netzwerkanfragen sind blockiert; keine JavaScript-Fehler
oder externen Anfragen. Chromium rendert den Mesh-Viewer mit SwiftShader.
Die Referenzbibliothek ezdxf wird nur zur unabhängigen Entwicklungsprüfung
verwendet; die Anwendung benötigt sie nicht.

Reproduktion:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DMOTION_WITH_RUCKIG=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
python3 tools/motion_viewer/tests/dxf_browser_smoke.py
```

Die vorhandene Zulassung prüft Modellgrenzen; die DXF-Funktion ist kein Nachweis
einer physischen Werkzeugbahn oder Kollisionsfreiheit. Konfiguration,
Spline-Toleranz und Formatgrenzen sind im [DXF-Vertrag](../formats/dxf_program_command.md)
explizit beschrieben. Sanitizer wurden lokal mit `ASAN_OPTIONS=detect_leaks=0`
und `UBSAN_OPTIONS=halt_on_error=1` ausgeführt; LeakSanitizer gilt nicht als geprüft.
