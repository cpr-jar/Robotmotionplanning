# Dokumentation für Entwicklung und Steuerungseinbindung

Stand: **0.15.0**. Die Zielsteuerung ist eine Eigenentwicklung; die konkrete
Adapterplanung folgt später. Der betrachtete Zyklusbereich beträgt **1–20 ms**.
Die Dokumente beschreiben die vorhandene Bibliothek und benennen noch offene
Integrationsaufgaben ausdrücklich.

| Lesereihenfolge | Dokument | Inhalt |
| --- | --- | --- |
| 1 | [Einsteiger-Tutorial mit Ablaufdiagrammen](tutorials/motion_planning_basics.md) | Vom Bewegungsauftrag zur Sollwertausgabe; FK/IK, PTP/LIN/CIRC, Grenzen und ein Zahlenbeispiel |
| 2 | [Integrationshandbuch](integration/controller_integration.md) | Verantwortlichkeiten, Build, Vorbereitung, zyklische Übergabe, Lebensdauer, Timing und Fehler |
| 3 | [Daten- und Schnittstellenvertrag](integration/interface_contracts.md) | Einheiten, Achsreihenfolge, Zustände, Rückgabewerte, Uhrdomänen und Threadzuordnung |
| 4 | [Architektur und Klassendiagramme](architecture/class_diagrams.md) | Acht UML-Sichten als SVG; Vererbung, Besitz und geliehene Referenzen |
| 5 | [Laufzeitabläufe](architecture/runtime_flows.md) | Sequenzen, Zustandstabelle und Live-Handshake |
| 6 | [Ordner- und Modulkatalog](architecture/module_catalog.md) | Zuständigkeit jedes Moduls und relevante Dateien |
| 7 | [Benennungsregeln](development/naming_and_migration.md) | Dateinamen, Variablennamen, mathematische Notation und Altpfad-Migration |
| 8 | [Test- und Abnahmeplan](integration/acceptance_plan.md) | Vorhandene Nachweise, spätere Steuerungstests und offene Entscheidungen |
| 9 | [Modellformat](formats/robot_model_yaml.md) / [Programmformat](formats/robot_program_yaml.md) | URDF/SI-Profil, Modellreferenzen, PTP/LIN/CIRC, WAIT/CALL und Quellorte |
| 10 | [Prüfbericht](testing/validation.md) / [Robustheitstests](testing/robustness.md) | Tatsächlich ausgeführte Prüfungen und reproduzierbare Seeds |
| 11 | [Roadmap](planning/roadmap.md) | Noch offene Funktionspakete und Hardwaregrenzen |
| 12 | [Architektur-Review](architecture/architecture_review_v0.13.0.md) / [Korrekturpaket](testing/architecture_review_fixes.md) | Historische Befunde, erledigte Maßnahmen und verbleibende Schritte |
| 13 | [Verwaltete Planübergabe](integration/live_motion_session.md) / [Prüfnachweis](testing/live_session_validation.md) | Lebensdauer, Annahme/Commit, Rücknahme und bestätigter Shutdown ab 0.14.0 |
| 14 | [Komponenten und Modulgrenzen](architecture/components.md) / [Prüfbericht](testing/components_validation.md) | Selektive CMake-Einbindung, Runtime ohne Parser, Kompatibilität und Schichtenprüfung |

Neu ab 0.13.0: [DXF als Programmbefehl](formats/dxf_program_command.md),
einschließlich Layern, Einheiten, Platzierung, Genauigkeit, Ablauf und Klassensicht.
Der [separate Editor](../tools/motion_viewer/EDITOR.md) kann DXF-Dateien öffnen
und zusammen mit dem Programm simulieren. Es gibt jetzt acht UML-Sichten.

Die Diagramme sind eingecheckt. Zum Lesen wird weder Python noch Graphviz
benötigt; die SVG-Dateien lassen sich im Browser öffnen. Für eine zusammenhängende
Offline-Ausgabe siehe [Handbuch erzeugen](development/naming_and_migration.md#dokumentation-prüfen-und-erzeugen).
Historische Wiederherstellung und Sicherung stehen unter [history](history/recovery_status.md).
