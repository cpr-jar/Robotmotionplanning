# Dokumentation für Entwicklung und Steuerungseinbindung

Stand: **0.12.3**. Die Zielsteuerung ist eine Eigenentwicklung; die konkrete
Adapterplanung folgt später. Der betrachtete Zyklusbereich beträgt **1–20 ms**.
Die Dokumente beschreiben die vorhandene Bibliothek und benennen noch offene
Integrationsaufgaben ausdrücklich.

| Lesereihenfolge | Dokument | Inhalt |
| --- | --- | --- |
| 1 | [Integrationshandbuch](integration/controller_integration.md) | Verantwortlichkeiten, Build, Vorbereitung, zyklische Übergabe, Lebensdauer, Timing und Fehler |
| 2 | [Daten- und Schnittstellenvertrag](integration/interface_contracts.md) | Einheiten, Achsreihenfolge, Zustände, Rückgabewerte, Uhrdomänen und Threadzuordnung |
| 3 | [Architektur und Klassendiagramme](architecture/class_diagrams.md) | Sieben UML-Sichten als SVG; Vererbung, Besitz und geliehene Referenzen |
| 4 | [Laufzeitabläufe](architecture/runtime_flows.md) | Sequenzen, Zustandstabelle und Live-Handshake |
| 5 | [Ordner- und Modulkatalog](architecture/module_catalog.md) | Zuständigkeit jedes Moduls und relevante Dateien |
| 6 | [Benennungsregeln](development/naming_and_migration.md) | Dateinamen, Variablennamen, mathematische Notation und Altpfad-Migration |
| 7 | [Test- und Abnahmeplan](integration/acceptance_plan.md) | Vorhandene Nachweise, spätere Steuerungstests und offene Entscheidungen |
| 8 | [Modellformat](formats/robot_model_yaml.md) / [Programmformat](formats/robot_program_yaml.md) | URDF/SI-Profil, Modellreferenzen, PTP/LIN/CIRC, WAIT/CALL und Quellorte |
| 9 | [Prüfbericht](testing/validation.md) / [Robustheitstests](testing/robustness.md) | Tatsächlich ausgeführte Prüfungen und reproduzierbare Seeds |
| 10 | [Roadmap](planning/roadmap.md) | Noch offene Funktionspakete und Hardwaregrenzen |

Die Diagramme sind eingecheckt. Zum Lesen wird weder Python noch Graphviz
benötigt; die SVG-Dateien lassen sich im Browser öffnen. Für eine zusammenhängende
Offline-Ausgabe siehe [Handbuch erzeugen](development/naming_and_migration.md#dokumentation-prüfen-und-erzeugen).
Historische Wiederherstellung und Sicherung stehen unter [history](history/recovery_status.md).
