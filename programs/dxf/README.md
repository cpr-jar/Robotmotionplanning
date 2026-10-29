# Referenzzeichnung

`rounded_path.dxf` enthält eine geschlossene Mittellinie auf Layer `PATH`:
`(0,0) → (25,0) → (30,5) → (30,20) → (0,20) → (0,0)`, Einheit mm.
Die Kante `(25,0) → (30,5)` hat Bulge `tan(π/8)` und beschreibt einen
Viertelkreis um `(25,5)` mit Radius 5 mm. Die übrigen vier Kanten sind Geraden.

Die Datei wurde mit **ezdxf 1.4.2** als R2000-Text-DXF erzeugt und anschließend
erneut mit diesem unabhängigen Reader gelesen und auditiert: keine Fehler oder
Reparaturen, fünf virtuelle LINE/ARC-Entities mit den genannten Geometriedaten.
Die nativen Tests vergleichen den Import gegen diese Referenzwerte.
ezdxf ist ausschließlich ein optionales Entwicklungswerkzeug und keine
Laufzeit- oder Build-Abhängigkeit der C++-Bibliothek/des Editors.

Verwendung: [ur5e_dxf_demo.yaml](../examples/ur5e_dxf_demo.yaml),
[DXF-Befehlsbeschreibung](../../docs/formats/dxf_program_command.md).
