# UR5e-Visual-Modell des separaten Viewers

Die sieben vollständigen Visual-Baugruppen stammen aus
[UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)
am Commit `81f7c9fe003c2f0e02eec5a7253f1b262ad38b61`. Das ist derselbe
Upstream-Stand wie beim vorhandenen nominalen UR5e-Kinematikmodell.
Die Originaldateien werden unverändert unter `source/` mitgeliefert;
[sources.json](sources.json) enthält URLs, Dateigrößen und SHA-256-Prüfsummen.

| Roboterlink | Originalmesh | Dreiecke |
| --- | --- | ---: |
| `base_link_inertia` | `base.dae` | 4.838 |
| `shoulder_link` | `shoulder.dae` | 23.290 |
| `upper_arm_link` | `upperarm.dae` | 40.124 |
| `forearm_link` | `forearm.dae` | 15.058 |
| `wrist_1_link` | `wrist1.dae` | 17.288 |
| `wrist_2_link` | `wrist2.dae` | 20.280 |
| `wrist_3_link` | `wrist3.dae` | 911 |

Insgesamt: **121.789 Dreiecke in 31 Material-/Primitivegruppen**, einschließlich
der zusätzlichen Linien aus den Originaldateien. Keine Vereinfachung der
Dreiecksflächen. Farben und Normalen stammen aus den COLLADA-Dateien. Die
Visuals enthalten keine Texturen, sodass keine Bilddateien nachgeladen werden.
Ein angebautes Werkzeug, die Steuerungsbox und die Umgebung gehören nicht zu
diesem Arm-Modell.

## Dateien und Koordinaten

- `source/`: unveränderte sieben DAE-Dateien, offizielle Visual-Parameter,
  Xacro-Dateien mit Autorennennungen sowie Upstream-README und BSD-Lizenz.
- `visuals.json`: für den Offline-Renderer abgeleitete Geometrie, gruppiert nach
  Link und Material. Positionen/Normalen sind Little-Endian-Float32,
  Indizes Little-Endian-Uint32; die Puffer sind Base64-codiert.
- `manifest.json`: passende Referenzkette und Prüfsumme des abgeleiteten Pakets.
- `ur5e_visual.urdf`: werkzeugeigene Kopie der vorhandenen Kinematik mit sieben
  `<visual>`-Einträgen und lokal auflösbaren Original-DAE-Pfaden. Die Kinematik
  wird dabei unverändert übernommen. Diese Datei wird nicht anstelle des
  Bibliotheksmodells verwendet und enthält keine Kollisionshüllen oder Dynamik.

Die COLLADA-Szenentransformationen und ihre Einheiten werden bei der Konvertierung
auf die Vertices angewendet. Normalen verwenden die inverse transponierte
Transformation. Der Renderer behält **Meter und Z nach oben**; es erfolgt keine
zusätzliche Y-up-Konvertierung. Die Montageversätze aus
`source/config/ur5e/visual_parameters.yaml` werden genau einmal angewendet:

```text
Welt ← Basispose ← Gelenkkette(q) ← Link ← Visual-Origin ← Meshpunkt
TCP  = Basispose × Gelenkkette(q) × Werkzeugoffset
```

Die beiden Ausgaben teilen dieselbe FK. Der Werkzeugoffset verschiebt den TCP,
nicht die am Flansch befestigte Handgelenkgeometrie. Der feste Basisversatz um
180° gehört zur Kinematikkette und wird zusätzlich zu den ausdrücklich in der
offiziellen Visual-Konfiguration angegebenen Mesh-Versätzen berücksichtigt.

## Regeneration für Entwickler

Die normale Viewer-Erstellung nutzt das bereits mitgelieferte Paket und benötigt
keine zusätzlichen Python-Pakete. **Nur die Regeneration** benötigt PyYAML:

```sh
python3 tools/motion_viewer/build_visual_assets.py
```

Der Konverter prüft zuerst alle Originalprüfsummen. Nicht unterstützte
COLLADA-Features, andere Hochachsen, Animationen oder Texturen führen zu einer
Diagnose. Die hier verwendeten Dateien müssen weder mit ROS noch mit Xacro
ausgeführt werden. Nach einer Regeneration Demo-HTML und Werkzeugtests erneut
erzeugen beziehungsweise ausführen, wie in der [Werkzeuganleitung](../../README.md)
beschrieben. Eine geänderte Roboterkalibrierung benötigt eine passende neue
Visual-Referenz; die Automatik hängt dieses nominale Modell nicht ungeprüft an.

## Lizenzen

Die ursprünglichen Lizenzbedingungen stehen unverändert in
[source/LICENSE](source/LICENSE), die Autorennennungen insbesondere in
[source/urdf/ur_macro.xacro](source/urdf/ur_macro.xacro). Die Quellen werden
zusammen mit der abgeleiteten Darstellung verteilt. Das separat mitgelieferte
Three.js r160 steht unter der [MIT-Lizenz](../../vendor/THREE-LICENSE.txt).
`vendor/manifest.json` protokolliert dessen Herkunft und Prüfsummen.

Die erzeugte HTML-Datei enthält die vollständigen BSD-/MIT-Bedingungen und
Autorennennungen ebenfalls unter **Modellherkunft und Lizenzen**, damit die
Einzeldatei auch ohne das Quellverzeichnis weitergegeben werden kann.
