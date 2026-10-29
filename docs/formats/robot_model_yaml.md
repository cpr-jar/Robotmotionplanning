# Kinematikbeschreibung und SI-Motion-Profil

Die aktuelle Austauschbeschreibung besteht aus **URDF-Geometrie plus SI-YAML-
Profil v1**. Der öffentliche Import liegt in
`motion/io/robot_model_import.hpp`. Er unterstützt serielle Ketten von 1–12
beweglichen Achsen einschließlich fester Zwischensegmente. ROS, Xacro, Meshes
oder Antriebsprotokolle werden dafür nicht benötigt.

## Datei- und API-Zuordnung

```cpp
const auto robot = motion::load_serial_robot_files(urdf_filename, profile_filename);
// Alternativ getrennt aus In-Memory-Texten:
const auto profile = motion::parse_serial_profile_yaml(profile_text);
const auto robot_from_text = motion::import_serial_urdf(urdf_text, profile);
```

Bei `load_robot_program()` bestimmt `robot_profile` den Profilpfad relativ zur
Programmdatei. Ohne `robot_urdf` liegt die erwartete Datei `robot.urdf` neben dem
Profil. Das optionale Programmfeld `robot_urdf` ist ebenfalls relativ zur
Programmdatei. **Das Modellprofil selbst enthält keinen Dateipfad zur URDF.**
`load_serial_robot_files()` erhält beide Dateinamen ausdrücklich als Argumente.

## Kleines vollständiges Formatbeispiel

Ein rotatorisches Einachsmodell dient nur zur Erklärung des Formats; es ist kein
Ersatz für ein Sechsachsmodell. Als `robot.urdf`:

```xml
<robot name="one_axis_example">
  <link name="base_link"/>
  <link name="flange"/>
  <joint name="axis_1" type="revolute">
    <parent link="base_link"/>
    <child link="flange"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="0.5"/>
  </joint>
</robot>
```

Daneben als `profile.yaml`, mit ausschließlich beispielhaften Motion-Grenzen:

```yaml
schema_version: 1
units: SI
robot_name: one_axis_example
base_link: base_link
tip_link: flange
base_pose:
  xyz: [0, 0, 0]
  rpy: [0, 0, 0]
tool_offset:
  xyz: [0.2, 0, 0]
  rpy: [0, 0, 0]
joints:
  axis_1:
    lower: -0.9
    upper: 0.9
    max_velocity: 0.25
    max_acceleration: 1.0
    max_jerk: 5.0
tcp_limits:
  max_linear_velocity: 0.25
  max_linear_acceleration: 0.5
  max_linear_jerk: 2.0
  max_angular_velocity: 0.5
  max_angular_acceleration: 1.0
  max_angular_jerk: 5.0
```

## Profilfelder

| Feld | Pflicht / Bedeutung |
| --- | --- |
| `schema_version`, `units` | Pflicht: 1 und SI |
| `robot_name`, `base_link`, `tip_link` | Pflicht: Name und ausgewählte serielle Kette |
| `base_pose` | Optional, Identität als Standard; `xyz` in m, `rpy` in rad |
| `tool_offset` | Optional, Identität als Standard; Transform von Tip/Flansch zum TCP |
| `joints` | Genau die beweglichen Gelenke der ausgewählten Kette, nach URDF-Namen |
| `max_acceleration`, `max_jerk` je Gelenk | Pflicht, endlich und positiv |
| `lower`, `upper`, `max_velocity` je Gelenk | Optional; dürfen URDF-Grenzen nur enger machen |
| `tcp_limits` | Alle sechs linearen/angularen v/a/Ruck-Grenzen, endlich und positiv |
| `solver` | Optional: lokale IK-Parameter, siehe unten |

Gelenkreihenfolge ergibt sich aus der Kette von Basis zu Tip, nicht aus der
Reihenfolge einer YAML-Map. Namen anschließend mit `joint_names()` überprüfen.
Für `continuous` sind im Profil endliche `lower`/`upper` als Softwarefenster
notwendig, da zulässige Motion-Grenzen endlich sein müssen. Winkel werden nicht
nachträglich auf ±π gefaltet. Gleichnamige/überzählige/fehlende Profilgelenke
werden abgelehnt; feste URDF-Gelenke bekommen keinen Dynamikeintrag.

Posekonvention: `base_pose` beschreibt Welt-aus-Basis, `tool_offset` den
angehängten Flansch-aus-TCP-Transform. Es gilt
`world_from_tcp = world_from_base * chain(q) * flange_from_tcp`.
RPY wird als `Rz(yaw) * Ry(pitch) * Rx(roll)` interpretiert. Im Programmformat
werden Orientierungen dagegen als Quaternionen `[w,x,y,z]` angegeben.

## Unterstützte URDF-Teilmenge

- `fixed`, `revolute`, `continuous`, `prismatic`, Origins und lokale Gelenkachsen.
- Bewegliche Gelenke benötigen ein positives Geschwindigkeitslimit;
  `revolute`/`prismatic` zusätzlich endliche untere/obere Grenzen.
- Die ausgewählte Verbindung zwischen Basis und Tip muss eindeutig, zusammenhängend
  und zyklusfrei sein. Mehrfach-Eltern und unterstützungsfremde Gelenktypen
  werden abgelehnt.
- `mimic`, `calibration` und `safety_controller` werden ausdrücklich nicht als
  wirksame Kopplungs-/Kalibrierungs-/Sicherheitsmodelle interpretiert, sondern
  bei betreffenden Gelenken abgelehnt.
- Visual-/Collision-/Inertialdaten sind keine vollständige Kollisions- oder
  Dynamikprüfung in diesem Import. Es gibt keine automatische Netzwerk-/Mesh-
  Auflösung und keine Makroausführung.

Die Dateilader begrenzen URDF auf 8 MiB und Profile auf 1 MiB. Profil-Parser
prüfen Schema, doppelte/unbekannte Schlüssel, endliche Werte und YAML-Budgets.
Importfehler werden als `ModelImportError` ausgegeben; beim Programmladen werden
sie mit dem Roboter-Quellort als `RobotProgramError` weitergegeben.

## Optionale lokale IK-Konfiguration

| `solver`-Feld | Standard |
| --- | --- |
| `max_iterations` | 300, zulässige positive Ganzzahl bis 10.000 |
| `position_tolerance_m` | 1e-7 |
| `orientation_tolerance_rad` | 1e-7 |
| `orientation_length_m` | 0.25 |
| `prismatic_scale_m` | 0.1 |
| `damping` | 1e-3 |
| `max_normalized_step` | 0.25 |
| `singularity_threshold` | 1e-6 |

Die Skalierungen mischen Positions-/Orientierungsfehler kontrolliert in der
lokalen numerischen Suche. Parameter nicht ohne Vergleichs-/Grenzfallprüfung
ändern. Ein abgelehnter lokaler IK-Lauf beweist nicht, dass keine andere
Gelenkverzweigung zum Ziel existiert.

Reale Referenzprofile und Herkunftsdaten liegen unter `models/`. Neue Grenzen,
Nullpunkte, Werkzeugdaten oder Kalibrierungen für euren Aufbau müssen explizit
versioniert und unabhängig von den öffentlichen nominalen Beispielen geprüft
werden. Für die spätere Host-Zuordnung gilt der
[Schnittstellenvertrag](../integration/interface_contracts.md).
