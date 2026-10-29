# Benennung und Migration ab 0.12.3

## Verbindliche Regeln für gepflegten Code

| Element | Regel | Beispiele |
| --- | --- | --- |
| Dateien / Ordner | Englisch, `lower_snake_case`, nach Aufgabe | `execution/live_motion_executor.hpp` |
| Klassen / Structs / Enumtypen | `UpperCamelCase`, fachliche Rolle | `SerialChainKinematics`, `ExecutionTimingPolicy` |
| Funktionen / Methoden | `lower_snake_case`, Tätigkeit oder lesbarer Getter | `prepare_live_plan()`, `joint_limits()` |
| Variablen / Parameter | `lower_snake_case`, Rolle vor generischem Typ | `prepared_program`, `plan_mailbox`, `next_command` |
| Private Daten | Aussagekräftiger Name plus `_` | `execution_epoch_s_`, `next_tick_` |
| Konstanten | `constexpr` mit `lower_snake_case` | `axis_count`, `controller_clock_id` |
| Enumwerte | `UpperCamelCase` | `ExecutionStatus::Finished` |
| Build-Optionen / Makros | Großbuchstaben mit Präfix `MOTION_` | `MOTION_BUILD_TESTS` |
| Maße / Zähler | Einheit bzw. Zählart als Suffix | `_s`, `_ms`, `_m`, `_rad`, `_count`, `_index`, `_ticks` |
| Transformationen | `target_from_source` | `world_from_camera`, `parent_from_child` |
| Container | Fachlicher Plural / Vektorrolle | `cached_commands_`, `joint_names()` |

Mathematische Standardnotation wird beibehalten: `q`, `dq`, `ddq`, `J`, `s`,
`u`, `N` und kurze Indizes sind innerhalb von Formeln eindeutig. Ein Gelenkvektor
bekommt nicht pauschal das Suffix `_rad`, weil eine Kette rotatorische und
prismatische Gelenke enthalten kann. Die Einheiten sind pro Achse definiert.
Außerhalb numerischer Formeln werden sprechende Namen verwendet. Es gibt keine
Typpräfixe wie `pObj`/`vecData` und keine neuen Sammeldateien `utils.hpp`.

## Tatsächlich bereinigte Namen

Private Dispatcher-Felder wie `next_`, `fence_`, `last_`, `end_`, `cycle_` und
`epoch_` heißen jetzt beispielsweise `next_tick_`, `committed_through_tick_`,
`last_observed_time_s_`, `terminal_tick_`, `cycle_period_s_` und
`execution_epoch_s_`. Veröffentlichte Snapshotwerte tragen `published_` im Namen.
Die Quintik verwendet `duration_s_`, `initial_position_`, `position_delta_` und
`start_timestamp_s_` anstelle der früheren Kurzformen. Mathematische Formeln
und öffentliche Zustandsfelder bleiben fachlich unverändert.

Allgemeine Uhrdaten heißen jetzt **`ClockTime`** und liegen unter
`core/clock_time.hpp`. Der bisherige Name `TrackingTime` ist ein Alias desselben
Typs. Die vorhandenen Feldnamen `seconds`, `clock`, `uncertainty_s` bleiben
quellkompatibel. `clock` ist eine Uhrdomänen-ID, kein Uhr-Objekt.

## Bestehenden C++-Code übernehmen

```cpp
// Bisher weiterhin zulässig:
#include <motion/live_execution.hpp>
#include <motion/robot_program.hpp>

// Für neuen / überarbeiteten Code:
#include <motion/core/clock_time.hpp>
#include <motion/execution/live_motion_executor.hpp>
#include <motion/program/robot_program.hpp>
```

Alle 34 bisherigen flachen Header sind Weiterleitungen auf den jeweils
zuständigen Header. Es gibt keine doppelte Implementierung. `namespace motion`,
die Datenfelder q/dq/ddq und die bestehenden Targetnamen bleiben nutzbar;
`RobotMotion::motion` ist das neue bevorzugte CMake-Target. Die Übernahme ist
auf **Quellkompatibilität** ausgerichtet. Alle abhängigen C++-Binaries neu bauen;
eine stabile ABI wird für diesen 0.x-Stand nicht zugesagt.

Die vollständige maschinenlesbare Alt-/Neuzuordnung steht in
[file_migration.json](../architecture/file_migration.json). Beispiele:

| Bisher | Aktuell |
| --- | --- |
| `include/motion/core.hpp` | `include/motion/core/motion_types.hpp` |
| `include/motion/serial.hpp` | `include/motion/kinematics/serial_chain_kinematics.hpp` |
| `include/motion/execution.hpp` | `include/motion/execution/deadline_motion_executor.hpp` |
| `include/motion/model_import.hpp` | `include/motion/io/robot_model_import.hpp` |
| `src/serial.cpp` | `src/kinematics/serial_chain_kinematics.cpp` |
| `tests/test_execution.cpp` | `tests/execution/test_live_execution.cpp` |
| `examples/program_simulator.cpp` | `examples/programs/program_simulator.cpp` |
| `programs/ur5e_demo.yaml` | `programs/examples/ur5e_demo.yaml` |
| `tools/generate_reference_models.py` | `tools/models/generate_reference_models.py` |
| `docs/VALIDATION.md` | `docs/testing/validation.md` |

Für verschobene Beispielprogramme sind die relativen Profilpfade angepasst.
Eigene Programmdateien können an beliebiger Stelle liegen; ihre Modellpfade
werden weiterhin relativ zu ihrer jeweiligen Programmdatei aufgelöst.
CTest- und ausführbare Targetnamen wie `program_simulator` und `serial_tests`
bleiben erhalten. Die Programme liegen weiterhin direkt im Build-Verzeichnis.

## Dokumentation prüfen und erzeugen

```sh
# Standardbibliothek von Python genügt; prüft auch lokale Dokumentationslinks:
python3 tools/architecture/check_structure.py
python3 tools/architecture/check_public_headers.py --with-serial

# Nur zur Regeneration der eingecheckten SVGs: zusätzlich Graphviz erforderlich.
python3 tools/architecture/render_diagrams.py
# Optional druckbare einzelne Diagramme:
python3 tools/architecture/render_diagrams.py --format pdf

# Zusammenhängendes Offline-Handbuch mit eingebetteten SVGs (optional):
python3 -m pip install -r tools/requirements-docs.txt
python3 tools/documentation/build_handbook.py
python3 tools/documentation/build_handbook.py --check
```

Die fertige Datei [docs/handbook.html](../handbook.html) ist eingecheckt und
lädt keine externen Ressourcen. Sie lässt sich lokal im Browser lesen oder mit
der Browser-Druckfunktion als PDF speichern. Die Mermaid-Abläufe bleiben darin
als Quelltexte erhalten; die UML-Klassendiagramme sind direkt eingebettete SVGs.
Zum Einzelheader-Check optionaler Ruckig-Header zusätzlich
`--ruckig-include build-all/_deps/ruckig-src/include` angeben.

Der Diagrammrenderer prüft Namen von Klassen und angezeigten Methoden gegen die
jeweiligen Header. Die Beziehungen und gezeigten Methodenausschnitte sind
redaktionell gepflegt; der Check ist kein vollständiger C++-/UML-Parser.
Die Spezifikation liegt in `docs/architecture/diagrams/class_diagrams.json`;
DOT und SVG werden daraus erzeugt. Strukturprüfung und CMake-Anwenderprojekt
laufen zusätzlich in CI. Fremdquellen und Lizenztexte werden nicht umbenannt
oder stilistisch angepasst. Historische Prüfergebnisse bleiben versioniert.
