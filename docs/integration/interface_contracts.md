# Daten- und Schnittstellenverträge

Diese Referenz beschreibt öffentliche Daten und Verhaltensregeln für die spätere
Steuerungseinbindung. Die tatsächliche Achs- und Threadzuordnung wird in eurem
Adapter festgelegt. Die Bibliothek hat keine implizite Feldbus- oder Encoder-
Zuordnung. Alle Beispiele beziehen sich auf die aufgabenbezogenen Header ab 0.12.3.

## Maße, Achsen und Koordinaten

| Größe | Bedeutung / Einheit |
| --- | --- |
| `q[i]` | Rotatorisch: rad; prismatisch: m; Index gemäß `joint_names()` |
| `dq[i]` | rad/s bzw. m/s |
| `ddq[i]` | rad/s² bzw. m/s² |
| Gelenkruck | rad/s³ bzw. m/s³; im Sample separat, nicht im festen Achsbefehl |
| TCP-Position | m im vereinbarten Weltkoordinatensystem |
| TCP-Orientierung | Einheitsquaternion **[w,x,y,z]**, gleiche Orientierung bei Q und −Q |
| `Twist.data[0..2]` | Linearer Weltvektor x/y/z |
| `Twist.data[3..5]` | Angularer Weltvektor x/y/z; kein Eulerwinkel-Zeitgradient |
| Lineare TCP-Grenzen | m/s, m/s², m/s³; Norm der jeweiligen drei Komponenten |
| Angulare TCP-Grenzen | rad/s, rad/s², rad/s³; Norm der jeweiligen drei Komponenten |
| `timestamp_s` | Absolute Zeit in der vereinbarten Uhrdomäne, s |
| `program_time_s` | Zeit ab Ausführungsepoche; im Live-Pfad zu globalem Tick gehörig |
| Geometrieparameter `s` | Dimensionslos, 0 bis 1; nicht Sekunden |
| `speed` | Dimensionsloser Faktor; effektive Programmgeschwindigkeit 0,001 bis 1 |

Eine serielle Kette kann gemischte Gelenktypen enthalten. Auch bei sechs Achsen
muss die Zuordnung explizit geprüft werden. Für Parameter mit gemischten Einheiten
(z. B. Gelenkfehlertoleranzen) später je Achse ein Feld vorsehen. Matrizen und
Quaternionen nicht in herstellerspezifische Eulerwinkel umdeuten, ohne die
Konvention separat zu vereinbaren.

## Grundtypen

| Typ / Felder | Vertrag |
| --- | --- |
| `JointState { q, dq, ddq, timestamp_s }` | Drei gleich lange dynamische Vektoren; Softwarezustand, keine automatische Istwertmessung |
| `JointTarget { q, dq, ddq }` | Zielrandzustand; einzelne Planer unterstützen nur Stillstand |
| `JointLimits { min_position, max_position, max_velocity, max_acceleration, max_jerk }` | Alle Vektoren in Achsreihenfolge; endliche Grenzen, min<max und positive Ableitungsgrenzen |
| `Pose { position, orientation }` | Drei Positionswerte, Quaternion wxyz |
| `CartesianLimits` | Je drei positive lineare und angulare Grenzen bis Ruck |
| `KinematicJointLimits { lower, upper }` | Geometrische Kinematikgrenzen; die Motion-Grenzen dürfen zusätzlich enger sein |
| `ImportedSerialRobot` | Roboternamen, Basis/Tip, geteilte serielle Kinematik sowie Gelenk-/TCP-Grenzen |

`check_state()` prüft im Kern nur Dimensionen. Endlichkeit und Grenzwerte werden
in den jeweiligen Zulassungs-/Ausführungspfaden geprüft; ein bloßer erfolgreicher
Dimensionscheck ist keine vollständige Zustandszulassung. `within_joint_limits()`
prüft q/dq/ddq gegen Grenzen und eine numerische Toleranz, aber keinen Ist-Soll-
Folgefehler und keine physische Bremswirkung.

## Uhrvertrag

```cpp
struct ClockTime {
  double seconds;
  ClockId clock;          // vereinbarte Domänen-ID, uint64_t
  double uncertainty_s;  // absolute Fehlergrenze, nicht Standardabweichung
};
```

- Alle Beobachtungen einer Ausführung gehören zur gleichen nichtnull Uhrdomäne.
- Sekunden und Unsicherheit sind endlich; Unsicherheit ist nichtnegativ und
  liegt unter der vereinbarten Maximalgrenze.
- Beobachtete Zeit darf nicht zurückspringen. Sekunden sind weder CPU-Ticks noch
  automatisch Unix-/ROS-/Kamera-Zeit.
- Ein eigener Clock-Adapter implementiert `ClockTime now() noexcept`.
- `TrackingTime` bleibt ein Quellkompatibilitätsalias für `ClockTime`.

`ExecutionTimingPolicy` enthält **zulässige Grenzen**, keine gemessenen
Statistikwerte. `max_early_s`/`max_late_s` begrenzen das Ausgabefenster,
`max_clock_uncertainty_s` die jeweilige Uhrbeobachtung und
`dispatch_latency_bound_s` den vertraglich begrenzten nachfolgenden Übergabepfad.
Eine empirische Durchschnittslatenz ist keine obere Latenzgrenze.

## Fester Achsbefehl

```cpp
template<std::size_t N>
struct FixedJointCommand {
  std::array<double, N> q, dq, ddq;
  double timestamp_s, program_time_s;
  std::uint64_t tick;
  bool terminal;
};
```

| Feld | Auslegung im Host |
| --- | --- |
| q/dq/ddq | Exakt N SI-Werte kopieren bzw. mit expliziter Achsabbildung umrechnen |
| `timestamp_s` | Geplanter Ausgabe-/Referenzzeitpunkt; nicht nachträglich auf „jetzt“ setzen |
| `tick` | Beginn bei 0, dann genau eins weiter; beim Live-Wechsel global fortlaufend |
| `terminal` | Dieser Befehl ist der letzte und liegt im Stillstand; einmal ausgeben |
| `LiveJointCommand::generation` | Referenzidentität zusätzlich zum globalen Tick |

Die Struktur ist ein C++-Werttyp, **kein paketiertes Drahtformat**. Padding,
Endianness und `bool`-Darstellung dürfen nicht per `memcpy` als Fremdprotokoll
festgeschrieben werden. Der Header enthält keinen Ruckkanal; wenn die Steuerung
Ruck als separaten Sollwert benötigt, ist das später explizit zu spezifizieren.

## Rückgabe von Ausführern

| `ExecutionStatus` | Neuer Ausgabewert? | Erforderliche Reaktion |
| --- | --- | --- |
| `Running` | Ja | Diesen neuen Befehl genau einmal übergeben |
| `Finished` | Ja, terminaler Stillstandsbefehl | Einmal übergeben, anschließend Programmende melden |
| `Inactive` | Nein, Ausgabe unverändert | Nichts erneut senden; kann „noch zu früh“ oder „bereits fertig“ bedeuten |
| `Error` | Nein, Ausgabe unverändert | Fehler verriegeln; Stop-/Fehlerreaktion des Hosts auslösen |

Es gibt kein automatisches Wiederholen, Aufholen, Überspringen, Reset oder
Fortsetzen nach `Error`. Nach Korrektur von Zeit/Feedback muss ein neuer Start
mit neuen Vorbedingungen vorbereitet werden. Ein Daten- oder Zeitfehler erzeugt
nicht automatisch eine neue Haltebahn. Ein bereits vorbereiteter Following-
Fallback ist ein gesonderter Mechanismus.

| `ExecutionFault` | Auslöser / Bedeutung |
| --- | --- |
| `None` | Bisher kein Dispatcherfehler |
| `InvalidClock` | Falsche Domäne, NaN/Inf oder negative Unsicherheit |
| `ClockRegression` | Beobachtete Sekunden laufen zurück |
| `ExcessUncertainty` | Beobachtete Uhrunsicherheit überschreitet die Policy |
| `DeadlineMissed` | Ganzes Unsicherheits-/Latenzintervall passt nicht mehr ins Fenster |
| `Underflow` | Gepufferter Pfad hat zur Fälligkeit keinen nächsten Befehl |
| `InvalidCommand` | Tick, Zeitstempel, Ableitungen, Limits oder Stillstandsmarkierung passen nicht |

Die Hardware kann eine Befehlsannahme zusätzlich ablehnen. Das ist ein
**Anwendungs-/Übergabefehler**, kein neuer `ExecutionFault` im Kern. Der Host muss
hier selbst die weitere Übergabe sperren. Das Referenzbeispiel demonstriert das.

## Planung, Import und Sampling

| API | Erfolg | Fehler / Zusage |
| --- | --- | --- |
| `parse_robot_program_yaml()` | Unveränderliche `RobotProgramSource` | `RobotProgramError`; keine Modell-Dateizugriffe durch reines Parsen |
| `compile_robot_program()` / `load_robot_program()` | Ganzes `PreparedRobotProgram` | `RobotProgramError` mit Quellort; keine teilweise erfolgreiche Veröffentlichung |
| `prepare_robot_program()` / `retime_robot_program()` | Referenz und daraus erzeugtes Zyklusraster | `std::invalid_argument`; keine Änderung des ursprünglichen Paars |
| `PreparedRobotProgram::validate()` / `simulate_robot_program()` | Herkunft von Referenz und Schedule stimmt überein | `std::invalid_argument` bei leeren/vertauschten Paaren; Simulation schreibt dann keine CSV |
| `IJointTrajectoryPlanner::plan()` | `PlanResult` mit gültigem bool und Trajektorie | `PlanStatus`/Diagnose; nur bei Erfolg Trajektorie verwenden |
| `SerialChainKinematics::ik_detailed()` | `SerialIKStatus::Success`, passende Lösung | Begrenzte lokale Suche, detaillierter Status; keine globale Unerreichbarkeitsgarantie |
| `prepare_live_plan<N>()` | `LivePlanResult` mit Cache | Leerer Plan mit Diagnose bei ungültigem Raster, Randzustand, Uhr oder Kapazität |
| `sample()` / `sample_tick()` | Vektorhaltiger Zustand | Kann allokieren und werfen; nicht als zyklischen Festwert-Dispatcher benutzen |

Bei eigenen `IExecutionSchedule`-Implementierungen müssen Limits, Anfangszustand
und sämtliche Samples dieselbe Achszahl besitzen. `prepare_live_plan<N>()` prüft
sie gegen **N**, bevor Daten in feste Arrays kopiert werden. Dauer und
`tick_time(tick)` müssen dem kanonischen Raster
`schedule_detail::tick_time(tick, cycle_s())` entsprechen; Sample-Zeitstempel sind
`initial_state().timestamp_s + tick_time(tick)`. Tick 0 muss dem Anfangszustand
entsprechen. Der Provider bleibt während der Vorbereitung unveränderlich.

`IBoundedJointPath` liefert pro Punkt gleich große, endliche Vektoren für
`q`, `q_s`, `q_ss`, `q_sss`, eine gültige TCP-Pose und endliche TCP-Ableitungen.
Bounds haben dieselbe Achszahl, geordnete endliche Positionen und nichtnegative
Ableitungshüllen. Die Zeitplanung prüft Anfang/Ende und die Form bei jedem
Sampling; sie kann die Wahrheit einer fremden Hüllenbehauptung nicht beweisen.

`IOnlineJointGenerator::step()` verwendet den getrennten `StepStatus`:
`Running`, `Finished`, `InvalidInput`, `Error`. Diese API plant/generiert und hat
einen anderen Vertrag als `ExecutionStatus`. Der Ruckig-Adapter ist kein
pauschaler RT-Nachweis; Planung und Prüfung bleiben außerhalb des
Dispatcherpfads. Den Generatorstatus nicht blind in einen Achsstatus umdeuten.

## Live-Wechsel und Besitz

`LivePlanStamp` enthält `generation`, `parent_generation`, `start_tick`, `clock`.
Die Anfangsgeneration hat Vorgänger 0 und Starttick 0. Weitere Generationen sind
höher als die aktuelle und benennen deren Identität als Vorgänger.

| `LiveResponseStatus` | Bedeutung / Besitzfolge |
| --- | --- |
| `Activated` | Neuer Cache aktiv; `retired_generation` wird vom Executor nicht mehr benutzt |
| `IdentityMismatch` | Vorgänger, Generation, Uhr, Epoche, Takt oder Grenzen passen nicht; Kandidat kann freigegeben werden |
| `StateMismatch` | q/dq/ddq/Zeitstempel am Wechselpunkt nicht exakt identisch; Kandidat abgelehnt |
| `Late` | Zu spät, zu geringer Mindestvorlauf oder außerhalb des aktiven Bereichs |
| `Committed` | Wechsel würde bereits übergebene Befehle ersetzen |
| `Faulted` | Ausführer ist fehlerhaft/beendet oder ausstehender Wechsel wurde beim Fehler abgebrochen |

Der Planer besitzt angebotene Caches mindestens bis zur zugehörigen Antwort.
Bei `Activated` behält er den **neuen** Cache; nur der nicht mehr referenzierte
alte darf außerhalb RT freigegeben werden. Bleibt er als Host-Snapshot oder
Diagnosereferenz anderweitig in Benutzung, ist auch dessen Lebensdauer zu
berücksichtigen. Rohzeiger im Postfach sind keine Besitzübertragung.
`PreparedLivePlan::command(tick)` ist ein ungeprüfter schneller Zugriff; gültig
ist nur `start_tick <= tick <= end_tick`. Direkten Anwendungszugriff begrenzen.

`snapshot()` liefert optional einen kohärenten atomaren Snapshot. Ein leeres
Ergebnis ist eine fehlgeschlagene begrenzte Leseprobe, kein Bewegungsfehler.
Andere direkte Getter sind nicht für beliebige parallele Leser freigegeben.
Ein angebotener Kandidat darf bis zur Antwort nicht geändert oder ersetzt werden.

## Beobachtungen, Frames und Kalibrierung

`ObjectObservation` trägt Objekt-ID, Frame, Pose, Messzeit, Empfangszeit und
Positions-/Orientierungsfehler. `EncoderObservation` trägt Position, v/a,
Mess-/Empfangszeit und deren Fehlergrenzen. Zeiten müssen nominal kausal sein:
Messung ≤ Empfang ≤ jetzt ≤ Prognosezeit. Alter, Latenz, Prognosehorizont und
Unsicherheit werden zusätzlich begrenzt.

`TrackingFrameSnapshot` ist ein unveränderlicher statischer Framegraph mit
Revision und Gültigkeitsintervall. `ConveyorObservationHistory` gehört zu
Objekt-ID und Revision. Neue Revisionen verlangen ein explizites Reset und
neue Beobachtungen; alte Daten werden nicht still neu interpretiert. Ein bereits
zugelassener Cache wird dadurch nicht automatisch ungültig oder sofort gestoppt.
Wie eure Steuerung bei einem Kalibrierungswechsel mit aktiven Plänen umgeht,
ist deshalb eine spätere explizite Host-Policy.

`fit_rigid_calibration()` bestimmt `target_from_source` aus
`PointCorrespondence {source, target, weight}`. Beide Punktmengen sind metrische
3D-Positionen. Es wird ein starrer Transform, keine Skalierung, keine bewegte
Framefusion und keine Gelenknullpunktkalibrierung geschätzt. Residuen und
ausgelagerte Validierungspunkte vor Übernahme prüfen.
