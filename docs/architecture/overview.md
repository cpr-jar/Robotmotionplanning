# Architektur des Stands 0.16.0

Die Bibliothek trennt Kinematik, Planung, Programminterpretation und zeitliche
Ausführung. Der Planungs-/Producerbereich darf dynamische Daten verwenden;
der zyklische Bereich gibt vorbereitete feste Befehle aus. Die Einbindung in
die eigene Steuerung mit 1–20 ms wird durch explizite Host-Verträge beschrieben.

Einstieg: [Modulkatalog](module_catalog.md), [Klassendiagramme](class_diagrams.md),
[Abläufe](runtime_flows.md), [Integrationshandbuch](../integration/controller_integration.md).
Die früheren flachen Header bleiben als Weiterleitungen; `ClockTime` ist der
allgemeine Uhrtyp und `TrackingTime` ein Alias. `RobotMotion::motion` bildet das
bevorzugte CMake-Einstiegsziel. Ursprung und Wiederherstellungsgrenzen stehen im
[historischen Bericht](../history/recovery_status.md). Ab 0.15.0 sind [Komponenten und Schichten](components.md) explizit geprüft;
gemeinsame Verträge liegen unter `contracts/`, Solver/Import/Programme werden
getrennt gebaut. Die verwaltete Cache-/Übergabegrundlage stammt aus 0.14.0.
Aktuelle Vertragsgrenzen:

1. **Daten und Modelle:** C++20, SI-Zustände q/dq/ddq und explizite Grenzen.
   Serielle Kinematik besitzt Basis-/Werkzeugtransformation, feste und bewegliche
   Gelenke; FK, geometrisches J und Matrixjets bis zum Ruck. IK ist lokales,
   begrenztes SVD-Verfahren mit Linien-Suche und Rang-/Restfehlerprüfung.
2. **Import:** URDF-Teilmenge plus SI-YAML-Profil v1 mit obligatorischen v/a/Ruck-
   Vorgaben. Profilgrenzen dürfen URDF-Grenzen nur einschränken. Keine Xacro-
   Ausführung, Mimic-/Kalibrier-/Safety-Tag-Interpretation oder Mesh-/Netzwerkauflösung.
   Serielle Rohmodelle und Motion-Limits bleiben getrennt.
3. **Referenzen:** Rest-to-rest-Quintiken und allgemeine q/v/a-Hermite-Quintiken.
   Bernstein-Hüllen und begrenzte Unterteilung prüfen q/v/a/Ruck. Numerische
   Intervalljets propagieren diese durch serielle, SCARA- und Delta-Kinematik.
   Transzendenten verwenden gepaddete libm-Werte; keine formale Zertifizierung.
4. **Kartesische Bahnen:** LIN/CIRC-Geometrie und Quaternion-Sweep sind von der
   Kinematikabbildung getrennt. Sequenzieller IK und C2-Gelenksplines werden in
   einem Pose-Schlauch über Intervall-Ableitungen und Lipschitz-Abschätzungen
   zugelassen. Konservative globale Quintik-Zeitplanung prüft vollständige
   Gelenk-/TCP-lineare und angulare Kettenregelterme bis zum Ruck.
5. **Programme:** Striktes SI-YAML, begrenzte CALL-Expansion, PTP/LIN/CIRC/WAIT;
   gesamter gemischter Plan wird vorab zugelassen. PTP prüft auch nichtlineare
   TCP-Grenzen. Exakte Stillstandsgrenzen werden auf ganzzahlige Ticks gelegt,
   indem nur verlängert wird. Kein verstecktes Blending.
6. **Ausführung:** Producer sampelt außerhalb des zyklischen Threads. SPSC-Ring
   und feste Befehle trennen ihn vom Dispatcher. Der Dispatcher prüft Tick,
   Epoch, Zustand, Stillstand, Uhrdomäne/-Unsicherheit/-Regression und Zeitfenster.
   Ein zweiter Zeitwert kann Verzögerung während der Prüfung erkennen.
   Fehler verriegeln; kein Wiederholen, Aufholen oder erfundener Not-Haltebefehl.
7. **Live-Austausch:** Immutable Caches mit globalen Ticks, Generation/Vorgänger,
   exaktem q/v/a-Zustand und Epoche. Ein SPSC-Postfach erlaubt eine offene Anfrage.
   Der Planner hält rohe Zeigerziele am Leben bis Ablehnung oder expliziter
   Retirement-Bestätigung. Keine Refcount-Freigabe, IK, Kurvensampling oder
   C++-Heap-Allokation im zyklischen Pfad. Direkte Getter gehören dem Ausführungsthread;
   parallele Planer verwenden bounded-attempt atomare Snapshots. 64-bit-Atomics
   müssen lock-free sein. Übergabegrenzen verhindern Rücknahme bereits
   irreversibel übergebener Befehle.
8. **Stopps und Following:** Optionaler lokaler Ruckig-Kandidat wird unabhängig
   in konstante Ruckphasen integriert und auf innere q/v-Extrema geprüft.
   Originale Zeitgesetze bleiben unverändert; lediglich terminaler Stillstand
   wird auf Ticks gepolstert. Following publiziert bewegte Horizonte inklusive
   vorab geprüftem Stillstands-Fallback. Generisches Following löst q/dq/ddq
   mit räumlichem SVD einschließlich Jdot*dq und prüft TCP-Hüllen separat für
   Horizont und Halt. Ausfall von Updates benötigt keinen Solver im RT-Pfad.
9. **Messdaten:** Kalibrierte statische Frame-Snapshots, explizite Objektzuordnung,
   Zeit-/Latenz-/Horizont-/Unsicherheitsgrenzen und begrenzte Historie. Conveyor-
   Modell: gerader Transport, konstante Orientierung, kein Schlupf. Eine vom
   Anwender vorgegebene Beschleunigungsänderungsgrenze erweitert die Prognosefehler.
   Mess- und Empfangszeiten müssen nominal kausal sein; Unsicherheit fließt
   in Alters-/Horizont-/Fehlergrenzen ein. Kein allgemeiner Sensorfusionssolver.
10. **Plattform:** Linux-Monotonic-Uhr und absolute Wartefunktion melden Fehler
    einschließlich EINTR zurück. FIFO/Affinität/mlock sind explizite Setup-Anfragen
    außerhalb der Ausführung. Der Timing-Probe misst den aktuellen Host, nicht
    den Raspberry Pi. Die Simulation verwendet eine ideale virtuelle Uhr.

Zusätzliche TCP-Boxhindernisse sind explizite Constraints. Vollständige
Payload-/Drehmoment- und Link-/Selbstkollisionsmodule sind noch nicht
wiederhergestellt. CANopen ist bewusst ausgeklammert. Keine dieser Lücken wird
mit Sample- oder Simulationswerten als physisch geprüft dargestellt.

Ab 0.16.0 ergänzt der [Vorbereitungskontext](../integration/preparation_resources.md)
gemeinsame Budgets und Abbruch ohne globale Zustände oder Referenzen in fertigen Plänen.
