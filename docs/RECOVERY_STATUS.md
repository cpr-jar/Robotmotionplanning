# Wiederherstellung aus ZIP und Gespräch — 0.12.1 rekonstruiert

Ausgangspunkt ist die am 29.10.2026 erneut hochgeladene ZIP **v0.2.0**. Diese
Originalquelle ist im ersten Git-Commit erhalten. Die alte Arbeitsumgebung und
die tatsächlichen späteren Quellarchive sind nicht zugänglich. Der aktuelle
Stand wurde daher neu implementiert und getestet; er ist **keine byteidentische
Kopie der verlorenen v0.12.0**. Auch vollständige ABI-Kompatibilität wird nicht
behauptet.

| Bereich | Wiederherstellungsstand |
| --- | --- |
| Sechsachs-Kinematik / Import | Rekonstruiert: FK/J/analytische Ableitungen, lokaler IK, URDF/SI YAML, nominale UR5e/ABB/FANUC-Modelle |
| PTP/LIN/CIRC / Blending | Rekonstruiert: Quintiken, rotierende Posen, C2-Splines/Blends und vollständige numerische Gelenk-/TCP-Hüllen |
| SCARA / Delta | Rekonstruierte Referenzmodelle und Ableitungen; Delta-LIN/CIRC-Hüllen erneut geprüft |
| Programmdateien | Rekonstruiert: Punkte, PTP/LIN/CIRC, Geschwindigkeit, WAIT/CALL, komplette Vorabprüfung, Simulation/CSV/Plots |
| Ausführung | Rekonstruiert: Zyklusraster, SPSC, Uhr-/Datenprüfungen, Live-Generationen, Übergabegrenzen und Snapshots |
| Stopps / Following | Optional rekonstruiert mit lokalem Ruckig, eigener Phasenintegration, vorbereiteten Übergängen und kontinuierlichem XYZ-/Sechsachs-Tracking |
| Kalibrierung / Hindernisse | Starrer Punkt-Fit und TCP-Boxhindernisse wiederhergestellt |
| Linux-Zeitwerkzeuge | Monotonic-Uhr, absolute Wartefunktion, explizites RT-Setup und Host-Timing-Diagnostik rekonstruiert |
| Payload/Torque, vollständige Link-/Selbstkollision | Noch nicht wiederhergestellt; keine entsprechende Prüfung in Programmen behauptet |
| Einzelne ältere Komfort-/Benchmark-APIs | Nicht identisch nachgebildet; aktuelle Einstiegspunkte sind dokumentiert |
| CANopen | Auf ausdrücklichen Nutzerwunsch ausgeklammert |

Die neu gepinnten öffentlichen Quellmodelle haben die im Gespräch dokumentierten
nominalen Nullposen. Exakte Identität mit dem verlorenen damaligen Manifest kann
nicht geprüft werden. Die Herstellerquellen und Lizenzen sind erneut erhalten;
Beispiel-Dynamikgrenzen sind klar als Prototypvorgaben markiert.

**Prüfergebnisse:** [VALIDATION.md](VALIDATION.md). Die vollständige neue lokale
Matrix besteht mit 16,16,14,8,10 Tests für Full-Release, ASan/UBSan, Standard,
Minimal und Ruckig-only. Frühere Zahlen aus dem Gespräch wurden nicht übernommen.

**Dauerhafte Sicherung:** https://github.com/cpr-jar/Robotmotionplanning

Branch `recovery/reconstructed-v0.12.1`; finale Kennzeichnung
`v0.12.1-reconstructed`. Zusätzlich werden ein Quell-ZIP, Git-Bundle und private
Sicherungsseite bereitgestellt. Ein Git-Bundle enthält den Git-Verlauf samt
Quellen und Abhängigkeiten und kann ohne die ursprüngliche Cloud-Umgebung
geklont werden. Native Repository-Zugangsdaten werden nicht in Dateien gespeichert.

Eine neue Rekonstruktion ersetzt keine physische Robotermessung, unabhängig
assurierte Sicherheit oder harten Echtzeitnachweis. Die betreffenden Grenzen
bleiben in Architektur und Roadmap explizit.
