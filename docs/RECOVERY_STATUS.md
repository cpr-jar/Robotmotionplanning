# Wiederherstellung aus ZIP und Gespräch

Ausgangspunkt ist die vom Nutzer erneut hochgeladene ZIP v0.2.0. Die spätere
Arbeitsumgebung ist nicht verfügbar. Dieser Stand ist eine eigenständige
Rekonstruktion, keine byteidentische Kopie der verlorenen Version v0.12.0.

Rekonstruiert und in funktionalen Tests geprüft: Sechsachs-Kinematik und strikter
URDF/SI-YAML-Import; drei öffentlich belegte nominale Robotermodelle; PTP/LIN/CIRC,
bewegte Randwertpolynome, numerische Intervallhüllen, Zyklusraster, fest dimensionierte
SPSC-Ausführung mit Zeitprüfungen, Live-Planwechsel und Übergabegrenzen, vorbereitete
Stopps mit Ruckig, Conveyor-Historie und kontinuierliches Sechsachs-Tracking;
Programmimport mit benannten Punkten, Geschwindigkeit, WAIT/CALL und CSV-Simulator.
SCARA/Delta-Referenzkinematik, kartesische Spline-Abbildung, Blending und starre
Rahmenkalibrierung sind ebenfalls neu implementiert und werden erneut geprüft.

Die vollständige Abschlussmatrix und die Dokumentation werden vor der finalen
Kennzeichnung aktualisiert. Frühere Testzahlen aus dem Gespräch gelten nicht als
Prüfergebnis dieses Quellstands. CANopen bleibt gemäß Nutzerwunsch ausgeklammert.
Zusätzliche lastabhängige Drehmoment- und vollständige Link-Kollisionsmodule sind
in diesem Rekonstruktionsstand noch nicht wiederhergestellt; TCP-Hindernishüllen
stehen zur Verfügung. Keine Hardware- oder Echtzeit-Zertifizierung wird behauptet.

GitHub-Ziel: https://github.com/cpr-jar/Robotmotionplanning
Branch: recovery/reconstructed-v0.12.1
