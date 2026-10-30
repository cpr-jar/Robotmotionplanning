# Nachweise zum erneuten Architektur-Review 0.16.0

Basis ausschließlich `5493c3a0d12229372663c73e93c7deec1ce3d79e`, 30.10.2026.
[Review A–H](../../../architecture/architecture_review_v0.16.0.md).
Implementierung und bestehende Tests wurden während des Reviews nicht geändert.

| Datei | Herkunft und Aussage |
| --- | --- |
| release-tests.log | Lokal erneut ausgeführt, 34/34 bestehende CTests; vorhandener Build |
| focused-asan-tests.log | Lokal erneut, sechs fokussierte ASan/UBSan-Vertragstests |
| focused-tsan-tests.log | Lokal erneut, drei fokussierte Clang-19-Threadprotokolltests |
| architecture-tests.log, layers.json | Sechs Guard-Fixtures und gemessener azyklischer Graph |
| mixed-components.log | F13: unterschiedliche Serial-Zulassung bei anderer Linkreihenfolge und TU-Makros |
| empty_differential.log | F14: erwarteter UBSan-Fehler im separat instrumentierten Nullachsenprobe |
| late_cancel.log | F15: Stop im letzten Callback, direkte API liefert noch admitted |
| probe_tools.log | F16: falsche Segment-/Schrittlabels akzeptiert; F17: drei gültige und drei nicht entfernte Ordner nach sechs Läufen |
| remote-visual-assets-tests.log | Nachtrag 1f9077c: sieben Visual-Asset-Tests mit erhaltenen UTF-8-/CRLF-Änderungen |
| ci-evidence.json | Über GitHub-Connector gelesene Run-/Jobzustände und ausgewählte Logzeilen, kein lokaler CI-Nachbau |
| reproductions.md | Originale externe Prüfprogramme als Text sowie Ausführungsbefehle |
| sha256.json | Hashes der aufgeführten Mess-/Reproduktionsdateien, ohne README/Manifest selbst |

GitHub: [Run 41037626654](https://github.com/cpr-jar/Robotmotionplanning/actions/runs/41037626654),
21/21 Jobs erfolgreich, inklusive nativer ARM64-Tests. Der zusätzliche Tag-Run
41037626471 ist ebenfalls erfolgreich. Ein erfolgreicher CI-Lauf schließt die
separat reproduzierten Vertrags-/Kompositionsfehler nicht aus.

Keine vollständige neue lokale Buildmatrix/Fuzzkampagne, kein vollständiges
Drittbibliotheksaudit und keine Pi-/Bus-/Drive-Abnahme. Größere Buildverzeichnisse
und Binaries bleiben außerhalb des Repositorys. Nachgestellte Leerzeichen in
Logs wurden entfernt; übrige Messausgaben sind erhalten.
