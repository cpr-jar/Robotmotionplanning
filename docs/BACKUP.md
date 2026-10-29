# Projekt wieder öffnen und sichern

## Aus GitHub

```sh
git clone https://github.com/cpr-jar/Robotmotionplanning.git
cd Robotmotionplanning
git checkout v0.12.1-reconstructed-final
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/program_simulator programs/ur5e_demo.yaml --csv trace.csv
```

Zum Weiterentwickeln einen Branch anlegen oder den Recovery-Branch auschecken:

```sh
git checkout recovery/reconstructed-v0.12.1
```

Geprüfte Änderungen anschließend committen und pushen. Ein nur lokaler Commit
schützt nicht vor Verlust der gesamten Cloud-Umgebung.

## Aus dem Git-Bundle

Die Bundle-Datei liegt unter `backups/robot-motion-cpp-v0.12.1-reconstructed.bundle`.
Sie enthält den vollständigen Quellverlauf samt den zum Sicherungszeitpunkt
vorhandenen Tags und Abhängigkeiten. Der nachfolgende Commit, der die Bundle-Datei
selbst hinzufügt, ist naturgemäß nicht Teil dieser Datei. Der gesicherte Quell-Commit
steht in `backups/checksums.json`. Es wird kein Zugriff auf die alte Arbeitsumgebung benötigt.

```sh
git clone backups/robot-motion-cpp-v0.12.1-reconstructed.bundle Robotmotionplanning
cd Robotmotionplanning
git remote set-url origin https://github.com/cpr-jar/Robotmotionplanning.git
git checkout recovery/reconstructed-v0.12.1
```

Danach kann mit den eigenen GitHub-Zugangsdaten normal gepusht werden.
Zugangstokens werden weder in Quelltexten noch im Bundle gespeichert.

## Aus dem ZIP

ZIP vollständig entpacken, im Projektordner die CMake-Befehle oben ausführen.
Die Quellen enthalten alle erforderlichen C++-Abhängigkeiten als hashgebundene
Archive; ein Python- oder Netzwerkzugang ist für den normalen Build unnötig.
ZIP ist eine Dateisicherung; für den Git-Verlauf das Bundle oder Repository nutzen.

Dieser Stand ist eine aus v0.2.0 und Gespräch neu geprüfte Rekonstruktion.
Details, Funktionsgrenzen und Prüfungen: RECOVERY_STATUS.md und VALIDATION.md.
