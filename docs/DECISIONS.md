# Projektentscheidungen und Wiederherstellungsumfang

- C++20, CMake, SI-Einheiten. Robot Controller / Sollbahnplanung; keine Servo-Firmware.
- Der Nutzer hat Raspberry Pi 4B, SocketCAN, Nanotec C5-E und CSP als späteres
  Ziel genannt. **CANopen bleibt für die jetzige Arbeit ausdrücklich außen vor.**
- Vorrang haben funktionierende eigene Programme, Sechsachs-Kinematik,
  vollständige Vorabprüfung und Simulation.
- URDF plus versioniertes SI-YAML ist das Kinematik-/Limitformat. Programme
  verwenden ein eigenes striktes SI-YAML-Schema.
- Ruckig Community v0.15.3 bleibt optional, lokal und austauschbar. Kein Cloud-
  Intermediate-Waypoint-Dienst und keine angenommene Pro-Lizenzabhängigkeit.
- Gelenk-Ruckgrenzen implizieren bei nichtlinearer Kinematik keine TCP-Grenzen.
  Deshalb prüfen Programme und generisches Following diese separat.
- Vor Publikation prüfen; Unsupported/Infeasible nicht still zu PTP oder
  ungeprüfter Bewegung abschwächen. Fehler verriegeln, keine Auto-Recovery.
- Thread-/Objektbesitz und irreversibel übergebene Daten sind explizit.
- Öffentliche Robotermodelle sind nominal und ihre Beispielgrenzen Prototyp-
  Vorgaben. Reale Kalibrierung, Payload/Geometrie und unabhängige Maschinensicherheit
  sind keine aus dem Internet ableitbaren Eigenschaften.
- Verlorene Quelltexte wurden aus v0.2.0 und dem Gespräch neu implementiert.
  Deshalb **0.12.1 rekonstruiert**, kein vermeintlich identisches v0.12.0.
- Neue Testergebnisse zählen; frühere Testzahlen aus dem Gespräch werden nicht
  übernommen. Grenzen der Rekonstruktion stehen im Wiederherstellungsbericht.
- Der Nutzer hat `https://github.com/cpr-jar/Robotmotionplanning` als Sicherungsziel
  angegeben und GitHub verbunden. Ein eigener Recovery-Branch schützt vorhandene
  Branches; zusätzliche ZIP-/Git-Bundle-Sicherungen erhalten den Quellstand.
