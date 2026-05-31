# Installation & Setup

## Voraussetzungen

### Software
- Arduino IDE 2.x (empfohlen) oder 1.8.x
- ESP32 Board Support
- USB-Treiber (CP210x oder CH340)

### Hardware
- ESP32 DevKit V1
- USB-Kabel (Micro-USB)
- Computer mit USB-Port

## 1. Arduino IDE einrichten

### 1.1 ESP32 Board Manager installieren

1. Arduino IDE öffnen
2. **Datei → Einstellungen**
3. "Zusätzliche Boardverwalter-URLs" hinzufügen:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. **OK** klicken
5. **Werkzeuge → Board → Boardverwalter**
6. Nach "esp32" suchen
7. "esp32 by Espressif Systems" installieren (Version 2.0.x empfohlen)

### 1.2 Bibliotheken installieren

**Sketch → Bibliothek einbinden → Bibliotheken verwalten**

Folgende Bibliotheken installieren:

| Bibliothek | Version | Autor |
|------------|---------|-------|
| ESP32Servo | Latest | Kevin Harrington |
| Adafruit BME280 Library | Latest | Adafruit |
| Adafruit Unified Sensor | Latest | Adafruit |
| TinyGPSPlus | Latest | Mikal Hart |
| AsyncTCP | Latest | me-no-dev |
| ESPAsyncWebServer | Latest | me-no-dev |

**AsyncTCP und ESPAsyncWebServer manuell installieren:**

1. AsyncTCP: https://github.com/me-no-dev/AsyncTCP
   - "Code → Download ZIP"
   - **Sketch → Bibliothek einbinden → .ZIP-Bibliothek hinzufügen**

2. ESPAsyncWebServer: https://github.com/me-no-dev/ESPAsyncWebServer
   - "Code → Download ZIP"
   - **Sketch → Bibliothek einbinden → .ZIP-Bibliothek hinzufügen**

## 2. Firmware kompilieren

### 2.1 Board-Einstellungen

**Werkzeuge** Menü:

| Einstellung | Wert |
|-------------|------|
| Board | "ESP32 Dev Module" |
| Upload Speed | 921600 |
| CPU Frequency | 240MHz (WiFi/BT) |
| Flash Frequency | 80MHz |
| Flash Mode | QIO |
| Flash Size | 4MB |
| Partition Scheme | **Minimal SPIFFS (1.9MB APP with OTA/190KB SPIFFS)** |
| Core Debug Level | None (oder "Debug" für Fehlersuche) |
| PSRAM | Disabled |
| Port | Automatisch erkennen (z.B. COM3, /dev/ttyUSB0) |

### 2.2 Kompilieren & Hochladen

1. `.ino` Datei öffnen
2. **Werkzeuge → Port** → ESP32-Port auswählen
3. **Sketch → Hochladen** (Pfeil-Symbol oder Strg+U)

**Wichtig während Upload:**
- ESP32 am USB angeschlossen lassen
- Bei manchen Boards: BOOT-Taste gedrückt halten beim Start

**Erfolgreiche Ausgabe:**
```
Leaving...
Hard resetting via RTS pin...
```

## 3. Erstkonfiguration

### 3.1 WiFi Access Point

Nach dem ersten Start öffnet der ESP32 einen Access Point:

```
SSID: S50-Enclosure-XXXXXX
Passwort: 12345678
IP-Adresse: 192.168.4.1
```

**XXXXXX** = Letzte 6 Zeichen der MAC-Adresse

### 3.2 WiFi-Zugangsdaten eingeben

1. Mit Smartphone/Laptop verbinden
2. Browser öffnen: `http://192.168.4.1`
3. Auf "WiFi Einstellungen" klicken
4. SSID und Passwort eingeben
5. **Speichern & Neustart**

ESP32 startet neu und verbindet sich mit dem WiFi.

### 3.3 IP-Adresse herausfinden

**Methode 1: Serial Monitor**
```
Arduino IDE → Werkzeuge → Serieller Monitor
Baudrate: 115200

Ausgabe:
...
WiFi verbunden!
IP-Adresse: 192.168.1.123
```

**Methode 2: Router-Admin-Panel**
- Router-Webinterface öffnen
- DHCP-Client-Liste suchen
- "S50-Enclosure" oder "ESP32" finden

**Methode 3: Netzwerk-Scanner**
- Fing (Android/iOS App)
- Advanced IP Scanner (Windows)
- Angry IP Scanner (Linux/Mac)

## 4. Hardware verbinden

**ACHTUNG: ESP32 NICHT am USB, während Hardware verbunden wird!**

### 4.1 Reihenfolge

1. ✅ Alle Netzteile **AUS**
2. ✅ ESP32 **NICHT** am USB
3. ✅ Sensoren verbinden (siehe HARDWARE.md)
4. ✅ Servo verbinden (6V Netzteil **AUS**)
5. ✅ Heizung verbinden (12V Netzteil **AUS**)
6. ✅ **GND-Verbindungen** prüfen!
7. ✅ Polaritäten prüfen
8. ✅ Kurzschlüsse ausschließen
9. ✅ Netzteile einschalten
10. ✅ ESP32 am USB verbinden

### 4.2 Erste Funktionstests

**Serial Monitor (115200 Baud):**

```
========================================
  Seestar S50 Sternwarten-Controller
  Firmware v3.1
  DM2NT 2025
========================================

WiFi verbunden!
IP-Adresse: 192.168.1.123

BME280 Init...
✓ BME280 gefunden!
  Temperatur: 21.3°C
  Luftfeuchte: 45%

RG-11 UART initialisiert (RX=35, 9600 Baud, ASCII Mode)
  Warte auf RG-11 ASCII Daten (max 10s)...
  'Acc' Frame gefunden: Acc 0.00
✓ RG-11 Regensensor erkannt!

GPS Module Init (optional)...
✓ GPS-Modul gefunden!

Web Server gestartet!
OTA Update bereit auf http://192.168.1.123/update
```

**Web-Interface testen:**
1. Browser: `http://ESP32_IP`
2. Alle Sensorwerte prüfen
3. "Öffnen" Button → Servo bewegt sich?
4. Reed-Schalter-Status prüfen

## 5. Feineinstellungen

### 5.1 Wind-Schwellwert

**Standard:** 40 km/h

**Anpassen:**
1. Web-Interface → "Wind Einstellungen"
2. Neuen Wert eingeben (z.B. 50 km/h)
3. **Speichern**

**Tipp:** Bei exponierter Lage höheren Wert wählen!

### 5.2 Heizungs-Hysterese

**Standard:** 2.0°C

**Bedeutung:**
```
Heizung AN wenn:  Innentemperatur < (Taupunkt + 2.0°C)
Heizung AUS wenn: Innentemperatur > (Taupunkt + 2.0°C)
```

**Anpassen:**
1. Web-Interface → "Heizungs-Einstellungen"
2. Wert eingeben (0.5 - 5.0°C)
3. **Speichern**

### 5.3 Seestar S50 IP-Adresse

**Wichtig:** Damit automatisches Parken funktioniert!

1. Seestar S50 einschalten
2. IP-Adresse herausfinden (Seestar App)
3. Web-Interface → "Seestar Einstellungen"
4. IP eingeben (z.B. `192.168.1.45`)
5. **Speichern**

**Test:**
```
http://SEESTAR_IP:8080/api/v1/system/state
```
→ Sollte JSON zurückgeben

### 5.4 GPS-Modul (Optional)

**Aktivieren:**
1. Web-Interface → "GPS Einstellungen"
2. Haken bei "GPS aktiviert"
3. **Speichern & Neustart**

**Deaktivieren:**
- Haken entfernen → spart 50mA Stromverbrauch

## 6. Servo kalibrieren

### 6.1 Reed-Schalter Position finden

**Geschlossen (90°):**
1. Abdeckung manuell GANZ schließen
2. Reed-Schalter so positionieren, dass Signal LOW
3. Web-Interface prüfen: "Reed Geschlossen: ✓"

**Offen (0°):**
1. Abdeckung manuell GANZ öffnen
2. Reed-Schalter so positionieren, dass Signal LOW
3. Web-Interface prüfen: "Reed Offen: ✓"

### 6.2 Bewegungstest

**Serial Monitor:**
```
Öffne Abdeckung...
Servo: 90° → 89° → 88° → ... → 1° → 0°
✓ Reed OFFEN erreicht!
Abdeckung geöffnet!
```

**Falls Probleme:**
- Servo-Richtung vertauscht? → Hardware umbauen
- Reed-Schalter zu weit? → Näher positionieren
- Motor stoppt zu früh? → Timeout erhöhen (Code)

## 7. RG-11 Regensensor

### 7.1 DIP-Switches einstellen

**ASCII "It's Raining" Mode:**

```
  RG-11 DIP-Switches (Rückseite)
┌─────────────────────────────────┐
│ ┌─┬─┬─┬─┬─┬─┬─┬─┐               │
│ │1│2│3│4│5│6│7│8│               │
│ ├─┼─┼─┼─┼─┼─┼─┼─┤               │
│ │▲│▲│ │ │ │ │▲│ │ ON            │
│ │ │ │▼│▼│▼│▼│ │▼│ OFF           │
│ └─┴─┴─┴─┴─┴─┴─┴─┘               │
└─────────────────────────────────┘
```

| DIP | Position | Funktion |
|-----|----------|----------|
| 1 | **ON** | Empfindlichkeit hoch |
| 2 | **ON** | Empfindlichkeit hoch |
| 3 | OFF | Keine Haltezeit |
| 5 | **OFF** | RS232 aktiv (WICHTIG!) |
| 7 | **ON** | It's Raining Mode |
| 8 | OFF | Normal |

### 7.2 Test

**Wasser auf RG-11 sprühen:**

Serial Monitor sollte zeigen:
```
💧 RG-11: REGEN! Acc=1.00
⚠️ REGEN ERKANNT! (Acc=1.00) Starte Notschließung...
```

Web-Interface:
- Status: **REGEN**
- Menge: **1.00 mm**
- Abdeckung: **GESCHLOSSEN**

## 8. OTA-Updates

**Firmware ohne USB-Kabel aktualisieren:**

1. Neue `.ino` Datei kompilieren
2. **Sketch → Kompilierte Binärdatei exportieren**
3. Datei finden: `Seestar_S50_Sternwarte_vXX.ino.bin`
4. Browser: `http://ESP32_IP/update`
5. Datei auswählen: **NUR die .ino.bin** (NICHT merged.bin!)
6. Upload → Warten → Neustart

**WICHTIG:**
- Während OTA-Update NICHT die Stromversorgung unterbrechen!
- NUR `.ino.bin` verwenden
- Nach Update: Einstellungen prüfen

## 9. Backup & Restore

### 9.1 Einstellungen sichern

ESP32 speichert alle Einstellungen in **NVS (Non-Volatile Storage)**:
- WiFi-Zugangsdaten
- Wind-Schwellwert
- Heizungs-Hysterese
- Seestar IP
- GPS An/Aus

**Backup erstellen:**
1. Serial Monitor öffnen (115200 Baud)
2. Einstellungen notieren:
   ```
   Wind-Schwellwert: 40 km/h
   Heizungs-Hysterese: 2.0°C
   Seestar IP: 192.168.1.45
   GPS: Aktiviert
   ```

**Alternativ:** Esptool.py NVS auslesen
```bash
esptool.py --port COM3 read_flash 0x9000 0x5000 nvs_backup.bin
```

### 9.2 Nach Firmware-Update

**Einstellungen bleiben erhalten!**

Falls zurücksetzen nötig:
1. Web-Interface → "System-Reset"
2. Oder: Reset-Button 10s gedrückt halten → AP-Modus

## 10. Troubleshooting

### Upload schlägt fehl

**"Failed to connect to ESP32"**
- USB-Kabel prüfen (manche sind nur Ladekabel!)
- Richtigen COM-Port gewählt?
- BOOT-Taste gedrückt halten beim Upload-Start
- USB-Treiber installiert? (CP210x/CH340)

**"Sketch zu groß"**
- Partition Scheme: "Minimal SPIFFS" gewählt?
- Alte Bibliotheken löschen
- Debug-Level auf "None"

### WiFi verbindet nicht

1. SSID und Passwort korrekt?
2. 2.4 GHz WiFi? (ESP32 kann KEIN 5 GHz!)
3. SSID sichtbar? (keine Hidden SSID)
4. Zu weit vom Router? (max 20m empfohlen)

### Serial Monitor zeigt Müll

- Baudrate: **115200** einstellen!
- "Both NL & CR" als Zeilenende
- USB-Kabel tauschen

### Sensoren werden nicht erkannt

**BME280:**
```
I2C Scanner laufen lassen:
File → Examples → Wire → i2c_scanner
```

**RG-11:**
- DIP 5 = OFF?
- Pull-Down 10kΩ vorhanden?
- 12V Versorgung?

**GPS:**
- 3.3V (NICHT 5V!)
- TX ↔ RX richtig?
- Draußen testen (Sicht zum Himmel)

## 11. Logs & Debug

### Serial Monitor aktivieren

```cpp
// In Arduino IDE:
Werkzeuge → Core Debug Level → "Debug"
```

**Ausgabe:**
```
[D][WiFiClient.cpp:125] connect(): Connected to 192.168.1.45:8080
[D][HTTPClient.cpp:334] POST: http://192.168.1.45:8080/api/v1/telescope/park
```

### Log-Dateien

ESP32 hat KEIN Dateisystem für Logs!

**Alternative:** Syslog-Server
```cpp
// Externe Bibliothek: Syslog
// Logs an Raspberry Pi / NAS senden
```

## Support

**Probleme?**
- GitHub Issues öffnen
- Serial Monitor Log anhängen
- Hardware-Fotos hochladen
- Firmware-Version angeben

**73!** 📡
