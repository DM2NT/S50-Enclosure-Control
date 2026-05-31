# Hardware-Dokumentation

## Vollständige Komponentenliste

### Elektronik

| Komponente | Modell/Spezifikation | Menge | Beschaffung |
|------------|---------------------|-------|-------------|
| ESP32 | DevKit V1 (30 Pin) | 1 | AliExpress/Amazon |
| Regensensor | Hydreon RG-11 | 1 | Hydreon/Conrad |
| Anemometer | Davis 6410 | 1 | Davis Instruments |
| Umweltsensor | BME280 (I2C) | 1 | Adafruit/AliExpress |
| Servo | MG996R 20kg | 1 | Amazon/AliExpress |
| GPS-Modul | NEO-6M oder NEO-7M | 1 | AliExpress (optional) |
| Reed-Schalter | NO (Normally Open) | 2 | Conrad/Reichelt |
| Heizwiderstand | 3 Ohm 15W | 1 | Reichelt |
| MOSFET | IRLZ44N | 1 | Reichelt |
| Widerstände | 10kΩ | 3 | Reichelt |
| Diode | 1N4007 | 1 | Reichelt |

### Mechanik & Gehäuse

| Komponente | Material | Hinweise |
|------------|----------|----------|
| Gehäuse | PLA/PETG | 3D-Druck, wetterfest |
| Servo-Halterung | PETG | 3D-Druck, verstärkt |
| Hebelarm | PLA | 3D-Druck, weiß |
| M3 Schrauben | Edelstahl | Diverse Längen |
| M4 Schrauben | Edelstahl | Servo-Befestigung |

### Kabel

| Typ | Spezifikation | Länge | Verwendung |
|-----|--------------|-------|------------|
| Stromkabel | 1.5mm² | Variabel | 230V → Netzteile |
| Signalkabel | 0.5mm² geschirmt | 5m | RG-11, Anemometer |
| Dupont-Kabel | Female-Female | 20cm | ESP32-Verbindungen |
| USB-Kabel | Micro-USB | 2m | ESP32 Stromversorgung |

### Netzteile

| Ausgang | Leistung | Verwendung |
|---------|----------|------------|
| 5V DC | 2A | ESP32 |
| 6V DC | 2A | Servo |
| 12V DC | 2A | Heizung |

## Detaillierte Verkabelung

### ESP32 Pinout

```
                    ESP32 DevKit V1
                  ┌─────────────────┐
              3V3 │1              30│ GND
              GND │2              29│ GPIO 23
      (Touch) D15 │3              28│ GPIO 22 ──┐ SCL (BME280)
      (Touch)  D2 │4              27│ TXD      │
      (Touch)  D4 │5              26│ RXD      │
              D16 │6 GPS_RX       25│ GPIO 21 ─┼─ SDA (BME280)
              D17 │7 GPS_TX       24│ GND      │
               D5 │8              23│ GPIO 19  │
              D18 │9              22│ GPIO 18  │
              D19 │10             21│ GPIO 5   │
              GND │11             20│ GPIO 17  │
              D21 │12             19│ GPIO 16  │
       RX0    D3  │13             18│ GPIO 4  ─── RAIN_TX
       TX0    D1  │14             17│ GPIO 2   │
      (Touch) D22 │15             16│ GPIO 15  │
              D23 │16             15│ GND      │
              GND │17             14│ GPIO 13 ─── SERVO_PIN
              3V3 │18             13│ GPIO 12 ─── HEATER_PIN
               EN │19             12│ GPIO 14  │
     ADC2_0   VP  │20             11│ GPIO 27 ─── WIND_PIN
     ADC2_3   VN  │21             10│ GPIO 26  │
   ADC2_4/34  D34 │22              9│ GPIO 25  │
   ADC2_5/35  D35 │23 RAIN_RX      8│ GPIO 33 ─── REED_OPEN
   ADC2_6/32  D32 │24 REED_CLOSED  7│ GPIO 32  │
   ADC2_7/33  D33 │25              6│ TDI/D5   │
   ADC1_4/25  D25 │26              5│ 3V3      │
   ADC1_5/26  D26 │27              4│ TDO/D18  │
   ADC1_6/27  D27 │28              3│ D23      │
   ADC1_7/14  D14 │29              2│ D22      │
   ADC1_0/12  D12 │30              1│ TXD      │
                  └─────────────────┘
```

### RG-11 Regensensor

**Anschluss:**
```
RG-11          ESP32
────────────────────────────
1 (+12V)   →   Externes 12V Netzteil
2 (GND)    →   GND (gemeinsam)
3 (OUT)    →   Pin 35 (RAIN_RX)
               + 10kΩ Pull-Down zu GND
```

**Wichtig:**
- RG-11 sendet 5V → direkter Anschluss an ESP32 ist OK (INPUT_ONLY Pin 35)
- Pull-Down Widerstand verhindert "floating" Pin
- UART Konfiguration: 9600 Baud, 8N1, ASCII Mode

**DIP-Switch Einstellung:**
```
┌─────────────────────────────┐
│  1  2  3  4  5  6  7  8     │
│ ON ON OFF OFF OFF OFF ON OFF│
└─────────────────────────────┘
```

### Davis Anemometer 6410

**Anschluss:**
```
Davis 6410     ESP32
────────────────────────────
Schwarz    →   Pin 27 (WIND_PIN)
Rot        →   3.3V
Grün       →   GND
Gelb       →   Nicht verbunden
```

**Schaltung:**
```
3.3V ──┬── 10kΩ ──┬── Pin 27
       │          │
       └── Reed ──┘ (im Anemometer)
                  │
                 GND
```

**Funktionsweise:**
- Jede Umdrehung → 1 Impuls (Reed-Schalter)
- Wind-Geschwindigkeit = `(1492 / T) * 1.60934` km/h
- T = Zeit zwischen Impulsen in ms

### BME280 Umweltsensor

**Anschluss (I2C):**
```
BME280         ESP32
────────────────────────────
VCC        →   3.3V
GND        →   GND
SCL        →   Pin 22 (SCL)
SDA        →   Pin 21 (SDA)
```

**I2C-Adresse:** `0x76` (Standard) oder `0x77`

### Servo MG996R

**Anschluss:**
```
Servo          Versorgung
────────────────────────────
Braun (GND) →  GND (gemeinsam mit ESP32!)
Rot (VCC)   →  6V/2A Netzteil
Orange (PWM)→  Pin 13 (ESP32)
```

**Wichtig:**
- **NIEMALS** Servo vom ESP32 5V versorgen!
- Separates 6V/2A Netzteil verwenden
- GND zwischen ESP32 und Servo-Netzteil verbinden

**Position:**
- 0° = Offen
- 90° = Geschlossen
- Sanfte Bewegung (1° pro 20ms)

### Reed-Schalter (Endlagen)

**Geschlossen-Position:**
```
Reed 1         ESP32
────────────────────────────
Pin 1      →   Pin 32 (REED_CLOSED)
Pin 2      →   GND
```

**Offen-Position:**
```
Reed 2         ESP32
────────────────────────────
Pin 1      →   Pin 33 (REED_OPEN)
Pin 2      →   GND
```

**Funktionsweise:**
- `INPUT_PULLUP` Mode
- Geschlossen: LOW = Magnet in Nähe
- Offen: HIGH = kein Magnet

### Heizung (Taupunkt-Schutz)

**Schaltung:**
```
12V ──┬── Heizwiderstand 3Ω/15W ──┬── GND
      │                           │
      └── MOSFET (IRLZ44N) ───────┘
              │
              ├── Gate: Pin 12 (ESP32)
              ├── 10kΩ Pull-Down (Gate-GND)
              └── 1N4007 Diode (parallel zum Widerstand)
```

**PWM-Steuerung:**
- Frequenz: 1000 Hz
- Duty Cycle: 0-255 (aktuell: 255 = 100%)

### GPS-Modul (Optional)

**Anschluss:**
```
GPS NEO-6M     ESP32
────────────────────────────
VCC        →   3.3V (NICHT 5V!)
GND        →   GND
TX         →   Pin 16 (GPS_RX)
RX         →   Pin 17 (GPS_TX)
```

**UART:** 9600 Baud, 8N1

## Montage-Hinweise

### Gehäuse

1. **3D-Druck Einstellungen:**
   - Layer: 0.2mm
   - Infill: 20%
   - Wandstärke: 3-4 Perimeter
   - Support: Ja (für Überhänge)

2. **Wetterfest machen:**
   - Alle Kabel-Durchführungen mit Silikon abdichten
   - Reed-Schalter in wasserdichten Gehäusen
   - RG-11 nach oben montieren (Kabel nach unten)

3. **Kabelführung:**
   - Zugentlastung vorsehen
   - Kabelbinder verwenden
   - Beschriftung empfohlen

### Servo-Mechanik

1. **Hebelarm:**
   - 3D-gedruckt aus PLA/PETG
   - Verstärkung mit 100% Infill
   - M3 Schraube zur Befestigung

2. **Servo-Halterung:**
   - Mit M4 Schrauben am Gehäuse befestigen
   - Spielfrei montieren
   - Bewegungsfreiheit prüfen

### Sensor-Platzierung

**RG-11 Regensensor:**
- Horizontal montieren
- Freie Sicht nach oben (keine Überhänge)
- Mindestens 30cm von Wänden entfernt

**Davis Anemometer:**
- Auf höchstem Punkt montieren
- 2m über Dachfläche empfohlen
- Freie Windanströmung

**BME280:**
- Im Gehäuse, aber belüftet
- Vor direkter Sonneneinstrahlung schützen
- Nicht direkt über Heizung

## Stromverbrauch

| Komponente | Standby | Aktiv | Max |
|------------|---------|-------|-----|
| ESP32 | 80mA | 150mA | 250mA |
| Servo | 10mA | 500mA | 2A |
| BME280 | 3µA | 3mA | 5mA |
| GPS | 20mA | 50mA | 50mA |
| Heizung | 0mA | 1.25A | 1.25A |
| **Gesamt** | ~100mA | ~700mA | ~3.5A |

## Fehlersuche Hardware

### RG-11 keine Daten

1. DIP-Switches prüfen (Foto im Web suchen)
2. 12V Versorgung messen
3. Pull-Down Widerstand 10kΩ vorhanden?
4. Serial Monitor: Zeichen empfangen?

### Servo ruckt/zittert

1. Separate Stromversorgung 6V/2A prüfen
2. GND-Verbindung ESP32 ↔ Servo OK?
3. Kabel zu lang? (max 50cm empfohlen)
4. Mechanische Blockade?

### Heizung funktioniert nicht

1. MOSFET richtig gepolt?
2. Pull-Down 10kΩ Gate-GND vorhanden?
3. Freilauf-Diode 1N4007 vorhanden?
4. 12V Versorgung messen

### BME280 nicht erkannt

1. I2C-Adresse prüfen: `0x76` oder `0x77`
2. SDA/SCL vertauscht?
3. Pull-Up Widerstände vorhanden? (meist auf Modul)
4. 3.3V (NICHT 5V!)

### Reed-Schalter falsch

1. `INPUT_PULLUP` Mode gesetzt?
2. Magnet-Abstand zu groß? (max 2cm)
3. Polarität egal bei Reed-Schaltern
4. Kabel-Bruch?

## Wartung

**Regelmäßig (1x Monat):**
- RG-11 Sensor reinigen (Isopropanol)
- Anemometer auf Leichtgängigkeit prüfen
- Servo-Bewegung testen
- Schrauben nachziehen

**Halbjährlich:**
- Gehäuse-Dichtungen prüfen
- Kabel auf Brüche untersuchen
- BME280 kalibrieren (Referenz-Thermometer)

**Jährlich:**
- Firmware-Update
- Komplette Funktionsprüfung
- Backup der Einstellungen

---

**Bei Fragen: Issues auf GitHub öffnen!**

73!
