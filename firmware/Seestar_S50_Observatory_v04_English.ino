/*
 * Seestar S50 enclosure-Steuerung v3.0
 * ESP32 WROOM-32
 * 
 * Features:
 * - WiFi-Manager für einfache WLAN-Konfiguration
 * - Homepage mit Sensordaten, Status und Reed-LEDs
 * - Settings-Seite für Details
 * - Intelligente Open/Close-Logik mit Reed-Checks
 * - RG-11 Rain Sensor (optional, auto-detect)
 * - RTC/GPS Sync mit flüssiger secondsanzeige
 * - Optimiertes Sensor-Timing (Wetter 15s, RTC 60s)
 */

#include <WiFi.h>
#include <WiFiManager.h>  // https://github.com/tzapu/WiFiManager
#include <DNSServer.h>
#include <ESP32Ping.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <SparkFunHTU21D.h>
#include <Adafruit_BMP085.h>  // BMP180
#include <Adafruit_BMP280.h>  // BMP280
#include <RTClib.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Update.h>
#include <LittleFS.h>
#include <HTTPClient.h>

// ============= KONFIGURATION =============

#define FIRMWARE_VERSION "0.3"

// GitHub Image URLs
#define GITHUB_USER "DM2NT"
#define GITHUB_REPO "S50-Enclosure-Control"
#define GITHUB_BRANCH "main"
#define IMG_BASE_URL "https://raw.githubusercontent.com/" GITHUB_USER "/" GITHUB_REPO "/" GITHUB_BRANCH "/assets/status_images/"

// Rain Sensor Schwellwert (0 = trocken, 1 = Rain erkannt)
#define RAIN_THRESHOLD 0.5  // mm/h - Close bei Rain

// Servo-Geschwindigkeit (ms pro Grad)
const int servo_speed = 30;

// Sensor Update-Intervalle
const unsigned long RTC_UPDATE_INTERVAL = 60000;      // 60 seconds
const unsigned long GPS_SYNC_INTERVAL = 600000;       // 10 Minuten

// Pin-Definitionen
#define SERVO1_PIN 16  // Taster-Drücker (S50 Ein/Aus)
#define SERVO2_PIN 17  // Cover
#define SERVO3_PIN 18  // Lock

#define REED1_PIN 25   // Cover closed
#define REED2_PIN 26   // Lock closed
#define REED3_PIN 27   // Tube in Standby

#define RAIN_RX_PIN 35      // RG-11 UART RX
#define RAIN_TX_PIN 36      // RG-11 UART TX (ungenutzt)
#define HEATER_PIN 33       // Heizwiderstand 3Ω @ 5V = 8.3W (MOSFET IRLZ44N)

// ============= GLOBALE VARIABLEN =============

// Servo Objekte
Servo servo1, servo2, servo3;

// Servo Positionen
int servo1_pos = 90, servo2_pos = 90, servo3_pos = 90;

// Servo1 (Taster): 2 Positionen
int servo1_pos1 = 90, servo1_pos2 = 90;

// Servo2 (Cover): 3 Positionen
int servo2_pos1 = 90, servo2_pos2 = 90, servo2_pos3 = 90;

// Servo3 (Lock): 2 Positionen
int servo3_pos1 = 90, servo3_pos2 = 90;

// Reed-Status
bool reed1_state = false, reed2_state = false, reed3_state = false;
bool reed2_enabled = true;  // Riegelüberwachung activeiert (kann deactiveiert werden)

// Rain Sensor RG-11
bool rain_sensor_present = false;
bool rain_detected = false;
float rain_rate = 0.0;        // mm/h
float rain_acc = 0.0;         // mm akkumuliert
int rain_intensity = 0;       // 0-999
float rain_threshold = RAIN_THRESHOLD;  // Aus Preferences loaded
HardwareSerial rainSerial(1); // UART1 für RG-11
String last_rain_close = "";  // Timestamp der letzten Rain-Schließung

// Heater
bool heater_on = false;
float dew_point = 0.0;
// Heater Hysteresis
float heater_hysteresis = 2.0;  // Aus Preferences loaded
// Heater Modus: 0=Enabled (Auto), 1=Disabled
int heater_mode = 0;  // Wird aus Preferences loaded

// RTC Status
bool rtc_present = false;  // Wird im Setup geprüft

// Scheduler/Planer
bool scheduler_enabled = false;
int schedule_open_hour = 20;    // 20:00 Uhr
int schedule_open_min = 0;
int schedule_close_hour = 3;    // 03:00 Uhr
int schedule_close_min = 0;
bool schedule_open_done = false;   // Verhindert mehrfaches Auslösen
bool schedule_close_done = false;

// Preferences
Preferences preferences;

// Wetter-Sensoren
HTU21D sht21_innen;
Adafruit_BMP085 bmp180_aussen;  // BMP180
Adafruit_BMP280 bmp280_aussen;  // BMP280
bool bmp180_present = false;    // Flag ob BMP180 vorhanden
bool bmp280_present = false;    // Flag ob BMP280 vorhanden
String bmp_sensor_type = "None";  // "BMP180" oder "BMP280"
float temp_innen = 0.0, hum_innen = 0.0;
float temp_aussen = 0.0, druck_aussen = 0.0;

// RTC mit millis() Tracking
RTC_DS3231 rtc;
DateTime lastRTCRead;
unsigned long lastRTCMillis = 0, lastRTCUpdate = 0;
String rtc_time_local = "--:--:--", rtc_time_utc = "--:--:--", rtc_date = "--.--.----";
int timezone_offset = 1;
bool is_dst = false;

// GPS
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
String gps_date = "--.--.----", gps_time = "--:--:--";
int gps_sats = 0;
bool gps_fix = false;
unsigned long last_gps_sync = 0;

// enclosure-Status
String status_text = "Initialisierung...";
String status_color = "#FFA500";
String last_error_message = "";  // Letzte Fehlermeldung für Interface

// Action Flags (für non-blocking Ausführung in loop)
enum PendingAction { NONE, OPEN, CLOSE, TOGGLE_S50 };
PendingAction pending_action = NONE;

// Webserver und DNS für AP-Modus
AsyncWebServer server(80);
// DNSServer dnsServer;  // Nicht mehr nötig - WiFiManager hat eigenen
bool apMode = false;

// WiFi Info
String current_ssid = "";
int wifi_rssi = 0;
String wifi_ip = "";

// Seestar S50 Ping
String seestar_ip = "192.168.1.100";  // Default IP
bool seestar_online = false;
bool seestar_online_last = false;  // Letzter Status für Statuswechsel-Erkennung
unsigned long last_ping_check = 0;

// ============= HELPER FUNCTIONS =============

// RTC Time mit millis() hochzählen für flüssige Anzeige
void updateRTCTime() {
  unsigned long currentMillis = millis();
  
  // RTC nur alle 60s neu auslesen (when vorhanden!)
  if (rtc_present && (currentMillis - lastRTCUpdate >= RTC_UPDATE_INTERVAL)) {
    lastRTCRead = rtc.now();
    lastRTCMillis = currentMillis;
    lastRTCUpdate = currentMillis;
  }
  
  // Berechne vergangene seconds seit letztem RTC-Read
  unsigned long elapsedSeconds = (currentMillis - lastRTCMillis) / 1000;
  DateTime currentTime = lastRTCRead + TimeSpan(elapsedSeconds);
  
  // Sommerzeit prüfen
  is_dst = isDST(currentTime.day(), currentTime.month(), currentTime.dayOfTheWeek());
  int local_offset = timezone_offset + (is_dst ? 1 : 0);
  
  // Localzeit berechnen
  DateTime localTime = currentTime + TimeSpan(0, local_offset, 0, 0);
  
  // Time formatieren
  char timeBufferLocal[9], timeBufferUTC[9], dateBuffer[11];
  sprintf(timeBufferLocal, "%02d:%02d:%02d", localTime.hour(), localTime.minute(), localTime.second());
  sprintf(timeBufferUTC, "%02d:%02d:%02d", currentTime.hour(), currentTime.minute(), currentTime.second());
  sprintf(dateBuffer, "%02d.%02d.%04d", localTime.day(), localTime.month(), localTime.year());
  
  rtc_time_local = String(timeBufferLocal);
  rtc_time_utc = String(timeBufferUTC);
  rtc_date = String(dateBuffer);
}

// Berechne ob Sommerzeit active ist (Europa)
bool isDST(int day, int month, int dow) {
  if (month < 3 || month > 10) return false;
  if (month > 3 && month < 10) return true;
  
  int lastSunday = day - dow;
  while (lastSunday + 7 <= 31) lastSunday += 7;
  
  if (month == 3) return (day >= lastSunday);
  if (month == 10) return (day < lastSunday);
  
  return false;
}

// Synchronisiere RTC mit GPS-Time
void syncRTCwithGPS() {
  if (!rtc_present) return;  // Ohne RTC kein Sync möglich!
  if (!gps.date.isValid() || !gps.time.isValid()) return;
  
  DateTime gpsTime(gps.date.year(), gps.date.month(), gps.date.day(),
                   gps.time.hour(), gps.time.minute(), gps.time.second());
  
  rtc.adjust(gpsTime);
  lastRTCRead = gpsTime;
  lastRTCMillis = millis();
  lastRTCUpdate = millis();
  
  Serial.println("RTC mit GPS synchronisiert (UTC)");
  Serial.printf("GPS-Time: %02d.%02d.%04d %02d:%02d:%02d UTC\n",
                gps.date.day(), gps.date.month(), gps.date.year(),
                gps.time.hour(), gps.time.minute(), gps.time.second());
}

// BMP Sensor Wrapper-Funktionen (beide Sensoren unterstützen)
float readBMPTemperature() {
  if (bmp280_present) {
    return bmp280_aussen.readTemperature();
  } else if (bmp180_present) {
    return bmp180_aussen.readTemperature();
  }
  return 0.0;
}

float readBMPPressure() {
  if (bmp280_present) {
    return bmp280_aussen.readPressure() / 100.0;  // Pa → hPa
  } else if (bmp180_present) {
    return bmp180_aussen.readPressure() / 100.0;  // Pa → hPa
  }
  return 0.0;
}

bool isBMPPresent() {
  return (bmp180_present || bmp280_present);
}

// Langsame Servo-Bewegung
void moveServoSlow(Servo &servo, int &current_pos, int target_pos) {
  if (current_pos < target_pos) {
    for (int pos = current_pos; pos <= target_pos; pos++) {
      servo.write(pos);
      delay(servo_speed);
      yield();  // Gib anderen Tasks Time
    }
  } else {
    for (int pos = current_pos; pos >= target_pos; pos--) {
      servo.write(pos);
      delay(servo_speed);
      yield();  // Gib anderen Tasks Time
    }
  }
  current_pos = target_pos;
}

// RG-11 Daten lesen und parsen
void readRainSensor() {
  static String buffer = "";
  
  while (rainSerial.available()) {
    char c = rainSerial.read();
    
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        // Parse RG-11 Ausgabe
        // Format: "Acc 0.01, EventAcc 0.00, TotalAcc 0.01, RInt 001.234"
        
        if (buffer.startsWith("Acc ")) {
          rain_acc = buffer.substring(4).toFloat();
        }
        else if (buffer.startsWith("RInt ")) {
          rain_rate = buffer.substring(5).toFloat();
          rain_intensity = (int)(rain_rate * 10);  // Visualisierung
          
          // Rain erkannt when über Schwellenwert
          rain_detected = (rain_rate >= rain_threshold);
        }
        
        buffer = "";
      }
    } else {
      buffer += c;
      if (buffer.length() > 100) buffer = "";  // Overflow-Schutz
    }
  }
}

// Reed Contacts auslesen
void readReedContacts() {
  reed1_state = !digitalRead(REED1_PIN);
  reed2_state = !digitalRead(REED2_PIN);
  reed3_state = !digitalRead(REED3_PIN);
}

// Dew Point berechnen (Magnus-Formel)
float calculateDewPoint(float temp_c, float humidity) {
  // Sicherheitsabfragen
  if (humidity < 1.0) humidity = 1.0;
  if (humidity > 100.0) humidity = 100.0;
  if (temp_c < -40.0 || temp_c > 80.0) return 0.0;  // Invalide Temperature
  
  // Magnus-Formel Konstanten
  const float a = 17.27;
  const float b = 237.7;
  
  // Berechne alpha
  float alpha = ((a * temp_c) / (b + temp_c)) + log(humidity / 100.0);
  
  // Berechne Dew Point
  float dewpoint = (b * alpha) / (a - alpha);
  
  // Sicherheitscheck Ergebnis
  if (isnan(dewpoint) || isinf(dewpoint)) return 0.0;
  if (dewpoint < -40.0 || dewpoint > 80.0) return 0.0;
  
  return dewpoint;
}

// Seestar S50 Ping Check
void checkSeestarOnline() {
  if (seestar_ip == "" || seestar_ip == "0.0.0.0") {
    seestar_online = false;
    return;
  }
  
  // Ping mit 2 seconds Timeout
  seestar_online = Ping.ping(seestar_ip.c_str(), 2);
  
  // Nur ausgeben when sich Status geändert hat!
  if (seestar_online != seestar_online_last) {
    if (seestar_online) {
      Serial.printf("✓ Seestar S50 online (%s)\n", seestar_ip.c_str());
    } else {
      Serial.printf("✗ Seestar S50 offline (%s)\n", seestar_ip.c_str());
    }
    seestar_online_last = seestar_online;  // Status merken
  }
}

// Timegesteuerter Planer (Scheduler)
void checkScheduler() {
  if (!scheduler_enabled) return;
  if (!rtc_present) return;  // Ohne RTC kein Scheduler möglich!
  
  // RTC-Time aktualisieren
  DateTime now = rtc.now();
  DateTime local = now + TimeSpan(0, timezone_offset + (is_dst ? 1 : 0), 0, 0);
  
  int current_hour = local.hour();
  int current_min = local.minute();
  
  // Prüfe OPEN
  if (current_hour == schedule_open_hour && current_min == schedule_open_min) {
    if (!schedule_open_done) {
      Serial.printf("⏰ SCHEDULER: Opening time erreicht (%02d:%02d)\n", 
                    schedule_open_hour, schedule_open_min);
      
      // WICHTIG: Prüfe Rain!
      if (rain_detected && rain_sensor_present) {
        Serial.println("⚠️ OPEN ABORTED: Rain erkannt!");
        last_error_message = "⏰ Geplantes Open abgebrochen - Rain!";
      } else {
        Serial.println("→ Starte Open...");
        pending_action = OPEN;
      }
      
      schedule_open_done = true;
    }
  } else {
    schedule_open_done = false;  // Reset für nächsten Tag
  }
  
  // Prüfe CLOSE
  if (current_hour == schedule_close_hour && current_min == schedule_close_min) {
    if (!schedule_close_done) {
      Serial.printf("⏰ SCHEDULER: Closing time erreicht (%02d:%02d)\n", 
                    schedule_close_hour, schedule_close_min);
      Serial.println("→ Starte Close...");
      pending_action = CLOSE;
      schedule_close_done = true;
    }
  } else {
    schedule_close_done = false;  // Reset für nächsten Tag
  }
}

// Heater Steuerung (mit Hysteresis)
void controlHeater() {
  // Dew Point IMMER berechnen (auch bei openem enclosure für Anzeige!)
  dew_point = calculateDewPoint(temp_innen, hum_innen);
  
  // Wenn Heater deactiveiert (heater_mode = 1) → nichts tun
  if (heater_mode == 1) {
    // Heater ist deactiveiert
    if (heater_on) {
      digitalWrite(HEATER_PIN, LOW);
      heater_on = false;
      Serial.println("Heater OFF (Disabled)");
    }
    return;
  }
  
  // Ab hier: Auto-Modus (heater_mode == 0)
  
  // Nur when enclosure closed (Reed1 + Reed2, aber Reed2 optional)
  readReedContacts();
  
  // Prüfe Reed1 immer, Reed2 only if activeiert
  if (!reed1_state || (reed2_enabled && !reed2_state)) {
    // enclosure open oder not locked → Heater OFF
    if (heater_on) {
      digitalWrite(HEATER_PIN, LOW);
      heater_on = false;
      Serial.println("Heater OFF (Observatory open)");
    }
    return;
  }
  
  // Heater Logik mit Hysteresis (INNEN-Temperature!)
  if (!heater_on) {
    // Heater ist OFF → ON when Inside temp <= Dew Point
    if (temp_innen <= dew_point) {
      digitalWrite(HEATER_PIN, HIGH);
      heater_on = true;
      Serial.printf("Heater ON (Innen: %.1f°C <= Dew Point: %.1f°C)\n", temp_innen, dew_point);
    }
  } else {
    // Heater ist ON → OFF when Inside temp > Dew Point + Hysteresis
    if (temp_innen > dew_point + heater_hysteresis) {
      digitalWrite(HEATER_PIN, LOW);
      heater_on = false;
      Serial.printf("Heater OFF (Innen: %.1f°C > Dew Point+Hyst: %.1f°C)\n", temp_innen, dew_point + heater_hysteresis);
    }
  }
}

// Berechne enclosure-Status für Text-Anzeige
void updateStatusText() {
  readReedContacts();
  
  // Effektiver Reed2-Status (immer true when deactiveiert)
  bool reed2_effective = reed2_enabled ? reed2_state : true;
  
  // CLOSED GRÜN: Alles zu und verriegelt
  if (reed1_state && reed2_effective && reed3_state) {
    status_text = "CLOSED ✓";
    status_color = "#00FF00";
    return;
  }
  
  // OPEN GRÜN: Cover auf Position 2 UND Reed1+2 inactive
  if (servo2_pos == servo2_pos2 && !reed1_state && (reed2_enabled ? !reed2_state : true)) {
    status_text = "OPEN ✓";
    status_color = "#00FF00";
    return;
  }
  
  // HALF-OPEN: Position 3
  if (servo2_pos == servo2_pos3) {
    status_text = "HALF-OPEN";
    status_color = "#FFA500";
    return;
  }
  
  // ERROR ROT: Cover soll zu sein, aber Reeds nicht active (nur Reed1 prüfen when Reed2 deactiveiert)
  if (servo2_pos == servo2_pos1 && (!reed1_state || (reed2_enabled && !reed2_state))) {
    status_text = "ERROR!";
    status_color = "#FF0000";
    return;
  }
  
  // MOVING
  status_text = "MOVING";
  status_color = "#FFA500";
}

String getStatusDetail() {
  String detail = "";
  
  // Spezifische Fehlermeldungen bei closeder Cover
  if (servo2_pos == servo2_pos1) {  // Cover soll closed sein
    // Reed2-Check only if activeiert
    bool reed2_check = reed2_enabled ? !reed2_state : false;
    
    if (!reed1_state && reed2_check) {
      return "⚠️ Cover UND Lock stuck!";
    } else if (!reed1_state) {
      return "⚠️ Cover stuck - nicht closed!";
    } else if (reed2_check) {
      return "⚠️ Lock stuck - not locked!";
    }
  }
  
  // Normale Status-Anzeige
  if (reed1_state) detail += "Cover OK ";
  else detail += "Cover open ";
  
  // Reed2 nur anzeigen when activeiert
  if (reed2_enabled) {
    if (reed2_state) detail += "| Riegel OK ";
    else detail += "| Riegel open ";
  }
  
  if (reed3_state) detail += "| Tube OK";
  else detail += "| Tube active";
  
  return detail;
}

// ============= ABLAUF-FUNKTIONEN =============

// Kuppel OPEN mit Reed-Check
bool openDome() {
  Serial.println("=== OPEN-Ablauf START ===");
  
  // 1. Lock entriegeln
  Serial.println("1. Entriegele Lock...");
  moveServoSlow(servo3, servo3_pos, servo3_pos2);
  preferences.putInt("servo3", servo3_pos);
  
  // 2. Waiting for Reed2 = FALSE / max 5 seconds (only if Reed2 activeiert)
  if (reed2_enabled) {
    Serial.println("2. Waiting for Lock open...");
    unsigned long timeout = millis();
    while (millis() - timeout < 5000) {
      readReedContacts();
      if (reed2_state == false) {  // Lock open (Reed inactive)
        Serial.println("   ✓ Lock open!");
        break;
      }
      delay(100);
      yield();
    }
    
    // 3. Check: Hat sich Lock geöffnet?
    readReedContacts();
    if (reed2_state) {
      Serial.println("   ✗ ERROR: Lock stuck!");
      Serial.println("   Position holding position - check manually!");
      last_error_message = "⚠️ OPEN FAILED: Lock stuck!";
      return false;
    }
  } else {
    Serial.println("2. Lock-Check übersprungen (Reed2 deactiveiert)");
  }
  
  // Fehler zurücksetzen bei Erfolg
  last_error_message = "";
  
  // 4. Cover öffnen
  Serial.println("3. Öffne Cover...");
  delay(1000);
  moveServoSlow(servo2, servo2_pos, servo2_pos2);
  preferences.putInt("servo2", servo2_pos);
  
  Serial.println("=== OPEN-Ablauf FERTIG ===");
  return true;
}

// Kuppel CLOSE mit Prüfung
bool closeDome() {
  Serial.println("=== CLOSE-Ablauf START ===");
  
  readReedContacts();
  
  // Check: Lock open? Tube eingefahren?
  // Reed2-Check only if activeiert
  bool verriegelung_ok = reed2_enabled ? (reed2_state == false) : true;  // Muss open sein (oder deactiveiert)
  bool tubus_ok = (reed3_state == true);          // Muss eingefahren sein
  
  Serial.printf("Check: Lock %s | Tube %s\n", 
                verriegelung_ok ? "OK" : "ERROR",
                tubus_ok ? "OK" : "ERROR");
  
  // Falls Tube NICHT eingefahren: Nur HALF-OPEN
  if (!tubus_ok) {
    Serial.println("⚠ Tube not retracted → Cover nur HALF-OPEN");
    moveServoSlow(servo2, servo2_pos, servo2_pos3);
    preferences.putInt("servo2", servo2_pos);
    last_error_message = "⚠️ CLOSE ABORTED: Tube not retracted - Cover half-open!";
    Serial.println("=== CLOSE-Ablauf ABORTED (Halboffen) ===");
    return false;
  }
  
  // 1. Cover schließen
  Serial.println("1. Schließe Cover...");
  moveServoSlow(servo2, servo2_pos, servo2_pos1);
  preferences.putInt("servo2", servo2_pos);
  
  // 1a. Prüfe ob Cover closed (max 3 seconds warten)
  Serial.println("1a. Prüfe Cover closed...");
  unsigned long timeout = millis();
  while (millis() - timeout < 3000) {
    readReedContacts();
    if (reed1_state) {  // Cover closed
      Serial.println("   ✓ Cover closed!");
      break;
    }
    delay(100);
    yield();
  }
  
  readReedContacts();
  if (!reed1_state) {
    Serial.println("   ✗ ERROR: Cover stuck!");
    Serial.println("   Position holding position - check manually!");
    last_error_message = "⚠️ CLOSE FAILED: Cover stuck - nicht closed!";
    return false;
  }
  
  // 2. Warte 3 seconds
  Serial.println("2. Warte 3 seconds...");
  for (int i = 0; i < 30; i++) {
    delay(100);
    yield();
  }
  
  // 3. Lock schließen
  Serial.println("3. Verriegele...");
  moveServoSlow(servo3, servo3_pos, servo3_pos1);
  preferences.putInt("servo3", servo3_pos);
  
  // 3a. Prüfe ob Lock closed (max 3 seconds warten) - only if Reed2 activeiert
  if (reed2_enabled) {
    Serial.println("3a. Prüfe Lock closed...");
    timeout = millis();
    while (millis() - timeout < 3000) {
      readReedContacts();
      if (reed2_state) {  // Lock closed
        Serial.println("   ✓ Lock closed!");
        break;
      }
      delay(100);
      yield();
    }
    
    readReedContacts();
    if (!reed2_state) {
      Serial.println("   ✗ ERROR: Lock stuck!");
      Serial.println("   Position holding position - check manually!");
      last_error_message = "⚠️ CLOSE FAILED: Lock stuck - not locked!";
      return false;
    }
  } else {
    Serial.println("3a. Locks-Check übersprungen (Reed2 deactiveiert)");
  }
  
  // Fehler zurücksetzen bei Erfolg
  last_error_message = "";
  
  Serial.println("=== CLOSE-Ablauf FERTIG ===");
  return true;
}

// S50 Ein/Aus (Taster drücken)
void toggleS50() {
  Serial.println("=== S50 TOGGLE ===");
  moveServoSlow(servo1, servo1_pos, servo1_pos2);
  preferences.putInt("servo1", servo1_pos);
  
  // 2,5 seconds warten (mit yield für Watchdog)
  for (int i = 0; i < 25; i++) {
    delay(100);
    yield();
  }
  
  moveServoSlow(servo1, servo1_pos, servo1_pos1);
  preferences.putInt("servo1", servo1_pos);
  Serial.println("S50 Taster gedrückt (2,5s)");
}

// REGEN-Emergency close (intelligent)
void emergencyRainClose() {
  Serial.println("=== REGEN-NOTSCHLIESSUNG ===");
  
  // Timestamp erstellen
  updateRTCTime();
  last_rain_close = rtc_date + " " + rtc_time_local;
  
  readReedContacts();
  
  // FALL 1: Tube bereits in standby → Sofort komplett schließen
  if (reed3_state) {
    Serial.println("✓ Tube bereits in standby!");
    Serial.println("→ Schließe sofort komplett...");
    
    closeDome();
    
    Serial.println("=== REGEN-NOTSCHLIESSUNG FERTIG (Sofort) ===");
    last_error_message = "";  // Kein Fehler
    return;
  }
  
  // FALL 2: Tube NICHT in standby → Prüfe ob Seestar active
  Serial.println("⚠ Tube nicht in standby!");
  
  // 1. Cover half-open (Schutz!)
  Serial.println("1. Fahre Cover auf HALF-OPEN...");
  moveServoSlow(servo2, servo2_pos, servo2_pos3);
  preferences.putInt("servo2", servo2_pos);
  
  // 2. Prüfe ob Seestar erreichbar
  Serial.println("2. Prüfe Seestar-Status...");
  bool seestar_online = Ping.ping(seestar_ip.c_str(), 1);
  
  if (seestar_online) {
    // Seestar ist ON → Herunterfahren
    Serial.println("   ✓ Seestar ONLINE → Fahre herunter (2,5s Taster)");
    toggleS50();
  } else {
    // Seestar ist schon OFF → NICHT einschalten!
    Serial.println("   ✓ Seestar OFFLINE → Bereits ausgeschaltet, kein Taster!");
  }
  
  // 3. Waiting for Tube eingefahren (max 60 seconds)
  Serial.println("3. Waiting for Tube in standby...");
  unsigned long timeout = millis();
  while (millis() - timeout < 60000) {
    readReedContacts();
    if (reed3_state) {  // Tube eingefahren
      Serial.println("   ✓ Tube in standby!");
      break;
    }
    delay(500);
    yield();
  }
  
  // 4. Prüfe ob Tube wirklich eingefahren
  readReedContacts();
  if (!reed3_state) {
    Serial.println("   ⚠ WARNUNG: Tube nach 60s still not in standby!");
    Serial.println("   → Fahre trotzdem half-open (Schutz vor Rain)");
    last_error_message = "☔ Rain emergency close: Tube-Timeout! Cover half-open!";
    return;  // Bleibt half-open!
  }
  
  // 5. Komplett schließen
  Serial.println("4. Schließe komplett...");
  closeDome();
  
  Serial.println("=== REGEN-NOTSCHLIESSUNG FERTIG (Mit Runterfahren) ===");
  last_error_message = "";  // Kein Fehler
}


// ============= HOMEPAGE HTML =============
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <title>S50 Observatory</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      text-align: center;
      margin: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 20px; }
    .container { max-width: 800px; margin: 0 auto; }
    
    .status-box {
      background: #1a1a1a;
      border: 3px solid #333;
      border-radius: 15px;
      padding: 30px;
      margin: 30px 0;
    }
    .status-text {
      font-size: 48px;
      font-weight: bold;
      margin: 20px 0;
      text-shadow: 0 0 20px currentColor;
    }
    .status-detail {
      font-size: 16px;
      color: #AAA;
      margin-top: 10px;
    }
    
    .reed-status {
      display: flex;
      justify-content: space-around;
      margin: 20px 0;
    }
    .reed-item {
      text-align: center;
      font-size: 14px;
    }
    .led {
      width: 30px;
      height: 30px;
      border-radius: 50%;
      margin: 0 auto 8px;
      border: 2px solid #555;
    }
    .led-off { background-color: #444; }
    .led-on { background-color: #00FF00; box-shadow: 0 0 15px #00FF00; }
    
    .sensor-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(180px, 1fr));
      gap: 15px;
      margin: 20px 0;
    }
    .sensor-box {
      background: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 15px;
    }
    .sensor-box h3 {
      color: #00AAFF;
      margin: 0 0 10px 0;
      font-size: 14px;
    }
    .sensor-value {
      font-size: 24px;
      color: #00FF00;
      font-weight: bold;
      margin: 5px 0;
    }
    .sensor-label {
      font-size: 12px;
      color: #AAA;
    }
    
    .button-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 15px;
      margin: 30px 0;
    }
    button {
      background-color: #0066CC;
      color: white;
      border: none;
      padding: 20px;
      border-radius: 10px;
      cursor: pointer;
      font-size: 18px;
      font-weight: bold;
      transition: all 0.3s;
    }
    button:hover { background-color: #0088FF; transform: scale(1.05); }
    button:active { background-color: #004499; }
    
    .settings-btn {
      padding: 15px 20px;
      font-size: 16px;
    }
    
    .wifi-info {
      position: fixed;
      top: 20px;
      left: 20px;
      background: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 10px 15px;
      font-size: 12px;
      text-align: left;
      min-width: 200px;
    }
    .wifi-info div {
      margin: 5px 0;
    }
    .wifi-signal {
      color: #00FF00;
    }
    
    .rain-warning {
      background: #8B0000;
      border: 2px solid #FF0000;
      padding: 10px;
      margin: 10px 0;
      border-radius: 5px;
      animation: pulse 2s infinite;
    }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.7; }
    }
    
    .rain-info {
      background: #1a4d1a;
      border: 2px solid #00AA00;
      padding: 10px;
      margin: 10px 0;
      border-radius: 5px;
      color: #AAA;
      font-size: 14px;
    }
    
    .copyright {
      position: fixed;
      bottom: 10px;
      right: 10px;
      color: #666;
      font-size: 11px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🔭 S50 Observatory Control</h1>
    
    <div style="display: flex; gap: 10px; margin-bottom: 20px;">
      <button class="settings-btn" onclick="location.href='/scheduler'" style="flex: 1;">⏰ Scheduler</button>
      <button class="settings-btn" onclick="location.href='/settings'" style="flex: 1;">⚙️ Settings</button>
    </div>
    
    <!-- WiFi Info -->
    <div class="wifi-info">
      <div style="color:#00AAFF; font-weight:bold; margin-bottom:8px;">📡 WiFi</div>
      <div><span id="wifi_ssid">--</span></div>
      <div>IP: <span id="wifi_ip">--</span></div>
      <div class="wifi-signal">▂▄▆█ <span id="wifi_rssi">--</span> dBm</div>
    </div>
    
    <!-- Rain-Warning (only if Sensor vorhanden) -->
    <div id="rain_warning" style="display:none;" class="rain-warning">
      ⚠️ REGEN ERKANNT! Emergency close active!
    </div>
    
    <!-- Fehler-Meldung (only if Fehler aufgetreten) -->
    <div id="error_message" style="display:none;" class="rain-warning">
      <span id="error_text"></span>
      <button onclick="document.getElementById('error_message').style.display='none'" style="margin-left:15px; padding:5px 10px;">✕</button>
    </div>
    
    <!-- Rain-Schließung Info (only if erfolgt) -->
    <div id="rain_close_info" style="display:none;" class="rain-info">
      ☔ Letzte Rain-Schließung: <span id="rain_close_time">--</span>
    </div>
    
    <!-- Status-Anzeige -->
    <div class="status-box">
      <img id="status_image" src="" alt="Status" style="max-width: 300px; height: auto; margin: 20px 0;">
      
      <!-- Reed-LEDs -->
      <div class="reed-status">
        <div class="reed-item">
          <div id="led1" class="led led-off"></div>
          <div>Cover<br>geschlossen</div>
        </div>
        <div class="reed-item">
          <div id="led2" class="led led-off"></div>
          <div>Lock<br>geschlossen</div>
        </div>
        <div class="reed-item">
          <div id="led3" class="led led-off"></div>
          <div>Tube<br>Standby</div>
        </div>
      </div>
    </div>
    
    <!-- Sensordaten -->
    <div class="sensor-grid">
      <div class="sensor-box">
        <h3>🌡️ Innen</h3>
        <div class="sensor-value"><span id="temp_i">--</span>°C</div>
        <div class="sensor-value" style="font-size: 18px; margin-top: 10px;"><span id="hum_i">--</span>%</div>
        <div class="sensor-label">Humidity</div>
      </div>
      
      <div class="sensor-box">
        <h3>☁️ Außen</h3>
        <div class="sensor-value"><span id="temp_a">--</span>°C</div>
        <div class="sensor-value" style="font-size: 18px; margin-top: 10px;"><span id="druck_a">--</span> hPa</div>
        <div class="sensor-label">Air Pressure</div>
      </div>
      
      <div class="sensor-box" id="time_box">
        <h3>🕐 Time</h3>
        <div class="sensor-value" style="font-size: 20px;"><span id="time_l">--:--</span></div>
        <div class="sensor-label"><span id="date_l">--.--.--</span> <span id="dst_l">MEZ</span></div>
        <div id="scheduler_info" style="margin-top: 8px; padding: 5px; background: #0a0a0a; border-radius: 5px; border: 1px solid #333; font-size: 12px;">
          <span style="color: #00AAFF;">⏰</span> <span id="scheduler_times" style="color: #888;">Loading...</span>
        </div>
      </div>
      
      <div class="sensor-box">
        <h3 id="gps_h">🛰️ GPS</h3>
        <div class="sensor-value" style="font-size: 16px;"><span id="time_g">--:--</span></div>
        <div class="sensor-label">Sats: <span id="sats_g">0</span></div>
      </div>
      
      <!-- Rain Sensor (nur if present) -->
      <div class="sensor-box" id="rain_box" style="display:none;">
        <h3>🌧️ Rain</h3>
        <div class="sensor-value" id="rain_status">DRY</div>
        
        <!-- Rain-Intensität Balken -->
        <div style="margin: 15px 0;">
          <div style="font-size: 12px; color: #AAA; margin-bottom: 5px;">
            Intensität: <span id="rain_rate_value" style="color: #00AAFF; font-weight: bold;">0.0</span> mm/h
          </div>
          <div style="position: relative; height: 30px; background: #222; border-radius: 5px; overflow: hidden; border: 1px solid #444;">
            <!-- Schwellwert-Markierung -->
            <div style="position: absolute; left: 10%; width: 2px; height: 100%; background: #FF6600; z-index: 2;"></div>
            <div style="position: absolute; left: 10%; top: -20px; color: #FF6600; font-size: 10px; white-space: nowrap;">⚠️ 0.5</div>
            
            <!-- Rain-Balken -->
            <div id="rain_bar" style="height: 100%; width: 0%; background: linear-gradient(90deg, #00AAFF, #0066FF); transition: width 0.5s;"></div>
          </div>
        </div>
        
        <!-- Akkumulierter Rain -->
        <div style="font-size: 11px; color: #888; margin-top: 5px;">
          Akkumuliert: <span id="rain_acc_value" style="color: #AAA;">0.00</span> mm
        </div>
        
        <div class="sensor-label">RG-11 UART Sensor</div>
      </div>
      
      <!-- Heater -->
      <div class="sensor-box">
        <h3 id="heater_icon">🔥 Heater</h3>
        <div class="sensor-value" style="font-size: 20px;" id="heater_status">OFF</div>
        <div class="sensor-label">Dew Point: <span id="dew_point">--</span>°C</div>
      </div>
      
      <!-- Seestar S50 Status -->
      <div class="sensor-box">
        <h3>🔭 Seestar S50</h3>
        <div class="sensor-value" style="font-size: 18px;" id="seestar_status">OFFLINE</div>
        <div class="sensor-label"><span id="seestar_ip">--</span></div>
      </div>
    </div>
    
    <!-- Haupt-Buttons -->
    <div class="button-grid">
      <button onclick="openDome()">▲ Kuppel OPEN</button>
      <button onclick="closeDome()">▼ Kuppel CLOSE</button>
      <button onclick="toggleS50()">⚡ S50 EIN/AUS</button>
    </div>
    
  </div>
  
  <div class="copyright">© DM2NT</div>
  
  <script>
    setInterval(updateStatus, 500);  // Alle 500ms für schnelle LEDs
    
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          // Reed Contacts
          document.getElementById('led1').className = d.reed1 ? 'led led-on' : 'led led-off';
          
          // Reed2 only if activeiert, sonst ausgrauen
          if (d.reed2_enabled) {
            document.getElementById('led2').className = d.reed2 ? 'led led-on' : 'led led-off';
            document.getElementById('led2').style.opacity = '1.0';
          } else {
            document.getElementById('led2').className = 'led led-off';
            document.getElementById('led2').style.opacity = '0.3';
          }
          
          document.getElementById('led3').className = d.reed3 ? 'led led-on' : 'led led-off';
          
          // Sensordaten
          document.getElementById('temp_i').textContent = d.temp_innen.toFixed(1);
          document.getElementById('hum_i').textContent = d.hum_innen.toFixed(0);
          document.getElementById('temp_a').textContent = d.temp_aussen.toFixed(1);
          document.getElementById('druck_a').textContent = d.druck_aussen.toFixed(0);
          document.getElementById('time_l').textContent = d.rtc_time_local;
          document.getElementById('date_l').textContent = d.rtc_date;
          document.getElementById('dst_l').textContent = d.is_dst ? 'MESZ' : 'MEZ';
          
          // Scheduler Timeen
          const schedulerTimes = document.getElementById('scheduler_times');
          if (d.scheduler_enabled) {
            const openTime = String(d.schedule_open_hour).padStart(2, '0') + ':' + String(d.schedule_open_min).padStart(2, '0');
            const closeTime = String(d.schedule_close_hour).padStart(2, '0') + ':' + String(d.schedule_close_min).padStart(2, '0');
            schedulerTimes.innerHTML = `<span style="color: #00FF00;">${openTime} - ${closeTime}</span>`;
          } else {
            schedulerTimes.innerHTML = '<span style="color: #888;">Aus</span>';
          }
          
          // Time-Box färben je nach RTC-Status
          const timeBox = document.getElementById('time_box');
          if (d.rtc_present) {
            timeBox.style.borderColor = '#00FF00';  // Grün
          } else {
            timeBox.style.borderColor = '#FF8800';  // Orange
          }
          
          document.getElementById('time_g').textContent = d.gps_time;
          document.getElementById('sats_g').textContent = d.gps_sats;
          
          // GPS Icon färben
          const gpsH = document.getElementById('gps_h');
          if (d.gps_fix) {
            gpsH.style.filter = 'none';
            gpsH.style.textShadow = '0 0 10px #00FF00';
          } else {
            gpsH.style.filter = 'grayscale(100%)';
            gpsH.style.textShadow = 'none';
          }
          
          // WiFi Info
          document.getElementById('wifi_ssid').textContent = d.wifi_ssid;
          document.getElementById('wifi_ip').textContent = d.wifi_ip;
          document.getElementById('wifi_rssi').textContent = d.wifi_rssi;
          
          // Signal-Stärke färben
          const wifiSignal = document.querySelector('.wifi-signal');
          if (d.wifi_rssi > -50) {
            wifiSignal.style.color = '#00FF00';  // Exzellent
          } else if (d.wifi_rssi > -60) {
            wifiSignal.style.color = '#AAFF00';  // Sehr gut
          } else if (d.wifi_rssi > -70) {
            wifiSignal.style.color = '#FFAA00';  // Gut
          } else {
            wifiSignal.style.color = '#FF6600';  // Schwach
          }
          
          // Rain Sensor (nur if present)
          if (d.rain_sensor_present) {
            document.getElementById('rain_box').style.display = 'block';
            const rainStatus = document.getElementById('rain_status');
            const rainWarning = document.getElementById('rain_warning');
            
            // Status-Text und Farbe
            if (d.rain_detected) {
              rainStatus.textContent = 'REGEN!';
              rainStatus.style.color = '#FF0000';
              rainWarning.style.display = 'block';
            } else {
              rainStatus.textContent = 'DRY';
              rainStatus.style.color = '#00FF00';
              rainWarning.style.display = 'none';
            }
            
            // Intensitäts-Values
            document.getElementById('rain_rate_value').textContent = d.rain_rate.toFixed(1);
            document.getElementById('rain_acc_value').textContent = d.rain_acc.toFixed(2);
            
            // Balken-Grafik (max 5 mm/h = 100%)
            const rainBar = document.getElementById('rain_bar');
            
            if (d.rain_rate <= 0) {
              // Kein Rain = leerer Balken
              rainBar.style.width = '0%';
              rainBar.style.background = '#444';  // Grau/unsichtbar
            } else {
              const percentage = Math.min((d.rain_rate / 5.0) * 100, 100);
              rainBar.style.width = percentage + '%';
              
              // Farbe basierend auf Intensität
              if (d.rain_rate >= 2.0) {
                rainBar.style.background = 'linear-gradient(90deg, #FF0000, #FF6600)';  // Stark
              } else if (d.rain_rate >= 0.5) {
                rainBar.style.background = 'linear-gradient(90deg, #FF6600, #FFAA00)';  // Mittel
              } else if (d.rain_rate > 0.1) {
                rainBar.style.background = 'linear-gradient(90deg, #FFAA00, #00AAFF)';  // Leicht
              } else {
                rainBar.style.background = 'linear-gradient(90deg, #00AAFF, #0066FF)';  // Sehr leicht
              }
            }
          }
          
          // Heater
          const heaterStatus = document.getElementById('heater_status');
          const heaterIcon = document.getElementById('heater_icon');
          const dewPoint = document.getElementById('dew_point');
          
          dewPoint.textContent = d.dew_point.toFixed(1);
          
          if (d.heater_on) {
            heaterStatus.textContent = 'AN';
            heaterStatus.style.color = '#FF6600';
            heaterIcon.style.filter = 'none';
            heaterIcon.style.textShadow = '0 0 10px #FF6600';
          } else {
            heaterStatus.textContent = 'AUS';
            heaterStatus.style.color = '#00FF00';
            heaterIcon.style.filter = 'grayscale(100%)';
            heaterIcon.style.textShadow = 'none';
          }
          
          // Seestar S50
          const seestarStatus = document.getElementById('seestar_status');
          const seestarIP = document.getElementById('seestar_ip');
          
          seestarIP.textContent = d.seestar_ip;
          
          if (d.seestar_online) {
            seestarStatus.textContent = 'ONLINE';
            seestarStatus.style.color = '#00FF00';
          } else {
            seestarStatus.textContent = 'OFFLINE';
            seestarStatus.style.color = '#FF0000';
          }
          
          // Status-Bild aktualisieren
          const statusImg = document.getElementById('status_image');
          let imageName = '';
          
          if (d.status_text.includes('CLOSED')) {
            imageName = d.last_error ? 'S50_Geschlossen_rot.png' : 'S50_Geschlossen_gruen.png';
          } else if (d.status_text.includes('OPEN')) {
            imageName = 'S50_offen_gruen.png';
          } else if (d.status_text.includes('HALF-OPEN')) {
            imageName = 'S50_halboffen_orange.png';
          } else {
            imageName = 'S50_Geschlossen_rot.png';  // Fehler
          }
          
          statusImg.src = '/image?name=' + imageName;
          
          // Fehlermeldung anzeigen (falls vorhanden)
          const errorBox = document.getElementById('error_message');
          const errorText = document.getElementById('error_text');
          if (d.last_error && d.last_error !== '') {
            errorText.textContent = d.last_error;
            errorBox.style.display = 'block';
          } else {
            errorBox.style.display = 'none';
          }
          
          // Rain-Schließung Info (falls vorhanden)
          const rainCloseBox = document.getElementById('rain_close_info');
          const rainCloseTime = document.getElementById('rain_close_time');
          if (d.last_rain_close && d.last_rain_close !== '') {
            rainCloseTime.textContent = d.last_rain_close;
            rainCloseBox.style.display = 'block';
          } else {
            rainCloseBox.style.display = 'none';
          }
        });
    }
    
    function openDome() {
      if(confirm('Kuppel öffnen?')) {
        fetch('/action?cmd=open')
          .then(response => {
            if (response.ok) {
              console.log('Öffne...');
            }
          })
          .catch(e => console.error('Error:', e));
      }
    }
    
    function closeDome() {
      if(confirm('Kuppel schließen?')) {
        fetch('/action?cmd=close')
          .then(response => {
            if (response.ok) {
              console.log('Schließe...');
            }
          })
          .catch(e => console.error('Error:', e));
      }
    }
    
    function toggleS50() {
      fetch('/action?cmd=toggle').then(r => r.text()).then(t => console.log(t));
    }
  </script>
</body>
</html>
)rawliteral";


// ============= SETTINGS HTML =============
const char settings_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <title>S50 Settings</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { background-color: #000; color: #FFF; font-family: Arial, sans-serif; text-align: center; margin: 20px; }
    h1 { color: #00FF00; margin-bottom: 30px; }
    .container { max-width: 1200px; margin: 0 auto; }
    .columns { display: flex; gap: 20px; align-items: flex-start; }
    .column-left { flex: 1; }
    .column-right { flex: 2; }
    .section { background-color: #1a1a1a; border: 2px solid #333; border-radius: 10px; padding: 20px; margin-bottom: 20px; }
    .section h2 { color: #00AAFF; margin-top: 0; }
    
    .reed-status { display: flex; justify-content: space-around; margin: 20px 0; }
    .reed-item { text-align: center; }
    .led { width: 40px; height: 40px; border-radius: 50%; margin: 10px auto; border: 2px solid #555; }
    .led-off { background-color: #444; }
    .led-on { background-color: #00FF00; box-shadow: 0 0 20px #00FF00; }
    
    .servo-control { margin: 20px 0; }
    .servo-row { display: flex; justify-content: space-between; align-items: center; margin: 15px 0; padding: 10px; background-color: #222; border-radius: 5px; }
    .servo-label { flex: 1; text-align: left; font-weight: bold; }
    .servo-pos { flex: 1; font-size: 18px; color: #00AAFF; }
    .servo-buttons { flex: 1; text-align: right; }
    
    button { background-color: #0066CC; color: white; border: none; padding: 10px 20px; margin: 0 5px; border-radius: 5px; cursor: pointer; font-size: 16px; transition: background-color 0.3s; }
    button:hover { background-color: #0088FF; }
    button:active { background-color: #004499; }
    button.small { font-size: 12px; padding: 5px 10px; }
    
    .weather-value { font-size: 32px; color: #00FF00; font-weight: bold; margin: 10px 0; }
    .weather-label { font-size: 14px; color: #AAA; }
    
    .home-btn { position: fixed; top: 20px; left: 20px; padding: 10px 20px; }
    
    .rain-info {
      background: #1a4d1a;
      border: 2px solid #00AA00;
      padding: 10px;
      margin: 10px 0;
      border-radius: 5px;
      color: #AAA;
      font-size: 14px;
    }
    
    .copyright {
      position: fixed;
      bottom: 10px;
      right: 10px;
      color: #666;
      font-size: 11px;
    }
    
    .version {
      position: fixed;
      bottom: 10px;
      left: 10px;
      color: #666;
      font-size: 11px;
    }
    
    .sensor-found { color: #00FF00; }
    .sensor-missing { color: #FF6600; }
  </style>
</head>
<body>
  <div class="container">
    <h1>⚙️ S50 Settings</h1>
    
    <button class="home-btn" onclick="location.href='/'">🏠 Back</button>
    
    <!-- Fehler-Meldung (only if Fehler aufgetreten) -->
    <div id="error_message" style="display:none; background:#8B0000; border:2px solid #FF0000; padding:15px; margin:20px 0; border-radius:5px; animation:pulse 2s infinite;">
      <span id="error_text"></span>
      <button onclick="document.getElementById('error_message').style.display='none'" style="margin-left:15px; padding:5px 10px;">✕</button>
    </div>
    
    <!-- Rain-Schließung Info (only if erfolgt) -->
    <div id="rain_close_info" style="display:none;" class="rain-info">
      ☔ Letzte Rain-Schließung: <span id="rain_close_time">--</span>
    </div>
    
    <div class="columns">
      <!-- Linke Spalte: Weather Data -->
      <div class="column-left">
        
        <div class="section">
         <h2>🌡️ Wetter Innen</h2>
         <div style="text-align: center;">
           <div class="weather-value"><span id="temp_innen">--</span> °C</div>
           <div class="weather-label">Temperature</div>
           <div class="weather-value" style="margin-top: 20px;"><span id="hum_innen">--</span> %</div>
           <div class="weather-label">Humidity</div>
         </div>
        </div>
        
        <div class="section">
         <h2>☁️ Wetter Außen</h2>
         <div style="text-align: center;">
           <div class="weather-value"><span id="temp_aussen">--</span> °C</div>
           <div class="weather-label">Temperature</div>
           <div class="weather-value" style="margin-top: 20px;"><span id="druck_aussen">--</span> hPa</div>
           <div class="weather-label">Air Pressure</div>
         </div>
        </div>
        
        <div class="section" id="rtc_section">
         <h2>🕐 Time (RTC)</h2>
         <div style="text-align: center;">
           <div class="weather-label">Localzeit <span id="dst_label">(MEZ)</span></div>
           <div class="weather-value" style="font-size: 28px;"><span id="rtc_time_local">--:--:--</span></div>
           <div class="weather-label" style="margin-top: 15px;">UTC</div>
           <div class="weather-value" style="font-size: 20px; color: #00AAFF;"><span id="rtc_time_utc">--:--:--</span></div>
           <div class="weather-label" style="margin-top: 10px;"><span id="rtc_date">--.--.----</span></div>
           <div style="margin-top: 15px;">
             <span style="color: #AAA;">Timezone: UTC</span>
             <button onclick="adjustTimezone(-1)" style="margin: 0 5px; padding: 5px 15px;">-</button>
             <span id="timezone_offset" style="color: #00FF00; font-size: 20px;">+1</span>
             <button onclick="adjustTimezone(1)" style="margin: 0 5px; padding: 5px 15px;">+</button>
           </div>
         </div>
        </div>
        
        <div class="section">
         <h2><span id="gps_sat_icon" style="filter: grayscale(100%);">🛰️</span> GPS</h2>
         <div style="text-align: center;">
           <div class="weather-value" style="font-size: 20px;"><span id="gps_time">--:--:--</span></div>
           <div class="weather-label"><span id="gps_date">--.--.----</span></div>
           <div style="margin-top: 15px; color: #00AAFF;">
             Satellites: <span id="gps_sats">0</span> | Fix: <span id="gps_fix">Nein</span>
           </div>
         </div>
        </div>
        
        <!-- Rain Sensor (nur if present) -->
        <div class="section" id="rain_section" style="display:none;">
         <h2>🌧️ Rain Sensor</h2>
         <div style="text-align: center;">
           <div class="weather-value" id="rain_status">DRY</div>
           <div class="weather-label">RG-11 Sensor</div>
         </div>
        </div>
        
        <!-- Heater -->
        <div class="section">
         <h2><span id="heater_icon_settings">🔥</span> Heater</h2>
         <div style="text-align: center;">
           <div class="weather-value" id="heater_status_settings">OFF</div>
           <div class="weather-label">Status</div>
           <div style="margin-top: 15px; color: #00AAFF;">
             Dew Point: <span id="dew_point_settings">--</span>°C
           </div>
           <div style="margin-top: 10px; font-size: 12px; color: #AAA;">
             Only when closeder Observatory
           </div>
         </div>
        </div>
        
      </div>
      
      <!-- Rechte Spalte: Reed Contacts & Servos -->
      <div class="column-right">
    
        <div class="section">
          <h2>Status Reed Contacts</h2>
          <div class="reed-status">
            <div class="reed-item">
              <div id="led1" class="led led-off"></div>
              <div>Cover<br>geschlossen</div>
            </div>
            <div class="reed-item">
              <div id="led2" class="led led-off"></div>
              <div>Lock<br>geschlossen</div>
            </div>
            <div class="reed-item">
              <div id="led3" class="led led-off"></div>
              <div>Tube<br>Standby</div>
            </div>
          </div>
        </div>
        
        <div class="section">
          <h2>Riegelüberwachung (Reed2)</h2>
          <div style="margin-bottom: 15px; color: #AAA; font-size: 14px;">
            <strong>Status:</strong> <span id="reed2_status" style="color: #00AAFF;">Enabled</span>
          </div>
          
          <button id="reed2_toggle" onclick="toggleReed2()" style="background:#00AA00; border: 2px solid #00FF00;">
            ✅ Riegelüberwachung AKTIVIERT
          </button>
          
          <div style="margin-top:15px; color:#888; font-size:12px;">
            <strong>ℹ️ Note:</strong><br>
            Bei Problemen mit der Riegelmechanik kann die Überwachung hier deactiveiert werden.<br>
            Bei deactiveierter Überwachung wird Reed2 ignoriert.
          </div>
        </div>
    
        <div class="section">
         <h2>Servo Steuerung</h2>
         <div class="servo-control">
           
           <!-- Servo 1: Taster (2 Positionen) -->
           <div class="servo-row">
             <div class="servo-label">S50 Taster</div>
             <div class="servo-pos"><span id="pos1">90</span>°</div>
             <div class="servo-buttons">
               <button onclick="moveServo(1, -1)">-</button>
               <button onclick="moveServo(1, 1)">+</button>
             </div>
           </div>
           <div class="servo-row" style="background-color: #1a1a1a; margin-top: -10px; padding-top: 5px;">
             <div style="flex: 2;">
               <button onclick="gotoPosition(1, 1)" style="margin: 5px; width: 45%;">Taster nicht gedrückt</button>
               <button onclick="gotoPosition(1, 2)" style="margin: 5px; width: 45%;">Taster gedrückt</button>
             </div>
             <div style="flex: 1; text-align: right;">
               <button onclick="savePosition(1, 1)" class="small">💾 Pos1</button>
               <button onclick="savePosition(1, 2)" class="small">💾 Pos2</button>
             </div>
           </div>
           
           <!-- Servo 2: Cover (3 Positionen) -->
           <div class="servo-row" style="margin-top: 20px;">
             <div class="servo-label">Cover</div>
             <div class="servo-pos"><span id="pos2">90</span>°</div>
             <div class="servo-buttons">
               <button onclick="moveServo(2, -1)">-</button>
               <button onclick="moveServo(2, 1)">+</button>
             </div>
           </div>
           <div class="servo-row" style="background-color: #1a1a1a; margin-top: -10px; padding-top: 5px;">
             <div style="flex: 3;">
               <button onclick="gotoPosition(2, 1)" style="margin: 5px; width: 30%;">Cover closed</button>
               <button onclick="gotoPosition(2, 2)" style="margin: 5px; width: 30%;">Cover open</button>
               <button onclick="gotoPosition(2, 3)" style="margin: 5px; width: 30%;">Cover half-open</button>
             </div>
             <div style="flex: 1; text-align: right;">
               <button onclick="savePosition(2, 1)" class="small">💾 Pos1</button>
               <button onclick="savePosition(2, 2)" class="small">💾 Pos2</button>
               <button onclick="savePosition(2, 3)" class="small">💾 Pos3</button>
             </div>
           </div>
           
           <!-- Servo 3: Lock (2 Positionen) -->
           <div class="servo-row" style="margin-top: 20px;">
             <div class="servo-label">Lock</div>
             <div class="servo-pos"><span id="pos3">90</span>°</div>
             <div class="servo-buttons">
               <button onclick="moveServo(3, -1)">-</button>
               <button onclick="moveServo(3, 1)">+</button>
             </div>
           </div>
           <div class="servo-row" style="background-color: #1a1a1a; margin-top: -10px; padding-top: 5px;">
             <div style="flex: 2;">
               <button onclick="gotoPosition(3, 1)" style="margin: 5px; width: 45%;">Sicherung verriegelt</button>
               <button onclick="gotoPosition(3, 2)" style="margin: 5px; width: 45%;">Sicherung entriegelt</button>
             </div>
             <div style="flex: 1; text-align: right;">
               <button onclick="savePosition(3, 1)" class="small">💾 Pos1</button>
               <button onclick="savePosition(3, 2)" class="small">💾 Pos2</button>
             </div>
           </div>
           
         </div>
        </div>
        
        <!-- Sensor Status -->
        <div class="section">
         <h2>📡 Sensor Status</h2>
         <div style="text-align: left; font-size: 14px;">
           <div id="sensor_htu" class="sensor-missing">✗ HTU21D/Si7021: Prüfe...</div>
           <div id="sensor_bmp" class="sensor-missing">✗ BMP-Sensor: Prüfe...</div>
           <div id="sensor_rtc" class="sensor-missing">✗ DS3231 RTC: Prüfe...</div>
           <div id="sensor_gps" class="sensor-missing">✗ GPS: Prüfe...</div>
           <div id="sensor_rain" class="sensor-missing">✗ RG-11 Rain: Prüfe...</div>
         </div>
        </div>
        
        <!-- Erweiterte Settings -->
        <div class="section" style="background:#0a0a0a;">
          <h2>🔧 Erweiterte Settings</h2>
          <button onclick="location.href='/wifi'" style="width:100%; padding:15px; margin:10px 0; background:#1a5080;">📡 WiFi Konfiguration</button>
          <button onclick="location.href='/seestar'" style="width:100%; padding:15px; margin:10px 0; background:#1a5080;">🔭 Seestar S50</button>
          <button onclick="location.href='/rain'" style="width:100%; padding:15px; margin:10px 0; background:#1a5080;">🌧️ Rain-Settings</button>
          <button onclick="location.href='/heater'" style="width:100%; padding:15px; margin:10px 0; background:#1a5080;">♨️ Heater & Dew Point</button>
          <button onclick="location.href='/system'" style="width:100%; padding:15px; margin:10px 0; background:#0066CC;">⚡ System & Version</button>
          <button onclick="location.href='/update'" style="width:100%; padding:15px; margin:10px 0; background:#00AA00;">🔄 Firmware Update</button>
        </div>
        
      </div>
    </div>
    
  </div>
  
  <div class="version">v<span id="version_display">0.1</span></div>
  <div class="copyright">© DM2NT</div>

  <script>
    setInterval(updateStatus, 500);  // Alle 500ms für schnelle LEDs
    
    function updateStatus() {
      fetch('/status')
        .then(response => response.json())
        .then(data => {
          // Reed Contacts
          document.getElementById('led1').className = data.reed1 ? 'led led-on' : 'led led-off';
          
          // Reed2 only if activeiert, sonst ausgrauen
          if (data.reed2_enabled) {
            document.getElementById('led2').className = data.reed2 ? 'led led-on' : 'led led-off';
            document.getElementById('led2').style.opacity = '1.0';
          } else {
            document.getElementById('led2').className = 'led led-off';
            document.getElementById('led2').style.opacity = '0.3';
          }
          
          document.getElementById('led3').className = data.reed3 ? 'led led-on' : 'led led-off';
          
          // Reed2-Überwachung Toggle
          const reed2Toggle = document.getElementById('reed2_toggle');
          const reed2Status = document.getElementById('reed2_status');
          if (data.reed2_enabled) {
            reed2Toggle.textContent = '✅ Riegelüberwachung AKTIVIERT';
            reed2Toggle.style.background = '#00AA00';
            reed2Toggle.style.borderColor = '#00FF00';
            reed2Status.textContent = 'Enabled';
            reed2Status.style.color = '#00FF00';
          } else {
            reed2Toggle.textContent = '❌ Riegelüberwachung DEAKTIVIERT';
            reed2Toggle.style.background = '#AA0000';
            reed2Toggle.style.borderColor = '#FF0000';
            reed2Status.textContent = 'Disabled';
            reed2Status.style.color = '#FF0000';
          }
          
          // Servo Positionen
          document.getElementById('pos1').textContent = data.servo1;
          document.getElementById('pos2').textContent = data.servo2;
          document.getElementById('pos3').textContent = data.servo3;
          
          // Weather Data
          document.getElementById('temp_innen').textContent = data.temp_innen.toFixed(1);
          document.getElementById('hum_innen').textContent = data.hum_innen.toFixed(0);
          document.getElementById('temp_aussen').textContent = data.temp_aussen.toFixed(1);
          document.getElementById('druck_aussen').textContent = data.druck_aussen.toFixed(0);
          
          // RTC Time
          document.getElementById('rtc_time_local').textContent = data.rtc_time_local;
          document.getElementById('rtc_time_utc').textContent = data.rtc_time_utc;
          document.getElementById('rtc_date').textContent = data.rtc_date;
          document.getElementById('timezone_offset').textContent = (data.timezone_offset >= 0 ? '+' : '') + data.timezone_offset;
          document.getElementById('dst_label').textContent = data.is_dst ? '(MESZ)' : '(MEZ)';
          
          // RTC-Section färben je nach RTC-Status
          const rtcSection = document.getElementById('rtc_section');
          if (data.rtc_present) {
            rtcSection.style.borderColor = '#00FF00';  // Grün
          } else {
            rtcSection.style.borderColor = '#FF8800';  // Orange
          }
          
          // GPS Daten
          document.getElementById('gps_time').textContent = data.gps_time;
          document.getElementById('gps_date').textContent = data.gps_date;
          document.getElementById('gps_sats').textContent = data.gps_sats;
          document.getElementById('gps_fix').textContent = data.gps_fix ? 'Ja' : 'Nein';
          
          const satIcon = document.getElementById('gps_sat_icon');
          if (data.gps_fix) {
            satIcon.style.filter = 'none';
            satIcon.style.textShadow = '0 0 10px #00FF00';
          } else {
            satIcon.style.filter = 'grayscale(100%)';
            satIcon.style.textShadow = 'none';
          }
          
          // Rain Sensor (nur if present)
          if (data.rain_sensor_present) {
            document.getElementById('rain_section').style.display = 'block';
            const rainStatus = document.getElementById('rain_status');
            
            if (data.rain_detected) {
              rainStatus.textContent = 'REGEN!';
              rainStatus.style.color = '#FF0000';
            } else {
              rainStatus.textContent = 'DRY';
              rainStatus.style.color = '#00FF00';
            }
          }
          
          // Heater
          const heaterStatusSettings = document.getElementById('heater_status_settings');
          const heaterIconSettings = document.getElementById('heater_icon_settings');
          const dewPointSettings = document.getElementById('dew_point_settings');
          
          dewPointSettings.textContent = data.dew_point.toFixed(1);
          
          if (data.heater_on) {
            heaterStatusSettings.textContent = 'AN';
            heaterStatusSettings.style.color = '#FF6600';
            heaterIconSettings.style.filter = 'none';
            heaterIconSettings.style.textShadow = '0 0 10px #FF6600';
          } else {
            heaterStatusSettings.textContent = 'AUS';
            heaterStatusSettings.style.color = '#00FF00';
            heaterIconSettings.style.filter = 'grayscale(100%)';
            heaterIconSettings.style.textShadow = 'none';
          }
          
          // Sensor Status aktualisieren
          // HTU21D - prüfe ob Temperature sinnvoll ist
          if (data.temp_innen > -50 && data.temp_innen < 100) {
            document.getElementById('sensor_htu').className = 'sensor-found';
            document.getElementById('sensor_htu').textContent = '✓ HTU21D/Si7021: OK';
          } else {
            document.getElementById('sensor_htu').className = 'sensor-missing';
            document.getElementById('sensor_htu').textContent = '✗ HTU21D/Si7021: Nicht gefunden';
          }
          
          // BMP-Sensor - zeige tatsächlichen Typ
          if (data.bmp_sensor_present && data.druck_aussen > 800 && data.druck_aussen < 1200) {
            document.getElementById('sensor_bmp').className = 'sensor-found';
            document.getElementById('sensor_bmp').textContent = '✓ ' + data.bmp_sensor_type + ': OK';
          } else {
            document.getElementById('sensor_bmp').className = 'sensor-missing';
            document.getElementById('sensor_bmp').textContent = '✗ BMP-Sensor: Nicht gefunden';
          }
          
          // RTC - prüfe rtc_present Flag
          if (data.rtc_present) {
            document.getElementById('sensor_rtc').className = 'sensor-found';
            document.getElementById('sensor_rtc').textContent = '✓ DS3231 RTC: OK';
          } else {
            document.getElementById('sensor_rtc').className = 'sensor-missing';
            document.getElementById('sensor_rtc').textContent = '✗ DS3231 RTC: Nicht gefunden';
          }
          
          // GPS - unterscheide zwischen Fix, Suche und nicht vorhanden
          if (data.gps_fix && data.gps_sats > 0) {
            // GPS hat Fix
            document.getElementById('sensor_gps').className = 'sensor-found';
            document.getElementById('sensor_gps').textContent = '✓ GPS: OK (' + data.gps_sats + ' Sats)';
          } else if (data.gps_sats > 0) {
            // GPS empfängt Satellites, aber kein Fix
            document.getElementById('sensor_gps').className = 'sensor-missing';
            document.getElementById('sensor_gps').textContent = '✗ GPS: Kein Fix (' + data.gps_sats + ' Sats)';
          } else {
            // 0 Satellites - GPS sucht noch oder nicht angeschlossen
            // Wir können nicht unterscheiden, also zeigen wir "Sucht Satellites"
            document.getElementById('sensor_gps').className = 'sensor-missing';
            document.getElementById('sensor_gps').textContent = '⚠ GPS: Sucht Satellites... (0 Sats)';
          }
          
          // Rain Sensor
          if (data.rain_sensor_present) {
            document.getElementById('sensor_rain').className = 'sensor-found';
            document.getElementById('sensor_rain').textContent = '✓ RG-11 Rain: OK';
          } else {
            document.getElementById('sensor_rain').className = 'sensor-missing';
            document.getElementById('sensor_rain').textContent = '✗ RG-11 Rain: Nicht gefunden';
          }
          
          
          // Fehlermeldung anzeigen (falls vorhanden)
          const errorBox = document.getElementById('error_message');
          const errorText = document.getElementById('error_text');
          if (data.last_error && data.last_error !== '') {
            errorText.textContent = data.last_error;
            errorBox.style.display = 'block';
          } else {
            errorBox.style.display = 'none';
          }
          
          // Rain-Schließung Info (falls vorhanden)
          const rainCloseBox = document.getElementById('rain_close_info');
          const rainCloseTime = document.getElementById('rain_close_time');
          if (data.last_rain_close && data.last_rain_close !== '') {
            rainCloseTime.textContent = data.last_rain_close;
            rainCloseBox.style.display = 'block';
          } else {
            rainCloseBox.style.display = 'none';
          }
          
          // Version anzeigen
          document.getElementById('version_display').textContent = data.firmware_version;
        });
    }
    
    function toggleReed2() {
      fetch('/reed2/toggle')
        .then(response => response.text())
        .then(data => {
          console.log('Reed2-Überwachung:', data);
          setTimeout(updateStatus, 500);
        });
    }
    
    function moveServo(servo, delta) {
      fetch('/servo?id=' + servo + '&delta=' + delta)
        .then(response => response.text())
        .then(data => console.log(data));
    }
    
    function gotoPosition(servo, pos) {
      fetch('/goto?servo=' + servo + '&pos=' + pos)
        .then(response => response.text())
        .then(data => console.log(data));
    }
    
    function savePosition(servo, pos) {
      if(confirm('Aktuelle Position als Preset ' + pos + ' speichern?')) {
        fetch('/save?servo=' + servo + '&pos=' + pos)
          .then(response => {
            if (response.ok) {
              console.log('Position saved');
            }
          })
          .catch(e => console.error('Error:', e));
      }
    }
    
    function adjustTimezone(delta) {
      fetch('/timezone?delta=' + delta)
        .then(response => response.text())
        .then(data => console.log(data));
    }
  </script>
</body>
</html>
)rawliteral";

// ============= SYSTEM-SEITE =============
const char system_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>System - S50</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      padding: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 30px; text-align: center; }
    .container { max-width: 600px; margin: 0 auto; }
    .section {
      background-color: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .section h2 { color: #00AAFF; margin-bottom: 15px; }
    button {
      background: #333;
      color: #FFF;
      border: 2px solid #555;
      padding: 15px 30px;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin: 10px 0;
    }
    button:hover { background: #555; }
    .home-btn {
      position: fixed;
      top: 20px;
      left: 20px;
      padding: 10px 20px;
      width: auto;
    }
    .version {
      font-size: 24px;
      color: #00FF00;
      text-align: center;
      margin: 20px 0;
    }
    .info {
      color: #AAA;
      font-size: 12px;
      margin-top: 10px;
    }
  </style>
</head>
<body>
  <button class="home-btn" onclick="location.href='/settings'">⬅️ Back</button>
  
  <div class="container">
    <h1>⚡ System</h1>
    
    <div class="section">
      <h2>Firmware</h2>
      <div class="version" id="fw_version">v0.1</div>
      <div class="info" style="text-align:center;">S50 Observatory Control</div>
    </div>
    
    <div class="section" style="background:#1a0a00; border-color:#FF6600;">
      <h2 style="color:#FF6600;">⚠️ Neustart</h2>
      <div style="color:#FFA500; padding:15px; background:#0a0a0a; border-radius:5px; margin:10px 0;">
        <strong>ESP32 Neustart:</strong><br><br>
        Zum Neustarten bitte kurz die <strong>Betriebsspannung trennen</strong> (USB-Kabel ziehen oder Netzteil ausschalten).<br><br>
        Ein Software-Neustart ist aus technischen Gründen nicht möglich.
      </div>
    </div>
    
    <div class="section">
      <h2>Aktionen</h2>
      
      <button onclick="loadDefaults()" style="background:#8B0000; margin-top:10px;">🔄 Defaultwerte laden</button>
      <div class="info">Alle Servo-Winkel → 90° (WiFi bleibt erhalten)</div>
    </div>
  </div>

  <script>
    // Lade Version beim Start
    fetch('/status')
      .then(r => r.json())
      .then(data => {
        document.getElementById('fw_version').textContent = 'v' + data.firmware_version;
      });
    
    function loadDefaults() {
      if (confirm('WARNUNG!\n\nAlle Servo-Winkel werden auf 90° zurückgesetzt!\n\nWiFi und Timezone bleiben erhalten.\n\nFortfahren?')) {
        fetch('/system/defaults', {
          method: 'GET',
          headers: {'Accept': 'text/plain'}
        })
          .then(r => {
            if (!r.ok) throw new Error('HTTP ' + r.status);
            return r.text();
          })
          .then(t => {
            alert('Defaults loaded!\n\nAlle Servo-Winkel auf 90°');
          })
          .catch(e => {
            console.error('Defaults Error:', e);
            alert('Defaults loaded!\n(Response-Fehler ignorieren)');
          });
      }
    }
  </script>
</body>
</html>
)rawliteral";

// ============= WIFI-SEITE =============
const char wifi_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WiFi - S50</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      padding: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 30px; text-align: center; }
    .container { max-width: 600px; margin: 0 auto; }
    .section {
      background-color: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .section h2 { color: #00AAFF; margin-bottom: 15px; }
    button {
      background: #333;
      color: #FFF;
      border: 2px solid #555;
      padding: 15px 30px;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin: 10px 0;
    }
    button:hover { background: #555; }
    .home-btn {
      position: fixed;
      top: 20px;
      left: 20px;
      padding: 10px 20px;
      width: auto;
    }
    .info { color: #AAA; font-size: 14px; margin-top: 10px; line-height: 1.6; }
    .warning { 
      background: #3a1a00; 
      border: 1px solid #FF6600; 
      color: #FFA500;
      padding: 15px; 
      border-radius: 5px;
      margin: 15px 0;
    }
  </style>
</head>
<body>
  <button class="home-btn" onclick="location.href='/settings'">⬅️ Back</button>
  
  <div class="container">
    <h1>📡 WiFi Konfiguration</h1>
    
    <div class="section">
      <h2>Aktuelle Verbindung</h2>
      <div style="margin-bottom: 15px;">
        <strong>SSID:</strong> <span id="current_ssid">--</span><br>
        <strong>IP:</strong> <span id="current_ip">--</span><br>
        <strong>Signal:</strong> <span id="current_rssi">--</span> dBm
      </div>
    </div>
    
    <div class="section">
      <h2>WLAN neu konfigurieren</h2>
      
      <div class="info">
        <strong>So funktioniert's:</strong><br>
        1. Trenne kurz die Betriebsspannung (USB/Netzteil)<br>
        2. Halte die BOOT-Taste gedrückt beim Einschalten<br>
        3. ESP32 startet im Config-Modus<br>
        4. Verbinde dich mit "S50-Setup"<br>
        5. Captive Portal öffnet automatisch<br>
        6. Wähle dein WLAN und gib Passwort ein
      </div>
      
      <div class="warning">
        ⚠️ <strong>Alternative:</strong><br>
        WiFiManager-Konfiguration löschen über Arduino IDE:<br>
        Tools → Erase Flash → "All Flash Contents" → Upload
      </div>
    </div>
  </div>

  <script>
    // Lade aktuelle Verbindung
    fetch('/status')
      .then(r => r.json())
      .then(d => {
        document.getElementById('current_ssid').textContent = d.wifi_ssid || '--';
        document.getElementById('current_ip').textContent = d.wifi_ip || '--';
        document.getElementById('current_rssi').textContent = d.wifi_rssi || '--';
      });
  </script>
</body>
</html>
)rawliteral";

// ============= SEESTAR-SEITE =============
const char seestar_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Seestar S50 - S50</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      padding: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 30px; text-align: center; }
    .container { max-width: 600px; margin: 0 auto; }
    .section {
      background-color: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .section h2 { color: #00AAFF; margin-bottom: 15px; }
    button {
      background: #333;
      color: #FFF;
      border: 2px solid #555;
      padding: 15px 30px;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin: 10px 0;
    }
    button:hover { background: #555; }
    .home-btn {
      position: fixed;
      top: 20px;
      left: 20px;
      padding: 10px 20px;
      width: auto;
    }
    input {
      width: 100%;
      padding: 10px;
      margin: 5px 0;
      background: #222;
      color: #FFF;
      border: 1px solid #555;
      border-radius: 5px;
    }
    .status-online { color: #00FF00; font-weight: bold; }
    .status-offline { color: #FF0000; font-weight: bold; }
  </style>
</head>
<body>
  <button class="home-btn" onclick="location.href='/settings'">⬅️ Back</button>
  
  <div class="container">
    <h1>🔭 Seestar S50</h1>
    
    <div class="section">
      <h2>Status</h2>
      <div style="margin-bottom: 15px; text-align: center;">
        <div style="font-size: 24px; margin: 20px 0;">
          <span id="seestar_status" class="status-offline">OFFLINE</span>
        </div>
        <div>
          <strong>IP Address:</strong> <span id="seestar_ip_display">--</span>
        </div>
        <div style="margin-top: 10px; color: #AAA; font-size: 12px;">
          Ping check every 5 seconds
        </div>
      </div>
    </div>
    
    <div class="section">
      <h2>Change IP Address</h2>
      <label style="color:#AAA;">Seestar S50 IP Address:</label>
      <input type="text" id="new_seestar_ip" placeholder="192.168.1.100">
      
      <button onclick="saveSeestarIP()" style="background:#0066CC; margin-top:15px;">💾 Save</button>
      
      <div style="margin-top: 10px; color: #AAA; font-size: 12px;">
        No restart required - active immediately
      </div>
    </div>
  </div>

  <script>
    setInterval(updateStatus, 2000);
    updateStatus();
    
    function updateStatus() {
      fetch('/status?t=' + Date.now())
        .then(r => r.json())
        .then(data => {
          const statusElem = document.getElementById('seestar_status');
          document.getElementById('seestar_ip_display').textContent = data.seestar_ip;
          
          if (data.seestar_online) {
            statusElem.textContent = 'ONLINE';
            statusElem.className = 'status-online';
          } else {
            statusElem.textContent = 'OFFLINE';
            statusElem.className = 'status-offline';
          }
        });
    }
    
    function saveSeestarIP() {
      const ip = document.getElementById('new_seestar_ip').value;
      
      if (!ip) {
        alert('Please IP-Adresse eingeben!');
        return;
      }
      
      const ipPattern = /^(\d{1,3}\.){3}\d{1,3}$/;
      if (!ipPattern.test(ip)) {
        alert('Invalide IP-Adresse!');
        return;
      }
      
      fetch('/seestar/save?ip=' + encodeURIComponent(ip))
        .then(response => response.text())
        .then(text => {
          console.log('Server Response:', text);
          alert('✅ Seestar IP saved!\n\n' + ip);
          document.getElementById('new_seestar_ip').value = '';
          setTimeout(updateStatus, 500);
        })
        .catch(e => {
          console.error('Error:', e);
          alert('❌ Fehler beim Save!\n\n' + e.message);
        });
    }
  </script>
</body>
</html>
)rawliteral";

// ============= REGEN-SEITE =============
const char rain_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Rain - S50</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      padding: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 30px; text-align: center; }
    .container { max-width: 600px; margin: 0 auto; }
    .section {
      background-color: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .section h2 { color: #00AAFF; margin-bottom: 15px; }
    button {
      background: #333;
      color: #FFF;
      border: 2px solid #555;
      padding: 15px 30px;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin: 10px 0;
    }
    button:hover { background: #555; }
    .home-btn {
      position: fixed;
      top: 20px;
      left: 20px;
      padding: 10px 20px;
      width: auto;
    }
    input {
      width: 100%;
      padding: 10px;
      margin: 5px 0;
      background: #222;
      color: #FFF;
      border: 1px solid #555;
      border-radius: 5px;
    }
    .value { 
      font-size: 24px; 
      color: #00AAFF; 
      font-weight: bold; 
      text-align: center;
      margin: 10px 0;
    }
    .info { color: #AAA; font-size: 12px; margin-top: 10px; }
  </style>
</head>
<body>
  <button class="home-btn" onclick="location.href='/settings'">⬅️ Back</button>
  
  <div class="container">
    <h1>🌧️ Rain-Settings</h1>
    
    <div class="section">
      <h2>Aktueller Status</h2>
      <div style="text-align: center;">
        <div id="rain_present" style="color: #888; margin: 10px 0;">Prüfe Sensor...</div>
        <div id="current_status" class="value">--</div>
        <div style="margin: 20px 0;">
          <div style="color: #AAA;">Intensität:</div>
          <div class="value" id="current_rate">0.0 mm/h</div>
        </div>
        <div style="color: #AAA;">
          Akkumuliert: <span id="current_acc" style="color: #FFF;">0.00 mm</span>
        </div>
      </div>
    </div>
    
    <div class="section">
      <h2>Schwellwert</h2>
      <label style="color:#AAA;">Close bei Rain ab (mm/h):</label>
      <input type="number" id="threshold_input" step="0.1" min="0.1" max="10.0" placeholder="0.5">
      
      <button onclick="saveThreshold()" style="background:#0066CC; margin-top:15px;">💾 Save</button>
      
      <div class="info">
        Recommended: 0.5 mm/h (leichter Rain)<br>
        Bei Überschreitung: Automatic Emergency close
      </div>
    </div>
  </div>

  <script>
    setInterval(updateStatus, 2000);
    updateStatus();
    
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          const presentDiv = document.getElementById('rain_present');
          const statusDiv = document.getElementById('current_status');
          
          if (d.rain_sensor_present) {
            presentDiv.textContent = '✓ RG-11 Sensor verbunden';
            presentDiv.style.color = '#00FF00';
            
            if (d.rain_detected) {
              statusDiv.textContent = 'REGEN!';
              statusDiv.style.color = '#FF0000';
            } else {
              statusDiv.textContent = 'DRY';
              statusDiv.style.color = '#00FF00';
            }
            
            document.getElementById('current_rate').textContent = d.rain_rate.toFixed(1) + ' mm/h';
            document.getElementById('current_acc').textContent = d.rain_acc.toFixed(2) + ' mm';
          } else {
            presentDiv.textContent = '✗ Kein RG-11 Sensor erkannt';
            presentDiv.style.color = '#FF6600';
            statusDiv.textContent = 'N/A';
            statusDiv.style.color = '#888';
          }
          
          // Aktuellen Schwellwert anzeigen
          document.getElementById('threshold_input').placeholder = d.rain_threshold.toFixed(1);
        });
    }
    
    function saveThreshold() {
      const val = parseFloat(document.getElementById('threshold_input').value);
      
      if (isNaN(val) || val < 0.1 || val > 10.0) {
        alert('Invalider Wert! Please 0.1 - 10.0 mm/h eingeben.');
        return;
      }
      
      fetch('/rain/threshold?value=' + val)
        .then(response => {
          if (response.ok) {
            document.getElementById('threshold_input').value = '';
            updateStatus();
          }
        })
        .catch(e => console.error('Error:', e));
    }
  </script>
</body>
</html>
)rawliteral";

// ============= HEIZUNGS-SEITE =============
const char heater_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Heater - S50</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      padding: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 30px; text-align: center; }
    .container { max-width: 600px; margin: 0 auto; }
    .section {
      background-color: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .section h2 { color: #00AAFF; margin-bottom: 15px; }
    button {
      background: #333;
      color: #FFF;
      border: 2px solid #555;
      padding: 15px 30px;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin: 10px 0;
    }
    button:hover { background: #555; }
    .home-btn {
      position: fixed;
      top: 20px;
      left: 20px;
      padding: 10px 20px;
      width: auto;
    }
    input {
      width: 100%;
      padding: 10px;
      margin: 5px 0;
      background: #222;
      color: #FFF;
      border: 1px solid #555;
      border-radius: 5px;
    }
    .value { 
      font-size: 24px; 
      color: #00AAFF; 
      font-weight: bold; 
      text-align: center;
      margin: 10px 0;
    }
    .info { color: #AAA; font-size: 12px; margin-top: 10px; }
    .indicator {
      display: inline-block;
      width: 15px;
      height: 15px;
      border-radius: 50%;
      margin-right: 8px;
    }
  </style>
</head>
<body>
  <button class="home-btn" onclick="location.href='/settings'">⬅️ Back</button>
  
  <div class="container">
    <h1>♨️ Heater & Dew Point</h1>
    
    <div class="section">
      <h2>Aktueller Status</h2>
      <div style="text-align: center;">
        <div style="margin: 20px 0;">
          <span class="indicator" id="heater_led" style="background: #888;"></span>
          <span id="heater_status" style="font-size: 20px; font-weight: bold;">--</span>
        </div>
        
        <div style="margin: 20px 0; padding: 15px; background: #0a0a0a; border-radius: 10px;">
          <div style="color: #AAA; margin-bottom: 5px;">Outside temperatur</div>
          <div class="value" id="temp_outside">--°C</div>
        </div>
        
        <div style="margin: 20px 0; padding: 15px; background: #0a0a0a; border-radius: 10px;">
          <div style="color: #AAA; margin-bottom: 5px;">Dew Point</div>
          <div class="value" id="dew_point">--°C</div>
          <div style="color: #888; font-size: 12px; margin-top: 5px;">
            Berechnet aus Inside temp & Luftfeuchte
          </div>
        </div>
        
        <div id="sensor_warning" style="display:none; color:#FF6600; margin-top:15px;">
          ⚠️ BMP180 Außensensor fehlt - Heater deactiveiert
        </div>
      </div>
    </div>
    
    <div class="section">
      <h2>Heater Steuerung</h2>
      <div style="margin-bottom: 15px; color: #AAA; font-size: 14px;">
        <strong>Status:</strong> <span id="current_mode" style="color: #00AAFF;">Enabled</span>
      </div>
      
      <button id="heater_toggle" onclick="toggleHeaterMode()" style="background:#0066CC; border: 2px solid #00FF00;">
        ✅ Heater AKTIVIERT
      </button>
      
      <div class="info">
        <strong>Automatic Dew Point Heater:</strong><br>
        • Heater switches automatically when Inside temp ≤ Dew Point<br>
        • Only when closeder Observatory active<br>
        • With this button you can the automatic komplett deactiveieren
      </div>
    </div>
    
    <div class="section">
      <h2>Dew Point-Hysteresis</h2>
      <div style="margin-bottom: 15px; color: #AAA; font-size: 14px;">
        <strong>Aktuell:</strong> <span id="current_hysteresis" style="color: #00AAFF;">2.0</span> °C
      </div>
      
      <label style="color:#AAA;">Hysteresis (°C):</label>
      <input type="number" id="hysteresis_input" step="0.5" min="0.5" max="10.0" placeholder="2.0">
      
      <button onclick="saveHysteresis()" style="background:#0066CC; margin-top:15px;">💾 Save</button>
      
      <div class="info">
        <strong>How does it work?</strong><br>
        • Heater ON: Inside temp ≤ Dew Point<br>
        • Heater OFF: Inside temp > Dew Point + Hysteresis<br><br>
        <strong>Recommended:</strong> 2.0°C (prevents frequent switching)<br>
        <strong>Default:</strong> ±2°C (Industrie-Default for telescopes)
      </div>
    </div>
  </div>

  <script>
    setInterval(updateStatus, 2000);
    updateStatus();
    
    function updateStatus() {
      fetch('/status?t=' + Date.now())
        .then(r => r.json())
        .then(d => {
          const heaterLed = document.getElementById('heater_led');
          const heaterStatus = document.getElementById('heater_status');
          const sensorWarning = document.getElementById('sensor_warning');
          
          // Heater Status LED
          if (d.heater_on) {
            heaterLed.style.background = '#FF6600';
            heaterStatus.textContent = 'HEIZUNG ON';
            heaterStatus.style.color = '#FF6600';
          } else {
            heaterLed.style.background = '#888';
            heaterStatus.textContent = 'HEIZUNG OFF';
            heaterStatus.style.color = '#888';
          }
          
          // Temperatures
          document.getElementById('temp_outside').textContent = d.temp_aussen.toFixed(1) + '°C';
          document.getElementById('dew_point').textContent = d.dew_point.toFixed(1) + '°C';
          
          // Sensor-Warning
          if (!d.bmp_sensor_present) {
            sensorWarning.style.display = 'block';
            sensorWarning.textContent = 'ℹ️ BMP-Sensor fehlt - Outside temperatur nicht verfügbar';
          } else {
            sensorWarning.style.display = 'none';
          }
          
          // Heater Toggle Button
          const toggleBtn = document.getElementById('heater_toggle');
          const modeText = document.getElementById('current_mode');
          
          if (d.heater_mode === 0) {
            // Enabled
            toggleBtn.textContent = '✅ Heater AKTIVIERT';
            toggleBtn.style.background = '#00AA00';
            toggleBtn.style.borderColor = '#00FF00';
            modeText.textContent = 'Enabled';
            modeText.style.color = '#00FF00';
          } else {
            // Disabled
            toggleBtn.textContent = '❌ Heater DEAKTIVIERT';
            toggleBtn.style.background = '#AA0000';
            toggleBtn.style.borderColor = '#FF0000';
            modeText.textContent = 'Disabled';
            modeText.style.color = '#FF0000';
          }
          
          // Hysteresis
          document.getElementById('current_hysteresis').textContent = d.heater_hysteresis.toFixed(1);
        });
    }
    
    function toggleHeaterMode() {
      // Toggle zwischen 0 (Enabled) und 1 (Disabled)
      fetch('/status?t=' + Date.now())
        .then(r => r.json())
        .then(d => {
          const newMode = (d.heater_mode === 0) ? 1 : 0;
          fetch('/heater/mode?mode=' + newMode);
          setTimeout(function() { updateStatus(); }, 500);
        });
    }
    
    function saveHysteresis() {
      const val = parseFloat(document.getElementById('hysteresis_input').value);
      
      if (isNaN(val) || val < 0.5 || val > 10.0) {
        alert('Invalider Wert! Please 0.5 - 10.0 °C eingeben.');
        return;
      }
      
      fetch('/heater/hysteresis?value=' + val);
      document.getElementById('hysteresis_input').value = '';
      setTimeout(function() { updateStatus(); }, 500);
    }
  </script>
</body>
</html>
)rawliteral";

// ============= ZEITPLANER-SEITE =============
const char scheduler_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Scheduler - S50</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      padding: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 30px; text-align: center; }
    .container { max-width: 600px; margin: 0 auto; }
    .section {
      background-color: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .section h2 { color: #00AAFF; margin-bottom: 15px; }
    button {
      background: #333;
      color: #FFF;
      border: 2px solid #555;
      padding: 15px 30px;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin: 10px 0;
    }
    button:hover { background: #555; }
    .home-btn {
      position: fixed;
      top: 20px;
      left: 20px;
      padding: 10px 20px;
      width: auto;
    }
    input {
      width: 100%;
      padding: 10px;
      margin: 5px 0;
      background: #222;
      color: #FFF;
      border: 1px solid #555;
      border-radius: 5px;
    }
    .time-input {
      display: flex;
      gap: 10px;
      align-items: center;
    }
    .time-input input {
      width: 80px;
      text-align: center;
      font-size: 18px;
    }
    .info { color: #AAA; font-size: 12px; margin-top: 10px; }
    .toggle {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding: 15px;
      background: #0a0a0a;
      border-radius: 10px;
      margin: 15px 0;
    }
    .switch {
      position: relative;
      width: 60px;
      height: 30px;
    }
    .switch input {
      opacity: 0;
      width: 0;
      height: 0;
    }
    .slider {
      position: absolute;
      cursor: pointer;
      top: 0;
      left: 0;
      right: 0;
      bottom: 0;
      background-color: #555;
      transition: .4s;
      border-radius: 30px;
    }
    .slider:before {
      position: absolute;
      content: "";
      height: 22px;
      width: 22px;
      left: 4px;
      bottom: 4px;
      background-color: white;
      transition: .4s;
      border-radius: 50%;
    }
    input:checked + .slider {
      background-color: #00AA00;
    }
    input:checked + .slider:before {
      transform: translateX(30px);
    }
  </style>
</head>
<body>
  <button class="home-btn" onclick="location.href='/'">⬅️ Back</button>
  
  <div class="container">
    <h1>⏰ Scheduler</h1>
    
    <div class="section">
      <h2>Status</h2>
      <div class="toggle">
        <span style="font-size: 16px;">Scheduler activeiert</span>
        <label class="switch">
          <input type="checkbox" id="scheduler_toggle">
          <span class="slider"></span>
        </label>
      </div>
      <div id="next_action" style="text-align: center; color: #00AAFF; margin-top: 15px; font-size: 14px;">
        Loading...
      </div>
    </div>
    
    <div class="section" style="background: #0a1a0a; border-color: #00AA00;">
      <h2 style="color: #00FF00;">✅ Gespeicherte Timeen</h2>
      <div id="saved_times" style="text-align: center; font-size: 16px; color: #AAA; padding: 10px;">
        Loading...
      </div>
    </div>
    
    <div class="section">
      <h2>⬆️ Open</h2>
      <label style="color:#AAA;">Time:</label>
      <div class="time-input">
        <input type="number" id="open_hour" min="0" max="23" placeholder="20">
        <span style="font-size: 20px;">:</span>
        <input type="number" id="open_min" min="0" max="59" placeholder="00">
      </div>
      <div class="info">
        ⚠️ Öffnung wird bei Rain automatisch abgebrochen
      </div>
    </div>
    
    <div class="section">
      <h2>⬇️ Close</h2>
      <label style="color:#AAA;">Time:</label>
      <div class="time-input">
        <input type="number" id="close_hour" min="0" max="23" placeholder="03">
        <span style="font-size: 20px;">:</span>
        <input type="number" id="close_min" min="0" max="59" placeholder="00">
      </div>
    </div>
    
    <button onclick="saveSchedule()" style="background:#0066CC; margin-top:15px;">💾 Save</button>
    
    <div class="info" style="margin-top: 20px; padding: 15px; background: #0a0a0a; border-radius: 10px;">
      <strong>💡 How does it work?</strong><br>
      • Planer nutzt RTC-Time (GPS-synchronisiert)<br>
      • Täglich zur eingestellten Time<br>
      • Open wird bei Rain verhindert<br>
      • Close läuft immer (außer Tube active)<br><br>
      <strong>Beispiel:</strong> Wie Seestar S50 - Session von 20:00 bis 03:00 Uhr
    </div>
  </div>

  <script>
    // Lade EINMAL beim Open der Seite
    loadSchedule();
    
    function loadSchedule() {
      console.log('Lade Status vom Server...');
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          console.log('Empfangene Daten:', d);
          document.getElementById('scheduler_toggle').checked = d.scheduler_enabled;
          document.getElementById('open_hour').value = d.schedule_open_hour;
          document.getElementById('open_min').value = d.schedule_open_min;
          document.getElementById('close_hour').value = d.schedule_close_hour;
          document.getElementById('close_min').value = d.schedule_close_min;
          updateSavedTimes(d);
          updateNextAction(d);
        });
    }
    
    function updateSavedTimes(d) {
      const savedDiv = document.getElementById('saved_times');
      const openTime = String(d.schedule_open_hour).padStart(2, '0') + ':' + 
                       String(d.schedule_open_min).padStart(2, '0');
      const closeTime = String(d.schedule_close_hour).padStart(2, '0') + ':' + 
                        String(d.schedule_close_min).padStart(2, '0');
      
      savedDiv.innerHTML = `
        <strong>Open:</strong> ${openTime} Uhr &nbsp;|&nbsp; 
        <strong>Close:</strong> ${closeTime} Uhr
      `;
      savedDiv.style.color = '#00FF00';
    }
    
    function updateNextAction(d) {
      const nextDiv = document.getElementById('next_action');
      
      if (!d.scheduler_enabled) {
        nextDiv.textContent = '⏸️ Scheduler deactiveiert';
        nextDiv.style.color = '#888';
        return;
      }
      
      const openTime = String(d.schedule_open_hour).padStart(2, '0') + ':' + 
                       String(d.schedule_open_min).padStart(2, '0');
      const closeTime = String(d.schedule_close_hour).padStart(2, '0') + ':' + 
                        String(d.schedule_close_min).padStart(2, '0');
      
      nextDiv.innerHTML = `
        ✅ Scheduler active<br>
        Öffnet täglich um ${openTime} Uhr | Schließt um ${closeTime} Uhr
      `;
      nextDiv.style.color = '#00FF00';
    }
    
    function saveSchedule() {
      const enabled = document.getElementById('scheduler_toggle').checked ? 1 : 0;
      const oh = parseInt(document.getElementById('open_hour').value) || 20;
      const om = parseInt(document.getElementById('open_min').value) || 0;
      const ch = parseInt(document.getElementById('close_hour').value) || 3;
      const cm = parseInt(document.getElementById('close_min').value) || 0;
      
      console.log('Speichere:', {enabled, oh, om, ch, cm});
      
      if (oh < 0 || oh > 23 || om < 0 || om > 59 || ch < 0 || ch > 23 || cm < 0 || cm > 59) {
        alert('Invalide Timeangabe!');
        return;
      }
      
      const url = `/scheduler/save?enabled=${enabled}&oh=${oh}&om=${om}&ch=${ch}&cm=${cm}`;
      console.log('Sende Request:', url);
      
      fetch(url, {
        method: 'GET',
        headers: {'Accept': 'text/plain'}
      })
        .then(r => {
          console.log('Response Status:', r.status);
          if (!r.ok) throw new Error('HTTP ' + r.status);
          return r.text();
        })
        .then(t => {
          console.log('Response Text:', t);
          
          const openTime = String(oh).padStart(2, '0') + ':' + String(om).padStart(2, '0');
          const closeTime = String(ch).padStart(2, '0') + ':' + String(cm).padStart(2, '0');
          
          // UPDATE SOFORT die Anzeigen MIT DEN GESENDETEN WERTEN!
          const newData = {
            scheduler_enabled: Boolean(enabled),
            schedule_open_hour: oh,
            schedule_open_min: om,
            schedule_close_hour: ch,
            schedule_close_min: cm
          };
          
          console.log('Update UI mit:', newData);
          
          // Setze die Felder nochmal (zur Sicherheit)
          document.getElementById('scheduler_toggle').checked = Boolean(enabled);
          document.getElementById('open_hour').value = oh;
          document.getElementById('open_min').value = om;
          document.getElementById('close_hour').value = ch;
          document.getElementById('close_min').value = cm;
          
          updateSavedTimes(newData);
          updateNextAction(newData);
          
          alert('✅ Scheduler saved!\n\n' +
                'Open: ' + openTime + ' Uhr\n' +
                'Close: ' + closeTime + ' Uhr\n' +
                'Status: ' + (enabled ? 'Aktiv ✅' : 'Inactive ⏸️') + '\n\n' +
                'Seite wird neu loaded...');
          
          // Lade nach 1 Sekunde neu vom Server
          setTimeout(() => {
            console.log('Lade neu vom Server...');
            loadSchedule();
          }, 1000);
        })
        .catch(e => {
          console.error('Save Error:', e);
          alert('Fehler beim Save:\n' + e);
        });
    }
  </script>
</body>
</html>
)rawliteral";

// ============= OTA UPDATE-SEITE =============
const char update_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html lang="de">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Firmware Update - S50</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      background-color: #000;
      color: #FFF;
      font-family: Arial, sans-serif;
      padding: 20px;
    }
    h1 { color: #00FF00; margin-bottom: 30px; text-align: center; }
    .container { max-width: 600px; margin: 0 auto; }
    .section {
      background-color: #1a1a1a;
      border: 2px solid #333;
      border-radius: 10px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .section h2 { color: #00AAFF; margin-bottom: 15px; }
    button, input[type="submit"] {
      background: #333;
      color: #FFF;
      border: 2px solid #555;
      padding: 15px 30px;
      border-radius: 5px;
      cursor: pointer;
      font-size: 16px;
      width: 100%;
      margin: 10px 0;
    }
    button:hover, input[type="submit"]:hover { background: #555; }
    .home-btn {
      position: fixed;
      top: 20px;
      left: 20px;
      padding: 10px 20px;
      width: auto;
    }
    input[type="file"] {
      width: 100%;
      padding: 10px;
      margin: 10px 0;
      background: #222;
      color: #FFF;
      border: 2px solid #555;
      border-radius: 5px;
      cursor: pointer;
    }
    .progress-container {
      width: 100%;
      background: #222;
      border-radius: 5px;
      margin: 20px 0;
      display: none;
    }
    .progress-bar {
      width: 0%;
      height: 30px;
      background: linear-gradient(90deg, #00AA00, #00FF00);
      border-radius: 5px;
      text-align: center;
      line-height: 30px;
      transition: width 0.3s;
    }
    .info {
      color: #AAA;
      font-size: 12px;
      margin-top: 10px;
    }
    .warning {
      color: #FF6600;
      font-weight: bold;
      margin: 10px 0;
    }
    .version {
      font-size: 20px;
      color: #00FF00;
      text-align: center;
      margin: 10px 0;
    }
  </style>
</head>
<body>
  <button class="home-btn" onclick="location.href='/settings'">⬅️ Back</button>
  
  <div class="container">
    <h1>🔄 Firmware Update</h1>
    
    <div class="section">
      <h2>Aktuelle Version</h2>
      <div class="version" id="current_version">v0.1</div>
    </div>
    
    <div class="section">
      <h2>Neue Firmware hochladen</h2>
      <div class="warning">⚠️ WARNUNG: Während des Updates nicht unterbrechen!</div>
      
      <form id="upload_form" enctype="multipart/form-data">
        <input type="file" id="file_input" name="update" accept=".bin" required>
        <div class="info">Nur .bin Dateien erlaubt (ESP32 Firmware)</div>
        
        <input type="submit" value="📤 Update starten" style="background:#00AA00; margin-top:20px;">
      </form>
      
      <div class="progress-container" id="progress_container">
        <div class="progress-bar" id="progress_bar">0%</div>
      </div>
      
      <div id="status_message" style="margin-top:20px; text-align:center;"></div>
    </div>
    
    <div class="section">
      <h2>ℹ️ Anleitung</h2>
      <div class="info">
        <strong>1.</strong> .bin Datei aus Arduino IDE erstellen:<br>
        &nbsp;&nbsp;&nbsp;Sketch → Export compiled Binary<br><br>
        <strong>2.</strong> .bin Datei auswählen<br><br>
        <strong>3.</strong> "Update starten" klicken<br><br>
        <strong>4.</strong> Warten bis Update abgeschlossen<br><br>
        <strong>5.</strong> ESP32 startet automatisch neu<br><br>
        ⏱️ Update dauert ca. 30-60 seconds
      </div>
    </div>
  </div>

  <script>
    // Lade aktuelle Version
    fetch('/status')
      .then(r => r.json())
      .then(data => {
        document.getElementById('current_version').textContent = 'v' + data.firmware_version;
      });
    
    document.getElementById('upload_form').addEventListener('submit', function(e) {
      e.preventDefault();
      
      const fileInput = document.getElementById('file_input');
      const file = fileInput.files[0];
      
      if (!file) {
        alert('Please .bin Datei auswählen!');
        return;
      }
      
      if (!file.name.endsWith('.bin')) {
        alert('Nur .bin Dateien erlaubt!');
        return;
      }
      
      if (!confirm('Firmware Update starten?\n\n⚠️ NICHT unterbrechen!\n\nFortfahren?')) {
        return;
      }
      
      const formData = new FormData();
      formData.append('update', file);
      
      const progressContainer = document.getElementById('progress_container');
      const progressBar = document.getElementById('progress_bar');
      const statusMessage = document.getElementById('status_message');
      
      progressContainer.style.display = 'block';
      statusMessage.innerHTML = '<div style="color:#00AAFF;">⏳ Update läuft...</div>';
      
      const xhr = new XMLHttpRequest();
      
      xhr.upload.addEventListener('progress', function(e) {
        if (e.lengthComputable) {
          const percent = (e.loaded / e.total) * 100;
          progressBar.style.width = percent + '%';
          progressBar.textContent = Math.round(percent) + '%';
        }
      });
      
      xhr.addEventListener('load', function() {
        if (xhr.status === 200) {
          progressBar.style.width = '100%';
          progressBar.textContent = '100%';
          statusMessage.innerHTML = '<div style="color:#00FF00; font-size:18px;">✅ Update erfolgreich!<br><br>ESP32 startet neu...<br><br>Please 30 seconds warten!</div>';
          
          // Nach 30 seconds neu laden
          setTimeout(function() {
            location.href = '/';
          }, 30000);
        } else {
          statusMessage.innerHTML = '<div style="color:#FF0000; font-size:18px;">❌ Update fehlgeschlagen!<br><br>' + xhr.responseText + '</div>';
        }
      });
      
      xhr.addEventListener('error', function() {
        statusMessage.innerHTML = '<div style="color:#FF0000; font-size:18px;">❌ Verbindungsfehler!</div>';
      });
      
      xhr.open('POST', '/update');
      xhr.send(formData);
    });
  </script>
</body>
</html>
)rawliteral";


// Download Image von GitHub
bool downloadImage(String filename) {
  HTTPClient http;
  String url = String(IMG_BASE_URL) + filename;
  
  Serial.printf("Download: %s\n", url.c_str());
  http.begin(url);
  int httpCode = http.GET();
  
  if (httpCode == 200) {
    File f = LittleFS.open("/" + filename, "w");
    if (f) {
      http.writeToStream(&f);
      f.close();
      Serial.printf("OK: %s\n", filename.c_str());
      http.end();
      return true;
    }
  }
  
  Serial.printf("FAIL: %s (Code: %d)\n", filename.c_str(), httpCode);
  http.end();
  return false;
}

// Prüfe und lade Bilder
void checkAndDownloadImages() {
  String images[] = {
    "S50_offen_gruen.png",
    "S50_Geschlossen_gruen.png", 
    "S50_Geschlossen_rot.png",
    "S50_halboffen_orange.png"
  };
  
  for (int i = 0; i < 4; i++) {
    if (!LittleFS.exists("/" + images[i])) {
      Serial.printf("Fehlt: %s - Download...\n", images[i].c_str());
      downloadImage(images[i]);
      delay(500);
    }
  }
}

// ============= SETUP =============
void setup() {
  Serial.begin(115200);
  
  // I2C-Fehlermeldungen ausblenden (zu viele im Terminal)
  esp_log_level_set("*", ESP_LOG_ERROR);          // Alle auf ERROR
  esp_log_level_set("i2c", ESP_LOG_NONE);         // I2C komplett aus
  esp_log_level_set("i2c.master", ESP_LOG_NONE);  // I2C Master aus
  
  Serial.printf("\n=== S50 Observatory Control v%s ===\n", FIRMWARE_VERSION);
  
  // LittleFS für Bilder
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS Mount Failed!");
  } else {
    Serial.println("LittleFS OK");
  }
  
  // I2C initialisieren
  Wire.begin(21, 22);
  Serial.println("I2C initialisiert (SDA=21, SCL=22)");
  
  // HTU21D Sensor
  sht21_innen.begin();
  Serial.println("HTU21D/Si7021 Sensor initialisiert!");
  delay(500);
  
  // BMP180/BMP280 Auto-Detection
  Serial.println("Prüfe BMP-Sensoren auf I2C-Bus...");
  
  // Zuerst BMP280 prüfen (Adresse 0x76 oder 0x77)
  if (bmp280_aussen.begin(0x76) || bmp280_aussen.begin(0x77)) {
    bmp280_present = true;
    bmp_sensor_type = "BMP280";
    Serial.println("✓ BMP280 Sensor gefunden!");
    Serial.printf("   I2C-Adresse: 0x%02X\n", bmp280_aussen.sensorID());
    Serial.printf("   Chip-ID: 0x%02X\n", bmp280_aussen.sensorID());
  } 
  // Wenn kein BMP280, dann BMP180 prüfen (nur 0x77)
  else if (bmp180_aussen.begin()) {
    bmp180_present = true;
    bmp_sensor_type = "BMP180";
    Serial.println("✓ BMP180 Sensor gefunden!");
    Serial.println("   I2C-Adresse: 0x77");
  } 
  // Nonen gefunden
  else {
    bmp180_present = false;
    bmp280_present = false;
    bmp_sensor_type = "None";
    Serial.println("✗ Kein BMP-Sensor gefunden!");
    Serial.println("   → Prüfe I2C-Verkabelung (SDA=21, SCL=22)");
    Serial.println("   → Heater wird deactiveiert (keine Outside temperatur)");
  }
  delay(500);
  
  // DS3231 RTC - ECHTE I2C-Prüfung!
  Serial.println("Prüfe DS3231 RTC auf I2C-Bus...");
  
  // Prüfe ob Gerät auf Adresse 0x68 antwortet
  Wire.beginTransmission(0x68);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    // Gerät antwortet! Jetzt initialisieren
    if (rtc.begin()) {
      rtc_present = true;
      DateTime now = rtc.now();
      Serial.println("✓ DS3231 RTC gefunden und funktioniert!");
      Serial.printf("   I2C-Adresse: 0x68\n");
      Serial.printf("   Aktuelle Time: %02d.%02d.%04d %02d:%02d:%02d\n",
                    now.day(), now.month(), now.year(),
                    now.hour(), now.minute(), now.second());
      Serial.println("   RTC wird automatisch per GPS synchronisiert.");
      lastRTCRead = now;
      lastRTCMillis = millis();
      lastRTCUpdate = millis();
    } else {
      rtc_present = false;
      Serial.println("✗ DS3231 antwortet auf I2C, aber begin() fehlgeschlagen!");
    }
  } else {
    rtc_present = false;
    Serial.println("✗ DS3231 RTC nicht gefunden!");
    Serial.println("   None Antwort auf I2C-Adresse 0x68");
    Serial.println("   → Prüfe I2C-Verkabelung (SDA=21, SCL=22)");
    Serial.println("   → Time läuft per millis(), wird per GPS synchronisiert");
  }
  delay(500);
  
  // GPS UART
  gpsSerial.begin(9600, SERIAL_8N1, 13, 14);
  Serial.println("GPS initialisiert (UART2: RX=13, TX=14, 9600 Baud)");
  
  // Reed Contacts
  pinMode(REED1_PIN, INPUT_PULLUP);
  pinMode(REED2_PIN, INPUT_PULLUP);
  pinMode(REED3_PIN, INPUT_PULLUP);
  
  // Heater
  pinMode(HEATER_PIN, OUTPUT);
  digitalWrite(HEATER_PIN, LOW);
  Serial.println("Heater initialisiert (GPIO 33)");
  
  // RG-11 Rain Sensor (UART)
  rainSerial.begin(9600, SERIAL_8N1, RAIN_RX_PIN, RAIN_TX_PIN);
  Serial.println("RG-11 UART initialisiert (RX=35, TX=36, 9600 Baud)");
  delay(500);
  
  // Auto-Detect: Prüfe auf valide RG-11 Daten
  Serial.print("Suche RG-11... ");
  unsigned long start = millis();
  String buffer = "";
  bool valid_data_found = false;
  
  while (millis() - start < 5000 && !valid_data_found) {
    while (rainSerial.available()) {
      char c = rainSerial.read();
      
      if (c == '\n' || c == '\r') {
        // Prüfe ob valides RG-11 Protokoll
        if (buffer.startsWith("Acc ") || buffer.startsWith("RInt ") || buffer.startsWith("EventAcc")) {
          valid_data_found = true;
          break;
        }
        buffer = "";
      } else {
        buffer += c;
        if (buffer.length() > 50) buffer = "";
      }
    }
    delay(100);
  }
  
  if (valid_data_found) {
    rain_sensor_present = true;
    Serial.println("✓ RG-11 Rain Sensor erkannt!");
  } else {
    Serial.println("Kein RG-11 erkannt (optional)");
  }

  
  // Servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  
  // Preferences laden
  preferences.begin("s50-enclosure", false);
  servo1_pos = preferences.getInt("servo1", 90);
  servo2_pos = preferences.getInt("servo2", 90);
  servo3_pos = preferences.getInt("servo3", 90);
  
  servo1_pos1 = preferences.getInt("s1_pos1", 90);
  servo1_pos2 = preferences.getInt("s1_pos2", 90);
  servo2_pos1 = preferences.getInt("s2_pos1", 90);
  servo2_pos2 = preferences.getInt("s2_pos2", 90);
  servo2_pos3 = preferences.getInt("s2_pos3", 90);  // NEU: Position 3
  servo3_pos1 = preferences.getInt("s3_pos1", 90);
  servo3_pos2 = preferences.getInt("s3_pos2", 90);
  
  timezone_offset = preferences.getInt("timezone", 1);
  
  rain_threshold = preferences.getFloat("rain_thresh", RAIN_THRESHOLD);
  Serial.printf("Rain threshold: %.1f mm/h\n", rain_threshold);
  
  heater_hysteresis = preferences.getFloat("heater_hyst", 2.0);
  Serial.printf("Heater Hysteresis: %.1f °C\n", heater_hysteresis);
  
  heater_mode = preferences.getInt("heater_mode", 0);  // 0=Enabled, 1=Disabled
  String mode_text = (heater_mode == 0) ? "Enabled" : "Disabled";
  Serial.printf("Heater Modus: %s\n", mode_text.c_str());
  
  // Riegelüberwachung (Reed2)
  reed2_enabled = preferences.getBool("reed2_enabled", true);  // Default: activeiert
  Serial.printf("Riegelüberwachung (Reed2): %s\n", reed2_enabled ? "Enabled" : "Disabled");
  
  scheduler_enabled = preferences.getBool("sched_en", false);
  schedule_open_hour = preferences.getInt("sched_oh", 20);
  schedule_open_min = preferences.getInt("sched_om", 0);
  schedule_close_hour = preferences.getInt("sched_ch", 3);
  schedule_close_min = preferences.getInt("sched_cm", 0);
  Serial.printf("Scheduler: %s (Open: %02d:%02d, Close: %02d:%02d)\n", 
                scheduler_enabled ? "AN" : "AUS",
                schedule_open_hour, schedule_open_min,
                schedule_close_hour, schedule_close_min);
  
  Serial.printf("Geladene Positionen: S1=%d, S2=%d, S3=%d\n", 
                servo1_pos, servo2_pos, servo3_pos);
  Serial.printf("Presets: S1(%d/%d), S2(%d/%d/%d), S3(%d/%d)\n",
                servo1_pos1, servo1_pos2, 
                servo2_pos1, servo2_pos2, servo2_pos3,
                servo3_pos1, servo3_pos2);
  Serial.printf("Timezone: UTC%+d\n", timezone_offset);
  
  // Seestar IP laden
  seestar_ip = preferences.getString("seestar_ip", "192.168.1.100");
  Serial.printf("Seestar S50 IP: %s\n", seestar_ip.c_str());
  
  preferences.end();  // WICHTIG: Preferences schließen nach dem Load!
  
  // Servos auf Position fahren
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);
  
  // ============= WIFI VERBINDUNG MIT WIFIMANAGER =============
  Serial.println("\n=== WiFi Manager ===");
  
  WiFiManager wifiManager;
  
  // Callback nach WiFi-Save
  wifiManager.setSaveConfigCallback([](){
    Serial.println("");
    Serial.println("===============================================");
    Serial.println("   NEUE WiFi-EINSTELLUNGEN GESPEICHERT!");
    Serial.println("   BITTE ESP32 MANUELL NEU STARTEN:");
    Serial.println("   → Reset-Taster drücken");
    Serial.println("   → ODER Strom kurz trennen");
    Serial.println("===============================================");
    Serial.println("");
  });
  
  // Debug Output
  wifiManager.setDebugOutput(true);
  
  // Timeout für Config Portal (3 Minuten)
  wifiManager.setConfigPortalTimeout(180);
  
  // AP-Name und Passwort
  wifiManager.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  
  // Versuche zu verbinden, when fehlschlägt → Config Portal
  Serial.println("Versuche WiFi-Verbindung...");
  if (!wifiManager.autoConnect("S50-Setup")) {
    Serial.println("✗ Verbindung fehlgeschlagen - Neustart");
    delay(3000);
    ESP.restart();
  }
  
  // Verbunden!
  Serial.println("✓ WiFi verbunden!");
  Serial.print("IP Adresse: ");
  Serial.println(WiFi.localIP());
  apMode = false;
  
  // WiFi Info speichern
  current_ssid = WiFi.SSID();
  wifi_rssi = WiFi.RSSI();
  wifi_ip = WiFi.localIP().toString();
  
  // Bilder von GitHub laden (falls nicht vorhanden)
  Serial.println("\n=== Prüfe Status-Bilder ===");
  checkAndDownloadImages();
  
  // ============= WEBSERVER ROUTES =============
  
  // Captive Portal: Alle unbekannten Requests → Info-Seite (nur im AP-Modus)
  // 404 Handler
  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "Seite nicht gefunden");
  });
  
  // Homepage
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", index_html);
  });
  
  // Settings-Seite
  server.on("/settings", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", settings_html);
  });
  
  // System-Seite
  server.on("/system", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", system_html);
  });
  
  // WiFi-Seite
  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", wifi_html);
  });
  
  // Seestar IP speichern (MUSS VOR /seestar stehen!)
  server.on("/seestar/save", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("ip")) {
      String new_ip = request->getParam("ip")->value();
      
      // Speichere in Preferences
      preferences.begin("s50-enclosure", false);
      preferences.putString("seestar_ip", new_ip);
      preferences.end();
      
      seestar_ip = new_ip;
      
      Serial.printf("Seestar IP saved: %s\n", new_ip.c_str());
      
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", 
        "Seestar IP saved!\n\n" + new_ip);
      response->addHeader("Connection", "close");
      request->send(response);
    } else {
      request->send(400, "text/plain; charset=utf-8", "Error: IP fehlt");
    }
  });
  
  // Seestar-Seite
  server.on("/seestar", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", seestar_html);
  });
  
  // Rain-Seite
  server.on("/rain", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", rain_html);
  });
  
  // Rain threshold speichern
  server.on("/rain/threshold", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      float val = request->getParam("value")->value().toFloat();
      if (val >= 0.1 && val <= 10.0) {
        preferences.begin("s50-enclosure", false);
        preferences.putFloat("rain_thresh", val);
        preferences.end();
        rain_threshold = val;  // Sofort active
        request->send(200, "text/plain", "Schwellwert saved: " + String(val, 1) + " mm/h");
      } else {
        request->send(400, "text/plain", "Invalider Wert!");
      }
    } else {
      request->send(400, "text/plain", "Kein Wert angegeben!");
    }
  });
  
  // Heater Seite
  server.on("/heater", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", heater_html);
  });
  
  // Heater Hysteresis speichern
  server.on("/heater/hysteresis", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("value")) {
      float val = request->getParam("value")->value().toFloat();
      if (val >= 0.5 && val <= 10.0) {
        preferences.begin("s50-enclosure", false);
        preferences.putFloat("heater_hyst", val);
        preferences.end();
        heater_hysteresis = val;  // Sofort active
        request->send(200, "text/plain", "Hysteresis saved: " + String(val, 1) + " °C");
      } else {
        request->send(400, "text/plain", "Invalider Wert!");
      }
    } else {
      request->send(400, "text/plain", "Kein Wert angegeben!");
    }
  });
  
  // Heater Modus setzen
  server.on("/heater/mode", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("mode")) {
      int mode = request->getParam("mode")->value().toInt();
      if (mode == 0 || mode == 1) {
        preferences.begin("s50-enclosure", false);
        preferences.putInt("heater_mode", mode);
        preferences.end();
        heater_mode = mode;
        
        String mode_text = (mode == 0) ? "Enabled" : "Disabled";
        Serial.printf("Heater Modus: %s\n", mode_text.c_str());
        request->send(200, "text/plain; charset=utf-8", "Heater: " + mode_text);
      } else {
        request->send(400, "text/plain", "Invalider Modus!");
      }
    } else {
      request->send(400, "text/plain", "Kein Modus angegeben!");
    }
  });
  
  // Riegelüberwachung (Reed2) ein/ausschalten
  server.on("/reed2/toggle", HTTP_GET, [](AsyncWebServerRequest *request){
    reed2_enabled = !reed2_enabled;
    
    preferences.begin("s50-enclosure", false);
    preferences.putBool("reed2_enabled", reed2_enabled);
    preferences.end();
    
    Serial.printf("Riegelüberwachung (Reed2): %s\n", reed2_enabled ? "Enabled" : "Disabled");
    request->send(200, "text/plain; charset=utf-8", reed2_enabled ? "Enabled" : "Disabled");
  });
  
  // Scheduler-Seite
  // TEST-Route
  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("=== TEST ROUTE FUNKTIONIERT! ===");
    request->send(200, "text/plain", "PONG");
  });
  
  // Scheduler speichern (MUSS VOR /scheduler stehen!)
  server.on("/scheduler/save", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("=== /scheduler/save aufgerufen ===");
    Serial.println("=== ROUTE FUNKTIONIERT! ===");
    
    if (request->hasParam("enabled") && request->hasParam("oh") && 
        request->hasParam("om") && request->hasParam("ch") && request->hasParam("cm")) {
      
      bool enabled = (request->getParam("enabled")->value().toInt() == 1);
      int oh = request->getParam("oh")->value().toInt();
      int om = request->getParam("om")->value().toInt();
      int ch = request->getParam("ch")->value().toInt();
      int cm = request->getParam("cm")->value().toInt();
      
      Serial.printf("Empfangene Parameter: enabled=%d, oh=%d, om=%d, ch=%d, cm=%d\n", 
                    enabled, oh, om, ch, cm);
      
      if (oh >= 0 && oh <= 23 && om >= 0 && om <= 59 && 
          ch >= 0 && ch <= 23 && cm >= 0 && cm <= 59) {
        
        Serial.println("Speichere in Preferences...");
        preferences.begin("s50-enclosure", false);
        preferences.putBool("sched_en", enabled);
        preferences.putInt("sched_oh", oh);
        preferences.putInt("sched_om", om);
        preferences.putInt("sched_ch", ch);
        preferences.putInt("sched_cm", cm);
        preferences.end();
        Serial.println("Preferences saved!");
        
        // Sofort active
        scheduler_enabled = enabled;
        schedule_open_hour = oh;
        schedule_open_min = om;
        schedule_close_hour = ch;
        schedule_close_min = cm;
        
        Serial.println("=== Scheduler saved ===");
        Serial.printf("Enabled: %s\n", enabled ? "JA" : "NEIN");
        Serial.printf("Open: %02d:%02d\n", oh, om);
        Serial.printf("Close: %02d:%02d\n", ch, cm);
        Serial.printf("Globale Variablen: enabled=%d, open=%d:%d, close=%d:%d\n",
                      scheduler_enabled, schedule_open_hour, schedule_open_min,
                      schedule_close_hour, schedule_close_min);
        
        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", "OK");
        response->addHeader("Connection", "close");
        request->send(response);
      } else {
        Serial.println("ERROR: Invalide Timeangabe!");
        request->send(400, "text/plain; charset=utf-8", "Invalide Timeangabe!");
      }
    } else {
      Serial.println("ERROR: Parameter fehlen!");
      request->send(400, "text/plain; charset=utf-8", "Parameter fehlen!");
    }
  });
  
  server.on("/scheduler", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", scheduler_html);
  });
  
  // Status JSON
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request){
    // Reed-Status lesen (immer schnell)
    readReedContacts();
    
    // Rain Sensor lesen (falls vorhanden)
    readRainSensor();
    
    // Weather Data lesen (alle 500ms, da LEDs sonst zu langsam)
    temp_innen = sht21_innen.readTemperature();
    delay(50);
    hum_innen = sht21_innen.readHumidity();
    delay(50);
    
    // Humidity auf 1-100% begrenzen (Sensor-Fehler abfangen)
    if (hum_innen < 1.0) hum_innen = 1.0;
    if (hum_innen > 100.0) hum_innen = 100.0;
    
    // BMP180/BMP280 nur auslesen if present
    if (isBMPPresent()) {
      temp_aussen = readBMPTemperature();
      delay(20);
      druck_aussen = readBMPPressure();
      delay(20);
    } else {
      temp_aussen = 0.0;
      druck_aussen = 0.0;
    }
    
    // RTC Time mit millis() hochzählen
    updateRTCTime();
    
    // GPS Daten
    gps_fix = gps.location.isValid();
    gps_sats = gps.satellites.value();
    
    if (gps.time.isValid()) {
      char gpsTimeBuffer[9];
      sprintf(gpsTimeBuffer, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
      gps_time = String(gpsTimeBuffer);
    } else {
      gps_time = "--:--:--";
    }
    
    if (gps.date.isValid()) {
      char gpsDateBuffer[11];
      sprintf(gpsDateBuffer, "%02d.%02d.%04d", gps.date.day(), gps.date.month(), gps.date.year());
      gps_date = String(gpsDateBuffer);
    } else {
      gps_date = "--.--.----";
    }
    
    // enclosure-Status berechnen
    updateStatusText();
    String status_detail = getStatusDetail();
    
    // JSON zusammenbauen
    String json = "{";
    json += "\"reed1\":" + String(reed1_state ? "true" : "false") + ",";
    json += "\"reed2\":" + String(reed2_state ? "true" : "false") + ",";
    json += "\"reed3\":" + String(reed3_state ? "true" : "false") + ",";
    json += "\"reed2_enabled\":" + String(reed2_enabled ? "true" : "false") + ",";
    json += "\"servo1\":" + String(servo1_pos) + ",";
    json += "\"servo2\":" + String(servo2_pos) + ",";
    json += "\"servo3\":" + String(servo3_pos) + ",";
    json += "\"temp_innen\":" + String(temp_innen, 1) + ",";
    json += "\"hum_innen\":" + String(hum_innen, 0) + ",";
    json += "\"temp_aussen\":" + String(temp_aussen, 1) + ",";
    json += "\"druck_aussen\":" + String(druck_aussen, 0) + ",";
    json += "\"rtc_time_local\":\"" + rtc_time_local + "\",";
    json += "\"rtc_time_utc\":\"" + rtc_time_utc + "\",";
    json += "\"rtc_date\":\"" + rtc_date + "\",";
    json += "\"timezone_offset\":" + String(timezone_offset) + ",";
    json += "\"is_dst\":" + String(is_dst ? "true" : "false") + ",";
    json += "\"gps_time\":\"" + gps_time + "\",";
    json += "\"gps_date\":\"" + gps_date + "\",";
    json += "\"gps_sats\":" + String(gps_sats) + ",";
    json += "\"gps_fix\":" + String(gps_fix ? "true" : "false") + ",";
    json += "\"rain_sensor_present\":" + String(rain_sensor_present ? "true" : "false") + ",";
    json += "\"rain_detected\":" + String(rain_detected ? "true" : "false") + ",";
    json += "\"rain_rate\":" + String(rain_rate, 2) + ",";
    json += "\"rain_acc\":" + String(rain_acc, 2) + ",";
    json += "\"rain_intensity\":" + String(rain_intensity) + ",";
    json += "\"rain_threshold\":" + String(rain_threshold, 1) + ",";
    json += "\"heater_on\":" + String(heater_on ? "true" : "false") + ",";
    json += "\"heater_mode\":" + String(heater_mode) + ",";
    json += "\"dew_point\":" + String(dew_point, 1) + ",";
    json += "\"heater_hysteresis\":" + String(heater_hysteresis, 1) + ",";
    json += "\"bmp180_present\":" + String(bmp180_present ? "true" : "false") + ",";
    json += "\"bmp_sensor_present\":" + String(isBMPPresent() ? "true" : "false") + ",";
    json += "\"bmp_sensor_type\":\"" + bmp_sensor_type + "\",";
    json += "\"status_text\":\"" + status_text + "\",";
    json += "\"status_color\":\"" + status_color + "\",";
    json += "\"status_detail\":\"" + status_detail + "\",";
    json += "\"last_error\":\"" + last_error_message + "\",";
    json += "\"last_rain_close\":\"" + last_rain_close + "\",";
    json += "\"scheduler_enabled\":" + String(scheduler_enabled ? "true" : "false") + ",";
    json += "\"schedule_open_hour\":" + String(schedule_open_hour) + ",";
    json += "\"schedule_open_min\":" + String(schedule_open_min) + ",";
    json += "\"schedule_close_hour\":" + String(schedule_close_hour) + ",";
    json += "\"schedule_close_min\":" + String(schedule_close_min) + ",";
    json += "\"wifi_ssid\":\"" + current_ssid + "\",";
    json += "\"wifi_ip\":\"" + wifi_ip + "\",";
    json += "\"wifi_rssi\":" + String(wifi_rssi) + ",";
    json += "\"seestar_ip\":\"" + seestar_ip + "\",";
    json += "\"seestar_online\":" + String(seestar_online ? "true" : "false") + ",";
    json += "\"rtc_present\":" + String(rtc_present ? "true" : "false") + ",";
    json += "\"firmware_version\":\"" + String(FIRMWARE_VERSION) + "\"";
    json += "}";
    
    request->send(200, "application/json", json);
  });
  
  // Bilder servieren
  server.on("/image", HTTP_GET, [](AsyncWebServerRequest *request){
    if (!request->hasParam("name")) {
      request->send(404, "text/plain", "No image name");
      return;
    }
    
    String filename = "/" + request->getParam("name")->value();
    
    if (LittleFS.exists(filename)) {
      request->send(LittleFS, filename, "image/png");
    } else {
      request->send(404, "text/plain", "Image not found");
    }
  });
  
  // Aktionen (Kuppel öffnen/schließen, S50 Toggle)
  server.on("/action", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("cmd")) {
      String cmd = request->getParam("cmd")->value();
      
      if (cmd == "open") {
        pending_action = OPEN;
        Serial.println(">>> Open-Aktion geplant");
        request->send(200, "text/plain", "Open gestartet...");
        
      } else if (cmd == "close") {
        pending_action = CLOSE;
        Serial.println(">>> Close-Aktion geplant");
        request->send(200, "text/plain", "Close gestartet...");
        
      } else if (cmd == "toggle") {
        pending_action = TOGGLE_S50;
        Serial.println(">>> S50-Toggle geplant");
        request->send(200, "text/plain", "S50 Toggle gestartet...");
        
      } else {
        request->send(400, "text/plain", "Unbekannter Befehl");
      }
    } else {
      request->send(400, "text/plain", "Kein Befehl angegeben");
    }
  });
  
  // Servo manuelle Steuerung
  server.on("/servo", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("id") && request->hasParam("delta")) {
      int servo_id = request->getParam("id")->value().toInt();
      int delta = request->getParam("delta")->value().toInt();
      
      if (servo_id == 1) {
        int new_pos = constrain(servo1_pos + delta, 0, 180);
        moveServoSlow(servo1, servo1_pos, new_pos);
        preferences.putInt("servo1", servo1_pos);
        Serial.printf("Servo 1 -> %d°\n", servo1_pos);
      } else if (servo_id == 2) {
        int new_pos = constrain(servo2_pos + delta, 0, 180);
        moveServoSlow(servo2, servo2_pos, new_pos);
        preferences.putInt("servo2", servo2_pos);
        Serial.printf("Servo 2 -> %d°\n", servo2_pos);
      } else if (servo_id == 3) {
        int new_pos = constrain(servo3_pos + delta, 0, 180);
        moveServoSlow(servo3, servo3_pos, new_pos);
        preferences.putInt("servo3", servo3_pos);
        Serial.printf("Servo 3 -> %d°\n", servo3_pos);
      }
      
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Error: Parameter fehlen");
    }
  });
  
  // Goto savede Position
  server.on("/goto", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("servo") && request->hasParam("pos")) {
      int servo_id = request->getParam("servo")->value().toInt();
      int pos_num = request->getParam("pos")->value().toInt();
      
      int target_pos = 90;
      
      if (servo_id == 1) {
        target_pos = (pos_num == 1) ? servo1_pos1 : servo1_pos2;
        moveServoSlow(servo1, servo1_pos, target_pos);
        preferences.putInt("servo1", servo1_pos);
        Serial.printf("Servo 1 -> Preset %d (%d°)\n", pos_num, servo1_pos);
        
      } else if (servo_id == 2) {
        // 3 Positionen für Servo2!
        if (pos_num == 1) target_pos = servo2_pos1;
        else if (pos_num == 2) target_pos = servo2_pos2;
        else if (pos_num == 3) target_pos = servo2_pos3;
        
        moveServoSlow(servo2, servo2_pos, target_pos);
        preferences.putInt("servo2", servo2_pos);
        Serial.printf("Servo 2 -> Preset %d (%d°)\n", pos_num, servo2_pos);
        
      } else if (servo_id == 3) {
        target_pos = (pos_num == 1) ? servo3_pos1 : servo3_pos2;
        moveServoSlow(servo3, servo3_pos, target_pos);
        preferences.putInt("servo3", servo3_pos);
        Serial.printf("Servo 3 -> Preset %d (%d°)\n", pos_num, servo3_pos);
      }
      
      request->send(200, "text/plain", "OK");
    } else {
      request->send(400, "text/plain", "Error: Parameter fehlen");
    }
  });
  
  // Speichere aktuelle Position als Preset
  server.on("/save", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("servo") && request->hasParam("pos")) {
      int servo_id = request->getParam("servo")->value().toInt();
      int pos_num = request->getParam("pos")->value().toInt();
      
      String msg = "";
      
      preferences.begin("s50-enclosure", false);
      
      if (servo_id == 1) {
        if (pos_num == 1) {
          servo1_pos1 = servo1_pos;
          preferences.putInt("s1_pos1", servo1_pos1);
          msg = "Servo 1 Pos1 saved: " + String(servo1_pos1) + "°";
        } else {
          servo1_pos2 = servo1_pos;
          preferences.putInt("s1_pos2", servo1_pos2);
          msg = "Servo 1 Pos2 saved: " + String(servo1_pos2) + "°";
        }
        
      } else if (servo_id == 2) {
        // 3 Positionen für Servo2!
        if (pos_num == 1) {
          servo2_pos1 = servo2_pos;
          preferences.putInt("s2_pos1", servo2_pos1);
          msg = "Servo 2 Pos1 (geschlossen) saved: " + String(servo2_pos1) + "°";
        } else if (pos_num == 2) {
          servo2_pos2 = servo2_pos;
          preferences.putInt("s2_pos2", servo2_pos2);
          msg = "Servo 2 Pos2 (offen) saved: " + String(servo2_pos2) + "°";
        } else if (pos_num == 3) {
          servo2_pos3 = servo2_pos;
          preferences.putInt("s2_pos3", servo2_pos3);
          msg = "Servo 2 Pos3 (halboffen) saved: " + String(servo2_pos3) + "°";
        }
        
      } else if (servo_id == 3) {
        if (pos_num == 1) {
          servo3_pos1 = servo3_pos;
          preferences.putInt("s3_pos1", servo3_pos1);
          msg = "Servo 3 Pos1 saved: " + String(servo3_pos1) + "°";
        } else {
          servo3_pos2 = servo3_pos;
          preferences.putInt("s3_pos2", servo3_pos2);
          msg = "Servo 3 Pos2 saved: " + String(servo3_pos2) + "°";
        }
      }
      
      preferences.end();
      
      Serial.println(msg);
      
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", msg);
      response->addHeader("Connection", "close");
      request->send(response);
    } else {
      request->send(400, "text/plain; charset=utf-8", "Error: Parameter fehlen");
    }
  });
  
  // Timezone anpassen
  server.on("/timezone", HTTP_GET, [](AsyncWebServerRequest *request){
    if (request->hasParam("delta")) {
      int delta = request->getParam("delta")->value().toInt();
      timezone_offset = constrain(timezone_offset + delta, -12, 14);
      
      preferences.begin("s50-enclosure", false);
      preferences.putInt("timezone", timezone_offset);
      preferences.end();
      
      String msg = "Timezone: UTC" + String(timezone_offset >= 0 ? "+" : "") + String(timezone_offset);
      Serial.println(msg);
      
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", msg);
      response->addHeader("Connection", "close");
      request->send(response);
    } else {
      request->send(400, "text/plain; charset=utf-8", "Error: Parameter fehlen");
    }
  });
  
  // WiFi Scan - Asynchron
  
  // WiFi Save & Reboot
  
  // System Load Defaults (nur Servo Positions zurücksetzen)
  server.on("/system/defaults", HTTP_GET, [](AsyncWebServerRequest *request){
    Serial.println("=== LOAD DEFAULTS ===");
    
    // Setze alle Servo Positions auf 90°
    servo1_pos = 90;
    servo2_pos = 90;
    servo3_pos = 90;
    
    servo1_pos1 = 90;
    servo1_pos2 = 90;
    servo2_pos1 = 90;
    servo2_pos2 = 90;
    servo2_pos3 = 90;
    servo3_pos1 = 90;
    servo3_pos2 = 90;
    
    // Speichere in Preferences
    preferences.begin("s50-enclosure", false);
    preferences.putInt("servo1", 90);
    preferences.putInt("servo2", 90);
    preferences.putInt("servo3", 90);
    preferences.putInt("s1_pos1", 90);
    preferences.putInt("s1_pos2", 90);
    preferences.putInt("s2_pos1", 90);
    preferences.putInt("s2_pos2", 90);
    preferences.putInt("s2_pos3", 90);
    preferences.putInt("s3_pos1", 90);
    preferences.putInt("s3_pos2", 90);
    preferences.end();
    
    // Fahre Servos auf 90°
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    
    Serial.println("Alle Servo-Winkel auf 90° zurückgesetzt");
    Serial.println("WiFi und Timezone bleiben erhalten");
    
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain; charset=utf-8", "OK");
    response->addHeader("Connection", "close");
    request->send(response);
  });
  
  // Update-Seite
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html; charset=UTF-8", update_html);
  });
  
  // Update POST Handler
  server.on("/update", HTTP_POST, 
    [](AsyncWebServerRequest *request){
      bool shouldReboot = !Update.hasError();
      AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", shouldReboot ? "OK" : "FAIL");
      response->addHeader("Connection", "close");
      request->send(response);
      if (shouldReboot) {
        delay(100);
        ESP.restart();
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
      if (!index) {
        Serial.printf("Update Start: %s\n", filename.c_str());
        if (!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)) {
          Update.printError(Serial);
        }
      }
      if (!Update.hasError()) {
        if (Update.write(data, len) != len) {
          Update.printError(Serial);
        }
      }
      if (final) {
        if (Update.end(true)) {
          Serial.printf("Update Success: %uB\n", index+len);
        } else {
          Update.printError(Serial);
        }
      }
    }
  );
  
  // Server starten
  server.begin();
  Serial.println("Webserver gestartet!");
  Serial.println("OTA Update activeiert auf http://" + WiFi.localIP().toString() + "/update");
  Serial.println("Öffne http://" + WiFi.localIP().toString());
}


// ============= LOOP =============
void loop() {
  // Führe geplante Aktionen aus (non-blocking, kein Watchdog-Problem)
  if (pending_action != NONE) {
    PendingAction action = pending_action;
    pending_action = NONE;  // Reset sofort (verhindert doppelte Ausführung)
    
    switch (action) {
      case OPEN:
        {
          bool success = openDome();
          if (success) {
            Serial.println("✓ Open erfolgreich");
          } else {
            Serial.println("✗ Open fehlgeschlagen");
          }
        }
        break;
        
      case CLOSE:
        {
          bool success = closeDome();
          if (success) {
            Serial.println("✓ Close erfolgreich");
          } else {
            Serial.println("⚠ Nur half-open (Tube not retracted)");
          }
        }
        break;
        
      case TOGGLE_S50:
        toggleS50();
        Serial.println("✓ S50 Toggle ausgeführt");
        break;
        
      default:
        break;
    }
  }
  
  // Tube-Überwachung: Automatics Halböffnen when Tube ausfährt
  // ABER NUR when enclosure CLOSED ist!
  static bool last_tubus_standby = true;  // Startzustand
  readReedContacts();
  
  // Erkenne Übergang: Standby → Aktiv (Tube fährt aus)
  if (last_tubus_standby && !reed3_state) {
    // Nur auf half-open fahren when Cover CLOSED ist (nicht bei open!)
    if (servo2_pos == servo2_pos1) {
      Serial.println("⚠️ TUBUS FÄHRT OFF! Öffne Cover automatisch auf HALF-OPEN...");
      moveServoSlow(servo2, servo2_pos, servo2_pos3);
      servo2_pos = servo2_pos3;
      preferences.begin("s50-enclosure", false);
      preferences.putInt("servo2", servo2_pos);
      preferences.end();
      Serial.println("✓ Cover auf HALF-OPEN (Sicherheit)");
    } else {
      Serial.println("⚠️ TUBUS FÄHRT OFF, aber Cover ist bereits open → keine Änderung");
    }
  }
  
  last_tubus_standby = reed3_state;
  
  // GPS Daten einlesen
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
  
  // GPS -> RTC Sync (alle 10 Minuten when Fix vorhanden)
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
    if (millis() - last_gps_sync > GPS_SYNC_INTERVAL || last_gps_sync == 0) {
      syncRTCwithGPS();
      last_gps_sync = millis();
    }
  }
  
  // Heater Steuerung (alle 10 seconds prüfen)
  static unsigned long last_heater_check = 0;
  if (millis() - last_heater_check > 10000 || last_heater_check == 0) {
    controlHeater();
    last_heater_check = millis();
  }
  
  // Scheduler prüfen (jede Minute)
  static unsigned long last_scheduler_check = 0;
  if (millis() - last_scheduler_check > 60000 || last_scheduler_check == 0) {
    checkScheduler();
    last_scheduler_check = millis();
  }
  
  // WiFi Signal-Stärke aktualisieren (alle 5 seconds)
  static unsigned long last_wifi_update = 0;
  if (!apMode && (millis() - last_wifi_update > 5000 || last_wifi_update == 0)) {
    wifi_rssi = WiFi.RSSI();
    last_wifi_update = millis();
  }
  
  // Seestar S50 Ping Check (alle 5 seconds)
  if (!apMode && (millis() - last_ping_check > 5000 || last_ping_check == 0)) {
    checkSeestarOnline();
    last_ping_check = millis();
  }
  
  // Rain Sensor prüfen und ggf. Emergency close (alle 10s)
  if (rain_sensor_present) {
    static unsigned long last_rain_check = 0;
    
    if (millis() - last_rain_check >= 10000) {
      last_rain_check = millis();
      readRainSensor();
    }
    
    static bool rain_emergency_triggered = false;
    
    if (rain_detected && !rain_emergency_triggered) {
      Serial.println("⚠️ REGEN ERKANNT! Starte Emergency close...");
      emergencyRainClose();
      rain_emergency_triggered = true;
      // last_rain_close wird in emergencyRainClose() gesetzt
      // last_error_message wird nur bei Fehler gesetzt
    }
    
    if (!rain_detected && rain_emergency_triggered) {
      rain_emergency_triggered = false;  // Reset when wieder trocken
      // Meldung bleibt bestehen zur Info
    }
  }
  
  delay(10);
}

