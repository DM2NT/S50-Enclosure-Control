# GitHub Upload - Schritt für Schritt

## Variante 1: GitHub Desktop (Einfach!)

### 1. GitHub Desktop installieren
- Download: https://desktop.github.com/
- Installieren und mit GitHub-Account anmelden

### 2. Neues Repository erstellen
1. **File → New Repository**
2. Name: `seestar-observatory`
3. Description: `ESP32-based weather station and automatic cover for Seestar S50 telescope`
4. Local Path: Ordner auswählen wo Projekt liegt
5. Git Ignore: None (wir haben schon .gitignore)
6. License: MIT (oder None, wir haben schon LICENSE)
7. **Create Repository**

### 3. Dateien committen
1. Alle Dateien sollten in der Liste erscheinen
2. Commit Message: `Initial commit - Firmware v3.1`
3. **Commit to main**

### 4. Zu GitHub hochladen
1. **Publish repository**
2. Haken bei "Keep this code private" entfernen (für öffentlich)
3. **Publish Repository**

**Fertig!** 🎉

Repository URL: `https://github.com/DEIN_USERNAME/seestar-observatory`

---

## Variante 2: Git Kommandozeile (Fortgeschritten)

### 1. GitHub Repository erstellen
1. Zu https://github.com/new gehen
2. Repository Name: `seestar-observatory`
3. Description: `ESP32-based weather station and automatic cover for Seestar S50 telescope`
4. Public
5. **NICHT** "Initialize with README" anklicken (haben wir schon!)
6. **Create Repository**

### 2. Git initialisieren
```bash
cd /pfad/zum/seestar-observatory

# Git initialisieren
git init

# Remote hinzufügen (DEIN_USERNAME anpassen!)
git remote add origin https://github.com/DEIN_USERNAME/seestar-observatory.git

# Dateien hinzufügen
git add .

# Ersten Commit
git commit -m "Initial commit - Firmware v3.1"

# Zu GitHub hochladen
git branch -M main
git push -u origin main
```

**Benutzername & Passwort:**
- Username: Dein GitHub Username
- Password: **Personal Access Token** (nicht Passwort!)
  - Erstellen unter: https://github.com/settings/tokens
  - Scope: `repo` auswählen

---

## Nach dem Upload

### README anpassen
1. Auf GitHub Repository gehen
2. README.md öffnen
3. Bei Bedarf YouTube-Link, Callsign, etc. anpassen
4. **Commit changes**

### Releases erstellen
```bash
# Tag für v3.1 erstellen
git tag -a v3.1 -m "Version 3.1 - ASCII Protocol RG-11"
git push origin v3.1
```

Auf GitHub:
1. **Releases** → **Create a new release**
2. Tag: `v3.1`
3. Title: `v3.1 - ASCII Protocol RG-11`
4. Description: Changelog einfügen
5. `.ino.bin` Datei anhängen (für OTA-Updates!)
6. **Publish release**

### Repository-Einstellungen

**About (rechts oben):**
- Description: Kurzbeschreibung
- Website: YouTube-Kanal oder Homepage
- Topics: `esp32`, `arduino`, `weather-station`, `astronomy`, `seestar-s50`

**README Badges hinzufügen (optional):**
```markdown
![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)
![Arduino](https://img.shields.io/badge/Arduino-IDE-00979D?logo=arduino)
![ESP32](https://img.shields.io/badge/ESP32-DevKit-blue)
```

---

## Updates hochladen

### Neue Version kompilieren
1. Code ändern
2. Version-Nummer erhöhen: `#define FIRMWARE_VERSION "3.2"`
3. Arduino IDE: Kompilieren

### Zu Git committen
```bash
git add .
git commit -m "v3.2 - Bug fixes"
git push
```

### Release erstellen
```bash
git tag -a v3.2 -m "Version 3.2 - Bug fixes"
git push origin v3.2
```

---

## Tipps

**Gute Commit-Messages:**
- ✅ `Fix RG-11 boot detection`
- ✅ `Add wind sensor calibration`
- ❌ `update`
- ❌ `fix stuff`

**Issues aktivieren:**
- Settings → Features → Issues ✓
- Für Bug-Reports und Feature-Requests

**Wiki aktivieren:**
- Settings → Features → Wiki ✓
- Für erweiterte Dokumentation

**GitHub Actions (CI/CD):**
- Automatisches Kompilieren bei jedem Push
- `.github/workflows/arduino.yml` erstellen
- Template: https://github.com/arduino/compile-sketches

---

**Bei Fragen: DM2NT kontaktieren!**

73! 📡
