[English](README.md) | [Deutsch](README.de.md)

# Inhero MR-1 Hardware-Variante

Der Inhero MR-1 ist ein solarbetriebener Mesh-Netzwerk-Knoten mit fortschrittlichem Batteriemanagement, konfigurierbarer Chemie-Unterstützung und MPPT (Maximum Power Point Tracking) Steuerung.

## Hardware-Übersicht

### Hauptkomponenten (v0.2 - Aktuell)
- **MCU**: Nordic nRF52840 (64MHz, 243KB RAM, 796KB Flash)
- **Funkmodul**: SX1262 LoRa-Transceiver mit DIO2-RF-Umschaltung
- **Batteriemanager**: BQ25798 mit integriertem MPPT und NTC-Thermistor-Unterstützung
- **Power Monitor**: INA228 (20mΩ Shunt, 1A max) für Coulomb Counting
- **RTC**: RV-3028-C7 für Wake-up Management
- **Stromeingang**: Solarpanel mit Power Good Interrupt-Erkennung
- **Speicher**: LittleFS-basierte Einstellungen mit SimplePreferences-Wrapper

### Hauptkomponenten (v0.1 - Legacy)
- **Digitales Potentiometer**: MCP4652 (Dual-Kanal) für TP2120 UVLO-Steuerung
- Keine INA228 oder RTC

### Pin-Konfiguration
```cpp
// LoRa-Funkmodul
P_LORA_DIO_1    = 47
P_LORA_NSS      = 42
P_LORA_BUSY     = 46
P_LORA_SCLK     = 43
P_LORA_MISO     = 45
P_LORA_MOSI     = 44
SX126X_POWER_EN = 37

// GPS (Optional)
PIN_GPS_1PPS    = 17
GPS_ADDRESS     = 0x42 (I2C)

// Batterieüberwachung
PIN_VBAT_READ   = 5
```

## Funktionen

### Batteriemanagement
- **Multi-Chemie-Unterstützung**:
  - LTO 2S (Lithium-Titanat-Oxid, 2 Zellen in Reihe)
  - LiFePO4 1S (Lithium-Eisenphosphat, 1 Zelle)
  - Li-Ion 1S (Lithium-Ionen, 1 Zelle)
  
- **JEITA-Temperatursteuerung**:
  - Automatische Reduzierung des Ladestroms bei niedrigen/hohen Temperaturen
  - NTC-Thermistor-basierte Batterietemperaturüberwachung
  - Konfigurierbare Temperaturschwellen (COOL, WARM, HOT, COLD)

- **Frost-Lade-Verhalten**:
  - `0%` - Kein Laden unter 0°C
  - `20%` - Ladestrom auf 20% bei niedrigen Temperaturen reduzieren
  - `40%` - Ladestrom auf 40% bei niedrigen Temperaturen reduzieren
  - `100%` - Keine Reduzierung (nicht empfohlen für kalte Umgebungen)

- **Konfigurierbare Parameter**:
  - Maximaler Ladestrom (10-1000mA)
  - Reduzierte Ladespannung (für verlängerte Batterielebensdauer)
  - MPPT aktivieren/deaktivieren

### Solar-Power-Management
- **MPPT-Task**: FreeRTOS-Task überwacht Solar-Eingang alle 15 Minuten
- **Interrupt-gesteuert**: Hardware-Interrupt bei Power Good Zustandsänderungen
- **Automatische Wiederherstellung**: Erkennt und reaktiviert MPPT bei Chip-Störungen
- **Visuelle Rückmeldung**: Blaues LED-Blinken bei Solar-Ereignissen

### Telemetrie-System
Verwendet CayenneLPP-Format mit zwei Kanälen:

**Kanal N (Solar-Eingang)**:
- Spannung (V)
- Strom (A)
- Leistung (W)
- MPPT-Status (boolean)

**Kanal N+1 (Batterie)**:
- Spannung (V)
- Strom (A) - Laden/Entladen
- Leistung (W)
- Temperatur (°C)

## Konfiguration

### CLI-Befehle

#### Get-Befehle
```bash
board.bat       # Aktuellen Batterietyp abrufen
board.frost     # Frost-Lade-Verhalten abrufen
board.life      # Reduzierte Spannungseinstellung abrufen (true/false)
board.imax      # Maximalen Ladestrom abrufen (mA)
board.mppt      # MPPT-Status abrufen
board.mpps      # 7-Tage-MPPT-Statistiken abrufen (Prozentsatz + Energie)
board.cinfo     # Ladegerätstatus abrufen (Power Good / Ladezustand)
board.telem     # Vollständige Telemetrie-Momentaufnahme abrufen
board.hwver     # Hardware-Version anzeigen
                # Ausgabe: v0.1 (MCP4652)
                # Hinweis: MR1 ist immer v0.1 Hardware
# Hinweis: Erweiterte Power-Management-Features (SOC, balance) sind nur in MR2 (v0.2) verfügbar
# Siehe ../inhero_mr2/README.md für v0.2 Features
```

#### Set-Befehle
```bash
set board.bat <typ>            # Batterietyp festlegen
                               # Optionen: lto2s | lifepo1s | liion1s

set board.frost <verhalten>    # Frost-Lade-Verhalten festlegen
                               # Optionen: 0% | 20% | 40% | 100%

set board.life <true|false>    # Reduzierte Ladespannung aktivieren/deaktivieren
                               # Verlängert Batterielebensdauer durch Reduzierung der max. Spannung

set board.imax <strom>         # Maximalen Ladestrom in mA festlegen
                               # Bereich: 10-1000mA

set board.mppt <true|false>    # MPPT aktivieren/deaktivieren

set board.bqreset              # BQ25798 zurücksetzen und Konfiguration aus FS laden
                               # Führt Software-Reset durch und konfiguriert
                               # Batterietyp, Ladelimits, MPPT-Einstellungen
                               # aus gespeicherten Präferenzen neu

set board.batcap <mAh>         # Batteriekapazität manuell setzen (v0.2) 🆕
                               # Bereich: 100-100000 mAh
```

### Konfigurationsbeispiele

**Standard-Li-Ion-Setup (Standard)**:
```bash
set board.bat liion1s
set board.frost 20%
set board.life false
set board.imax 500
set board.mppt true
```

**LiFePO4-Langlebigkeits-Setup**:
```bash
set board.bat lifepo1s
set board.frost 40%
set board.life true
set board.imax 300
set board.mppt true
```

**Kaltwetter-Setup**:
```bash
set board.frost 0%     # Kein Laden unter 0°C
set board.imax 200     # Reduzierter Strom
```

### MPPT-Statistiken

Das Board verfolgt automatisch den MPPT-Status und die Solar-Energiegewinnung über die Zeit mit einem Interrupt-gesteuerten Ansatz:

- **Überwachungszeitraum**: 7 Tage (168 Stunden)
- **Update-Trigger**: BQ25798-Interrupts (bei MPPT-Statusänderungen) + periodische 15-Min-Checks
- **Speicherung**: Rolling Circular Buffer (im Speicher)
- **Metriken**: 
  - MPPT-Betriebszeit: Prozentsatz der Zeit, in der MPPT aktiviert war (7-Tage-Durchschnitt)
  - Solarenergie: Durchschnittliche täglich gewonnene Energie in mWh (3-Tage-Durchschnitt)
- **Energie-Integration**: Berechnet E = P × Δt mit Solar-Panel-Leistungsmessungen
- **Task-Integration**: Verwendet bestehenden `solarMpptTask` - keine zusätzliche Task-Overhead

**Beispielausgabe**:
```bash
board.mpptstat
> MPPT 7d avg: 87.3%, E_daily 3d: 1250mWh (3.0d data)
```

Dies zeigt:
- MPPT war für 87,3% der Zeit in den letzten 7 Tagen aktiviert
- Das Solarpanel hat durchschnittlich 1250 mWh pro Tag in den letzten 3 Tagen geerntet
- 3,0 Tage gültige Daten sind für die Energieberechnung verfügbar

Der 7-Tage-MPPT-Prozentsatz liefert eine langfristige Stabilitätsindikation, während der 3-Tage-Energiedurchschnitt Reaktionsfähigkeit auf Wetteränderungen mit Stabilität gegen kurzfristige Schwankungen ausbalanciert.

**Anwendungsfälle**:
- MPPT-Stabilität über die Zeit überwachen
- Konfigurationsprobleme oder Hardware-Störungen erkennen
- Solar-Lade-Effizienz und tägliche Energiegewinnung verfolgen
- Energiegewinnung über verschiedene Tage/Jahreszeiten vergleichen
- Automatische MPPT-Wiederherstellung überprüfen
- Batterielebensdauer basierend auf tatsächlichem Solar-Eingang schätzen

**Implementierung**:
Die Statistikverfolgung ist vollständig in den bestehenden `solarMpptTask` integriert und nutzt das Interrupt-System des BQ25798. Wenn sich der MPPT-Status ändert, wird ein Interrupt ausgelöst und die vergangene Zeit wird berücksichtigt. Zusätzlich wird die Solar-Panel-Leistung (Spannung × Strom) kontinuierlich integriert, um die gewonnene Energie zu berechnen: E = P × Δt. Dieser Interrupt-gesteuerte Ansatz ist hocheffizient und präzise und erfordert keinen Polling-Overhead. Die Zeitverfolgung verwendet RTC, wenn verfügbar, mit automatischem Fallback auf millis() während des Systemstarts.

## Technische Details v0.2

### INA228 Power Monitor (v0.2)
- **I2C-Adresse**: 0x45 (A0=GND, A1=GND)
- **Shunt-Widerstand**: 20mΩ (R027)
- **Messbereich**: ±40.96mV (ADC Range)
- **Maximaler Strom**: 1A durch Shunt
- **ADC-Auflösung**: 20-bit
- **Funktionen**:
  - Spannungs-/Strom-/Leistungsüberwachung
  - Energieakkumulation (Coulomb Counter)
  - Ladungsakkumulation (mAh-Tracking)
  - Hardware-UVLO-Alarm
  - Die-Temperatursensor
  - Shutdown-Modus (~1µA)

### Batterie-SOC (Ladezustand)
- **Coulomb Counting**: Primäre SOC-Berechnungsmethode
- **Spannungs-Fallback**: Chemie-spezifische Spannungskurven
- **Auto-Learning**: Lernt Kapazität während vollständiger Ladezyklen
- **Manueller Override**: Kapazität via `set board.batcap <mAh>` setzen
- **Genauigkeit**: ±2% bei richtiger Kapazitätskonfiguration

### Tägliche Energiebilanz
Verfolgt Energiefluss über 7-Tage-Rolling-Fenster:
- **Heutige Bilanz**: Solarladung vs. Entladung
- **3-Tage-Durchschnitt**: Trendanalyse für Prognosen
- **Lebensstatus**: Erkennt automatisch "leben von Batterie" vs "leben von Solar"
- **Persistenz**: Überlebt Reboots durch statistische Glättung

### Time To Live (TTL) Prognose
Bei Leben von Batterie (Netto-Defizit) berechnet:
```
TTL (Stunden) = (Aktueller SOC × Kapazität) / Durchschnittliches tägliches Defizit
```
Beispiel:
- SOC: 60% (1200 mAh verbleibend)
- 3-Tage-Durchschnitt Defizit: -100 mAh/Tag
- TTL: 1200 / 100 = 12 Tage = 288 Stunden

### RV-3028 RTC-Integration
- **Countdown-Timer**: Stündliche Aufwachzyklen während Shutdown
- **INT-Pin**: GPIO17 löst Aufwachen aus SYSTEMOFF aus
- **Niedrigspannungs-Wiederherstellung**: 
  1. RAK erreicht "Danger Zone" (z.B. 2.9V für LiFePO4)
  2. INA228 geht in Shutdown-Modus (stoppt Coulomb Counter)
  3. Kontrollierter Shutdown mit Dateisystem-Sync
  4. RTC wacht jede Stunde auf
  5. Prüft Spannungserholung
  6. INA228 wacht auf und prüft Ladezustand
  7. Fährt fort, wenn Spannung erholt UND ausreichende Ladung gewonnen

### Power Management Flow
Beim Eintritt in Niedrigspannungs-Shutdown:
1. **Hintergrund-Tasks stoppen**: Verhindert Dateisystem-Korruption
2. **INA228 Shutdown**: Versetzt Power Monitor in Power-Down-Modus (~1µA)
   - Stoppt alle ADC-Konversionen
   - Deaktiviert Coulomb Counter (kein Zählen bei 0% SOC sowieso)
3. **RTC Wake konfigurieren**: Setzt Countdown-Timer auf 1 Stunde
4. **Shutdown-Grund speichern**: Speichert in GPREGRET2 für nächsten Boot
5. **SYSTEMOFF eintreten**: nRF52 Tiefschlaf (1-5µA gesamt)

### Hardware-UVLO (Under-Voltage Lockout)
- **INA228 Alert-Pin**: Steuert direkt TPS62840 EN-Pin
- **Chemie-spezifische Schwellenwerte**:
  - Li-Ion: 3.2V (absolutes Minimum)
  - LiFePO4: 2.8V (absolutes Minimum)
  - LTO 2S: 4.0V (absolutes Minimum)
- **Schutzschichten**:
  1. Software "Danger Zone" (200mV vor Hardware-Cutoff)
  2. Hardware-Alarm (INA228 → TPS EN)
  3. Null Stromverbrauch bei Auslösung (0 µA)

### Batterieladegerät (BQ25798)
- **Eingangsspannungsbereich**: 3,6V - 24V
- **Ladestrom**: Konfigurierbar 10mA - 5A
- **MPPT**: Automatisches Maximum Power Point Tracking für Solar
- **ADC-Auflösung**: 15-bit (konfigurierbar auf 12/13/14-bit)
- **Temperatursensor**: NTC-Thermistor mit Beta-Gleichungsberechnung
- **Interrupt-System**: Nur Solar-Interrupts (Power Good Überwachung)

### Digitales Potentiometer (MCP4652) - nur v0.1
- **Kanäle**: 2 unabhängige Schleifer
- **Auflösung**: 257 Stufen (0-256)
- **Schnittstelle**: I2C (Standardadresse 0x2F)
- **Zweck**: TP2120 UVLO-Komparator-Steuerung
- **Hinweis**: In v0.2 durch INA228 ersetzt

### Einstellungsspeicher
Persistente Konfiguration gespeichert in LittleFS:
- **Namespace**: `inhero_mr1`
- **Schlüssel**: 
  - `battery_type` - Aktuelle Batterie-Chemie
  - `frost_behavior` - Niedrigtemperatur-Lade-Verhalten
  - `max_charge_current_ma` - Ladestrom-Limit
  - `reduced_voltage` - Reduzierte Spannungs-Flag

### Temperaturberechnung
Das Board verwendet eine Steinhart-Hart / Beta-Gleichung für genaue Batterietemperatur:
```cpp
// NTC-Widerstandsnetzwerk
R_PULLUP    = 5600Ω   (RT1)
R_PARALLEL  = 27000Ω  (RT2)
R_NTC_25    = 10000Ω  (NCP15XH103F03RC bei 25°C)
BETA_VAL    = 3380    (Beta-Koeffizient)
```

## Build-Informationen

### Abhängigkeiten
- RadioLib @ 7.5.0
- Adafruit BQ25798 Library
- Adafruit BME280 Library
- Adafruit GFX Library
- CayenneLPP @ 1.6.1
- ArduinoJson @ 7.4.2

### Build-Konfiguration
- **Plattform**: Nordic nRF52 (10.10.0)
- **Framework**: Arduino
- **Board-Definition**: Custom `inhero_mr1`
- **Umgebung**: `Inhero_MR1_repeater`

### Speicherauslastung (Typisch)
```
RAM:   11,6% (28.772 Bytes / 248.832 Bytes)
Flash: 37,1% (302.080 Bytes / 815.104 Bytes)
```

## Sicherheitsfunktionen

1. **Thermischer Schutz**: JEITA-konforme Temperaturüberwachung
2. **Eingangsschutz**: VINDPM (Input Voltage Dynamic Power Management)
3. **Batterieschutz**: Überstrom-, Überspannungs-, Unterspannungsschutz
4. **Grenzwertprüfung**: Alle Konfigurationswerte werden vor der Anwendung validiert
5. **Watchdog**: Hardware-Watchdog (600s Timeout, aktiviert in Release-Builds)

## Entwicklung

### Watchdog-Timer

Der Inhero MR-1 verfügt über einen Hardware-Watchdog-Timer (nRF52 WDT) mit folgenden Eigenschaften:

- **Timeout**: 600 Sekunden (10 Minuten)
- **Aktivierung**: Automatisch in Release-Builds aktiviert (deaktiviert im DEBUG_MODE)
- **Zweck**: Erkennt und behebt System-Hänger, Deadlocks oder Task-Fehler
- **Verhalten**: Läuft während des Schlafmodus weiter, pausiert beim Debuggen
- **Fütterung**: Muss aus der Hauptschleife über `board.tick()` aufgerufen werden
- **OTA-kompatibel**: 600s Timeout ermöglicht OTA-Updates (typisch 2-5 Minuten)

**Wichtig**: Ihre Anwendung **muss** regelmäßig `board.tick()` aufrufen, um den Watchdog zu füttern:

```cpp
void loop() {
  board.tick();        // Watchdog füttern - ERFORDERLICH!
  the_mesh.loop();
  sensors.loop();
  rtc_clock.tick();
}
```

Ohne den Aufruf von `board.tick()` wird das System nach 120 Sekunden zurückgesetzt. Dies ist beabsichtigt - es stellt sicher, dass die Hauptschleife ordnungsgemäß läuft.

**Reset-Erkennung**: Watchdog-Resets werden über `NRF52Board::getResetReasonString()` protokolliert und erscheinen als "Watchdog" in der Debug-Ausgabe.

### Hinzufügen benutzerdefinierter Funktionen
Die Board-Klasse `InheroMr1Board` erbt von `mesh::MainBoard` und bietet:
- `getCustomGetter()` - Benutzerdefinierte CLI-Get-Befehle
- `setCustomSetter()` - Benutzerdefinierte CLI-Set-Befehle  
- `queryBoardTelemetry()` - CayenneLPP-Telemetrie-Export
- `getBattMilliVolts()` - Batteriespannungsablesung

### FreeRTOS-Task
Der `solarMpptTask` läuft kontinuierlich:
- **Priorität**: 1 (niedrig)
- **Stack-Größe**: 4096 Bytes
- **Aufwachbedingungen**: 
  - Solar-Interrupt (Power Good Zustandsänderung)
  - 15-Minuten-Timeout (periodische Prüfung)

## Fehlerbehebung

### MPPT funktioniert nicht
1. Solarpanel-Spannung prüfen (muss > Batteriespannung sein)
2. Überprüfen, ob MPPT aktiviert ist: `board.mppt`
3. Ladegerätstatus prüfen: `board.info`
4. Telemetrie überwachen: `board.tele`

### Batterie lädt nicht
1. Temperatur im Bereich prüfen: `board.tele`
2. Ladestrom ist nicht 0 überprüfen: `board.imax`
3. Frost-Verhaltenseinstellung prüfen: `board.frost`
4. Batterie-Chemie korrekt überprüfen: `board.bat`

### Einstellungen werden nicht gespeichert
1. Überprüfen, ob LittleFS korrekt gemountet ist
2. Ausreichenden Flash-Speicherplatz überprüfen
3. Logs auf Dateisystemfehler prüfen

## Lizenz

```
Copyright (c) 2026 Inhero GmbH
SPDX-License-Identifier: MIT
```

## Autor

Entwickelt für das MeshCore-Projekt von Inhero GmbH.

## Versionshistorie

### v0.2 (Januar 2026) 🆕
- **Hardware**: INA228 Power Monitor ersetzt MCP4652
- **RTC**: RV-3028-C7 Integration für Wake-up-Management
- **Coulomb Counter**: Echtzeit-Energie-Tracking und SOC-Berechnung
- **Tägliche Bilanz**: 7-Tage-Energie-Tracking mit Solar- vs. Batterie-Analyse
- **TTL-Prognose**: Batterielebensdauer basierend auf Nutzungsmustern vorhersagen
- **Auto-Learning**: Automatische Batteriekapazitätserkennung
- **Hardware-UVLO**: INA228 Alert → TPS EN für ultimativen Schutz
- **Power Management**: INA228 Shutdown-Modus während SYSTEMOFF (~1µA)
- **Neue CLI-Befehle**: `board.hwver`, `board.soc`, `board.balance`, `set board.batcap`

### v0.1 (Dezember 2025)
- Erstveröffentlichung
- BQ25798 Batteriemanagement mit MPPT
- MCP4652 Digitales Potentiometer für TP2120-Steuerung
- Multi-Chemie-Unterstützung (Li-Ion, LiFePO4, LTO)
- JEITA-Temperaturregelung
- 7-Tage-MPPT-Statistiken
- Telemetriesystem mit CayenneLPP
