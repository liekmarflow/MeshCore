# Inhero MR-2 Energieverwaltung - Implementierungs-Dokumentation

## Inhaltsverzeichnis

- [Überblick](#überblick)
- [Hardware-Architektur](#hardware-architektur)
- [1. Software-Monitoring (Adaptiv)](#1-software-monitoring-adaptiv)
- [2. Coulomb Counter & SOC (Ladezustand)](#2-coulomb-counter--soc-ladezustand)
- [3. Tägliche Energiebilanz](#3-tägliche-energiebilanz)
- [4. Solar-Energieverwaltung & Interrupt-Loop-Vermeidung](#4-solar-energieverwaltung--interrupt-loop-vermeidung)
  - [BQ25798 ADC bei niedrigen Batteriespannungen](#bq25798-adc-bei-niedrigen-batteriespannungen)
- [5. Time-To-Live (TTL)-Prognose](#5-time-to-live-ttl-prognose)
- [6. RTC-Wakeup-Management](#6-rtc-wakeup-management)
- [7. Energieverwaltungsablauf](#7-energieverwaltungsablauf)
- [Siehe auch](#siehe-auch)

> ✅ **STATUS: IMPLEMENTIERT (laufend gepflegt)** ✅
> 
> Diese Dokumentation beschreibt die vollständige Energieverwaltungs-Implementierung für das Inhero MR-2 Board.
> Hardware mit INA228 Power Monitor und RV-3028-C7 RTC ist funktional implementiert.
> Hinweis: Konkrete Zeilennummern können durch Refactorings abweichen.
> 
> Datum: 15. März 2026
> Version: 3.0 (Architektur-Refactoring: Flag/Tick-Pattern ersetzt I2C-Mutex + FreeRTOS-Tasks)
> Hardware: INA228 + RTC

---

## Überblick

Das System kombiniert **3 Schutz-Schichten** + **Coulomb Counter** + **tägliche Energiebilanz** + **CE-Pin Hardware-Ladesicherung** für Energie-Effizienz und Batterie-Monitoring:

1. **Software-Spannungsüberwachung** (in `tickPeriodic()`) - Danger-Zone-Erkennung
2. **RTC-Wakeup-Management** (RV-3028-C7) - periodische Recovery-Checks
3. **Hardware-UVLO** (INA228 Alert → TPS62840 EN) - ultimative Schutzebene
4. **Coulomb Counter** (INA228) - Echtzeit-SOC-Tracking
5. **Tägliche Energiebilanz** (7-Tage rolling) - Solar vs. Batterie
6. **INA228 Shutdown-Modus** - Stromsparen während Danger Zone (~1µA)
7. **BQ CE-Pin** (P0.04) - Hardware-Ladesicherung (Dual-Layer: GPIO + I2C)

### I2C-Architektur: Flag/Tick-Pattern

Alle I2C-Zugriffe (BQ25798, INA228, RV-3028, BME280) erfolgen ausschließlich aus dem Main-Loop-Kontext:

```
InheroMr2Board::tick()
  ├─ RTC TF-Flag clearen (bei rtc_irq_pending)
  ├─ BoardConfigContainer::tickPeriodic()
  │    ├─ Boot-Sequenz: sofortige Spannungsprüfung + 2s Recheck
  │    ├─ runMpptCycle()        ─ alle 60s (MPPT-Solarverwaltung)
  │    ├─ updateBatterySOC()    ─ alle 60s (Coulomb Counter)
  │    ├─ runVoltageMonitor()   ─ alle 60s (Unterspannungsschutz)
  │    └─ updateHourlyStats()   ─ alle 60min (Energiebilanz)
  └─ feedWatchdog()  ─ nur wenn tick() vollständig abgeschlossen
```

Da `tick()` den MeshCore-Core blockiert, kann während unserer I2C-Operationen kein paralleler I2C-Zugriff von MeshCore stattfinden — Bus-Contention ist **architektonisch unmöglich**. Ein früherer Mutex-Ansatz (`I2CMutex.h`) wurde entfernt, da er nur unseren Code schützte, nicht aber MeshCore's eigene I2C-Zugriffe.

**Watchdog-Strategie**: `feedWatchdog()` steht am Ende von `tick()`. Bleibt `tickPeriodic()` in einem I2C-Hang hängen, wird der WDT nicht gefüttert → automatischer Hardware-Reset.

**Verbleibende FreeRTOS-Tasks** (kein I2C):
- `heartbeatTask` — LED-Blinken (nur GPIO)
- Error-LED Lambda — Fehleranzeige (nur GPIO)

### Aktuelle Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| Spannungsüberwachung + Danger-Zone-Shutdown | Aktiv | Produktiv im Betrieb |
| Hardware-UVLO (INA228 Alert → TPS62840 EN) | Aktiv | Hardware-Schutz aktiv |
| RTC-Wakeup (Danger-Zone-Recovery) | Aktiv | 1h (stündlich) |
| SOC via INA228 + manuelle Batteriekapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-Ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |
| Auto-Learning (Method 1/2) | Deprecated | Aktuell nicht umgesetzt/aktiv |
| Erweiterte Auto-Learning-Reaktivierung | Geplant | Nur als zukünftige Aufgabe dokumentiert |

---

## Hardware-Architektur

### Komponenten
| Komponente | Funktion | I2C | Pin | Details |
|------------|----------|-----|-----|---------|
| **INA228** | Power Monitor | 0x45 | Alert→TPS_EN | 100mΩ Shunt, 1.6A max, Coulomb Counter |
| **RV-3028-C7** | RTC | 0x52 | INT→GPIO17 | Countdown-Timer, Wake-up |
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC (IBUS ~±30mA error at low currents; ADC hat VBAT-abhängige Schwellen, siehe [Abschnitt 4](#bq25798-adc-bei-niedrigen-batteriespannungen)) |
| **BQ CE-Pin** | Charge Enable | — | GPIO4 (P0.04) | Active LOW, ext. Pull-Up zu VSYS, Dual-Layer mit I2C |
| **TPS62840** | Buck Converter | - | EN←INA_Alert | 750mA, EN controlled by INA228 |

---

## 1. Software-Monitoring (Adaptiv)

### Implementierung
- **Scheduling**: `tickPeriodic()` im Main-Loop-Kontext (aufgerufen von `InheroMr2Board::tick()`)
- **Messung**: INA228 via I²C (batterie.voltage)
- **Frequenz**: 60s (millis()-basierter Timer)
- **Boot-Sequenz**: Sofortige Spannungsprüfung beim ersten `tickPeriodic()`-Aufruf, 2s später Recheck, dann 60s-Kadenz

### Monitoring-Intervalle

**Konfiguration:** Feste Intervalle
- **Normalbetrieb**: 60s
- **Danger Zone**: 1 Stunde (RTC-Wake)

**Two-Stage Strategy (Power-Optimized):**

| System State | Voltage (Li-Ion) | Interval | Cost per Check | Rationale |
|--------------|------------------|----------|----------------|----------|
| **Running (Normal)** | ≥ 3.4V | **60 seconds** | ~1µAh | System already awake, INA228 I²C read minimal cost |
| **System ON Idle (Danger Zone)** | < 3.4V | **1 hour** | ~0.01mAh | System ON Idle: GPIO-Latches aktiv, RTC-Wakeup, NVIC_SystemReset bei Erholung |

**Kernpunkt:** Checks im Normalmodus sind praktisch kostenlos (System läuft ohnehin). In der Danger Zone wird **System ON Idle** verwendet (statt SYSTEMOFF), um GPIO-Latches aktiv zu halten — insbesondere den BQ CE-Pin (LOW = Laden freigegeben). Die Spannung wird direkt im Idle-Loop via `readVBATDirect()` geprüft. Erst bei tatsächlicher Erholung erfolgt `NVIC_SystemReset()`.

**Action at < 3.4V**: `runVoltageMonitor()` → `board.initiateShutdown(LOW_VOLTAGE)` → System ON Idle + RTC timer 1h + NVIC_SystemReset bei Erholung

**Hinweis:** Die Implementierung befindet sich in `BoardConfigContainer::tickPeriodic()` → `runVoltageMonitor()` → `board.initiateShutdown()`.

### Chemie-spezifische Schwellen (2-Level System)

| Chemie | HW UVLO (Alert) | Critical (0% SOC) | Hysterese | ADC Averaging |
|--------|-----------------|-------------------|----------|---------------|
| **Li-Ion 1S** | 3.1V | 3.4V | +300mV | 64 samples (TX peak filtering) |
| **LiFePO4 1S** | 2.7V | 2.9V | +200mV | 64 samples (TX peak filtering) |
| **LTO 2S** | 3.9V | 4.2V | +300mV | 64 samples (TX peak filtering) |

**Implementierung**: `BoardConfigContainer.cpp` - Statische Methoden
- `getVoltageCriticalThreshold()` - Danger zone boundary, 0% SOC, software shutdown trigger
- `getVoltageHardwareCutoff()` - INA228 UVLO Alert threshold (hardware protection)

**Wrapper in InheroMr2Board.cpp**: Zeile 647-665
- Ruft BoardConfigContainer-Methoden auf mit aktueller Batterie-Chemie

---

## 2. Coulomb Counter & SOC (Ladezustand)

### INA228 Integration
- **Driver**: `lib/Ina228Driver.cpp` (255 Zeilen, vollständig implementiert)
- **Init**: `BoardConfigContainer::begin()` Zeile 592-641
  - 100mΩ Shunt-Kalibrierung
  - CURRENT_LSB = 1A / 524288 ≈ 1.91µA
  - ADC Range ±163.84mV (ADCRANGE=0, optimal für 1A @ 100mΩ)
  - **ADC Averaging**: 64 samples (filters TX voltage peaks, prevents false UVLO)
  - Chemie-spezifischer UVLO-Alert setzen

### SOC-Berechnung
**Methode**: `updateBatterySOC()` in `BoardConfigContainer.cpp` Zeile 1337-1418
- **Primary**: Coulomb Counting (INA228 CHARGE Register)
- **Fallback**: Voltage-based SOC via `estimateSOCFromVoltage()` (Zeile 1491-1541)
- **Update-Intervall**: `tickPeriodic()` ruft `updateBatterySOC()` auf (60s, Main-Loop-Kontext)

**Formel**:
```
SOC_delta = charge_delta_mah / capacity_mah × 100%
SOC_new = SOC_old + SOC_delta
```

**Auto-Learning** (deprecated, nicht aktiv):
- Code vorhanden in Zeile 1369-1388
- Trigger: BQ25798 "Charge Done" + Entladung bis Dangerzone
- Berechnet: capacity = accumulated_discharge_mah

### Kapazitäts-Management

#### Konfiguration erforderlich
Die Akkukapazität **muss manuell gesetzt werden**, da sie in der Praxis stark variiert:
- **Typischer Bereich**: 4000-24000mAh (4-24Ah)
- **CLI-Befehl**: `set board.batcap <mAh>`
- **Erlaubter Bereich**: 100-100000mAh

**Wichtig**: Ohne korrekte Kapazität sind SOC% und TTL-Berechnungen ungenau!

#### Persistenz-Mechanik
**Storage Path**: `/prefs/battery_capacity` (LittleFS)
**Save-Methode**: `saveBatteryCapacity()` in `BoardConfigContainer.cpp` Zeile 1256-1260
**Load-Methode**: `loadBatteryCapacity()` Zeile 1304-1329

**Speichern bei**:
1. **Manuelles Setzen**: CLI-Befehl `set board.batcap <mAh>`
   - Schreibt sofort in LittleFS
   - Aktualisiert `batteryStats.capacity_mah`
   
2. **Auto-Learning** (deprecated, nicht aktiv):
   - Trigger: BQ25798 "Charge Done" → Entladung bis Dangerzone
   - Berechnet neue Kapazität aus Coulomb Counter
   - Speichert automatisch via `saveBatteryCapacity()`

**Laden bei**:
- **Boot-Zeit**: `BoardConfigContainer::begin()` ruft `loadBatteryCapacity()` auf
- **Fallback**: Wenn keine gespeicherte Kapazität vorhanden
- **Validierung**: Range-Check 100-100000mAh

**Persistenz-Eigenschaften**:
- ✅ **Überlebt** Software-Shutdowns (SYSTEMOFF)
- ✅ **Überlebt** Hardware-UVLO (RAM-Verlust)
- ✅ **Überlebt** Power-Cycle
- ✅ **Überlebt** Firmware-Update (LittleFS bleibt erhalten)
- ⚠️ **Verloren** bei: Flash-Erase, `rm -rf /prefs/`, Filesystem-Korruption

---

## 3. Tägliche Energiebilanz

### Tracking (7-Day Rolling Window)
**Methode**: `updateDailyBalance()` in `BoardConfigContainer.cpp` Zeile 1420-1489
- **Aufgerufen von**: `tickPeriodic()` → `updateHourlyStats()` (alle 60 Minuten)
- **Frequenz**: Bei Tag-Wechsel (RTC time % 86400 < 60)

**Datenstruktur**: `BatterySOCStats.daily_stats[7]` (Zeile 74-89 in .h)
```cpp
struct DailyBatteryStats {
  uint32_t timestamp_day;       // Unix day start
  int32_t total_charge_mah;     // Summe Ladungen
  int32_t total_discharge_mah;  // Summe Entladungen
  int32_t net_balance_mah;      // = charge - discharge
  bool valid;                   // Daten vorhanden
};
```

### Berechnungen
**Net Balance**:
```
net_balance = today_charge - today_discharge
```

**7-Day Average Deficit** (für TTL):
```
avg_deficit = (day0 + day1 + ... + day6).net_balance / 7
```

**Living Status**:
- `living_on_battery = true` wenn `avg_deficit < 0` (Netto-Entladung)
- `living_on_solar = true` wenn `avg_deficit > 0` (Netto-Ladung)

---

## 4. Solar-Energieverwaltung & Interrupt-Loop-Vermeidung

### Problem: Interrupt Loop zwischen MPPT und BQ25798

**Root Cause**:
- BQ25798 setzt MPPT=0 automatisch wenn PowerGood=0 (kein Solar)
- `checkAndFixSolarLogic()` schreibt MPPT=1 zurück
- **Schreiben auf MPPT-Register triggert BQ25798 Interrupt**
- Interrupt-basiertes Aufwachen des MPPT-Handlers → erneuter `checkAndFixSolarLogic()`-Aufruf → schreibt MPPT → neuer Interrupt → **Loop**

**Solution: Cooldown Timer (60 Sekunden)**

**Implementierung**: `BoardConfigContainer.cpp` Zeile 69-73
```cpp
static uint32_t lastMpptWriteTime = 0;      // 60-second cooldown for MPPT register writes
static uint32_t lastHizToggleTime = 0;      // 5-minute cooldown for HIZ toggles
#define HIZ_TOGGLE_COOLDOWN_MS (5 * 60 * 1000)  // 5 minutes
```

### Stuck PGOOD Detection

**Problem**: Bei langsamen Sonnenaufgang (slow sunrise) kann BQ25798 input source nicht erkennen.

**Symptom**: 
- VBUS vorhanden (>3.5V)
- PowerGood bleibt bei 0
- PG_STAT flag nicht gesetzt (kein PGOOD-Wechsel erkannt)

**Solution: HIZ Toggle**

**Implementierung**: `checkAndFixPgoodStuck()` in `BoardConfigContainer.cpp` Zeile 228-305

**Trigger-Bedingungen** (alle müssen erfüllt sein):
1. PowerGood = 0 (stuck low)
2. VBUS > 3.5V (Solar vorhanden)
3. PG_STAT flag = 0 (keine Änderung erkannt)
4. Cooldown abgelaufen (>5 Minuten seit letztem Toggle)

**Ablauf**:
1. Toggle HIZ: `setHIZMode(true)` → `setHIZMode(false)`
2. Zwingt BQ25798 zu input source qualification
3. **Resettet MPPT-Cooldown** (`lastMpptWriteTime = 0`)
4. Erlaubt sofortiges MPPT=1 schreiben wenn PG=1 wird
5. Setzt HIZ-Cooldown-Timer (5 Minuten)

### MPPT Recovery

**Problem**: BQ25798 deaktiviert MPPT automatisch bei PG=0.

**Solution: Conditional MPPT Re-enable**

**Implementierung**: `checkAndFixSolarLogic()` in `BoardConfigContainer.cpp` Zeile 307-363

**Kritische Bedingung**: **Nur wenn PowerGood=1**
- Verhindert false positives
- Verhindert Interrupt Loop bei instabilem Solar

**Cooldown-Logik**:
- 60 Sekunden zwischen MPPT-Writes
- **Exception**: Cooldown wird resettet nach HIZ toggle
- Nur schreiben wenn tatsächliche Änderung nötig (readback check)

**Ablauf**:
```
1. PowerGood=1? → JA
2. Cooldown abgelaufen? → JA
3. MPPT aktuell=0? → JA
4. Schreibe MPPT=1
5. Setze lastMpptWriteTime
```

### Interrupt Handling

**Problem**: BQ25798 Interrupts müssen acknowledged werden, sonst Lockup.

**Solution: Always Clear Flags**

**Implementierung**: `onBqInterrupt()` in `BoardConfigContainer.cpp` Zeile 404-420

**KRITISCH**: **Immer** Register 0x1B lesen, auch wenn Event ignoriert wird.

```cpp
// BQ25798 Interrupt-Flags werden in runMpptCycle() per Polling geclearet:
// bqDriverInstance->readReg(0x1B); // Clear interrupt flags
// Kein ISR-Handler oder Semaphore mehr nötig — polling-basiert im tick()-Kontext.
```

**Flag Clearing auf Boot**: `BoardConfigContainer::configureBq()` Zeile 1075
- Liest FAULT_STATUS Register (0x20, 0x21) nach Konfiguration
- Vermeidet stale faults von vorherigem Power-Cycle

### Tick-basierte Architektur

`runMpptCycle()` wird alle 60s von `tickPeriodic()` aufgerufen (Main-Loop-Kontext):
```
tickPeriodic() — millis()-basierter Timer (60s)
  └─ runMpptCycle()
       ├─ Interrupt-Flags clearen (0x1B)
       ├─ Panel-Klassifikation (UNKNOWN/LOW_V/HIGH_V)
       ├─ LOW_V: checkParasiticDischarge() + checkAndFixPgoodStuck() + checkAndFixSolarLogic()
       └─ HIGH_V: HIZ-Gated State Machine (HIZ_IDLE/CHARGE_ACTIVE)
```

**Timing Summary**:
- **MPPT Write Cooldown**: 60 Sekunden (verhindert Loop)
- **HIZ Toggle Cooldown**: 5 Minuten (verhindert excessive toggling)
- **Periodic Check**: 60 Sekunden (via `tickPeriodic()`)
- **Interrupt Response**: N/A (kein Interrupt-Trigger mehr, rein polling-basiert)

### BQ25798 ADC bei niedrigen Batteriespannungen

> **Referenz:** BQ25798 Datasheet (TI SLUSE22), Section 9.3.16 — ADC

#### Problem

Der 15-bit ADC im BQ25798 hat **spannungsabhängige Betriebsschwellen**, die im reinen Batteriebetrieb (ohne Solar) relevant werden. Bei niedrigen Batteriespannungen kann der ADC seine Konvertierung nicht abschließen — `ADC_EN` bleibt gesetzt und die Firmware läuft in einen Timeout.

#### Datasheet-Zitat (Section 9.3.16)

> *"The ADC is allowed to operate if either VBUS > 3.4V or VBAT > 2.9V is valid.
> At battery only condition, if the TS_ADC channel is enabled, the ADC only works
> when battery voltage is higher than 3.2V, otherwise, the ADC works when the
> battery voltage is higher than 2.9V."*

#### Betriebsszenarien

| Bedingung | VBUS | VBAT | TS-Kanal | ADC | Temperatur |
|-----------|------|------|----------|-----|------------|
| Solar angeschlossen | > 3.4V | beliebig | aktiviert | ✅ läuft | ✅ verfügbar |
| Batteriebetrieb, normal | — | ≥ 3.2V | aktiviert | ✅ läuft | ✅ verfügbar |
| Batteriebetrieb, niedrig | — | 2.9–3.2V | **deaktiviert** | ✅ läuft | ❌ nicht verfügbar |
| Batteriebetrieb, kritisch | — | < 2.9V | deaktiviert | ❌ Timeout | ❌ nicht verfügbar |

#### Firmware-Lösung: VBAT-abhängige TS-Kanal-Steuerung

Die Firmware liest die aktuelle Batteriespannung vom INA228 und übergibt sie an `BqDriver::getTelemetryData(vbat_mv)`:

- **VBAT ≥ 3.2V** (oder unbekannt): TS-Kanal aktiviert → ADC-Schwelle 3.2V, Temperatur verfügbar
- **VBAT < 3.2V**: TS-Kanal deaktiviert → ADC-Schwelle sinkt auf 2.9V, Temperatur als "N/A" angezeigt

Dadurch funktioniert der ADC im Bereich 2.9–3.2V weiterhin für Solar-Messungen (VBUS, IBUS), auch wenn die Batterietemperatur nicht gelesen werden kann.

#### ADC-Kanal-Konfiguration (nur benötigte Kanäle)

Auf dem MR-2 sind D+, D−, VAC1, VAC2 nicht verbunden. Die Firmware aktiviert nur die tatsächlich genutzten Kanäle:

| Register | Wert (TS ein) | Wert (TS aus) | Aktive Kanäle |
|----------|---------------|---------------|---------------|
| 0x2F (ADC_FUNCTION_DISABLE_0) | `0x5A` | `0x5E` | IBUS, VBUS, (TS) |
| 0x30 (ADC_FUNCTION_DISABLE_1) | `0xF0` | `0xF0` | keine (D+/D−/VAC disabled) |

**Wichtig:** Im One-Shot-Modus wird `ADC_EN` erst gelöscht, wenn **alle aktivierten Kanäle** fertig konvertiert haben. Nicht-verbundene Kanäle können dies blockieren → deshalb werden nur die benötigten Kanäle aktiviert.

#### Temperatur-Sentinel-Werte

Die Firmware verwendet spezielle Rückgabewerte für ungültige Temperaturen:

| Wert | Bedeutung | Anzeige |
|------|-----------|--------|
| −999.0 | I2C-Kommunikationsfehler | N/A |
| −888.0 | ADC nicht bereit / TS deaktiviert (niedriges VBAT) | N/A |
| −99.0 | NTC offen/nicht angeschlossen | N/A |
| +99.0 | NTC Kurzschluss | N/A |
| −50…+90°C | Gültiger Messwert | XX°C |

**Anzeigeregel:** Werte ≤ −100°C werden in der CLI als "N/A" angezeigt und in CayenneLPP-Paketen weggelassen.

#### Code-Referenzen
- `BqDriver::getTelemetryData(vbat_mv)` — Hauptfunktion mit VBAT-abhängiger TS-Steuerung
- `BqDriver::startADCOneShot(ts_enabled)` — Konfiguriert ADC-Kanäle und startet Konvertierung
- `BoardConfigContainer::getTelemetryData()` — Übergibt INA228-VBAT an BqDriver

---

## 5. Time-To-Live (TTL)-Prognose

### Datenbasis und Zeitbasis

Die TTL-Berechnung basiert auf dem **7-Tage gleitenden Durchschnitt** des täglichen Netto-Energieverbrauchs, der aus einem **168-Stunden-Ringpuffer** (7 Tage) stündlicher INA228-Coulomb-Counter-Messungen berechnet wird.

#### Datenfluss

```
INA228 Hardware Coulomb Counter (24-bit ADC, ±0.1% Genauigkeit)
        │
        ▼
updateHourlyStats() — jede Stunde
        │  Speichert pro Stunde: charged_mah, discharged_mah, solar_mah
        │  in hours[168] Ringpuffer (BatterySOCStats.hours[])
        ▼
calculateRollingStats() — nach jedem Stunden-Update
        │  Summiert letzte 168 Stunden → teilt durch 7
        │  → avg_7day_daily_net_mah (= solar − discharged pro Tag)
        │  Mindestvoraussetzung: ≥ 24 Stunden gültige Daten
        ▼
calculateTTL() — nach calculateRollingStats()
        │  remaining_mah / |deficit_per_day| × 24 = TTL Stunden
        ▼
socStats.ttl_hours → getTTL_Hours() → board.stats / Telemetrie
```

### Berechnung
**Methode**: `calculateTTL()` in `BoardConfigContainer.cpp`
- **Aufgerufen**: Nach `calculateRollingStats()` (stündlich)
- **Zeitbasis**: 7-Tage gleitender Durchschnitt (`avg_7day_daily_net_mah`) aus stündlichen Samples

**Voraussetzungen für TTL > 0**:
1. `living_on_battery == true` (24h-Netto ist negativ, d.h. Energiedefizit)
2. `avg_7day_daily_net_mah < 0` (7-Tage-Durchschnitt zeigt Netto-Entladung)
3. `capacity_mah > 0` (Batteriekapazität bekannt, via `set board.batcap`)
4. Mindestens **24 Stunden** gültige Daten im Ringpuffer

**Formel**:
```
remaining_capacity_mah = (SOC% / 100) × capacity_mah
daily_deficit_mah = -avg_7day_daily_net_mah  (positiver Wert)
TTL_hours = remaining_capacity_mah / daily_deficit_mah × 24
```

**TTL = 0 bedeutet**:
- Gerät wird solar versorgt (Netto-Überschuss) → `living_on_battery == false`
- Noch keine 24h Daten gesammelt (Kaltstart)
- Batteriekapazität unbekannt

**Infinite TTL (Telemetrie)**:
- Wenn `living_on_battery == false` und SOC valide → wird als 990 Tage (Max-Wert) übertragen

**Beispiel**:
- SOC: 60% = 1200mAh remaining (bei 2000mAh Kapazität)
- 7-day avg: -100 mAh/day (aus 168h Stunden-Samples)
- TTL: 1200 / 100 × 24 = 288 Stunden = 12 Tage

**CLI-Ausgabe**: `board.stats`
```
+150.0/+120.0/+90.0mAh SOL M:85%           ← Solar-Überschuss, keine TTL
```
oder
```
-80.0/-100.0/-110.0mAh BAT M:45% TTL:288h  ← 288h bis leer (7d-Avg-Basis)
```

---

## 6. RTC-Wakeup-Management

### RV-3028-C7 Integration
**Pin**: GPIO17 (WB_IO1) → RTC INT
**Init**: `InheroMr2Board::begin()` Zeile 459+
- `attachInterrupt(RTC_INT_PIN, rtcInterruptHandler, FALLING)`
- Prüft `GPREGRET2` für den Wake-up-Grund

### Countdown-Timer Konfiguration
**Methode**: `configureRTCWake()` in `InheroMr2Board.cpp`
- **Tick Rate**: 1/60 Hz (1 Minute pro Tick), konfiguriert via TD=11 in CTRL1
- **Max Countdown**: 65535 Minuten ≈ 45 Tage
- **Danger Zone Interval**: `DANGER_ZONE_SLEEP_MINUTES` = 60 min (1h)
- **Rationale**: Jeder Wake kostet nur ~0.03 mAh (I2C-Read im Idle-Loop, KEIN vollständiger Reboot)

**Register**:
```cpp
RV3028_CTRL1 (0x00):     TE=1, TD=11 (1/60 Hz), TRPT=0 (Single shot)
RV3028_CTRL2 (0x10):     TIE=1 (Timer Interrupt Enable, bit 4)
RV3028_STATUS (0x0E):    TF (Timer Flag, bit 3) — nach Wake clearen!
RV3028_TIMER_VALUE_0 (0x0A): Countdown value LSB
RV3028_TIMER_VALUE_1 (0x0B): Countdown value MSB (upper 4 bits)
```

### Interrupt Handler
**Methode**: `rtcInterruptHandler()` — setzt nur `rtc_irq_pending = true`.

Der eigentliche TF-Clear passiert im Idle-Loop per I2C:
```cpp
// In initiateShutdown() Idle-Loop:
Wire.beginTransmission(RTC_I2C_ADDR);
Wire.write(RV3028_REG_STATUS);
Wire.write(0x00);  // Clear TF → INT pin geht via Pull-Up wieder HIGH
Wire.endTransmission();
```

**Warum nicht in der ISR?** I2C (Wire) darf nicht aus einem ISR-Kontext aufgerufen werden.
Der ISR setzt nur das Flag `rtc_irq_pending`, `tick()` prüft es und cleart TF im Main-Loop-Kontext.

---

## 7. Energieverwaltungsablauf

### Shutdown-Sequenz
**Methode**: `initiateShutdown()` in `InheroMr2Board.cpp`

**Bei Low-Voltage → System ON Idle** (GPIO-Latches bleiben aktiv für CE-Pin):

**Ablauf:** `runVoltageMonitor()` (BoardConfigContainer.cpp) ruft `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)` auf (InheroMr2Board.cpp):

1. **Stop Background Tasks**: `BoardConfigContainer::stopBackgroundTasks()`
   - Stoppt Heartbeat task (einziger verbleibender FreeRTOS-Task mit I/O)
   - MPPT und SOC-Arbeit laufen in `tickPeriodic()` und werden durch den Shutdown-Flow natürlich gestoppt
   - Verhindert Filesystem-Korruption
   
2. **INA228 Shutdown**: `ina228.shutdown()` → MODE=0x0 (Power-down, ~1µA)
   
3. **SX1262 Sleep**: `radio_driver.powerOff()` → `radio.sleep(false)` (Cold Sleep via SPI, ~0.16µA)
   - MUSS vor PE4259-Abschaltung passieren — SPI braucht stabile Stromversorgung
   - Ohne diesen Schritt: SX1262 bleibt im RX-Modus (~5mA!)

4. **PE4259 RF-Switch aus**: `digitalWrite(SX126X_POWER_EN, LOW)` (VDD abschalten)
   
5. **LEDs aus**: PIN_LED1, PIN_LED2 LOW
   
6. **RTC Wake konfigurieren**: `configureRTCWake(DANGER_ZONE_SLEEP_MINUTES)`
   
7. **Shutdown-Grund speichern**: `NRF_POWER->GPREGRET2 = GPREGRET2_IN_DANGER_ZONE | reason`
   
8. **BQ Interrupt deaktivieren**: `detachInterrupt(BQ_INT_PIN)` — verhindert Spurious-Wakeups

9. **SysTick deaktivieren**: `SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk`
   - Ohne dies: SysTick feuert alle 1ms → FreeRTOS scheduled weiter `loopTask` (Mesh, Radio, BLE)
   
10. **SoftDevice (BLE) deaktivieren**: `sd_softdevice_disable()`
   - SoftDevice hält BLE-Radio aktiv (~2-3mA Advertising/Scanning) selbst bei deaktiviertem SysTick
   - Nach Deaktivierung: `sd_app_evt_wait()` nicht mehr verfügbar → `__WFI()` direkt nutzen
   - GPIOTE vollständig unter eigener Kontrolle → `attachInterrupt()` funktioniert zuverlässig
   - Wire/I2C (TWIM Peripheral) funktioniert weiterhin — keine SoftDevice-Abhängigkeit
   - GPIO-Latches bleiben erhalten (CE-Pin bleibt LOW)

11. **GPIOTE Interrupt re-aktivieren**: `attachInterrupt(RTC_INT_PIN, rtcInterruptHandler, FALLING)`
   - Erst NACH `sd_softdevice_disable()` — vorher besitzt SoftDevice den GPIOTE-Dispatcher

12. **System ON Idle Loop** (Hauptschleife):
   ```cpp
   while (true) {
     rtc_irq_pending = false;
     while (!rtc_irq_pending)
       __WFI();                 // CPU schläft bis GPIOTE-Interrupt
     // TF-Flag clearen → INT-Pin geht wieder HIGH
     Wire.write(RV3028_REG_STATUS, 0x00);
     vbat_mv = readVBATDirect();  // INA228 One-Shot via I2C
     if (vbat_mv >= critical)
       NVIC_SystemReset();        // Voltage OK → normaler Reboot
     configureRTCWake(MINUTES);   // Sonst weiterschlafen
   }
   ```
   - **`__WFI`** (Wait For Interrupt) — NICHT `__WFE` (Wait For Event)!
     `__WFE` wacht nur auf SEV-Instruktionen auf, nicht auf NVIC-Interrupts
   - GPIO-Latches bleiben aktiv → BQ_CE_PIN bleibt LOW → Solar-Laden möglich
   - Stromverbrauch: **~0.6mA** gemessen (nRF52840 System ON Idle + I2C Peripherals)
   - Kein vollständiger Reboot nötig — Spannungsprüfung direkt im Loop

**Nicht-Low-Voltage Shutdowns** (User Request, Thermal): Verwenden `sd_power_system_off()` (echtes System OFF).

**Warum System ON Idle statt System OFF?**
- System OFF → alle GPIOs werden High-Z → CE-Pin Pull-Up → HIGH → Laden gesperrt
- System ON Idle → GPIO-Latches bleiben aktiv → CE-Pin bleibt LOW → **Solar-Laden weiterhin möglich**
- Stromverbrauch: **~0.6mA gemessen** (System ON Idle) vs. ~2µA (System OFF)
- Die ~0.6mA setzen sich zusammen aus: nRF52840 Idle (~0.4mA mit I2C-Peripherals) + RV-3028 + INA228 Shutdown + SX1262 Cold Sleep

**Warum SoftDevice deaktivieren?**
- SoftDevice (BLE stack) hält das 2.4GHz Radio aktiv (~2-3mA Advertising) auch bei deaktiviertem SysTick
- `sd_app_evt_wait()` blockiert nur SoftDevice-Events — GPIOTE-Events werden nicht durchgereicht
- Nach `sd_softdevice_disable()`: GPIOTE funktioniert zuverlässig, `__WFI()` wacht auf Interrupts auf
- `NVIC_SystemReset()` bei Recovery initialisiert SoftDevice komplett neu — kein Problem

**Warum `__WFI()` statt `__WFE()`?**
- `__WFE` (Wait For Event) wacht nur auf SEV-Instruktionen und Debug-Events auf — GPIOTE-Interrupts sind KEINE Events
- `__WFI` (Wait For Interrupt) wacht auf jeden aktivierten NVIC-Interrupt auf, inkl. GPIOTE
- Beide haben identischen Stromverbrauch im Idle-Modus

**168h-Statistiken gehen bei `NVIC_SystemReset()` verloren** — es existiert kein Persistenzmechanismus
(kein `.noinit`-Section, kein LittleFS-Snapshot) für die Ring-Buffer-Daten. Nach System ON Idle → NVIC_SystemReset
starten die Statistiken bei Null.

### Wake-up-Check (Anti-Motorboating)
**Methode**: `InheroMr2Board::begin()`

Der Code prüft `GPREGRET2` für den Shutdown-Grund und die Batteriespannung für Wake-up-Entscheidungen.

**Problem**: Nach Hardware-UVLO ist `GPREGRET2 = 0x00` (kompletter RAM-Verlust)
→ Ohne universelle Voltage Check: Motorboating bei knapper Spannung

**3 Fälle**:

**Case 1: Wake from Danger Zone (System ON Idle → NVIC_SystemReset)** (`GPREGRET2 = SHUTDOWN_REASON_LOW_VOLTAGE`)
```cpp
// InheroMr2Board::begin() — Voltage wurde bereits im Idle-Loop geprüft
if ((shutdown_reason & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE) {
  // Flags löschen, Danger-Zone-Recovery markieren, SOC auf 0%
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
  boardConfig.setDangerZoneRecovery();
}
```

**Case 2: ColdBoot after Hardware-UVLO** (`GPREGRET2` beliebig, voltage still low)
```cpp
// Prüft auch bei Cold Boot die Spannung
else if (vbat_mv < critical_threshold) {
  configureRTCWake(DANGER_ZONE_SLEEP_MINUTES);  // 1h
  NRF_POWER->GPREGRET2 = GPREGRET2_IN_DANGER_ZONE | SHUTDOWN_REASON_LOW_VOLTAGE;
  sd_power_system_off();
}
```
**CRITICAL**: Verhindert Motorboating! Auch nach Hardware-Cutoff wird geprüft.

**Case 3: Normal ColdBoot** (Power-On, Reset button, voltage OK)
```cpp
else {
  // Continue normal boot
  // INA228 und alle anderen Komponenten werden initialisiert
}
```

**Direct ADC Read** (boardConfig noch nicht ready):
```cpp
// RAK4630 cannot measure battery voltage - there's no voltage divider on GPIO!
// Must read directly from INA228 ADC registers (24-bit, ±0.1% accuracy)
uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, 0x40);

// Ina228Driver::readVBATDirect() - Static method in lib/Ina228Driver.cpp
// Uses INA228 One-Shot ADC for fresh, accurate measurement
```


**Voltage Thresholds** (Chemistry-Specific, 2-Level System):
| Chemistry | Hardware UVLO (Alert) | Critical (0% SOC) | Hysteresis | ADC Filter |
|-----------|-----------------------|-------------------|------------|------------|
| Li-Ion 1S | 3100mV (INA228 Alert) | 3400mV | 300mV | 64-sample avg (TX peaks) |
| LiFePO4 1S | 2700mV (INA228 Alert) | 2900mV | 200mV | 64-sample avg (TX peaks) |
| LTO 2S | 3900mV (INA228 Alert) | 4200mV | 300mV | 64-sample avg (TX peaks) |

**UVLO-Verhalten** (Li-Ion-Beispiel):
1. Hardware UVLO @ 3.1V → INA228 Alert → TPS62840 EN=LOW → RAK stromlos
2. INA228 ALERT ist **latched** → TPS62840 EN bleibt LOW, auch wenn VBAT wieder steigt
3. RAK bleibt permanent aus → kein automatischer Neustart möglich
4. **Manuelles Eingreifen erforderlich** (Akkutausch/Reset → INA228 Latch wird zurückgesetzt)

**Anti-Motorboating nach manuellem Reset** (ColdBoot Case 2):
Nach manuellem Reset (z.B. Akku getauscht) bootet der RAK mit `GPREGRET2 = 0x00`.
1. `begin()` → Early Boot Check → `vbat < critical_threshold` ✅ DETECTED
2. **Action** → `configureRTCWake(DANGER_ZONE_SLEEP_MINUTES)` + `GPREGRET2 = LOW_VOLTAGE` + `sd_power_system_off()`
3. **Result** → Kein Motorboating! System geht in System OFF (RTC weckt in 1h)
   *Hinweis: Dieser Pfad (ColdBoot, Case 2) nutzt System OFF — nur `initiateShutdown()` nutzt System ON Idle*

**Stromverbrauch während System ON Idle (Danger Zone, gemessen)**:
- **Gesamt: ~0.6mA** (gemessen an VBAT via INA228)
- Davon: nRF52840 System ON Idle mit I2C-Peripherals (~0.4mA), SX1262 Cold Sleep (~0.16µA), RV-3028 (~45nA), INA228 Shutdown (~1µA), BQ25798 Quiescent (~30µA)
- SoftDevice (BLE) ist deaktiviert — spart ~2-3mA gegenüber vorheriger Implementierung
- SysTick ist deaktiviert — FreeRTOS Scheduler läuft nicht
- **Vorteil**: GPIO-Latches aktiv → BQ CE-Pin bleibt LOW → Solar-Laden möglich

---

## 8. Hardware UVLO (INA228 → TPS62840)

### Funktionsweise
**Alert-Pin Verdrahtung**: INA228.ALERT → TPS62840.EN (direkt)
**Config**: `BoardConfigContainer::begin()` Zeile 614-620

**Schwellenwerte** (with safety margins below software Critical thresholds):
```cpp
switch (batteryType) {
  case LTO_2S:      uvlo_mv = 3900; break;  // 3.9V (300mV margin)
  case LIFEPO4_1S:  uvlo_mv = 2700; break;  // 2.7V (200mV margin)
  case LIION_1S:    uvlo_mv = 3100; break;  // 3.1V (300mV margin, default)
}

ina228.setUnderVoltageAlert(uvlo_mv);
ina228.enableAlert(true, false, true);  // Nur UVLO, active-high
```

**INA228 Alert Register** (Ina228Driver.cpp Zeile 200-219):
- BUVL (Bus Under-Voltage Limit): `voltage_mv / 195.3125µV`
- DIAG_ALRT: Enable BUSUL bit (bit 3)
- APOL (Alert Polarity): Active-HIGH (TPS EN = HIGH im Normalfall)

**Hardware-Verhalten**:
- Initial (Power-on): ALERT=HIGH → TPS62840 EN=HIGH → RAK powered
- VBAT < UVLO → ALERT=LOW → TPS62840 EN=LOW → RAK stromlos (0µA)
- ALERT ist **latched** → bleibt LOW auch wenn VBAT wieder steigt → kein automatischer Neustart
- Reset nur durch INA228 Power-Cycle (Akkutausch) oder I²C Register-Read

**Schutzschichten** (Li-Ion example):
1. Software Critical (3.4V / 0% SOC) → Controlled shutdown, danger zone boundary
2. Hardware UVLO (3.1V) → Emergency cutoff via INA228 Alert
3. Hysterese (300mV) → Verhindert Motorboating zwischen UVLO und Critical
4. ADC Filter (64-sample avg) → Filtert TX-Peaks (~100ms), verhindert false triggers

---

## 9. SX1262 Power Control & PE4259 RF-Switch

### Hardware-Architektur
- **SX1262**: LoRa Transceiver (SPI-Bus), Sleep-Mode via `SetSleep` SPI-Befehl
- **PE4259**: SPDT RF-Antennenweiche im **Single-Pin-Modus**:
  - **Pin 6 (VDD)**: GPIO 37 (P1.05, `SX126X_POWER_EN`) — Stromversorgung (muss HIGH sein für Betrieb)
  - **Pin 4 (CTRL)**: SX1262 DIO2 — TX/RX Umschaltung (automatisch via `setDio2AsRfSwitch(true)`)

### Shutdown-Sequenz (in `initiateShutdown()`)
Die SX1262 wird in **zwei Schritten** abgeschaltet — **Reihenfolge ist kritisch**:

```cpp
// Schritt 1: SX1262 in Cold Sleep via SPI (MUSS zuerst!)
radio_driver.powerOff();  // → radio.sleep(false) → SPI SetSleep command
delay(10);

// Schritt 2: PE4259 RF-Switch Stromversorgung abschalten
digitalWrite(SX126X_POWER_EN, LOW);  // VDD weg → PE4259 aus
```

**Warum diese Reihenfolge?**
- `radio.sleep(false)` sendet einen SPI-Befehl an den SX1262 → SPI braucht stabile Stromversorgung
- PE4259 VDD (GPIO 37) versorgt den RF-Switch, NICHT den SX1262 direkt
- Wenn PE4259 zuerst abgeschaltet wird, ist SPI noch möglich, aber der RF-Switch ist bereits aus
- Sicherheitshalber: Erst SX1262 schlafen legen, dann PE4259 abschalten

### Boot-Sequenz (in `begin()`)
```cpp
// PE4259 VDD einschalten → RF-Switch betriebsbereit
pinMode(SX126X_POWER_EN, OUTPUT);
digitalWrite(SX126X_POWER_EN, HIGH);
delay(10);  // PE4259 Einschaltzeit

// Später in radio_init() → target.cpp:
radio.std_init(&SPI);  // → setDio2AsRfSwitch(true) → DIO2 steuert TX/RX
```

**Wichtige Details**:
- **`SX126X_POWER_EN`** (GPIO 37 / P1.05) steuert die **PE4259 VDD**, NICHT die SX1262 Power
- **`DIO2`** wird intern vom SX1262 gesteuert (`setDio2AsRfSwitch(true)`) — kein GPIO nötig
- **Sleep-Strom SX1262**: ~0.16µA (Cold Sleep) — Datenblatt-Wert
- **Ohne `radio_driver.powerOff()`**: SX1262 bleibt im RX-Modus → ~5mA Stromverbrauch!

---

## 10. UVLO CLI Integration

### Persistent UVLO Setting
**Dauerhafte Speicherung**: LittleFS preferences (`/prefs/uvloEn`)

**Funktionen**:
- `loadUvloEnabled()` - Lädt Einstellung beim Boot (Default: DISABLED für Feldtests)
- `setUvloEnabled(bool)` - Setzt Einstellung und speichert persistent
- `getUvloEnabled()` - Gibt aktuelle Einstellung zurück
- `applyUvloSetting()` - Wendet chemie-spezifische UVLO-Schwellen an

**Automatische Anwendung**:
- Bei `BoardConfigContainer::begin()` (Boot)
- Bei `setBatteryType()` (Batteriewechsel)

**Chemie-spezifische Schwellen** (in `applyUvloSetting()`):
| Chemie | Hardware UVLO (Alert) |
|--------|----------------------|
| **Li-Ion 1S** (default) | 3.1V |
| **LiFePO4 1S** | 2.7V |
| **LTO 2S** | 3.9V |

### CLI-Integration
**Getter**: `board.uvlo`
```bash
board.uvlo  # UVLO-Status abfragen
            # Output: "ENABLED" oder "DISABLED"
            # Code: InheroMr2Board.cpp getCustomGetter() Zeile 486
```

**Setter**: `set board.uvlo`
```bash
set board.uvlo 0|1|false|true  # UVLO-Einstellung setzen
                               # 0/false = DISABLED (Standard für Feldtests)
                               # 1/true = ENABLED (für kritische Anwendungen)
                               # Wird persistent gespeichert
                               # Code: InheroMr2Board.cpp setCustomSetter() Zeile 643
```

**Persistenz-Verhalten**:
- ✅ Überlebt SYSTEMOFF
- ✅ Überlebt Hardware-UVLO (RAM-Verlust)
- ✅ Überlebt Firmware-Update (LittleFS bleibt)
- Geladen beim Boot bei `BoardConfigContainer::begin()`

---

## 11. BQ25798 CE-Pin Safety (Hardware-Ladesicherung)

### Problem
Der BQ25798 startet mit Default-Konfiguration (1S Li-Ion, 4.2V Ladespannung). Wenn eine LiFePO4-Batterie (3.5V max) verbunden ist und der RAK noch nicht gebootet hat, würde der BQ25798 die Batterie überladen → **Brandgefahr**.

### Hardware-Design
- **Pin**: `BQ_CE_PIN` = GPIO 4 (P0.04 / WB_IO4)
- **Externer Pull-Up**: 10kΩ zu VSYS → CE HIGH = Laden gesperrt (sicherer Default)
- **Aktive Logik**: LOW = Laden freigegeben, HIGH = Laden gesperrt

### 3-Schicht-Sicherung

| Schicht | Ort | Mechanismus | Wann |
|---|---|---|---|
| **1. Hardware (passiv)** | Pull-Up Widerstand | CE HIGH wenn RAK stromlos | Immer |
| **2. Early Boot** | `InheroMr2Board::begin()` | `digitalWrite(BQ_CE_PIN, HIGH)` | Vor I2C-Init |
| **3. Chemie-Konfiguration** | `configureChemistry()` | CE LOW nur bei bekannter Chemie | Nach BQ25798-Konfiguration |

### Dual-Layer Safety (Hardware + Software)

```cpp
// In configureChemistry() — nach BQ25798 Register-Konfiguration:
bq.setChargeEnable(props->charge_enable);     // Software-Schicht (I2C Register)
#ifdef BQ_CE_PIN
  pinMode(BQ_CE_PIN, OUTPUT);
  digitalWrite(BQ_CE_PIN, props->charge_enable ? LOW : HIGH);  // Hardware-Schicht
#endif
```

- `charge_enable` ist Teil der `BatteryProperties`-Tabelle
- `BAT_UNKNOWN` → `charge_enable = false` → CE HIGH + Register disabled
- Bekannte Chemie → `charge_enable = true` → CE LOW + Register enabled

### Verhalten im System ON Idle (Danger Zone)

In der Danger Zone wird **System ON Idle** verwendet (via `initiateShutdown()`):
- SoftDevice (BLE) wird deaktiviert → spart ~2-3mA
- SysTick wird deaktiviert → FreeRTOS Scheduler stoppt
- SX1262 wird per SPI in Cold Sleep versetzt → ~0.16µA
- PE4259 VDD wird abgeschaltet → RF-Switch stromlos
- System ON Idle → GPIO-Latches bleiben aktiv → **CE-Pin bleibt LOW**
- BQ25798 MPPT/CC/CV läuft autonom in Hardware → Solar-Laden möglich
- CPU wacht nur bei GPIOTE-Interrupt (RTC Timer) auf → `__WFI()`
- Gemessener Gesamtstromverbrauch: **~0.6mA**

| Zustand | CE-Pin | Laden | Solar-Recovery |
|---|---|---|---|
| RAK stromlos | HIGH (Pull-Up) | Gesperrt | Nein |
| Early Boot | HIGH (explizit) | Gesperrt | Nein |
| BAT_UNKNOWN | HIGH | Gesperrt | Nein |
| Chemie konfiguriert | LOW | Freigegeben | Ja |
| System ON Idle (Danger Zone) | LOW (gelatcht) | **Freigegeben** | **Ja** |
| System OFF (ColdBoot Case 2) | HIGH (Pull-Up) | Gesperrt | Nein (ALERT latched, manueller Reset nötig) |

---

## 12. Statistik-Persistenz

### Aktueller Stand

Die 168h-Ringpuffer-Statistiken (Coulomb Counter, MPPT-Daten, SOC-Zustand) sind **nur im RAM** gespeichert und gehen bei jedem Reboot verloren — egal ob System OFF, Hardware-UVLO oder Cold Boot. Es existiert kein Persistenzmechanismus (weder `.noinit`-Section noch LittleFS-Snapshot).

**Persistente Daten** (überleben Reboots via LittleFS):
- Batterietyp (`batType`)
- Batteriekapazität (`batCap`)
- INA228 Kalibrierung (`ina228Cal`, `ina228Off`)
- NTC Kalibrierung (`tcCal`)
- UVLO-Einstellung (`uvloEn`)
- MPPT-Einstellung (`mpptEn`)
- Frostverhalten (`frost`)
- Max. Ladestrom (`maxChrg`)
- LED-Einstellung (`leds`)

**Nicht-persistente Daten** (gehen bei Reboot verloren):
- 168h Energie-Ringpuffer (stündliche Charge/Discharge/Solar mAh)
- MPPT-Statistiken (168h MPPT-Aktivitätsbuffer)
- SOC-Prozentwert (wird nach Recovery auf 0% gesetzt, bei "Charging Done" auf 100% synchronisiert)
- TTL-Berechnung (benötigt mind. 24h Daten nach jedem Neustart)
- Tägliche Energiebilanz (7-Tage-Fenster baut sich nach Neustart neu auf)

---

## 13. CLI-Befehle

### Getter (Implemented)
```bash
board.hwver     # Hardware-Information
                # Output: "v0.2 (INA228+RTC)"
                # Code: InheroMr2Board.cpp Zeile 175-179

board.telem     # Full telemetry with SOC
                # Output: "B:3.85V/125.432mA/22.3C SOC:68.5% S:5.12V/245mA"
                # Output: "B:3.85V/125.432mA/22.3C SOC:N/A S:5.12V/245mA" (if not synced)
                # Code: InheroMr2Board.cpp - getCustomGetter()

board.stats     # Combined energy statistics (balance + MPPT)
                # Output: "+1250/+450mWh SOL M:85%" oder
                #         "-300/-450mWh BAT M:45% TTL:72h"
                # Components: Today/Avg3d Status MPPT% [TTL]
                # SOL = Solar sufficient (self-sufficient)
                # BAT = Battery deficit (living on battery)
                # Code: InheroMr2Board.cpp - getCustomGetter()

board.cinfo     # Charger info
                # Output: "PG / CC" (Power Good, Constant Current)
                # Code: InheroMr2Board.cpp Zeile 215-221

board.diag      # Detailed BQ25798 diagnostics
                # Output: PG CE HIZ MPPT CHG VBUS VINDPM IINDPM | voltages | temps
                # Code: InheroMr2Board.cpp Zeile 223-226

board.togglehiz # Manual HIZ cycle for input detection
                # Output: "HIZ cycle <was set|forced>: VBUS=5.2V PG=OK"
                # Same logic as automatic checkAndFixPgoodStuck()
                # Always ends with HIZ=0
                # Code: InheroMr2Board.cpp - getCustomGetter()

board.conf      # Gesamte Konfiguration
                # Output: "B:liion1s F:0% M:1 I:500 Vco:4.10"
                # Code: InheroMr2Board.cpp - getCustomGetter()

board.ibcal     # INA228 calibration factor
                # Output: "INA228 calibration: 0.9850 (1.0=default)"
                # Code: InheroMr2Board.cpp - getCustomGetter()

board.leds      # LED enable status
                # Output: "LEDs: ON (Heartbeat + BQ Stat)"
                # Code: InheroMr2Board.cpp - getCustomGetter()

board.uvlo      # UVLO-Einstellung abfragen 🆕
                # Output: "ENABLED" | "DISABLED"
                # Chemie-spezifische Schwellen aktiv
                # Code: InheroMr2Board.cpp - getCustomGetter() Zeile 486
```

### Setter (Implemented)
```bash
set board.batcap <mAh>      # Set battery capacity
                            # Range: 100-100000 mAh
                            # Example: set board.batcap 2200
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.ibcal <current>   # Calibrate INA228 current sensor
                            # Range: -2000 to +2000 mA (actual measured current)
                            # Example: set board.ibcal 100.5
                            # Output: "INA228 calibrated: factor=0.9824"
                            # Or: set board.ibcal reset
                            # Resets calibration to default value (1.0)
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.bat <type>        # Set battery chemistry
                            # Options: liion1s | lifepo1s | lto2s
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.imax <mA>         # Set max charge current
                            # Range: 10-1000 mA
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.mppt <0|1>        # Enable/disable MPPT
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.frost <mode>      # Set frost charge behavior
                            # Options: 0% | 20% | 40% | 100%
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.leds <on|off>     # Enable/disable heartbeat + BQ stat LED
                            # Options: on/1 | off/0
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.soc <percent>     # Manually set SOC percentage
                            # Range: 0-100 (INA228 must be ready)
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.bqreset           # Reset BQ25798 and reload config
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.uvlo <0|1|true|false> # UVLO-Einstellung setzen 🆕
                            # 0/false = DISABLED (Standard für Feldtests)
                            # 1/true = ENABLED (für kritische Anwendungen)
                            # Wird persistent gespeichert
                            # Chemie-spezifische Schwellen werden angewendet
                            # Code: InheroMr2Board.cpp - setCustomSetter() Zeile 643
```

---

## Dateien-Übersicht

### Hauptimplementierung
| Datei | Zeilen | Beschreibung |
|-------|--------|--------------|
| **InheroMr2Board.h** | 116 | Board-Klasse, Energieverwaltungsdefinitionen |
| **InheroMr2Board.cpp** | 761 | Board-Init, Shutdown, RTC, CLI-Commands |
| **BoardConfigContainer.h** | 276 | Battery Management, SOC, Daily Balance Structures |
| **BoardConfigContainer.cpp** | ~2300 | BQ25798, INA228, MPPT, SOC, Daily Balance, tickPeriodic |
| **lib/Ina228Driver.h** | ~180 | INA228 Register, Methods, BatteryData Struct |
| **lib/Ina228Driver.cpp** | ~255 | INA228 I2C Communication, Calibration, Coulomb Counter |
| **lib/BqDriver.h** | ~100 | BQ25798 Driver Interface |
| **lib/BqDriver.cpp** | ~600 | BQ25798 I2C Communication, MPPT, Charging |

### Schlüssel-Methoden
| Methode | Datei | Funktion |
|---------|-------|----------|
| `begin()` | InheroMr2Board.cpp | Board-Initialisierung, Wake-up-Check |
| `initiateShutdown()` | InheroMr2Board.cpp | System ON Idle Shutdown (aufgerufen von `runVoltageMonitor()`) |
| `configureRTCWake()` | InheroMr2Board.cpp | RTC Countdown Timer |
| `rtcInterruptHandler()` | InheroMr2Board.cpp | RTC INT Flag Clear (Read-Modify-Write) |
| `queryBoardTelemetry()` | InheroMr2Board.cpp | CayenneLPP Telemetry Collection |
| `getVoltageCriticalThreshold()` | InheroMr2Board.cpp | Chemistry-specific Critical Voltage (0% SOC) |
| `getVoltageHardwareCutoff()` | InheroMr2Board.cpp | Chemistry-specific UVLO Voltage (INA228 Alert) |
| `runVoltageMonitor()` | BoardConfigContainer.cpp | Spannungsüberwachung, delegiert an `board.initiateShutdown()` |
| `tickPeriodic()` | BoardConfigContainer.cpp | Periodischer Tick-Handler — dispatcht alle I2C-Arbeit (MPPT, SOC, Voltage) |
| `runMpptCycle()` | BoardConfigContainer.cpp | Ein MPPT-Solarverwaltungszyklus (aufgerufen von tickPeriodic) |
| `updateBatterySOC()` | BoardConfigContainer.cpp | Coulomb Counter SOC Calculation |
| `updateDailyBalance()` | BoardConfigContainer.cpp | 7-Day Energy Balance Tracking |
| `calculateTTL()` | BoardConfigContainer.cpp | Time To Live Forecast |
| `Ina228Driver::begin()` | lib/Ina228Driver.cpp | 100mΩ Calibration, ADC Config |
| `Ina228Driver::shutdown()` | lib/Ina228Driver.cpp | Power-down Mode |

---

## Code-Fragmente (Key Sections)

### INA228 Shutdown Mode
```cpp
// Ina228Driver.cpp Zeile 79-83
void Ina228Driver::shutdown() {
  // Set operating mode to Shutdown (MODE = 0x0)
  // This disables all conversions and Coulomb Counter
  uint16_t adc_config = 0x0000;  // MODE = 0x0 (Shutdown)
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}
```

### INA228 Wake-up
```cpp
// Ina228Driver.cpp Zeile 85-90
void Ina228Driver::wakeup() {
  // Re-enable continuous measurement mode
  uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) |  // Continuous all
                        (INA228_ADC_AVG_64 << 0);             // 64 samples average (TX peak filtering)
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}
```

### RTC Interrupt Handler\n```cpp\n// InheroMr2Board.cpp — ISR setzt nur Flag\nvoid InheroMr2Board::rtcInterruptHandler() {\n  rtc_irq_pending = true;\n}\n\n// tick() — Main-Loop-Kontext cleart TF per I2C\nvoid InheroMr2Board::tick() {\n  if (rtc_irq_pending) {\n    rtc_irq_pending = false;\n    // Clear TF via I2C (safe — runs in main loop context)\n    Wire.beginTransmission(RTC_I2C_ADDR);\n    Wire.write(RV3028_REG_STATUS);\n    Wire.endTransmission(false);\n    Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)1);\n    if (Wire.available()) {\n      uint8_t status = Wire.read();\n      status &= ~(1 << 3); // Clear TF bit\n      Wire.beginTransmission(RTC_I2C_ADDR);\n      Wire.write(RV3028_REG_STATUS);\n      Wire.write(status);\n      Wire.endTransmission();\n    }\n  }\n  BoardConfigContainer::tickPeriodic();\n  BoardConfigContainer::feedWatchdog();\n}\n```

### INA228 Driver Zugriff
```cpp
// Direkter Zugriff auf INA228 Driver
if (boardConfig.getIna228Driver() != nullptr) {
  // INA228 specific code
}
```

---

## Szenarien

### Szenario A: Normale Entladung (Software-Shutdown) - Li-Ion
```
t=0:      VBAT = 3.7V → Normal (60s checks, Coulomb Counter läuft)
          Daily balance: Today +150mAh SOLAR
          
t=+1h:    VBAT = 3.5V → Warning (30s checks)
          SOC: 45%
          
t=+2h:    VBAT = 3.45V → Critical (10s checks)
          
t=+2.5h:  VBAT = 3.39V → Software Dangerzone (< 3.4V)
          - board.initiateShutdown(LOW_VOLTAGE)
          - SX1262 power off, INA228 shutdown
          - RTC: Wake in 1h (60 Minuten)
          - System ON Idle (~0.6mA, CE-Pin bleibt LOW)
          
t=+3.5h:  RTC weckt auf (first check after 1h)
          - readVBATDirect() in Idle-Loop
          - VBAT = 3.25V (noch unter Critical 3.4V)
          - Zurück zu System ON Idle (1h)
          
t=+4.5h:  RTC weckt auf (second check)
          - VBAT = 3.20V (noch unter Critical 3.4V)
          - Zurück zu System ON Idle (1h)
          
t=+5.5h:  RTC weckt auf (third check, battery recovered via Solar)
          - readVBATDirect() → VBAT = 4.05V (OK, über Critical 3.4V)
          - NVIC_SystemReset() → Warm Reboot
          - begin() → Danger Zone Recovery markiert
          - Coulomb Counter startet neu bei 0% SOC
          - Daily balance baut sich neu auf
```

### Szenario B: Kritische Entladung (Hardware-UVLO) - Li-Ion
```
t=0:      VBAT = 3.35V → Software Dangerzone
          - board.initiateShutdown(LOW_VOLTAGE)
          - System ON Idle (~0.6mA, CE-Pin LOW)
          
t=+30min: VBAT = 3.05V (weiter gesunken im Sleep!)
          → INA228 ALERT LOW (< 3.1V UVLO)
          → TPS62840 EN = LOW
          → RAK komplett stromlos (0µA)
          → CE-Pin geht HIGH (Pull-Up, kein GPIO-Latch mehr)
          → Laden gesperrt (CE HIGH = Charge Inhibit)
          → RTC läuft weiter (eigene Batterie-Domain)
          → INA228 in Shutdown (1µA)
          
t=∞:      INA228 ALERT ist LATCHED → TPS62840 EN bleibt LOW
          → RAK bleibt stromlos (0µA)
          → Board kommt von selbst NICHT wieder hoch
          → Auch Solar-Laden hilft nicht (CE HIGH → Laden gesperrt)
          → Manuelles Eingreifen erforderlich (Akkutausch oder Reset)
          
          Rationale: Wenn VBAT unter die UVLO-Schwelle fällt, ist der Akku
          wahrscheinlich defekt oder tiefentladen. Automatische Recovery wäre
          hier kontraproduktiv — ein defekter Akku sollte nicht weiter
          geladen/entladen werden.
```

### Szenario C: Daily Balance Tracking - LiFePO4
```
Day 0:    VBAT = 3.2V, SOC = 85%
          Charge: +800mAh (Solar)
          Discharge: -450mAh (TX/RX)
          Net balance: +350mAh → SOLAR
          daily_stats[0] = {timestamp, 800, 450, +350, true}
          
Day 1:    VBAT = 3.15V, SOC = 72%
          Charge: +650mAh
          Discharge: -520mAh
          Net balance: +130mAh → SOLAR
          daily_stats[1] = {timestamp, 650, 520, +130, true}
          
Day 2:    VBAT = 3.05V, SOC = 58%
          Charge: +200mAh (cloudy)
          Discharge: -480mAh
          Net balance: -280mAh → BATTERY
          daily_stats[2] = {timestamp, 200, 480, -280, true}
          
          3-day avg: (350+130-280)/3 = +66.7 mAh/day
          7-day avg: not yet available (only 3 days of data)
          → Still SOLAR (24h net positive)
          
Day 3:    VBAT = 2.95V, SOC = 42%
          Charge: +150mAh (very cloudy)
          Discharge: -500mAh
          Net balance: -350mAh → BATTERY
          daily_stats[3] = {timestamp, 150, 500, -350, true}
          
          3-day avg: (130-280-350)/3 = -166.7 mAh/day
          7-day avg: (350+130-280-350)/4 = -37.5 mAh/day → BATTERY (used for TTL)
          living_on_battery = true
          
          TTL calculation (7-day avg basis):
          remaining = 42% × 1500mAh = 630mAh
          deficit = |-37.5| = 37.5 mAh/day
          TTL = (630 / 37.5) × 24 = 403.2 hours ≈ 16.8 days
          
          CLI output: "Today:-350mAh BATTERY 7dAvg:-38mAh TTL:403h"
```

---

## Testing-Historie

### Phase 1: Compilation ✅
- **Datum**: 31. Januar 2026
- **Status**: Erfolgreich
- **Build**: PlatformIO, Inhero_MR2_repeater environment
- **Exit Code**: 0

### Phase 2: INA228 Kommunikation (TODO)
- I2C-Probe INA228 @ 0x45
- Manufacturer ID: 0x5449 ("TI")
- Device ID: 0x228
- Verify communication stability

### Phase 3: INA228 Calibration (TODO)
- Shunt: 100mΩ ± 1%
- Current LSB: 1.91µA
- Test: 100mA load → 100mA reading
- Test: 500mA load → 500mA reading
- Test: 1A load → 1A reading (max)

### Phase 4: Coulomb Counter (TODO)
- Charge 1000mAh → INA228 charge register ≈ 1000
- Discharge 500mAh → INA228 charge register ≈ 500
- Verify energy register accumulation

### Phase 5: RTC Integration (TODO)
- Countdown 10s test → INT pin trigger
- Countdown 1h test → Wake from SYSTEMOFF
- INT flag clearing → Pin releases to HIGH

### Phase 6: Energieverwaltung End-to-End (TODO)
- Simulate low voltage (3.3V Li-Ion)
- Verify shutdown sequence (all 5 steps)
- Verify INA228 enters shutdown mode
- Verify RTC wake after 1h
- Verify voltage recovery check
- Verify resume at Critical (3.4V+) and 0% SOC initialization

### Phase 7: Daily Balance (TODO)
- Run 7 days with varying solar
- Verify daily statistics accumulation
- Verify 7-day average calculation
- Verify living_on_battery flag
- Verify TTL calculation (7-day avg basis)

### Phase 8: Long-term Stability (TODO)
- 7-day continuous operation
- Solar panel connected
- Monitor daily balance
- Monitor SOC accuracy
- Monitor MPPT statistics
- Verify RTC drift (±3ppm spec)

---

## Known Issues & Workarounds

### Issue 1: SimplePreferences Float Support
**Problem**: SimplePreferences doesn't support `putFloat()` / `getFloat()`
**Solution**: Store as Integer mAh, convert with `(uint16_t)capacity_mah`
**Code**: BoardConfigContainer.cpp Zeile 1258

### Issue 2: RTC INT Pin nicht released
**Problem**: INT pin blieb LOW nach Timer-Flag
**Root Cause**: `Wire.write(0x00)` überschrieb TIE bit
**Solution**: Read-Modify-Write, nur TF bit clearen
**Code**: InheroMr2Board.cpp
**Fixed**: 31. Januar 2026

### Issue 3: Vereinfachte Hardware-Architektur
**Design**: Einheitliche Hardware-Plattform
**Vorteil**: Einfacherer, wartbarerer Code
**Implementierung**: Direkter Zugriff auf INA228/RTC

### Issue 4: SOC-Anzeige in Companion App (Workaround)
**Problem**: Das MeshCore-Protokoll überträgt aktuell nur die Batteriespannung (`getBattMilliVolts()`), keinen direkten SOC-Prozentwert. Die Companion App interpretiert die empfangene Spannung über eine fest hinterlegte Li-Ion-Entladekurve und leitet daraus den SOC% ab. Bei LiFePO4- und LTO-Chemien, deren Spannungsprofile erheblich von Li-Ion abweichen, führt das zu falschen Prozentanzeigen.
**Workaround**: Wenn ein valider Coulomb-Counting-SOC vorliegt (`soc_valid == true`), gibt `getBattMilliVolts()` nicht die echte Batteriespannung zurück, sondern eine aus dem SOC rückgerechnete Li-Ion 1S OCV (Open Circuit Voltage, 3000–4200 mV). Damit interpretiert die App den Wert korrekt — unabhängig von der tatsächlichen Zellchemie.
**Lookup-Tabelle**: Standard Li-Ion NMC/NCA OCV, 11 Stützstellen (0–100% in 10%-Schritten) mit stückweiser linearer Interpolation.
**Fallback**: Solange kein valider SOC vorliegt (z.B. vor dem ersten „Charging Done"), wird die echte Batteriespannung zurückgegeben.
**Code**: `InheroMr2Board::getBattMilliVolts()` + `socToLiIonMilliVolts()` in InheroMr2Board.cpp
**TODO**: Diesen Workaround entfernen, sobald MeshCore die Übertragung des tatsächlichen SOC% (neben oder anstelle der Batterie-mV) unterstützt. Dann soll `getBattMilliVolts()` wieder die echte Spannung liefern.

---

## Future Enhancements

### Short-term
- [ ] Deprecated: Auto-Learning reaktivieren/neu implementieren (BQ25798 CHARGE_DONE detection)
- [ ] Load Shedding implementieren (TX power reduction, BLE disable)
- [ ] CLI-Command: `pwrmgt.test shutdown` für Testing
- [ ] CLI-Command: `pwrmgt.rtc status` für RTC diagnostics

### Medium-term
- [ ] Adaptive RTC wake interval (15min → 1h → 3h bei langer Low-Voltage)
- [ ] SOC persistence in LittleFS (survive cold boots) — aktuell gehen alle Stats bei Reboot verloren
- [ ] Daily balance persistence (survive cold boots) — aktuell gehen alle Stats bei Reboot verloren
- [ ] Web-UI für Energy Dashboard
- [ ] CayenneLPP channel für SOC/Balance

### Long-term
- [ ] Machine Learning für Solar-Prognose
- [ ] Seasonal adjustment (Winter vs. Summer)
- [ ] Multi-device energy sharing (Mesh-level)
- [ ] Battery health estimation (internal resistance)
- [ ] Predictive maintenance alerts

---

## Referenzen

### Datasheets
- **INA228**: https://www.ti.com/product/INA228
- **RV-3028-C7**: https://www.microcrystal.com/en/products/real-time-clock-rtc-modules/rv-3028-c7/
- **BQ25798**: https://www.ti.com/product/BQ25798
- **TPS62840**: https://www.ti.com/product/TPS62840
- **nRF52840**: https://www.nordicsemi.com/products/nrf52840

### Code-Repositories
- **MeshCore**: https://github.com/[repo]/MeshCore
- **Variant**: `variants/inhero_mr2/`

### Related Documentation
- [README.md](README.md) - User-facing documentation (DE)
- [BATTERY_AUTO_LEARNING.md](BATTERY_AUTO_LEARNING.md) - Deprecated: Battery capacity auto-learning details

---

## Changelog

### v3.0 - 15. März 2026 (Architektur-Refactoring: Flag/Tick-Pattern)
- 🛠️ **Architektur**: I2C-Mutex (`I2CMutex.h`) komplett entfernt
- 🛠️ **Architektur**: FreeRTOS-Tasks `solarMpptTask`, `socUpdateTask`, `voltageMonitorTask` eliminiert
- 🛠️ **Architektur**: Neue `tickPeriodic()` — alle I2C-Arbeit im Main-Loop-Kontext (millis()-basiertes Scheduling)
- 🛠️ **Architektur**: `runMpptCycle()` extrahiert aus solarMpptTask-Body
- 🛠️ **Architektur**: `tick()` vereinfacht: RTC-Clear + `tickPeriodic()` + `feedWatchdog()`
- 🛠️ **WDT**: Liveness-Tracking (`mpptTaskLastAlive`/`socTaskLastAlive`) und `recoverI2CBus()` entfernt — WDT feuert natürlich wenn `tick()` hängt
- 🛠️ **Grund**: Mutex schützte nur eigenen Code, nicht MeshCore's I2C-Zugriffe. tick() blockiert den Core → Bus-Contention architektonisch unmöglich
- 📝 Dokumentation vollständig aktualisiert (README.md + IMPLEMENTATION_SUMMARY.md)

### v2.2 - 25. Februar 2026 (Code-Fix + Dokumentation an Code-Stand angepasst)
- 🐛 **Code-Fix**: `runVoltageMonitor()` delegiert jetzt an `board.initiateShutdown(LOW_VOLTAGE)` statt inline `sd_power_system_off()` — CE-Pin bleibt LOW, Solar-Laden in Danger Zone möglich
- 📝 `.noinit Stats Preservation` entfernt — Feature existiert nicht im Code (kein Linker-Script, keine Structs/Funktionen)
- 📝 Danger Zone korrekt als System ON Idle dokumentiert (CE-Pin LOW, GPIO-Latches aktiv)
- 📝 `initiateShutdown()` als aktiver Code-Pfad dokumentiert (aufgerufen von `runVoltageMonitor()`)
- 📝 RTC-Wake-Intervall von 12h auf 6h korrigiert, dann auf 1h reduziert (Code: `DANGER_ZONE_SLEEP_MINUTES = 60`)
- 📝 `voltageMonitorTask` und `socUpdateTask` als historisch dokumentiert, ersetzt durch `tickPeriodic()`
- 📝 Statistik-Persistenz-Sektion neu geschrieben: keine Persistenz für 168h-Ringpuffer
- 📝 Szenarien und Future Enhancements aktualisiert

### v2.1 - 4. Februar 2026 (Dokumentation aktualisiert)
- 📝 Dokumentation vollständig überarbeitet und aktualisiert
- 📝 Datei- und Zeilenreferenzen korrigiert für InheroMr2Board.cpp
- 📝 CLI-Command `board.diag` dokumentiert (detaillierte BQ25798 Diagnostics)
- 📝 Code-Struktur dokumentiert und vereinfacht
- ✅ Code-Implementierung unverändert (bereits vollständig in v2.0)

### v2.0 - 31. Januar 2026 (Implementiert)
- ✅ INA228 Driver vollständig implementiert (255 Zeilen)
- ✅ Coulomb Counter mit SOC-Berechnung
- ✅ Tägliche Energiebilanz (7-Tage rolling)
- ✅ TTL Forecast Algorithmus
- ✅ RTC Wake-up Management
- ✅ INA228 Shutdown Mode
- ✅ Energieverwaltungsablauf (5 Schritte)
- ✅ CLI Commands (hwver, soc, balance, batcap, diag)
- ✅ Voltage Monitor Task (adaptive)
- ✅ Hardware UVLO (INA228 → TPS EN)
- ✅ RTC Interrupt Handler Fix (Read-Modify-Write)
- ✅ Chemie-spezifische Schwellen
- ✅ Preferences als Integer-Storage
- ✅ README dokumentiert
- ✅ Compilation erfolgreich (Exit 0)

### v1.0 - 29. Januar 2026 (Initial Design)
- Design-Phase Dokumentation
- Hardware-Architektur definiert
- Grundkonzept RTC Wake-up
- Test-Strategie erstellt

---

## Autoren

**Implementierung**: GitHub Copilot (Claude Sonnet 4.5)
**Hardware-Design**: Inhero GmbH
**Projekt**: MeshCore

**Kontakt**: Siehe README.md

---

## Siehe auch

- [README.md](README.md) - Übersicht, Feature-Matrix und Diagnose
- [QUICK_START.md](QUICK_START.md) - Schnellstart fuer Inbetriebnahme und CLI-Setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) - Alle board-spezifischen CLI-Befehle auf einen Blick
- [BATTERY_AUTO_LEARNING.md](BATTERY_AUTO_LEARNING.md) - Veraltet: historisches Auto-Learning-Konzept

---

*Letzte Aktualisierung: 25. Februar 2026*
*Status: ✅ Dokumentation an aktuellen Code-Stand angepasst*
