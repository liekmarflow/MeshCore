# Inhero MR-2 Energieverwaltung - Implementierungs-Dokumentation

## Inhaltsverzeichnis

- [Überblick](#überblick)
- [Hardware-Architektur](#hardware-architektur)
- [1. Software-Monitoring (Adaptiv)](#1-software-monitoring-adaptiv)
- [2. Coulomb Counter & SOC (Ladezustand)](#2-coulomb-counter--soc-ladezustand)
- [3. Tägliche Energiebilanz](#3-tägliche-energiebilanz)
- [4. Solar-Energieverwaltung & Interrupt-Loop-Vermeidung](#4-solar-energieverwaltung--interrupt-loop-vermeidung)
- [5. Time-To-Live (TTL)-Prognose](#5-time-to-live-ttl-prognose)
- [6. RTC-Wakeup-Management](#6-rtc-wakeup-management)
- [7. Energieverwaltungsablauf](#7-energieverwaltungsablauf)
- [12. .noinit Stats Preservation](#12-noinit-stats-preservation-statistik-erhaltung)
- [Siehe auch](#siehe-auch)

> ✅ **STATUS: IMPLEMENTIERT (laufend gepflegt)** ✅
> 
> Diese Dokumentation beschreibt die vollständige Energieverwaltungs-Implementierung für das Inhero MR-2 Board.
> Hardware mit INA228 Power Monitor und RV-3028-C7 RTC ist funktional implementiert.
> Hinweis: Konkrete Zeilennummern können durch Refactorings abweichen.
> 
> Datum: 4. Februar 2026
> Version: 2.1 (Implementiert und dokumentiert)
> Hardware: INA228 + RTC

---

## Überblick

Das System kombiniert **3 Schutz-Schichten** + **Coulomb Counter** + **tägliche Energiebilanz** + **CE-Pin Hardware-Ladesicherung** + **.noinit Stats Preservation** für optimalen Filesystem-Schutz, Energie-Effizienz und Batterie-Monitoring:

1. **Software-Spannungsüberwachung** (adaptive Task) - Danger-Zone-Erkennung
2. **RTC-Wakeup-Management** (RV-3028-C7) - periodische Recovery-Checks
3. **Hardware-UVLO** (INA228 Alert → TPS62840 EN) - ultimative Schutzebene
4. **Coulomb Counter** (INA228) - Echtzeit-SOC-Tracking
5. **Tägliche Energiebilanz** (7-Tage rolling) - Solar vs. Batterie
6. **INA228 Shutdown-Modus** - Stromsparen während Danger Zone (~1µA)
7. **BQ CE-Pin** (P0.04) - Hardware-Ladesicherung (Dual-Layer: GPIO + I2C)
8. **.noinit Stats Preservation** - 168h-Statistiken überleben NVIC_SystemReset() (Magic+CRC32)

### Aktuelle Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| Spannungsüberwachung + Danger-Zone-Shutdown | Aktiv | Produktiv im Betrieb |
| Hardware-UVLO (INA228 Alert → TPS62840 EN) | Aktiv | Hardware-Schutz aktiv |
| RTC-Wakeup (SYSTEMOFF-Recovery) | Aktiv | 12h (Produktion) / 60s (Test) |
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
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC (IBUS has ~±30mA error at low currents) |
| **BQ CE-Pin** | Charge Enable | — | GPIO4 (P0.04) | Active LOW, ext. Pull-Up zu VSYS, Dual-Layer mit I2C |
| **TPS62840** | Buck Converter | - | EN←INA_Alert | 750mA, EN controlled by INA228 |

---

## 1. Software-Monitoring (Adaptiv)

### Implementierung
- **Task**: `socUpdateTask()` (ruft `runVoltageMonitor()` pro Minute)
- **Messung**: INA228 via I²C (batterie.voltage)
- **Frequenz**: 60s

### Monitoring-Intervalle

**Konfiguration:** Feste Intervalle
- **Normalbetrieb**: 60s
- **Danger Zone**: 6 Stunden (RTC-Wake)

**Two-Stage Strategy (Power-Optimized):**

| System State | Voltage (Li-Ion) | Interval | Cost per Check | Rationale |
|--------------|------------------|----------|----------------|----------|
| **Running (Normal)** | ≥ 3.4V | **60 seconds** | ~1µAh | System already awake, INA228 I²C read minimal cost |
| **System ON Idle (Danger Zone)** | < 3.4V | **6 hours** | ~0.03mAh | System ON Idle: GPIO-Latches aktiv, RTC-Wakeup, NVIC_SystemReset |

**Kernpunkt:** Checks im Normalmodus sind praktisch kostenlos (System läuft ohnehin). In der Danger Zone wird **System ON Idle** verwendet (statt SYSTEMOFF), um GPIO-Latches aktiv zu halten — insbesondere den BQ CE-Pin (LOW = Laden freigegeben). Nach RTC-Ablauf erfolgt `NVIC_SystemReset()` für einen sauberen Neustart.

**Action at < 3.4V**: `initiateShutdown()` → System ON Idle + RTC timer 6 hours + NVIC_SystemReset

**Hinweis:** Die exakte Implementierung befindet sich in BoardConfigContainer.cpp im voltageMonitorTask().

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
- **Update-Intervall**: voltageMonitorTask() ruft auf (1h normal, 6h Danger Zone System ON Idle wake)

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
- **Aufgerufen von**: voltageMonitorTask()
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
- Interrupt weckt solarMpptTask() → ruft `checkAndFixSolarLogic()` → schreibt MPPT → neuer Interrupt → **Loop**

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
void BoardConfigContainer::onBqInterrupt() {
  // CRITICAL: Always clear interrupt flags by reading CHARGER_STATUS_0 register
  // The BQ25798 requires reading register 0x1B to acknowledge interrupts
  if (bqDriverInstance) {
    bqDriverInstance->readReg(0x1B); // Clear interrupt flags
  }
  
  // Wake solarMpptTask for immediate processing
  if (solarEventSem != NULL) {
    xSemaphoreGiveFromISR(solarEventSem, NULL);
  }
}
```

**Flag Clearing auf Boot**: `BoardConfigContainer::configureBq()` Zeile 1075
- Liest FAULT_STATUS Register (0x20, 0x21) nach Konfiguration
- Vermeidet stale faults von vorherigem Power-Cycle

### Task-Architektur

**solarMpptTask** (15-Minuten Intervall + Interrupt-getriggert):
```
Periodic Check (15min):
  ├─ checkAndFixPgoodStuck()  ← 5-Min Cooldown
  └─ checkAndFixSolarLogic()  ← 60s Cooldown + PG=1 Check

Interrupt Event:
  ├─ onBqInterrupt() clears 0x1B
  └─ Task wacht auf → läuft sofort
```

**Timing Summary**:
- **MPPT Write Cooldown**: 60 Sekunden (verhindert Loop)
- **HIZ Toggle Cooldown**: 5 Minuten (verhindert excessive toggling)
- **Periodic Check**: 15 Minuten (normal task run)
- **Interrupt Response**: Sofort (weckt task)

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
**Methode**: `configureRTCWake()` in `InheroMr2Board.cpp` Zeile 704-737
- **Tick Rate**: 1Hz (1 Sekunde pro Tick)
- **Max Countdown**: 65535 Sekunden ≈ 18.2 Stunden
- **Danger Zone Interval**: 6 Stunden (21600 Ticks)
- **Rationale**: Each wake costs ~50-150mAh (full boot), long interval maximizes battery life

**Register**:
```cpp
RV3028_CTRL1 (0x00):     TE bit (Timer Enable)
RV3028_CTRL2 (0x01):     TIE bit (Timer Interrupt Enable), TF bit (Timer Flag)
RV3028_COUNTDOWN_LSB (0x09): Countdown value LSB
RV3028_COUNTDOWN_MSB (0x0A): Countdown value MSB
```

### Interrupt Handler
**Methode**: `rtcInterruptHandler()` Zeile 739-761
- **Read-Modify-Write** CTRL2 register
- **Clear nur TF bit** (bit 3), preserve TIE (bit 7)
- **Fehler vorher**: `Wire.write(0x00)` löschte alle Bits → INT blieb LOW
- **Fix**: `ctrl2 &= ~(1 << 3)` clear nur Timer Flag

---

## 7. Energieverwaltungsablauf

### Shutdown-Sequenz
**Methode**: `initiateShutdown()` in `InheroMr2Board.cpp`

**Bei Low-Voltage → System ON Idle** (GPIO-Latches bleiben aktiv für CE-Pin):

**6 Schritte**:
1. **Stop Background Tasks**: `BoardConfigContainer::stopBackgroundTasks()`
   - Stoppt MPPT task, Heartbeat task, Voltage Monitor task
   - Verhindert Filesystem-Korruption
   
2. **INA228 Shutdown**:
   ```cpp
   if (boardConfig.getIna228Driver() != nullptr) {
     boardConfig.getIna228Driver()->shutdown();  // MODE=0x0 → Power-down
   }
   ```
   - Stoppt alle ADC-Konversionen
   - Deaktiviert Coulomb Counter (kein Zählen bei 0% SOC sowieso)
   - Reduziert INA228-Stromverbrauch auf ~1µA
   
3. **BQ25798 Interrupt deaktivieren**:
   ```cpp
   detachInterrupt(BQ_INT_PIN);
   // Verhindert Spurious-Wakeups durch Solar-Events (PGOOD-Toggles)
   // BQ25798 MPPT/CC/CV läuft autonom in Hardware weiter
   ```

4. **RTC Wake konfigurieren** (bei LOW_VOLTAGE):
   ```cpp
   if (reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
     delay(100);  // Allow I/O to complete
     configureRTCWake(21600);  // 6 Stunden (21600 Sekunden)
   }
   ```
   
5. **Shutdown-Grund speichern**:
   ```cpp
   NRF_POWER->GPREGRET2 = reason;  // Persistent über Reset
   ```
   
6. **System ON Idle Loop** (bei LOW_VOLTAGE):
   ```cpp
   // GPIO-Latches bleiben aktiv → BQ_CE_PIN bleibt LOW → Solar-Laden möglich
   while (!rtc_expired) {
     sd_app_evt_wait();    // CPU schläft (~3µA)
     sd_evt_get(nullptr);  // SoftDevice Events verarbeiten
     // RTC Timer-Flag pollen (kein Interrupt nötig)
   }
   NVIC_SystemReset();     // Sauberer Neustart
   ```

7. **Stats in .noinit RAM sichern** (vor Step 6):
   ```cpp
   // Coulomb Counter + MPPT-Stats überleben NVIC_SystemReset() dank .noinit-Section
   BoardConfigContainer::saveStatsToNoinit((uint8_t)boardConfig.getBatteryType());
   // ~4.8 KB mit Magic (0x494E4852) + CRC32 (IEEE 802.3)
   ```

8. **Stats nach Warm Reset wiederherstellen** (in `begin()`):
   ```cpp
   // Nach voltage-recovery: .noinit-Daten validieren und restoren
   uint8_t restoredBatType = 0;
   if (BoardConfigContainer::restoreStatsFromNoinit(&restoredBatType)) {
       noinit_stats_invalidate(); // Nur einmal verwenden
   }
   ```

**Nicht-Low-Voltage Shutdowns** (User Request, Thermal): Verwenden weiterhin `sd_power_system_off()`.

**Warum System ON Idle statt System OFF?**
- System OFF → alle GPIOs werden High-Z → CE-Pin Pull-Up → HIGH → Laden gesperrt
- System ON Idle → GPIO-Latches bleiben aktiv → CE-Pin bleibt LOW → **Solar-Laden weiterhin möglich**
- Stromverbrauch: ~3µA (System ON Idle) vs. ~2µA (System OFF) — vernachlässigbar

### Wake-up-Check (Anti-Motorboating)
**Methode**: `InheroMr2Board::begin()` Zeile 459+

Der Code prüft `GPREGRET2` für den Shutdown-Grund und die Batteriespannung für Wake-up-Entscheidungen.

**Problem**: Nach Hardware-UVLO ist `GPREGRET2 = 0x00` (kompletter RAM-Verlust)
→ Ohne universelle Voltage Check: Motorboating bei knapper Spannung

**3 Fälle**:

**Case 1: RTC Wake from Software-SHUTDOWN** (`GPREGRET2 = SHUTDOWN_REASON_LOW_VOLTAGE`)
```cpp
// InheroMr2Board::begin() prüft Spannung bei Wake-up
if (shutdown_reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
  if (vbat_mv < critical_threshold) {
    configureRTCWake(21600);  // 6 Stunden (minimize boot cost)
    sd_power_system_off();
  }
  // Voltage recovered - resume at 0% SOC
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
  boardConfig.startReverseLearning();  // Start at 0% SOC
}
```

**Case 2: ColdBoot after Hardware-UVLO** (`GPREGRET2 = 0x00`, voltage still low)
```cpp
// Prüft auch bei Cold Boot die Spannung
else if (vbat_mv < critical_threshold) {
  MESH_DEBUG_PRINTLN("ColdBoot in danger zone - likely Hardware-UVLO");
  configureRTCWake(21600);  // 6h (minimize repeated boot attempts)
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_LOW_VOLTAGE;
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

**Motorboating-Präventionsablauf** (Li-Ion-Beispiel):
1. Hardware UVLO @ 3.1V → INA228 Alert → TPS62840 EN=LOW → RAK stromlos
2. Solar lädt auf 3.15V → TPS62840 EN=HIGH (50mV Hysterese)
3. RAK bootet → `GPREGRET2 = 0x00` (RAM war gelöscht)
4. **Early Boot Check** → `vbat=3150mV < critical_threshold=3400mV` ✅ DETECTED
5. **Action** → `configureRTCWake(12)` + `GPREGRET2 = LOW_VOLTAGE` + System ON Idle
6. **Result** → Kein Motorboating! System schläft im System ON Idle (GPIO-Latches aktiv, CE-Pin bleibt LOW)

**Stromverbrauch während System ON Idle (Danger Zone)**:
- nRF52840 (System ON Idle): ~2-3µA
- INA228 (Shutdown): ~1µA
- RV-3028 (Running): ~45nA
- TPS62840 (disabled): <1µA
- **Total**: ~3-4µA (vs. ~2-3µA bei System OFF — vernachlässigbar)
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
- VBAT < UVLO → ALERT=LOW → TPS62840 EN=LOW → RAK stromlos (0µA)
- VBAT > UVLO → ALERT=HIGH → TPS62840 EN=HIGH → RAK powered

**Schutzschichten** (Li-Ion example):
1. Software Critical (3.4V / 0% SOC) → Controlled shutdown, danger zone boundary
2. Hardware UVLO (3.1V) → Emergency cutoff via INA228 Alert
3. Hysterese (300mV) → Verhindert Motorboating zwischen UVLO und Critical
4. ADC Filter (64-sample avg) → Filtert TX-Peaks (~100ms), verhindert false triggers

---

## 9. SX1262 Power Control (RadioLib)

### Shutdown-Sequenz Erweiterung
**Methode**: `initiateShutdown()` in `InheroMr2Board.cpp` + `BoardConfigContainer.cpp`

Vor SYSTEMOFF wird die SX1262 LoRa-Radio korrekt in den Sleep-Modus versetzt:

```cpp
// In BoardConfigContainer.cpp before shutdown:
extern RadiolibInterface radio_driver;  // Declared in target.h
if (radio_driver.isRadioAvailable()) {
  radio_driver.powerOff();  // RadioLib sleep command, NOT GPIO toggling
}
```

**Wichtige Details**:
- **Richtige Methode**: `radio_driver.powerOff()` via RadioLib library
- **NICHT verwenden**: SPI SetSleep commands oder SX126X_POWER_EN GPIO toggling 
  - Grund: SX126X_POWER_EN steuert die RF-Antennenweiche (DIO2), nicht die Power-Supply
  - RF-Switch kann TX-Stromverbrauch beeinflussen, aber nicht den Sleep-Strom
- **Stromverbrauch im Sleep**: ~0.66µA (korrekt) vs. 7.4mA (falsch ohne RadioLib powerOff)
- **ADC Averaging**: 64 samples filtern TX-Spitzen, aber nur RadioLib powerOff stellt sicher, dass der Sleep-Mode aktiv ist

**Sleep-Strom Messungen**:
- **Mit RadioLib powerOff()**: ~0.66µA (Correct)
- **Ohne (nur GPIO)**: ~7.4mA (High - Radio still partially active)
- **Danger Zone Cost**: 21600s * 0.66µA ≈ 14mAh overhead (minimal)

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

Im Danger-Zone-Modus wird **System ON Idle** statt System OFF verwendet:
- System ON Idle → GPIO-Latches bleiben aktiv → **CE-Pin bleibt LOW**
- BQ25798 MPPT/CC/CV läuft autonom in Hardware → Solar-Laden möglich
- `detachInterrupt(BQ_INT_PIN)` verhindert Spurious-Wakeups durch Solar-Events

| Zustand | CE-Pin | Laden | Solar-Recovery |
|---|---|---|---|
| RAK stromlos | HIGH (Pull-Up) | Gesperrt | Nein |
| Early Boot | HIGH (explizit) | Gesperrt | Nein |
| BAT_UNKNOWN | HIGH | Gesperrt | Nein |
| Chemie konfiguriert | LOW | Freigegeben | Ja |
| System ON Idle | LOW (gelatcht) | **Freigegeben** | **Ja** |
| System OFF | HIGH (Pull-Up) | Gesperrt | Nein |

---

## 12. .noinit Stats Preservation (Statistik-Erhaltung)

### Problem
Bei `NVIC_SystemReset()` (Danger Zone Recovery nach System ON Idle) löscht der C-Startup-Code die `.bss`-Section und lädt `.data` aus Flash. Alle im RAM liegenden Statistiken — 168h Coulomb Counter Ring Buffer, MPPT-Statistiken, SOC-Zustand — gehen verloren, obwohl der RAM physisch bestromt bleibt.

### Lösung: `.noinit` Linker-Section
Eine spezielle `(NOLOAD)`-Section im Linker-Script wird vom C-Startup **nicht** initialisiert. Variablen mit `__attribute__((section(".noinit")))` behalten ihren RAM-Inhalt über Warm Resets.

### Architektur

```
                    inhero_mr2.ld (Custom Linker Script)
                    ├── Basis: nrf52840_s140_v6.ld
                    └── Zusatz: .noinit (NOLOAD) INSERT AFTER .bss

NoInitStatsPreserve (~4830 Bytes, __attribute__((packed))):
┌──────────────┬─────────────────┬───────────────────┬─────────────┬──────────┐
│ magic (4B)   │ socStats (2788B)│ mpptStats (2036B) │ batType (1B)│ crc32 (4B)│
│ 0x494E4852   │ 168h Ringpuffer │ 168h MPPT-Buffer  │ Chemistry   │ IEEE 802.3│
└──────────────┴─────────────────┴───────────────────┴─────────────┴──────────┘
```

### Validierung (2-stufig)
1. **Magic Check** (`0x494E4852` = "INHR"): Schnelle Prüfung, ob der Speicher überhaupt gültige Daten enthält. Filtert Garbage bei Cold Boot.
2. **CRC32** (IEEE 802.3, Polynom `0xEDB88320`): Vollständige Integritätsprüfung über alle Felder außer dem CRC-Feld selbst. Erkennt Bit-Fehler und partielle Korruption.

### API

```cpp
// In BoardConfigContainer (static Wrappers für private Members):
static void saveStatsToNoinit(uint8_t batteryType);
static bool restoreStatsFromNoinit(uint8_t* batteryType);

// Freie Funktionen:
void noinit_stats_save(const BatterySOCStats*, const MpptStatistics*, uint8_t);
bool noinit_stats_restore(BatterySOCStats*, MpptStatistics*, uint8_t*);
void noinit_stats_invalidate(void);  // magic = 0
```

### Datenfluss

```
initiateShutdown(LOW_VOLTAGE)
    ├── stopBackgroundTasks()
    ├── ina228.shutdown()
    ├── saveStatsToNoinit(batteryType)    ← NEU: Stats sichern
    ├── System ON Idle (sd_app_evt_wait loop)
    └── NVIC_SystemReset()
           │
           ├── C-Startup: .bss=0, .data aus Flash
           ├── .noinit: UNBERÜHRT (NOLOAD)
           │
           └── begin() → GPREGRET2 check
                  ├── Voltage recovered?
                  │     ├── Ja → restoreStatsFromNoinit()
                  │     │         ├── Magic OK? CRC32 OK?
                  │     │         │     ├── Ja → memcpy → socStats, mpptStats
                  │     │         │     └── noinit_stats_invalidate()
                  │     │         └── Nein → Fresh Start
                  │     └── Nein → Sleep again
                  └── Cold Boot → Magic/CRC fail → Fresh Start
```

### Wann bleiben Stats erhalten?

| Szenario | Erhalten? | Grund |
|---|---|---|
| System ON Idle → NVIC_SystemReset() | ✅ | RAM bestromt, .noinit nicht genullt |
| System OFF (sd_power_system_off) | ❌ | Kein RAM Retention konfiguriert |
| Cold Boot (erstmals einschalten) | ❌ | Garbage → Magic/CRC32 ungültig |
| Hardware-UVLO (TPS62840 EN) | ❌ | Kompletter Spannungsverlust |
| User-initiated Reboot | ❌ | Nur Danger Zone speichert Stats |

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
| **BoardConfigContainer.cpp** | ~2300 | BQ25798, INA228, MPPT, SOC, Daily Balance, Tasks |
| **lib/Ina228Driver.h** | ~180 | INA228 Register, Methods, BatteryData Struct |
| **lib/Ina228Driver.cpp** | ~255 | INA228 I2C Communication, Calibration, Coulomb Counter |
| **lib/BqDriver.h** | ~100 | BQ25798 Driver Interface |
| **lib/BqDriver.cpp** | ~600 | BQ25798 I2C Communication, MPPT, Charging |

### Schlüssel-Methoden
| Methode | Datei | Zeile | Funktion |
|---------|-------|-------|----------|
| `begin()` | InheroMr2Board.cpp | 459+ | Board-Initialisierung, Wake-up-Check |
| `initiateShutdown()` | InheroMr2Board.cpp | 667-702 | 5-Step Shutdown Sequenz |
| `configureRTCWake()` | InheroMr2Board.cpp | 704-737 | RTC Countdown Timer |
| `rtcInterruptHandler()` | InheroMr2Board.cpp | 739-761 | RTC INT Flag Clear (Read-Modify-Write) |
| `queryBoardTelemetry()` | InheroMr2Board.cpp | 129-159 | CayenneLPP Telemetry Collection |
| `getVoltageCriticalThreshold()` | InheroMr2Board.cpp | 570-574 | Chemistry-specific Critical Voltage (0% SOC) |
| `getVoltageHardwareCutoff()` | InheroMr2Board.cpp | 577-580 | Chemistry-specific UVLO Voltage (INA228 Alert) |
| `voltageMonitorTask()` | BoardConfigContainer.cpp | ~1100+ | Adaptive Voltage Monitoring |
| `updateBatterySOC()` | BoardConfigContainer.cpp | ~1400+ | Coulomb Counter SOC Calculation |
| `updateDailyBalance()` | BoardConfigContainer.cpp | ~1500+ | 7-Day Energy Balance Tracking |
| `calculateTTL()` | BoardConfigContainer.cpp | ~1600+ | Time To Live Forecast |
| `Ina228Driver::begin()` | lib/Ina228Driver.cpp | 12-47 | 100mΩ Calibration, ADC Config |
| `Ina228Driver::shutdown()` | lib/Ina228Driver.cpp | 79-83 | Power-down Mode |

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

### RTC Interrupt Handler Fix
```cpp
// InheroMr2Board.cpp Zeile 739-761
void InheroMr2Board::rtcInterruptHandler() {
  // Read current CTRL2 register
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL2);  // 0x01
  Wire.endTransmission(false);
  Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)1);
  
  if (Wire.available()) {
    uint8_t ctrl2 = Wire.read();
    
    // Clear TF bit (bit 3) by writing 0 to it
    ctrl2 &= ~(1 << 3);  // Clear bit 3 (TF - Timer Flag)
    
    // Write back to clear the flag and release INT pin
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_CTRL2);
    Wire.write(ctrl2);
    Wire.endTransmission();
  }
}
```

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
          - INA228 shutdown mode
          - Filesystem sync
          - RTC: Wake in 12h (43200s)
          - SYSTEMOFF (2-6µA total)
          
t=+14.5h: RTC weckt auf (first check after 12h)
          - VBAT = 3.25V (noch unter Critical 3.4V)
          - INA228 wakeup
          - Check voltage
          - Zurück zu SYSTEMOFF (12h)
          
t=+26.5h: RTC weckt auf (second check)
          - VBAT = 3.20V (noch unter Critical 3.4V)
          - Zurück zu SYSTEMOFF (12h)
          
t=+32h:   Sonne kommt → VBAT = 3.85V
          (Aber: Noch kein RTC-Wake, RAK schläft)
          
t=+38.5h: RTC weckt auf (third check, battery recovered)
          - VBAT = 4.05V (OK, über Critical 3.4V)
          - Normal weiter bei 0% SOC ✅
          - Reverse Learning startet
          - Coulomb Counter resumes
          - Daily balance continues
```

### Szenario B: Kritische Entladung (Hardware-UVLO) - Li-Ion
```
t=0:      VBAT = 3.35V → Software Dangerzone
          - INA228 shutdown
          - SYSTEMOFF
          
t=+30min: VBAT = 3.05V (weiter gesunken im Sleep!)
          → INA228 ALERT LOW (< 3.1V UVLO)
          → TPS62840 EN = LOW
          → RAK komplett stromlos (0µA)
          → RTC läuft weiter
          → INA228 in Shutdown (1µA)
          
t=+6h:    Solar lädt → VBAT = 3.65V
          → INA228 ALERT HIGH (> 3.1V + 50mV hysteresis)
          → TPS62840 EN = HIGH
          → Hardware-Boot (Cold boot)
          → begin() checkt GPREGRET2
          → Aber: GPREGRET2 = 0x00 (kein SYSTEMOFF, war Hardware-Cutoff)
          → Normal weiter ✅
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
- [ ] Adaptive RTC wake interval (1h → 3h → 6h bei langer Low-Voltage)
- [x] ~~SOC persistence in LittleFS (survive cold boots)~~ → Teilweise gelöst: `.noinit` preserviert Stats über Warm Resets (NVIC_SystemReset). Für Cold Boots wäre LittleFS-Persistenz nötig.
- [x] ~~Daily balance persistence (survive cold boots)~~ → Teilweise gelöst: Ditto — `.noinit` deckt Danger Zone Recovery ab.
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

*Letzte Aktualisierung: 4. Februar 2026, 12:00 UTC*
*Status: ✅ Vollständig implementiert und dokumentiert für MR-2, Compilation erfolgreich, Hardware-Testing ausstehend*
