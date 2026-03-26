# Inhero MR-2 Energieverwaltung - Implementierungs-Dokumentation (Rev 1.0)

## Inhaltsverzeichnis

- [Überblick](#überblick)
- [Hardware-Architektur](#hardware-architektur)
- [1. Low-Voltage-Erkennung (INA228 ALERT ISR)](#1-low-voltage-erkennung-ina228-alert-isr)
- [2. Coulomb Counter & SOC (Ladezustand)](#2-coulomb-counter--soc-ladezustand)
- [3. Tägliche Energiebilanz](#3-tägliche-energiebilanz)
- [4. Solar-Energieverwaltung & Interrupt-Loop-Vermeidung](#4-solar-energieverwaltung--interrupt-loop-vermeidung)
  - [BQ25798 ADC bei niedrigen Batteriespannungen](#bq25798-adc-bei-niedrigen-batteriespannungen)
- [5. Time-To-Live (TTL)-Prognose](#5-time-to-live-ttl-prognose)
- [6. RTC-Wakeup-Management](#6-rtc-wakeup-management)
- [7. Energieverwaltungsablauf](#7-energieverwaltungsablauf)
- [Siehe auch](#siehe-auch)

> ✅ **STATUS: IMPLEMENTIERT (Rev 1.0)** ✅
> 
> Diese Dokumentation beschreibt die vollständige Energieverwaltungs-Implementierung für das Inhero MR-2 Board.
> Hardware Rev 1.0: INA228 ALERT auf P1.02, TPS62840 EN an VDD, CE-Pin via DMN2004TK-7 FET (invertiert).
> Hinweis: Konkrete Zeilennummern können durch Refactorings abweichen.
> 
> Datum: 02. Juni 2026
> Version: 3.0 (Rev 1.0 Architektur)
> Hardware: INA228 + RTC + DMN2004TK-7 CE-FET

---

## Überblick

Das System kombiniert **INA228 ALERT-basierte Low-Voltage-Erkennung** + **System-Off** + **Coulomb Counter** + **tägliche Energiebilanz** + **CE-Pin FET-Safety** für maximale Energie-Effizienz:

1. **INA228 ALERT ISR** (P1.02) - Low-Voltage-Erkennung via Hardware-Interrupt
2. **System-Off** (~15µA) mit RTC-Wake - Minimaler Stromverbrauch bei Low-Voltage
3. **CE-Pin FET-Safety** (DMN2004TK-7) - Invertierte Logik, Solar-Laden in System-Off möglich
4. **Coulomb Counter** (INA228) - Echtzeit-SOC-Tracking
5. **Tägliche Energiebilanz** (7-Tage rolling) - Solar vs. Batterie
6. **RTC-Wakeup-Management** (RV-3028-C7) - Periodische Recovery-Checks

### Architektur-Unterschiede v0.2 → Rev 1.0

| Aspekt | v0.2 | Rev 1.0 |
|--------|------|---------|
| Low-Voltage-Erkennung | Software-Polling (60s) + Hardware-UVLO (INA228→TPS EN) | INA228 ALERT ISR auf P1.02 (Hardware-Interrupt) |
| Shutdown-Modus | System ON Idle (__WFI-Loop, ~0.6mA) | System-Off (~15µA) |
| CE-Pin Logik | Direkt (LOW=enable) | FET-invertiert via DMN2004TK-7 (HIGH=enable) |
| TPS62840 EN | Software-steuerbar (UVLO kann EN abschalten) | An VDD gebunden (immer an) |
| Schwellen-Modell | 2 Stufen (Danger Zone + UVLO) | 1 Stufe (lowv_sleep_mv / lowv_wake_mv) |
| GPIO-Latching | Erforderlich (System ON für CE-Pin) | Nicht nötig (CE-FET hält Zustand in System-Off) |

### Aktuelle Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| INA228 ALERT → Low-Voltage System-Off | Aktiv | ISR auf P1.02 → Task-Notification → System-Off + RTC-Wake |
| RTC-Wakeup (Low-Voltage-Recovery) | Aktiv | 15 min (periodisch) |
| BQ CE-Pin Safety (FET-invertiert) | Aktiv | HIGH=Laden an (via DMN2004TK-7), Dual-Layer: GPIO + I2C |
| System-Off mit gelatchtem CE | Aktiv | ~15µA, CE-Pin bleibt aktiv → Solar-Laden möglich |
| SOC via INA228 + manuelle Batteriekapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-Ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |
| Auto-Learning (Method 1/2) | Deprecated | Aktuell nicht umgesetzt/aktiv |

---

## Hardware-Architektur

### Komponenten
| Komponente | Funktion | I2C | Pin | Details |
|------------|----------|-----|-----|---------|
| **INA228** | Power Monitor | 0x40 | ALERT→P1.02 (ISR) | 100mΩ Shunt, 1.6A max, Coulomb Counter, BUVL Alert |
| **RV-3028-C7** | RTC | 0x52 | INT→GPIO17 | Countdown-Timer, Wake-up |
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC (IBUS ~±30mA error at low currents; ADC hat VBAT-abhängige Schwellen, siehe [Abschnitt 4](#bq25798-adc-bei-niedrigen-batteriespannungen)) |
| **BQ CE-Pin** | Charge Enable | — | GPIO4 (P0.04) | Via DMN2004TK-7 FET: HIGH=enable, ext. Pull-Up zu VSYS |
| **TPS62840** | Buck Converter | - | EN←VDD (immer an) | 750mA, kein Software-UVLO-Cutoff |
| **DMN2004TK-7** | CE-FET | — | Gate←GPIO4 | N-FET, invertiert CE-Logik: GPIO HIGH → CE LOW → Laden an |

---

## 1. Low-Voltage-Erkennung (INA228 ALERT ISR)

### Implementierung (Rev 1.0 — Flag/Tick-Architektur)
- **Trigger**: INA228 BUVL (Bus Under-Voltage Limit) ALERT auf P1.02
- **ISR**: `BoardConfigContainer::lowVoltageAlertISR()` → setzt `lowVoltageAlertFired = true` (nur Flag, kein FreeRTOS-Aufruf)
- **Verarbeitung**: `tickPeriodic()` prüft Flag im Main-Loop-Kontext → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`
- **Arming**: `armLowVoltageAlert()` wird bei Batterie-Konfiguration aufgerufen (setzt BUVL-Schwelle + aktiviert ISR)

### Low-Voltage-Flow

```
INA228 BUVL Alert (P1.02, FALLING edge)
        │
        ▼
lowVoltageAlertISR()  [ISR-Kontext]
        │ Sets lowVoltageAlertFired = true (volatile Flag)
        ▼
tickPeriodic()  [Main-Loop-Kontext, nächster tick()]
        │ Prüft lowVoltageAlertFired == true
        ▼
board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)
        │ CE HIGH (FET-invertiert: Laden bleibt aktiv in System-Off)
        │ RTC-Wake konfiguriert (LOW_VOLTAGE_SLEEP_MINUTES = 60)
        │ SOC → 0%
        │ GPREGRET2 → LOW_VOLTAGE_SLEEP flag
        ▼
sd_power_system_off() → System-Off (~15µA)
```

### Chemie-spezifische Schwellen (1-Level System, einheitliche 200mV Hysterese)

| Chemie | lowv_sleep_mv (ALERT) | lowv_wake_mv (0% SOC) | Hysterese |
|--------|----------------------|----------------------|-----------|
| **Li-Ion 1S** | 3100 | 3300 | 200mV |
| **LiFePO4 1S** | 2700 | 2900 | 200mV |
| **LTO 2S** | 3900 | 4100 | 200mV |

**Implementierung**: `BoardConfigContainer` — `battery_properties[]` Lookup-Tabelle
- `lowv_sleep_mv` → INA228 BUVL Alert-Schwelle, löst System-Off aus
- `lowv_wake_mv` → RTC-Wake-Schwelle (Early Boot prüft VBAT, entscheidet ob Boot oder erneut Sleep)
- Statische Methoden: `getLowVoltageSleepThreshold(type)`, `getLowVoltageWakeThreshold(type)`

---

## 2. Coulomb Counter & SOC (Ladezustand)

### INA228 Integration
- **Driver**: `lib/Ina228Driver.cpp` (255 Zeilen, vollständig implementiert)
- **Init**: `BoardConfigContainer::begin()` Zeile 592-641
  - 100mΩ Shunt-Kalibrierung
  - CURRENT_LSB = 1A / 524288 ≈ 1.91µA
  - ADC Range ±163.84mV (ADCRANGE=0, optimal für 1A @ 100mΩ)
  - **ADC Averaging**: 64 samples (filters TX voltage peaks)
  - BUVL Alert konfiguriert auf `lowv_sleep_mv` (chemie-spezifisch)

### SOC-Berechnung
**Methode**: `updateBatterySOC()` in `BoardConfigContainer.cpp` Zeile 1337-1418
- **Primary**: Coulomb Counting (INA228 CHARGE Register)
- **Fallback**: Voltage-based SOC via `estimateSOCFromVoltage()` (Zeile 1491-1541)
- **Update-Intervall**: tickPeriodic() ruft auf (60s normal, stündlich im Low-Voltage RTC-Wake)

**Formel**:
```
SOC_delta = charge_delta_mah / capacity_mah × 100%
SOC_new = SOC_old + SOC_delta
```

**Auto-Learning** (deprecated, nicht aktiv):
- Code vorhanden in Zeile 1369-1388
- Trigger: BQ25798 "Charge Done" + Entladung bis Low-Voltage-Sleep
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
   - Trigger: BQ25798 "Charge Done" + Entladung bis Low-Voltage
   - Berechnet neue Kapazität aus Coulomb Counter
   - Speichert automatisch via `saveBatteryCapacity()`

**Laden bei**:
- **Boot-Zeit**: `BoardConfigContainer::begin()` ruft `loadBatteryCapacity()` auf
- **Fallback**: Wenn keine gespeicherte Kapazität vorhanden
- **Validierung**: Range-Check 100-100000mAh

**Persistenz-Eigenschaften**:
- ✅ **Überlebt** Software-Shutdowns (System-Off)
- ✅ **Überlebt** Power-Cycle und Low-Voltage-Recovery
- ✅ **Überlebt** Power-Cycle
- ✅ **Überlebt** Firmware-Update (LittleFS bleibt erhalten)
- ⚠️ **Verloren** bei: Flash-Erase, `rm -rf /prefs/`, Filesystem-Korruption

---

## 3. Tägliche Energiebilanz

### Tracking (7-Day Rolling Window)
**Methode**: `updateDailyBalance()` in `BoardConfigContainer.cpp` Zeile 1420-1489
- **Aufgerufen von**: tickPeriodic()
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

## 4. Solar-Energieverwaltung

### Designprinzip

Der BQ25798 entscheidet **selbst** über PowerGood (PG), ob ein Eingang nutzbar ist.
Der Charger läuft im Always-Active-Modus (HIZ deaktiviert).
Die Firmware überwacht Solar-Status und reaktiviert MPPT bei Bedarf.

Kein INT-Pin-Interrupt — alles läuft über Polling in `runMpptCycle()` (60s Intervall).

### Solar-Checks

`runMpptCycle()` führt bei jedem Zyklus zwei Prüfungen durch:
1. `checkAndFixSolarLogic()` — Reaktiviert MPPT wenn PG=1 (Readback-Check)
2. `updateMpptStats()` — Aktualisiert MPPT-Statistiken für 7-Tage-Durchschnitt

### PFM Forward Mode

- Permanent aktiviert (optimiert für 5-6V Panels)
- PFM verbessert Effizienz bei niedrigen Strömen

### MPPT Recovery

BQ25798 deaktiviert MPPT automatisch bei PG=0. `checkAndFixSolarLogic()` reaktiviert MPPT
wenn PG=1 ist (Readback-Check: nur schreiben wenn tatsächliche Änderung nötig).

### BQ25798 Interrupt Handling

**BQ INT-Pin (GPIO 21)**: Nicht als Interrupt genutzt — `INPUT_PULLUP` gegen Floating.
BQ-Status wird via Polling in `runMpptCycle()` alle 60s geprüft.

**Flag Clearing auf Boot**: `BoardConfigContainer::configureBq()`
- Liest FAULT_STATUS Register (0x20, 0x21) nach Konfiguration
- Vermeidet stale faults von vorherigem Power-Cycle

### Flag/Tick-Architektur

Alle I2C-Operationen laufen im Main-Loop-Kontext über `tickPeriodic()` (aufgerufen von `InheroMr2Board::tick()`). Es gibt keine FreeRTOS-Tasks für I2C-Zugriffe — dadurch entfallen Mutex, Race Conditions und I2C-Bus-Recovery.

**tickPeriodic()** dispatcht periodische Arbeit via `millis()`-Timer:
```
tickPeriodic()  [aufgerufen von tick(), Main-Loop]
  ├─ Low-Voltage Alert Flag prüfen → initiateShutdown()
  ├─ Alle 60s: runMpptCycle()
  │   ├─ checkAndFixSolarLogic() — MPPT-Recovery bei PG=1
  │   └─ updateMpptStats() — MPPT-Statistiken aktualisieren
  ├─ Alle 60s: updateBatterySOC()
  └─ Alle 60min: updateHourlyStats()
```

**Verbleibende FreeRTOS-Tasks** (nur GPIO, kein I2C):
- `heartbeatTask` — blaue LED Blink-Muster
- `ErrorLED` Lambda — rote LED bei fehlenden Komponenten

**Timing Summary**:
- **MPPT Cycle**: 60 Sekunden (via tickPeriodic)
- **SOC Update**: 60 Sekunden (via tickPeriodic)
- **Hourly Stats**: 60 Minuten (via tickPeriodic)

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
- **Low-Voltage Sleep Interval**: `LOW_VOLTAGE_SLEEP_MINUTES` = 60 min (1h)
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
Der ISR setzt nur das Flag, der Idle-Loop prüft es nach `__WFI()` Return.

---

## 7. Energieverwaltungsablauf

### Shutdown-Sequenz (Rev 1.0 — System-Off)
**Methode**: `initiateShutdown()` in `InheroMr2Board.cpp`

**Bei Low-Voltage → System-Off** (~15µA, CE-FET hält Zustand):

**Ablauf:** INA228 ALERT ISR → Flag → tickPeriodic() → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`:

1. **Stop Background Tasks**: `BoardConfigContainer::stopBackgroundTasks()`
   - Stoppt Heartbeat task (einziger verbleibender FreeRTOS-Task mit GPIO)
   - Disarmt INA228 Low-Voltage Alert
   
2. **INA228 ALERT disarmen**: `BoardConfigContainer::disarmLowVoltageAlert()` → ISR detachen, BUVL deaktivieren

3. **CE-Pin HIGH setzen** (FET-invertiert: HIGH = Laden aktiv):
   - `digitalWrite(BQ_CE_PIN, HIGH)` → DMN2004TK-7 ON → CE an GND → Laden aktiv
   - In System-Off werden alle GPIOs High-Z → FET OFF → ext. Pull-Up → CE HIGH → **Laden bleibt aktiv**
   
4. **SX1262 Sleep**: `radio_driver.powerOff()` → `radio.sleep(false)` (Cold Sleep via SPI, ~0.16µA)
   - MUSS vor PE4259-Abschaltung passieren — SPI braucht stabile Stromversorgung

5. **PE4259 RF-Switch aus**: `digitalWrite(SX126X_POWER_EN, LOW)` (VDD abschalten)
   
6. **LEDs aus**: PIN_LED1, PIN_LED2 LOW
   
7. **RTC Wake konfigurieren**: `configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES)` (60 min)
   
8. **Shutdown-Grund speichern**: `NRF_POWER->GPREGRET2 = GPREGRET2_LOW_VOLTAGE_SLEEP | reason`
   
9. **BQ INT-Pin**: `detachInterrupt(BQ_INT_PIN)` — nicht als Interrupt genutzt (Polling), aber Safety-Detach vor System-Off

10. **SOC auf 0% setzen**: `setSOCManually(0.0)` — SOC wird bei Recovery bei 0% starten

11. **System-Off**: `sd_power_system_off()` → nRF52840 System-Off (~15µA)
    - Alle GPIOs werden High-Z → CE-FET: GPIO High-Z → ext. Pull-Up → CE HIGH → **Laden aktiv**
    - RAM-Inhalt geht verloren (168h-Statistiken, SOC, etc.)
    - RTC-Interrupt auf GPIO17 weckt System nach Timer-Ablauf

**Warum System-Off statt System ON Idle (v0.2)?**
- Rev 1.0 nutzt DMN2004TK-7 FET für CE-Pin → In System-Off floaten alle GPIOs → ext. Pull-Up → CE HIGH → Laden aktiv
- System ON Idle war nur nötig um GPIO-Latches für CE-Pin LOW zu halten (v0.2 ohne FET)
- System-Off: **~15µA** vs. System ON Idle: **~0.6mA** → **40× effizienter**

**168h-Statistiken gehen bei System-Off verloren** — es existiert kein Persistenzmechanismus für die Ring-Buffer-Daten. Nach Recovery starten die Statistiken bei Null.

### Wake-up-Check (Anti-Motorboating)
**Methode**: `InheroMr2Board::begin()`

Der Code prüft `GPREGRET2` für den Shutdown-Grund und die Batteriespannung für Wake-up-Entscheidungen.

**2 Fälle**:

**Case 1: Wake from Low-Voltage Sleep** (`GPREGRET2 & GPREGRET2_LOW_VOLTAGE_SLEEP`)
```cpp
// InheroMr2Board::begin() — Early Boot Check
if (NRF_POWER->GPREGRET2 & GPREGRET2_LOW_VOLTAGE_SLEEP) {
  uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR);
  uint16_t wake_threshold = BoardConfigContainer::getLowVoltageWakeThreshold(batType);
  
  if (vbat_mv < wake_threshold) {
    // Spannung noch zu niedrig → sofort wieder System-Off
    configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES);
    sd_power_system_off();  // Bleibt in Low-Voltage-Sleep-Zyklus
  }
  // Spannung OK → Low-Voltage-Recovery markieren, SOC auf 0%, normaler Boot
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
  boardConfig.setLowVoltageRecovery();
}
```

**Case 2: Normal ColdBoot** (Power-On, Reset button, voltage OK)
```cpp
else {
  // Continue normal boot
  // INA228 und alle anderen Komponenten werden initialisiert
}
```

**Direct ADC Read** (boardConfig noch nicht ready):
```cpp
// Must read directly from INA228 ADC registers (24-bit, ±0.1% accuracy)
uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR);
```

**Voltage Thresholds** (Chemistry-Specific, 1-Level System):
| Chemistry | lowv_sleep_mv (ALERT) | lowv_wake_mv (Recovery) | Hysteresis |
|-----------|----------------------|------------------------|------------|
| Li-Ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |

**Anti-Motorboating**: Der Early-Boot-Check in `begin()` verhindert, dass das System bei knapper Spannung immer wieder bootet und sofort abstürzt. Erst wenn VBAT über `lowv_wake_mv` liegt, wird normal gebootet.

**Stromverbrauch im System-Off (Low-Voltage Sleep)**:
- **Gesamt: ~15µA** (nRF52840 System-Off + RTC + quiescent currents)
- CE-FET: GPIO High-Z → ext. Pull-Up → CE HIGH → **Solar-Laden aktiv**
- Verglichen mit v0.2 System ON Idle (~0.6mA): **40× effizienter**

---

## 8. INA228 ALERT-Pin (Rev 1.0)

### Verdrahtung
**Pin**: INA228 ALERT → P1.02 (nRF52840 GPIO, mit ext. Pull-Up)
**TPS62840 EN**: An VDD gebunden (immer an) — kein Hardware-UVLO-Cutoff

### Funktionsweise (Rev 1.0)
Im Gegensatz zu v0.2 (ALERT→TPS EN, latched hardware cutoff) wird der ALERT-Pin in Rev 1.0 als
**Software-Interrupt** genutzt:

1. `armLowVoltageAlert()` konfiguriert INA228 BUVL (Bus Under-Voltage Limit) auf `lowv_sleep_mv`
2. ALERT feuert als FALLING-Edge-Interrupt auf P1.02
3. ISR (`lowVoltageAlertISR()`) setzt `lowVoltageAlertFired = true` (nur Flag, kein FreeRTOS-Aufruf)
4. `tickPeriodic()` prüft Flag im nächsten Main-Loop-Tick und ruft `initiateShutdown()` → System-Off

**Kein Latch-Problem**: Da der ALERT nicht an TPS62840 EN geht, gibt es kein latched-off-Verhalten.
Das System kann nach RTC-Wake normal booten und die Spannung in `begin()` prüfen.

### Alert-Schwellen
Identisch mit den Low-Voltage-Schwellen (siehe Section 1):
| Chemie | BUVL Threshold |
|--------|---------------|
| Li-Ion 1S | 3100mV |
| LiFePO4 1S | 2700mV |
| LTO 2S | 3900mV |

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

## 10. (Entfernt — UVLO CLI war v0.2-Feature, in Rev 1.0 nicht mehr vorhanden)

---

## 11. BQ25798 CE-Pin Safety (Rev 1.0 — FET-invertiert)

### Problem
Der BQ25798 startet mit Default-Konfiguration (1S Li-Ion, 4.2V Ladespannung). Wenn eine LiFePO4-Batterie (3.5V max) verbunden ist und der RAK noch nicht gebootet hat, würde der BQ25798 die Batterie überladen → **Brandgefahr**.

### Hardware-Design (Rev 1.0 — FET-invertiert)
- **Pin**: `BQ_CE_PIN` = GPIO 4 (P0.04 / WB_IO4)
- **DMN2004TK-7 N-FET**: Gate ← GPIO4, Drain → CE, Source → GND
- **Externer Pull-Up**: 10kΩ zu VSYS → CE HIGH = **Laden aktiv** (wenn FET OFF)
- **GPIO HIGH** → FET ON → CE an GND → Laden aktiv (gleiche Wirkung wie Pull-Up)
- **GPIO LOW** → FET OFF → ext. Pull-Up → CE HIGH → Laden aktiv
- **GPIO High-Z** (System-Off) → FET OFF → ext. Pull-Up → CE HIGH → **Laden aktiv**

**Kernpunkt Rev 1.0**: Egal ob GPIO HIGH, LOW oder High-Z — CE ist immer HIGH → **Laden immer aktiv** (bei bekannter Chemie). Die FET-Schaltung stellt sicher, dass Solar-Laden auch in System-Off funktioniert.

### 3-Schicht-Sicherung (Rev 1.0)

| Schicht | Ort | Mechanismus | Wann |
|---|---|---|---|
| **1. Hardware (passiv)** | Pull-Up + FET | CE HIGH wenn RAK stromlos (FET OFF, Pull-Up) → **Laden aktiv** | Immer |
| **2. Early Boot** | `InheroMr2Board::begin()` | CE bleibt HIGH via Pull-Up, Laden noch nicht konfiguriert | Vor I2C-Init |
| **3. Chemie-Konfiguration** | `configureChemistry()` | CE HIGH explizit (FET-invertiert) + I2C Register bei bekannter Chemie | Nach BQ25798-Konfiguration |

### Dual-Layer Safety (Hardware + Software)

```cpp
// In configureChemistry() — nach BQ25798 Register-Konfiguration:
bq.setChargeEnable(props->charge_enable);     // Software-Schicht (I2C Register)
#ifdef BQ_CE_PIN
  pinMode(BQ_CE_PIN, OUTPUT);
  // Rev 1.0 FET-invertiert: HIGH → FET ON → CE LOW → Laden aktiv (BQ25798: CE active-low)
  // Aber ext. Pull-Up → CE HIGH → Laden auch aktiv (BQ25798 CE ist active-low internally)
  digitalWrite(BQ_CE_PIN, props->charge_enable ? HIGH : LOW);  // HIGH=an, LOW=aus (FET-invertiert)
#endif
```

- `charge_enable` ist Teil der `BatteryProperties`-Tabelle
- `BAT_UNKNOWN` → `charge_enable = false` → CE HIGH + Register disabled
- Bekannte Chemie → `charge_enable = true` → CE LOW + Register enabled

### Verhalten im System-Off (Rev 1.0)

In Rev 1.0 wird **System-Off** verwendet (via `initiateShutdown()`):
- Alle GPIOs werden High-Z → DMN2004TK-7 FET OFF → ext. Pull-Up → CE HIGH → **Laden aktiv**
- BQ25798 MPPT/CC/CV läuft autonom in Hardware → Solar-Laden möglich
- Stromverbrauch: **~15µA** (nRF52840 System-Off + RTC + quiescent currents)

| Zustand | CE-Pin | Laden | Solar-Recovery |
|---|---|---|---|
| RAK stromlos (kein Akku) | HIGH (Pull-Up) | Aktiv (aber kein Akku) | N/A |
| Early Boot | HIGH (Pull-Up) | Aktiv (BQ Default-Config) | Ja |
| BAT_UNKNOWN | LOW (FET OFF, kein Pull-Up-Override) | Gesperrt (I2C Register) | Nein |
| Chemie konfiguriert | HIGH (FET ON) | **Aktiv** | **Ja** |
| System-Off (Low-Voltage) | HIGH (Pull-Up, FET OFF/High-Z) | **Aktiv** | **Ja** |

---

## 12. Statistik-Persistenz

### Aktueller Stand

Die 168h-Ringpuffer-Statistiken (Coulomb Counter, MPPT-Daten, SOC-Zustand) sind **nur im RAM** gespeichert und gehen bei jedem Reboot verloren — egal ob System-Off oder Cold Boot. Es existiert kein Persistenzmechanismus (weder `.noinit`-Section noch LittleFS-Snapshot).

**Persistente Daten** (überleben Reboots via LittleFS):
- Batterietyp (`batType`)
- Batteriekapazität (`batCap`)
- INA228 Kalibrierung (`ina228Cal`)
- NTC Kalibrierung (`tcCal`)
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
                # Code: InheroMr2Board.cpp

board.conf      # Gesamte Konfiguration
                # Output: "B:liion1s F:0% M:1 I:500 Vco:4.10"
                # Code: InheroMr2Board.cpp - getCustomGetter()

board.leds      # LED enable status
                # Output: "LEDs: ON (Heartbeat + BQ Stat)"
                # Code: InheroMr2Board.cpp - getCustomGetter()
```

### Setter (Implemented)
```bash
set board.batcap <mAh>      # Set battery capacity
                            # Range: 100-100000 mAh
                            # Example: set board.batcap 2200
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.bat <type>        # Set battery chemistry
                            # Options: liion1s | lifepo1s | lto2s | none
                            # Code: InheroMr2Board.cpp - setCustomSetter()

set board.imax <mA>         # Set max charge current
                            # Range: 50-1500 mA
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
| Methode | Datei | Funktion |
|---------|-------|----------|
| `begin()` | InheroMr2Board.cpp | Board-Initialisierung, Wake-up-Check, Early-Boot Low-Voltage Check |
| `initiateShutdown()` | InheroMr2Board.cpp | System-Off Shutdown (aufgerufen von tickPeriodic nach ALERT) |
| `configureRTCWake()` | InheroMr2Board.cpp | RTC Countdown Timer |
| `rtcInterruptHandler()` | InheroMr2Board.cpp | RTC INT ISR (setzt Flag) |
| `queryBoardTelemetry()` | InheroMr2Board.cpp | CayenneLPP Telemetry Collection |
| `getLowVoltageSleepThreshold()` | InheroMr2Board.cpp | Chemistry-specific Sleep Voltage (INA228 ALERT) |
| `getLowVoltageWakeThreshold()` | InheroMr2Board.cpp | Chemistry-specific Wake Voltage (0% SOC) |
| `armLowVoltageAlert()` | BoardConfigContainer.cpp | INA228 BUVL Alert armen + ISR registrieren |
| `disarmLowVoltageAlert()` | BoardConfigContainer.cpp | INA228 Alert disarmen + ISR detachen |
| `lowVoltageAlertISR()` | BoardConfigContainer.cpp | ISR: setzt lowVoltageAlertFired Flag (geprüft in tickPeriodic) |
| `tickPeriodic()` | BoardConfigContainer.cpp | Main-Loop Dispatch: MPPT (60s), SOC (60s), Hourly (60min), Low-V Check |
| `runMpptCycle()` | BoardConfigContainer.cpp | Einzelner MPPT-Zyklus (Solar-Checks, MPPT-Recovery) |
| `updateBatterySOC()` | BoardConfigContainer.cpp | Coulomb Counter SOC Calculation |
| `updateDailyBalance()` | BoardConfigContainer.cpp | 7-Day Energy Balance Tracking |
| `calculateTTL()` | BoardConfigContainer.cpp | Time To Live Forecast |
| `Ina228Driver::begin()` | lib/Ina228Driver.cpp | 100mΩ Calibration, ADC Config |
| `Ina228Driver::readVBATDirect()` | lib/Ina228Driver.cpp | Static Early-Boot VBAT Read |

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

### Szenario A: Normale Entladung (Low-Voltage System-Off) - Li-Ion
```
t=0:      VBAT = 3.7V → Normal (60s checks, Coulomb Counter läuft)
          Daily balance: Today +150mAh SOLAR
          
t=+1h:    VBAT = 3.5V → Normal (INA228 ALERT nicht getriggert)
          SOC: 45%
          
t=+2h:    VBAT = 3.15V → INA228 ALERT feuert (< 3100mV lowv_sleep_mv)
          - lowVoltageAlertISR() → setzt lowVoltageAlertFired Flag
          - tickPeriodic() erkennt Flag im nächsten tick()
          - board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)
          - CE HIGH (FET-invertiert → Laden bleibt aktiv)
          - RTC: Wake in 1h (LOW_VOLTAGE_SLEEP_MINUTES = 60)
          - SOC → 0%
          - sd_power_system_off() → System-Off (~15µA)
          
t=+3h:    RTC weckt → System bootet → Early Boot Check
          - Ina228Driver::readVBATDirect() → VBAT = 3.15V
          - VBAT < lowv_wake_mv (3300mV) → sofort wieder System-Off
          - configureRTCWake(60) + sd_power_system_off()
          
t=+4h:    RTC weckt → System bootet → Early Boot Check
          - VBAT = 3.20V → noch unter 3300mV → wieder System-Off
          
t=+5h:    RTC weckt → System bootet → Early Boot Check
          - VBAT = 3.45V (Solar-Recovery!)
          - VBAT > lowv_wake_mv (3300mV) → normaler Boot
          - Low-Voltage-Recovery markiert, SOC bei 0%
          - Coulomb Counter startet neu
          - Daily balance baut sich neu auf
```

### Szenario B: Kritische Entladung (Rev 1.0 — kein Hardware-UVLO)
```
In Rev 1.0 gibt es kein Hardware-UVLO (TPS62840 EN an VDD, immer an).
Der INA228 ALERT auf P1.02 dient als Software-Interrupt für System-Off.

t=0:      VBAT = 3.15V → INA228 ALERT feuert
          - tickPeriodic() → initiateShutdown()
          - System-Off (~15µA), CE aktiv, RTC-Wake 1h
          
t=+1h:    RTC-Wake → Early Boot → VBAT = 3.05V (noch unter 3300mV)
          - Sofort wieder System-Off (CE bleibt aktiv → Solar-Laden möglich)
          
t=+2h:    RTC-Wake → VBAT = 2.95V (weiter gesunken, kein Solar)
          - Sofort wieder System-Off
          - Board pendelt weiter mit ~15µA + stündlichem Boot (~0.03mAh)
          
t=+∞:     Bei ~15µA kann die Batterie monatelang überleben
          - Sobald Solar verfügbar → VBAT steigt → normaler Boot bei >3300mV
          - KEIN Latching: System kann IMMER von selbst recovern
          
Vergleich v0.2: Dort hätte INA228 ALERT → TPS EN=LOW das System
permanent abgeschaltet (latched) — manuelles Eingreifen nötig.
Rev 1.0 ist hier resilienter — Solar-Recovery immer möglich.
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

### v3.2 - 26. März 2026 (Rev 1.1 Cleanup)
- ❌ **Entfernt**: HIZ-Gate State Machine (`HizGateState`, `checkAndFixPgoodStuck()`, `readVbusInHiz()`, `canSafelyEnterHiz()`, `toggleHizAndCheck()`) — Rev 1.1 PCB stabil ohne HIZ-Gating
- ❌ **Entfernt**: PFM get/set CLI (`set board.pfm`, `get board.pfm`, `setPFMEnabled()`, `getPFMEnabled()`, `loadPfmEnabled()`) — PFM permanent aktiviert
- ❌ **Entfernt**: INA228 Offset-Kalibrierung (`set board.iboffset`, `get board.iboffset`, `setIna228CurrentOffset()`, `getIna228CurrentOffset()`, `performIna228OffsetCalibration()`, `_current_offset_mA`)
- ❌ **Entfernt**: Getter `get board.hwver`, `get board.diag`, `get board.energy`
- ❌ **Entfernt**: `getDetailedDiagnostics()` (verwaiste Funktion)
- ✅ **Neu**: `set board.bat none` für BAT_UNKNOWN (Laden deaktiviert)
- 🔧 **Geändert**: `set board.imax` Obergrenze von 1000 auf 1500 mA erhöht
- 📝 Dokumentation komplett an Rev 1.1 Code-Stand angepasst

### v3.1 - 15. März 2026 (Flag/Tick-Architektur)
- 🔧 **Flag/Tick-Pattern**: Alle I2C-Operationen aus FreeRTOS-Tasks in `tickPeriodic()` (Main-Loop) verlagert
- ❌ **Entfernt**: `solarMpptTask()`, `socUpdateTask()` — ersetzt durch `tickPeriodic()` + `runMpptCycle()`
- ❌ **Entfernt**: `I2CMutex.h`, `g_i2c_mutex`, `i2c_mutex_init()` — kein Mutex mehr nötig (single-threaded I2C)
- ❌ **Entfernt**: `recoverI2CBus()`, `areBackgroundTasksAlive()` — I2C-Bus-Recovery überflüssig ohne Tasks
- ❌ **Entfernt**: `xTaskNotifyFromISR()` aus `lowVoltageAlertISR()` — nur noch volatile Flag
- ❌ **Entfernt**: ADC_CONFIG Race-Condition Check in `begin()` — keine konkurrierenden Tasks mehr
- 🔧 **`stopBackgroundTasks()`**: Nur noch Heartbeat-Task + Alert-Disarm (keine MPPT/SOC-Tasks mehr)
- 🔧 **`InheroMr2Board::tick()`**: Vereinfacht auf RTC-Clear + `tickPeriodic()` + `feedWatchdog()`
- 📝 Dokumentation für Flag/Tick-Architektur aktualisiert

### v3.0 - 02. Juni 2026 (Rev 1.0 Architektur)
- 🔧 **Architektur-Umstellung**: System ON Idle (v0.2) → System-Off (Rev 1.0) — 40× effizienter (~15µA vs. ~0.6mA)
- 🔧 **INA228 ALERT auf P1.02**: ISR-basierte Low-Voltage-Erkennung statt Software-Polling + Hardware-UVLO
- 🔧 **TPS62840 EN an VDD**: Immer an, kein Hardware-UVLO-Cutoff — System recovert immer selbstständig
- 🔧 **CE-Pin FET-invertiert**: DMN2004TK-7 N-FET ermöglicht Solar-Laden in System-Off (GPIO High-Z → Pull-Up → CE HIGH → Laden aktiv)
- 🔧 **1-Stufen-Schwellenmodell**: lowv_sleep_mv / lowv_wake_mv mit einheitlicher 200mV Hysterese für alle Chemien
- ❌ **Entfernt**: UVLO CLI (board.uvlo getter/setter), uvloEn Preference, runVoltageMonitor(), voltageMonitorTask()
- ❌ **Entfernt**: Hardware-UVLO (INA228 Alert → TPS62840 EN), Danger Zone System ON Idle, __WFI Loop
- 📝 Dokumentation komplett für Rev 1.0 überarbeitet

### v2.2 - 25. Februar 2026 (Code-Fix + Dokumentation an Code-Stand angepasst)
- 🐛 **Code-Fix**: `runVoltageMonitor()` delegiert jetzt an `board.initiateShutdown(LOW_VOLTAGE)` statt inline `sd_power_system_off()` — CE-Pin bleibt LOW, Solar-Laden in Danger Zone möglich
- 📝 `.noinit Stats Preservation` entfernt — Feature existiert nicht im Code (kein Linker-Script, keine Structs/Funktionen)
- 📝 Danger Zone korrekt als System ON Idle dokumentiert (CE-Pin LOW, GPIO-Latches aktiv)
- 📝 `initiateShutdown()` als aktiver Code-Pfad dokumentiert (aufgerufen von `runVoltageMonitor()`)
- 📝 RTC-Wake-Intervall von 12h auf 6h korrigiert, dann auf 1h reduziert (Code: `DANGER_ZONE_SLEEP_MINUTES = 60`)
- 📝 `voltageMonitorTask` als Stub dokumentiert, Verweis auf `socUpdateTask` + `runVoltageMonitor()`
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
- ✅ CLI Commands (soc, batcap)
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

*Letzte Aktualisierung: 26. März 2026*
*Status: ✅ Dokumentation an Rev 1.1 Code-Stand angepasst (PFM permanent, HIZ-Gate/IBCAL/Diag entfernt, imax 50-1500mA)*
