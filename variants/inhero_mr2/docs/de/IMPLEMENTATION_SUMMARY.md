# Inhero MR-2 Energieverwaltung - Implementierungs-Dokumentation (Rev 1.1)

> 🇬🇧 [English version](../IMPLEMENTATION_SUMMARY.md)

## Inhaltsverzeichnis

- [Überblick](#überblick)
- [Hardware-Architektur](#hardware-architektur)
- [1. Low-Voltage-Erkennung (INA228 ALERT ISR)](#1-low-voltage-erkennung-ina228-alert-isr)
- [2. Coulomb Counter & SOC (Ladezustand)](#2-coulomb-counter--soc-ladezustand)
- [3. Tägliche Energiebilanz](#3-tägliche-energiebilanz)
- [4. Solar-Energieverwaltung & Interrupt-Loop-Vermeidung](#4-solar-energieverwaltung--interrupt-loop-vermeidung)
  - [BQ25798 ADC bei niedrigen Batteriespannungen](#bq25798-adc-bei-niedrigen-batteriespannungen)
  - [JEITA-WARM-Zone & VBAT_OVP-Vermeidung](#jeita-warm-zone--vbat_ovp-vermeidung)
- [5. Time-To-Live (TTL)-Prognose](#5-time-to-live-ttl-prognose)
- [6. RTC-Wakeup-Management](#6-rtc-wakeup-management)
- [7. Energieverwaltungsablauf](#7-energieverwaltungsablauf)
- [Siehe auch](#siehe-auch)

> Diese Dokumentation beschreibt die Energieverwaltungs-Implementierung für das Inhero MR-2 Board.
> Hardware Rev 1.1: INA228 ALERT auf P1.02, TPS62840 EN via 3.3V_off-Schalter, CE-Pin via DMN2004TK-7 FET (invertiert).

---

## Überblick

Das System kombiniert **INA228 ALERT-basierte Low-Voltage-Erkennung** + **System Sleep mit GPIO-Latch** + **Coulomb Counter** + **tägliche Energiebilanz** + **CE-Pin FET-Safety** für maximale Energie-Effizienz:

1. **INA228 ALERT ISR** (P1.02) - Low-Voltage-Erkennung via Hardware-Interrupt
2. **System Sleep mit GPIO-Latch** (< 500µA) mit RTC-Wake - Minimaler Stromverbrauch bei Low-Voltage
3. **CE-Pin FET-Safety** (DMN2004TK-7) - Invertierte Logik, Solar-Laden in System Sleep möglich
4. **Coulomb Counter** (INA228) - Echtzeit-SOC-Tracking
5. **Tägliche Energiebilanz** (7-Tage rolling) - Solar vs. Batterie
6. **RTC-Wakeup-Management** (RV-3028-C7) - Periodische Recovery-Checks

### Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| INA228 ALERT → Low-Voltage System Sleep | Aktiv | ISR auf P1.02 → volatile Flag → tickPeriodic() → System Sleep mit GPIO-Latch + RTC-Wake |
| RTC-Wakeup (Low-Voltage-Recovery) | Aktiv | 60 min (periodisch) |
| BQ CE-Pin Safety (FET-invertiert) | Aktiv | GPIO HIGH → FET ON → CE LOW → Laden an (BQ25798 CE active-low), Dual-Layer: GPIO + I2C |
| System Sleep mit gelatchtem CE | Aktiv | < 500µA, GPIO4-Latch HIGH erhalten → FET ON → CE LOW → Solar-Laden möglich |
| SOC via INA228 + manuelle Batteriekapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-Ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |


---

## Hardware-Architektur

### Komponenten
| Komponente | Funktion | I2C | Pin | Details |
|------------|----------|-----|-----|---------|
| **RAK4630** | Core Module | — | — | nRF52840 SoC + SX1262 LoRa Transceiver |
| **INA228** | Power Monitor | 0x40 | ALERT→P1.02 (ISR) | 100mΩ Shunt, 1.6A max, Coulomb Counter, BUVL Alert |
| **RV-3028-C7** | RTC | 0x52 | INT→GPIO17 | Zeitbasis, Countdown-Timer, Wake-up |
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC (IBUS ~±30mA error at low currents; ADC hat VBAT-abhängige Schwellen, siehe [Abschnitt 4](#bq25798-adc-bei-niedrigen-batteriespannungen)) |
| **BQ CE-Pin** | Charge Enable | — | GPIO4 (P0.04) | Via DMN2004TK-7 FET: GPIO HIGH → FET ON → CE LOW → Laden an (BQ25798 CE active-low) |
| **TPS62840** | Buck Converter | - | EN via 3.3V_off-Schalter | 750mA, 3.3V Rail |
| **DMN2004TK-7** | CE-FET | — | Gate←GPIO4 (ext. Pull-Down) | N-FET, Drain→CE, Source→GND. GPIO HIGH → FET ON → CE LOW → Laden an. Pull-Down zieht Gate LOW wenn floating. |
| **SS34** | USB→VBUS-Diode | — | — | Schottky-Diode: VBUS-USB → VBUS-BQ (Solareingang). USB-C CC1/CC2 über 4,7kΩ auf GND (USB-Sink). **⚠ Solar-Kurzschluss shortet auch VBUS-USB.** |

---

## 1. Low-Voltage-Erkennung (INA228 ALERT ISR)

### Implementierung (Rev 1.1 — Flag/Tick-Architektur)
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
        │ CE gelatcht HIGH (GPIO-Latch erhalten → FET ON → CE LOW → Laden bleibt AN)
        │ RTC-Wake konfiguriert (LOW_VOLTAGE_SLEEP_MINUTES = 60)
        │ SOC → 0%
        │ GPREGRET2 → LOW_VOLTAGE_SLEEP flag
        ▼
sd_power_system_off() → System Sleep mit GPIO-Latch (< 500µA)
```

### Chemie-spezifische Schwellen (1-Level System, einheitliche 200mV Hysterese)

| Chemie | lowv_sleep_mv (ALERT) | lowv_wake_mv (0% SOC) | Hysterese |
|--------|----------------------|----------------------|-----------|
| **Li-Ion 1S** | 3100 | 3300 | 200mV |
| **LiFePO4 1S** | 2700 | 2900 | 200mV |
| **LTO 2S** | 3900 | 4100 | 200mV |
| **Na-Ion 1S** | 2500 | 2700 | 200mV |

**Implementierung**: `BoardConfigContainer` — `battery_properties[]` Lookup-Tabelle
- `lowv_sleep_mv` → INA228 BUVL Alert-Schwelle, löst System Sleep aus
- `lowv_wake_mv` → RTC-Wake-Schwelle (Early Boot prüft VBAT, entscheidet ob Boot oder erneut Sleep)
- Statische Methoden: `getLowVoltageSleepThreshold(type)`, `getLowVoltageWakeThreshold(type)`

---

## 2. Coulomb Counter & SOC (Ladezustand)

### INA228 Integration
- **Driver**: `lib/Ina228Driver.cpp`
- **Init**: `BoardConfigContainer::begin()`
  - 100mΩ Shunt-Kalibrierung
  - CURRENT_LSB = 1A / 524288 ≈ 1.91µA
  - ADC Range ±163.84mV (ADCRANGE=0, optimal für 1A @ 100mΩ)
  - **ADC Averaging**: 64 samples (filters TX voltage peaks)
  - BUVL Alert konfiguriert auf `lowv_sleep_mv` (chemie-spezifisch)

### SOC-Berechnung
**Methode**: `updateBatterySOC()` in `BoardConfigContainer.cpp`
- **Primary**: Coulomb Counting (INA228 CHARGE Register)
- **Update-Intervall**: tickPeriodic() ruft auf (60s normal, stündlich im Low-Voltage RTC-Wake)

**Formel**:
```
SOC_delta = charge_delta_mah / capacity_mah × 100%
SOC_new = SOC_old + SOC_delta
```

**Auto-Sync**: Bei BQ25798 "Charge Done" wird SOC auf 100% gesetzt.

### Kapazitäts-Management

#### Konfiguration erforderlich
Die Akkukapazität **muss manuell gesetzt werden**, da sie in der Praxis stark variiert:
- **CLI-Befehl**: `set board.batcap <mAh>`
- **Erlaubter Bereich**: 100-100000mAh

**Wichtig**: Ohne korrekte Kapazität sind SOC% und TTL-Berechnungen ungenau!

#### Persistenz-Mechanik
**Storage Path**: `/prefs/battery_capacity` (LittleFS)
**Save-Methode**: `saveBatteryCapacity()` in `BoardConfigContainer.cpp`
**Load-Methode**: `loadBatteryCapacity()`

**Speichern bei**:
1. **Manuelles Setzen**: CLI-Befehl `set board.batcap <mAh>`
   - Schreibt sofort in LittleFS
   - Aktualisiert `batteryStats.capacity_mah`

**Laden bei**:
- **Boot-Zeit**: `BoardConfigContainer::begin()` ruft `loadBatteryCapacity()` auf
- **Fallback**: Wenn keine gespeicherte Kapazität vorhanden
- **Validierung**: Range-Check 100-100000mAh

**Persistenz-Eigenschaften**:
- ✅ **Überlebt** Software-Shutdowns (System Sleep)
- ✅ **Überlebt** Power-Cycle und Low-Voltage-Recovery
- ✅ **Überlebt** Power-Cycle
- ✅ **Überlebt** Firmware-Update (LittleFS bleibt erhalten)
- ⚠️ **Verloren** bei: Flash-Erase, `rm -rf /prefs/`, Filesystem-Korruption

---

## 3. Tägliche Energiebilanz

### Tracking (7-Day Rolling Window)
**Methode**: `updateDailyBalance()` in `BoardConfigContainer.cpp`
- **Aufgerufen von**: tickPeriodic()
- **Frequenz**: Bei Tag-Wechsel (RTC time % 86400 < 60)

**Datenstruktur**: `BatterySOCStats.daily_stats[7]`
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
1. `checkAndFixSolarLogic()` — PG-Stuck Recovery + MPPT-Reaktivierung
2. `updateMpptStats()` — Aktualisiert MPPT-Statistiken für 7-Tage-Durchschnitt

### PFM Forward Mode

- Permanent aktiviert
- PFM verbessert Effizienz bei niedrigen Solarströmen

### MPPT Recovery + PG-Stuck

`checkAndFixSolarLogic()` behandelt zwei Szenarien:

**PG=1**: MPPT-Reaktivierung — BQ25798 deaktiviert MPPT automatisch bei Faults.
Readback-Check: nur schreiben wenn tatsächliche Änderung nötig.

**PG=0 + VBUS ≥ 4.5V**: PG-Stuck Recovery — Panel liefert Spannung, aber BQ hat die
Inputquelle nicht qualifiziert (typisch bei langsamem Sonnenaufgang). HIZ-Toggle erzwingt
neue Input-Qualifikation. 5-Minuten-Cooldown verhindert übermäßiges Toggling.
Konstante: `PG_STUCK_VBUS_THRESHOLD_MV = 4500` in BoardConfigContainer.h

### BQ25798 Interrupt Handling

**BQ INT-Pin (GPIO 21)**: Nicht als Interrupt genutzt — `INPUT_PULLUP` gegen Floating.
BQ-Status wird via Polling in `runMpptCycle()` alle 60s geprüft.

**Flag Clearing auf Boot**: `BoardConfigContainer::configureBq()`
- Liest FAULT_STATUS Register (0x20, 0x21) nach Konfiguration
- Vermeidet stale faults von vorherigem Power-Cycle

### Flag/Tick-Architektur

Alle I2C-Operationen laufen im Main-Loop-Kontext über `tickPeriodic()` (aufgerufen von `InheroMr2Board::tick()`). Es gibt keine FreeRTOS-Tasks für I2C-Zugriffe — dadurch entfallen Mutex und Race Conditions.

**I2C Bus Recovery** (in `InheroMr2Board::begin()`): Nach OTA/Warm-Reset kann ein I2C-Slave SDA festhalten. Vor `Wire.begin()` werden bis zu 9 SCL-Pulse + STOP-Condition generiert um den Bus freizugeben.

**tickPeriodic()** dispatcht periodische Arbeit via `millis()`-Timer:
```
tickPeriodic()  [aufgerufen von tick(), Main-Loop]
  ├─ Low-Voltage Alert Flag prüfen → initiateShutdown()
  ├─ Alle 60s: runMpptCycle()
  │   ├─ checkAndFixSolarLogic() — PG-Stuck Recovery (HIZ-Toggle) + MPPT-Recovery
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

### JEITA-WARM-Zone & VBAT_OVP-Vermeidung

#### Problem: Standard-JEITA-Konfiguration + Inhero-Teiler

Das Inhero MR2 verwendet einen nicht-standardmäßigen NTC-Spannungsteiler (RT1=5,6 kΩ Pullup an REGN, RT2=27 kΩ parallel zu GND) anstelle des TI-Referenzdesigns (5,24 kΩ / 30,31 kΩ). Dies verschiebt TS-Schwellen um einen **temperaturabhängigen** Betrag nach unten: ~5–6 °C im Kaltbereich (wo der NTC-Widerstand groß relativ zu RT2 ist und den Teiler-Unterschied verstärkt) und ~2–3 °C im Warm-/Hot-Bereich.

Mit den BQ25798 POR-Defaults (`TS_WARM = 45°C`, `JEITA_VSET = VREG−400mV`, `EN_AUTO_IBATDIS = 1`) verursachte dies eine kritische Fehlerkette bei moderaten Temperaturen (~42 °C):

```
42°C Umgebung → TS = 44,65% REGN (unter VT3_FALL = 44,8%)
  → BQ tritt in WARM-Zone ein
  → JEITA_VSET reduziert VREG: 3,5V − 400mV = 3,1V (LiFePO4)
  → Batterie bei 3,47V > 104% × 3,1V = 3,224V → VBAT_OVP ausgelöst
  → Wandler stoppt, EN_AUTO_IBATDIS zieht IBAT_LOAD = 30mA aus Batterie
  → Gesamtverbrauch: −11mA (System) + −30mA (IBAT_LOAD) = −41mA
  → Recovery erfordert VBAT < 102% × 3,1V = 3,162V → stundenlanger Batterieverbrauch
```

#### Fix: Drei Register-Einstellungen in `configureBaseBQ()`

| Einstellung | Register | Wert | Wirkung |
|-------------|----------|------|---------|
| `setTsWarm(BQ25798_TS_WARM_55C)` | Charger Control 2 | 55 °C (37,7% REGN) | WARM-Zone beginnt bei ~52 °C (Inhero), nicht ~42 °C |
| `setJeitaVSet(BQ25798_JEITA_VSET_UNCHANGED)` | Charger Control 5 | UNCHANGED | Keine VREG-Reduktion in WARM — verhindert VBAT_OVP |
| `JEITA_ISETH` (POR-Default beibehalten) | NTC Control 0, Bits 4:3 | 11b = ICHG unchanged | Keine Ladestrom-Reduktion in WARM |
| `setAutoIBATDIS(false)` | Charger Control 0, Bit 7 | 0 | Deaktiviert 30-mA-Batterieentladung bei OVP |

> **Ergebnis:** Mit JEITA_VSET=UNCHANGED und JEITA_ISETH=ICHG unchanged ist die WARM-Zone (T3–T5) effektiv neutralisiert. Die Ladung läuft mit voller Spannung und vollem Strom bis T-Hot (~58 °C), wo die Ladung komplett gesperrt wird.

#### TS-Schwellenvergleich

| Zonengrenze | BQ-Register | % REGN | TI-Referenz (°C) | Inhero MR2 (°C) | Shift |
|-------------|-------------|--------|-------------------|------------------|-------|
| VT1 (Cold) | — | 72,0% | +3,7 | −2,0 | −5,7 °C |
| VT2 (Cool) | — | 69,8% | +7,9 | +2,8 | −5,1 °C |
| VT3 (Warm) | TS_WARM=55°C | 37,7% | +54,5 | +52,2 | −2,3 °C |
| VT5 (Hot) | — | 34,2% | +59,9 | +57,7 | −2,2 °C |

> NTC-Modelle: 103AT (B25/50=3435) für TI-Referenz, NCP15XH103F03RC (B25/85=3380) für Inhero. Typische %REGN-Werte aus BQ25798-Datenblatt.

#### Diagnoseverifikation

Der `bqdiag`-Befehl zeigt den JEITA-Status im TS-Feld:

```
bqdiag-Ausgabe: PG / CC TS:OK CE:1 HIZ:0 F:00/00 S:00.00.01.00.00 N:14 VREG:3500mV
                              ^^^^
                              OK = normal, WARM/HOT/COLD = JEITA-Zone aktiv
```

- `TS:WARM` + `VBAT_OVP` im Fault-Register = der ursprüngliche Bug (behoben durch obige Einstellungen)
- `TS:OK` mit `F:00/00` = normaler Betrieb, keine Fehler

#### Code-Referenzen
- `BoardConfigContainer::configureBaseBQ()` — Wendet alle drei Einstellungen beim Start an
- `BqDriver::setTsWarm()` / `setJeitaVSet()` — Bestehende Driver-API
- `BqDriver::setAutoIBATDIS()` — Zum Driver hinzugefügt (Charger Control 0, Bit 7)
- `BoardConfigContainer::getBqDiagnostics()` — Liest TS-Zone aus STATUS_4-Register

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
+150/+120/+90mAh C:200 D:50 3C:180 3D:60 7C:160 7D:70 SOL M:85% T:N/A   ← Solar-Überschuss
```
oder
```
-80/-100/-110mAh C:10 D:90 3C:15 3D:115 7C:20 7D:130 BAT M:45% T:12d0h  ← 12 Tage bis leer
```

---

## 6. RTC-Wakeup-Management

### RV-3028-C7 Integration
**Pin**: GPIO17 (WB_IO1) → RTC INT
**Init**: `InheroMr2Board::begin()`
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
RV3028_CTRL1 (0x0F):     TE=1, TD=11 (1/60 Hz), TRPT=0 (Single shot)
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

### Shutdown-Sequenz (Rev 1.1 — System Sleep mit GPIO-Latch)
**Methode**: `initiateShutdown()` in `InheroMr2Board.cpp`

**Bei Low-Voltage → System Sleep mit GPIO-Latch** (< 500µA, CE-FET hält Zustand):

**Ablauf:** INA228 ALERT ISR → Flag → tickPeriodic() → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`:

1. **Stop Background Tasks**: `BoardConfigContainer::stopBackgroundTasks()`
   - Stoppt Heartbeat task (einziger verbleibender FreeRTOS-Task mit GPIO)
   - Disarmt INA228 Low-Voltage Alert
   
2. **INA228 ALERT disarmen**: `BoardConfigContainer::disarmLowVoltageAlert()` → ISR detachen, BUVL deaktivieren

3. **CE-Pin HIGH latchen** (GPIO-Output-Latch für P0.04 erhalten):
   - `digitalWrite(BQ_CE_PIN, HIGH)` → DMN2004TK-7 ON → CE LOW → Laden aktiv
   - P0.04 wird von `disconnectLeakyPullups()` ausgeschlossen → GPIO-Latch bleibt HIGH im System Sleep
   - Ohne Latch: ext. Pull-Down am Gate → FET OFF → Pull-Up am CE → CE HIGH → **Laden AUS**
   
4. **SX1262 Sleep**: `radio_driver.powerOff()` → `radio.sleep(false)` (Cold Sleep via SPI, ~0.16µA)

5. **PE4259 RF-Switch aus**: `digitalWrite(SX126X_POWER_EN, LOW)` (VDD abschalten)
   
6. **LEDs aus**: PIN_LED1, PIN_LED2 LOW
   
7. **RTC Wake konfigurieren**: `configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES)` (60 min)
   
8. **Shutdown-Grund speichern**: `NRF_POWER->GPREGRET2 = GPREGRET2_LOW_VOLTAGE_SLEEP | reason`
   
9. **BQ INT-Pin**: `detachInterrupt(BQ_INT_PIN)` — nicht als Interrupt genutzt (Polling), aber Safety-Detach vor System Sleep

10. **SOC auf 0% setzen**: `setSOCManually(0.0)` — SOC wird bei Recovery bei 0% starten

11. **System Sleep mit GPIO-Latch**: `sd_power_system_off()` → nRF52840 System-Off (< 500µA gesamt)
    - GPIO4-Latch erhalten (von disconnectLeakyPullups ausgeschlossen) → FET bleibt ON → CE LOW → **Laden aktiv**
    - RAM-Inhalt geht verloren (168h-Statistiken, SOC, etc.)
    - RTC-Interrupt auf GPIO17 weckt System nach Timer-Ablauf

**Warum System Sleep mit GPIO-Latch?**
- DMN2004TK-7 FET für CE-Pin → GPIO4-Latch HIGH erhalten → FET ON → CE LOW → Laden aktiv
- Gesamtverbrauch: **< 500µA** (nRF52840 System-Off + RTC + quiescent currents aller Komponenten)

**168h-Statistiken gehen bei System Sleep verloren** — es existiert kein Persistenzmechanismus für die Ring-Buffer-Daten. Nach Recovery starten die Statistiken bei Null.

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
    // Spannung noch zu niedrig → sofort wieder System Sleep
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
| Na-Ion 1S | 2500 | 2700 | 200mV |

**Anti-Motorboating**: Der Early-Boot-Check in `begin()` verhindert, dass das System bei knapper Spannung immer wieder bootet und sofort abstürzt. Erst wenn VBAT über `lowv_wake_mv` liegt, wird normal gebootet.

**Stromverbrauch im System Sleep mit GPIO-Latch (Low-Voltage Sleep)**:
- **Gesamt: < 500µA** (nRF52840 System-Off + RTC + quiescent currents aller Komponenten)
- CE-FET: GPIO4-Latch HIGH erhalten → FET ON → CE LOW → **Solar-Laden aktiv**

---

## 8. INA228 ALERT-Pin (Rev 1.1)

### Verdrahtung
**Pin**: INA228 ALERT → P1.02 (nRF52840 GPIO, mit ext. Pull-Up)
**TPS62840 EN**: Via 3.3V_off-Schalter geschaltet

### Funktionsweise
Der ALERT-Pin wird als **Software-Interrupt** genutzt:

1. `armLowVoltageAlert()` konfiguriert INA228 BUVL (Bus Under-Voltage Limit) auf `lowv_sleep_mv`
2. ALERT feuert als FALLING-Edge-Interrupt auf P1.02
3. ISR (`lowVoltageAlertISR()`) setzt `lowVoltageAlertFired = true` (nur Flag, kein FreeRTOS-Aufruf)
4. `tickPeriodic()` prüft Flag im nächsten Main-Loop-Tick und ruft `initiateShutdown()` → System Sleep

**Kein Latch-Problem**: Da der ALERT nicht an TPS62840 EN geht, gibt es kein latched-off-Verhalten.
Das System kann nach RTC-Wake normal booten und die Spannung in `begin()` prüfen.

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
- `radio.sleep(false)` sendet einen SPI-Befehl an den SX1262 → sauberer Radio-Shutdown
- PE4259 VDD (GPIO 37) versorgt den RF-Switch, NICHT den SX1262 direkt
- SPI wird über den nRF52840 3.3V Rail versorgt, nicht über PE4259
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

## 11. BQ25798 CE-Pin Safety (Rev 1.1 — FET-invertiert)

### Problem
Der BQ25798 startet mit Default-Konfiguration (1S Li-Ion, 4.2V Ladespannung). Wenn eine LiFePO4-Batterie (3.5V max) verbunden ist und der RAK noch nicht gebootet hat, würde der BQ25798 die Batterie überladen → **Brandgefahr**.

### Hardware-Design (Rev 1.1 — FET-invertiert)
- **Pin**: `BQ_CE_PIN` = GPIO 4 (P0.04 / WB_IO4)
- **DMN2004TK-7 N-FET**: Gate ← GPIO4 (ext. Pull-Down), Drain → CE, Source → GND
- **Externer Pull-Down am Gate**: Zieht Gate LOW wenn GPIO floated → FET OFF
- **Externer Pull-Up am CE**: 10kΩ zu VSYS → CE HIGH wenn FET OFF → **Laden AUS** (BQ25798 CE active-low)
- **GPIO HIGH** → FET ON → CE an GND (LOW) → **Laden AN**
- **GPIO LOW** → Pull-Down am Gate → FET OFF → Pull-Up am CE → CE HIGH → **Laden AUS**
- **GPIO High-Z** (stromlos/Reset) → Pull-Down am Gate → FET OFF → Pull-Up am CE → CE HIGH → **Laden AUS**

**Kernpunkt Rev 1.1**: Laden ist nur aktiv, wenn GPIO4 HIGH getrieben wird (durch Firmware oder GPIO-Output-Latch im System Sleep). Wenn der RAK stromlos oder ungeflasht ist, sorgt der externe Pull-Down für FET OFF → CE HIGH → **Laden deaktiviert** — ein bewusstes Safety-Feature.

### 3-Schicht-Sicherung (Rev 1.1)

| Schicht | Ort | Mechanismus | Wann |
|---|---|---|---|
| **1. Hardware (passiv)** | Pull-Down + Pull-Up | RAK stromlos → Pull-Down am Gate → FET OFF → Pull-Up am CE → CE HIGH → **Laden AUS** | Immer (Safety-Default) |
| **2. Early Boot** | `InheroMr2Board::begin()` | GPIO4 noch nicht getrieben → FET OFF → CE HIGH → **Laden AUS** bis Firmware konfiguriert | Vor I2C-Init |
| **3. Chemie-Konfiguration** | `configureChemistry()` | GPIO HIGH → FET ON → CE LOW → **Laden AN** + I2C Register bei bekannter Chemie | Nach BQ25798-Konfiguration |

### Dual-Layer Safety (Hardware + Software)

```cpp
// In configureChemistry() — nach BQ25798 Register-Konfiguration:
bq.setChargeEnable(props->charge_enable);     // Software-Schicht (I2C Register)
#ifdef BQ_CE_PIN
  pinMode(BQ_CE_PIN, OUTPUT);
  // Rev 1.1 FET-invertiert: HIGH → FET ON → CE LOW → Laden aktiv (BQ25798: CE active-low)
  // FET OFF → Pull-Up am CE → CE HIGH → Laden deaktiviert (Safety-Default)
  digitalWrite(BQ_CE_PIN, props->charge_enable ? HIGH : LOW);  // HIGH=FET ON=CE LOW=Laden an
#endif
```

- `charge_enable` ist Teil der `BatteryProperties`-Tabelle
- `BAT_UNKNOWN` → `charge_enable = false` → GPIO LOW → FET OFF → CE HIGH → **Laden deaktiviert** + Register disabled
- Bekannte Chemie → `charge_enable = true` → GPIO HIGH → FET ON → CE LOW → **Laden aktiviert** + Register enabled

### Verhalten im System Sleep mit GPIO-Latch (Rev 1.1)

In Rev 1.1 wird **System Sleep mit GPIO-Latch** verwendet (via `initiateShutdown()`):
- `digitalWrite(BQ_CE_PIN, HIGH)` wird vor System Sleep aufgerufen
- P0.04 wird von `disconnectLeakyPullups()` ausgeschlossen → GPIO-Output-Latch bleibt HIGH
- GPIO4 gelatcht HIGH → DMN2004TK-7 FET ON → CE LOW → **Laden aktiv**
- BQ25798 MPPT/CC/CV läuft autonom in Hardware → Solar-Laden möglich
- Stromverbrauch: **< 500µA** (nRF52840 System-Off + RTC + quiescent currents aller Komponenten)

| Zustand | CE-Pin | Laden | Solar-Recovery |
|---|---|---|---|
| RAK stromlos (kein Akku) | HIGH (Pull-Up, FET OFF) | **Deaktiviert** (Safety-Default) | N/A |
| Early Boot | HIGH (Pull-Up, GPIO nicht getrieben) | **Deaktiviert** (noch nicht konfiguriert) | Nein |
| BAT_UNKNOWN | HIGH (GPIO LOW → FET OFF) | **Deaktiviert** (CE + I2C Register) | Nein |
| Chemie konfiguriert | LOW (GPIO HIGH → FET ON) | **Aktiv** | **Ja** |
| System Sleep (Low-Voltage) | LOW (GPIO-Latch HIGH → FET ON) | **Aktiv** | **Ja** |

---

## 12. Statistik-Persistenz

### Aktueller Stand

Die 168h-Ringpuffer-Statistiken (Coulomb Counter, MPPT-Daten, SOC-Zustand) sind **nur im RAM** gespeichert und gehen bei jedem Reboot verloren — egal ob System Sleep oder Cold Boot. Es existiert kein Persistenzmechanismus (weder `.noinit`-Section noch LittleFS-Snapshot).

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

### Getter
```bash
board.bat       # Batterietyp abfragen
                # Ausgabe: liion1s | lifepo1s | lto2s | naion1s | none

board.fmax      # Frost-Ladeverhalten abfragen
                # Ausgabe: 0% | 20% | 40% | 100% (LTO: N/A)

board.imax      # Maximaler Ladestrom abfragen
                # Ausgabe: <strom>mA (z.B. 500mA)

board.mppt      # MPPT-Status abfragen
                # Ausgabe: MPPT=1 | MPPT=0

board.telem     # Echtzeit-Telemetrie mit SOC
                # Ausgabe: B:<V>V/<I>mA/<T>C SOC:<Prozent>% S:<V>V/<SolarStrom>
                # Beispiel: B:3.85V/125.4mA/22C SOC:68.5% S:5.12V/385mA
                # Beispiel: B:3.85V/-8.2mA/N/A SOC:N/A S:0.00V/0mA

board.stats     # Energie-Statistiken (Bilanz + MPPT + TTL)
                # Ausgabe: <24h>/<3d>/<7d>mAh C:<24h> D:<24h> 3C:<3d> 3D:<3d> 7C:<7d> 7D:<7d> <SOL|BAT> M:<mppt>% T:<ttl>
                # Beispiel: +125/+45/+38mAh C:200 D:75 3C:150 3D:105 7C:140 7D:102 SOL M:85% T:N/A
                # Beispiel: -30/-45/-40mAh C:10 D:40 3C:5 3D:50 7C:8 7D:48 BAT M:45% T:12d0h
                # SOL = Solar-Überschuss, BAT = Energiedefizit
                # T: Time To Live (N/A bei Solar-Überschuss oder <24h Daten)

board.cinfo     # Ladegerät-Info + letzter PG-Stuck HIZ-Toggle
                # Ausgabe: "PG / CC HIZ:never" oder "!PG / !CHG HIZ:3m ago"

board.selftest  # I²C-Hardware-Probe (alle Onboard-Komponenten)
                # Ausgabe: "INA:OK BQ:OK RTC:OK BME:OK"
                # States je Gerät: OK | NACK | WR_FAIL (nur RTC)

board.conf      # Alle Konfigurationswerte
                # Ausgabe: B:<bat> F:<fmax> M:<mppt> I:<imax> Vco:<V> V0:<V>
                # Beispiel: B:liion1s F:0% M:1 I:500mA Vco:4.10 V0:3.30

board.tccal     # NTC-Temperatur-Kalibrieroffset
                # Ausgabe: TC offset: +0.00 C (0.00=default)

board.leds      # LED-Aktivstatus (Heartbeat + BQ Stat)
                # Ausgabe: "LEDs: ON (Heartbeat + BQ Stat)"

board.batcap    # Batteriekapazität
                # Ausgabe: 10000 mAh (set) oder 2200 mAh (default)
```

### Setter
```bash
set board.bat <type>        # Batteriechemie setzen
                            # Optionen: liion1s | lifepo1s | lto2s | naion1s | none

set board.fmax <wert>       # Frost-Ladestromabsenkung setzen
                            # Optionen: 0% | 20% | 40% | 100%
                            # Begrenzt Ladestrom im T-Cool-Bereich (0°C bis -5°C)
                            # Bei LTO / Na-Ion ohne Wirkung (JEITA deaktiviert)

set board.imax <mA>         # Maximalen Ladestrom setzen
                            # Bereich: 50-1500 mA

set board.mppt <0|1>        # MPPT ein-/ausschalten

set board.batcap <mAh>      # Batteriekapazität setzen
                            # Bereich: 100-100000 mAh

set board.tccal             # NTC-Temperatur kalibrieren (auto via BME280)
set board.tccal reset       # Offset auf 0.00 zurücksetzen

set board.leds <on|off>     # LEDs ein-/ausschalten (on/1, off/0)

set board.soc <percent>     # SOC manuell setzen (0-100, INA228 muss bereit sein)
```

---

## Dateien-Übersicht

### Hauptimplementierung
| Datei | Beschreibung |
|-------|--------------|
| **InheroMr2Board.h/cpp** | Board-Klasse, Init, Shutdown, RTC, CLI-Commands |
| **BoardConfigContainer.h/cpp** | Battery Management, BQ25798, INA228, MPPT, SOC, Daily Balance |
| **lib/Ina228Driver.h/cpp** | INA228 I2C Communication, Calibration, Coulomb Counter |
| **lib/BqDriver.h/cpp** | BQ25798 I2C Communication, MPPT, Charging |

### Schlüssel-Methoden
| Methode | Datei | Funktion |
|---------|-------|----------|
| `begin()` | InheroMr2Board.cpp | Board-Initialisierung, Wake-up-Check, Early-Boot Low-Voltage Check |
| `initiateShutdown()` | InheroMr2Board.cpp | System Sleep Shutdown (aufgerufen von tickPeriodic nach ALERT) |
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
// Ina228Driver.cpp
void Ina228Driver::shutdown() {
  // Set operating mode to Shutdown (MODE = 0x0)
  // This disables all conversions and Coulomb Counter
  uint16_t adc_config = 0x0000;  // MODE = 0x0 (Shutdown)
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}
```

### INA228 Wake-up
```cpp
// Ina228Driver.cpp
void Ina228Driver::wakeup() {
  // Re-enable continuous measurement mode
  uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) |  // Continuous all
                        (INA228_ADC_AVG_64 << 0);             // 64 samples average (TX peak filtering)
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}
```

### RTC Interrupt Handler
```cpp
// InheroMr2Board.cpp — ISR setzt nur Flag, kein I2C!
void InheroMr2Board::rtcInterruptHandler() {
  rtc_irq_pending = true;
}
// TF-Clear passiert im Main-Loop-Kontext (tick())
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

### Szenario A: Normale Entladung (Low-Voltage System Sleep) - Li-Ion
```
t=0:      VBAT = 3.7V → Normal (60s checks, Coulomb Counter läuft)
          Daily balance: Today +150mAh SOLAR
          
t=+1h:    VBAT = 3.5V → Normal (INA228 ALERT nicht getriggert)
          SOC: 45%
          
t=+2h:    VBAT = 3.15V → INA228 ALERT feuert (< 3100mV lowv_sleep_mv)
          - lowVoltageAlertISR() → setzt lowVoltageAlertFired Flag
          - tickPeriodic() erkennt Flag im nächsten tick()
          - board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)
          - CE gelatcht (GPIO4-Latch HIGH → FET ON → CE LOW → Laden aktiv)
          - RTC: Wake in 1h (LOW_VOLTAGE_SLEEP_MINUTES = 60)
          - SOC → 0%
          - sd_power_system_off() → System Sleep mit GPIO-Latch (< 500µA)
          
t=+3h:    RTC weckt → System bootet → Early Boot Check
          - Ina228Driver::readVBATDirect() → VBAT = 3.15V
          - VBAT < lowv_wake_mv (3300mV) → sofort wieder schlafen
          - configureRTCWake(60) + sd_power_system_off()
          
t=+4h:    RTC weckt → System bootet → Early Boot Check
          - VBAT = 3.20V → noch unter 3300mV → wieder schlafen
          
t=+5h:    RTC weckt → System bootet → Early Boot Check
          - VBAT = 3.45V (Solar-Recovery!)
          - VBAT > lowv_wake_mv (3300mV) → normaler Boot
          - Low-Voltage-Recovery markiert, SOC bei 0%
          - Coulomb Counter startet neu
          - Daily balance baut sich neu auf
```

### Szenario B: Kritische Entladung (Rev 1.1 — kein Hardware-UVLO)
```
In Rev 1.1 gibt es kein Hardware-UVLO (TPS62840 EN via 3.3V_off-Schalter).
Der INA228 ALERT auf P1.02 dient als Software-Interrupt für System Sleep.

t=0:      VBAT = 3.15V → INA228 ALERT feuert
          - tickPeriodic() → initiateShutdown()
          - System Sleep mit GPIO-Latch (< 500µA), CE gelatcht LOW (Laden aktiv), RTC-Wake 1h
          
t=+1h:    RTC-Wake → Early Boot → VBAT = 3.05V (noch unter 3300mV)
          - Sofort wieder schlafen (CE bleibt gelatcht LOW → Solar-Laden möglich)
          
t=+2h:    RTC-Wake → VBAT = 2.95V (weiter gesunken, kein Solar)
          - Sofort wieder schlafen
          - Board pendelt weiter mit < 500µA + stündlichem Boot (~0.03mAh)
          
t=+∞:     Bei < 500µA kann die Batterie monatelang überleben
          - Sobald Solar verfügbar → VBAT steigt → normaler Boot bei >3300mV
          - KEIN Latching: System kann IMMER von selbst recovern
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
          
          CLI output: "-350/-167/-38mAh C:150 D:500 3C:.. 3D:.. 7C:.. 7D:.. BAT M:45% T:16d23h"
```

---

## Siehe auch

- [README.md](README.md) — Benutzer-Dokumentation und CLI-Referenz
- [TELEMETRY.md](TELEMETRY.md) — Telemetrie-Kanäle erklärt (was die App anzeigt)
- [QUICK_START.md](QUICK_START.md) — Inbetriebnahme und Konfiguration
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle CLI-Befehle auf einen Blick

### Datasheets
- **INA228**: https://www.ti.com/product/INA228
- **RV-3028-C7**: https://www.microcrystal.com/en/products/real-time-clock-rtc-modules/rv-3028-c7/
- **BQ25798**: https://www.ti.com/product/BQ25798
- **TPS62840**: https://www.ti.com/product/TPS62840
- **nRF52840**: https://www.nordicsemi.com/products/nrf52840
