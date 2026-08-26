# Inhero MR2 Energieverwaltung - Implementierungs-Dokumentation (Rev 1.1)

> 🇬🇧 [English version](../POWER_MANAGEMENT.md)

## Inhaltsverzeichnis

- [Überblick](#überblick)
- [Hardware-Architektur](#hardware-architektur)
- [1. Low-Voltage-Erkennung (INA228 ALERT ISR)](#1-low-voltage-erkennung-ina228-alert-isr)
- [2. Coulomb Counter & SOC (Ladezustand)](#2-coulomb-counter--soc-ladezustand)
- [3. Tägliche Energiebilanz](#3-tägliche-energiebilanz)
- [4. Solar-Energieverwaltung](#4-solar-energieverwaltung)
  - [BQ25798 ADC bei niedrigen Akkuspannungen](#bq25798-adc-bei-niedrigen-akkuspannungen)
  - [JEITA-WARM-Zone & VBAT_OVP-Vermeidung](#jeita-warm-zone--vbat_ovp-vermeidung)
- [5. Batt-TTL-Prognose](#5-batt-ttl-prognose)
- [6. RTC-Wakeup-Management](#6-rtc-wakeup-management)
- [7. Energieverwaltungsablauf](#7-energieverwaltungsablauf)
- [8. INA228 ALERT-Pin (Rev 1.1)](#8-ina228-alert-pin-rev-11)
- [9. SX1262 Power Control & PE4259 RF-Switch](#9-sx1262-power-control--pe4259-rf-switch)
- [10. BQ25798 CE-Pin Safety (Rev 1.1 — FET-invertiert)](#10-bq25798-ce-pin-safety-rev-11--fet-invertiert)
- [11. Statistik-Persistenz](#11-statistik-persistenz)
- [12. CLI-Befehle](#12-cli-befehle)
- [Siehe auch](#siehe-auch)

> Diese Dokumentation beschreibt die Energieverwaltungs-Implementierung für das Inhero MR2 Board.
> Hardware Rev 1.1: INA228 ALERT auf P1.02, TPS62840 EN via 3.3V_off-Schalter, CE-Pin via N-FET (invertiert).

---

## Überblick

Das System kombiniert **INA228 ALERT-basierte Low-Voltage-Erkennung** + **System Sleep mit GPIO-Latch** + **Coulomb Counter** + **tägliche Energiebilanz** + **CE-Pin FET-Safety** für maximale Energie-Effizienz:

1. **INA228 ALERT ISR** (P1.02) - Low-Voltage-Erkennung via Hardware-Interrupt
2. **System Sleep mit GPIO-Latch** (< 500µA) mit RTC-Wake - Minimaler Stromverbrauch bei Low-Voltage
3. **CE-Pin FET-Safety** - Invertierte Logik, Solar-Laden in System Sleep möglich
4. **Coulomb Counter** (INA228) - Echtzeit-SOC-Tracking
5. **Tägliche Energiebilanz** (7-Tage rolling) - Solar vs. Akku
6. **RTC-Wakeup-Management** (RV-3028-C7) - Periodische Recovery-Checks

### Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| INA228 ALERT → Low-Voltage System Sleep | Aktiv | ISR auf P1.02 → volatile Flag → tickPeriodic() → System Sleep mit GPIO-Latch + RTC-Wake |
| RTC-Wakeup (Low-Voltage-Recovery) | Aktiv | 60 min (periodisch) |
| BQ CE-Pin Safety (FET-invertiert) | Aktiv | GPIO HIGH → FET ON → CE LOW → Laden an (BQ25798 CE active-low), Dual-Layer: GPIO + I2C |
| System Sleep mit gelatchtem CE | Aktiv | < 500µA, GPIO4-Latch HIGH erhalten → FET ON → CE LOW → Solar-Laden möglich |
| SOC via INA228 + manuelle Akkukapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |


---

## Hardware-Architektur

### Komponenten
| Komponente | Funktion | I2C | Pin | Details |
|------------|----------|-----|-----|---------|
| **RAK4630** | Core Module | — | — | nRF52840 SoC + SX1262 LoRa Transceiver |
| **INA228** | Power Monitor | 0x40 | ALERT→P1.02 (ISR) | 100mΩ Shunt, 1.6A max, Coulomb Counter, BUVL Alert |
| **BME280** | Temperatur-/Feuchte-/Drucksensor | 0x76 | — | NTC-Kalibrier-Referenz (`set board.tccal`), Selftest |
| **RV-3028-C7** | RTC | 0x52 | INT→GPIO17 | Zeitbasis, Countdown-Timer, Wake-up. Siehe [FAQ #23](FAQ.md#23-warum-braucht-das-repeater-board-eine-korrekte-uhrzeit) zur Bedeutung der Uhrzeit. |
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC (IBUS ~±30mA error at low currents; ADC hat VBAT-abhängige Schwellen, siehe [Abschnitt 4](#bq25798-adc-bei-niedrigen-akkuspannungen)) |
| **BQ CE-Pin** | Charge Enable | — | GPIO4 (P0.04) | Via N-FET: GPIO HIGH → FET ON → CE LOW → Laden an (BQ25798 CE active-low) |
| **TPS62840** | Buck Converter | - | EN via 3.3V_off-Schalter | 750mA, 3.3V Rail |
| **CE-FET** | N-FET | — | Gate←GPIO4 (ext. Pull-Down) | Drain→CE, Source→GND. GPIO HIGH → FET ON → CE LOW → Laden an. Pull-Down zieht Gate LOW wenn floating. |
| **Schottky-Diode** | USB→VBUS-Diode | — | — | VBUS-USB → VBUS-BQ (Solareingang). USB-C CC1/CC2 über 4,7kΩ auf GND (USB-Sink). **⚠ Solar-Kurzschluss shortet auch VBUS-USB.** |

---

## 1. Low-Voltage-Erkennung (INA228 ALERT ISR)

### Implementierung (Rev 1.1 — Flag/Tick-Architektur)
- **Trigger**: INA228 BUVL (Bus Under-Voltage Limit) ALERT auf P1.02
- **ISR**: `BoardConfigContainer::lowVoltageAlertISR()` → setzt `lowVoltageAlertFired = true` (nur Flag, kein FreeRTOS-Aufruf)
- **Verarbeitung**: `tickPeriodic()` prüft Flag im Main-Loop-Kontext → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`
- **Arming**: `armLowVoltageAlert()` wird bei Akku-Konfiguration aufgerufen (setzt BUVL-Schwelle + aktiviert ISR)

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
        │ GPREGRET2 → LOW_VOLTAGE_SLEEP-Flag
        ▼
sd_power_system_off() → System Sleep mit GPIO-Latch (< 500µA)
```

### Chemie-spezifische Schwellen (1-Level System, einheitliche 200mV Hysterese)

| Chemie | lowv_sleep_mv (ALERT) | lowv_wake_mv (0% SOC) | Hysterese |
|--------|----------------------|----------------------|-----------|
| **Li-ion 1S** | 3100 | 3300 | 200mV |
| **LiFePO4 1S** | 2700 | 2900 | 200mV |
| **LTO 2S** | 3900 | 4100 | 200mV |
| **Na-ion 1S** | 2500 | 2700 | 200mV |

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
  - CURRENT_LSB = 1.6384A / 524288 ≈ 3.125µA
  - ADC-Bereich ±163.84mV (ADCRANGE=0, optimal für 1A @ 100mΩ)
  - **ADC-Mittelung**: 256 Samples (filtert TX-Spannungsspitzen)
  - BUVL Alert konfiguriert auf `lowv_sleep_mv` (chemie-spezifisch)

### SOC-Berechnung
**Methode**: `updateBatterySOC()` in `BoardConfigContainer.cpp`
- **Primär**: Coulomb Counting (INA228 CHARGE-Register)
- **Update-Intervall**: alle 60s via tickPeriodic(); im Low-Voltage-Sleep gibt es keine SOC-Updates (der RTC-Wake prüft nur VBAT und schläft weiter oder bootet)

**Formel**:
```
SOC_delta = charge_delta_mah / capacity_mah × 100%
SOC_new = SOC_old + SOC_delta
```

**Auto-Sync**: Bei BQ25798 "Charge Done" wird SOC auf 100% gesetzt.

### Kapazitäts-Management

#### Konfiguration erforderlich
Die Akkukapazität **muss manuell gesetzt werden**, da sie in der Praxis stark variiert:
- **Typischer Bereich**: 4000-24000mAh (4-24Ah)
- **CLI-Befehl**: `set board.batcap <mAh>`
- **Erlaubter Bereich**: 100-100000mAh

**Wichtig**: Ohne korrekte Kapazität sind SOC% und Batt-TTL-Berechnungen ungenau!

#### Persistenz-Mechanik
**Speicherpfad**: `/inheromr2/batCap.txt` (LittleFS via SimplePreferences)
**Save-Methode**: `setBatteryCapacity()` in `BoardConfigContainer.cpp` (persistiert via SimplePreferences)
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
- ✅ **Überlebt** Firmware-Update (LittleFS bleibt erhalten)
- ⚠️ **Verloren** bei: Flash-Erase, `rm -rf /inheromr2/`, Filesystem-Korruption

---

## 3. Tägliche Energiebilanz

### Tracking (168-Stunden-Ringpuffer)
**Methoden**: `updateHourlyStats()` + `calculateRollingStats()` in `BoardConfigContainer.cpp`
- **Aufgerufen von**: tickPeriodic() (alle 60 min)
- **Abtastung**: An jeder Stundengrenze (RTC-Zeit auf volle Stunden abgeschnitten) wird die abgeschlossene Stunde in den Ringpuffer geschrieben

**Datenstruktur**: `BatterySOCStats.hours[168]` (7 Tage × 24 Stunden)
```cpp
typedef struct {
  uint32_t timestamp;      // Stundenbeginn, Unix-Sekunden
  float charged_mah;       // in dieser Stunde geladen
  float discharged_mah;    // in dieser Stunde entladen
  float solar_mah;         // Solar-Anteil dieser Stunde
} HourlyBatteryStats;
```

Die Stundenwerte werden in `updateBatterySOC()` (alle 60s) aus den Deltas des INA228-CHARGE-Registers aufsummiert: ein positives Delta zählt als geladen (und solar), ein negatives Delta als entladen.

### Berechnungen
**Rollierende Summen** (`calculateRollingStats()`, nach jeder abgeschlossenen Stunde):
```
last_24h_net_mah       = Σ(solar − entladen) über die letzten 24 Stunden
avg_3day_daily_net_mah = Σ(solar − entladen) über 72h / 3    (braucht ≥ 24h Daten)
avg_7day_daily_net_mah = Σ(solar − entladen) über 168h / 7   (braucht ≥ 24h Daten; Basis für Batt-TTL)
```

**Living Status**:
- `living_on_battery = true` wenn `last_24h_net_mah < 0` (Netto-Defizit über die letzten 24h)
- Solar-Überschuss (SOL in `board.stats`) ist schlicht `living_on_battery == false`

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

- PFM-Forward-Modus ist ab Werk im BQ25798 aktiv (PFM_FWD_DIS=0, REG0x12); die Firmware ändert ihn nicht
- PFM verbessert die Effizienz bei niedrigen Solarströmen

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

**Flag-Clearing beim Boot**: `BqDriver::clearInterruptFlags()` (aufgerufen aus `BoardConfigContainer::begin()`)
- Liest die CHARGER_FLAG/FAULT_FLAG-Register 0x22–0x27 → INT-Leitung wird freigegeben
- Vermeidet stehengebliebene Faults vom vorherigen Power-Cycle

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

### BQ25798 ADC bei niedrigen Akkuspannungen

> **Referenz:** BQ25798 Datasheet (TI SLUSE22), Section 9.3.16 — ADC

#### Problem

Der 15-bit ADC im BQ25798 hat **spannungsabhängige Betriebsschwellen**, die im reinen Akkubetrieb (ohne Solar) relevant werden. Bei niedrigen Akkuspannungen kann der ADC seine Konvertierung nicht abschließen — `ADC_EN` bleibt gesetzt und die Firmware läuft in einen Timeout.

#### Datasheet-Zitat (Section 9.3.16)

> *"The ADC is allowed to operate if either VBUS > 3.4V or VBAT > 2.9V is valid.
> At battery only condition, if the TS_ADC channel is enabled, the ADC only works
> when battery voltage is higher than 3.2V, otherwise, the ADC works when the
> battery voltage is higher than 2.9V."*

#### Betriebsszenarien

| Bedingung | VBUS | VBAT | TS-Kanal | ADC | Temperatur |
|-----------|------|------|----------|-----|------------|
| Solar angeschlossen | > 3.4V | beliebig | aktiviert | ✅ läuft | ✅ verfügbar |
| Akkubetrieb, normal | — | ≥ 3.2V | aktiviert | ✅ läuft | ✅ verfügbar |
| Akkubetrieb, niedrig | — | 2.9–3.2V | **deaktiviert** | ✅ läuft | ❌ nicht verfügbar |
| Akkubetrieb, kritisch | — | < 2.9V | deaktiviert | ❌ Timeout | ❌ nicht verfügbar |

#### Firmware-Lösung: VBAT-abhängige TS-Kanal-Steuerung

Die Firmware liest die aktuelle Akkuspannung vom INA228 und übergibt sie an `BqDriver::getTelemetryData(vbat_mv)`:

- **VBAT ≥ 3.2V** (oder unbekannt): TS-Kanal aktiviert → ADC-Schwelle 3.2V, Temperatur verfügbar
- **VBAT < 3.2V**: TS-Kanal deaktiviert → ADC-Schwelle sinkt auf 2.9V, Temperatur als "N/A" angezeigt

Dadurch funktioniert der ADC im Bereich 2.9–3.2V weiterhin für Solar-Messungen (VBUS, IBUS), auch wenn die Akkutemperatur nicht gelesen werden kann.

#### ADC-Kanal-Konfiguration (nur benötigte Kanäle)

Auf dem MR2 sind D+, D−, VAC1, VAC2 nicht verbunden. Die Firmware aktiviert nur die tatsächlich genutzten Kanäle:

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
  → Akku bei 3,47V > 104% × 3,1V = 3,224V → VBAT_OVP ausgelöst
  → Wandler stoppt, EN_AUTO_IBATDIS zieht IBAT_LOAD = 30mA aus dem Akku
  → Gesamtverbrauch: −11mA (System) + −30mA (IBAT_LOAD) = −41mA
  → Recovery erfordert VBAT < 102% × 3,1V = 3,162V → stundenlanger Akkuverbrauch
```

#### Fix: Drei Register-Einstellungen in `configureBaseBQ()`

| Einstellung | Register | Wert | Wirkung |
|-------------|----------|------|---------|
| `setTsWarm(BQ25798_TS_WARM_55C)` | NTC Control 1 (0x18), Bits 5:4 | 55 °C (37,7% REGN) | WARM-Zone beginnt bei ~52 °C (Inhero), nicht ~42 °C |
| `setJeitaVSet(BQ25798_JEITA_VSET_UNCHANGED)` | NTC Control 0 (0x17), Bits 7:5 | UNCHANGED | Keine VREG-Reduktion in WARM — verhindert VBAT_OVP |
| `JEITA_ISETH` (POR-Default beibehalten) | NTC Control 0, Bits 4:3 | 11b = ICHG unchanged | Keine Ladestrom-Reduktion in WARM |
| `setAutoIBATDIS(false)` | Charger Control 0, Bit 7 | 0 | Deaktiviert 30-mA-Akkuentladung bei OVP |

> **Ergebnis:** Mit JEITA_VSET=UNCHANGED und JEITA_ISETH=ICHG unchanged ist die WARM-Zone (T3–T5) effektiv neutralisiert. Die Ladung läuft mit voller Spannung und vollem Strom bis T-Hot (~58 °C), wo die Ladung komplett gesperrt wird.

#### TS-Schwellenvergleich

| Zonengrenze | BQ-Register | % REGN | TI-Referenz (°C) | Inhero MR2 (°C) | Shift |
|-------------|-------------|--------|-------------------|------------------|-------|
| VT1 (Cold) | — | 72,0% | +3,7 | −2,0 | −5,7 °C |
| VT2 (Cool) | — | 69,8% | +7,9 | +2,8 | −5,1 °C |
| VT3 (Warm) | TS_WARM=55°C | 37,7% | +54,5 | +52,2 | −2,3 °C |
| VT5 (Hot) | — | 34,2% | +59,9 | +57,7 | −2,2 °C |

> NTC-Modelle: 103AT (B25/50=3435) für TI-Referenz, NCP15XH103F03RC (B25/85=3380) für Inhero. Typische %REGN-Werte aus BQ25798-Datenblatt.

#### Code-Referenzen
- `BoardConfigContainer::configureBaseBQ()` — Wendet alle drei Einstellungen beim Start an
- `BqDriver::setTsWarm()` / `setJeitaVSet()` — Bestehende Driver-API
- `BqDriver::setAutoIBATDIS()` — Zum Driver hinzugefügt (Charger Control 0, Bit 7)

---

## 5. Batt-TTL-Prognose

> **Batt-TTL** steht für *Battery Time-To-Live* — die geschätzte Restlaufzeit im Akkubetrieb. Gemeint ist nicht das Hop-Limit, das „TTL“ im Mesh-Netz bezeichnet.

### Datenbasis und Zeitbasis

Die Batt-TTL-Berechnung basiert auf dem **7-Tage gleitenden Durchschnitt** des täglichen Netto-Energieverbrauchs, der aus einem **168-Stunden-Ringpuffer** (7 Tage) stündlicher INA228-Coulomb-Counter-Messungen berechnet wird.

#### Datenfluss

```
INA228 Hardware Coulomb Counter (20-bit ADC, ±0.1% Genauigkeit)
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
        │  extractable_mah / |deficit_per_day| × 24 = Batt-TTL Stunden
        │  (extractable = gespeichert − eingeschlossene Ladung, siehe Formel unten)
        ▼
socStats.ttl_hours → getTTL_Hours() → board.stats / Telemetrie
```

### Berechnung
**Methode**: `calculateTTL()` in `BoardConfigContainer.cpp`
- **Aufgerufen**: Nach `calculateRollingStats()` (stündlich)
- **Zeitbasis**: 7-Tage gleitender Durchschnitt (`avg_7day_daily_net_mah`) aus stündlichen Samples

**Voraussetzungen für Batt-TTL > 0**:
1. `living_on_battery == true` (24h-Netto ist negativ, d.h. Energiedefizit)
2. `avg_7day_daily_net_mah < 0` (7-Tage-Durchschnitt zeigt Netto-Entladung)
3. `capacity_mah > 0` (Akkukapazität bekannt, via `set board.batcap`)
4. Mindestens **24 Stunden** gültige Daten im Ringpuffer

**Formel** (Trapped-Charge-Modell):
```
remaining_capacity_mah = (SOC% / 100) × capacity_mah
trapped_mah            = capacity_mah × (1 − f(T))
extractable_mah        = max(0, remaining_capacity_mah − trapped_mah)
daily_deficit_mah = -avg_7day_daily_net_mah  (positiver Wert)
TTL_hours = extractable_mah / daily_deficit_mah × 24
```
f(T) ist der chemie-spezifische Kälte-Derating-Faktor (`temp_derating_factor`); f(T) = 1 bei ≥ 25 °C — bei moderaten Temperaturen ist also nichts eingeschlossen und die Formel reduziert sich auf remaining/deficit.

**Batt-TTL = 0 bedeutet**:
- Gerät wird solar versorgt (Netto-Überschuss) → `living_on_battery == false`
- Noch keine 24h Daten gesammelt (Kaltstart)
- Akkukapazität unbekannt

**Infinite Batt-TTL (Telemetrie)**:
- Wenn `living_on_battery == false` und SOC valide → wird als 990 Tage (Max-Wert) übertragen

**Beispiel**:
- SOC: 60% = 1200mAh gespeichert (bei 2000mAh Kapazität)
- Temperatur ≥ 25 °C → f(T) = 1, nichts eingeschlossen → entnehmbar = 1200mAh
- 7-Tage-Durchschnitt: -100 mAh/Tag (aus 168h Stunden-Samples)
- Batt-TTL: 1200 / 100 × 24 = 288 Stunden = 12 Tage

**CLI-Ausgabe**: `board.stats`
```
+150/+120/+90mAh C:200 D:50 3C:180 3D:60 7C:160 7D:70 SOL M:85% BT:N/A   ← Solar-Überschuss
```
oder
```
-80/-100/-110mAh C:10 D:90 3C:15 3D:115 7C:20 7D:130 BAT M:45% BT:12d0h  ← 12 Tage bis leer
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
- **Tick-Rate**: 1/60 Hz (1 Minute pro Tick), konfiguriert via TD=11 in CTRL1
- **Max. Countdown**: 4095 Minuten ≈ 2,8 Tage (12-bit-Timer-Register)
- **Low-Voltage-Sleep-Intervall**: `LOW_VOLTAGE_SLEEP_MINUTES` = 60 min (1h)
- **Begründung**: Jeder Wake ist ein System-ON-Reset mit Early-Boot-Fast-Path (minimales I2C: RTC-TF clearen, VBAT lesen, wieder schlafen) und kostet nur ~0.03 mAh

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

Der eigentliche TF-Clear passiert im Main-Loop-Kontext in `tick()` per I2C (Read-Modify-Write, nur das TF-Bit wird gelöscht):
```cpp
// In InheroMr2Board::tick() — Main-Loop-Kontext:
if (rtc_irq_pending) {
  rtc_irq_pending = false;
  // RV3028_REG_STATUS lesen ...
  uint8_t status = Wire.read();
  status &= ~(1 << 3);  // Nur TF-Bit löschen → INT-Pin geht via Pull-Up wieder HIGH
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_STATUS);
  Wire.write(status);   // zurückschreiben — die übrigen Status-Flags bleiben erhalten
  Wire.endTransmission();
}
```

**Warum nicht in der ISR?** I2C (Wire) darf nicht aus einem ISR-Kontext aufgerufen werden.
Der ISR setzt nur das Flag; `tick()` prüft es im Main-Loop.

---

## 7. Energieverwaltungsablauf

### Shutdown-Sequenz (Rev 1.1 — System Sleep mit GPIO-Latch)
**Methode**: `initiateShutdown()` in `InheroMr2Board.cpp`

**Bei Low-Voltage → System Sleep mit GPIO-Latch** (< 500µA, CE-FET hält Zustand):

**Ablauf:** INA228 ALERT ISR → Flag → tickPeriodic() → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`:

1. **Background-Tasks stoppen**: `BoardConfigContainer::stopBackgroundTasks()`
   - Stoppt Heartbeat-Task (einziger verbleibender FreeRTOS-Task mit GPIO)
   - Disarmt INA228 Low-Voltage Alert (ISR detachen, BUVL deaktivieren)

2. **INA228 auf Minimalstrom**: ALERT-Pin freigeben (`enableAlert(false, ...)`, `setUnderVoltageAlert(0)` — ein LOW gelatchter ALERT würde ~330µA über den Pull-Up verheizen), danach `shutdown()` (ADC aus, ~3.5µA)

3. **SX1262 Sleep + PE4259 aus**: `inhero::prepareRadioForSystemOff()` — zuerst `radio.sleep(false)` (Cold Sleep via SPI, ~0.16µA), dann `digitalWrite(SX126X_POWER_EN, LOW)` (PE4259 VDD abschalten)

4. **LEDs aus**: PIN_LED1, PIN_LED2 LOW

5. **CE-Pin HIGH latchen** (GPIO-Output-Latch für P0.04 erhalten):
   - `digitalWrite(BQ_CE_PIN, HIGH)` → CE-FET ON → CE LOW → Laden aktiv
   - P0.04 wird von `disconnectLeakyPullups()` ausgeschlossen → GPIO-Latch bleibt HIGH im System Sleep
   - Ohne Latch: ext. Pull-Down am Gate → FET OFF → Pull-Up am CE → CE HIGH → **Laden AUS**

6. **INA228 + BQ25798 auf Minimalstrom**: `inhero::prepareIcsForSystemOff()` (Raw-I2C-Sicherheitsnetz, wiederholt den INA228-Shutdown mit Readback)

7. **BME280 schlafen legen**: Sleep-Modus per I2C erzwingen (spart ~1–7µA; harmloser NACK, wenn nicht bestückt)

8. **RTC-Wake konfigurieren**: `configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES)` (60 min)

9. **P0-LATCH für den RTC-INT-Pin löschen** (ein stehengebliebener Latch würde DETECT sofort auslösen → sofortiger Wake → Boot-Schleife)

10. **I2C freigeben**: `Wire.end()`, danach `inhero::disconnectLeakyPullups()` (jeder LOW gehaltene Pull-Up verheizt ~250µA)

11. **Shutdown-Grund speichern**: `NRF_POWER->GPREGRET2 = GPREGRET2_LOW_VOLTAGE_SLEEP | reason`

12. **System Sleep mit GPIO-Latch**: `sd_power_system_off()` → nRF52840 System-Off (< 500µA gesamt)
    - GPIO4-Latch erhalten (von disconnectLeakyPullups ausgeschlossen) → FET bleibt ON → CE LOW → **Laden aktiv**
    - RAM-Inhalt geht verloren (168h-Statistiken, SOC, etc.)
    - RTC-Interrupt auf GPIO17 weckt System nach Timer-Ablauf

Der SOC wird beim Shutdown nicht geschrieben — beim nächsten erfolgreichen Recovery-Boot ruft `begin()` `setSOCManually(0.0)` auf (Low-Voltage-Recovery), der SOC startet also bei 0%.

**Warum System Sleep mit GPIO-Latch?**
- N-FET für CE-Pin → GPIO4-Latch HIGH erhalten → FET ON → CE LOW → Laden aktiv
- Gesamtverbrauch: **< 500µA** (nRF52840 System-Off + RTC + quiescent currents aller Komponenten)

**168h-Statistiken gehen bei System Sleep verloren** — es existiert kein Persistenzmechanismus für die Ring-Buffer-Daten. Nach Recovery starten die Statistiken bei Null.

### Wake-up-Check (Anti-Motorboating)
**Methode**: `InheroMr2Board::begin()`

Der Code prüft `GPREGRET2` für den Shutdown-Grund und die Akkuspannung für Wake-up-Entscheidungen.

**2 Fälle**:

**Fall 1: Wake aus dem Low-Voltage-Sleep** (`(GPREGRET2 & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE`)
```cpp
// InheroMr2Board::begin() — Early-Boot-Fast-Path (vereinfacht)
uint8_t shutdown_reason = NRF_POWER->GPREGRET2;
if ((shutdown_reason & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE) {
  Wire.begin();
  inhero::clearTimerFlag();  // Wake war ein Reset — der ISR hat das RTC-Event nie gesehen
  uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR);
  uint16_t wake_threshold = getLowVoltageWakeThreshold();

  if (vbat_mv == 0 || vbat_mv < wake_threshold) {
    // Spannung noch zu niedrig → zurück in den System Sleep.
    // Der Wake-Reset hat alle PIN_CNF gelöscht — der GPIO-Latch aus dem Sleep
    // überlebt ihn NICHT. CE muss neu als OUTPUT HIGH getrieben werden,
    // sonst stoppt das Solar-Laden.
    pinMode(BQ_CE_PIN, OUTPUT);
    digitalWrite(BQ_CE_PIN, HIGH);
    inhero::prepareIcsForSystemOff();        // INA228 + BQ25798 auf Minimalstrom
    inhero::prepareRadioForSystemOff(false); // SX1262 zurück in Cold Sleep
    configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES);
    inhero::disconnectLeakyPullups();
    NRF_POWER->GPREGRET2 = GPREGRET2_LOW_VOLTAGE_SLEEP | SHUTDOWN_REASON_LOW_VOLTAGE;
    sd_power_system_off();  // Bleibt im Low-Voltage-Sleep-Zyklus
  }
  // Spannung OK → normaler Boot; Low-Voltage-Recovery-Markierung + SOC=0%
  // folgen erst nach boardConfig.begin()
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
}
```

**Fall 2: Normaler Kaltstart** (Power-On, Reset-Taste, Spannung OK)
```cpp
else {
  // Normaler Boot wird fortgesetzt
  // INA228 und alle anderen Komponenten werden initialisiert
}
```

**Direkter ADC-Read** (boardConfig noch nicht bereit):
```cpp
// Muss direkt aus den INA228-ADC-Registern lesen (20-Bit-ADC, linksbündig im 24-Bit-Register, ±0,1 % Genauigkeit)
uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR);
```

**Spannungsschwellen** (chemie-spezifisch, 1-Level-System):
| Chemie | lowv_sleep_mv (ALERT) | lowv_wake_mv (Recovery) | Hysterese |
|--------|----------------------|------------------------|-----------|
| Li-ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |
| Na-ion 1S | 2500 | 2700 | 200mV |

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

## 10. BQ25798 CE-Pin Safety (Rev 1.1 — FET-invertiert)

### Problem
Der BQ25798 startet mit Default-Konfiguration (1S Li-ion, 4.2V Ladespannung). Wenn ein LiFePO4-Akku (3.5V max) verbunden ist und der RAK noch nicht gebootet hat, würde der BQ25798 den Akku überladen → **Brandgefahr**.

### Hardware-Design (Rev 1.1 — FET-invertiert)
- **Pin**: `BQ_CE_PIN` = GPIO 4 (P0.04 / WB_IO4)
- **CE-N-FET**: Gate ← GPIO4 (ext. Pull-Down), Drain → CE, Source → GND
- **Externer Pull-Down am Gate**: Zieht Gate LOW wenn GPIO floated → FET OFF
- **Externer Pull-Up am CE**: 100 kΩ auf REGN → CE HIGH wenn FET OFF → **Laden AUS** (BQ25798 CE active-low)
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
- GPIO4 gelatcht HIGH → CE-FET ON → CE LOW → **Laden aktiv**
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

## 11. Statistik-Persistenz

### Aktueller Stand

Die 168h-Ringpuffer-Statistiken (Coulomb Counter, MPPT-Daten, SOC-Zustand) sind **nur im RAM** gespeichert und gehen bei jedem Reboot verloren — egal ob System Sleep oder Cold Boot. Es existiert kein Persistenzmechanismus (weder `.noinit`-Section noch LittleFS-Snapshot).

**Persistente Daten** (überleben Reboots via LittleFS):
- Akkutyp (`batType`)
- Akkukapazität (`batCap`)
- NTC-Kalibrierung (`tcCal`)
- MPPT-Einstellung (`mpptEn`)
- Frostverhalten (`frost`)
- Max. Ladestrom (`maxChrg`)
- LED-Einstellung (`leds_en`)

**Nicht-persistente Daten** (gehen bei Reboot verloren):
- 168h Energie-Ringpuffer (stündliche Charge/Discharge/Solar mAh)
- MPPT-Statistiken (168h MPPT-Aktivitätsbuffer)
- SOC-Prozentwert (wird nach Recovery auf 0% gesetzt, bei "Charging Done" auf 100% synchronisiert)
- Batt-TTL-Berechnung (benötigt mind. 24h Daten nach jedem Neustart)
- Tägliche Energiebilanz (7-Tage-Fenster baut sich nach Neustart neu auf)

Die INA228-Kalibrierung (SHUNT_CAL aus CURRENT_LSB und dem 100mΩ-Shunt) wird bei jedem Boot in `Ina228Driver::begin()` aus festen Konstanten berechnet und braucht deshalb keine Persistenz. Einen Laufzeit-Korrekturfaktor gibt es nicht mehr — bei Rev 1.1 ist er durch das PCB-Layout und die Shunt-Toleranz entbehrlich.

---

## 12. CLI-Befehle

### Getter
```bash
board.bat       # Akkutyp abfragen
                # Ausgabe: liion1s | lifepo1s | lto2s | naion1s | none

board.fmax      # Frost-Ladeverhalten abfragen
                # Ausgabe: 0% | 20% | 40% | 100% (LTO/Na-ion: N/A)

board.imax      # Maximaler Ladestrom abfragen
                # Ausgabe: <strom>mA (z.B. 500mA)

board.mppt      # MPPT-Status abfragen
                # Ausgabe: MPPT=1 | MPPT=0

board.telem     # Echtzeit-Telemetrie mit SOC
                # Ausgabe: B:<V>V/<I>mA/<T>C SOC:<Prozent>% S:<V>V/<SolarStrom>
                # Beispiel: B:3.85V/125.4mA/22C SOC:68.5% S:5.12V/385mA
                # Beispiel: B:3.85V/-8.2mA/N/A SOC:N/A S:0.00V/0mA

board.stats     # Energie-Statistiken (Bilanz + MPPT + Batt-TTL)
                # Ausgabe: <24h>/<3d>/<7d>mAh C:<24h> D:<24h> 3C:<3d> 3D:<3d> 7C:<7d> 7D:<7d> <SOL|BAT> M:<mppt>% BT:<ttl>
                # Beispiel: +125/+45/+38mAh C:200 D:75 3C:150 3D:105 7C:140 7D:102 SOL M:85% BT:N/A
                # Beispiel: -30/-45/-40mAh C:10 D:40 3C:5 3D:50 7C:8 7D:48 BAT M:45% BT:12d0h
                # SOL = Solar-Überschuss, BAT = Energiedefizit
                # BT: Batt-TTL (N/A bei Solar-Überschuss oder <24h Daten)

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

board.batcap    # Akkukapazität
                # Ausgabe: 10000 mAh (gesetzt) oder 2000 mAh (Default; LiFePO4-Default 1500 mAh)
```

### Setter
```bash
set board.bat <type>        # Akkuchemie setzen
                            # Optionen: liion1s | lifepo1s | lto2s | naion1s | none

set board.fmax <wert>       # Frost-Ladestromabsenkung setzen
                            # Optionen: 0% | 20% | 40% | 100%
                            # Begrenzt Ladestrom im T-Cool-Bereich (ca. -2 °C bis +3 °C, siehe JEITA-Tabelle im README)
                            # Bei LTO / Na-ion ohne Wirkung (JEITA deaktiviert)

set board.imax <mA>         # Maximalen Ladestrom setzen
                            # Bereich: 50-1500 mA

set board.mppt <0|1>        # MPPT ein-/ausschalten

set board.batcap <mAh>      # Akkukapazität setzen
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
| `begin()` | InheroMr2Board.cpp | Board-Initialisierung, Wake-up-Check, Early-Boot-Low-Voltage-Check |
| `initiateShutdown()` | InheroMr2Board.cpp | System-Sleep-Shutdown (aufgerufen von tickPeriodic nach ALERT) |
| `configureRTCWake()` | InheroMr2Board.cpp | RTC-Countdown-Timer |
| `rtcInterruptHandler()` | InheroMr2Board.cpp | RTC-INT-ISR (setzt Flag) |
| `queryBoardTelemetry()` | InheroMr2Board.cpp | CayenneLPP-Telemetrie-Erfassung |
| `getLowVoltageSleepThreshold()` | InheroMr2Board.cpp | Chemie-spezifische Sleep-Spannung (INA228 ALERT) |
| `getLowVoltageWakeThreshold()` | InheroMr2Board.cpp | Chemie-spezifische Wake-Spannung (0% SOC) |
| `armLowVoltageAlert()` | BoardConfigContainer.cpp | INA228-BUVL-Alert armen + ISR registrieren |
| `disarmLowVoltageAlert()` | BoardConfigContainer.cpp | INA228-Alert disarmen + ISR detachen |
| `lowVoltageAlertISR()` | BoardConfigContainer.cpp | ISR: setzt lowVoltageAlertFired-Flag (geprüft in tickPeriodic) |
| `tickPeriodic()` | BoardConfigContainer.cpp | Main-Loop-Dispatch: MPPT (60s), SOC (60s), stündlich (60min), Low-V-Check |
| `runMpptCycle()` | BoardConfigContainer.cpp | Einzelner MPPT-Zyklus (Solar-Checks, MPPT-Recovery) |
| `updateBatterySOC()` | BoardConfigContainer.cpp | Coulomb-Counter-SOC-Berechnung |
| `updateHourlyStats()` | BoardConfigContainer.cpp | Stündliche Abtastung in den 168h-Ringpuffer |
| `calculateRollingStats()` | BoardConfigContainer.cpp | Rollierende 24h/3d/7d-Summen + living_on_battery |
| `calculateTTL()` | BoardConfigContainer.cpp | Batt-TTL-Prognose |
| `Ina228Driver::begin()` | lib/Ina228Driver.cpp | 100mΩ-Kalibrierung, ADC-Konfiguration |
| `Ina228Driver::readVBATDirect()` | lib/Ina228Driver.cpp | Statischer Early-Boot-VBAT-Read |

---

## Code-Fragmente (Key Sections)

### INA228 Shutdown Mode
```cpp
// Ina228Driver.cpp — liefert bool: false, wenn der INA228 im Continuous-Modus bleibt
bool Ina228Driver::shutdown() {
  // Betriebsmodus auf Shutdown setzen (MODE = 0x0)
  // Deaktiviert alle Konvertierungen und den Coulomb Counter.
  // Bis zu 3 Versuche mit Readback — I2C-Writes können still fehlschlagen.
  uint16_t adc_config = 0x0000;  // MODE = 0x0 (Shutdown)
  // ... Write + Readback-Retry-Schleife, prüft MODE-Bits [15:12] ...
}
```

### INA228 Wake-up
```cpp
// Ina228Driver.cpp
void Ina228Driver::wakeup() {
  // Continuous-Messmodus mit voller ADC-Konfiguration reaktivieren
  // Konvertierungszeiten aus begin() müssen wiederhergestellt werden —
  // die Defaults sind deutlich kürzer (50µs)
  uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) |  // MODE: Continuous all
                        (INA228_ADC_CT_2074us << 9)      |  // VBUSCT: 2074µs
                        (INA228_ADC_CT_4120us << 6)      |  // VSHCT: 4120µs
                        (INA228_ADC_CT_540us << 3)       |  // VTCT: 540µs
                        (INA228_ADC_AVG_256 << 0);          // AVG: 256 Samples (TX-Peak-Filterung)
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

### Szenario A: Normale Entladung (Low-Voltage System Sleep) - Li-ion
```
t=0:      VBAT = 3.7V → Normal (60s checks, Coulomb Counter läuft)
          Daily balance: Today +150mAh SOLAR
          
t=+1h:    VBAT = 3.5V → Normal (INA228 ALERT nicht getriggert)
          SOC: 45%
          
t=+2h:    VBAT = 3.08V → INA228 ALERT feuert (< 3100mV lowv_sleep_mv)
          - lowVoltageAlertISR() → setzt lowVoltageAlertFired Flag
          - tickPeriodic() erkennt Flag im nächsten tick()
          - board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)
          - CE gelatcht (GPIO4-Latch HIGH → FET ON → CE LOW → Laden aktiv)
          - RTC: Wake in 1h (LOW_VOLTAGE_SLEEP_MINUTES = 60)
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

t=0:      VBAT = 3.08V → INA228 ALERT feuert
          - tickPeriodic() → initiateShutdown()
          - System Sleep mit GPIO-Latch (< 500µA), CE gelatcht LOW (Laden aktiv), RTC-Wake 1h
          
t=+1h:    RTC-Wake → Early Boot → VBAT = 3.05V (noch unter 3300mV)
          - Sofort wieder schlafen (CE bleibt gelatcht LOW → Solar-Laden möglich)
          
t=+2h:    RTC-Wake → VBAT = 2.95V (weiter gesunken, kein Solar)
          - Sofort wieder schlafen
          - Board pendelt weiter mit < 500µA + stündlichem Boot (~0.03mAh)
          
t=+∞:     Bei < 500µA kann der Akku monatelang überleben
          - Sobald Solar verfügbar → VBAT steigt → normaler Boot bei >3300mV
          - KEIN Latching: System kann IMMER von selbst recovern
```

### Szenario C: Energiebilanz-Tracking - LiFePO4
```
Tag 0:    VBAT = 3.2V, SOC = 85%
          24 Stunden-Einträge landen in hours[]: Σ geladen +800mAh (Solar), Σ entladen -450mAh
          last_24h_net = +350mAh → SOLAR

Tag 1:    VBAT = 3.15V, SOC = 72%
          Geladen: +650mAh, Entladen: -520mAh
          last_24h_net = +130mAh → SOLAR

Tag 2:    VBAT = 3.05V, SOC = 58%
          Geladen: +200mAh (dichte Bewölkung), Entladen: -480mAh
          last_24h_net = -280mAh → BAT (living_on_battery = true)

          3-Tage-Durchschnitt: (350+130-280)/3 = +66.7 mAh/Tag
          7-Tage-Durchschnitt: (350+130-280)/7 = +28.6 mAh/Tag
          (168h-Fenster noch teilgefüllt — die Summe wird immer durch 7 geteilt)
          → 7-Tage-Durchschnitt positiv → Batt-TTL bleibt 0 (Anzeige N/A)

Tag 3:    VBAT = 2.95V, SOC = 42%
          Geladen: +150mAh (dichte Bewölkung), Entladen: -500mAh
          last_24h_net = -350mAh → BAT

          3-Tage-Durchschnitt: (130-280-350)/3 = -166.7 mAh/Tag
          7-Tage-Durchschnitt: (350+130-280-350)/7 = -21.4 mAh/Tag → negativ → Batt-TTL wird berechnet
          living_on_battery = true

          Batt-TTL-Berechnung (Basis 7-Tage-Durchschnitt, ≥25 °C → f(T)=1, nichts eingeschlossen):
          gespeichert = 42% × 1500mAh = 630mAh
          Defizit = |-21.4| = 21.4 mAh/Tag
          Batt-TTL = (630 / 21.4) × 24 ≈ 706 Stunden ≈ 29.4 Tage

          CLI-Ausgabe: "-350/-167/-21mAh C:150 D:500 3C:.. 3D:.. 7C:.. 7D:.. BAT M:45% BT:29d10h"
```

---

## Siehe auch

- [README.md](README.md) — Benutzer-Dokumentation und CLI-Referenz
- [DATASHEET.md](DATASHEET.md) — Hardware-Spezifikationen und Pinout
- [TELEMETRY.md](TELEMETRY.md) — Telemetrie-Kanäle erklärt (was die App anzeigt)
- [QUICK_START.md](QUICK_START.md) — Inbetriebnahme und Konfiguration
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Akkuchemie-Vergleich und Einsatzempfehlungen
- [FAQ.md](FAQ.md) — Häufig gestellte Fragen
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle CLI-Befehle auf einen Blick

### Datasheets
- **INA228**: https://www.ti.com/product/INA228
- **RV-3028-C7**: https://www.microcrystal.com/en/products/real-time-clock-rtc-modules/rv-3028-c7/
- **BQ25798**: https://www.ti.com/product/BQ25798
- **TPS62840**: https://www.ti.com/product/TPS62840
- **nRF52840**: https://www.nordicsemi.com/products/nrf52840
