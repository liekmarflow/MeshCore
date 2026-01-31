# Inhero MR-1 Power Management - Implementierungs-Dokumentation

> ✅ **STATUS: v0.2 VOLLSTÄNDIG IMPLEMENTIERT** ✅
> 
> Diese Dokumentation beschreibt die vollständige Power-Management-Implementierung für das Inhero MR-1 Board v0.2.
> Hardware mit INA228 Power Monitor und RV-3028-C7 RTC ist funktional implementiert.
> 
> Datum: 31. Januar 2026
> Version: 2.0 (Implementiert)
> Hardware: v0.2 (INA228 + RTC)

---

## Überblick

Das System kombiniert **3 Schutz-Schichten** + **Coulomb Counter** + **Daily Energy Balance** für optimalen Filesystem-Schutz, Energie-Effizienz und Batterie-Monitoring:

1. **Software Voltage Monitoring** (Adaptive Task) - Dangerzone Detection
2. **RTC Wake-up Management** (RV-3028-C7) - Periodic Recovery Checks
3. **Hardware UVLO** (INA228 Alert → TPS62840 EN) - Ultimate Protection
4. **Coulomb Counter** (INA228) - Real-time SOC Tracking
5. **Daily Energy Balance** (7-day rolling) - Solar vs. Battery Analysis
6. **INA228 Shutdown Mode** - Power saving during SYSTEMOFF (~1µA)

---

## Hardware-Architektur

### v0.2 Komponenten
| Komponente | Funktion | I2C | Pin | Details |
|------------|----------|-----|-----|---------|
| **INA228** | Power Monitor | 0x45 | Alert→TPS_EN | 20mΩ shunt, 1A max, Coulomb Counter |
| **RV-3028-C7** | RTC | 0x52 | INT→GPIO17 | Countdown timer, wake-up |
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC |
| **TPS62840** | Buck Converter | - | EN←INA_Alert | 750mA, EN controlled by INA228 |

### v0.1 Legacy (MCP4652 + TP2120)
Wurde durch INA228 ersetzt. Hardware-Detection erfolgt automatisch bei Boot via I2C-Probe.

---

## 1. Software-Monitoring (Adaptiv)

### Implementierung
- **Task**: `voltageMonitorTask()` in `BoardConfigContainer.cpp` (Zeile 1068-1165)
- **Messung**: BQ25798 15-bit ADC via I²C (batterie.voltage)
- **Frequenz**: Dynamisch 10s/30s/60s
- **Trigger**: `board.getHardwareVersion() == HW_V0_2`

### Monitoring-Intervalle

| Spannung (Li-Ion) | Zustand | Intervall | Aktion |
|-------------------|---------|-----------|--------|
| > 3.6V | NORMAL | 60s | Coulomb Counter + Daily Balance |
| 3.5-3.6V | WARNING | 30s | Load shedding vorbereitet |
| 3.4-3.5V | CRITICAL | 10s | Minimalbetrieb |
| < 3.4V | DANGERZONE | Sofort | `initiateShutdown()` |

### Chemie-spezifische Schwellen

| Chemie | HW UVLO | SW Dangerzone | Wake Threshold |
|--------|---------|---------------|----------------|
| **Li-Ion 1S** | 3.2V | 3.4V (-200mV) | 3.6V (+200mV) |
| **LiFePO4 1S** | 2.8V | 2.9V (-100mV) | 3.0V (+100mV) |
| **LTO 2S** | 4.0V | 4.2V (-200mV) | 4.4V (+200mV) |

**Implementierung**: `InheroMr1Board.cpp` Zeile 578-619
- `getVoltageCriticalThreshold()` - Software Dangerzone
- `getVoltageHardwareCutoff()` - INA228 UVLO Alert
- `getVoltageWakeThreshold()` - Recovery mit Hysterese

---

## 2. Coulomb Counter & SOC (State of Charge)

### INA228 Integration
- **Driver**: `lib/Ina228Driver.cpp` (255 Zeilen, vollständig implementiert)
- **Init**: `BoardConfigContainer::begin()` Zeile 592-641
  - 20mΩ Shunt-Kalibrierung
  - CURRENT_LSB = 1A / 524288 ≈ 1.91µA
  - ADC Range ±40.96mV (optimal für 1A @ 20mΩ)
  - Chemie-spezifischer UVLO-Alert setzen

### SOC-Berechnung
**Methode**: `updateBatterySOC()` in `BoardConfigContainer.cpp` Zeile 1337-1418
- **Primary**: Coulomb Counting (INA228 CHARGE Register)
- **Fallback**: Voltage-based SOC via `estimateSOCFromVoltage()` (Zeile 1491-1541)
- **Update-Intervall**: voltageMonitorTask() ruft auf (10s/30s/60s)

**Formel**:
```
SOC_delta = charge_delta_mah / capacity_mah × 100%
SOC_new = SOC_old + SOC_delta
```

**Auto-Learning** (vorbereitet, nicht aktiv):
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
   
2. **Auto-Learning** (vorbereitet, nicht aktiv):
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

## 3. Daily Energy Balance

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

**3-Day Average Deficit** (für TTL):
```
avg_deficit = (day0 + day1 + day2).net_balance / 3
```

**Living Status**:
- `living_on_battery = true` wenn `avg_deficit < 0` (Netto-Entladung)
- `living_on_solar = true` wenn `avg_deficit > 0` (Netto-Ladung)

---

## 4. Time To Live (TTL) Forecast

### Berechnung
**Methode**: `calculateTTL()` in `BoardConfigContainer.cpp` Zeile 1543-1572
- **Aufgerufen**: Nach updateDailyBalance()
- **Voraussetzung**: living_on_battery == true

**Formel**:
```
remaining_capacity_mah = (SOC% / 100) × capacity_mah
daily_deficit_mah = avg_deficit_3day (negativ)
TTL_hours = remaining_capacity_mah / |daily_deficit_mah| × 24
```

**Beispiel**:
- SOC: 60% = 1200mAh remaining (bei 2000mAh Kapazität)
- 3-day avg: -100 mAh/day
- TTL: 1200 / 100 × 24 = 288 Stunden = 12 Tage

**CLI-Ausgabe**: `board.balance`
```
Today:+150mAh SOLAR 3dAvg:+120mAh
```
oder
```
Today:-80mAh BATTERY 3dAvg:-100mAh TTL:288h
```

---

## 5. RTC Wake-up Management

### RV-3028-C7 Integration
**Pin**: GPIO17 (WB_IO1) → RTC INT
**Init**: `InheroMr1Board::begin()` Zeile 454-481
- `attachInterrupt(RTC_INT_PIN, rtcInterruptHandler, FALLING)`
- Check `GPREGRET2` für wake-up reason

### Countdown-Timer Konfiguration
**Methode**: `configureRTCWake()` in `InheroMr1Board.cpp` Zeile 653-686
- **Tick Rate**: 1Hz (1 Sekunde pro Tick)
- **Max Countdown**: 65535 Sekunden ≈ 18.2 Stunden
- **Standard-Intervall**: 1 Stunde (3600 Ticks)

**Register**:
```cpp
RV3028_CTRL1 (0x00):     TE bit (Timer Enable)
RV3028_CTRL2 (0x01):     TIE bit (Timer Interrupt Enable), TF bit (Timer Flag)
RV3028_COUNTDOWN_LSB (0x09): Countdown value LSB
RV3028_COUNTDOWN_MSB (0x0A): Countdown value MSB
```

### Interrupt Handler
**Methode**: `rtcInterruptHandler()` Zeile 688-703
- **Read-Modify-Write** CTRL2 register
- **Clear nur TF bit** (bit 3), preserve TIE (bit 7)
- **Fehler vorher**: `Wire.write(0x00)` löschte alle Bits → INT blieb LOW
- **Fix**: `ctrl2 &= ~(1 << 3)` clear nur Timer Flag

---

## 6. Power Management Flow

### Shutdown-Sequenz
**Methode**: `initiateShutdown()` in `InheroMr1Board.cpp` Zeile 611-642

**5 Schritte**:
1. **Stop Background Tasks**: `BoardConfigContainer::stopBackgroundTasks()` (Zeile 616)
   - Stoppt MPPT task, Heartbeat task, Voltage Monitor task
   - Verhindert Filesystem-Korruption
   
2. **INA228 Shutdown** (v0.2 only, Zeile 619-623):
   ```cpp
   if (board.getHardwareVersion() == HW_V0_2 && boardConfig.getIna228Driver()) {
     boardConfig.getIna228Driver()->shutdown();  // MODE=0x0 → Power-down
   }
   ```
   - Stoppt alle ADC-Konversionen
   - Deaktiviert Coulomb Counter (kein Zählen bei 0% SOC sowieso)
   - Reduziert INA228-Stromverbrauch auf ~1µA
   
3. **RTC Wake konfigurieren** (bei LOW_VOLTAGE, Zeile 625-633):
   ```cpp
   if (reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
     configureRTCWake(1);  // 1 Stunde
   }
   ```
   
4. **Shutdown-Grund speichern** (Zeile 635):
   ```cpp
   NRF_POWER->GPREGRET2 = reason;  // Persistent über SYSTEMOFF
   ```
   
5. **SYSTEMOFF eintreten** (Zeile 638-641):
   ```cpp
   sd_power_system_off();  // nRF52 Tiefschlaf 1-5µA
   // Never returns
   ```

### Wake-up Check (Anti-Motorboating)
**Methode**: `InheroMr1Board::begin()` Zeile 447-498

**Problem**: Nach Hardware-UVLO ist `GPREGRET2 = 0x00` (kompletter RAM-Verlust)
→ Ohne universelle Voltage Check: Motorboating bei knapper Spannung

**3 Fälle**:

**Case 1: RTC Wake from Software-SHUTDOWN** (`GPREGRET2 = SHUTDOWN_REASON_LOW_VOLTAGE`)
```cpp
if (shutdown_reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
  if (vbat_mv < wake_threshold) {
    configureRTCWake(1);  // 1 Stunde
    sd_power_system_off();
  }
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
}
```

**Case 2: ColdBoot after Hardware-UVLO** (`GPREGRET2 = 0x00`, voltage still low)
```cpp
else if (vbat_mv < wake_threshold) {
  MESH_DEBUG_PRINTLN("ColdBoot with low voltage - likely Hardware-UVLO");
  configureRTCWake(1);
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_LOW_VOLTAGE;
  sd_power_system_off();
}
```
**CRITICAL**: Verhindert Motorboating! Auch nach Hardware-Cutoff wird geprüft.

**Case 3: Normal ColdBoot** (Power-On, Reset button, voltage OK)
```cpp
else {
  // Continue normal boot
}
```

**Direct ADC Read** (boardConfig noch nicht ready):
```cpp
// RAK4630 cannot measure battery voltage - there's no voltage divider on GPIO!
// Must read directly from BQ25798 ADC registers
uint16_t vbat_mv = BqDriver::readVBATDirect(&Wire);

// BqDriver::readVBATDirect() - Static method in lib/BqDriver.cpp Line ~600
// Uses BQ25798 register 0x3B (BQ25798_REG_VBAT_ADC from Adafruit library)
uint16_t BqDriver::readVBATDirect(TwoWire* wire) {
  const uint8_t BQ25798_I2C_ADDR = 0x6B;
  
  wire->beginTransmission(BQ25798_I2C_ADDR);
  wire->write(BQ25798_REG_VBAT_ADC);  // From Adafruit_BQ25798.h
  wire->endTransmission(false);
  
  wire->requestFrom(BQ25798_I2C_ADDR, (uint8_t)2);
  uint8_t msb = wire->read();
  uint8_t lsb = wire->read();
  
  return (msb << 8) | lsb;  // BQ25798 returns voltage directly in mV
}
```

**Voltage Thresholds** (Chemistry-Specific):
| Chemistry | Hardware Cutoff | Software Danger | Wake Threshold | Hysteresis |
|-----------|----------------|-----------------|----------------|------------|
| Li-Ion 1S | 3200mV (INA228 Alert) | 3400mV | 3600mV | 400mV |
| LiFePO4 1S | 2800mV (INA228 Alert) | 2900mV | 3000mV | 200mV |
| LTO 2S | 4000mV (INA228 Alert) | 4200mV | 4400mV | 400mV |

**Motorboating-Prevention-Flow**:
1. Hardware UVLO @ 3.2V → INA228 Alert → TPS62840 EN=LOW → RAK stromlos
2. Solar lädt auf 3.25V → TPS62840 EN=HIGH (50mV Hysterese)
3. RAK bootet → `GPREGRET2 = 0x00` (RAM war gelöscht)
4. **Early Boot Check** → `vbat=3250mV < wake_threshold=3600mV` ✅ DETECTED
5. **Action** → `configureRTCWake(1)` + `GPREGRET2 = LOW_VOLTAGE` + SYSTEMOFF
6. **Result** → Kein Motorboating! System schläft bis 3.6V erreicht

**Stromverbrauch während SYSTEMOFF**:
- nRF52840: 1-5µA
- INA228 (Shutdown): ~1µA
- RV-3028 (Running): ~45nA
- TPS62840 (disabled): <1µA
- **Total**: ~2-6µA

---

## 7. Hardware UVLO (INA228 → TPS62840)

### Funktionsweise
**Alert-Pin Verdrahtung**: INA228.ALERT → TPS62840.EN (direkt)
**Config**: `BoardConfigContainer::begin()` Zeile 614-620

**Schwellenwerte**:
```cpp
switch (batteryType) {
  case LTO_2S:      uvlo_mv = 4000; break;  // 4.0V
  case LIFEPO4_1S:  uvlo_mv = 2800; break;  // 2.8V
  case LIION_1S:    uvlo_mv = 3200; break;  // 3.2V (default)
}

ina228.setUnderVoltageAlert(uvlo_mv);
ina228.enableAlert(true, false, true);  // UVLO only, active-high
```

**INA228 Alert Register** (Ina228Driver.cpp Zeile 200-219):
- BUVL (Bus Under-Voltage Limit): `voltage_mv / 195.3125µV`
- DIAG_ALRT: Enable BUSUL bit (bit 3)
- APOL (Alert Polarity): Active-HIGH (TPS EN = HIGH normal)

**Hardware-Verhalten**:
- VBAT < UVLO → ALERT=LOW → TPS62840 EN=LOW → RAK stromlos (0µA)
- VBAT > UVLO → ALERT=HIGH → TPS62840 EN=HIGH → RAK powered

**Schutzschichten**:
1. Software Dangerzone (3.4V Li-Ion) → Controlled shutdown
2. Hardware UVLO (3.2V Li-Ion) → Emergency cutoff
3. Hysterese beim Wake-up (3.6V Li-Ion) → Verhindert Bounce

---

## 8. Hardware-Version Detection

### Implementierung
**Methode**: `detectHardwareVersion()` in `InheroMr1Board.cpp` Zeile 495-523

**Logik**:
```cpp
// 1. Probe INA228 @ 0x45
uint16_t mfg_id = readRegister16(INA228_REG_MANUFACTURER);
if (mfg_id == 0x5449) {  // "TI"
  uint16_t dev_id = readRegister16(INA228_REG_DEVICE_ID);
  if ((dev_id & 0x0FFF) == 0x228) {
    return HW_V0_2;  // INA228 found
  }
}

// 2. Probe MCP4652 @ 0x2F
Wire.beginTransmission(0x2F);
if (Wire.endTransmission() == 0) {
  return HW_V0_1;  // MCP4652 found
}

// 3. Fallback
return HW_UNKNOWN;
```

**Verwendung**:
- `BoardConfigContainer::begin()` Zeile 592, 662, 697, 709
- `InheroMr1Board::initiateShutdown()` Zeile 619
- `BoardConfigContainer::updateBatterySOC()` Zeile 1337

---

## CLI-Befehle

### Getter (Implemented)
```bash
board.hwver     # Hardware-Version (v0.1 / v0.2)
                # Output: "v0.2 (INA228+RTC)" oder "v0.1 (MCP4652)"
                # Code: InheroMr1Board.cpp Zeile 206-211

board.soc       # Battery State of Charge (v0.2 only)
                # Output: "SOC:67.5% Cap:2000mAh(learned|config)"
                # Code: InheroMr1Board.cpp Zeile 243-251

board.balance   # Daily energy balance & TTL (v0.2 only)
                # Output: "Today:+150mAh SOLAR 3dAvg:+120mAh" oder
                #         "Today:-80mAh BATTERY 3dAvg:-100mAh TTL:288h"
                # Code: InheroMr1Board.cpp Zeile 253-269

board.telem     # Full telemetry
                # Output: "B:3.85V/150mA/22C S:5.20V/85mA Y:3.30V"
                # Code: InheroMr1Board.cpp Zeile 240-242

board.cinfo     # Charger info
                # Output: "PG / CC" (Power Good, Constant Current)
                # Code: InheroMr1Board.cpp Zeile 232-238

board.mpps      # MPPT statistics
                # Output: "MPPT 7d avg: 87.3%, E_daily 3d: 1250mWh"
                # Code: InheroMr1Board.cpp Zeile 228-230

board.conf      # All configuration
                # Output: "B:liion1s F:0% M:1 I:500mA Vco:4.20"
                # Code: InheroMr1Board.cpp Zeile 271-285
```

### Setter (Implemented)
```bash
set board.batcap <mAh>  # Set battery capacity (v0.2 only)
                        # Range: 100-100000 mAh
                        # Example: set board.batcap 2200
                        # Code: InheroMr1Board.cpp Zeile 383-396

set board.bat <type>    # Set battery chemistry
                        # Options: liion1s | lifepo1s | lto2s
                        # Code: InheroMr1Board.cpp Zeile 321-334

set board.imax <mA>     # Set max charge current
                        # Range: 10-1000 mA
                        # Code: InheroMr1Board.cpp Zeile 367-377

set board.mppt <0|1>    # Enable/disable MPPT
                        # Code: InheroMr1Board.cpp Zeile 379-391
```

---

## Dateien-Übersicht

### Hauptimplementierung
| Datei | Zeilen | Beschreibung |
|-------|--------|--------------|
| **InheroMr1Board.h** | 130 | Board-Klasse, Power Management Definitionen |
| **InheroMr1Board.cpp** | 705 | Board-Init, Shutdown, RTC, CLI-Commands |
| **BoardConfigContainer.h** | 242 | Battery Management, SOC, Daily Balance Structures |
| **BoardConfigContainer.cpp** | 1582 | BQ25798, INA228, MPPT, SOC, Daily Balance, Tasks |
| **Ina228Driver.h** | 184 | INA228 Register, Methods, BatteryData Struct |
| **Ina228Driver.cpp** | 255 | INA228 I2C Communication, Calibration, Coulomb Counter |

### Schlüssel-Methoden
| Methode | Datei | Zeile | Funktion |
|---------|-------|-------|----------|
| `detectHardwareVersion()` | InheroMr1Board.cpp | 495-523 | I2C-Probe INA228/MCP4652 |
| `initiateShutdown()` | InheroMr1Board.cpp | 611-642 | 5-Step Shutdown Sequenz |
| `configureRTCWake()` | InheroMr1Board.cpp | 653-686 | RTC Countdown Timer |
| `rtcInterruptHandler()` | InheroMr1Board.cpp | 688-703 | RTC INT Flag Clear (Read-Modify-Write) |
| `voltageMonitorTask()` | BoardConfigContainer.cpp | 1068-1165 | Adaptive Voltage Monitoring |
| `updateBatterySOC()` | BoardConfigContainer.cpp | 1337-1418 | Coulomb Counter SOC Calculation |
| `updateDailyBalance()` | BoardConfigContainer.cpp | 1420-1489 | 7-Day Energy Balance Tracking |
| `calculateTTL()` | BoardConfigContainer.cpp | 1543-1572 | Time To Live Forecast |
| `Ina228Driver::begin()` | Ina228Driver.cpp | 12-47 | 20mΩ Calibration, ADC Config |
| `Ina228Driver::shutdown()` | Ina228Driver.cpp | 79-83 | Power-down Mode |

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
                        (INA228_ADC_AVG_16 << 0);             // 16 samples average
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}
```

### RTC Interrupt Handler Fix
```cpp
// InheroMr1Board.cpp Zeile 688-703
void InheroMr1Board::rtcInterruptHandler() {
  // Read current CTRL2 register
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(0x01);  // CTRL2 register address
  Wire.endTransmission(false);
  Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)1);
  
  if (Wire.available()) {
    uint8_t ctrl2 = Wire.read();
    
    // Clear TF bit (bit 3) by writing 0 to it
    ctrl2 &= ~(1 << 3);  // Clear bit 3 (TF - Timer Flag)
    
    // Write back to clear the flag and release INT pin
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(0x01);  // CTRL2 register
    Wire.write(ctrl2);
    Wire.endTransmission();
  }
}
```

### Hardware Version Check Pattern
```cpp
// Verwendung in mehreren Methoden
if (board.getHardwareVersion() == HW_V0_2) {
  // v0.2 specific code (INA228, RTC)
} else if (board.getHardwareVersion() == HW_V0_1) {
  // v0.1 specific code (MCP4652, TP2120)
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
          - RTC: Wake in 1h
          - SYSTEMOFF (2-6µA total)
          
t=+3.5h:  RTC weckt auf
          - VBAT = 3.35V (noch unter 3.6V)
          - INA228 wakeup
          - Check voltage
          - Zurück zu SYSTEMOFF (1h)
          
t=+4.5h:  RTC weckt auf
          - VBAT = 3.38V (noch unter 3.6V)
          - Zurück zu SYSTEMOFF (1h)
          
t=+5.5h:  Sonne kommt → VBAT = 3.85V
          (Aber: Noch kein RTC-Wake, RAK schläft)
          
t=+6.5h:  RTC weckt auf
          - VBAT = 4.05V (OK, über 3.6V)
          - Normal weiter ✅
          - Coulomb Counter resumes
          - Daily balance continues
```

### Szenario B: Kritische Entladung (Hardware-UVLO) - Li-Ion
```
t=0:      VBAT = 3.35V → Software Dangerzone
          - INA228 shutdown
          - SYSTEMOFF
          
t=+30min: VBAT = 3.15V (weiter gesunken im Sleep!)
          → INA228 ALERT LOW (< 3.2V UVLO)
          → TPS62840 EN = LOW
          → RAK komplett stromlos (0µA)
          → RTC läuft weiter
          → INA228 in Shutdown (1µA)
          
t=+6h:    Solar lädt → VBAT = 3.65V
          → INA228 ALERT HIGH (> 3.2V)
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
          
          3-day avg: (350+130-280)/3 = +66.7 mAh/day → Still SOLAR
          
Day 3:    VBAT = 2.95V, SOC = 42%
          Charge: +150mAh (very cloudy)
          Discharge: -500mAh
          Net balance: -350mAh → BATTERY
          daily_stats[3] = {timestamp, 150, 500, -350, true}
          
          3-day avg: (130-280-350)/3 = -166.7 mAh/day → BATTERY
          living_on_battery = true
          
          TTL calculation:
          remaining = 42% × 1500mAh = 630mAh
          deficit = |-166.7| = 166.7 mAh/day
          TTL = (630 / 166.7) × 24 = 90.7 hours ≈ 3.8 days
          
          CLI output: "Today:-350mAh BATTERY 3dAvg:-167mAh TTL:91h"
```

---

## Testing-Historie

### Phase 1: Compilation ✅
- **Datum**: 31. Januar 2026
- **Status**: Erfolgreich
- **Build**: PlatformIO, Inhero_MR1_repeater environment
- **Exit Code**: 0

### Phase 2: Hardware Detection (TODO)
- I2C-Probe INA228 @ 0x45
- Manufacturer ID: 0x5449 ("TI")
- Device ID: 0x228
- Verify MCP4652 detection fallback @ 0x2F

### Phase 3: INA228 Calibration (TODO)
- Shunt: 20mΩ ± 1%
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

### Phase 6: Power Management End-to-End (TODO)
- Simulate low voltage (3.3V Li-Ion)
- Verify shutdown sequence (all 5 steps)
- Verify INA228 enters shutdown mode
- Verify RTC wake after 1h
- Verify voltage recovery check
- Verify resume at 3.6V+

### Phase 7: Daily Balance (TODO)
- Run 3 days with varying solar
- Verify daily statistics accumulation
- Verify 3-day average calculation
- Verify living_on_battery flag
- Verify TTL calculation

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
**Code**: InheroMr1Board.cpp Zeile 688-703
**Fixed**: 31. Januar 2026

### Issue 3: Hardware Version Detection bei Cold Boot
**Problem**: Nach Hardware-UVLO ist GPREGRET2 = 0x00
**Impact**: Kein SHUTDOWN_REASON_LOW_VOLTAGE erkannt
**Workaround**: Hardware-Cutoff ist fail-safe, normal boot OK
**TODO**: Erweiterte Diagnostik via RTC-Register

---

## Future Enhancements

### Short-term (v0.2.1)
- [ ] Auto-Learning Capacity aktivieren (BQ25798 CHARGE_DONE detection)
- [ ] Load Shedding implementieren (TX power reduction, BLE disable)
- [ ] CLI-Command: `pwrmgt.test shutdown` für Testing
- [ ] CLI-Command: `pwrmgt.rtc status` für RTC diagnostics

### Medium-term (v0.3)
- [ ] Adaptive RTC wake interval (1h → 3h → 6h bei langer Low-Voltage)
- [ ] SOC persistence in LittleFS (survive cold boots)
- [ ] Daily balance persistence (survive cold boots)
- [ ] Web-UI für Energy Dashboard
- [ ] CayenneLPP channel für SOC/Balance

### Long-term (v1.0)
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
- **Variant**: `variants/inhero_mr1/`

### Related Documentation
- [README.md](README.md) - User-facing documentation (EN)
- [README.de.md](README.de.md) - User-facing documentation (DE)
- [Inhero_MR1_Datasheet_DE.md](Inhero_MR1_Datasheet_DE.md) - Hardware datasheet

---

## Changelog

### v2.0 - 31. Januar 2026 (Implementiert)
- ✅ INA228 Driver vollständig implementiert (255 Zeilen)
- ✅ Hardware-Detection via I2C-Probe
- ✅ Coulomb Counter mit SOC-Berechnung
- ✅ Daily Energy Balance (7-day rolling)
- ✅ TTL Forecast Algorithmus
- ✅ RTC Wake-up Management
- ✅ INA228 Shutdown Mode
- ✅ Power Management Flow (5 Schritte)
- ✅ CLI Commands (hwver, soc, balance, batcap)
- ✅ Voltage Monitor Task (adaptive)
- ✅ Hardware UVLO (INA228 → TPS EN)
- ✅ RTC Interrupt Handler Fix (Read-Modify-Write)
- ✅ Chemie-spezifische Schwellen
- ✅ Preferences als Integer-Storage
- ✅ Deutsche + Englische README aktualisiert
- ✅ Compilation erfolgreich (Exit 0)

### v1.0 - 29. Januar 2026 (Design)
- Design-Phase Dokumentation
- Architektur mit TP2120 + MCP4652 (v0.1)
- Grundkonzept RTC Wake-up
- Test-Strategie erstellt

---

## Autoren

**Implementierung**: GitHub Copilot (Claude Sonnet 4.5)
**Hardware-Design**: Inhero GmbH
**Projekt**: MeshCore

**Kontakt**: Siehe README.md

---

*Letzte Aktualisierung: 31. Januar 2026, 15:30 UTC*
*Status: ✅ Vollständig implementiert, Compilation erfolgreich, Hardware-Testing ausstehend*
