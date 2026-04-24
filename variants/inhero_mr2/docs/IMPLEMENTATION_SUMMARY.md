# Inhero MR-2 Power Management - Implementation Documentation (Rev 1.1)

> 🇩🇪 [Deutsche Version](de/IMPLEMENTATION_SUMMARY.md)

## Table of Contents

- [Overview](#overview)
- [Hardware Architecture](#hardware-architecture)
- [1. Low-Voltage Detection (INA228 ALERT ISR)](#1-low-voltage-detection-ina228-alert-isr)
- [2. Coulomb Counter & SOC (State of Charge)](#2-coulomb-counter--soc-state-of-charge)
- [3. Daily Energy Balance](#3-daily-energy-balance)
- [4. Solar Power Management & Interrupt Loop Avoidance](#4-solar-power-management--interrupt-loop-avoidance)
  - [BQ25798 ADC at Low Battery Voltages](#bq25798-adc-at-low-battery-voltages)
  - [JEITA WARM Zone & VBAT_OVP Prevention](#jeita-warm-zone--vbat_ovp-prevention)
- [5. Time-To-Live (TTL) Prediction](#5-time-to-live-ttl-prediction)
- [6. RTC Wakeup Management](#6-rtc-wakeup-management)
- [7. Power Management Flow](#7-power-management-flow)
- [See Also](#see-also)

> This documentation describes the power management implementation for the Inhero MR-2 board.
> Hardware Rev 1.1: INA228 ALERT on P1.02, TPS62840 EN via 3.3V_off switch, CE pin via DMN2004TK-7 FET (inverted).

---

## Overview

The system combines **INA228 ALERT-based low-voltage detection** + **System Sleep with GPIO latch** + **Coulomb Counter** + **daily energy balance** + **CE pin FET safety** for maximum energy efficiency:

1. **INA228 ALERT ISR** (P1.02) - Low-voltage detection via hardware interrupt
2. **System Sleep with GPIO latch** (< 500µA) with RTC wake - Minimal power consumption during low-voltage
3. **CE Pin FET Safety** (DMN2004TK-7) - Inverted logic, solar charging possible in System Sleep
4. **Coulomb Counter** (INA228) - Real-time SOC tracking
5. **Daily Energy Balance** (7-day rolling) - Solar vs. battery
6. **RTC Wakeup Management** (RV-3028-C7) - Periodic recovery checks

### Feature Matrix

| Feature | Status | Notes |
|---------|--------|-------|
| INA228 ALERT → Low-Voltage System Sleep | Active | ISR on P1.02 → volatile flag → tickPeriodic() → System Sleep with GPIO latch + RTC Wake |
| RTC Wakeup (Low-Voltage Recovery) | Active | 60 min (periodic) |
| BQ CE Pin Safety (FET-inverted) | Active | GPIO HIGH → FET ON → CE LOW → charge ON (BQ25798 CE active-low), Dual-Layer: GPIO + I2C |
| System Sleep with latched CE | Active | < 500µA, GPIO4 latch preserved HIGH → FET ON → CE LOW → solar charging possible |
| SOC via INA228 + manual battery capacity | Active | `set board.batcap` available |
| SOC→Li-Ion mV Mapping (workaround) | Active | Will be removed when MeshCore transmits SOC% natively |
| MPPT Recovery + Stuck-PGOOD Handling | Active | Cooldown logic active |


---

## Hardware Architecture

### Components
| Component | Function | I2C | Pin | Details |
|-----------|----------|-----|-----|---------|
| **RAK4630** | Core Module | — | — | nRF52840 SoC + SX1262 LoRa transceiver |
| **INA228** | Power Monitor | 0x40 | ALERT→P1.02 (ISR) | 100mΩ shunt, 1.6A max, Coulomb Counter, BUVL Alert |
| **RV-3028-C7** | RTC | 0x52 | INT→GPIO17 | Countdown timer, wake-up. See [FAQ #23](FAQ.md#23-why-does-the-repeater-board-need-a-correct-time) |
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC (IBUS ~±30mA error at low currents; ADC has VBAT-dependent thresholds, see [Section 4](#bq25798-adc-at-low-battery-voltages)) |
| **BQ CE Pin** | Charge Enable | — | GPIO4 (P0.04) | Via DMN2004TK-7 FET: GPIO HIGH → FET ON → CE LOW → charge ON (BQ25798 CE active-low) |
| **TPS62840** | Buck Converter | - | EN via 3.3V_off switch | 750mA, 3.3V rail |
| **DMN2004TK-7** | CE FET | — | Gate←GPIO4 (ext. pull-down) | N-FET, Drain→CE, Source→GND. GPIO HIGH → FET ON → CE LOW → charging on. Pull-down defaults gate LOW when floating. |
| **Schottky diode** | USB→VBUS Diode | — | — | VBUS-USB → VBUS-BQ (solar input). USB-C CC1/CC2 via 4.7kΩ to GND (USB sink). **⚠ Solar short also shorts VBUS-USB.** |

---

## 1. Low-Voltage Detection (INA228 ALERT ISR)

### Implementation (Rev 1.1 — Flag/Tick Architecture)
- **Trigger**: INA228 BUVL (Bus Under-Voltage Limit) ALERT on P1.02
- **ISR**: `BoardConfigContainer::lowVoltageAlertISR()` → sets `lowVoltageAlertFired = true` (flag only, no FreeRTOS call)
- **Processing**: `tickPeriodic()` checks flag in main loop context → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`
- **Arming**: `armLowVoltageAlert()` is called during battery configuration (sets BUVL threshold + enables ISR)

### Low-Voltage Flow

```
INA228 BUVL Alert (P1.02, FALLING edge)
        │
        ▼
lowVoltageAlertISR()  [ISR context]
        │ Sets lowVoltageAlertFired = true (volatile flag)
        ▼
tickPeriodic()  [Main loop context, next tick()]
        │ Checks lowVoltageAlertFired == true
        ▼
board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)
        │ CE latched HIGH (GPIO latch preserved → FET ON → CE LOW → charging stays ON)
        │ RTC wake configured (LOW_VOLTAGE_SLEEP_MINUTES = 60)
        │ SOC → 0%
        │ GPREGRET2 → LOW_VOLTAGE_SLEEP flag
        ▼
sd_power_system_off() → System Sleep with GPIO latch (< 500µA)
```

### Chemistry-Specific Thresholds (1-Level System, uniform 200mV hysteresis)

| Chemistry | lowv_sleep_mv (ALERT) | lowv_wake_mv (0% SOC) | Hysteresis |
|-----------|----------------------|----------------------|------------|
| **Li-Ion 1S** | 3100 | 3300 | 200mV |
| **LiFePO4 1S** | 2700 | 2900 | 200mV |
| **LTO 2S** | 3900 | 4100 | 200mV |
| **Na-Ion 1S** | 2500 | 2700 | 200mV |

**Implementation**: `BoardConfigContainer` — `battery_properties[]` lookup table
- `lowv_sleep_mv` → INA228 BUVL Alert threshold, triggers System Sleep
- `lowv_wake_mv` → RTC wake threshold (early boot checks VBAT, decides boot or sleep again)
- Static methods: `getLowVoltageSleepThreshold(type)`, `getLowVoltageWakeThreshold(type)`

---

## 2. Coulomb Counter & SOC (State of Charge)

### INA228 Integration
- **Driver**: `lib/Ina228Driver.cpp`
- **Init**: `BoardConfigContainer::begin()`
  - 100mΩ shunt calibration
  - CURRENT_LSB = 1A / 524288 ≈ 1.91µA
  - ADC Range ±163.84mV (ADCRANGE=0, optimal for 1A @ 100mΩ)
  - **ADC Averaging**: 64 samples (filters TX voltage peaks)
  - BUVL Alert configured to `lowv_sleep_mv` (chemistry-specific)

### SOC Calculation
**Method**: `updateBatterySOC()` in `BoardConfigContainer.cpp`
- **Primary**: Coulomb Counting (INA228 CHARGE register)
- **Update interval**: tickPeriodic() calls it (60s normal, hourly during low-voltage RTC wake)

**Formula**:
```
SOC_delta = charge_delta_mah / capacity_mah × 100%
SOC_new = SOC_old + SOC_delta
```

**Auto-Sync**: On BQ25798 "Charge Done", SOC is set to 100%.

### Capacity Management

#### Configuration Required
Battery capacity **must be set manually**, as it varies widely in practice:
- **Typical range**: 4000-24000mAh (4-24Ah)
- **CLI command**: `set board.batcap <mAh>`
- **Allowed range**: 100-100000mAh

**Important**: Without correct capacity, SOC% and TTL calculations are inaccurate!

#### Persistence Mechanism
**Storage Path**: `/prefs/battery_capacity` (LittleFS)
**Save Method**: `saveBatteryCapacity()` in `BoardConfigContainer.cpp`
**Load Method**: `loadBatteryCapacity()`

**Saved on**:
1. **Manual setting**: CLI command `set board.batcap <mAh>`
   - Writes immediately to LittleFS
   - Updates `batteryStats.capacity_mah`

**Loaded on**:
- **Boot time**: `BoardConfigContainer::begin()` calls `loadBatteryCapacity()`
- **Fallback**: When no saved capacity exists
- **Validation**: Range check 100-100000mAh

**Persistence properties**:
- ✅ **Survives** software shutdowns (System Sleep)
- ✅ **Survives** power cycles and low-voltage recovery
- ✅ **Survives** power cycles
- ✅ **Survives** firmware updates (LittleFS preserved)
- ⚠️ **Lost** on: flash erase, `rm -rf /prefs/`, filesystem corruption

---

## 3. Daily Energy Balance

### Tracking (7-Day Rolling Window)
**Method**: `updateDailyBalance()` in `BoardConfigContainer.cpp`
- **Called by**: tickPeriodic()
- **Frequency**: On day change (RTC time % 86400 < 60)

**Data structure**: `BatterySOCStats.daily_stats[7]`
```cpp
struct DailyBatteryStats {
  uint32_t timestamp_day;       // Unix day start
  int32_t total_charge_mah;     // Sum of charges
  int32_t total_discharge_mah;  // Sum of discharges
  int32_t net_balance_mah;      // = charge - discharge
  bool valid;                   // Data available
};
```

### Calculations
**Net Balance**:
```
net_balance = today_charge - today_discharge
```

**7-Day Average Deficit** (for TTL):
```
avg_deficit = (day0 + day1 + ... + day6).net_balance / 7
```

**Living Status**:
- `living_on_battery = true` when `avg_deficit < 0` (net discharge)
- `living_on_solar = true` when `avg_deficit > 0` (net charge)

---

## 4. Solar Power Management

### Design Principle

The BQ25798 decides **itself** via PowerGood (PG) whether an input is usable.
The charger runs in always-active mode (HIZ disabled).
The firmware monitors solar status and re-enables MPPT as needed.

No INT pin interrupt — everything runs via polling in `runMpptCycle()` (60s interval).

### Solar Checks

`runMpptCycle()` performs two checks each cycle:
1. `checkAndFixSolarLogic()` — PG-stuck recovery + MPPT re-enablement
2. `updateMpptStats()` — Updates MPPT statistics for 7-day average

### PFM Forward Mode

- Permanently enabled
- PFM improves efficiency at low solar currents

### MPPT Recovery + PG-Stuck

`checkAndFixSolarLogic()` handles two scenarios:

**PG=1**: MPPT re-enablement — BQ25798 automatically disables MPPT on faults.
Readback check: only write when actual change needed.

**PG=0 + VBUS ≥ 4.5V**: PG-stuck recovery — panel delivers voltage, but BQ has not
qualified the input source (typical during slow sunrise). HIZ toggle forces
new input qualification. 5-minute cooldown prevents excessive toggling.
Constant: `PG_STUCK_VBUS_THRESHOLD_MV = 4500` in BoardConfigContainer.h

### BQ25798 Interrupt Handling

**BQ INT pin (GPIO 21)**: Not used as interrupt — `INPUT_PULLUP` against floating.
BQ status is checked via polling in `runMpptCycle()` every 60s.

**Flag clearing on boot**: `BoardConfigContainer::configureBq()`
- Reads FAULT_STATUS registers (0x20, 0x21) after configuration
- Prevents stale faults from previous power cycle

### Flag/Tick Architecture

All I2C operations run in main loop context via `tickPeriodic()` (called by `InheroMr2Board::tick()`). There are no FreeRTOS tasks for I2C access — this eliminates mutex and race conditions.

**I2C Bus Recovery** (in `InheroMr2Board::begin()`): After OTA/warm reset, an I2C slave may hold SDA low. Before `Wire.begin()`, up to 9 SCL pulses + STOP condition are generated to free the bus.

**tickPeriodic()** dispatches periodic work via `millis()` timers:
```
tickPeriodic()  [called by tick(), main loop]
  ├─ Check low-voltage alert flag → initiateShutdown()
  ├─ Every 60s: runMpptCycle()
  │   ├─ checkAndFixSolarLogic() — PG-stuck recovery (HIZ toggle) + MPPT recovery
  │   └─ updateMpptStats() — Update MPPT statistics
  ├─ Every 60s: updateBatterySOC()
  └─ Every 60min: updateHourlyStats()
```

**Remaining FreeRTOS tasks** (GPIO only, no I2C):
- `heartbeatTask` — blue LED blink pattern
- `ErrorLED` lambda — red LED on missing components

**Timing Summary**:
- **MPPT Cycle**: 60 seconds (via tickPeriodic)
- **SOC Update**: 60 seconds (via tickPeriodic)
- **Hourly Stats**: 60 minutes (via tickPeriodic)

### BQ25798 ADC at Low Battery Voltages

> **Reference:** BQ25798 Datasheet (TI SLUSE22), Section 9.3.16 — ADC

#### Problem

The 15-bit ADC in the BQ25798 has **voltage-dependent operating thresholds** that become relevant in battery-only operation (without solar). At low battery voltages, the ADC cannot complete its conversion — `ADC_EN` stays set and the firmware runs into a timeout.

#### Datasheet Quote (Section 9.3.16)

> *"The ADC is allowed to operate if either VBUS > 3.4V or VBAT > 2.9V is valid.
> At battery only condition, if the TS_ADC channel is enabled, the ADC only works
> when battery voltage is higher than 3.2V, otherwise, the ADC works when the
> battery voltage is higher than 2.9V."*

#### Operating Scenarios

| Condition | VBUS | VBAT | TS Channel | ADC | Temperature |
|-----------|------|------|------------|-----|-------------|
| Solar connected | > 3.4V | any | enabled | ✅ runs | ✅ available |
| Battery operation, normal | — | ≥ 3.2V | enabled | ✅ runs | ✅ available |
| Battery operation, low | — | 2.9–3.2V | **disabled** | ✅ runs | ❌ not available |
| Battery operation, critical | — | < 2.9V | disabled | ❌ timeout | ❌ not available |

#### Firmware Solution: VBAT-dependent TS Channel Control

The firmware reads the current battery voltage from the INA228 and passes it to `BqDriver::getTelemetryData(vbat_mv)`:

- **VBAT ≥ 3.2V** (or unknown): TS channel enabled → ADC threshold 3.2V, temperature available
- **VBAT < 3.2V**: TS channel disabled → ADC threshold drops to 2.9V, temperature shown as "N/A"

This allows the ADC to continue working in the 2.9–3.2V range for solar measurements (VBUS, IBUS), even when battery temperature cannot be read.

#### ADC Channel Configuration (only required channels)

On the MR-2, D+, D−, VAC1, VAC2 are not connected. The firmware enables only the actually used channels:

| Register | Value (TS on) | Value (TS off) | Active Channels |
|----------|---------------|----------------|-----------------|
| 0x2F (ADC_FUNCTION_DISABLE_0) | `0x5A` | `0x5E` | IBUS, VBUS, (TS) |
| 0x30 (ADC_FUNCTION_DISABLE_1) | `0xF0` | `0xF0` | none (D+/D−/VAC disabled) |

**Important:** In one-shot mode, `ADC_EN` is only cleared when **all enabled channels** have completed conversion. Unconnected channels can block this → therefore only required channels are enabled.

#### Temperature Sentinel Values

The firmware uses special return values for invalid temperatures:

| Value | Meaning | Display |
|-------|---------|---------|
| −999.0 | I2C communication error | N/A |
| −888.0 | ADC not ready / TS disabled (low VBAT) | N/A |
| −99.0 | NTC open/not connected | N/A |
| +99.0 | NTC short circuit | N/A |
| −50…+90°C | Valid measurement | XX°C |

**Display rule:** Values ≤ −100°C are shown as "N/A" in CLI and omitted from CayenneLPP packets.

#### Code References
- `BqDriver::getTelemetryData(vbat_mv)` — Main function with VBAT-dependent TS control
- `BqDriver::startADCOneShot(ts_enabled)` — Configures ADC channels and starts conversion
- `BoardConfigContainer::getTelemetryData()` — Passes INA228 VBAT to BqDriver

### JEITA WARM Zone & VBAT_OVP Prevention

#### Problem: Default JEITA Configuration + Inhero Divider

The Inhero MR2 uses a non-standard NTC voltage divider (RT1=5.6 kΩ pullup to REGN, RT2=27 kΩ parallel to GND) instead of the TI reference design (5.24 kΩ / 30.31 kΩ). This shifts TS thresholds lower by a **temperature-dependent** amount: ~5–6 °C in the cold range (where NTC resistance is large relative to RT2, amplifying the divider mismatch) and ~2–3 °C in the warm/hot range.

With the BQ25798 POR defaults (`TS_WARM = 45°C`, `JEITA_VSET = VREG−400mV`, `EN_AUTO_IBATDIS = 1`), this caused a critical failure chain at moderate temperatures (~42 °C):

```
42°C ambient → TS = 44.65% REGN (below VT3_FALL = 44.8%)
  → BQ enters WARM zone
  → JEITA_VSET reduces VREG: 3.5V − 400mV = 3.1V (LiFePO4)
  → Battery at 3.47V > 104% × 3.1V = 3.224V → VBAT_OVP triggers
  → Converter stops, EN_AUTO_IBATDIS sinks IBAT_LOAD = 30mA from battery
  → Total drain: −11mA (system) + −30mA (IBAT_LOAD) = −41mA
  → Recovery requires VBAT < 102% × 3.1V = 3.162V → hours of battery drain
```

#### Fix: Three Register Settings in `configureBaseBQ()`

| Setting | Register | Value | Effect |
|---------|----------|-------|--------|
| `setTsWarm(BQ25798_TS_WARM_55C)` | Charger Control 2 | 55 °C (37.7% REGN) | WARM zone starts at ~52 °C (Inhero), not ~42 °C |
| `setJeitaVSet(BQ25798_JEITA_VSET_UNCHANGED)` | Charger Control 5 | UNCHANGED | No VREG reduction in WARM — prevents VBAT_OVP |
| `JEITA_ISETH` (POR default retained) | NTC Control 0, bits 4:3 | 11b = ICHG unchanged | No charge current reduction in WARM |
| `setAutoIBATDIS(false)` | Charger Control 0, bit 7 | 0 | Disables 30 mA active battery discharge during OVP |

> **Result:** With JEITA_VSET=UNCHANGED and JEITA_ISETH=ICHG unchanged, the WARM zone (T3–T5) is effectively neutralized. Charging continues at full voltage and full current until T-Hot (~58 °C), where charging is suspended entirely.

#### TS Threshold Comparison

| Zone Boundary | BQ Register | % REGN | TI Reference (°C) | Inhero MR2 (°C) | Shift |
|---------------|-------------|--------|--------------------|------------------|-------|
| VT1 (Cold) | — | 72.0% | +3.7 | −2.0 | −5.7 °C |
| VT2 (Cool) | — | 69.8% | +7.9 | +2.8 | −5.1 °C |
| VT3 (Warm) | TS_WARM=55°C | 37.7% | +54.5 | +52.2 | −2.3 °C |
| VT5 (Hot) | — | 34.2% | +59.9 | +57.7 | −2.2 °C |

> NTC models: 103AT (B25/50=3435) for TI reference, NCP15XH103F03RC (B25/85=3380) for Inhero. Typical %REGN from BQ25798 datasheet.

#### Code References
- `BoardConfigContainer::configureBaseBQ()` — Applies all three settings at startup
- `BqDriver::setTsWarm()` / `setJeitaVSet()` — Existing driver API
- `BqDriver::setAutoIBATDIS()` — Added to driver (Charger Control 0, bit 7)

---

## 5. Time-To-Live (TTL) Prediction

### Data Source and Time Base

The TTL calculation is based on the **7-day moving average** of daily net energy consumption, calculated from a **168-hour ring buffer** (7 days) of hourly INA228 coulomb counter measurements.

#### Data Flow

```
INA228 Hardware Coulomb Counter (24-bit ADC, ±0.1% accuracy)
        │
        ▼
updateHourlyStats() — every hour
        │  Stores per hour: charged_mah, discharged_mah, solar_mah
        │  in hours[168] ring buffer (BatterySOCStats.hours[])
        ▼
calculateRollingStats() — after each hourly update
        │  Sums last 168 hours → divides by 7
        │  → avg_7day_daily_net_mah (= solar − discharged per day)
        │  Minimum requirement: ≥ 24 hours of valid data
        ▼
calculateTTL() — after calculateRollingStats()
        │  remaining_mah / |deficit_per_day| × 24 = TTL hours
        ▼
socStats.ttl_hours → getTTL_Hours() → board.stats / telemetry
```

### Calculation
**Method**: `calculateTTL()` in `BoardConfigContainer.cpp`
- **Called**: After `calculateRollingStats()` (hourly)
- **Time base**: 7-day moving average (`avg_7day_daily_net_mah`) from hourly samples

**Prerequisites for TTL > 0**:
1. `living_on_battery == true` (24h net is negative, i.e. energy deficit)
2. `avg_7day_daily_net_mah < 0` (7-day average shows net discharge)
3. `capacity_mah > 0` (battery capacity known, via `set board.batcap`)
4. At least **24 hours** of valid data in the ring buffer

**Formula**:
```
remaining_capacity_mah = (SOC% / 100) × capacity_mah
daily_deficit_mah = -avg_7day_daily_net_mah  (positive value)
TTL_hours = remaining_capacity_mah / daily_deficit_mah × 24
```

**TTL = 0 means**:
- Device is solar-powered (net surplus) → `living_on_battery == false`
- Less than 24h of data collected (cold start)
- Battery capacity unknown

**Infinite TTL (telemetry)**:
- When `living_on_battery == false` and SOC valid → transmitted as 990 days (max value)

**Example**:
- SOC: 60% = 1200mAh remaining (with 2000mAh capacity)
- 7-day avg: -100 mAh/day (from 168h hourly samples)
- TTL: 1200 / 100 × 24 = 288 hours = 12 days

**CLI output**: `board.stats`
```
+150/+120/+90mAh C:200 D:50 3C:180 3D:60 7C:160 7D:70 SOL M:85% T:N/A   ← Solar surplus
```
or
```
-80/-100/-110mAh C:10 D:90 3C:15 3D:115 7C:20 7D:130 BAT M:45% T:12d0h  ← 12 days until empty
```

---

## 6. RTC Wakeup Management

### RV-3028-C7 Integration
**Pin**: GPIO17 (WB_IO1) → RTC INT
**Init**: `InheroMr2Board::begin()`
- `attachInterrupt(RTC_INT_PIN, rtcInterruptHandler, FALLING)`
- Checks `GPREGRET2` for wake-up reason

### Countdown Timer Configuration
**Method**: `configureRTCWake()` in `InheroMr2Board.cpp`
- **Tick Rate**: 1/60 Hz (1 minute per tick), configured via TD=11 in CTRL1
- **Max Countdown**: 65535 minutes ≈ 45 days
- **Low-Voltage Sleep Interval**: `LOW_VOLTAGE_SLEEP_MINUTES` = 60 min (1h)
- **Rationale**: Each wake costs only ~0.03 mAh (I2C read in idle loop, NO full reboot)

**Registers**:
```cpp
RV3028_CTRL1 (0x0F):     TE=1, TD=11 (1/60 Hz), TRPT=0 (Single shot)
RV3028_CTRL2 (0x10):     TIE=1 (Timer Interrupt Enable, bit 4)
RV3028_STATUS (0x0E):    TF (Timer Flag, bit 3) — must be cleared after wake!
RV3028_TIMER_VALUE_0 (0x0A): Countdown value LSB
RV3028_TIMER_VALUE_1 (0x0B): Countdown value MSB (upper 4 bits)
```

### Interrupt Handler
**Method**: `rtcInterruptHandler()` — only sets `rtc_irq_pending = true`.

The actual TF clear happens in the idle loop via I2C:
```cpp
// In initiateShutdown() idle loop:
Wire.beginTransmission(RTC_I2C_ADDR);
Wire.write(RV3028_REG_STATUS);
Wire.write(0x00);  // Clear TF → INT pin goes HIGH via pull-up
Wire.endTransmission();
```

**Why not in the ISR?** I2C (Wire) must not be called from an ISR context.
The ISR only sets the flag, the idle loop checks it after `__WFI()` return.

---

## 7. Power Management Flow

### Shutdown Sequence (Rev 1.1 — System Sleep with GPIO latch)
**Method**: `initiateShutdown()` in `InheroMr2Board.cpp`

**On Low-Voltage → System Sleep with GPIO latch** (< 500µA, CE FET holds state):

**Flow:** INA228 ALERT ISR → Flag → tickPeriodic() → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`:

1. **Stop Background Tasks**: `BoardConfigContainer::stopBackgroundTasks()`
   - Stops heartbeat task (only remaining FreeRTOS task with GPIO)
   - Disarms INA228 low-voltage alert
   
2. **Disarm INA228 ALERT**: `BoardConfigContainer::disarmLowVoltageAlert()` → detach ISR, disable BUVL

3. **Latch CE pin HIGH** (GPIO output latch preserved for P0.04):
   - `digitalWrite(BQ_CE_PIN, HIGH)` → DMN2004TK-7 ON → CE LOW → charging active
   - P0.04 is excluded from `disconnectLeakyPullups()` → GPIO latch stays HIGH in System Sleep
   - Without latch: ext. pull-down on gate → FET OFF → pull-up on CE → CE HIGH → **charging OFF**
   
4. **SX1262 Sleep**: `radio_driver.powerOff()` → `radio.sleep(false)` (Cold Sleep via SPI, ~0.16µA)

5. **PE4259 RF Switch off**: `digitalWrite(SX126X_POWER_EN, LOW)` (VDD off)
   
6. **LEDs off**: PIN_LED1, PIN_LED2 LOW
   
7. **Configure RTC wake**: `configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES)` (60 min)
   
8. **Save shutdown reason**: `NRF_POWER->GPREGRET2 = GPREGRET2_LOW_VOLTAGE_SLEEP | reason`
   
9. **BQ INT pin**: `detachInterrupt(BQ_INT_PIN)` — not used as interrupt (polling), but safety detach before System Sleep

10. **Set SOC to 0%**: `setSOCManually(0.0)` — SOC will start at 0% on recovery

11. **System Sleep with GPIO latch**: `sd_power_system_off()` → nRF52840 System-Off (< 500µA total)
    - GPIO4 latch preserved (excluded from disconnectLeakyPullups) → FET stays ON → CE LOW → **charging active**
    - RAM contents are lost (168h statistics, SOC, etc.)
    - RTC interrupt on GPIO17 wakes system after timer expires

**Why System Sleep with GPIO latch?**
- DMN2004TK-7 FET for CE pin → GPIO4 latch preserved HIGH → FET ON → CE LOW → charging active
- Total consumption: **< 500µA** (nRF52840 System-Off + RTC + quiescent currents of all components)

**168h statistics are lost on System Sleep** — no persistence mechanism exists for the ring buffer data. After recovery, statistics start from zero.

### Wake-up Check (Anti-Motorboating)
**Method**: `InheroMr2Board::begin()`

The code checks `GPREGRET2` for shutdown reason and battery voltage for wake-up decisions.

**2 Cases**:

**Case 1: Wake from Low-Voltage Sleep** (`GPREGRET2 & GPREGRET2_LOW_VOLTAGE_SLEEP`)
```cpp
// InheroMr2Board::begin() — Early Boot Check
if (NRF_POWER->GPREGRET2 & GPREGRET2_LOW_VOLTAGE_SLEEP) {
  uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR);
  uint16_t wake_threshold = BoardConfigContainer::getLowVoltageWakeThreshold(batType);
  
  if (vbat_mv < wake_threshold) {
    // Voltage still too low → immediately back to System Sleep
    configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES);
    sd_power_system_off();  // Stays in low-voltage sleep cycle
  }
  // Voltage OK → mark low-voltage recovery, SOC at 0%, normal boot
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
  boardConfig.setLowVoltageRecovery();
}
```

**Case 2: Normal Cold Boot** (power-on, reset button, voltage OK)
```cpp
else {
  // Continue normal boot
  // INA228 and all other components are initialized
}
```

**Direct ADC Read** (boardConfig not yet ready):
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

**Anti-Motorboating**: The early-boot check in `begin()` prevents the system from repeatedly booting and immediately crashing at marginal voltage. Only when VBAT is above `lowv_wake_mv` does it boot normally.

**Power consumption in System Sleep with GPIO latch (Low-Voltage Sleep)**:
- **Total: < 500µA** (nRF52840 System-Off + RTC + quiescent currents of all components)
- CE FET: GPIO4 latch preserved HIGH → FET ON → CE LOW → **solar charging active**

---

## 8. INA228 ALERT Pin (Rev 1.1)

### Wiring
**Pin**: INA228 ALERT → P1.02 (nRF52840 GPIO, with ext. pull-up)
**TPS62840 EN**: Switched via 3.3V_off slide switch

### Operation
The ALERT pin is used as a **software interrupt**:

1. `armLowVoltageAlert()` configures INA228 BUVL (Bus Under-Voltage Limit) to `lowv_sleep_mv`
2. ALERT fires as FALLING edge interrupt on P1.02
3. ISR (`lowVoltageAlertISR()`) sets `lowVoltageAlertFired = true` (flag only, no FreeRTOS call)
4. `tickPeriodic()` checks flag in the next main loop tick and calls `initiateShutdown()` → System Sleep

**No latch problem**: Since ALERT does not go to TPS62840 EN, there is no latched-off behavior.
The system can boot normally after RTC wake and check voltage in `begin()`.

---

## 9. SX1262 Power Control & PE4259 RF Switch

### Hardware Architecture
- **SX1262**: LoRa transceiver (SPI bus), sleep mode via `SetSleep` SPI command
- **PE4259**: SPDT RF antenna switch in **single-pin mode**:
  - **Pin 6 (VDD)**: GPIO 37 (P1.05, `SX126X_POWER_EN`) — power supply (must be HIGH for operation)
  - **Pin 4 (CTRL)**: SX1262 DIO2 — TX/RX switching (automatic via `setDio2AsRfSwitch(true)`)

### Shutdown Sequence (in `initiateShutdown()`)
The SX1262 is powered down in **two steps** — **order is critical**:

```cpp
// Step 1: SX1262 to Cold Sleep via SPI (MUST be first!)
radio_driver.powerOff();  // → radio.sleep(false) → SPI SetSleep command
delay(10);

// Step 2: PE4259 RF switch power off
digitalWrite(SX126X_POWER_EN, LOW);  // VDD off → PE4259 off
```

**Why this order?**
- `radio.sleep(false)` sends an SPI command to the SX1262 → ensures clean radio shutdown
- PE4259 VDD (GPIO 37) powers the RF switch, NOT the SX1262 directly
- SPI is powered by the nRF52840 3.3V rail, not by PE4259
- For safety: First put SX1262 to sleep, then power off PE4259

### Boot Sequence (in `begin()`)
```cpp
// PE4259 VDD on → RF switch ready
pinMode(SX126X_POWER_EN, OUTPUT);
digitalWrite(SX126X_POWER_EN, HIGH);
delay(10);  // PE4259 power-on time

// Later in radio_init() → target.cpp:
radio.std_init(&SPI);  // → setDio2AsRfSwitch(true) → DIO2 controls TX/RX
```

**Important details**:
- **`SX126X_POWER_EN`** (GPIO 37 / P1.05) controls the **PE4259 VDD**, NOT the SX1262 power
- **`DIO2`** is controlled internally by the SX1262 (`setDio2AsRfSwitch(true)`) — no GPIO needed
- **Sleep current SX1262**: ~0.16µA (Cold Sleep) — datasheet value
- **Without `radio_driver.powerOff()`**: SX1262 remains in RX mode → ~5mA power consumption!

---

## 11. BQ25798 CE Pin Safety (Rev 1.1 — FET-inverted)

### Problem
The BQ25798 starts with default configuration (1S Li-Ion, 4.2V charge voltage). If a LiFePO4 battery (3.5V max) is connected and the RAK has not yet booted, the BQ25798 would overcharge the battery → **fire hazard**.

### Hardware Design (Rev 1.1 — FET-inverted)
- **Pin**: `BQ_CE_PIN` = GPIO 4 (P0.04 / WB_IO4)
- **DMN2004TK-7 N-FET**: Gate ← GPIO4 (ext. pull-down), Drain → CE, Source → GND
- **External pull-down on Gate**: Defaults gate LOW when GPIO is floating → FET OFF
- **External pull-up on CE**: 10kΩ to VSYS → CE HIGH when FET OFF → **charging OFF** (BQ25798 CE active-low)
- **GPIO HIGH** → FET ON → CE pulled to GND (LOW) → **charging ON**
- **GPIO LOW** → pull-down on gate → FET OFF → pull-up on CE → CE HIGH → **charging OFF**
- **GPIO High-Z** (unpowered/reset) → pull-down on gate → FET OFF → pull-up on CE → CE HIGH → **charging OFF**

**Key point Rev 1.1**: Charging is only active when GPIO4 is driven HIGH (by firmware or GPIO output latch in System Sleep). When the RAK is unpowered or unflashed, the external pull-down ensures FET OFF → CE HIGH → **charging disabled** — a deliberate safety feature.

### 3-Layer Protection (Rev 1.1)

| Layer | Location | Mechanism | When |
|---|---|---|---|
| **1. Hardware (passive)** | Pull-down + Pull-up | RAK unpowered → pull-down on gate → FET OFF → pull-up on CE → CE HIGH → **charging OFF** | Always (safety default) |
| **2. Early Boot** | `InheroMr2Board::begin()` | GPIO4 not yet driven → FET OFF → CE HIGH → **charging OFF** until firmware configures it | Before I2C init |
| **3. Chemistry Configuration** | `configureChemistry()` | GPIO HIGH → FET ON → CE LOW → **charging ON** + I2C register for known chemistry | After BQ25798 configuration |

### Dual-Layer Safety (Hardware + Software)

```cpp
// In configureChemistry() — after BQ25798 register configuration:
bq.setChargeEnable(props->charge_enable);     // Software layer (I2C register)
#ifdef BQ_CE_PIN
  pinMode(BQ_CE_PIN, OUTPUT);
  // Rev 1.1 FET-inverted: HIGH → FET ON → CE LOW → charging active (BQ25798: CE active-low)
  // FET OFF → pull-up on CE → CE HIGH → charging disabled (safety default)
  digitalWrite(BQ_CE_PIN, props->charge_enable ? HIGH : LOW);  // HIGH=FET ON=CE LOW=charge on
#endif
```

- `charge_enable` is part of the `BatteryProperties` table
- `BAT_UNKNOWN` → `charge_enable = false` → GPIO LOW → FET OFF → CE HIGH → **charging disabled** + register disabled
- Known chemistry → `charge_enable = true` → GPIO HIGH → FET ON → CE LOW → **charging enabled** + register enabled

### Behavior in System Sleep with GPIO latch (Rev 1.1)

In Rev 1.1, **System Sleep with GPIO latch** is used (via `initiateShutdown()`):
- `digitalWrite(BQ_CE_PIN, HIGH)` is called before entering System Sleep
- P0.04 is excluded from `disconnectLeakyPullups()` → GPIO output latch preserved at HIGH
- GPIO4 latched HIGH → DMN2004TK-7 FET ON → CE LOW → **charging active**
- BQ25798 MPPT/CC/CV runs autonomously in hardware → solar charging possible
- Power consumption: **< 500µA** (nRF52840 System-Off + RTC + quiescent currents of all components)

| State | CE Pin | Charging | Solar Recovery |
|---|---|---|---|
| RAK unpowered (no battery) | HIGH (pull-up, FET OFF) | **Disabled** (safety default) | N/A |
| Early Boot | HIGH (pull-up, GPIO not driven) | **Disabled** (not yet configured) | No |
| BAT_UNKNOWN | HIGH (GPIO LOW → FET OFF) | **Disabled** (CE + I2C register) | No |
| Chemistry configured | LOW (GPIO HIGH → FET ON) | **Active** | **Yes** |
| System Sleep (Low-Voltage) | LOW (GPIO latch HIGH → FET ON) | **Active** | **Yes** |

---

## 12. Statistics Persistence

### Current State

The 168h ring buffer statistics (coulomb counter, MPPT data, SOC state) are stored **in RAM only** and are lost on every reboot — whether System Sleep or cold boot. No persistence mechanism exists (neither `.noinit` section nor LittleFS snapshot).

**Persistent data** (survives reboots via LittleFS):
- Battery type (`batType`)
- Battery capacity (`batCap`)
- INA228 calibration (`ina228Cal`)
- NTC calibration (`tcCal`)
- MPPT setting (`mpptEn`)
- Frost behavior (`frost`)
- Max charge current (`maxChrg`)
- LED setting (`leds`)

**Non-persistent data** (lost on reboot):
- 168h energy ring buffer (hourly charge/discharge/solar mAh)
- MPPT statistics (168h MPPT activity buffer)
- SOC percentage (set to 0% after recovery, synchronized to 100% on "Charging Done")
- TTL calculation (requires min. 24h data after each restart)
- Daily energy balance (7-day window rebuilds after restart)

---

## 13. CLI Commands

### Getters
```bash
board.bat       # Query battery type
                # Output: liion1s | lifepo1s | lto2s | naion1s | none

board.fmax      # Query frost charge behavior
                # Output: 0% | 20% | 40% | 100% (LTO/Na-Ion: N/A)

board.imax      # Query maximum charge current
                # Output: <current>mA (e.g. 500mA)

board.mppt      # Query MPPT status
                # Output: MPPT=1 | MPPT=0

board.telem     # Real-time telemetry with SOC
                # Output: B:<V>V/<I>mA/<T>C SOC:<percent>% S:<V>V/<solar current>
                # Example: B:3.85V/125.4mA/22C SOC:68.5% S:5.12V/385mA
                # Example: B:3.85V/-8.2mA/N/A SOC:N/A S:0.00V/0mA

board.stats     # Energy statistics (balance + MPPT + TTL)
                # Output: <24h>/<3d>/<7d>mAh C:<24h> D:<24h> 3C:<3d> 3D:<3d> 7C:<7d> 7D:<7d> <SOL|BAT> M:<mppt>% T:<ttl>
                # Example: +125/+45/+38mAh C:200 D:75 3C:150 3D:105 7C:140 7D:102 SOL M:85% T:N/A
                # Example: -30/-45/-40mAh C:10 D:40 3C:5 3D:50 7C:8 7D:48 BAT M:45% T:12d0h
                # SOL = Solar surplus, BAT = Energy deficit
                # T: Time To Live (N/A if solar surplus or <24h data)

board.cinfo     # Charger info + last PG-stuck HIZ toggle
                # Output: "PG / CC HIZ:never" or "!PG / !CHG HIZ:3m ago"

board.selftest  # I²C hardware probe (all on-board devices)
                # Output: "INA:OK BQ:OK RTC:OK BME:OK"
                # Per-device states: OK | NACK | WR_FAIL (RTC only)

board.conf      # All configuration values
                # Output: B:<bat> F:<fmax> M:<mppt> I:<imax> Vco:<V> V0:<V>
                # Example: B:liion1s F:0% M:1 I:500mA Vco:4.10 V0:3.30

board.tccal     # NTC temperature calibration offset
                # Output: TC offset: +0.00 C (0.00=default)

board.leds      # LED enable status (Heartbeat + BQ Stat)
                # Output: "LEDs: ON (Heartbeat + BQ Stat)"

board.batcap    # Battery capacity
                # Output: 10000 mAh (set) or 2200 mAh (default)
```

### Setters
```bash
set board.bat <type>        # Set battery chemistry
                            # Options: liion1s | lifepo1s | lto2s | naion1s | none

set board.fmax <value>      # Set frost charge current reduction
                            # Options: 0% | 20% | 40% | 100%
                            # Limits charge current in T-Cool range (0°C to -5°C)
                            # No effect on LTO / Na-Ion (JEITA disabled)

set board.imax <mA>         # Set maximum charge current
                            # Range: 50-1500 mA

set board.mppt <0|1>        # Enable/disable MPPT

set board.batcap <mAh>      # Set battery capacity
                            # Range: 100-100000 mAh

set board.tccal             # Calibrate NTC temperature (auto via BME280)
set board.tccal reset       # Reset offset to 0.00

set board.leds <on|off>     # Enable/disable LEDs (on/1, off/0)

set board.soc <percent>     # Manually set SOC (0-100, INA228 must be ready)
```

---

## File Overview

### Main Implementation
| File | Description |
|------|-------------|
| **InheroMr2Board.h/cpp** | Board class, init, shutdown, RTC, CLI commands |
| **BoardConfigContainer.h/cpp** | Battery management, BQ25798, INA228, MPPT, SOC, daily balance |
| **lib/Ina228Driver.h/cpp** | INA228 I2C communication, calibration, coulomb counter |
| **lib/BqDriver.h/cpp** | BQ25798 I2C communication, MPPT, charging |

### Key Methods
| Method | File | Function |
|--------|------|----------|
| `begin()` | InheroMr2Board.cpp | Board initialization, wake-up check, early-boot low-voltage check |
| `initiateShutdown()` | InheroMr2Board.cpp | System Sleep shutdown (called by tickPeriodic after ALERT) |
| `configureRTCWake()` | InheroMr2Board.cpp | RTC countdown timer |
| `rtcInterruptHandler()` | InheroMr2Board.cpp | RTC INT ISR (sets flag) |
| `queryBoardTelemetry()` | InheroMr2Board.cpp | CayenneLPP telemetry collection |
| `getLowVoltageSleepThreshold()` | InheroMr2Board.cpp | Chemistry-specific sleep voltage (INA228 ALERT) |
| `getLowVoltageWakeThreshold()` | InheroMr2Board.cpp | Chemistry-specific wake voltage (0% SOC) |
| `armLowVoltageAlert()` | BoardConfigContainer.cpp | Arm INA228 BUVL alert + register ISR |
| `disarmLowVoltageAlert()` | BoardConfigContainer.cpp | Disarm INA228 alert + detach ISR |
| `lowVoltageAlertISR()` | BoardConfigContainer.cpp | ISR: sets lowVoltageAlertFired flag (checked in tickPeriodic) |
| `tickPeriodic()` | BoardConfigContainer.cpp | Main loop dispatch: MPPT (60s), SOC (60s), hourly (60min), low-V check |
| `runMpptCycle()` | BoardConfigContainer.cpp | Single MPPT cycle (solar checks, MPPT recovery) |
| `updateBatterySOC()` | BoardConfigContainer.cpp | Coulomb counter SOC calculation |
| `updateDailyBalance()` | BoardConfigContainer.cpp | 7-day energy balance tracking |
| `calculateTTL()` | BoardConfigContainer.cpp | Time To Live forecast |
| `Ina228Driver::begin()` | lib/Ina228Driver.cpp | 100mΩ calibration, ADC config |
| `Ina228Driver::readVBATDirect()` | lib/Ina228Driver.cpp | Static early-boot VBAT read |

---

## Code Fragments (Key Sections)

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
// InheroMr2Board.cpp — ISR only sets flag, no I2C!
void InheroMr2Board::rtcInterruptHandler() {
  rtc_irq_pending = true;
}
// TF clear happens in main loop context (tick())
```

### INA228 Driver Access
```cpp
// Direct access to INA228 driver
if (boardConfig.getIna228Driver() != nullptr) {
  // INA228 specific code
}
```

---

## Scenarios

### Scenario A: Normal Discharge (Low-Voltage System Sleep) - Li-Ion
```
t=0:      VBAT = 3.7V → Normal (60s checks, coulomb counter running)
          Daily balance: Today +150mAh SOLAR
          
t=+1h:    VBAT = 3.5V → Normal (INA228 ALERT not triggered)
          SOC: 45%
          
t=+2h:    VBAT = 3.15V → INA228 ALERT fires (< 3100mV lowv_sleep_mv)
          - lowVoltageAlertISR() → sets lowVoltageAlertFired flag
          - tickPeriodic() detects flag in next tick()
          - board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)
          - CE latched (GPIO4 latch HIGH → FET ON → CE LOW → charging active)
          - RTC: Wake in 1h (LOW_VOLTAGE_SLEEP_MINUTES = 60)
          - SOC → 0%
          - sd_power_system_off() → System Sleep with GPIO latch (< 500µA)
          
t=+3h:    RTC wakes → system boots → early boot check
          - Ina228Driver::readVBATDirect() → VBAT = 3.15V
          - VBAT < lowv_wake_mv (3300mV) → immediately back to sleep
          - configureRTCWake(60) + sd_power_system_off()
          
t=+4h:    RTC wakes → system boots → early boot check
          - VBAT = 3.20V → still below 3300mV → back to sleep
          
t=+5h:    RTC wakes → system boots → early boot check
          - VBAT = 3.45V (solar recovery!)
          - VBAT > lowv_wake_mv (3300mV) → normal boot
          - Low-voltage recovery marked, SOC at 0%
          - Coulomb counter restarts
          - Daily balance rebuilds
```

### Scenario B: Critical Discharge (Rev 1.1 — no hardware UVLO)
```
In Rev 1.1 there is no hardware UVLO (TPS62840 EN via 3.3V_off switch).
The INA228 ALERT on P1.02 serves as software interrupt for System Sleep.

t=0:      VBAT = 3.15V → INA228 ALERT fires
          - tickPeriodic() → initiateShutdown()
          - System Sleep with GPIO latch (< 500µA), CE latched LOW (charging active), RTC wake 1h
          
t=+1h:    RTC wake → early boot → VBAT = 3.05V (still below 3300mV)
          - Immediately back to sleep (CE remains latched LOW → solar charging possible)
          
t=+2h:    RTC wake → VBAT = 2.95V (dropped further, no solar)
          - Immediately back to sleep
          - Board continues cycling at < 500µA + hourly boot (~0.03mAh)
          
t=+∞:     At < 500µA the battery can survive for months
          - As soon as solar available → VBAT rises → normal boot at >3300mV
          - NO latching: system can ALWAYS recover on its own
```

### Scenario C: Daily Balance Tracking - LiFePO4
```
Day 0:    VBAT = 3.2V, SOC = 85%
          Charge: +800mAh (solar)
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

## See Also

- [README.md](README.md) — User documentation and CLI reference
- [DATASHEET.md](DATASHEET.md) — Hardware specifications and pinout
- [TELEMETRY.md](TELEMETRY.md) — Telemetry channels explained (what the app displays)
- [QUICK_START.md](QUICK_START.md) — Commissioning and configuration
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Battery chemistry comparison and deployment guide
- [FAQ.md](FAQ.md) — Frequently asked questions
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — All CLI commands at a glance

### Datasheets
- **INA228**: https://www.ti.com/product/INA228
- **RV-3028-C7**: https://www.microcrystal.com/en/products/real-time-clock-rtc-modules/rv-3028-c7/
- **BQ25798**: https://www.ti.com/product/BQ25798
- **TPS62840**: https://www.ti.com/product/TPS62840
- **nRF52840**: https://www.nordicsemi.com/products/nrf52840
