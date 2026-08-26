# Inhero MR2 Power Management - Implementation Documentation (Rev 1.1)

> 🇩🇪 [Deutsche Version](de/POWER_MANAGEMENT.md)

## Table of Contents

- [Overview](#overview)
- [Hardware Architecture](#hardware-architecture)
- [1. Low-Voltage Detection (INA228 ALERT ISR)](#1-low-voltage-detection-ina228-alert-isr)
- [2. Coulomb Counter & SOC (State of Charge)](#2-coulomb-counter--soc-state-of-charge)
- [3. Daily Energy Balance](#3-daily-energy-balance)
- [4. Solar Power Management](#4-solar-power-management)
  - [BQ25798 ADC at Low Battery Voltages](#bq25798-adc-at-low-battery-voltages)
  - [JEITA WARM Zone & VBAT_OVP Prevention](#jeita-warm-zone--vbat_ovp-prevention)
- [5. Batt-TTL Prediction](#5-batt-ttl-prediction)
- [6. RTC Wakeup Management](#6-rtc-wakeup-management)
- [7. Power Management Flow](#7-power-management-flow)
- [8. INA228 ALERT Pin (Rev 1.1)](#8-ina228-alert-pin-rev-11)
- [9. SX1262 Power Control & PE4259 RF Switch](#9-sx1262-power-control--pe4259-rf-switch)
- [10. BQ25798 CE Pin Safety (Rev 1.1 — FET-inverted)](#10-bq25798-ce-pin-safety-rev-11--fet-inverted)
- [11. Statistics Persistence](#11-statistics-persistence)
- [12. CLI Commands](#12-cli-commands)
- [See Also](#see-also)

> This documentation describes the power management implementation for the Inhero MR2 board.
> Hardware Rev 1.1: INA228 ALERT on P1.02, TPS62840 EN via 3.3V_off switch, CE pin via N-FET (inverted).

---

## Overview

The system combines **INA228 ALERT-based low-voltage detection** + **System Sleep with GPIO latch** + **Coulomb Counter** + **daily energy balance** + **CE pin FET safety** for maximum energy efficiency:

1. **INA228 ALERT ISR** (P1.02) - Low-voltage detection via hardware interrupt
2. **System Sleep with GPIO latch** (< 500µA) with RTC wake - Minimal power consumption during low-voltage
3. **CE Pin FET Safety** - Inverted logic, solar charging possible in System Sleep
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
| SOC→Li-ion mV Mapping (workaround) | Active | Will be removed when MeshCore transmits SOC% natively |
| MPPT Recovery + Stuck-PGOOD Handling | Active | Cooldown logic active |


---

## Hardware Architecture

### Components
| Component | Function | I2C | Pin | Details |
|-----------|----------|-----|-----|---------|
| **RAK4630** | Core Module | — | — | nRF52840 SoC + SX1262 LoRa transceiver |
| **INA228** | Power Monitor | 0x40 | ALERT→P1.02 (ISR) | 100mΩ shunt, 1.6A max, Coulomb Counter, BUVL Alert |
| **BME280** | Temp/Humidity/Pressure sensor | 0x76 | — | NTC calibration reference (`set board.tccal`), selftest |
| **RV-3028-C7** | RTC | 0x52 | INT→GPIO17 | Countdown timer, wake-up. See [FAQ #23](FAQ.md#23-why-does-the-repeater-board-need-a-correct-time) |
| **BQ25798** | Battery Charger | 0x6B | INT→GPIO21 | MPPT, JEITA, 15-bit ADC (IBUS ~±30mA error at low currents; ADC has VBAT-dependent thresholds, see [Section 4](#bq25798-adc-at-low-battery-voltages)) |
| **BQ CE Pin** | Charge Enable | — | GPIO4 (P0.04) | Via N-FET: GPIO HIGH → FET ON → CE LOW → charge ON (BQ25798 CE active-low) |
| **TPS62840** | Buck Converter | - | EN via 3.3V_off switch | 750mA, 3.3V rail |
| **CE FET** | N-FET | — | Gate←GPIO4 (ext. pull-down) | Drain→CE, Source→GND. GPIO HIGH → FET ON → CE LOW → charging on. Pull-down defaults gate LOW when floating. |
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
        │ GPREGRET2 → LOW_VOLTAGE_SLEEP flag
        ▼
sd_power_system_off() → System Sleep with GPIO latch (< 500µA)
```

### Chemistry-Specific Thresholds (1-Level System, uniform 200mV hysteresis)

| Chemistry | lowv_sleep_mv (ALERT) | lowv_wake_mv (0% SOC) | Hysteresis |
|-----------|----------------------|----------------------|------------|
| **Li-ion 1S** | 3100 | 3300 | 200mV |
| **LiFePO4 1S** | 2700 | 2900 | 200mV |
| **LTO 2S** | 3900 | 4100 | 200mV |
| **Na-ion 1S** | 2500 | 2700 | 200mV |

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
  - CURRENT_LSB = 1.6384A / 524288 ≈ 3.125µA
  - ADC Range ±163.84mV (ADCRANGE=0, optimal for 1A @ 100mΩ)
  - **ADC Averaging**: 256 samples (filters TX voltage peaks)
  - BUVL Alert configured to `lowv_sleep_mv` (chemistry-specific)

### SOC Calculation
**Method**: `updateBatterySOC()` in `BoardConfigContainer.cpp`
- **Primary**: Coulomb Counting (INA228 CHARGE register)
- **Update interval**: every 60s via tickPeriodic(); no SOC updates while in low-voltage sleep (the RTC wake only checks VBAT and re-sleeps or boots)

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

**Important**: Without correct capacity, SOC% and Batt-TTL calculations are inaccurate!

#### Persistence Mechanism
**Storage Path**: `/inheromr2/batCap.txt` (LittleFS via SimplePreferences)
**Save Method**: `setBatteryCapacity()` in `BoardConfigContainer.cpp` (persists via SimplePreferences)
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
- ✅ **Survives** firmware updates (LittleFS preserved)
- ⚠️ **Lost** on: flash erase, `rm -rf /inheromr2/`, filesystem corruption

---

## 3. Daily Energy Balance

### Tracking (168-Hour Ring Buffer)
**Methods**: `updateHourlyStats()` + `calculateRollingStats()` in `BoardConfigContainer.cpp`
- **Called by**: tickPeriodic() (every 60 min)
- **Sampling**: On each hour boundary (RTC time truncated to full hours), the completed hour is written into the ring buffer

**Data structure**: `BatterySOCStats.hours[168]` (7 days × 24 hours)
```cpp
typedef struct {
  uint32_t timestamp;      // start of hour, unix seconds
  float charged_mah;       // charged this hour
  float discharged_mah;    // discharged this hour
  float solar_mah;         // solar share this hour
} HourlyBatteryStats;
```

The per-hour values are accumulated from INA228 CHARGE register deltas in `updateBatterySOC()` (every 60s): a positive delta counts as charged (and solar), a negative delta as discharged.

### Calculations
**Rolling sums** (`calculateRollingStats()`, after each completed hour):
```
last_24h_net_mah       = Σ(solar − discharged) over the last 24 hours
avg_3day_daily_net_mah = Σ(solar − discharged) over 72h / 3    (needs ≥ 24h of data)
avg_7day_daily_net_mah = Σ(solar − discharged) over 168h / 7   (needs ≥ 24h of data; used for Batt-TTL)
```

**Living Status**:
- `living_on_battery = true` when `last_24h_net_mah < 0` (net deficit over the last 24h)
- Solar surplus (SOL in `board.stats`) is simply `living_on_battery == false`

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

- PFM forward mode is enabled by BQ25798 power-on default (PFM_FWD_DIS=0, REG0x12); the firmware does not modify it
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

**Flag clearing on boot**: `BqDriver::clearInterruptFlags()` (called from `BoardConfigContainer::begin()`)
- Reads the CHARGER_FLAG/FAULT_FLAG registers 0x22–0x27 to de-assert the INT line
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

On the MR2, D+, D−, VAC1, VAC2 are not connected. The firmware enables only the actually used channels:

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
| `setTsWarm(BQ25798_TS_WARM_55C)` | NTC Control 1 (0x18), bits 5:4 | 55 °C (37.7% REGN) | WARM zone starts at ~52 °C (Inhero), not ~42 °C |
| `setJeitaVSet(BQ25798_JEITA_VSET_UNCHANGED)` | NTC Control 0 (0x17), bits 7:5 | UNCHANGED | No VREG reduction in WARM — prevents VBAT_OVP |
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

## 5. Batt-TTL Prediction

> **Batt-TTL** is short for *battery time-to-live* — the estimated remaining runtime on battery. It is not the packet hop limit that "TTL" denotes in mesh networking.

### Data Source and Time Base

The Batt-TTL calculation is based on the **7-day moving average** of daily net energy consumption, calculated from a **168-hour ring buffer** (7 days) of hourly INA228 coulomb counter measurements.

#### Data Flow

```
INA228 Hardware Coulomb Counter (20-bit ADC, ±0.1% accuracy)
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
        │  extractable_mah / |deficit_per_day| × 24 = Batt-TTL hours
        │  (extractable = remaining − trapped charge, see formula below)
        ▼
socStats.ttl_hours → getTTL_Hours() → board.stats / telemetry
```

### Calculation
**Method**: `calculateTTL()` in `BoardConfigContainer.cpp`
- **Called**: After `calculateRollingStats()` (hourly)
- **Time base**: 7-day moving average (`avg_7day_daily_net_mah`) from hourly samples

**Prerequisites for Batt-TTL > 0**:
1. `living_on_battery == true` (24h net is negative, i.e. energy deficit)
2. `avg_7day_daily_net_mah < 0` (7-day average shows net discharge)
3. `capacity_mah > 0` (battery capacity known, via `set board.batcap`)
4. At least **24 hours** of valid data in the ring buffer

**Formula** (Trapped-Charge model):
```
remaining_capacity_mah = (SOC% / 100) × capacity_mah
trapped_mah            = capacity_mah × (1 − f(T))
extractable_mah        = max(0, remaining_capacity_mah − trapped_mah)
daily_deficit_mah = -avg_7day_daily_net_mah  (positive value)
TTL_hours = extractable_mah / daily_deficit_mah × 24
```
f(T) is the chemistry-specific cold-temperature derating factor (`temp_derating_factor`); f(T) = 1 at ≥ 25 °C, so at moderate temperatures nothing is trapped and the formula reduces to remaining/deficit.

**Batt-TTL = 0 means**:
- Device is solar-powered (net surplus) → `living_on_battery == false`
- Less than 24h of data collected (cold start)
- Battery capacity unknown

**Infinite Batt-TTL (telemetry)**:
- When `living_on_battery == false` and SOC valid → transmitted as 990 days (max value)

**Example**:
- SOC: 60% = 1200mAh remaining (with 2000mAh capacity)
- Temperature ≥ 25 °C → f(T) = 1, no trapped charge → extractable = 1200mAh
- 7-day avg: -100 mAh/day (from 168h hourly samples)
- Batt-TTL: 1200 / 100 × 24 = 288 hours = 12 days

**CLI output**: `board.stats`
```
+150/+120/+90mAh C:200 D:50 3C:180 3D:60 7C:160 7D:70 SOL M:85% BT:N/A   ← Solar surplus
```
or
```
-80/-100/-110mAh C:10 D:90 3C:15 3D:115 7C:20 7D:130 BAT M:45% BT:12d0h  ← 12 days until empty
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
- **Max Countdown**: 4095 minutes ≈ 2.8 days (12-bit timer register)
- **Low-Voltage Sleep Interval**: `LOW_VOLTAGE_SLEEP_MINUTES` = 60 min (1h)
- **Rationale**: Each wake is a System-ON reset with an early-boot fast path (minimal I2C: clear RTC TF, read VBAT, re-sleep) costing only ~0.03 mAh

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

The actual TF clear happens in main loop context in `tick()` via I2C (read-modify-write, clears only the TF bit):
```cpp
// In InheroMr2Board::tick() — main loop context:
if (rtc_irq_pending) {
  rtc_irq_pending = false;
  // Read RV3028_REG_STATUS ...
  uint8_t status = Wire.read();
  status &= ~(1 << 3);  // Clear TF bit only → INT pin goes HIGH via pull-up
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_STATUS);
  Wire.write(status);   // write back — other status flags stay untouched
  Wire.endTransmission();
}
```

**Why not in the ISR?** I2C (Wire) must not be called from an ISR context.
The ISR only sets the flag; `tick()` checks it in the main loop.

---

## 7. Power Management Flow

### Shutdown Sequence (Rev 1.1 — System Sleep with GPIO latch)
**Method**: `initiateShutdown()` in `InheroMr2Board.cpp`

**On Low-Voltage → System Sleep with GPIO latch** (< 500µA, CE FET holds state):

**Flow:** INA228 ALERT ISR → Flag → tickPeriodic() → `board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)`:

1. **Stop Background Tasks**: `BoardConfigContainer::stopBackgroundTasks()`
   - Stops heartbeat task (only remaining FreeRTOS task with GPIO)
   - Disarms INA228 low-voltage alert (detach ISR, disable BUVL)

2. **INA228 to minimum current**: release the ALERT pin (`enableAlert(false, ...)`, `setUnderVoltageAlert(0)` — a latched-LOW ALERT would waste ~330µA through the pull-up), then `shutdown()` (ADC off, ~3.5µA)

3. **SX1262 Sleep + PE4259 off**: `inhero::prepareRadioForSystemOff()` — first `radio.sleep(false)` (Cold Sleep via SPI, ~0.16µA), then `digitalWrite(SX126X_POWER_EN, LOW)` (PE4259 VDD off)

4. **LEDs off**: PIN_LED1, PIN_LED2 LOW

5. **Latch CE pin HIGH** (GPIO output latch preserved for P0.04):
   - `digitalWrite(BQ_CE_PIN, HIGH)` → CE FET ON → CE LOW → charging active
   - P0.04 is excluded from `disconnectLeakyPullups()` → GPIO latch stays HIGH in System Sleep
   - Without latch: ext. pull-down on gate → FET OFF → pull-up on CE → CE HIGH → **charging OFF**

6. **INA228 + BQ25798 minimal current**: `inhero::prepareIcsForSystemOff()` (raw-I2C safety net, repeats the INA228 shutdown with readback)

7. **BME280 to sleep**: forced Sleep mode via I2C (saves ~1–7µA; harmless NACK if not populated)

8. **Configure RTC wake**: `configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES)` (60 min)

9. **Clear P0 LATCH for the RTC INT pin** (a stale latch would fire DETECT immediately → instant wake → boot loop)

10. **Release I2C**: `Wire.end()`, then `inhero::disconnectLeakyPullups()` (each held-LOW pull-up wastes ~250µA)

11. **Save shutdown reason**: `NRF_POWER->GPREGRET2 = GPREGRET2_LOW_VOLTAGE_SLEEP | reason`

12. **System Sleep with GPIO latch**: `sd_power_system_off()` → nRF52840 System-Off (< 500µA total)
    - GPIO4 latch preserved (excluded from disconnectLeakyPullups) → FET stays ON → CE LOW → **charging active**
    - RAM contents are lost (168h statistics, SOC, etc.)
    - RTC interrupt on GPIO17 wakes system after timer expires

SOC is not written during shutdown — on the next successful recovery boot, `begin()` calls `setSOCManually(0.0)` (low-voltage recovery), so SOC restarts at 0%.

**Why System Sleep with GPIO latch?**
- N-FET for CE pin → GPIO4 latch preserved HIGH → FET ON → CE LOW → charging active
- Total consumption: **< 500µA** (nRF52840 System-Off + RTC + quiescent currents of all components)

**168h statistics are lost on System Sleep** — no persistence mechanism exists for the ring buffer data. After recovery, statistics start from zero.

### Wake-up Check (Anti-Motorboating)
**Method**: `InheroMr2Board::begin()`

The code checks `GPREGRET2` for shutdown reason and battery voltage for wake-up decisions.

**2 Cases**:

**Case 1: Wake from Low-Voltage Sleep** (`(GPREGRET2 & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE`)
```cpp
// InheroMr2Board::begin() — Early Boot Fast Path (simplified)
uint8_t shutdown_reason = NRF_POWER->GPREGRET2;
if ((shutdown_reason & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE) {
  Wire.begin();
  inhero::clearTimerFlag();  // wake was a reset — the ISR never saw the RTC event
  uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR);
  uint16_t wake_threshold = getLowVoltageWakeThreshold();

  if (vbat_mv == 0 || vbat_mv < wake_threshold) {
    // Voltage still too low → back to System Sleep.
    // The wake reset cleared all PIN_CNF — the sleep-time GPIO latch does NOT
    // survive it. CE must be re-driven OUTPUT HIGH or solar charging stops.
    pinMode(BQ_CE_PIN, OUTPUT);
    digitalWrite(BQ_CE_PIN, HIGH);
    inhero::prepareIcsForSystemOff();        // INA228 + BQ25798 to minimal current
    inhero::prepareRadioForSystemOff(false); // SX1262 back to Cold Sleep
    configureRTCWake(LOW_VOLTAGE_SLEEP_MINUTES);
    inhero::disconnectLeakyPullups();
    NRF_POWER->GPREGRET2 = GPREGRET2_LOW_VOLTAGE_SLEEP | SHUTDOWN_REASON_LOW_VOLTAGE;
    sd_power_system_off();  // Stays in low-voltage sleep cycle
  }
  // Voltage OK → normal boot; low-voltage recovery marking + SOC=0%
  // are applied after boardConfig.begin()
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
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
// Must read directly from INA228 ADC registers (20-bit ADC, left-aligned in 24-bit registers, ±0.1% accuracy)
uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, INA228_I2C_ADDR);
```

**Voltage Thresholds** (Chemistry-Specific, 1-Level System):
| Chemistry | lowv_sleep_mv (ALERT) | lowv_wake_mv (Recovery) | Hysteresis |
|-----------|----------------------|------------------------|------------|
| Li-ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |
| Na-ion 1S | 2500 | 2700 | 200mV |

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

## 10. BQ25798 CE Pin Safety (Rev 1.1 — FET-inverted)

### Problem
The BQ25798 starts with default configuration (1S Li-ion, 4.2V charge voltage). If a LiFePO4 battery (3.5V max) is connected and the RAK has not yet booted, the BQ25798 would overcharge the battery → **fire hazard**.

### Hardware Design (Rev 1.1 — FET-inverted)
- **Pin**: `BQ_CE_PIN` = GPIO 4 (P0.04 / WB_IO4)
- **CE N-FET**: Gate ← GPIO4 (ext. pull-down), Drain → CE, Source → GND
- **External pull-down on Gate**: Defaults gate LOW when GPIO is floating → FET OFF
- **External pull-up on CE**: 100 kΩ to REGN → CE HIGH when FET OFF → **charging OFF** (BQ25798 CE active-low)
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
- GPIO4 latched HIGH → CE FET ON → CE LOW → **charging active**
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

## 11. Statistics Persistence

### Current State

The 168h ring buffer statistics (coulomb counter, MPPT data, SOC state) are stored **in RAM only** and are lost on every reboot — whether System Sleep or cold boot. No persistence mechanism exists (neither `.noinit` section nor LittleFS snapshot).

**Persistent data** (survives reboots via LittleFS):
- Battery type (`batType`)
- Battery capacity (`batCap`)
- NTC calibration (`tcCal`)
- MPPT setting (`mpptEn`)
- Frost behavior (`frost`)
- Max charge current (`maxChrg`)
- LED setting (`leds_en`)

**Non-persistent data** (lost on reboot):
- 168h energy ring buffer (hourly charge/discharge/solar mAh)
- MPPT statistics (168h MPPT activity buffer)
- SOC percentage (set to 0% after recovery, synchronized to 100% on "Charging Done")
- Batt-TTL calculation (requires min. 24h data after each restart)
- Daily energy balance (7-day window rebuilds after restart)

The INA228 calibration (SHUNT_CAL derived from CURRENT_LSB and the 100mΩ shunt) is computed from fixed constants in `Ina228Driver::begin()` on every boot, so it needs no persistence. There is no runtime correction factor any more — on Rev 1.1 the PCB layout and shunt tolerance make one unnecessary.

---

## 12. CLI Commands

### Getters
```bash
board.bat       # Query battery type
                # Output: liion1s | lifepo1s | lto2s | naion1s | none

board.fmax      # Query frost charge behavior
                # Output: 0% | 20% | 40% | 100% (LTO/Na-ion: N/A)

board.imax      # Query maximum charge current
                # Output: <current>mA (e.g. 500mA)

board.mppt      # Query MPPT status
                # Output: MPPT=1 | MPPT=0

board.telem     # Real-time telemetry with SOC
                # Output: B:<V>V/<I>mA/<T>C SOC:<percent>% S:<V>V/<solar current>
                # Example: B:3.85V/125.4mA/22C SOC:68.5% S:5.12V/385mA
                # Example: B:3.85V/-8.2mA/N/A SOC:N/A S:0.00V/0mA

board.stats     # Energy statistics (balance + MPPT + Batt-TTL)
                # Output: <24h>/<3d>/<7d>mAh C:<24h> D:<24h> 3C:<3d> 3D:<3d> 7C:<7d> 7D:<7d> <SOL|BAT> M:<mppt>% BT:<ttl>
                # Example: +125/+45/+38mAh C:200 D:75 3C:150 3D:105 7C:140 7D:102 SOL M:85% BT:N/A
                # Example: -30/-45/-40mAh C:10 D:40 3C:5 3D:50 7C:8 7D:48 BAT M:45% BT:12d0h
                # SOL = Solar surplus, BAT = Energy deficit
                # BT: Batt-TTL (N/A if solar surplus or <24h data)

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
                # Output: 10000 mAh (set) or 2000 mAh (default; LiFePO4 defaults to 1500 mAh)
```

### Setters
```bash
set board.bat <type>        # Set battery chemistry
                            # Options: liion1s | lifepo1s | lto2s | naion1s | none

set board.fmax <value>      # Set frost charge current reduction
                            # Options: 0% | 20% | 40% | 100%
                            # Limits charge current in T-Cool range (approx. -2 °C to +3 °C, see JEITA table in README)
                            # No effect on LTO / Na-ion (JEITA disabled)

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
| `updateHourlyStats()` | BoardConfigContainer.cpp | Hourly sampling into the 168h ring buffer |
| `calculateRollingStats()` | BoardConfigContainer.cpp | 24h/3d/7d rolling sums + living_on_battery |
| `calculateTTL()` | BoardConfigContainer.cpp | Batt-TTL forecast |
| `Ina228Driver::begin()` | lib/Ina228Driver.cpp | 100mΩ calibration, ADC config |
| `Ina228Driver::readVBATDirect()` | lib/Ina228Driver.cpp | Static early-boot VBAT read |

---

## Code Fragments (Key Sections)

### INA228 Shutdown Mode
```cpp
// Ina228Driver.cpp — returns bool: false if the INA228 stays in continuous mode
bool Ina228Driver::shutdown() {
  // Set operating mode to Shutdown (MODE = 0x0)
  // This disables all conversions and Coulomb Counter.
  // Retries up to 3× with readback — I2C writes can fail silently.
  uint16_t adc_config = 0x0000;  // MODE = 0x0 (Shutdown)
  // ... write + readback retry loop, checks MODE bits [15:12] ...
}
```

### INA228 Wake-up
```cpp
// Ina228Driver.cpp
void Ina228Driver::wakeup() {
  // Re-enable continuous measurement mode with full ADC configuration
  // Must restore conversion times from begin() - defaults are much shorter (50µs)
  uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) |  // MODE: Continuous all
                        (INA228_ADC_CT_2074us << 9)      |  // VBUSCT: 2074µs
                        (INA228_ADC_CT_4120us << 6)      |  // VSHCT: 4120µs
                        (INA228_ADC_CT_540us << 3)       |  // VTCT: 540µs
                        (INA228_ADC_AVG_256 << 0);          // AVG: 256 samples (TX peak filtering)
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

### Scenario A: Normal Discharge (Low-Voltage System Sleep) - Li-ion
```
t=0:      VBAT = 3.7V → Normal (60s checks, coulomb counter running)
          Daily balance: Today +150mAh SOLAR
          
t=+1h:    VBAT = 3.5V → Normal (INA228 ALERT not triggered)
          SOC: 45%
          
t=+2h:    VBAT = 3.08V → INA228 ALERT fires (< 3100mV lowv_sleep_mv)
          - lowVoltageAlertISR() → sets lowVoltageAlertFired flag
          - tickPeriodic() detects flag in next tick()
          - board.initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE)
          - CE latched (GPIO4 latch HIGH → FET ON → CE LOW → charging active)
          - RTC: Wake in 1h (LOW_VOLTAGE_SLEEP_MINUTES = 60)
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

t=0:      VBAT = 3.08V → INA228 ALERT fires
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

### Scenario C: Energy Balance Tracking - LiFePO4
```
Day 0:    VBAT = 3.2V, SOC = 85%
          24 hourly entries land in hours[]: Σ charged +800mAh (solar), Σ discharged -450mAh
          last_24h_net = +350mAh → SOLAR

Day 1:    VBAT = 3.15V, SOC = 72%
          Charged: +650mAh, Discharged: -520mAh
          last_24h_net = +130mAh → SOLAR

Day 2:    VBAT = 3.05V, SOC = 58%
          Charged: +200mAh (heavy clouds), Discharged: -480mAh
          last_24h_net = -280mAh → BAT (living_on_battery = true)

          3-day avg: (350+130-280)/3 = +66.7 mAh/day
          7-day avg: (350+130-280)/7 = +28.6 mAh/day
          (168h window still part-filled — the sum is always divided by 7)
          → 7-day avg positive → Batt-TTL stays 0 (shown as N/A)

Day 3:    VBAT = 2.95V, SOC = 42%
          Charged: +150mAh (heavy clouds), Discharged: -500mAh
          last_24h_net = -350mAh → BAT

          3-day avg: (130-280-350)/3 = -166.7 mAh/day
          7-day avg: (350+130-280-350)/7 = -21.4 mAh/day → negative → Batt-TTL is calculated
          living_on_battery = true

          Batt-TTL calculation (7-day avg basis, ≥25 °C → f(T)=1, nothing trapped):
          remaining = 42% × 1500mAh = 630mAh
          deficit = |-21.4| = 21.4 mAh/day
          Batt-TTL = (630 / 21.4) × 24 ≈ 706 hours ≈ 29.4 days

          CLI output: "-350/-167/-21mAh C:150 D:500 3C:.. 3D:.. 7C:.. 7D:.. BAT M:45% BT:29d10h"
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
