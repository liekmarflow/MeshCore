# Telemetry Channels

> 🇩🇪 [Deutsche Version](de/TELEMETRY.md)

The Inhero MR2 transmits telemetry data in [CayenneLPP](https://docs.mydevices.com/docs/lorawan/cayenne-lpp) format across four channels. The MeshCore app displays these as **Channel 1–4**.

---

## Channel 1 — Device Status

Base data from the node.

| Field | Unit | Source | Description |
|-------|------|--------|-------------|
| Battery Level | % / V | INA228 | See note below on SOC workaround |
| Temperature | °C / °F | nRF52840 die | Die temperature of the processor, read via `getMCUTemperature()` and added by the generic MeshCore telemetry path. See [Which Temperature Is Which](#which-temperature-is-which) |

### Battery Level & SOC Workaround

MeshCore currently transmits only battery **voltage** on Channel 1 — there is no native SOC% field. The MeshCore app converts this voltage back to a percentage using a hardcoded **Li-ion discharge curve**. This works well for Li-ion cells but produces wrong readings for LiFePO4, LTO, or Na-ion chemistries (which have a much flatter voltage curve).

The MR2 works around this limitation:

| SOC State | What `getBattMilliVolts()` returns | App displays |
|-----------|------------------------------------|--------------|
| **SOC not yet valid** | Real battery voltage from INA228 | Percentage based on Li-ion curve (may be inaccurate for non-Li-ion) |
| **SOC valid** (coulomb counter calibrated) | Fake Li-ion OCV reverse-mapped from true SOC% (`socToLiIonMilliVolts()`) | Correct percentage — the app's Li-ion curve decodes back to the original SOC% |

> **OCV** = Open Circuit Voltage — the battery's resting voltage without load. The OCV curve (voltage vs. SOC%) is characteristic for each battery chemistry and is used here as a lookup table to reverse-map SOC% back to a voltage the app can interpret.

The SOC becomes valid as soon as a **reference point** exists — either manually via `set board.soc <percent>` or automatically on a "Charging Done" event (which sets SOC to 100%).

Without `set board.batcap <mAh>` a chemistry-typical default capacity (1500–2000 mAh) is assumed. Setting the real capacity is what makes the displayed percentage and the Batt-TTL accurate.

The reverse mapping uses a piecewise-linear Li-ion OCV table (3000 mV at 0% → 4200 mV at 100%). This ensures the app displays the correct coulomb-counted SOC regardless of the actual battery chemistry.

---

## Channel 2 — Environment (BME280)

Data from the BME280 environment sensor (always present on the MR2).

| Field | Unit | Source | Description |
|-------|------|--------|-------------|
| Temperature | °C / °F | BME280 | Ambient temperature |
| Relative Humidity | % | BME280 | Relative humidity |
| Barometric Pressure | hPa | BME280 | Barometric pressure |
| Altitude | m / ft | BME280 | Altitude derived from barometric pressure (reference: sea level) |

> **Note:** The altitude calculation is based on standard sea level pressure (1013.25 hPa) and may deviate depending on weather conditions.

---

## Channel 3 — Battery (INA228 / BQ25798)

High-precision battery data from the INA228 coulomb counter and BQ25798 charge controller.

| Field | LPP Type | Unit | Source | Description |
|-------|----------|------|--------|-------------|
| Voltage | Voltage | V | INA228 | Battery voltage (20-bit ADC, ±0.1% accuracy) |
| SOC | Percentage | % | INA228 | State of charge via coulomb counting — *optional, only when calibrated* |
| Current | Current | A | INA228 | Battery current. Negative = discharging, positive = charging |
| Temperature | Temperature | °C / °F | NTC on the BQ25798 TS pin | Battery temperature — *optional, omitted when unavailable* |
| Batt-TTL | Distance | days | calculated | Estimated time-to-live (remaining runtime) — *optional, only with valid SOC* |

### SOC & Batt-TTL

SOC and Batt-TTL only appear when the coulomb counter has a valid reference point — either a manual SOC set (`set board.soc`) or a "Charging Done" event. The percentage is based on the configured battery capacity (`set board.batcap`; a chemistry-typical default of 1500–2000 mAh is assumed otherwise). Until the SOC is valid, these fields are omitted.

### Batt-TTL Encoding

The Batt-TTL is transmitted as a **CayenneLPP Distance value** in days, since CayenneLPP has no native "duration" type. The MeshCore app displays it as a distance (e.g. "42 m"), but the value represents **days of remaining runtime**.

| Condition | Transmitted Value | Meaning |
|-----------|-------------------|---------|
| Finite Batt-TTL | `ttlHours / 24.0` | Estimated remaining days on battery |
| Surplus (charging > consumption) | `990.0` (sentinel value) | Effectively infinite — device is gaining charge |
| Unknown (SOC not yet valid) | *not sent* | Batt-TTL cannot be calculated yet |

### Battery Temperature

The temperature on this channel is measured at the battery, by the NTC on the BQ25798 TS pin. The BQ25798 reports the TS pin voltage as a percentage of REGN; the firmware decodes it through the Inhero divider (RT1 = 5.6 kΩ, RT2 = 27 kΩ) with a Steinhart-Hart equation, adds the `set board.tccal` offset and runs a plausibility check before the value is transmitted.

Four conditions must hold for a battery temperature to appear:

1. The BQ25798 ADC one-shot completes.
2. The TS ADC channel is enabled. It is switched off while the INA228 reports a battery voltage above 0 and below 3200 mV and no input source is qualified — with TS enabled the BQ25798 ADC needs VBAT ≥ 3.2 V in battery-only operation, and switching TS off drops that threshold to 2.9 V so the solar readings keep working. With an input source qualified the channel stays on, so a cold and nearly empty cell reports its temperature while it is being charged.
3. The decoded value lies within −50 … +90 °C.
4. The value passes the BME280 plausibility check (below).

The battery chemistry plays no part in this decision. The TS channel runs for every chemistry, so LiFePO4, Li-ion, LTO and Na-ion all report a battery temperature when an NTC is fitted and VBAT allows the channel.

Condition 2 is the one that shows up in the field: at a nominal 3.2 V (LiFePO4) or 3.1 V (Na-ion) these chemistries spend much of their discharge curve below 3200 mV, and without solar input the battery temperature reads N/A there even with an NTC fitted — on a solar node, at night. LTO 2S (4.6–5.4 V) stays above the threshold throughout, and Li-ion 1S is above it over most of its curve.

#### BME280 Plausibility Check

A missing or open NTC can decode to a value the window check accepts. With RT1 = 5.6 kΩ and RT2 = 27 kΩ the divider sits on the RT2-only pole. Exactly on the pole the decode returns −99 °C, which the −50 … +90 °C window catches. One TS ADC step (0.09765625 % of REGN) off the pole the value lands around −46 °C, inside that window and indistinguishable from a real deep-cold reading. Each accepted reading is therefore compared against a fresh BME280 measurement:

| BME280 reading | Difference (calibrated NTC value vs. BME280) | Result |
|----------------|----------------------------------------------|--------|
| Above −100 °C and below +100 °C | ≤ 15.0 °C | Value is transmitted |
| Above −100 °C and below +100 °C | > 15.0 °C | Replaced by −999 → N/A |
| Unreadable (returns −999) | not evaluated | Check stands down, value is transmitted unchanged |

A difference of exactly 15.0 °C passes. The comparison uses the calibrated value (raw reading + `set board.tccal` offset).

A rejected reading also does not refresh the cached temperature used for SOC derating; after 5 minutes without an accepted NTC reading, the derating falls back to the BME280. The `set board.tccal` auto-calibration is unaffected — it reads the BQ25798 driver directly with the offset zeroed and skips only the driver's own error codes.

### Temperature Sentinel Values

The BQ25798 driver produces these values:

| Value | Meaning |
|-------|---------|
| −999 °C | I²C communication error |
| −888 °C | ADC not ready: the one-shot did not complete, the TS register still read 0 or 0xFFFF after three retries, or the TS channel was switched off |
| −99 °C | NTC open (not connected), or the decode landed on the RT2-only pole |
| +99 °C | NTC shorted |

Anything outside −50 … +90 °C is converted to −999 in the board layer, and a reading rejected by the BME280 check becomes −999 as well. −999 is therefore the only sentinel that leaves that layer, and it carries three causes at once: an I²C error, a driver error code, or an implausible reading. The three are not distinguishable from outside.

Values ≤ −100 °C are not transmitted: the CayenneLPP temperature field on the battery channel is omitted entirely, and `get board.telem` prints `N/A` in its place.

---

## Channel 4 — Solar (BQ25798)

Solar input data from the BQ25798 charge controller.

| Field | LPP Type | Unit | Source | Description |
|-------|----------|------|--------|-------------|
| Voltage | Voltage | V | BQ25798 | Solar input voltage (VBUS) |
| Current | Current | A | BQ25798 | Solar input current (IBUS) |
| MPPT 7-Day | Percentage | % | Firmware | MPPT activation over the last 7 days. Shows what percentage of time the MPPT regulator was actively harvesting solar energy. |

> **Note — Solar current accuracy:** The BQ25798 IBUS ADC has a resolution of 1 mA (15-bit mode) but exhibits significant measurement error at low currents (~±30 mA). Values below approximately 150 mA should be treated as rough estimates. For precise current measurement, the battery side uses the INA228 instead.

> **Note:** The MPPT percentage is a rolling 7-day average. A low value (e.g. 1%) means the panel rarely delivers enough power to activate the MPPT regulator — e.g. during overcast conditions or suboptimal panel angle.

---

## Which Temperature Is Which

The MR2 reports four temperatures from four different sensors:

| Where it appears | Value | Sensor | Notes |
|------------------|-------|--------|-------|
| Channel 1 | MCU die temperature | nRF52840 on-chip sensor | Added by the generic MeshCore telemetry path (`getMCUTemperature()`), alongside the base battery voltage. Runs warmer than ambient under load |
| Channel 2 | Ambient / board temperature | BME280 | Also the reference for `set board.tccal`, the plausibility check and the SOC-derating fallback |
| Channel 3 | Battery temperature | NTC on the BQ25798 TS pin | Steinhart-Hart decode + `tccal` offset + plausibility check; omitted when unavailable |
| `get board.cinfo`, field `TDIE:` | Charger die temperature | BQ25798 on-chip sensor | Junction temperature of the charge controller, from the last completed ADC one-shot, so up to one telemetry period old. Runs warmer than ambient while charging. Not on any LPP channel |

Only the Channel 3 value is measured at the battery.

---

## Channel Assignment in Code

Channels are assigned dynamically:

1. **Channel 1** (`TELEM_CHANNEL_SELF`) is statically defined and contains the MeshCore base data (battery voltage and MCU die temperature).
2. `querySensors()` assigns each active sensor its own channel starting right after Channel 1 — the BME280 therefore lands on **Channel 2**.
3. The **battery channel** is determined by `queryBoardTelemetry()` as the next free channel (`findNextFreeLppChannel`).
4. The **solar channel** = battery channel + 1.

`querySensors()` assigns the BME280 to Channel 2 before `queryBoardTelemetry()` runs, so battery data lands on Channel 3 and solar on Channel 4 in practice.

```
Order in CayenneLPP packet:
┌──────────────────────────────────────────────────┐
│ Channel 1: Voltage (INA228 / SOC fake)           │  ← MyMesh.cpp (getBattMilliVolts)
│ Channel 2: Temp, Humidity, Pressure, Alt.        │  ← BME280 (querySensors)
│ Channel 3: VBAT, [SOC], IBAT, [TBAT], [Batt-TTL] │  ← queryBoardTelemetry()
│ Channel 4: VSOL, ISOL, MPPT%                     │  ← queryBoardTelemetry()
│ Channel 1: MCU die temperature                   │  ← MyMesh.cpp (getMCUTemperature)
└──────────────────────────────────────────────────┘
```

Fields in square brackets are optional: `[SOC]` and `[Batt-TTL]` need a valid coulomb-counter reference point, `[TBAT]` needs an available battery temperature.

> **Permissions:** Channels 2–4 are only sent if the requesting client has the `TELEM_PERM_ENVIRONMENT` permission. Guests (Guest role) receive only Channel 1 with base voltage and MCU temperature.

## See Also

- [README.md](README.md) — Overview, feature matrix and diagnostics
- [DATASHEET.md](DATASHEET.md) — Hardware specifications and pinout
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — All board-specific CLI commands at a glance
- [QUICK_START.md](QUICK_START.md) — Quick start for commissioning and CLI setup
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Battery chemistry comparison and deployment guide
- [FAQ.md](FAQ.md) — Frequently asked questions
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Complete technical documentation
