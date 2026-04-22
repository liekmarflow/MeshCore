# Telemetry Channels

> 🇩🇪 [Deutsche Version](de/TELEMETRY.md)

The Inhero MR-2 transmits telemetry data in [CayenneLPP](https://docs.mydevices.com/docs/lorawan/cayenne-lpp) format across three channels. The companion app displays these as **Channel 1–3**.

---

## Channel 1 — Device Status & Environment

Base data from the node plus BME280 environment sensor (always present on the MR-2).

| Field | Unit | Source | Description |
|-------|------|--------|-------------|
| Battery Level | % / V | INA228 | See note below on SOC workaround |
| Temperature | °C / °F | BME280 | Ambient temperature |
| Relative Humidity | % | BME280 | Relative humidity |
| Barometric Pressure | hPa | BME280 | Barometric pressure |
| Altitude | m / ft | BME280 | Altitude derived from barometric pressure (reference: sea level) |

### Battery Level & SOC Workaround

MeshCore currently transmits only battery **voltage** on Channel 1 — there is no native SOC% field. The companion app converts this voltage back to a percentage using a hardcoded **Li-Ion discharge curve**. This works well for Li-Ion cells but produces wrong readings for LiFePO₄, LTO, or Na-Ion chemistries (which have a much flatter voltage curve).

The MR-2 works around this limitation:

| SOC State | What `getBattMilliVolts()` returns | App displays |
|-----------|------------------------------------|--------------|
| **SOC not yet valid** | Real battery voltage from INA228 | Percentage based on Li-Ion curve (may be inaccurate for non-Li-Ion) |
| **SOC valid** (coulomb counter calibrated) | Fake Li-Ion OCV reverse-mapped from true SOC% (`socToLiIonMilliVolts()`) | Correct percentage — the app's Li-Ion curve decodes back to the original SOC% |

> **OCV** = Open Circuit Voltage — the battery's resting voltage without load. The OCV curve (voltage vs. SOC%) is characteristic for each battery chemistry and is used here as a lookup table to reverse-map SOC% back to a voltage the app can interpret.

The SOC becomes valid when **both** conditions are met:

1. **Battery capacity is set** — via `set board.batcap <mAh>`
2. **SOC reference point exists** — either manually via `set board.soc <percent>` or automatically on a "Charging Done" event (which sets SOC to 100%)

The reverse mapping uses a piecewise-linear Li-Ion OCV table (3000 mV at 0% → 4200 mV at 100%). This ensures the app displays the correct coulomb-counted SOC regardless of the actual battery chemistry.

> **Note:** The altitude calculation is based on standard sea level pressure (1013.25 hPa) and may deviate depending on weather conditions.

---

## Channel 2 — Battery (INA228 / BQ25798)

High-precision battery data from the INA228 coulomb counter and BQ25798 charge controller.

| Field | LPP Type | Unit | Source | Description |
|-------|----------|------|--------|-------------|
| Voltage | Voltage | V | INA228 | Battery voltage (24-bit ADC, ±0.1% accuracy) |
| SOC | Percentage | % | INA228 | State of charge via coulomb counting — *optional, only when calibrated* |
| Current | Current | A | INA228 | Battery current. Negative = discharging, positive = charging |
| Temperature | Temperature | °C / °F | BQ25798 NTC | Battery temperature from NTC thermistor |
| TTL | Distance | days | calculated | Estimated time-to-live (remaining runtime) — *optional, only with valid SOC* |

### SOC & TTL

SOC and TTL only appear when the coulomb counter has a valid reference point. This requires a configured battery capacity (`set board.batcap`) and either a manual SOC set (`set board.soc`) or a "Charging Done" event. Until the SOC is valid, these fields are omitted.

### TTL Encoding

The TTL (Time-To-Live) is transmitted as a **CayenneLPP Distance value** in days, since CayenneLPP has no native "duration" type. The companion app displays it as a distance (e.g. "42 m"), but the value represents **days of remaining runtime**.

| Condition | Transmitted Value | Meaning |
|-----------|-------------------|---------|
| Finite TTL | `ttlHours / 24.0` | Estimated remaining days on battery |
| Surplus (charging > consumption) | `990.0` (max encodable) | Effectively infinite — device is gaining charge |
| Unknown (SOC not yet valid) | *not sent* | TTL cannot be calculated yet |

### Temperature Sentinel Values

Invalid temperature readings are indicated by sentinel values and not transmitted to the app:

| Value | Meaning |
|-------|---------|
| −999 °C | I²C communication error |
| −888 °C | ADC not ready |
| −99 °C | NTC open (not connected) |
| +99 °C | NTC shorted |

---

## Channel 3 — Solar (BQ25798)

Solar input data from the BQ25798 charge controller.

| Field | LPP Type | Unit | Source | Description |
|-------|----------|------|--------|-------------|
| Voltage | Voltage | V | BQ25798 | Solar input voltage (VBUS) |
| Current | Current | A | BQ25798 | Solar input current (IBUS) |
| MPPT 7-Day | Percentage | % | Firmware | MPPT activation over the last 7 days. Shows what percentage of time the MPPT regulator was actively harvesting solar energy. |

> **Note — Solar current accuracy:** The BQ25798 IBUS ADC has a resolution of 1 mA (15-bit mode) but exhibits significant measurement error at low currents (~±30 mA). Values below approximately 150 mA should be treated as rough estimates. For precise current measurement, the battery side uses the INA228 instead.

> **Note:** The MPPT percentage is a rolling 7-day average. A low value (e.g. 1%) means the panel rarely delivers enough power to activate the MPPT regulator — e.g. during overcast conditions or suboptimal panel angle.

---

## Channel Assignment in Code

Channels are assigned dynamically:

1. **Channel 1** (`TELEM_CHANNEL_SELF`) is statically defined and contains MeshCore base data plus BME280.
2. **Channel 2** is determined by `queryBoardTelemetry()` as the next free channel (`findNextFreeChannel`).
3. **Channel 3** = Channel 2 + 1.

Since Channel 1 is the only channel occupied before `queryBoardTelemetry()`, the result is always Channel 2 (battery) and Channel 3 (solar) in practice.

```
Order in CayenneLPP packet:
┌─────────────────────────────────────────────┐
│ Channel 1: Voltage (INA228 / SOC fake)    │  ← MyMesh.cpp (getBattMilliVolts)
│ Channel 1: Temp, Humidity, Pressure, Alt.  │  ← BME280 (querySensors)
│ Channel 2: VBAT, [SOC], IBAT, TBAT, [TTL] │  ← queryBoardTelemetry()
│ Channel 3: VSOL, ISOL, MPPT%              │  ← queryBoardTelemetry()
└─────────────────────────────────────────────┘
```

> **Permissions:** Channels 2 and 3 are only sent if the requesting client has the `TELEM_PERM_ENVIRONMENT` permission. Guests (Guest role) receive only Channel 1 with base voltage.

## See Also

- [README.md](README.md) — Overview, feature matrix and diagnostics
- [DATASHEET.md](DATASHEET.md) — Hardware specifications and pinout
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — All board-specific CLI commands at a glance
- [QUICK_START.md](QUICK_START.md) — Quick start for commissioning and CLI setup
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Complete technical documentation
