# Inhero MR2 — FAQ

> 🇩🇪 [Deutsche Version](de/FAQ.md)

## Contents


**⚡ Battery & Chemistry**

1. [Which battery chemistry should I choose?](#1-which-battery-chemistry-should-i-choose)
2. [Can I use battery packs without a built-in NTC?](#2-can-i-use-battery-packs-without-a-built-in-ntc)
3. [Why does current draw increase when battery voltage drops?](#3-why-does-current-draw-increase-when-battery-voltage-drops)

**🔋 Charging & Solar**

4. [What mAh value should I enter for `set board.batcap`?](#4-what-mah-value-should-i-enter-for-set-boardbatcap)
5. [Why is it important to set the maximum charge current with `set board.imax`?](#5-why-is-it-important-to-set-the-maximum-charge-current-with-set-boardimax)
6. [What is frost charging, and how do `fmax` and `jeitaignore` work together?](#6-what-is-frost-charging-and-how-do-fmax-and-jeitaignore-work-together)
7. [Can I charge the board via USB?](#7-can-i-charge-the-board-via-usb)
8. [Which solar panels can I connect?](#8-which-solar-panels-can-i-connect)
9. [The red LED (BQ status LED) blinks slowly and the battery is not charging.](#9-the-red-led-bq-status-led-blinks-slowly-and-the-battery-is-not-charging)
10. [Why doesn't the board charge without flashed firmware?](#10-why-doesnt-the-board-charge-without-flashed-firmware)

**📊 SOC & Monitoring**

11. [Why does the SOC show 0% or N/A?](#11-why-does-the-soc-show-0-or-na)
12. [When should I run `set board.tccal`?](#12-when-should-i-run-set-boardtccal)
13. [How does temperature derating work?](#13-how-does-temperature-derating-work)
14. [What is Batt-TTL?](#14-what-is-batt-ttl)

**🔩 Hardware**

15. [Does the board have reverse polarity protection?](#15-does-the-board-have-reverse-polarity-protection)
16. [What does the "3.3V off" switch do, and when would I use it?](#16-what-does-the-33v-off-switch-do-and-when-would-i-use-it)
17. [What do the LEDs mean?](#17-what-do-the-leds-mean)
18. [Can I operate the board without an antenna?](#18-can-i-operate-the-board-without-an-antenna)
19. [Why does the RTC have no backup battery?](#19-why-does-the-rtc-have-no-backup-battery)
20. [What are the dimensions of the mounting holes?](#20-what-are-the-dimensions-of-the-mounting-holes)
21. [Are interfaces (UART/I2C) exposed on the board?](#21-are-interfaces-uarti2c-exposed-on-the-board)

**⚙️ Firmware**

22. [Are my settings preserved during a firmware update?](#22-are-my-settings-preserved-during-a-firmware-update)
23. [Why does the repeater board need a correct time?](#23-why-does-the-repeater-board-need-a-correct-time)
24. [Why can't the repeater clock be set backwards?](#24-why-cant-the-repeater-clock-be-set-backwards)

---

**⚡ Battery & Chemistry**

### 1. Which battery chemistry should I choose?

The Inhero MR2 supports **Li-ion**, **LiFePO4**, **LTO (2S)**, and **Na-ion**. The right choice depends on your deployment conditions — especially temperature range, available space, and expected service life.

In short: **LiFePO4** for most indoor/temperate setups, **LTO** for extreme cold or maximum cycle life, **Li-ion** when space is tight, **Na-ion** for sustainable cold-weather deployments.

→ **Full guide:** [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Detailed comparison, pros & cons, deployment recommendations, capacity planning, solar sizing, safety tips, and long-term aging.

→ **Setup:** [QUICK_START.md — Step 6](QUICK_START.md#6-set-battery-chemistry) | [CLI_CHEAT_SHEET.md — Quick-Start Recipes](CLI_CHEAT_SHEET.md#quick-start-recipes)

---

### 2. Can I use battery packs without a built-in NTC?

**Yes — the onboard NTC covers this case.** Close the solder bridge on the back side of the board — this activates the onboard NTC (NCP15XH103F03RC, 10 kΩ @ 25 °C, Beta 3380). The TS pin on the battery connector remains unused in this case.

If your battery pack has a built-in NTC, it must be wired between **TS (Pin 3)** and **GND (Pin 2)** (see [DATASHEET.md — Battery Connector](DATASHEET.md#pinout--battery-connector-jst-ph20-3p-left-to-right)). A compatible 10k NTC (Beta ~3380) is sufficient for basic frost protection — however, temperature accuracy will be slightly reduced.

**Important:** Without an NTC (solder bridge open and no external NTC connected), the BQ25798 interprets the TS pin as a frost condition for Li-ion and LiFePO4 — charging is blocked and the BQ status LED blinks. LTO and Na-ion are unaffected: these chemistries need no JEITA supervision, so the firmware runs them with the JEITA override permanently on. For an NTC-less Li-ion or LiFePO4 installation, `set board.jeitaignore 1` takes the TS pin out of the charger's decision entirely — see [FAQ #6](#6-what-is-frost-charging-and-how-do-fmax-and-jeitaignore-work-together) for the gate and what it costs. Fitting an NTC remains the better fix.

**Battery temperature readout:** With an NTC fitted, all four chemistries report a battery temperature — the readout does not depend on the chemistry. Two conditions apply: the BQ25798 measures the TS pin at any battery voltage while an input source is qualified (panel or USB actually supplying power), and on battery alone only from about 3.2 V upward (LiFePO4 and Na-ion spend much of their discharge curve below that, and `get board.telem` then shows `N/A`), and a reading that differs from the onboard BME280 by more than 15 °C is discarded as implausible and also shown as `N/A`. The second check exists because a missing or open NTC does not produce an obviously wrong value — the divider decodes it to a plausible-looking deep-cold reading.

---

### 3. Why does current draw increase when battery voltage drops?

The Inhero MR2 has a high-efficiency **buck converter** that converts the battery voltage down to 3.3 V for the MCU and radio. Because this converter is efficient, the board draws roughly **constant power** (watts), not constant current (amps).

Since Power = Voltage × Current:
- At 4.6 V (LTO full): ~6.3 mA
- At 3.7 V (Li-ion nominal): ~7.8 mA
- At 3.2 V (LiFePO4 nominal): ~9.1 mA

All three cases consume exactly **29 mW**. This is normal, not a fault.

**Practical consequence:** When sizing batteries, always calculate in **Wh** (energy), not mAh — especially when comparing different chemistries. A naive "mA × hours" calculation overestimates the capacity needed for higher-voltage chemistries like LTO.

→ **Full explanation:** [BATTERY_GUIDE.md — Why current depends on battery voltage](BATTERY_GUIDE.md#why-current-depends-on-battery-voltage)

---

**🔋 Charging & Solar**

### 4. What mAh value should I enter for `set board.batcap`?

Enter the **nominal capacity minus a deduction**. Since the charge cutoff voltage is reduced for battery longevity, the full nominal capacity is not available. A slightly pessimistic value is safer: when the SOC shows 10 %, there really is ≥ 10 % left in the battery. This prevents being surprised by an unexpected low-voltage sleep. The Batt-TTL prediction also becomes more conservative and reliable.

**Rule of thumb: 90 % of nominal capacity.** Example: 10,000 mAh nominal → `set board.batcap 9000`.

For parallel cells, add capacities before the deduction: Two 5,000 mAh cells in parallel = 10,000 mAh nominal → `set board.batcap 9000`.

---

### 5. Why is it important to set the maximum charge current with `set board.imax`?

`imax` sets the **maximum charge current** — the maximum current flowing into the battery. The firmware also uses `imax` together with the configured chemistry's charge voltage to automatically calculate how much current it may draw from the solar panel. This prevents weak panels from being overloaded and the charger from shutting down.

Why set `imax` correctly?

1. **Basis for frost protection:** `imax` is the reference value for `fmax`. Example: `imax 500` with `fmax 20%` results in a maximum of 100 mA charge current in the T-Cool range (+3 °C to –2 °C).

2. **Battery care:** Lower charge currents are always gentler on the battery. Set `imax` only as high as necessary.

3. **Panel compatibility:** If `imax` is set too high, the board briefly tries to draw more current from the panel than it can deliver — the charger detects the voltage drop and stops charging.

4. **Ceiling for the JEITA override:** `set board.jeitaignore 1` only becomes active while `imax` is at or below 0.05C of `board.batcap` — 9,000 mAh allows up to `imax 450`. On a site where the override is wanted, that ceiling can sit well below what the panel could deliver (see [FAQ #6](#6-what-is-frost-charging-and-how-do-fmax-and-jeitaignore-work-together)).

**Calculation:** Panel power ÷ battery voltage = imax.
Example: 2 W panel, Li-ion (3.7 V) → 2000 / 3.7 ≈ 540 mA → `set board.imax 540`.

---

### 6. What is frost charging, and how do `fmax` and `jeitaignore` work together?

**JEITA** stands for the *Japan Electronics and Information Technology Industries Association*, which in 2007 — together with the battery industry — published guidelines for charging lithium batteries safely. Their core idea is a division into temperature zones: full charging only in the middle zone, reduced charge current or charge voltage toward the edges, and charging suspended entirely beyond them. Chargers like the BQ25798 implement this zone model in hardware, on the MR2 via the NTC on the TS pin. In everyday use, "JEITA" therefore simply means the charger's built-in temperature guard.

On the MR2 the zones sit at (with the Inhero voltage divider; exact thresholds in the [JEITA table in the README](README.md#jeita-temperature-zone-configuration)):

| JEITA zone | Temperature | Charging |
|---|---|---|
| T-Cold | below –2 °C | suspended |
| T-Cool | –2 °C to +3 °C | reduced — the zone `fmax` governs |
| Normal | +3 °C to ~52 °C | full current |
| T-Warm | ~52 °C to ~58 °C | full current (the JEITA reduction here is neutralized in firmware) |
| T-Hot | above ~58 °C | suspended |

"Frost charging" means charging in the two cold zones, where this guard normally steps in: in light frost (T-Cool) `fmax` sets the allowed rate, while the hardware block of hard frost (T-Cold) falls only to `jeitaignore`.

The two settings combine into three stages:

| Configuration | T-Cool (+3 °C to –2 °C) | T-Cold (below –2 °C) |
|---|---|---|
| `jeitaignore 0`, `fmax 0%` *(default)* | blocked | blocked |
| `jeitaignore 0`, `fmax` 20/40/100% | charges at fmax % of `imax` | blocked |
| `jeitaignore 1` *(within the 0.05C gate)* | charges at full `imax` | charges at full `imax` |

Stage one is the conservative default: no charging below +3 °C. Stage two is bolder: reduced charging in light frost, hard frost stays blocked. Stage three is full frost charging: JEITA is switched off entirely — the hot-side charge suspend included — and `fmax` is out of the game (`N/A`).

**`fmax` — the dial for the T-Cool zone**

`fmax` limits the maximum charge current in the **T-Cool range** (+3 °C to –2 °C with the Inhero voltage divider) to a percentage of `imax`:

| Setting | Behavior in T-Cool range |
|---|---|
| `0%` | Charging completely blocked |
| `20%` | Max. 20 % of imax (e.g., 500 mA → 100 mA) |
| `40%` | Max. 40 % of imax (e.g., 500 mA → 200 mA) |
| `100%` | No reduction, full charge current |

**Below approx. –2 °C (T-Cold),** charging is completely blocked by JEITA for **Li-ion and LiFePO4** — regardless of `fmax`. `set board.jeitaignore 1` lifts that block within the gate; see below.

**Important:** Only charging is restricted. With sufficient solar power, the board continues to run on solar — the battery is neither charged nor discharged.

**LTO / Na-ion:** `fmax` has no effect. These chemistries need no JEITA supervision, so the firmware runs them with the JEITA override permanently on: `get board.fmax` answers `N/A`, and `set board.fmax` is refused with `Err: Fmax setting N/A for this chemistry (JEITA disabled)`.

**Full frost charging: `set board.jeitaignore 1`**

The command sets the TS_IGNORE bit in the BQ25798, which makes the charger treat the TS pin as always good: the cold block below –2 °C and the hot-side suspend above ~58 °C both stop acting, and the firmware has no software replacement for either. Charging continues through frost at the configured `imax`. It is off by default and is only accepted for Li-ion and LiFePO4; for LTO and Na-ion it is refused with `Err: This chemistry runs without JEITA (always 1)`.

The override is bounded by a gate: `set board.batcap` must have been set, and `imax` must be at or below **0.05C** of that capacity — 5,000 mAh allows up to `imax 250`. (With the 50 mA lower limit of `imax`, the gate can only pass from about 1,000 mAh upward.) Inside the gate the reply is `jeitaignore set to 1`. Outside it, the setting is stored but stays inactive and the reply names the blocker: `jeitaignore set to 1, N/A, C>0.05` or `jeitaignore set to 1, N/A, batcap not set`. Nothing is discarded in that case — a later `set board.imax` or `set board.batcap` that satisfies the gate arms the override on its own and says so with `; jeitaignore 1`. `get board.jeitaignore` reports the current state.

**What it costs:** Charging a cold Li-ion or LiFePO4 cell is at the operator's own risk. The 0.05C gate bounds the rate; lithium plating on the graphite anode stays cumulative and permanent, and it shows up as capacity quietly gone.

→ **Field experience, the temperature scope and the full argument:** [BATTERY_GUIDE.md — Charging in Cold Conditions](BATTERY_GUIDE.md#charging-in-cold-conditions)

On Li-ion and LiFePO4 with the override armed, `set board.fmax` is refused with `Err: Fmax N/A while jeitaignore is on`, and `get board.conf` appends ` J:1`. `get board.fmax` answers `N/A` for as long as the override is active, on every chemistry. `set board.jeitaignore 0` switches it off again, and the stored `fmax` setting takes effect once more.

---

### 7. Can I charge the board via USB?

**Yes.** USB-C VBUS (5 V) is connected to the BQ25798 VBUS input via a **Schottky diode** — the **same single input** as the solar panel (see [DATASHEET.md — USB Charging Path](DATASHEET.md#usb-charging-path)). The BQ25798 has only one VBUS input and does not distinguish between the two sources.

When USB is detected (nRF52840 VBUS sense), the firmware automatically limits the input current to **500 mA** (USB 2.0 spec). When USB is removed, the input current limit is recalculated from the configured chemistry and `board.imax`.

Whichever source provides the higher voltage at the VBUS input is active: If USB voltage (minus Schottky drop) exceeds the solar voltage, USB charges. Otherwise, solar charges. Both sources cannot charge simultaneously.

> **⚠ WARNING:** The Schottky diode prevents backflow from the solar panel to the USB bus, but current **can** flow from USB-VBUS out through the solar connector. A **short circuit on the solar connector will also short USB-VBUS**. Never short-circuit the solar input while USB is connected.

---

### 8. Which solar panels can I connect?

**Requirements:**
- **Input voltage:** 3.6 V – 24 V (MPPT range of the BQ25798)
- **Max. open-circuit voltage (Voc):** 25 V — do not exceed!
- **Connector:** JST PH2.0-2P (Solar+, Solar–)

**Typical panels:** 5 V or 6 V monocrystalline solar panels. The buck/boost charger can also charge higher battery voltages from lower panel voltages (e.g., 5 V panel → LTO 2S at 5.4 V).

**Not suitable:** 24 V panels or series connections whose Voc can exceed 25 V. See [DATASHEET.md — Specifications](DATASHEET.md#specifications) for the full electrical limits.

**Sizing (Central Europe):**
- **1 W monocrystalline** is the minimum requirement — only with south-facing, vertical mounting, unshaded, and battery capacity ≥ 7 Ah.
- **From 2 W**, reliable year-round operation is possible.

---

### 9. The red LED (BQ status LED) blinks slowly and the battery is not charging.

Slow blinking of the BQ status LED indicates a **charger fault**. Most common causes:

1. **No NTC connected (most frequent):** Neither an external NTC on the TS pin nor the solder bridge for the onboard NTC is closed. The BQ25798 interprets the open TS pin as a frost condition and blocks charging. → **Solution:** Close the solder bridge or connect a compatible NTC (10 kΩ @ 25 °C, Beta ~3380) between TS (Pin 3) and GND (Pin 2). If no NTC can be fitted, [FAQ #6](#6-what-is-frost-charging-and-how-do-fmax-and-jeitaignore-work-together) describes `set board.jeitaignore` as an alternative and what it costs.

2. **Actually too cold / too warm:** Below –2 °C (T-Cold threshold with Inhero voltage divider), charging is completely blocked by JEITA for Li-ion and LiFePO4. Above ~58 °C (T-Hot threshold), charging is also suspended. → This does not occur with LTO and Na-ion, which run with the JEITA override permanently on, nor on a board where `set board.jeitaignore 1` is active — that override removes both the cold block and the hot-side suspend (see [FAQ #6](#6-what-is-frost-charging-and-how-do-fmax-and-jeitaignore-work-together)).

3. **Other charger fault:** The BQ25798 can also signal faults such as VBAT overvoltage (VBAT_OVP), input overvoltage (VBUS_OVP), or watchdog timeout. These are less common in normal operation.

→ Check with [`get board.telem`](CLI_CHEAT_SHEET.md#getters-query-status) for the current temperature and [`get board.cinfo`](CLI_CHEAT_SHEET.md#getters-query-status) for the charger status; fault flags are shown by [`get board.bqdiag`](CLI_CHEAT_SHEET.md#getters-query-status).

---

### 10. Why doesn’t the board charge without flashed firmware?

This is a deliberate safety feature. The BQ25798 charger is controlled via the **CE pin (Charge Enable)**, which requires the firmware to actively drive GPIO4 HIGH.

**Without firmware** (or with the 3.3V off switch engaged):
- External pull-down on the CE FET gate → FET OFF → CE HIGH → **charging disabled**

This ensures the battery cannot be overcharged if the firmware locks up or is not installed. Flash the firmware via USB and configure the battery chemistry (`set board.bat …`) to enable charging. See [POWER_MANAGEMENT.md — CE Pin Safety](POWER_MANAGEMENT.md#10-bq25798-ce-pin-safety-rev-11--fet-inverted) for the hardware design.

---

**📊 SOC & Monitoring**

### 11. Why does the SOC show 0% or N/A?

**SOC shows N/A** until the battery has been **fully charged at the board for the first time**. The coulomb counter needs a known reference point (100% = "Charge Done" event) to calculate SOC accurately. Charge the battery completely once via USB after commissioning. See [POWER_MANAGEMENT.md — Coulomb Counter & SOC](POWER_MANAGEMENT.md#2-coulomb-counter--soc-state-of-charge) for the tracking mechanism.

**SOC shows 0%** after the board wakes from **low-voltage sleep**. This is intentional: the coulomb counter was not running during sleep, so the charge state is unknown. SOC restarts at 0% and begins accumulating again. When the battery next reaches "Charge Done", the SOC synchronizes cleanly to 100%.

**Note:** In cold conditions, the extractable capacity is lower than stored charge. `get board.telem` shows this as `SOC:95.0% (79%)`. The Batt-TTL accounts for this automatically. See [FAQ #13](FAQ.md#13-how-does-temperature-derating-work).

---

### 12. When should I run `set board.tccal`?

**Ideally in the early morning**, before sunrise. At that time the battery temperature has equalized with the ambient temperature overnight, and no solar radiation has warmed the enclosure yet. This gives the BME280 and the NTC the most consistent baseline for calibration.

**Why timing matters:** During the day, solar radiation heats the enclosure unevenly — the NTC (close to the battery) and the BME280 (on the PCB) may report different temperatures, resulting in an inaccurate offset. In the early morning, both sensors are at thermal equilibrium.

**Why TCCal exists:** An NTC and its associated voltage divider resistors are subject to component tolerances that produce measurement errors significantly larger than those of the BME280. Since the BME280 is on the board, it can serve as a reference to calibrate the NTC reading.

**Important limitations:**
- **Affects telemetry and CLI only.** TCCal corrects the battery temperature displayed via `get board.telem` and transmitted over telemetry. The BQ25798 JEITA thresholds are **not** affected — the charger evaluates the TS pin directly in hardware. Therefore, the actual JEITA switching temperatures may differ slightly from the calibrated CLI readout.
- **Single-point calibration.** The offset is determined at one temperature. Away from the calibration temperature the correction drifts, because NTC non-linearity and divider errors are temperature-dependent.

**Command:** `set board.tccal` — auto-calibrates the NTC offset using the BME280 as reference. Use `set board.tccal reset` to reset the offset to 0.00. See [`get board.tccal`](CLI_CHEAT_SHEET.md#getter-quick-reference) to verify the current offset.

**Note:** The calibration reads the raw NTC value directly and needs 3 of 5 valid samples. It is therefore independent of the BME280 plausibility check that filters the displayed battery temperature (see [FAQ #2](#2-can-i-use-battery-packs-without-a-built-in-ntc)) and still runs on a board whose readout is currently being discarded.

---

### 13. How does temperature derating work?

SOC% is **purely Coulomb-based** — it reflects the actual stored charge and does not change with temperature. Only real charge flow (measured by the INA228 coulomb counter) changes SOC%.

However, the **extractable capacity** decreases at cold temperatures due to slower electrochemical kinetics and increased internal resistance during TX peaks (~100 mA). The firmware calculates a per-chemistry derating factor `f(T)` that is used for:
- **Batt-TTL calculation** — Trapped Charge model: extractable = max(0, remaining − capacity × (1−f(T)))
- **CLI display** — `get board.telem` shows the derated value in parentheses: `SOC:95.0% (78%)` = stored (extractable)

The derating factor is visible in `get board.socdebug` (field `d=`).

→ **Full details:** [POWER_MANAGEMENT.md — Temperature Derating](POWER_MANAGEMENT.md#5-batt-ttl-prediction)

---

### 14. What is Batt-TTL?

> **Batt-TTL** is short for *battery time-to-live* — the estimated remaining runtime on battery. It is not the packet hop limit that "TTL" denotes in mesh networking.

Batt-TTL is an estimated **remaining runtime** based on the current energy balance. It is shown in [`get board.stats`](CLI_CHEAT_SHEET.md#getters-query-status). See [POWER_MANAGEMENT.md — Batt-TTL Prediction](POWER_MANAGEMENT.md#5-batt-ttl-prediction) for the algorithm.

**How it works:**
- A 168-hour ring buffer (7 days) records hourly charge/discharge data from the INA228 coulomb counter.
- **Formula:** `Batt-TTL = extractable capacity / |7-day avg. daily net consumption| × 24h` — where extractable = SOC%-based remaining charge minus the temperature-locked share (see Cold weather below; identical to `SOC% × capacity / 100` at moderate temperatures)
- **Display format:** `BT:12d0h` (12 days, 0 hours) or `BT:12h` (< 24 hours)

**Batt-TTL shows N/A or 0 when:**
- Less than 24 hours of data have been collected
- The board is running on solar surplus (no deficit)

**Note:** If `set board.batcap` is not set, a rough chemistry default (1500–2000 mAh) is used — set the real capacity for a meaningful Batt-TTL.

**Cold weather:** Batt-TTL uses the Trapped Charge model — cold temperatures lock the bottom of the discharge curve, so extractable capacity drops faster than SOC% at low charge levels. This is especially critical in winter: at 20% SOC, the extractable capacity may already be near zero. See [FAQ #13](FAQ.md#13-how-does-temperature-derating-work).

---

**🔩 Hardware**

### 15. Does the board have reverse polarity protection?

**No.** The board has **no hardware reverse polarity protection** — neither on the battery nor on the solar input. A reverse-connected battery or solar panel can cause **immediate, irreversible damage** to the board.

**Always verify polarity before plugging in any cable.** See [DATASHEET.md — Safety & Protection Features](DATASHEET.md#safety--protection-features).

---

### 16. What does the "3.3V off" switch do, and when would I use it?

The slide switch labeled **"3.3V off"** on the bottom-left of the PCB controls the EN pin of the TPS62840 buck converter (see [DATASHEET.md — Connectors, Buttons & LEDs](DATASHEET.md#connectors-buttons--leds--front-side)).

> **⚠ Caution — Inverted logic:**
> - Switch position **"ON"** = EN pin low = board **powered off**
> - Switch position **"OFF"** = EN pin high = board **running**

With the 3.3V rail disabled, the nRF52840, the RF frontend, and all 3.3V-powered components (INA228, RV-3028 RTC, BME280) are completely de-energized. Only the BQ25798 charger IC remains powered from VBAT (~15 µA quiescent current). **Charging is disabled** in this state — the firmware must be running to supervise the charger. Note: the RTC loses its time when the 3.3V rail is off — see [FAQ #23](#23-why-does-the-repeater-board-need-a-correct-time).

**Use cases:**
- **Antenna swap:** Safely power down the RF frontend on a deployed board without disconnecting battery or solar.
- **Transport:** Switch off the board during shipping or relocation.
- **Short/medium-term storage:** ~15 µA total consumption. For longer storage (months), disconnect the battery entirely.

---

### 17. What do the LEDs mean?

The board has three LEDs:

| LED | Location | Color | Meaning |
|-----|----------|-------|--------|
| **LED1** | Right side, top | Blue | Heartbeat (periodic blink during normal operation). Short flash during boot for each successfully initialized component (INA228, BQ25798, RTC). |
| **LED2** | Right side, bottom | Red | Hardware error indicator. Blinks permanently if a critical component (BQ25798, INA228, or RTC) was not found during initialization. |
| **Charge LED** | Bottom right, next to solar connector | Red | BQ25798 charge status output (hardware-controlled). Solid on = charging active. Off = not charging or charging done. Slow blinking = charger fault (see FAQ #9). |

All three LEDs can be disabled with [`set board.leds off`](CLI_CHEAT_SHEET.md#setters-change-configuration).

**Note:** The descriptions for LED1/LED2 apply only after the firmware has booted. The bootloader uses its own LED patterns (e.g., slow blue pulsing during OTA/UF2 updates).

---

### 18. Can I operate the board without an antenna?

**No.** Operating without an antenna risks **irreversible damage** to the RF frontend (SX1262 radio). Always connect both antennas (LoRa and BLE) before powering on.

If you need to swap or install antennas on an already deployed board, use the **3.3V off switch** (see FAQ #16) to safely de-energize the RF frontend without disconnecting battery or solar.

---

### 19. Why does the RTC have no backup battery?

The RV-3028-C7 RTC has two main functions:
1. **Stable time base** with minimal drift for MeshCore.
2. **Wake-up timer** for low-voltage sleep (hourly wake-up for voltage check).

As long as a battery is connected, the RTC is continuously powered — including during System Sleep. After a low-voltage sleep and reboot, the time is preserved.

A backup battery (e.g., CR2032) was intentionally omitted. Its only additional benefit would be to preserve the time when the battery is disconnected. This does not justify the space required on the compact 45 × 40 mm form factor. See [FAQ #23](#23-why-does-the-repeater-board-need-a-correct-time) for why a correct clock matters.

---

### 20. What are the dimensions of the mounting holes?

The board has **4× M2.5 mounting holes** with a diameter of **2.5 mm** and a hole spacing of **35 × 40 mm**. The PCB itself measures 45 × 40 mm.

---

### 21. Are interfaces (UART/I2C) exposed on the board?

**Yes.** Two rows of castellated pads are available on the back side of the PCB:

**Row 1 — UART / I2C:**

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | GND | Ground |
| 2 | RX | UART Receive |
| 3 | TX | UART Transmit |
| 4 | SDA | I2C Data |
| 5 | SCL | I2C Clock |
| 6 | 3.3V | 3.3 V output (max. 500 mA, shared with board consumption) |

**Row 2 — SWD (Debug):**

| Pin | Signal | Description |
|-----|--------|-------------|
| 1 | RESET | nRF52840 Reset |
| 2 | GND | Ground |
| 3 | SWCLK | SWD Clock |
| 4 | SWDIO | SWD Data |
| 5 | 3.3V | 3.3 V output (max. 500 mA, shared with board consumption) |

The castellated pads can be soldered directly to a carrier board. See [DATASHEET.md — Headers & Pads](DATASHEET.md#headers--pads--back-side) for the complete pad layout.

---

**⚙️ Firmware**

### 22. Are my settings preserved during a firmware update?

**Yes.** All board-specific settings are stored on the **LittleFS filesystem**, which is preserved during firmware updates. This includes:

- Battery chemistry (`set board.bat`)
- Battery capacity (`set board.batcap`)
- Charge current (`set board.imax`)
- Frost protection (`set board.fmax`)
- JEITA override (`set board.jeitaignore`)
- MPPT, LED settings
- NTC calibration offset

**JEITA override:** `board.jeitaignore` stores the setting itself; the resulting state is derived. On every boot the charger starts with its temperature guard active, and the firmware derives the override from the stored setting and the 0.05C gate once the configuration is applied. An override whose gate no longer passes stays off, and the setting is kept.

**Note:** Energy statistics (168h ring buffers for Batt-TTL) are held in RAM only and restart after any reboot or update.

Settings are only lost on a full flash erase or filesystem corruption (rare). See [POWER_MANAGEMENT.md — Statistics Persistence](POWER_MANAGEMENT.md#11-statistics-persistence) for technical details.

### 23. Why does the repeater board need a correct time?

The firmware uses the RTC (Real-Time Clock) for several protection mechanisms. An incorrect clock does not cause a total outage — packets are still forwarded — but noticeable problems arise:

- **Adverts are rejected:** Every advert is cryptographically signed (Ed25519 over public key + timestamp + app data). Receivers compare the contained timestamp against the last stored value and discard timestamps that are equal or smaller as potential replay attacks. The existing contact entry on other nodes is preserved but no longer updated — name, position and "last seen" become increasingly stale.
- **Debug logs with incorrect timestamps:** `getLogDateTime()` shows wrong absolute times. Relative calculations like "last heard X seconds ago" remain correct as they use the same (wrong) clock source internally.

Login, admin commands and the rate limiter are **not affected** — login/commands compare client-provided timestamps against each other only, and the rate limiter uses only relative time differences which remain correct as long as the clock ticks monotonically. A `clock sync` can therefore be run at any time after logging in.

**How is the clock set?**
The Inhero MR2 has a hardware RTC that retains its time during a normal reboot. However, if the battery is disconnected or the 3.3V rail is switched off via the onboard switch, the RV-3028 loses its time and falls back to the POR default (January 2000). The clock can be set via CLI (`clock sync` or `time <epoch>`) from an admin client. The repeater is **not** automatically synchronized by clients.

> **Recommendation:** After every battery swap or power-down of the board's 3.3V rail via the onboard switch, run `clock sync` via CLI as soon as possible. A normal reboot is not affected.

---

### 24. Why can't the repeater clock be set backwards?

`clock sync` and `time <epoch>` only allow setting the clock **forward** — setting it backwards is rejected with `ERR: clock cannot go backwards`.

**Why?** The firmware uses increasing timestamps to protect against replay attacks. Both adverts and admin commands are rejected if their timestamp is equal to or lower than the last stored value. Since other nodes in the mesh store the last (high) timestamp, a clock rollback would cause new adverts to be rejected mesh-wide.

**Solution: `clkreboot`** — resets the clock to a low value and reboots the board; the reboot resets the per-client replay timestamps (stored client entries are kept). Run `clock sync` afterwards to set the correct time.

> **Note:** After `clkreboot`, adverts will temporarily be rejected by nodes that still have the old timestamp stored. Visibility normalises once those entries expire.

See also [FAQ #23](#23-why-does-the-repeater-board-need-a-correct-time).

---

## See Also

- [README.md](README.md) — Overview, feature matrix and diagnostics
- [DATASHEET.md](DATASHEET.md) — Hardware datasheet, pinouts and specifications
- [QUICK_START.md](QUICK_START.md) — Quick start for commissioning and CLI setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — All board-specific CLI commands at a glance
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Complete technical documentation
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Battery chemistry comparison and deployment guide
