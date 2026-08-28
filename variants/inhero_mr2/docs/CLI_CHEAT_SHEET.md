# Inhero MR2 – CLI Cheat-Sheet

> 🇩🇪 [Deutsche Version](de/CLI_CHEAT_SHEET.md)

All board-specific CLI commands at a glance.
Prefix is always `board.` — i.e. `get board.<cmd>` or `set board.<cmd> <value>`.

See [FAQ.md](FAQ.md) for explanations of key parameters (`imax`, `fmax`, `batcap`) and [DATASHEET.md](DATASHEET.md#supported-battery-chemistries) for chemistry details.

---

## Setters (Change Configuration)

```bash
# Battery chemistry
set board.bat liion1s          # Li-ion 1S (3.7V nominal)
set board.bat lifepo1s         # LiFePO4 1S (3.2V nominal)
set board.bat lto2s            # LTO 2S (2x 2.3V nominal)
set board.bat naion1s          # Na-ion 1S (3.1V nominal)
set board.bat none             # No battery / unknown (charging disabled)

# Battery capacity (100–100000 mAh)
# Rule of thumb: 90% of nominal capacity (see FAQ #4)
set board.batcap 10000

# Maximum charge current (50–1500 mA)
set board.imax 500

# Frost charge current reduction (T-Cool approx. -2 °C to +3 °C, see JEITA table in README)
set board.fmax 0%              # Charging blocked
set board.fmax 20%             # max. 20% of imax
set board.fmax 40%             # max. 40% of imax
set board.fmax 100%            # no reduction
# Refused on LTO / Na-ion:  "Err: Fmax setting N/A for this chemistry (JEITA disabled)"
# Refused while the JEITA override is on:  "Err: Fmax N/A while jeitaignore is on"
# Refused before a chemistry is set:  "Err: Set board.bat first"

# JEITA override — keep charging below the T-Cold threshold (approx. -2 °C)
# Li-ion / LiFePO4 only; off by default; at the operator's own risk (see section below)
set board.jeitaignore 1        # Ignore the TS pin (1/true)
set board.jeitaignore 0        # Back to hardware JEITA (0/false)

# MPPT on/off
set board.mppt 1               # Enable MPPT
set board.mppt 0               # Disable MPPT

# LEDs on/off (Heartbeat + BQ Stat)
set board.leds on              # Enable LEDs  (on/1)
set board.leds off             # Disable LEDs (off/0)

# Manually set SOC (0–100%)
set board.soc 85.0

# Unknown setter:  "Err: bat|imax|fmax|mppt|batcap|tccal|leds|soc|jeitaignore"
```

### Calibration

```bash
# NTC temperature calibration
# Best practice: run in the early morning before sunrise,
# when battery temperature has equalized with ambient (see FAQ #12).
set board.tccal                # Auto-calibration via BME280
set board.tccal reset          # Reset offset to 0.00
```

---

## Getters (Query Status)

```bash
# Configuration & Hardware
get board.bat                  # Current battery type
get board.batcap               # Battery capacity in mAh (set/default)
get board.imax                 # Maximum charge current in mA
get board.fmax                 # Frost charge behavior (0%/20%/40%/100%, or N/A while
                               #   the JEITA override is active)
get board.jeitaignore          # JEITA override state (see section below)
get board.mppt                 # MPPT status (0/1)
get board.leds                 # LED status (ON/OFF)
get board.conf                 # Summary of all configs (B, F, M, I, Vco, V0)
                               #   plus " J:1" while the JEITA override is active
                               #   on a chemistry that uses JEITA

# Real-time telemetry
get board.telem                # Battery+Solar: V, I, T, SOC

# Energy & Statistics
get board.stats                # Energy balance (24h/3d/7d), C/D, MPPT%, Batt-TTL
                               #   Batt-TTL = battery time-to-live (hours until battery empty),
                               #   not a packet hop limit
                               #   Basis: 7-day average of daily net deficit
                               #   from hourly INA228 coulomb counter samples (168h ring buffer)
                               #   Formula: extractable / |7d-avg-deficit| × 24, where
#   extractable = SOC% × capacity − trapped charge
#   (cold-temperature derating; 0 at normal temperatures)
                               #   Batt-TTL only shown in BAT mode (net deficit)
                               #   Prerequisite: min. 24h data + capacity known

# Charger & Diagnostics
get board.cinfo                # Charger status + last PG-stuck HIZ toggle
get board.bqdiag               # Diagnostic/debug: compact BQ25798 register dump
                               #   PG/charge state, TS region (COLD/COOL/WARM/HOT),
                               #   active status/fault flags (e.g. VINDPM, VBAT_OVP)
get board.selftest             # Probe all I2C devices (INA228/BQ25798/RV-3028/BME280)
                               #   Output: "INA:OK BQ:OK RTC:OK BME:OK"
                               #   RTC includes user-RAM write/readback verify
                               #   to catch cold-solder joints (chip ACKs but
                               #   rejects writes). Possible per-device states:
                               #     OK      — device responds and (RTC) persists writes
                               #     NACK    — device does not ACK on I2C bus
                               #     WR_FAIL — (RTC only) ACKs but write/readback mismatched
get board.socdebug             # Diagnostic/debug: SOC tracking internals
                               #   SHUNT_CAL, precise current, CHARGE register (mAh),
                               #   current-hour charge/discharge accumulators,
                               #   update count, RTC time, temperature derating factor

# Calibration
get board.tccal                # NTC temperature offset in °C (0.00 = default)

# Unknown getter:
#   "Err: bat|fmax|imax|mppt|telem|stats|cinfo|conf|tccal|leds|batcap|jeitaignore"
#   bqdiag, selftest and socdebug work but are not part of that list
```

---

## Getter Quick Reference

| Command | Description |
|---|---|
| `get board.bat` | Battery type (`liion1s`, `lifepo1s`, `lto2s`, `naion1s`, `none`) |
| `get board.batcap` | Battery capacity in mAh (set/default) |
| `get board.imax` | Maximum charge current in mA |
| `get board.fmax` | Frost charge behavior (`0%`/`20%`/`40%`/`100%`; `N/A` while the JEITA override is active) |
| `get board.jeitaignore` | JEITA override state — `jeitaignore 1`, `jeitaignore 0`, `jeitaignore 1 (chemistry)`, a blocked variant, or `N/A` before a chemistry is set |
| `get board.mppt` | MPPT status (`0`/`1`) |
| `get board.leds` | LED status Heartbeat + BQ Stat (`ON`/`OFF`) |
| `get board.conf` | Summary: B(at) F(max) M(ppt) I(max) Vco V0, plus `J:1` on Li-ion / LiFePO4 while the JEITA override is active; with `none` the whole reply is `B:none (no battery, charging disabled)` |
| `get board.telem` | Real-time telemetry: Battery/Solar V, I, T, SOC — see [TELEMETRY.md](TELEMETRY.md) |
| `get board.stats` | Energy balance (24h/3d/7d), C/D, MPPT%, Batt-TTL (7d-avg-based) |
| `get board.cinfo` | Charger status + PG-stuck HIZ toggle (e.g. "PG / CC HIZ:3m ago") |
| `get board.bqdiag` | Diagnostic/debug: BQ25798 register dump — PG/charge state, TS region, active fault flags |
| `get board.selftest` | I2C device probe — `INA:OK BQ:OK RTC:OK BME:OK` (RTC also write-verified) |
| `get board.socdebug` | Diagnostic/debug: SOC internals — SHUNT_CAL, current, CHARGE, hour accumulators, derating factor |
| `get board.tccal` | NTC temperature offset in °C (`0.00` = default) |

---

## Setter Quick Reference

| Command | Range | Description |
|---|---|---|
| `set board.bat` | `liion1s` · `lifepo1s` · `lto2s` · `naion1s` · `none` | Set battery chemistry — re-derives the JEITA override and resets `fmax` to `0%` on Li-ion / LiFePO4 |
| `set board.batcap` | `100`–`100000` (mAh) | Set battery capacity — also the reference for the `jeitaignore` gate |
| `set board.imax` | `50`–`1500` (mA) | Set max charge current — also the `jeitaignore` gate quantity |
| `set board.fmax` | `0%` · `20%` · `40%` · `100%` | Frost charge reduction (refused on LTO/Na-ion and while `jeitaignore` is on) |
| `set board.jeitaignore` | `1`/`0` · `true`/`false` | JEITA override, Li-ion/LiFePO4 only — gate: `batcap` set and `imax` ≤ 0.05C |
| `set board.mppt` | `0`/`1` · `true`/`false` | Enable/disable MPPT |
| `set board.leds` | `on`/`off` · `1`/`0` | Enable/disable LEDs |
| `set board.soc` | `0`–`100` (%) | Manually set SOC |
| `set board.tccal` | `reset` · *(empty = auto)* | Calibrate or reset NTC temperature |

---

## JEITA Override (`board.jeitaignore`)

`set board.jeitaignore 1` sets the BQ25798 TS_IGNORE bit, so the charger treats the TS pin as
always good and keeps charging below the T-Cold threshold of about -2 °C. Two conditions must
hold for the override to take effect:

- `set board.batcap` has been written explicitly, and
- `imax` is at or below 0.05C of that capacity (300 mA on 6000 mAh, 500 mA on 10000 mAh).

The setting is stored even when the gate fails; lowering `imax` or raising `batcap` re-arms it
on its own, and the reply of those commands says so. LTO 2S and Na-ion 1S run without JEITA
anyway and refuse the command.

TS_IGNORE bypasses all four TS regions, so the hot-side charge suspend at roughly +58 °C is gone
as well and `fmax` has no effect while the override is on. Below freezing, charging Li-ion or
LiFePO4 causes cumulative, permanent lithium plating on the anode. The 0.05C gate bounds the rate;
it does not remove the mechanism, so the override runs at the operator's own risk. The full
treatment — field experience, counter-arguments and sources — is in
[BATTERY_GUIDE.md](BATTERY_GUIDE.md).

```bash
set board.jeitaignore 1
#  "jeitaignore set to 1"                              — gate passes, override active
#  "jeitaignore set to 1, N/A, C>0.05"                 — imax above 0.05C; stored, re-arms later
#  "jeitaignore set to 1, N/A, batcap not set"         — no batcap; stored, re-arms later
#  "Err: This chemistry runs without JEITA (always 1)" — lto2s / naion1s
#  "Err: Set board.bat first"                          — no chemistry set yet
#  "Err: Use 1|0"                                      — argument is not 1|0|true|false
#  "Err: Failed to store setting"                      — the write to flash failed

set board.jeitaignore 0
#  "jeitaignore set to 0"                              — stored fmax behavior applies again

get board.jeitaignore
#  "jeitaignore 1 (chemistry)"                         — lto2s / naion1s
#  "N/A"                                               — no chemistry set yet
#  "jeitaignore 1"                                     — override active
#  "jeitaignore 1, N/A, C>0.05"                        — set, blocked by imax
#  "jeitaignore 1, N/A, batcap not set"                — set, blocked by missing batcap
#  "jeitaignore 0"                                     — not set

# imax and batcap are gate quantities — their reply names a state change:
set board.imax 500
#  "Max charge current set to 500mA"                          — no change
#  "Max charge current set to 500mA; jeitaignore 1"           — override re-armed
#  "Max charge current set to 500mA; jeitaignore N/A, C>0.05" — override lost

set board.batcap 10000
#  "Battery capacity set to 10000 mAh"                          — no change
#  "Battery capacity set to 10000 mAh; jeitaignore 1"           — override re-armed
#  "Battery capacity set to 10000 mAh; jeitaignore N/A, C>0.05" — override lost
```

The effective bit can be read back with `get board.bqdiag`: the reply ends with `N:<hex>`, the
raw NTC_CONTROL_1 register — an odd value means TS_IGNORE is programmed.

---

## Quick-Start Recipes

### Li-ion 1S with 10Ah and Solar
```bash
set board.bat liion1s
set board.batcap 10000
set board.imax 500
set board.fmax 20%
set board.mppt 1
set board.leds off
```

### LiFePO4 1S with 6Ah and Solar
```bash
set board.bat lifepo1s
set board.batcap 6000
set board.imax 300
set board.fmax 40%
set board.mppt 1
set board.leds off
```

### LTO 2S with 18Ah and Solar
```bash
set board.bat lto2s
set board.batcap 18000
set board.imax 700
set board.mppt 1
set board.leds off
```

### Na-ion 1S with 10Ah and Solar
```bash
set board.bat naion1s
set board.batcap 10000
set board.imax 500
set board.mppt 1
set board.leds off
```

### Li-ion 1S with 10Ah, Solar and JEITA Override
```bash
set board.bat liion1s
set board.batcap 10000         # gate reference — set this before jeitaignore
set board.imax 500             # 0.05C of 10000 mAh — at the gate limit
set board.jeitaignore 1        # "jeitaignore set to 1"
set board.mppt 1
set board.leds off
# set board.fmax is refused while the override is on; get board.fmax reads N/A
```

### Status Check (everything at a glance)
```bash
get board.conf
get board.telem
get board.stats
get board.cinfo
```

---

## See Also

- [README.md](README.md) — Overview, feature matrix and diagnostics
- [DATASHEET.md](DATASHEET.md) — Hardware specifications and pinout
- [TELEMETRY.md](TELEMETRY.md) — Telemetry channels explained (what the app displays)
- [QUICK_START.md](QUICK_START.md) — Quick start for commissioning and CLI setup
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Battery chemistry comparison and deployment guide
- [FAQ.md](FAQ.md) — Frequently asked questions
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Complete technical documentation
