# Inhero MR2 – CLI Cheat-Sheet

> 🇩🇪 [Deutsche Version](de/CLI_CHEAT_SHEET.md)

All board-specific CLI commands at a glance.
Prefix is always `board.` — i.e. `get board.<cmd>` or `set board.<cmd> <value>`.

---

## Setters (Change Configuration)

```bash
# Battery chemistry
set board.bat liion1s          # Li-Ion 1S (3.7V nominal)
set board.bat lifepo1s         # LiFePO4 1S (3.2V nominal)
set board.bat lto2s            # LTO 2S (2x 2.3V nominal)
set board.bat naion1s          # Na-Ion 1S (3.1V nominal)
set board.bat none             # No battery / unknown (charging disabled)

# Battery capacity (100–100000 mAh)
set board.batcap 10000

# Maximum charge current (50–1500 mA)
set board.imax 500

# Frost charge current reduction (T-Cool 0°C to -5°C)
set board.fmax 0%              # Charging blocked
set board.fmax 20%             # max. 20% of imax
set board.fmax 40%             # max. 40% of imax
set board.fmax 100%            # no reduction
# Note: No effect on LTO / Na-Ion (JEITA disabled)

# MPPT on/off
set board.mppt 1               # Enable MPPT
set board.mppt 0               # Disable MPPT

# LEDs on/off (Heartbeat + BQ Stat)
set board.leds on              # Enable LEDs  (on/1)
set board.leds off             # Disable LEDs (off/0)

# Manually set SOC (0–100%)
set board.soc 85.0
```

### Calibration

```bash
# NTC temperature calibration
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
get board.fmax                 # Frost charge behavior (0%/20%/40%/100% or N/A)
get board.mppt                 # MPPT status (0/1)
get board.leds                 # LED status (ON/OFF)
get board.conf                 # Summary of all configs (B, F, M, I, Vco, V0)

# Real-time telemetry
get board.telem                # Battery+Solar: V, I, T, SOC

# Energy & Statistics
get board.stats                # Energy balance (24h/3d/7d), C/D, MPPT%, TTL
                               #   TTL = Time To Live (hours until battery empty)
                               #   Basis: 7-day average of daily net deficit
                               #   from hourly INA228 coulomb counter samples (168h ring buffer)
                               #   Formula: (SOC% × capacity) / |7d-avg-deficit| × 24
                               #   TTL only shown in BAT mode (net deficit)
                               #   Prerequisite: min. 24h data + capacity known

# Charger & Diagnostics
get board.cinfo                # Charger status + last PG-stuck HIZ toggle

# Calibration
get board.tccal                # NTC temperature offset in °C (0.00 = default)
```

---

## Getter Quick Reference

| Command | Description |
|---|---|
| `get board.bat` | Battery type (`liion1s`, `lifepo1s`, `lto2s`, `naion1s`, `none`) |
| `get board.batcap` | Battery capacity in mAh (set/default) |
| `get board.imax` | Maximum charge current in mA |
| `get board.fmax` | Frost charge behavior (`0%`/`20%`/`40%`/`100%`, LTO/Na-Ion: `N/A`) |
| `get board.mppt` | MPPT status (`0`/`1`) |
| `get board.leds` | LED status Heartbeat + BQ Stat (`ON`/`OFF`) |
| `get board.conf` | Summary: B(at) F(max) M(ppt) I(max) Vco V0 |
| `get board.telem` | Real-time telemetry: Battery/Solar V, I, T, SOC — see [TELEMETRY.md](TELEMETRY.md) |
| `get board.stats` | Energy balance (24h/3d/7d), C/D, MPPT%, TTL (7d-avg-based) |
| `get board.cinfo` | Charger status + PG-stuck HIZ toggle (e.g. "PG / CC HIZ:3m ago") |
| `get board.tccal` | NTC temperature offset in °C (`0.00` = default) |

---

## Setter Quick Reference

| Command | Range | Description |
|---|---|---|
| `set board.bat` | `liion1s` · `lifepo1s` · `lto2s` · `naion1s` · `none` | Set battery chemistry |
| `set board.batcap` | `100`–`100000` (mAh) | Set battery capacity |
| `set board.imax` | `50`–`1500` (mA) | Set max charge current |
| `set board.fmax` | `0%` · `20%` · `40%` · `100%` | Frost charge reduction (not for LTO/Na-Ion) |
| `set board.mppt` | `0`/`1` · `true`/`false` | Enable/disable MPPT |
| `set board.leds` | `on`/`off` · `1`/`0` | Enable/disable LEDs |
| `set board.soc` | `0`–`100` (%) | Manually set SOC |
| `set board.tccal` | `reset` · *(empty = auto)* | Calibrate or reset NTC temperature |

---

## Quick-Start Recipes

### Li-Ion 1S with 10Ah and Solar
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

### Na-Ion 1S with 10Ah and Solar
```bash
set board.bat naion1s
set board.batcap 10000
set board.imax 500
set board.mppt 1
set board.leds off
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
- [TELEMETRY.md](TELEMETRY.md) — Telemetry channels explained (what the app displays)
- [QUICK_START.md](QUICK_START.md) — Quick start for commissioning and CLI setup
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Complete technical documentation
