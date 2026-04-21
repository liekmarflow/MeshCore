# Inhero MR2 Quick-Start

> 🇩🇪 [Deutsche Version](de/QUICK_START.md)

This guide walks you through commissioning and the most important CLI commands.

## 1) Prepare Temperature Sensor (TS/NTC)
- Either use the 3-pin battery connector with TS/NTC, or close the onboard NTC solder bridge on the back side.
- Firmware NTC type: NCP15XH103F03RC (10k @ 25C, Beta 3380).
- Purpose: The charger uses the TS pin for JEITA/frost logic.

## 2) Connect Antennas
- Never operate without an antenna — risk of damage to the RF frontend.

## 3) Connect Battery
- A charge level >90% is recommended so the SOC calculation starts reliably.

> **⚠ WARNING — No Reverse Polarity Protection:** The board has no hardware reverse polarity protection. Connecting the battery with reversed polarity will cause immediate, irreversible damage. Always verify correct polarity before plugging in.

## 4) Configure Repeater via USB
- Connect the repeater to a computer via USB cable.
- Go to https://flasher.meshcore.co.uk/ -> Repeater Setup to configure (LoRa settings, name, admin password, etc.).
- This sets the basic parameters on the device.

## 5) Open CLI
- https://flasher.meshcore.co.uk/ -> Console
- or MeshCore App -> Manage -> Command-Line
- Board-specific commands are set here.

## 6) Set Battery Chemistry
- Command:
  - set board.bat liion1s
  - or set board.bat lifepo1s
  - or set board.bat lto2s
  - or set board.bat naion1s
- Defines charge parameters and low-voltage thresholds.

## 7) Set Battery Capacity
- Command: set board.batcap <mAh>
- Example: set board.batcap 10000
- Important for accurate SOC calculation.

## 8) Set Maximum Charge Current
- Command: set board.imax <mA>
- Firmware range: 50 to 1500 mA (BQ25798 minimum: 50mA).
- Choose to match your solar setup so currents fit the PG check.
- Rule of thumb: panel power / panel voltage * 1.2

## 9) Set Frost Charge Current Reduction
- Command: set board.fmax <0%|20%|40%|100%>
- Limits the maximum charge current in the T-Cool range (0°C to -5°C) to X% of board.imax.
- 0% = Charging blocked in T-Cool range.
- 20% = max. 20% of imax (e.g. 500mA → 100mA at 0°C to -5°C).
- 40% = max. 40% of imax (e.g. 500mA → 200mA at 0°C to -5°C).
- 100% = no reduction, full charge current even in cold conditions.
- Below -5°C (T-Cold): Charging always completely blocked by JEITA.
- Important: Only charging is restricted. With sufficient solar, the board continues to run on solar power — the battery is neither charged nor discharged.
- Note: For LTO and Na-Ion, JEITA is disabled (fmax has no effect, charges even in frost).

## 10) Enable MPPT
- Command: set board.mppt <0|1>
- 1 = MPPT on, 0 = MPPT off.
- Typically enable for solar input.

## 11) Enable/Disable LEDs
- Command: set board.leds <on|off> or set board.leds <1|0>
- Controls heartbeat LED and BQ status LED (boot LEDs remain active).

## 12) Fully Charge Battery (SOC Sync)
- Fully charge the battery once via USB so the SOC synchronizes cleanly.

## Additional Notes (Practical)
- After setting the battery chemistry, a quick check with `get board.bat` confirms the setting was saved.
- For solar operation, `set board.mppt 1` is recommended; for USB-only operation, MPPT can stay off.

## Example Values per Battery Chemistry (Starting Point)
These values are safe starting points and should be adjusted to match battery, panel, and usage profile.

### Li-Ion 1S (3.7V nominal)
```bash
set board.bat liion1s
set board.imax 500
set board.fmax 20%
```

### LiFePO4 1S (3.2V nominal)
```bash
set board.bat lifepo1s
set board.imax 300
set board.fmax 40%
```

### LTO 2S (2x 2.3V nominal)
```bash
set board.bat lto2s
set board.imax 700
set board.fmax 0%
```

### Na-Ion 1S (3.1V nominal)
```bash
set board.bat naion1s
set board.imax 500
```

Note: `set board.fmax` has no effect on LTO and Na-Ion (JEITA disabled).

## Solar Panel Notes
- Maximum open-circuit voltage (Voc) for the input: 25V.
- Typical panels are 5V or 6V (MPP below that).
- The board has buck/boost and can charge higher battery voltages from lower panel voltages.
- 24V panels or series connections may exceed the 25V Voc limit and are not suitable.
- Wattage class: at least 1W, typically 2W.
- For 1W panels, a battery capacity of >7Ah is recommended.
- This applies only with south-facing, vertical mounting, and an unshaded location.
- In worse solar conditions, either use 2W or increase battery capacity for "winter survival".

## USB Charging
- The board can also be charged via USB-C (5V).
- USB-C VBUS is routed to the BQ25798 VBUS input via an **SS34 Schottky diode** — the same single input as the solar panel. The BQ25798 has only one VBUS input and does not distinguish between USB and solar.
- The SS34 diode prevents backflow from the solar panel to the USB bus. However, current **can** flow from USB-VBUS out through the solar connector.
- CC1/CC2 are pulled to GND via 4.7kΩ (USB sink, 5V default).
- **⚠ Warning:** Since VBUS-USB and VBUS-BQ share the same bus (via SS34 diode), a **short circuit on the solar connector will also short VBUS-USB**. Never short-circuit the solar input while USB is connected.

## Voltage Thresholds per Battery Chemistry
Thresholds are optimized for maximum lifespan and stable operation.

| Battery Chemistry | lowv_sleep_mv (System Sleep) | lowv_wake_mv (0% SOC) | Hysteresis |
|---|---|---|---|
| Li-Ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |

## Low-Voltage Behavior
- **Low-Voltage System Sleep:** When VBAT drops below `lowv_sleep_mv`, the INA228 ALERT interrupt fires (P1.02). The firmware latches CE HIGH (`digitalWrite(BQ_CE_PIN, HIGH)` → FET ON → CE LOW → charging active), configures the RTC wake timer, and enters System Sleep with GPIO latch (< 500µA). P0.04 is excluded from `disconnectLeakyPullups()` so the GPIO latch stays HIGH. Periodic RTC wakes (hourly) check voltage — only when recovery above `lowv_wake_mv` does it boot normally.
- **Solar Recovery:** In System Sleep, GPIO4 latch is preserved HIGH → DMN2004TK-7 FET ON → CE LOW → charging active. Solar charging continues autonomously until the battery charges above `lowv_wake_mv`. Without GPIO latch (RAK unpowered): ext. pull-down on gate → FET OFF → CE HIGH → charging OFF (safety default).

## CLI Examples (Compact)
```bash
# Battery chemistry and capacity
set board.bat liion1s
set board.batcap 10000

# Charge parameters
set board.imax 500
set board.fmax 20%
set board.mppt 1

# LEDs
set board.leds off

# Status checks
get board.bat
get board.imax
get board.fmax
get board.mppt
get board.leds
get board.batcap
get board.telem
get board.stats
get board.cinfo
get board.selftest
get board.conf
```

## Getter Quick Reference (all relevant board getters)
- `get board.bat` - Current battery type (liion1s, lifepo1s, lto2s, none).
- `get board.fmax` - Current frost charge behavior (0%/20%/40%/100%).
- `get board.imax` - Maximum charge current in mA.
- `get board.mppt` - MPPT status (0/1).
- `get board.leds` - LED status (Heartbeat + BQ Stat).
- `get board.batcap` - Battery capacity in mAh (set/default).
- `get board.telem` - Real-time telemetry (Battery/Solar incl. SOC, V/I/T). See [TELEMETRY.md](TELEMETRY.md) for what the app displays.
- `get board.stats` - Energy balance (24h/3d/7d), charge/discharge breakdown and MPPT ratio.
- `get board.cinfo` - Charger status (Charger State + Flags).
- `get board.selftest` - I²C hardware probe (`INA:OK BQ:OK RTC:OK BME:OK`). RTC includes a write/readback verify (state `WR_FAIL` on mismatch).
- `get board.conf` - Summary of all configs (B, F, M, I, Vco, V0).
- `get board.tccal` - NTC temperature calibration offset in °C (0.00 = default).

---

## See Also

- [README.md](README.md) — Overview, feature matrix and diagnostics
- [TELEMETRY.md](TELEMETRY.md) — Telemetry channels explained (what the app displays)
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — All CLI commands at a glance
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Complete technical documentation
