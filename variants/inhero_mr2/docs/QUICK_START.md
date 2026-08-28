# Inhero MR2 Quick-Start

> 🇩🇪 [Deutsche Version](de/QUICK_START.md)

This guide walks you through commissioning and the most important CLI commands.

## 1) Prepare Temperature Sensor (TS/NTC)
- Either use the 3-pin battery connector with TS/NTC, or close the onboard NTC solder bridge on the back side.
- Firmware NTC type: NCP15XH103F03RC (10k @ 25C, Beta 3380).
- Purpose: The charger uses the TS pin for JEITA/frost logic.
- Without an NTC the open TS pin reads as frost, and the charger blocks charging for Li-ion and LiFePO4. Fitting one is the correct fix; for an installation that has none, the override in step 10 takes the TS pin out of the decision.
- → [FAQ #2 — Battery packs without NTC](FAQ.md#2-can-i-use-battery-packs-without-a-built-in-ntc)

## 2) Connect Antennas
- Never operate without an antenna — risk of damage to the RF frontend.

## 3) Connect Battery
- A charge level >90% is recommended so the battery can be fully charged via USB and the SOC calculation starts reliably.

> **⚠ WARNING — No Reverse Polarity Protection:** The board has no hardware reverse polarity protection. Connecting the battery with reversed polarity will cause immediate, irreversible damage. Always verify correct polarity before plugging in.

## 4) Flash the Firmware and Configure the Repeater
- Connect the board to a computer via USB cable.
- The board ships with a bootloader only. Firmware for the MR2 is released from the Inhero fork — flasher.meshcore.io does not carry this board yet (upstream PRs #3131 / #3132 are pending).
- Download the UF2 from https://github.com/liekmarflow/MeshCore/releases
- Double-tap the reset button (right side, below USB-C). A mass-storage device named `RAK4630` or `FTHR840` appears.
- Drag the UF2 onto that drive; the board reboots into the firmware.
- The MR2 runs MeshCore's **repeater** role (build `Inhero_MR2_repeater`); a **sensor** build (`Inhero_MR2_sensor`) is also provided.
- Then go to https://flasher.meshcore.io -> Repeater Setup to configure LoRa settings, name and admin password.

## 5) Open CLI
- https://flasher.meshcore.io -> Console
- or MeshCore App -> Manage -> Command-Line
- Board-specific commands are set here.

## 6) Set Battery Chemistry
- Command:
  - set board.bat liion1s
  - or set board.bat lifepo1s
  - or set board.bat lto2s
  - or set board.bat naion1s
- Defines charge parameters and low-voltage thresholds.
- → [FAQ #1](FAQ.md#1-which-battery-chemistry-should-i-choose) | [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Which battery chemistry should I choose?

## 7) Set Battery Capacity
- Command: set board.batcap <mAh>
- Example: set board.batcap 10000
- Important for accurate SOC calculation, and the precondition for the frost charging override in step 10.
- → [FAQ #4 — What mAh value?](FAQ.md#4-what-mah-value-should-i-enter-for-set-boardbatcap)

## 8) Set Maximum Charge Current
- Command: set board.imax <mA>
- Firmware range: 50 to 1500 mA (BQ25798 minimum: 50mA).
- Choose to match your solar setup so currents fit the PG check.
- Rule of thumb: panel power / panel voltage * 1.2
- → [FAQ #5 — Why set imax?](FAQ.md#5-why-is-it-important-to-set-the-maximum-charge-current-with-set-boardimax)

## 9) Set Frost Charge Current Reduction
- Command: set board.fmax <0%|20%|40%|100%>
- Limits the maximum charge current in the T-Cool range (approx. -2 °C to +3 °C, see JEITA table in README) to X% of board.imax.
- 0% = Charging blocked in T-Cool range.
- 20% = max. 20% of imax (e.g. 500mA → 100mA at approx. -2 °C to +3 °C).
- 40% = max. 40% of imax (e.g. 500mA → 200mA at approx. -2 °C to +3 °C).
- 100% = no reduction, full charge current even in cold conditions.
- Below approx. -2 °C (T-Cold): Charging completely blocked by JEITA, unless the override from step 10 is armed.
- Important: Only charging is restricted. With sufficient solar, the board continues to run on solar power — the battery is neither charged nor discharged.
- Note: For LTO and Na-ion, JEITA is disabled (`set board.fmax` is rejected with an error, charging works even in frost).
- → [FAQ #6 — What does fmax control?](FAQ.md#6-what-does-set-boardfmax-control)

## 10) Frost Charging Override (optional)
- Command: set board.jeitaignore <1|0> — default 0.
- Only for Li-ion and LiFePO4. LTO and Na-ion charge in frost anyway and reject the command with `Err: This chemistry runs without JEITA (always 1)`.
- With 1 the charger ignores the TS pin: charging continues below -2 °C, and the charger's upper cut-off at approx. +58 °C is dropped as well.
- Precondition: `board.batcap` must be set (step 7) and `board.imax` must be at or below 0.05C of that capacity (10000 mAh → 500 mA). Otherwise the reply names the blocker — `jeitaignore set to 1, N/A, batcap not set` or `jeitaignore set to 1, N/A, C>0.05`. The setting stays stored either way and takes effect on its own once imax or batcap pass.
- The 0.05C ceiling holds for as long as the override is on, and it can sit well below what the panel delivers: a 4000 mAh pack allows 200 mA; the 2 W panel from step 8 gives about 480 mA.
- Charging Li-ion or LiFePO4 in frost plates metallic lithium on the anode, cumulatively and permanently; it shows up later as lost capacity.
- While the override is on, `get board.fmax` reads N/A and `set board.fmax` is rejected with `Err: Fmax N/A while jeitaignore is on`.
- → [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — cold charging, field evidence and the full trade

## 11) Enable MPPT
- Command: set board.mppt <0|1>
- 1 = MPPT on, 0 = MPPT off.
- Typically enable for solar input.

## 12) Enable/Disable LEDs
- Command: set board.leds <on|off> or set board.leds <1|0>
- Controls heartbeat LED and BQ status LED (bootloader LED patterns are unaffected).
- → [FAQ #17 — What do the LEDs mean?](FAQ.md#17-what-do-the-leds-mean)

## 13) Fully Charge Battery (SOC Sync)
- Fully charge the battery once via USB so the SOC synchronizes cleanly.
- → [FAQ #11 — SOC shows 0% or N/A?](FAQ.md#11-why-does-the-soc-show-0-or-na)

> **Cold weather note:** SOC% is purely Coulomb-based and does not change with temperature. However, `get board.telem` shows both the stored and extractable capacity when it's cold: `SOC:95.0% (78%)`. The firmware uses a Trapped Charge model — at low SOC and cold temperatures, the extractable value drops steeply (the bottom of the discharge curve is "locked"). See [FAQ #13](FAQ.md#13-how-does-temperature-derating-work) for details.

## Additional Notes (Practical)
- After setting the battery chemistry, a quick check with `get board.bat` confirms the setting was saved.
- For solar operation, `set board.mppt 1` is recommended; for USB-only operation, MPPT can stay off.

## Example Values per Battery Chemistry (Starting Point)
These values are safe starting points and should be adjusted to match battery, panel, and usage profile.

The `imax` values below are derived from the rule of thumb from section 8:
**`imax ≈ panel power ÷ panel voltage × 1.2`** (e.g. 2 W ÷ 5 V × 1.2 ≈ 480 mA → round to 500).
`fmax` is given as a percentage of `imax` and only applies in the T-Cool zone (approx. -2 °C to +3 °C, see JEITA table in README).

### Li-ion 1S (3.7V nominal)
```bash
set board.bat liion1s    # chemistry: 1S Li-ion (sets charge profile + low-V thresholds)
set board.batcap 10000   # pack capacity — SOC, and the 0.05C ceiling for step 10 (→ 500 mA)
set board.imax 500       # max charge current — ≈ 2 W panel @ 5 V (2 W ÷ 5 V × 1.2 ≈ 480 mA)
set board.fmax 20%       # T-Cool (approx. -2…+3 °C): cap at 20 % × 500 mA = 100 mA
```

### LiFePO4 1S (3.2V nominal)
```bash
set board.bat lifepo1s   # chemistry: 1S LiFePO4 (sets charge profile + low-V thresholds)
set board.batcap 9000    # pack capacity — SOC, and the 0.05C ceiling for step 10 (→ 450 mA)
set board.imax 300       # max charge current — ≈ 1 W panel @ 5 V (1 W ÷ 5 V × 1.2 ≈ 240 mA, rounded up for headroom)
set board.fmax 40%       # T-Cool (approx. -2…+3 °C): cap at 40 % × 300 mA = 120 mA
```

### LTO 2S (2x 2.3V nominal)
```bash
set board.bat lto2s      # chemistry: 2S LTO (sets charge profile + low-V thresholds)
set board.batcap 10000   # pack capacity — for the SOC calculation
set board.imax 700       # max charge current — ≈ 3 W panel @ 5 V (3 W ÷ 5 V × 1.2 = 720 mA → 700)
                         # fmax is omitted: rejected for LTO (JEITA disabled — LTO charges even at frost)
```

### Na-ion 1S (3.1V nominal)
```bash
set board.bat naion1s    # chemistry: 1S Na-ion (sets charge profile + low-V thresholds)
set board.batcap 10000   # pack capacity — for the SOC calculation
set board.imax 500       # max charge current — ≈ 2 W panel @ 5 V (2 W ÷ 5 V × 1.2 ≈ 480 mA)
                         # fmax is omitted: rejected for Na-ion (JEITA disabled)
```

Note: `set board.fmax` is rejected with an error for LTO and Na-ion (JEITA disabled); `get board.fmax` shows N/A. The same applies to Li-ion and LiFePO4 while the frost charging override from step 10 is on.

## Solar Panel Notes
- Maximum open-circuit voltage (Voc) for the input: 25V.
- Typical panels are 5V or 6V (MPP below that).
- The board has buck/boost and can charge higher battery voltages from lower panel voltages.
- 24V panels or series connections may exceed the 25V Voc limit and are not suitable.
- Wattage class: at least 1W, typically 2W.
- For 1W panels, a battery capacity of >7Ah is recommended.
- This applies only with south-facing, vertical mounting, and an unshaded location.
- In worse solar conditions, either use 2W or increase battery capacity for "winter survival".

→ [FAQ #8 — Which solar panels?](FAQ.md#8-which-solar-panels-can-i-connect)

## USB Charging
- The board can also be charged via USB-C (5V).
- USB-C VBUS is routed to the BQ25798 VBUS input via a **Schottky diode** — the same single input as the solar panel. The BQ25798 has only one VBUS input and does not distinguish between USB and solar.
- The Schottky diode prevents backflow from the solar panel to the USB bus. However, current **can** flow from USB-VBUS out through the solar connector.
- CC1/CC2 are pulled to GND via 4.7kΩ (USB sink, 5V default).
- **⚠ Warning:** Since VBUS-USB and VBUS-BQ share the same bus (via the Schottky diode), a **short circuit on the solar connector will also short VBUS-USB**. Never short-circuit the solar input while USB is connected.

## Voltage Thresholds per Battery Chemistry
Thresholds are chosen for long service life and stable operation.

| Battery Chemistry | lowv_sleep_mv (System Sleep) | lowv_wake_mv (0% SOC) | Hysteresis |
|---|---|---|---|
| Li-ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |
| Na-ion 1S | 2500 | 2700 | 200mV |

## Low-Voltage Behavior
- **Low-Voltage System Sleep:** When VBAT drops below `lowv_sleep_mv`, the INA228 ALERT interrupt fires (P1.02). The firmware latches CE HIGH (`digitalWrite(BQ_CE_PIN, HIGH)` → FET ON → CE LOW → charging active), configures the RTC wake timer, and enters System Sleep with GPIO latch (< 500µA). P0.04 is excluded from `disconnectLeakyPullups()` so the GPIO latch stays HIGH. Periodic RTC wakes (hourly) check voltage — only when recovery above `lowv_wake_mv` does it boot normally.
- **Solar Recovery:** In System Sleep, GPIO4 latch is preserved HIGH → CE FET ON → CE LOW → charging active. Solar charging continues autonomously until the battery charges above `lowv_wake_mv`. Without GPIO latch (RAK unpowered): ext. pull-down on gate → FET OFF → CE HIGH → charging OFF (safety default).

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
get board.jeitaignore
get board.telem
get board.stats
get board.cinfo
get board.selftest
get board.conf
```

## Getter Quick Reference (all relevant board getters)
- `get board.bat` - Current battery type (liion1s, lifepo1s, lto2s, naion1s, none).
- `get board.fmax` - Current frost charge behavior (0%/20%/40%/100%; N/A whenever the JEITA override is active).
- `get board.imax` - Maximum charge current in mA.
- `get board.mppt` - MPPT status (0/1).
- `get board.leds` - LED status (Heartbeat + BQ Stat).
- `get board.batcap` - Battery capacity in mAh (set/default).
- `get board.jeitaignore` - Frost charging override: `jeitaignore 0`, `jeitaignore 1`, `jeitaignore 1 (chemistry)` for LTO/Na-ion, or the stored setting with its blocker (`jeitaignore 1, N/A, batcap not set` / `jeitaignore 1, N/A, C>0.05`).
- `get board.telem` - Real-time telemetry (Battery/Solar incl. SOC, V/I/T). See [TELEMETRY.md](TELEMETRY.md) for what the app displays.
- `get board.stats` - Energy balance (24h/3d/7d), charge/discharge breakdown and MPPT ratio.
- `get board.cinfo` - Charger status (Charger State + Flags).
- `get board.selftest` - I²C hardware probe (`INA:OK BQ:OK RTC:OK BME:OK`). RTC includes a write/readback verify (state `WR_FAIL` on mismatch).
- `get board.conf` - Summary of all configs (B, F, M, I, Vco, V0; plus `J:1` while the frost charging override is on for Li-ion/LiFePO4).
- `get board.tccal` - NTC temperature calibration offset in °C (0.00 = default).
  - → [FAQ #12 — When should I run tccal?](FAQ.md#12-when-should-i-run-set-boardtccal)

---

## See Also

- [README.md](README.md) — Overview, feature matrix and diagnostics
- [DATASHEET.md](DATASHEET.md) — Hardware specifications and pinout
- [TELEMETRY.md](TELEMETRY.md) — Telemetry channels explained (what the app displays)
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Battery chemistry comparison and deployment guide
- [FAQ.md](FAQ.md) — Frequently asked questions
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — All board-specific CLI commands at a glance
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Complete technical documentation
