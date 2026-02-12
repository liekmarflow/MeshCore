# Inhero MR-2 (Hardware v0.2)

## Übersicht

Das Inhero MR-2 ist die zweite Generation des Mesh-Repeaters mit verbessertem Power-Management.

**Hardware Version:** v0.2  
**Key Features:**
- INA228 Power Monitor mit Coulomb Counter
- RV-3028-C7 RTC für Wake-up Management
- TPS62840 Buck Converter mit Hardware-UVLO
- BQ25798 Battery Charger mit MPPT

## Hardware-Unterschiede zu MR-1

| Feature | MR-1 (v0.1) | MR-2 (v0.2) |
|---------|-------------|-------------|
| Power Monitor | MCP4652 (Analog) | INA228 (Digital, I2C) |
| UVLO Protection | TP2120 (Hardware only) | INA228 Alert + Software |
| RTC | Keine | RV-3028-C7 |
| Coulomb Counter | Nein | Ja (INA228) |
| Wake-up | Nein | Ja (RTC Timer) |
| Battery SOC | Voltage-based | Coulomb Counting + Voltage |

## Power Management Features (v0.2)

### 2-Level Protection System
1. **Software Voltage Monitoring** - Two-stage strategy:
   - **Normal Mode**: 1 hour checks (system running, minimal cost)
   - **Danger Zone**: 12 hour RTC wake-ups (SYSTEMOFF, expensive boot)
2. **Hardware UVLO** - INA228 Alert → TPS62840 EN (absolute protection at hardware level)

**Key Insight:** Normal checks cost ~1µAh (INA228 I²C read), Danger Zone wake costs ~50-150mAh (full system boot). Strategy optimizes for maximum battery life.

### Voltage Thresholds (Li-Ion 1S)
- **Hardware Cutoff (UVLO):** 3.1V (INA228 Alert → TPS62840 EN, 64-sample averaging filters TX peaks)
- **Critical Threshold (0% SOC):** 3.4V (Danger zone boundary, software shutdown)
- **Hysteresis:** 300mV prevents motorboating and provides safety margin for voltage dips

### Coulomb Counter & Auto-Learning 🆕
- **Real-time SOC tracking** via INA228 (±0.1% accuracy)
- **20mΩ shunt resistor** (1A max current)
- **Dual-method auto-learning** capacity calibration:
  * **Method 1:** Full discharge cycle (100% → 10% danger zone, ~29 days @ 13mA)
  * **Method 2:** USB-C charging from danger zone (0% → 100%, ~hours)
- **Learning gate:** Auto-learning only starts if capacity not yet learned
- **Filesystem persistence:** Learned capacity retained across reboots
- **Manual override:** `set board.batcap` or `board.relearn` to re-enable learning
- **7-day energy balance** analysis for TTL forecasting

### Solar Power Management 🆕
- **Stuck PGOOD Detection:** Automatically detects slow sunrise conditions where PGOOD gets stuck and triggers input qualification via HIZ toggle (5-minute cooldown to prevent excessive toggling)
- **MPPT Recovery:** Re-enables MPPT when PowerGood=1 with 60-second cooldown to prevent interrupt loops between solar logic and BQ25798 interrupts
- **Interrupt Clearing:** Always clears BQ25798 interrupt flags (CHARGER_STATUS_0 register 0x1B) to ensure continued operation and prevent interrupt lockup
- **Fault Monitoring:** Diagnostic commands expose FAULT_STATUS registers (0x20, 0x21) for detailed error analysis including VBAT_OVP, VBUS_OVP, and thermal conditions
- **VREG Display:** Shows actual configured battery regulation voltage in diagnostics for voltage threshold verification

## Firmware Build

```bash
platformio run -e Inhero_MR2_repeater
```

## CLI Commands

### Get Commands
```bash
board.bat       # Get current battery type
                # Output: liion1s | lifepo1s | lto2s

board.hwver     # Get hardware version
                # Output: v0.2 (INA228+RTC)
                # Note: MR2 is always v0.2 hardware

board.frost     # Get frost charge behavior
                # Output: 0% | 20% | 40% | 100%
                # LTO batteries: N/A (JEITA disabled)

board.imax      # Get max charge current
                # Output: <current>mA (e.g., 200mA)

board.mppt      # Get MPPT status
                # Output: MPPT=1 (enabled) | MPPT=0 (disabled)

board.telem     # Get real-time telemetry with SOC 🆕
                # Output: B:<V>V/<I>mA/<T>C SOC:<percent>% S:<V>V/<I>mA
                # Example: B:3.85V/125.432mA/22.3C SOC:68.5% S:5.12V/245mA
                # If SOC not synced: B:3.85V/125.432mA/22.3C SOC:N/A S:5.12V/245mA
                # Components:
                # - B: Battery (Voltage/Current/Temperature/SOC)
                # - S: Solar (Voltage/Current)

board.stats     # Get energy statistics (balance + MPPT) 🆕
                # Output: <today>/<avg3d>mWh <SOL|BAT> M:<mppt>% [TTL:<hours>h]
                # Example: +1250/+450mWh SOL M:85%
                # Example: -300/-450mWh BAT M:45% TTL:72h
                # Components:
                # - +1250: Today's net balance (solar - consumption) in mWh
                # - +450: 3-day average balance in mWh  
                # - SOL: Running on solar (self-sufficient)
                # - BAT: Living on battery (deficit mode)
                # - M:85%: MPPT enabled percentage (7-day average)
                # - TTL:72h: Time To Live (hours until empty, only shown if BAT mode)

board.cinfo     # Get charger info (BQ25798 status)
                # Output: <state> + flags
                # States: !CHG, PRE, CC, CV, TRICKLE, TOP, DONE

board.diag      # Get detailed BQ25798 diagnostics 
                # Output: PG CE HIZ MPPT CHG VBUS VINDPM IINDPM | voltages | temps | registers | VOC config
                # Example: PG:1 CE:1 HIZ:0 MPPT:1 CHG:CC VBUS:UnkAdp VINDPM:1 IINDPM:0 | 
                #          Vbus:6.22V Vbat:3.35V Ibat:0mA Temp:31C | 
                #          TS: OK | R0F:0x23 R15:0xAB | VOC:87.5%/300ms/2min
                # Key diagnostics for debugging charging issues:
                # - PG: Power Good status (1=good, 0=no power)
                # - CE: Charge Enable (1=enabled, 0=disabled)
                # - HIZ: High Impedance mode (0=normal, 1=input disabled)
                # - MPPT: Maximum Power Point Tracking (1=active, 0=inactive)
                # - CHG: Charge state (!CHG|TRKL|PRE|CC|CV|TOP|DONE)
                # - VBUS: Input source type (NoIn|SDP|CDP|DCP|UnkAdp|NStd|NotQual|DirPwr)
                # - VINDPM: Voltage input DPM active (1=limiting, 0=ok)
                # - IINDPM: Current input DPM active (1=limiting, 0=ok)
                # - VOC: MPPT VOC configuration (percentage/delay/rate)

board.togglehiz # Force input detection via HIZ cycle 🆕
                # Output: HIZ cycle <was set|forced>: VBUS=<V>V PG=<status>
                # Same logic as automatic task in checkAndFixPgoodStuck()
                # If HIZ=1: Clears HIZ → input qualification
                # If HIZ=0: Set HIZ briefly, then clear → triggers input detection
                # Always ends with HIZ=0
                # Useful for manually triggering stuck PGOOD recovery

board.conf      # Get all configuration values
                # Output: B:<bat> F:<frost> M:<mppt> I:<imax> Vco:<voltage>

board.ibcal     # Get INA228 calibration factor (v0.2 feature)
                # Output: INA228 calibration: <factor> (1.0=default)
                # Used to correct current measurement errors

board.leds      # Get LED enable state (v0.2 feature)
                # Output: "LEDs: ON (Heartbeat + BQ Stat)" or "LEDs: OFF (Heartbeat + BQ Stat)"
                # Shows whether heartbeat LED and BQ25798 stat LED are enabled
```

### Set Commands
```bash
set board.bat <type>           # Set battery type
                               # Options: lto2s | lifepo1s | liion1s

set board.frost <behavior>     # Set frost charge behavior
                               # Options: 0% | 20% | 40% | 100%
                               # N/A for LTO batteries

set board.imax <current>       # Set max charge current in mA
                               # Range: 10-1000mA

set board.mppt <1|0>           # Enable/disable MPPT
                               # 1 = enabled, 0 = disabled

set board.batcap <capacity>    # Set battery capacity in mAh (v0.2 feature)
                               # Range: 100-100000 mAh
                               # Used for accurate SOC calculation

set board.ibcal <current_mA>   # Calibrate INA228 current sensor (v0.2 feature)
                               # Range: -2000 to +2000 mA
                               # Measures actual current, calculates correction factor
                               # Example: set board.ibcal 100.5
                               # Output: INA228 calibrated: factor=0.9850

set board.bqreset              # Reset BQ25798 and reload config from FS
                               # Performs software reset and reconfigures
                               # all settings from stored preferences

set board.leds <on|off>        # Enable/disable heartbeat + BQ stat LED (v0.2)
                               # on/1 = enable, off/0 = disable
                               # Boot LEDs (3 blue flashes) always active
```

## Diagnostics & Debugging

### BQ25798 Register Verification
Die Diagnosefunktionen ermöglichen präzise Verifikation der BQ25798-Register gegen das Datasheet:

**Wichtige Register:**
- **0x0F (CHARGER_CONTROL_0)**: EN_HIZ (Bit 2), EN_CHG (Bit 5)
- **0x15 (MPPT_CONTROL)**: EN_MPPT (Bit 0), VOC_PCT (Bits 7-5), VOC_DLY (Bits 4-3), VOC_RATE (Bits 2-1)
- **0x1B (CHARGER_STATUS_0)**: PG_STAT (Bit 3), VINDPM (Bit 6), IINDPM (Bit 7)
- **0x1C (CHARGER_STATUS_1)**: CHG_STAT (Bits 7-5), VBUS_STAT (Bits 4-1)
- **0x1F (CHARGER_STATUS_4)**: Temperature status (Bits 3-0)

**Bekannte Probleme:**
1. **Stuck PGOOD**: Langsamer Sonnenaufgang kann Input-Qualifikation verhindern
   - Symptom: VBUS >3.5V aber PG=0
   - Lösung: `board.togglehiz` oder automatisch via `checkAndFixPgoodStuck()` (15min Intervall)
   - Mechanismus: HIZ-Toggle triggert Input-Qualifikation (per BQ25798 Datasheet)
2. **MPPT deaktiviert**: BQ25798 setzt MPPT=0 automatisch bei PG=0
   - Lösung: `checkAndFixSolarLogic()` reaktiviert MPPT bei PG=1 (60s Cooldown)

### INA228 Current Calibration (v0.2)
Bei Messdifferenzen zwischen INA228 und Referenzmessgerät:
```bash
# Messe echten Strom mit DMM während konstantem Load: 100.5mA
# INA228 zeigt: 102.3mA
# Kalibriere mit tatsächlichem Wert:
set board.ibcal 100.5
# Output: INA228 calibrated: factor=0.9824

# Prüfe Ergebnis:
board.ibcal
# Output: INA228 calibration: 0.9824 (1.0=default)
```

**Kalibrationsprozess:**
1. Anlegen einer bekannten Last (z.B. USB-C Charge mit DMM)
2. DMM-Messung als Referenz nehmen
3. `set board.ibcal <actual_mA>` aufrufen
4. Faktor wird berechnet: `factor = actual / ina228_reading`
5. Faktor persistent gespeichert und auf alle zukünftigen Messungen angewendet

## Siehe auch

- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
- [BATTERY_AUTO_LEARNING.md](BATTERY_AUTO_LEARNING.md) - Battery capacity auto-learning (future feature)
- [../inhero_mr1/README.md](../inhero_mr1/README.md) - Legacy v0.1 Hardware
