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

### 3-Layer Protection System
1. **Software Voltage Monitoring** - Adaptive task (10s/30s/60s intervals)
2. **RTC Wake-up Management** - Periodic recovery checks during SYSTEMOFF
3. **Hardware UVLO** - INA228 Alert → TPS62840 EN

### Voltage Thresholds (Li-Ion 1S)
- **Hardware Cutoff:** 3.2V (INA228 Alert)
- **Software Dangerzone:** 3.4V (initiateShutdown)
- **Wake Threshold:** 3.6V (Resume operation)

### Coulomb Counter
- **Real-time SOC tracking** via INA228
- **20mΩ shunt resistor** (1A max current)
- **Auto-learning** capacity calibration
- **7-day energy balance** analysis

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

board.life      # Get reduced voltage setting
                # Output: 1 (enabled) | 0 (disabled)

board.imax      # Get max charge current
                # Output: <current>mA (e.g., 200mA)

board.mppt      # Get MPPT status
                # Output: MPPT=1 (enabled) | MPPT=0 (disabled)

board.telem     # Get real-time telemetry
                # Output: B:<V>V/<I>mA/<T>C S:<V>V/<I>mA Y:<V>V
                # Example: B:3.85V/150mA/22C S:5.20V/85mA Y:3.30V

board.cinfo     # Get charger info (BQ25798 status)
                # Output: <state> + flags
                # States: !CHG, PRE, CC, CV, TRICKLE, TOP, DONE

board.diag      # Get detailed BQ25798 diagnostics 🆕
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

board.togglehiz # Toggle HIZ mode for debugging stuck PGOOD 🆕
                # Output: HIZ toggled: VBUS=<V>V PG=<status>
                # Manually toggles EN_HIZ bit to clear stuck states
                # Useful when PG=1 but HIZ=1 (stuck condition)

board.clearhiz  # Force clear HIZ mode 🆕
                # Output: HIZ cleared: VBUS=<V>V PG=<status>
                # Bypasses PGOOD check and forces HIZ=0
                # Use when MPPT won't activate despite solar present

board.soc       # Get battery State of Charge (v0.2 feature)
                # Output: SOC:<percent>% Cap:<mAh>mAh(learned|config)
                # Example: SOC:67.5% Cap:2000mAh(learned)

board.balance   # Get daily energy balance and forecast (v0.2 feature)
                # Output: Today:<+/- mAh> <SOLAR|BATTERY> 3dAvg:<mAh> [TTL:<hours>h]
                # Example: Today:+150mAh SOLAR 3dAvg:+120mAh
                # Example: Today:-80mAh BATTERY 3dAvg:-75mAh TTL:120h

board.mpps      # Get MPPT statistics (7-day average)
                # Output: MPPT enabled percentage and energy harvest data

board.conf      # Get all configuration values
                # Output: B:<bat> F:<frost> M:<mppt> I:<imax>mA Vco:<voltage>

board.wdtstatus # Get watchdog status
                # Output: WDT: enabled (600s timeout) or disabled (DEBUG_MODE)

board.ibcal     # Get INA228 calibration factor (v0.2 feature) 🆕
                # Output: INA228 calibration: <factor> (1.0=default)
                # Used to correct current measurement errors
```

### Set Commands
```bash
set board.bat <type>           # Set battery type
                               # Options: lto2s | lifepo1s | liion1s

set board.frost <behavior>     # Set frost charge behavior
                               # Options: 0% | 20% | 40% | 100%
                               # N/A for LTO batteries

set board.life <1|0>           # Enable/disable reduced charge voltage
                               # 1 = enabled, 0 = disabled

set board.imax <current>       # Set max charge current in mA
                               # Range: 10-1000mA

set board.mppt <1|0>           # Enable/disable MPPT
                               # 1 = enabled, 0 = disabled

set board.batcap <capacity>    # Set battery capacity in mAh (v0.2 feature)
                               # Range: 100-100000 mAh
                               # Used for accurate SOC calculation

set board.ibcal <factor>       # Set INA228 calibration factor (v0.2 feature) 🆕
                               # Range: 0.5-2.0 (default 1.0)
                               # Corrects current measurement errors
                               # Example: set board.ibcal 0.95 (5% reduction)

set board.bqreset              # Reset BQ25798 and reload config from FS
                               # Performs software reset and reconfigures
                               # all settings from stored preferences
```

## Diagnostics & Debugging 🆕

### BQ25798 Register Verification
Die neuen Diagnosefunktionen ermöglichen präzise Verifikation der BQ25798-Register gegen das Datasheet:

**Wichtige Register:**
- **0x0F (CHARGER_CONTROL_0)**: EN_HIZ (Bit 2), EN_CHG (Bit 5)
- **0x15 (MPPT_CONTROL)**: EN_MPPT (Bit 0), VOC_PCT (Bits 7-5), VOC_DLY (Bits 4-3), VOC_RATE (Bits 2-1)
- **0x1B (CHARGER_STATUS_0)**: PG_STAT (Bit 3), VINDPM (Bit 6), IINDPM (Bit 7)
- **0x1C (CHARGER_STATUS_1)**: CHG_STAT (Bits 7-5), VBUS_STAT (Bits 4-1)
- **0x1F (CHARGER_STATUS_4)**: Temperature status (Bits 3-0)

**Bekannte Probleme:**
1. **Stuck PGOOD**: HIZ bleibt manchmal auf 1 nach PG 0→1 Transition
   - Lösung: `board.togglehiz` oder automatisch via `checkAndFixPgoodStuck()`
2. **MPPT deaktiviert**: BQ25798 setzt MPPT=0 bei PG=0
   - Lösung: `checkAndFixSolarLogic()` reaktiviert MPPT bei PG=1

### INA228 Current Calibration (v0.2)
Bei Messdifferenzen zwischen INA228 und Referenzmessgerät:
```bash
# Messe echten Strom mit DMM: 100mA
# INA228 zeigt: 105mA
# Kalibrationsfaktor: 100/105 = 0.952
set board.ibcal 0.952
```

## Siehe auch

- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
- [Inhero_MR2_Datasheet_DE.md](Inhero_MR2_Datasheet_DE.md) - Datenblatt (falls vorhanden)
- [../inhero_mr1/README.md](../inhero_mr1/README.md) - Legacy v0.1 Hardware
