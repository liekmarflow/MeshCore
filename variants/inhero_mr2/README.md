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

set board.bqreset              # Reset BQ25798 and reload config from FS
                               # Performs software reset and reconfigures
                               # all settings from stored preferences
```

## Siehe auch

- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
- [Inhero_MR2_Datasheet_DE.md](Inhero_MR2_Datasheet_DE.md) - Datenblatt
- [../inhero_mr1/README.md](../inhero_mr1/README.md) - Legacy v0.1 Hardware
