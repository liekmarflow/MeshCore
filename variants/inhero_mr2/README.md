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

board.learning  # Get auto-learning status (v0.2 feature) 🆕
                # Output: Learning IDLE | M1 ACTIVE | M2 ACTIVE
                # Shows current state of capacity auto-learning
                # Method 1: Full discharge cycle (100% → 10%)
                # Method 2: USB-C charging from danger zone (0% → 100%)
                # Example: M1 ACTIVE (100%→10%, 850 mAh) Cap:2000mAh(manual)
                # Example: Learning IDLE Cap:2150mAh(learned)

board.relearn   # Reset learned capacity flag (v0.2 feature) 🆕
                # Output: Learning reset - auto-learning enabled
                # Enables auto-learning to run again
                # Useful when battery capacity has changed

board.leds      # Get LED enable state (v0.2 feature) 🆕
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
                               # ⚠️ Resets learned flag - auto-learning will restart

set board.ibcal <factor>       # Set INA228 calibration factor (v0.2 feature) 🆕
                               # Range: 0.5-2.0 (default 1.0)
                               # Corrects current measurement errors
                               # Example: set board.ibcal 0.95 (5% reduction)

set board.bqreset              # Reset BQ25798 and reload config from FS
                               # Performs software reset and reconfigures
                               # all settings from stored preferences

set board.leds <on|off>        # Enable/disable heartbeat + BQ stat LED (v0.2) 🆕
                               # on/1 = enable, off/0 = disable
                               # Boot LEDs (3 blue flashes) always active
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

## Battery Capacity Auto-Learning 🆕

### Übersicht
Das MR-2 v0.2 kann die Batteriekapazität **automatisch erlernen** ohne manuelle Konfiguration. Dies ist besonders wichtig für LiFePO4-Batterien, bei denen Spannungsmessung unzuverlässig ist (flache Entladekurve @ 3.2-3.3V).

### Lernmethoden

#### Method 1: Full-Cycle Learning (100% → 10%)
**Automatischer Start:** Nach CHARGE_DONE (BQ25798 meldet "Fully Charged")  
**Endpunkt:** Danger Zone erreicht (z.B. 3.4V bei Li-Ion)  
**Dauer:** Abhängig von Batterie & Stromverbrauch (z.B. 29 Tage bei 10Ah/13mA)  
**Vorteil:** Sehr genau, funktioniert für alle Chemien  
**Nachteil:** Sehr langsam bei großen Batterien

**Wie es funktioniert:**
1. Batterie wird voll geladen (100% SOC)
2. System setzt Coulomb Counter auf 0 mAh
3. INA228 akkumuliert entladene Kapazität
4. Bei Danger Zone Eintritt: Gelernte Kapazität = Akkumulierte mAh

**Beispiel:**
```bash
board.learning
# Output: M1 ACTIVE (100%→10%, 850 mAh) Cap:2000mAh(manual)
```

#### Method 2: Reverse Learning (0% → 100%)
**Automatischer Start:** Nach Wake-up aus Danger Zone (Spannung erholt)  
**Endpunkt:** CHARGE_DONE (USB-C lädt Batterie voll)  
**Dauer:** Stunden (abhängig von Ladekapazität)  
**Vorteil:** Schnell, nutzerfreundlich  
**Nachteil:** Benötigt USB-C Ladung

**Wie es funktioniert:**
1. Batterie erreicht Danger Zone (< 3.4V) → Software Shutdown
2. RTC weckt Gerät nach 1 Stunde → Spannung geprüft
3. Spannung über Critical Threshold (≥ 3.4V) → Reverse Learning startet bei 0% SOC
4. Benutzer steckt USB-C ein → BQ25798 lädt Batterie
5. Bei CHARGE_DONE: Gelernte Kapazität = Akkumulierte mAh

**Beispiel:**
```bash
board.learning
# Output: M2 ACTIVE (0%→100%, 1850 mAh) Cap:2000mAh(manual)
```

### Learning Gate (Anti-Restart)
**Problem:** Ohne Gate würde Learning nach jedem Reboot neu starten  
**Lösung:** `capacity_learned` Flag im Filesystem

**Verhalten:**
- ✅ **Erste Inbetriebnahme:** `capacity_learned=false` → Auto-Learning startet
- ✅ **Nach erfolgreichem Learning:** `capacity_learned=true` → Kein Auto-Learning mehr
- ✅ **Nach Reboot:** Flag bleibt erhalten → Learning-Status persistent
- ✅ **Nach manuellem `set board.batcap`:** Flag wird auf `false` gesetzt → Learning startet neu
- ✅ **Nach `board.relearn` Befehl:** Flag wird auf `false` gesetzt → Learning erlaubt

### CLI Workflow

**1. Status prüfen:**
```bash
board.learning
# Output: Learning IDLE Cap:2150mAh(learned)
```

**2. Manuelle Kapazität setzen (optional):**
```bash
set board.batcap 10000
# Output: Battery capacity set to 10000 mAh
# ⚠️ Setzt capacity_learned=false → Auto-Learning aktiviert!
```

**3. Learning Status überwachen:**
```bash
board.learning
# Output: M2 ACTIVE (0%→100%, 3450 mAh) Cap:10000mAh(manual)
```

**4. Nach Abschluss:**
```bash
board.learning
# Output: Learning IDLE Cap:3520mAh(learned)
board.soc
# Output: SOC:45.2% Cap:3520mAh(learned)
```

**5. Re-Learning erzwingen:**
```bash
board.relearn
# Output: Learning reset - auto-learning enabled
board.learning
# Output: Learning IDLE Cap:3520mAh(manual/default)
# Beim nächsten CHARGE_DONE oder Danger Zone → Learning startet
```

### Best Practices

**Für neue Installationen:**
1. Batterie voll laden (USB-C oder Solar)
2. Gerät warten lassen → Method 1 startet automatisch
3. Nach ~29 Tagen (10Ah @ 13mA): Kapazität gelernt

**Für schnelles Learning:**
1. Batterie komplett entladen (Danger Zone erreichen)
2. USB-C einstecken → Method 2 startet automatisch
3. Nach Vollladung (CHARGE_DONE): Kapazität gelernt

**Für Batterie-Austausch:**
```bash
board.relearn  # Alte Kapazität vergessen
# Dann entweder Method 1 oder Method 2 warten
```

### Technische Details

**Persistenz:**
- Kapazität: `/inheromr2/batCap.txt` (uint16_t mAh)
- Learned Flag: `/inheromr2/cap_learned.txt` ("0" oder "1")

**Genauigkeit:**
- INA228: ±0.1% (24-bit ADC)
- Shunt: 20mΩ ±1% (0.5W)
- Temperaturkompensation: ±50ppm/°C

**Sicherheit:**
- Learning stoppt bei Danger Zone (Method 1)
- Learning stoppt bei CHARGE_DONE (Method 2)
- Keine Learning-Starts wenn `capacity_learned=true`

## Siehe auch

- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
- [Inhero_MR2_Datasheet_DE.md](Inhero_MR2_Datasheet_DE.md) - Datenblatt (falls vorhanden)
- [../inhero_mr1/README.md](../inhero_mr1/README.md) - Legacy v0.1 Hardware
