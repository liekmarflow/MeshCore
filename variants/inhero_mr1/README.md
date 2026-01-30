[English](README.md) | [Deutsch](README.de.md)

# Inhero MR-1 Hardware Variant

The Inhero MR-1 is a solar-powered mesh network node featuring advanced battery management, configurable chemistry support, and MPPT (Maximum Power Point Tracking) control.

## Hardware Overview

### Main Components
- **MCU**: Nordic nRF52840 (64MHz, 243KB RAM, 796KB Flash)
- **Radio**: SX1262 LoRa transceiver with DIO2 RF switching
- **Battery Manager**: BQ25798 with integrated MPPT and NTC thermistor support
- **Digital Potentiometer**: MCP4652 (dual channel) for voltage adjustment
- **Power Input**: Solar panel with Power Good interrupt detection
- **Storage**: LittleFS-based preferences with SimplePreferences wrapper

### Pin Configuration
```cpp
// LoRa Radio
P_LORA_DIO_1    = 47
P_LORA_NSS      = 42
P_LORA_BUSY     = 46
P_LORA_SCLK     = 43
P_LORA_MISO     = 45
P_LORA_MOSI     = 44
SX126X_POWER_EN = 37

// GPS (Optional)
PIN_GPS_1PPS    = 17
GPS_ADDRESS     = 0x42 (I2C)

// Battery Monitoring
PIN_VBAT_READ   = 5
```

## Features

### Battery Management
- **Multi-Chemistry Support**:
  - LTO 2S (Lithium Titanate Oxide, 2 cells in series)
  - LiFePO4 1S (Lithium Iron Phosphate, 1 cell)
  - Li-Ion 1S (Lithium-Ion, 1 cell)
  
- **JEITA Temperature Control**:
  - Automatic charge current reduction at low/high temperatures
  - NTC thermistor-based battery temperature monitoring
  - Configurable temperature thresholds (COOL, WARM, HOT, COLD)

- **Frost Charge Behavior**:
  - `0%` - No charging below 0°C
  - `20%` - Reduce charge current to 20% at low temperatures
  - `40%` - Reduce charge current to 40% at low temperatures
  - `100%` - No reduction (not recommended for cold environments)

- **Configurable Parameters**:
  - Maximum charge current (0-3000mA)
  - Reduced charge voltage (for extended battery life)
  - MPPT enable/disable

### Solar Power Management
- **MPPT Task**: FreeRTOS task monitoring solar input every 15 minutes
- **Interrupt-Driven**: Hardware interrupt on Power Good state changes
- **Automatic Recovery**: Detects and re-enables MPPT if disabled by chip glitches
- **Visual Feedback**: Blue LED flash on solar events

### Telemetry System
Uses CayenneLPP format with two channels:

**Channel N (Solar Input)**:
- Voltage (V)
- Current (A)
- Power (W)
- MPPT Status (boolean)

**Channel N+1 (Battery)**:
- Voltage (V)
- Current (A) - Charge/Discharge
- Power (W)
- Temperature (°C)

## Configuration

### CLI Commands

#### Get Commands
```bash
board.bat       # Get current battery type
                # Output: liion1s | lifepo1s | lto2s

board.frost     # Get frost charge behavior
                # Output: 0% | 20% | 40% | 100%
                # LTO batteries: N/A (JEITA disabled)

board.life      # Get reduced voltage setting
                # Output: 1 (enabled) | 0 (disabled)

board.imax      # Get max charge current
                # Output: <number> (in mA, e.g., "200")

board.mppt      # Get MPPT status
                # Output: MPPT=1 (enabled) | MPPT=0 (disabled)

board.mpps      # Get MPPT statistics (7-day average)
                # Output: 7d:<percent>% 3d:<energy>mWh (<days>d)
                # Example: 7d:87.3% 3d:1250mWh (3.0d)

board.cinfo     # Get charger status
                # Output: <power_status> / <charge_status>
                # Power: PG (Power Good) | !PG (Power not Good)
                # Charge: !CHG | PRE | CC | CV | TRICKLE | TOP | DONE | Unknown

board.telem     # Get full telemetry snapshot
                # Output: B:<V>V/<mA>mA/<T>C S:<V>V/<mA>mA Y:<V>V
                # B = Battery, S = Solar, Y = System voltage
                # Example: B:3.85V/150mA/22C S:5.20V/85mA Y:3.30V

board.conf      # Get all configuration values
                # Output: B:<bat> F:<frost> M:<mppt> I:<imax>mA Vco:<voltage>
                # Example: B:liion1s F:0% M:0 I:200mA Vco:4.20
```

#### Set Commands
```bash
set board.bat <type>           # Set battery type
                               # Options: lto2s | lifepo1s | liion1s

set board.frost <behavior>     # Set frost charge behavior
                               # Options: 0% | 20% | 40% | 100%
                               # N/A for LTO batteries

set board.life <1|0>           # Enable/disable reduced charge voltage
                               # 1 = enabled, 0 = disabled
                               # Extends battery life by reducing max voltage

set board.imax <current>       # Set max charge current in mA
                               # Range: 10-1000mA

set board.mppt <1|0>           # Enable/disable MPPT
                               # 1 = enabled, 0 = disabled
```

#### Output Abbreviations

**board.cinfo (Charger Info)**:
- `PG` - Power Good (solar input sufficient)
- `!PG` - No Power Good (solar input insufficient)
- `!CHG` - Not Charging
- `PRE` - Precharging (battery voltage too low)
- `CC` - Constant Current charging
- `CV` - Constant Voltage charging
- `TRICKLE` - Trickle charging (low current maintenance)
- `TOP` - Top of Timer Active
- `DONE` - Charging complete

**board.telem (Telemetry)**:
- `B` - Battery voltage/current/temperature
- `S` - Solar panel voltage/current
- `Y` - System voltage
- Temperature in Celsius (C)

**board.conf (Configuration)**:
- `B` - Battery type
- `F` - Frost charge behavior
- `M` - MPPT enabled (1/0)
- `I` - Max charge current (mA)
- `Vco` - Charge cutoff voltage (V)

**board.mpps (MPPT Statistics)**:
- `7d` - 7-day MPPT uptime percentage
- `3d` - 3-day average daily energy harvest (mWh)
- Last value shows days of available data

### Configuration Examples

**Standard Li-Ion Setup (Default)**:
```bash
set board.bat liion1s
set board.frost 20%
set board.life 0
set board.imax 500
set board.mppt 1
```

**LiFePO4 Long-Life Setup**:
```bash
set board.bat lifepo1s
set board.frost 40%
set board.life 1
set board.imax 300
set board.mppt 1
```

**Cold Weather Setup**:
```bash
set board.frost 0%     # No charging below 0°C
set board.imax 200     # Reduced current
```

### MPPT Statistics

The board automatically tracks MPPT status and solar energy harvesting over time using an interrupt-driven approach:

- **Tracking Period**: 7 days (168 hours)
- **Update Trigger**: BQ25798 interrupts (on MPPT status changes) + periodic 15-min checks
- **Storage**: Rolling circular buffer (in-memory)
- **Metrics**: 
  - MPPT uptime: Percentage of time MPPT was enabled (7-day moving average)
  - Solar energy: Average daily harvested energy in mWh (3-day moving average)
- **Energy Integration**: Calculates E = P × Δt using solar panel power readings
- **Task Integration**: Uses existing `solarMpptTask` - no extra task overhead

**Example Output**:
```bash
board.mpps
> 7d:87.3% 3d:1250mWh (3.0d)
```

This shows that:
- MPPT was enabled for 87.3% of the time over the last 7 days
- The solar panel harvested an average of 1250 mWh per day over the last 3 days
- 3.0 days of valid data are available for the energy calculation

The 7-day MPPT percentage provides long-term stability indication, while the 3-day energy average balances responsiveness to weather changes with stability against short-term fluctuations.

**Use Cases**:
- Monitor MPPT stability over time
- Detect configuration issues or hardware glitches
- Track solar charging efficiency and daily energy harvest
- Compare energy harvest across different days/seasons
- Verify automatic MPPT recovery is working
- Estimate battery lifetime based on actual solar input

**Implementation**:
The statistics tracking is fully integrated into the existing `solarMpptTask` and leverages the BQ25798's interrupt system. When the MPPT status changes, an interrupt is triggered and the elapsed time is accounted for. Additionally, solar panel power (voltage × current) is continuously integrated to calculate harvested energy: E = P × Δt. This interrupt-driven approach is highly efficient and accurate, requiring no polling overhead. Time tracking uses RTC when available, with automatic fallback to millis() during system startup.

## Technical Details

### Battery Charger (BQ25798)
- **Input Voltage Range**: 3.6V - 24V
- **Charge Current**: Configurable 10mA - 5A
- **MPPT**: Automatic maximum power point tracking for solar
- **ADC Resolution**: 15-bit (configurable to 12/13/14-bit)
- **Temperature Sensing**: NTC thermistor with Beta equation calculation
- **Interrupt System**: Solar-only interrupts (Power Good monitoring)

### Digital Potentiometer (MCP4652)
- **Channels**: 2 independent wipers
- **Resolution**: 257 steps (0-256)
- **Interface**: I2C (default address 0x2F)
- **Purpose**: Battery chemistry voltage adjustment

### Preferences Storage
Persistent configuration stored in LittleFS:
- **Namespace**: `inhero_mr1`
- **Keys**: 
  - `battery_type` - Current battery chemistry
  - `frost_behavior` - Low temperature charging behavior
  - `max_charge_current_ma` - Charge current limit
  - `reduced_voltage` - Reduced voltage flag

### Temperature Calculation
The board uses a Steinhart-Hart / Beta equation for accurate battery temperature:
```cpp
// NTC Resistor Network
R_PULLUP    = 5600Ω   (RT1)
R_PARALLEL  = 27000Ω  (RT2)
R_NTC_25    = 10000Ω  (NCP15XH103F03RC at 25°C)
BETA_VAL    = 3380    (Beta coefficient)
```

## Build Information

### Dependencies
- RadioLib @ 7.5.0
- Adafruit BQ25798 Library
- Adafruit BME280 Library
- Adafruit GFX Library
- CayenneLPP @ 1.6.1
- ArduinoJson @ 7.4.2

### Build Configuration
- **Platform**: Nordic nRF52 (10.10.0)
- **Framework**: Arduino
- **Board Definition**: Custom `inhero_mr1`
- **Environment**: `Inhero_MR1_repeater`

### Memory Usage (Typical)
```
RAM:   11.6% (28,772 bytes / 248,832 bytes)
Flash: 37.1% (302,080 bytes / 815,104 bytes)
```

## Safety Features

1. **Thermal Protection**: JEITA-compliant temperature monitoring
2. **Input Protection**: VINDPM (Input voltage dynamic power management)
3. **Battery Protection**: Over-current, over-voltage, under-voltage protection
4. **Bounds Checking**: All configuration values validated before applying
5. **Watchdog**: Hardware watchdog (600s timeout, enabled in release builds)

## Development

### Watchdog Timer

The Inhero MR-1 features a hardware watchdog timer (nRF52 WDT) with the following characteristics:

- **Timeout**: 120 seconds (2 minutes)
- **Activation**: Automatically enabled in release builds (disabled in DEBUG_MODE)
- **Purpose**: Detects and recovers from system hangs, deadlocks, or task failures
- **Behavior**: Continues running during sleep, pauses during debugging
- **Feeding**: Must be called from main loop via `board.tick()`

**Important**: Your application **must** call `board.tick()` regularly to feed the watchdog:

```cpp
void loop() {
  board.tick();        // Feed watchdog - REQUIRED!
  the_mesh.loop();
  sensors.loop();
  rtc_clock.tick();
}
```

Without calling `board.tick()`, the system will reset after 600 seconds. This is intentional - it ensures the main loop is running properly.

**Reset Detection**: Watchdog resets are logged via `NRF52Board::getResetReasonString()` and will show as "Watchdog" in debug output.

### Adding Custom Features
The board class `InheroMr1Board` inherits from `mesh::MainBoard` and provides:
- `getCustomGetter()` - Custom CLI get commands
- `setCustomSetter()` - Custom CLI set commands  
- `queryBoardTelemetry()` - CayenneLPP telemetry export
- `getBattMilliVolts()` - Battery voltage reading

### FreeRTOS Task
The `solarMpptTask` runs continuously:
- **Priority**: 1 (low)
- **Stack Size**: 4096 bytes
- **Wake Conditions**: 
  - Solar interrupt (Power Good state change)
  - 15-minute timeout (periodic check)

## Troubleshooting

### MPPT Not Working
1. Check solar panel voltage (must be > battery voltage)
2. Verify MPPT is enabled: `board.mppt` (should show `MPPT=1`)
3. Check charger status: `board.cinfo` (should show `PG` for Power Good)
4. Monitor telemetry: `board.telem`

### Battery Not Charging
1. Check temperature is within range: `board.telem`
2. Verify charge current is not 0: `board.imax`
3. Check frost behavior setting: `board.frost`
4. Verify battery chemistry is correct: `board.bat`
5. Check charger info for status: `board.cinfo`

### Preferences Not Saving
1. Check LittleFS is mounted correctly
2. Verify sufficient flash space available
3. Check logs for filesystem errors

## License

```
Copyright (c) 2026 Inhero GmbH
SPDX-License-Identifier: MIT
```

## Author

Developed for the MeshCore project by Inhero GmbH.

## Version History

- **v1.0** (2026-01-27)
  - Initial release
  - Multi-chemistry battery support
  - MPPT solar management
  - CLI configuration interface
  - CayenneLPP telemetry export
