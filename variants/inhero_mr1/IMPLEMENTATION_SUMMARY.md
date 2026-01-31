# Inhero MR-1 Power Management - Implementierungs-Plan

> ⚠️ **STATUS: NICHT IMPLEMENTIERT** ⚠️
> 
> Diese Dokumentation beschreibt den geplanten Power-Management-Ansatz für das Inhero MR-1 Board.
> Die Implementierung erfolgt **nach Verfügbarkeit der Hardware mit RV-3028-C7 RTC**.
> 
> Datum: 29. Januar 2026
> Version: 1.0 (Design-Phase)

> **Gedankenstütze:** Beim Start muss der RAK zuerst die board.bat-Konfiguration vom Filesystem lesen und anschließend die Batteriespannung (VBAT) über den BQ25798 messen. Erst danach entscheidet die Firmware anhand der hinterlegten Voltage-Schwellen, ob sofort wieder geschlafen wird oder der normale TX-Betrieb aufgenommen wird. Es ist möglich, dass der RAK zwischenzeitlich durch den On-Board-Komparator komplett stromlos war.

---

## Überblick

Das System kombiniert **3 Schutz-Schichten** für optimalen Filesystem-Schutz und Energie-Effizienz:

---

## 1. Software-Monitoring (Adaptiv)

### Implementierung
- **Task**: `voltageMonitorTask()` in `VoltageMonitor_Implementation.cpp`
- **Messung**: BQ25798 15-bit ADC via I²C
- **Frequenz**: Dynamisch

| Spannung | Zustand | Intervall | Aktion |
|----------|---------|-----------|--------|
| > 3.6V (Li-Ion) | NORMAL | 60s | Normalbetrieb |
| 3.5-3.6V | WARNING | 30s | Load shedding vorbereiten |
| 3.4-3.5V | CRITICAL | 10s | Minimalbetrieb |
| < 3.4V | SHUTDOWN | Sofort | Controlled shutdown |

### Chemie-spezifische Schwellen

| Chemie | HW Cutoff | SW Sleep | Normalbetrieb |
|--------|-----------|----------|---------------|
| Li-Ion 1S | 3.2V | 3.4V | 3.6V |
| LiFePO4 1S | 2.8V | 2.9V | 3.0V |
| LTO 2S | 4.0V | 4.2V | 4.4V |

> **Hinweis:** Die Hardware-Hysterese kann relativ klein gewählt werden (z.B. Li-Ion: 100mV, LiFePO4: 80mV), da der nRF52 beim Booten ohnehin die Batteriespannung prüft und bei zu niedriger Spannung sofort wieder in den Sleep-Modus geht. Die Software-Schicht sorgt für die eigentliche Sicherheit und verhindert Datenverlust.

---

## 2. RTC Wake-up (Periodisch)

### Hardware
- **Chip**: RV-3028-C7
- **INT-Pin**: GPIO17 (WB_IO1)
- **I²C-Adresse**: 0x52

### Funktionsweise
```cpp
// Bei Shutdown:
1. configureRTCWake(12);  // 12 Stunden
2. SYSTEMOFF

// Nach 12h:
3. RTC INT-Pin → Wake-up
4. begin() prüft GPREGRET2
5. Wenn shutdown_reason == LOW_VOLTAGE:
   - Checke VBAT via BQ25798
   - Wenn < Wake-Threshold: Zurück zu SYSTEMOFF
   - Wenn >= Wake-Threshold: Normal weiter
```

### Energie-Bilanz
- **Stromverbrauch SYSTEMOFF**: 1-5 µA
- **Wake-up 2x/Tag**: ~0.06 mAh/Tag
- **Vernachlässigbar** bei Solar-Betrieb

---

## 3. TP2120 Hardware-Cutoff (Ultimativ)

### Konfiguration via MCP4652
| Chemie | Voff (Hardware) | Von (Hardware) | Hysterese |
|--------|----------------|----------------|-----------|
| Li-Ion | 3.2V | 3.6V | 400mV |
| LiFePO4 | 2.8V | 3.0V | 200mV |
| LTO | 4.0V | 4.4V | 400mV |

### Schutz-Verhalten
- Bei VBAT < Voff: TPS62840 Enable → LOW
- RAK komplett stromlos (0 µA)
- BQ25798 lädt weiter
- Bei VBAT > Von: TPS62840 Enable → HIGH
- Hardware-Boot, normal weiter

---

## Szenarien

### Szenario A: Normale Entladung (Software-Shutdown) - Li-Ion
```
t=0:    VBAT = 3.7V → Normal (60s checks)
t=+1h:  VBAT = 3.5V → Warning (30s checks)
t=+2h:  VBAT = 3.45V → Critical (10s checks)
t=+2.5h: VBAT = 3.39V → Software Sleep (< 3.4V)
         - Filesystem sync
         - RTC: Wake in 12h
         - SYSTEMOFF
t=+14.5h: RTC weckt auf
         - VBAT = 3.5V (noch unter Normalbetrieb 3.6V)
         - Zurück zu SYSTEMOFF (12h)
t=+26.5h: RTC weckt auf
         - VBAT = 3.65V (OK, über 3.6V)
         - Normal weiter ✅
```

### Szenario B: Kritische Entladung (Hardware-Cutoff) - Li-Ion
```
t=0:    VBAT = 3.4V → Software Sleep
        - Filesystem sync
        - RTC: Wake in 12h
        - SYSTEMOFF
t=+1h:  VBAT = 3.15V (weiter gesunken im Sleep!)
        → TP2120 schaltet ab (< 3.2V)
        → RAK stromlos, RTC läuft weiter
t=+6h:  Solar lädt → VBAT = 3.65V
        → TP2120 schaltet ein (> 3.6V)
        → Hardware-Boot
        → begin() checkt Shutdown-Grund
        → Normal weiter ✅
```

### Szenario C: Schnelle Recovery - Li-Ion
```
t=0:    VBAT = 3.35V → Software Sleep (< 3.4V)
        - RTC: Wake in 12h
        - SYSTEMOFF
t=+0.5h: Sonne kommt → VBAT = 3.8V
         (Aber: Noch kein RTC-Wake)
t=+12h: RTC weckt auf
        - VBAT = 4.1V (voll geladen, über 3.6V)
        - Normal weiter ✅
```

---

## Integration

### Dateien
1. `InheroMr1Board.h` - Power Management Definitionen
2. `InheroMr1Board.cpp` - Shutdown/Wake-up Logik
3. `VoltageMonitor_Implementation.cpp` - Adaptive Monitoring-Task
4. `BoardConfigContainer.cpp` - Integration der Monitoring-Task

### Aktivierung
Die Monitoring-Task muss in `BoardConfigContainer::begin()` gestartet werden:

```cpp
// In BoardConfigContainer.cpp, nach MPPT-Task:
xTaskCreate(voltageMonitorTask, "VoltMon", 2048, NULL, 2, &voltageMonitorTaskHandle);
```

---

## Vorteile der Lösung

✅ **Filesystem-Schutz**: Controlled shutdown 200mV VOR Hardware-Cutoff
✅ **Kein Forever-Sleep**: RTC weckt 2x/Tag (12h-Intervall)
✅ **Energie-effizient**: Adaptive Monitoring spart Energie
✅ **Chemie-agnostisch**: Automatische Schwellen-Anpassung
✅ **Fail-Safe**: TP2120 als ultimativer Schutz bei Software-Crash
✅ **Diagnostik**: GPREGRET2 + CLI für Debugging
✅ **Hardware-verdrahtet**: RTC INT-Pin bereits auf PCB vorhanden

---

## CLI-Befehle (zukünftig)

```bash
> get pwrmgt.status
State: NORMAL, VBAT: 4.05V
Monitor interval: 60s
Last shutdown: None

> get pwrmgt.thresholds
Chemistry: liion1s
Critical: 3200mV
Wake: 3300mV (100mV hysteresis)
Hardware Voff: 3000mV
Hardware Von: 3450mV (450mV hysteresis)

> get pwrmgt.rtc
RTC: Enabled
Next wake: 2026-01-30 14:30:00 (11h 23m)

> set pwrmgt.test shutdown
Initiating test shutdown...
```

---

## Code-Implementierung (Vorbereitet)

### 1. InheroMr1Board.h - Erweiterungen

```cpp
// Power Management Configuration
#define RTC_INT_PIN 17  // GPIO17 (WB_IO1) - RTC Interrupt
#define RTC_I2C_ADDR 0x52  // RV-3028-C7 I2C address

// Shutdown reason codes (stored in GPREGRET2)
#define SHUTDOWN_REASON_NONE 0x00
#define SHUTDOWN_REASON_LOW_VOLTAGE 0x01
#define SHUTDOWN_REASON_USER_REQUEST 0x02
#define SHUTDOWN_REASON_THERMAL 0x03

// Power Management Methods (zu InheroMr1Board Klasse hinzufügen)
public:
  /// @brief Initiate controlled shutdown with filesystem protection
  /// @param reason Shutdown reason code (stored in GPREGRET2 for next boot)
  void initiateShutdown(uint8_t reason);
  
  /// @brief Configure RV-3028 RTC countdown timer for periodic wake-up
  /// @param hours Wake-up interval in hours (typically 12 = 2x per day)
  void configureRTCWake(uint32_t hours);
  
  /// @brief Get voltage threshold for critical shutdown (chemistry-specific)
  /// @return Threshold in millivolts
  uint16_t getVoltageCriticalThreshold();
  
  /// @brief Get voltage threshold for wake-up with hysteresis (chemistry-specific)
  /// @return Threshold in millivolts (typically critical + 100mV)
  uint16_t getVoltageWakeThreshold();
  
  /// @brief RTC interrupt handler (called by hardware interrupt)
  static void rtcInterruptHandler();
```

### 2. InheroMr1Board.cpp - begin() Erweiterung

```cpp
void InheroMr1Board::begin() {
  // ... existing initialization ...
  
  boardConfig.begin();  // Initializes BQ25798, MCP4652, TP2120 thresholds
  
  // Enable DC/DC converter
  NRF52BoardDCDC::begin();
  
  // ... existing SX1262 power-up ...
  
  // ===== POWER MANAGEMENT: Check wake-up reason =====
  uint32_t reset_reason = NRF_POWER->RESETREAS;
  uint8_t shutdown_reason = NRF_POWER->GPREGRET2;
  
  if (shutdown_reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
    MESH_DEBUG_PRINTLN("PWRMGT: Woke from low voltage shutdown");
    
    // Check if voltage has recovered (with hysteresis)
    uint16_t vbat_mv = getBattMilliVolts();
    uint16_t wake_threshold = getVoltageWakeThreshold();
    
    MESH_DEBUG_PRINTLN("PWRMGT: VBAT=%dmV, Wake threshold=%dmV", vbat_mv, wake_threshold);
    
    if (vbat_mv < wake_threshold) {
      // Still too low, go back to sleep (12h wait)
      MESH_DEBUG_PRINTLN("PWRMGT: Voltage still low, going back to sleep");
      delay(100);  // Let debug output complete
      initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE);
      // Never returns
    }
    
    // Voltage OK, continue boot
    MESH_DEBUG_PRINTLN("PWRMGT: Voltage recovered, resuming operation");
    NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
  }
  
  // Configure RTC INT pin for wake-up from SYSTEMOFF
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), rtcInterruptHandler, FALLING);
}
```

### 3. Shutdown-Sequenz

```cpp
void InheroMr1Board::initiateShutdown(uint8_t reason) {
  MESH_DEBUG_PRINTLN("PWRMGT: Initiating shutdown (reason=0x%02X)", reason);
  
  // 1. Stop background tasks to prevent filesystem corruption
  BoardConfigContainer::stopBackgroundTasks();
  
  if (reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
    MESH_DEBUG_PRINTLN("PWRMGT: Low voltage shutdown - syncing filesystem");
    
    // TODO: Explicit filesystem sync when LittleFS is integrated
    // For now, stopBackgroundTasks() should flush pending writes
    delay(100);  // Allow I/O to complete
    
    // 2. Configure RTC to wake us up in 12 hours
    configureRTCWake(12);
  }
  
  // 3. Store shutdown reason for next boot
  NRF_POWER->GPREGRET2 = reason;
  
  // 4. Enter SYSTEMOFF mode (1-5 µA)
  MESH_DEBUG_PRINTLN("PWRMGT: Entering SYSTEMOFF");
  delay(50);  // Let debug output complete
  
  sd_power_system_off();
  // Never returns
}
```

### 4. RTC Wake-up Konfiguration

```cpp
void InheroMr1Board::configureRTCWake(uint32_t hours) {
  MESH_DEBUG_PRINTLN("PWRMGT: Configuring RTC wake in %d hours", hours);
  
  // RV-3028-C7 Register addresses
  const uint8_t RV3028_CTRL1 = 0x00;
  const uint8_t RV3028_CTRL2 = 0x01;
  const uint8_t RV3028_COUNTDOWN_LSB = 0x09;
  const uint8_t RV3028_COUNTDOWN_MSB = 0x0A;
  
  // Calculate countdown value
  // Using 1Hz tick rate: countdown = hours * 3600 seconds
  uint16_t countdown = (hours > 18) ? 65535 : (hours * 3600);
  
  // Write countdown value
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_COUNTDOWN_LSB);
  Wire.write(countdown & 0xFF);        // LSB
  Wire.write((countdown >> 8) & 0xFF); // MSB
  Wire.endTransmission();
  
  // Enable countdown timer
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_CTRL1);
  Wire.write(0x01);  // TE (Timer Enable) bit
  Wire.endTransmission();
  
  // Enable countdown interrupt
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_CTRL2);
  Wire.write(0x80);  // TIE (Timer Interrupt Enable) bit
  Wire.endTransmission();
  
  MESH_DEBUG_PRINTLN("PWRMGT: RTC countdown configured (%d ticks)", countdown);
}

void InheroMr1Board::rtcInterruptHandler() {
  // RTC countdown elapsed - device woke from SYSTEMOFF
  // Clear interrupt flag to allow next countdown
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(0x01);  // CTRL2 register
  Wire.write(0x00);  // Clear TF (Timer Flag)
  Wire.endTransmission();
}
```

### 5. Chemie-spezifische Schwellen

```cpp
uint16_t InheroMr1Board::getVoltageCriticalThreshold() {
  BoardConfigContainer::BatteryType chemType = boardConfig.getBatteryType();
  
  switch (chemType) {
    case BoardConfigContainer::BatteryType::LTO_2S:
      return 4200;  // 4.2V for LTO 2S (200mV before 4.0V hardware cutoff)
    case BoardConfigContainer::BatteryType::LIFEPO4_1S:
      return 2900;  // 2.9V for LiFePO4 1S (100mV before 2.8V hardware cutoff)
    case BoardConfigContainer::BatteryType::LIION_1S:
    default:
      return 3400;  // 3.4V for Li-Ion 1S (200mV before 3.2V hardware cutoff)
  }
}

uint16_t InheroMr1Board::getVoltageWakeThreshold() {
  BoardConfigContainer::BatteryType chemType = boardConfig.getBatteryType();
  
  switch (chemType) {
    case BoardConfigContainer::BatteryType::LTO_2S:
      return 4400;  // 4.4V for LTO 2S (Normalbetrieb)
    case BoardConfigContainer::BatteryType::LIFEPO4_1S:
      return 3000;  // 3.0V for LiFePO4 1S (Normalbetrieb)
    case BoardConfigContainer::BatteryType::LIION_1S:
    default:
      return 3600;  // 3.6V for Li-Ion 1S (Normalbetrieb)
  }
}
```

### 6. Adaptive Voltage Monitoring Task

**Datei**: `VoltageMonitor_Implementation.cpp` (Referenz-Implementierung)

```cpp
// Monitoring states
typedef enum {
  VOLTAGE_STATE_NORMAL = 0,    // > warning threshold
  VOLTAGE_STATE_WARNING = 1,   // critical < V < warning
  VOLTAGE_STATE_CRITICAL = 2   // near critical threshold
} VoltageState_t;

void voltageMonitorTask(void* parameter) {
  VoltageState_t currentState = VOLTAGE_STATE_NORMAL;
  uint32_t checkInterval_ms = 60000;  // Start with 60s
  
  // Get chemistry-specific thresholds
  uint16_t critical_mv = InheroMr1Board::getVoltageCriticalThreshold();
  uint16_t warning_mv = critical_mv + 100;  // +100mV above critical
  uint16_t normal_mv = InheroMr1Board::getVoltageWakeThreshold();  // Normalbetrieb threshold
  
  while (true) {
    // Read battery voltage via BQ25798
    uint16_t vbat_mv = getBattMilliVolts();
    
    // Determine state and update interval
    VoltageState_t newState;
    if (vbat_mv < critical_mv) {
      // Below critical - initiate shutdown immediately
      MESH_DEBUG_PRINTLN("PWRMGT: Critical voltage %dmV < %dmV - shutting down", 
                         vbat_mv, critical_mv);
      InheroMr1Board::initiateShutdown(SHUTDOWN_REASON_LOW_VOLTAGE);
      // Never returns
    } else if (vbat_mv < warning_mv) {
      newState = VOLTAGE_STATE_CRITICAL;
      checkInterval_ms = 10000;  // Check every 10s
    } else if (vbat_mv < normal_mv) {
      newState = VOLTAGE_STATE_WARNING;
      checkInterval_ms = 30000;  // Check every 30s
    } else {
      newState = VOLTAGE_STATE_NORMAL;
      checkInterval_ms = 60000;  // Check every 60s
    }
    
    // State transition actions
    if (newState != currentState) {
      currentState = newState;
      
      if (currentState == VOLTAGE_STATE_WARNING) {
        MESH_DEBUG_PRINTLN("PWRMGT: Entering WARNING state (VBAT=%dmV)", vbat_mv);
        // TODO: Reduce TX power, disable BLE, etc.
      } else if (currentState == VOLTAGE_STATE_CRITICAL) {
        MESH_DEBUG_PRINTLN("PWRMGT: Entering CRITICAL state (VBAT=%dmV)", vbat_mv);
        // TODO: Minimal operation mode
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(checkInterval_ms));
  }
}

// In BoardConfigContainer::begin():
// xTaskCreate(voltageMonitorTask, "VoltMon", 2048, NULL, 2, &voltageMonitorTaskHandle);
```

---

## Hardware-Voraussetzungen (zu prüfen)

### RTC Verdrahtung
- **RTC**: RV-3028-C7 (I²C 0x52)
- **INT-Pin**: Laut Datasheet "verdrahtet, aber noch nicht in Software implementiert"
- **Annahme**: GPIO17 (WB_IO1) ist mit RTC INT verbunden
- **⚠️ ZU PRÜFEN**: Schematic prüfen, welcher Pin tatsächlich verdrahtet ist

### Pin-Konfiguration
```cpp
// Bekannte Pins (aus variant.h)
#define PIN_VBAT_READ 5    // ADC Pin (aber ohne Voltage Divider!)
#define WB_IO1 17          // Vermutlich RTC INT
```

### I²C-Bus
- **SDA**: PIN_BOARD_SDA
- **SCL**: PIN_BOARD_SCL
- **Devices**: BQ25798 (0x6B), MCP4652 (0x2F), RV-3028 (0x52)

---

## Test-Strategie

### Phase 1: RTC Hardware-Test (ohne Power Management)
```cpp
// Einfacher Test: RTC lesen/schreiben
void test_rtc_basic() {
  Wire.beginTransmission(0x52);
  Wire.write(0x00);  // Seconds register
  if (Wire.endTransmission() == 0) {
    Serial.println("RTC found at 0x52");
  } else {
    Serial.println("RTC NOT found!");
  }
}

// Countdown-Timer Test
void test_rtc_countdown() {
  // 10 Sekunden Timer
  Wire.beginTransmission(0x52);
  Wire.write(0x09);  // Countdown LSB
  Wire.write(10);    // 10 ticks at 1Hz
  Wire.write(0);     // MSB
  Wire.endTransmission();
  
  Wire.beginTransmission(0x52);
  Wire.write(0x00);  // CTRL1
  Wire.write(0x01);  // Enable timer
  Wire.endTransmission();
  
  Serial.println("Timer started - waiting 10s...");
  delay(10000);
  
  // Check timer flag
  Wire.beginTransmission(0x52);
  Wire.write(0x01);  // CTRL2
  Wire.endTransmission();
  Wire.requestFrom(0x52, 1);
  uint8_t ctrl2 = Wire.read();
  if (ctrl2 & 0x08) {
    Serial.println("Timer elapsed! ✓");
  }
}
```

### Phase 2: INT-Pin Test
```cpp
volatile bool rtc_int_fired = false;

void test_int_callback() {
  rtc_int_fired = true;
}

void test_rtc_interrupt() {
  pinMode(17, INPUT_PULLUP);  // Assumed INT pin
  attachInterrupt(digitalPinToInterrupt(17), test_int_callback, FALLING);
  
  // Start 5 second countdown
  Wire.beginTransmission(0x52);
  Wire.write(0x09);
  Wire.write(5);  // 5 ticks
  Wire.write(0);
  Wire.endTransmission();
  
  Wire.beginTransmission(0x52);
  Wire.write(0x01);
  Wire.write(0x80);  // Enable interrupt
  Wire.endTransmission();
  
  Wire.beginTransmission(0x52);
  Wire.write(0x00);
  Wire.write(0x01);  // Start timer
  Wire.endTransmission();
  
  Serial.println("Waiting for INT pin...");
  delay(6000);
  
  if (rtc_int_fired) {
    Serial.println("INT pin works! ✓");
  } else {
    Serial.println("INT pin not triggered - check wiring!");
  }
}
```

### Phase 3: SYSTEMOFF Wake-up Test
```cpp
void test_systemoff_wake() {
  Serial.println("Configuring 30-second RTC wake...");
  configureRTCWake(0);  // Custom: 30 seconds instead of hours
  
  NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_LOW_VOLTAGE;
  
  Serial.println("Entering SYSTEMOFF in 5s...");
  delay(5000);
  
  sd_power_system_off();
  // Device should wake after 30s and print wake message in begin()
}
```

### Phase 4: Voltage Monitoring Integration
1. Start voltage monitor task
2. Manuell VBAT reduzieren (Variablen setzen für Test)
3. Beobachten: State transitions, intervals, shutdown trigger

### Phase 5: End-to-End Test
1. Device mit niedriger Batterie betreiben
2. Shutdown bei 3.2V beobachten
3. RTC Wake nach 12h verifizieren
4. Voltage recovery check testen
5. Normal boot nach Recovery verifizieren

---

## Nächste Schritte

### Sofort (Design-Phase)
- ✅ Architektur dokumentiert
- ✅ Code vorbereitet
- ✅ Test-Strategie erstellt

### Nach Hardware-Lieferung
1. ⏳ **RTC Hardware-Test**: I²C Kommunikation, Countdown-Timer
2. ⏳ **INT-Pin Verdrahtung prüfen**: GPIO17 oder anderer Pin?
3. ⏳ **Interrupt Test**: FALLING edge auf INT-Pin
4. ⏳ **SYSTEMOFF Wake Test**: 30s countdown, wake-up beobachten
5. ⏳ **Power Management aktivieren**: Code in InheroMr1Board integrieren
6. ⏳ **Voltage Monitor Task starten**: In BoardConfigContainer::begin()
7. ⏳ **End-to-End Test**: Echter Battery drain Szenario
8. ⏳ **Load Shedding**: BLE disable, TX power reduction
9. ⏳ **CLI Commands**: Diagnostik und Manual testing
10. ⏳ **Field Test**: Solar-Panel, mehrere Tage Betrieb
