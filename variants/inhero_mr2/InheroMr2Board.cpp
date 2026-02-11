/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Includes
#include "InheroMr2Board.h"
#include "BoardConfigContainer.h"
#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>
#include <nrf_soc.h>

// Static declarations
static BLEDfu bledfu;
static BoardConfigContainer boardConfig;

// Static callback functions
static void connect_callback(uint16_t conn_handle) {
  (void)conn_handle;
  MESH_DEBUG_PRINTLN("BLE client connected");
}

static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;
  MESH_DEBUG_PRINTLN("BLE client disconnected");
}

// ===== Public Methods =====

void InheroMr2Board::begin() {
  pinMode(PIN_VBAT_READ, INPUT);

#ifdef PIN_USER_BTN
  pinMode(PIN_USER_BTN, INPUT_PULLUP);
#endif

#ifdef PIN_USER_BTN_ANA
  pinMode(PIN_USER_BTN_ANA, INPUT_PULLUP);
#endif

#if defined(PIN_BOARD_SDA) && defined(PIN_BOARD_SCL)
  Wire.setPins(PIN_BOARD_SDA, PIN_BOARD_SCL);
#endif

  Wire.begin();
  delay(50);  // Give I2C bus time to stabilize

  // MR2 is v0.2 hardware only - no detection needed
  MESH_DEBUG_PRINTLN("Inhero MR2 - Hardware v0.2 (INA228 + RTC)");
  
  // Initialize board configuration (BQ25798, INA228, etc.)
  // Note: voltageMonitorTask will check Danger Zone within 10s after boot
  boardConfig.begin();
  
  // === v0.2 hardware initialization ===
  MESH_DEBUG_PRINTLN("Initializing v0.2 features (RTC, INA228 alerts)");
    
    // === CRITICAL: Configure RTC INT pin for wake-up from SYSTEMOFF ===
    // attachInterrupt() alone is NOT sufficient for SYSTEMOFF wake-up!
    // We MUST configure the pin with SENSE for nRF52 SYSTEMOFF wake capability
    pinMode(RTC_INT_PIN, INPUT_PULLUP);
    
    // Configure GPIO SENSE for wake-up from SYSTEM OFF mode
    // This is essential - without SENSE configuration, SYSTEMOFF wake-up will not work
    NRF_GPIO->PIN_CNF[RTC_INT_PIN] = 
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos);  // Wake on LOW (RTC interrupt is active-low)
    
    attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), rtcInterruptHandler, FALLING);
    
    // === CRITICAL: Early Boot Voltage Check ===
    // Prevents motorboating after Hardware-UVLO or Software-SHUTDOWN
    // When INA228 Alert cuts power via TPS62840 EN, RAK loses all RAM (GPREGRET2=0x00)
    // We must check voltage on EVERY ColdBoot, not just after Software-SHUTDOWN
    
    uint8_t shutdown_reason = NRF_POWER->GPREGRET2;  // Read full register for Early Boot check
    
    // === High-Precision INA228 voltage measurement (24-bit ADC, ±0.1% accuracy) ===
    // RAK4630 cannot measure battery voltage - there's no voltage divider on GPIO!
    // Use static INA228 method with One-Shot ADC for fresh, accurate measurement
    // This is critical for wake/sleep decisions in danger zone
    MESH_DEBUG_PRINTLN("Early Boot: Reading VBAT from INA228 @ 0x40...");
    uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, 0x40);
    MESH_DEBUG_PRINTLN("Early Boot: readVBATDirect returned %dmV", vbat_mv);
    
    // CRITICAL: readVBATDirect() sets ADC_CONFIG to 0x1000 (Single-Shot mode)
    // We must restore it to 0xF003 (Continuous mode, 64 samples avg) for normal operation
    Wire.beginTransmission(0x40);
    Wire.write(0x01);  // ADC_CONFIG register
    Wire.write(0xF0);  // MSB: Continuous all channels (0xF)
    Wire.write(0x03);  // LSB: 64 samples averaging (0x3) - consistent with ina228.begin()
    uint8_t i2c_result = Wire.endTransmission();
    delay(50);  // Longer delay for I2C bus to stabilize before BQ init
    MESH_DEBUG_PRINTLN("Early Boot: ADC_CONFIG restored to 0xF003 (result=%d)", i2c_result);
    
    if (vbat_mv == 0) {
      MESH_DEBUG_PRINTLN("Early Boot: Failed to read battery voltage, assuming OK");
      // Continue boot if we can't read voltage (better than blocking)
    } else {
      uint16_t critical_threshold = getVoltageCriticalThreshold();
      uint16_t uvlo_threshold = getVoltageHardwareCutoff();
      
      MESH_DEBUG_PRINTLN("Early Boot Check: VBAT=%dmV, Critical=%dmV (0%% SOC), UVLO=%dmV, Reason=0x%02X", 
                         vbat_mv, critical_threshold, uvlo_threshold, shutdown_reason);
      
      // Case 1: Waking from Software-SHUTDOWN (mask out upper bits, only check SHUTDOWN_REASON)
      if ((shutdown_reason & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE) {
        MESH_DEBUG_PRINTLN("Detected RTC wake from software shutdown");
        
        // Visual indication: 3x fast blue blink to show RTC wake-up
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_BLUE, HIGH);
          delay(150);
          digitalWrite(LED_BLUE, LOW);
          delay(150);
        }
        
        if (vbat_mv < critical_threshold) {
          MESH_DEBUG_PRINTLN("Voltage still in danger zone (%dmV < %dmV)", vbat_mv, critical_threshold);
          MESH_DEBUG_PRINTLN("Will enter low-power mode after boot completes");
          delay(100);
          
          // Do NOT send sleep command here - RadioLib not yet initialized!
          // The voltageMonitorTask will handle sleep after full system init
          // Just preserve the flags and continue boot
          
          // CRITICAL: Preserve GPREGRET2_IN_DANGER_ZONE flag!
          // Only clear lower bits (SHUTDOWN_REASON) for clean state
          NRF_POWER->GPREGRET2 = (NRF_POWER->GPREGRET2 & 0xFC) | SHUTDOWN_REASON_LOW_VOLTAGE;
          
          // Configure RTC to wake in 1 minute, then shutdown
          // voltageMonitorTask will send sleep command on next boot after init completes
          configureRTCWake(1);
          sd_power_system_off();
          // Never returns
        }
        
        // Voltage recovered above critical threshold - THIS IS 0% SOC!
        MESH_DEBUG_PRINTLN("Voltage recovered to %dmV, resuming normal operation at 0%% SOC", vbat_mv);
        // Clear ALL flags including Danger Zone - we're exiting!
        NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
        
        // Start reverse learning (Method 2: 0% → 100% via USB-C charging)
        // This is the perfect moment: we just exited danger zone = 0% SOC
        // User will plug in USB-C, we accumulate all charged mAh to 100%
        boardConfig.startReverseLearning();
      }
      // Case 2: ColdBoot after Hardware-UVLO (GPREGRET2 may have Danger Zone flag from previous session)
      // This is the critical case to prevent motorboating!
      else if (vbat_mv < critical_threshold) {
        MESH_DEBUG_PRINTLN("ColdBoot in danger zone detected (%dmV < %dmV)", vbat_mv, critical_threshold);
        MESH_DEBUG_PRINTLN("Likely Hardware-UVLO recovery at %dmV - voltage not stable yet", uvlo_threshold);
        MESH_DEBUG_PRINTLN("Going to sleep for 1min to avoid motorboating");
        
        delay(100);
        
        // Do NOT send sleep command here - would cause race condition with bootup
        // voltageMonitorTask will handle sleep after full system initialization
        
        // Configure RTC wake-up before shutdown
        configureRTCWake(1);  // Wake up in 1 minute (TEST MODE)
        
        // Store reason AND set Danger Zone flag (this was ColdBoot, no flag set yet)
        NRF_POWER->GPREGRET2 = GPREGRET2_IN_DANGER_ZONE | SHUTDOWN_REASON_LOW_VOLTAGE;
        
        sd_power_system_off();
        // Never returns
      }
      // Case 3: Normal ColdBoot (Power-On, Reset button, firmware update, voltage OK)
      else {
        MESH_DEBUG_PRINTLN("Normal ColdBoot - voltage OK (%dmV >= %dmV)", vbat_mv, critical_threshold);
      }
    }
  
  // Enable DC/DC converter for improved power efficiency
  // Done after peripheral initialization to avoid voltage glitches
  NRF52BoardDCDC::begin();
  
  // LEDs already initialized in boardConfig.begin()
  // Blue LED was used for boot sequence visualization
  // Red LED indicates missing components (if blinking)

  // Danger Zone status was already checked earlier in begin()
  // SX1262 sleep state is now managed synchronously on boot before RadioLib init
  
  // Start hardware watchdog (600s timeout)
  // Must be last - after all initializations are complete
  BoardConfigContainer::setupWatchdog();
}

void InheroMr2Board::tick() {
  // Feed watchdog to prevent system reset
  // This ensures the main loop is running properly
  BoardConfigContainer::feedWatchdog();
}

uint16_t InheroMr2Board::getBattMilliVolts() {
  const Telemetry* telemetry = boardConfig.getTelemetryData();

  return telemetry->batterie.voltage;
}

bool InheroMr2Board::startOTAUpdate(const char* id, char reply[]) {
  // Note: 600s watchdog timeout allows OTA to complete (typically 2-5 min)
  // No need to disable watchdog as timeout is sufficient
  
  // Stop all background tasks and clean up peripherals before OTA
  // This is critical - without stopping tasks, OTA will fail
  BoardConfigContainer::stopBackgroundTasks();
  
  // Use standard NRF52Board OTA implementation (same as RAK4631)
  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.configPrphConn(92, BLE_GAP_EVENT_LENGTH_MIN, 16, 16);

  Bluefruit.begin(1, 0);
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(4);
  // Set the BLE device name
  Bluefruit.setName("InheroMR2_OTA");

  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Set up and start advertising
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();

  /* Start Advertising
    - Enable auto advertising if disconnected
    - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
    - Timeout for fast mode is 30 seconds
    - Start(timeout) with timeout = 0 will advertise forever (until connected)

    For recommended advertising interval
    https://developer.apple.com/library/content/qa/qa1931/_index.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds

  uint8_t mac_addr[6];
  memset(mac_addr, 0, sizeof(mac_addr));
  Bluefruit.getAddr(mac_addr);
  sprintf(reply, "OK - mac: %02X:%02X:%02X:%02X:%02X:%02X", mac_addr[5], mac_addr[4], mac_addr[3],
          mac_addr[2], mac_addr[1], mac_addr[0]);

  return true;
}

/// @brief Collects board telemetry and appends to CayenneLPP packet
/// @param telemetry CayenneLPP packet to append data to
/// @return true if successful, false if telemetry data unavailable
bool InheroMr2Board::queryBoardTelemetry(CayenneLPP& telemetry) {
  const Telemetry* telemetryData = boardConfig.getTelemetryData();
  if (!telemetryData) {
    return false;
  }

  uint8_t channel = this->findNextFreeChannel(telemetry);
  
  // Current solar telemetry
  telemetry.addVoltage(channel, telemetryData->solar.voltage / 1000.0f);
  telemetry.addCurrent(channel, telemetryData->solar.current / 1000.0f);
  telemetry.addDigitalInput(channel, telemetryData->solar.mppt);

  channel++;
  // Battery telemetry
  telemetry.addVoltage(channel, telemetryData->batterie.voltage / 1000.0f);
  telemetry.addCurrent(channel, telemetryData->batterie.current / 1000.0f);
  telemetry.addTemperature(channel, telemetryData->batterie.temperature);
  
  channel++;
  // MPPT statistics (separate channel)
  telemetry.addPercentage(channel, boardConfig.getMpptEnabledPercentage7Day());
  telemetry.addAnalogInput(channel, boardConfig.getAvgDailyEnergy3Day()); // 3-day avg daily solar energy in mWh

  return true;
}

/// @brief Handles custom CLI getter commands for board configuration
/// @param getCommand Command string (without "board." prefix)
/// @param reply Buffer to write response to
/// @param maxlen Maximum length of reply buffer
/// @return true if command was handled, false otherwise
bool InheroMr2Board::getCustomGetter(const char* getCommand, char* reply, uint32_t maxlen) {

  // Trim trailing whitespace from command
  char trimmedCommand[100];
  strncpy(trimmedCommand, getCommand, sizeof(trimmedCommand) - 1);
  trimmedCommand[sizeof(trimmedCommand) - 1] = '\0';
  char* cmd = BoardConfigContainer::trim(trimmedCommand);

  if (strcmp(cmd, "bat") == 0) {
    snprintf(reply, maxlen, "%s", BoardConfigContainer::getBatteryTypeCommandString(boardConfig.getBatteryType()));
    return true;
  } else if (strcmp(cmd, "hwver") == 0) {
    // MR2 is always v0.2 hardware
    snprintf(reply, maxlen, "v0.2 (INA228+RTC)");
    return true;
  } else if (strcmp(cmd, "frost") == 0) {
    // LTO batteries ignore JEITA temperature control
    if (boardConfig.getBatteryType() == BoardConfigContainer::BatteryType::LTO_2S) {
      snprintf(reply, maxlen, "N/A");
    } else {
      snprintf(
          reply, maxlen, "%s",
          BoardConfigContainer::getFrostChargeBehaviourCommandString(boardConfig.getFrostChargeBehaviour()));
    }
    return true;
  } else if (strcmp(cmd, "life") == 0) {
    snprintf(reply, maxlen, "%s", boardConfig.getReduceChargeVoltage() ? "1" : "0");
    return true;
  } else if (strcmp(cmd, "imax") == 0) {
    snprintf(reply, maxlen, "%s", boardConfig.getChargeCurrentAsStr());
    return true;
  } else if (strcmp(cmd, "mppt") == 0) {
    snprintf(reply, maxlen, "MPPT=%s", boardConfig.getMPPTEnabled() ? "1" : "0");
    return true;
  } else if (strcmp(cmd, "mpps") == 0) {
    boardConfig.getMpptStatsString(reply, maxlen);
    return true;
  } else if (strcmp(cmd, "cinfo") == 0) {
    char infoBuffer[100];
    boardConfig.getChargerInfo(infoBuffer, sizeof(infoBuffer));
    snprintf(reply, maxlen, "%s", infoBuffer);
    return true;
  } else if (strcmp(cmd, "diag") == 0) {
    // Detailed diagnostics for debugging charging issues
    char diagBuffer[256];
    boardConfig.getDetailedDiagnostics(diagBuffer, sizeof(diagBuffer));
    snprintf(reply, maxlen, "%s", diagBuffer);
    return true;
  } else if (strcmp(cmd, "togglehiz") == 0) {
    // Manual HIZ toggle for debugging stuck PGOOD
    char hizBuffer[100];
    boardConfig.toggleHizAndCheck(hizBuffer, sizeof(hizBuffer));
    snprintf(reply, maxlen, "%s", hizBuffer);
    return true;
  } else if (strcmp(cmd, "clearhiz") == 0) {
    // Force clear HIZ mode
    char hizBuffer[100];
    boardConfig.clearHiz(hizBuffer, sizeof(hizBuffer));
    snprintf(reply, maxlen, "%s", hizBuffer);
    return true;
  } else if (strcmp(cmd, "telem") == 0) {
    const Telemetry* telemetry = boardConfig.getTelemetryData();
    
    // Get high-precision current directly from INA228
    float precise_current_ma = 0.0f;
    Ina228Driver* ina = boardConfig.getIna228Driver();
    if (ina != nullptr) {
      precise_current_ma = ina->readCurrent_mA_precise();
    } else {
      precise_current_ma = (float)telemetry->batterie.current;
    }
    
    snprintf(reply, maxlen, "B:%.2fV/%.3fmA/%.0fC S:%.2fV/%imA",
             telemetry->batterie.voltage / 1000.0f, precise_current_ma,
             telemetry->batterie.temperature,
             telemetry->solar.voltage / 1000.0f, telemetry->solar.current);
    return true;
  } else if (strcmp(cmd, "soc") == 0) {
    // Battery State of Charge (v0.2 feature)
    char socBuffer[80];
    boardConfig.getBatterySOCString(socBuffer, sizeof(socBuffer));
    snprintf(reply, maxlen, "%s", socBuffer);
    return true;
  } else if (strcmp(cmd, "balance") == 0) {
    // Daily balance and forecast (v0.2 feature)
    char balanceBuffer[100];
    boardConfig.getDailyBalanceString(balanceBuffer, sizeof(balanceBuffer));
    uint16_t ttl = boardConfig.getTTL_Hours();
    if (ttl > 0) {
      snprintf(reply, maxlen, "%s TTL:%dh", balanceBuffer, ttl);
    } else {
      snprintf(reply, maxlen, "%s", balanceBuffer);
    }
    return true;
  } else if (strcmp(cmd, "conf") == 0) {
    // Display all configuration values
    const char* batType = BoardConfigContainer::getBatteryTypeCommandString(boardConfig.getBatteryType());
    const char* frostBehaviour;
    if (boardConfig.getBatteryType() == BoardConfigContainer::BatteryType::LTO_2S) {
      frostBehaviour = "N/A";
    } else {
      frostBehaviour = BoardConfigContainer::getFrostChargeBehaviourCommandString(boardConfig.getFrostChargeBehaviour());
    }
    float chargeVoltage = boardConfig.getMaxChargeVoltage();
    const char* imax = boardConfig.getChargeCurrentAsStr();
    bool mpptEnabled = boardConfig.getMPPTEnabled();
    
    snprintf(reply, maxlen, "B:%s F:%s M:%s I:%s Vco:%.2f",
             batType, frostBehaviour, mpptEnabled ? "1" : "0", imax, chargeVoltage);
    return true;
  } else if (strcmp(cmd, "wdtstatus") == 0) {
    #ifndef DEBUG_MODE
      snprintf(reply, maxlen, "WDT: enabled (600s timeout)");
    #else
      snprintf(reply, maxlen, "WDT: disabled (DEBUG_MODE)");
    #endif
    return true;
  } else if (strcmp(cmd, "ibcal") == 0) {
    // Get current INA228 calibration factor
    float factor = boardConfig.getIna228CalibrationFactor();
    snprintf(reply, maxlen, "INA228 calibration: %.4f (1.0=default)", factor);
    return true;
  } else if (strcmp(cmd, "learning") == 0) {
    // Get auto-learning status (both methods)
    float capacity = boardConfig.getBatteryCapacity();
    const char* source = boardConfig.isCapacityLearned() ? "learned" : "manual/default";
    bool method1_active = boardConfig.isLearningActive();
    
    // Check Method 2 (reverse learning) via socStats
    const BatterySOCStats* socStats = boardConfig.getSOCStats();
    bool method2_active = (socStats != nullptr) ? socStats->reverse_learning_active : false;
    
    if (method1_active) {
      float accumulated = boardConfig.getLearningAccumulatedMah();
      snprintf(reply, maxlen, "M1 ACTIVE (100%%→10%%, %.0f mAh) Cap:%.0f mAh (%s)", 
               accumulated, capacity, source);
    } else if (method2_active) {
      float accumulated = socStats->reverse_learning_accumulated_mah;
      snprintf(reply, maxlen, "M2 ACTIVE (0%%→100%%, %.0f mAh) Cap:%.0f mAh (%s)", 
               accumulated, capacity, source);
    } else {
      snprintf(reply, maxlen, "Learning IDLE Cap:%.0f mAh (%s)", capacity, source);
    }
    return true;
  } else if (strcmp(cmd, "relearn") == 0) {
    // Reset learned flag to enable auto-learning again
    boardConfig.resetLearning();
    snprintf(reply, maxlen, "Learning reset - auto-learning enabled");
    return true;
  } else if (strcmp(cmd, "leds") == 0) {
    // Get LED enable state (heartbeat + BQ stat LED)
    bool enabled = boardConfig.getLEDsEnabled();
    snprintf(reply, maxlen, "LEDs: %s (Heartbeat + BQ Stat)", enabled ? "ON" : "OFF");
    return true;
  } else if (strcmp(cmd, "i2cscan") == 0) {
    // I2C bus scan - find all devices
    char scanBuffer[200];
    int pos = 0;
    int devicesFound = 0;
    
    pos += snprintf(scanBuffer + pos, sizeof(scanBuffer) - pos, "I2C: ");
    
    for (uint8_t addr = 1; addr < 127; addr++) {
      Wire.beginTransmission(addr);
      if (Wire.endTransmission() == 0) {
        if (devicesFound > 0 && pos < sizeof(scanBuffer) - 10) {
          pos += snprintf(scanBuffer + pos, sizeof(scanBuffer) - pos, ", ");
        }
        if (pos < sizeof(scanBuffer) - 10) {
          pos += snprintf(scanBuffer + pos, sizeof(scanBuffer) - pos, "0x%02X", addr);
        }
        devicesFound++;
      }
    }
    
    if (devicesFound == 0) {
      snprintf(reply, maxlen, "I2C: No devices found!");
    } else {
      snprintf(reply, maxlen, "%s (%d found)", scanBuffer, devicesFound);
    }
    return true;
  } else if (strcmp(cmd, "inatest") == 0) {
    // Detailed INA228 diagnostic test - Line 1: Device Info + Voltage
    char line1[150];
    char line2[150];
    int pos = 0;
    
    // Test address 0x40
    Wire.beginTransmission(0x40);
    uint8_t ack = Wire.endTransmission();
    pos += snprintf(line1 + pos, sizeof(line1) - pos, "ACK:%d ", ack);
    
    if (ack == 0) {
      // Read MFG_ID
      Wire.beginTransmission(0x40);
      Wire.write(0x3E);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
      uint16_t mfg = 0;
      if (Wire.available() >= 2) {
        mfg = (Wire.read() << 8) | Wire.read();
        pos += snprintf(line1 + pos, sizeof(line1) - pos, "MFG:0x%04X ", mfg);
      }
      
      // Read DEV_ID
      Wire.beginTransmission(0x40);
      Wire.write(0x3F);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
      uint16_t dev = 0;
      if (Wire.available() >= 2) {
        dev = (Wire.read() << 8) | Wire.read();
        pos += snprintf(line1 + pos, sizeof(line1) - pos, "DEV:0x%04X ", dev);
      }
      
      // Read VBUS raw
      Wire.beginTransmission(0x40);
      Wire.write(0x05);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x40, (uint8_t)3);
      if (Wire.available() >= 3) {
        uint8_t b0 = Wire.read();
        uint8_t b1 = Wire.read();
        uint8_t b2 = Wire.read();
        int32_t raw = b0 << 16 | b1 << 8 | b2;
        if (raw & 0x800000) raw |= 0xFF000000;
        int32_t raw_shifted = raw >> 4;
        float v_calc = raw_shifted * 195.3125e-6 * 1000.0f;
        pos += snprintf(line1 + pos, sizeof(line1) - pos, "VBUS:%dmV", (int)v_calc);
      }
      
      // Line 2: Current, ADC Config, SHUNT_CAL
      int pos2 = 0;
      
      // Read CURRENT register (0x07)
      Wire.beginTransmission(0x40);
      Wire.write(0x07);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x40, (uint8_t)3);
      if (Wire.available() >= 3) {
        uint8_t b0 = Wire.read();
        uint8_t b1 = Wire.read();
        uint8_t b2 = Wire.read();
        int32_t raw = b0 << 16 | b1 << 8 | b2;
        if (raw & 0x800000) raw |= 0xFF000000;
        int32_t raw_shifted = raw >> 4;
        // CURRENT_LSB = 1.91 µA for our config
        float current_ma = raw_shifted * 1.91e-6 * 1000.0f;
        pos2 += snprintf(line2 + pos2, sizeof(line2) - pos2, "I:0x%05X(%.3fmA) ", raw_shifted & 0xFFFFF, current_ma);
      }
      
      // Read ADC_CONFIG (0x01)
      Wire.beginTransmission(0x40);
      Wire.write(0x01);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
      if (Wire.available() >= 2) {
        uint16_t adc_cfg = (Wire.read() << 8) | Wire.read();
        pos2 += snprintf(line2 + pos2, sizeof(line2) - pos2, "ADC:0x%04X ", adc_cfg);
      }
      
      // Read SHUNT_CAL (0x02)
      Wire.beginTransmission(0x40);
      Wire.write(0x02);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
      if (Wire.available() >= 2) {
        uint16_t shunt_cal = (Wire.read() << 8) | Wire.read();
        pos2 += snprintf(line2 + pos2, sizeof(line2) - pos2, "CAL:0x%04X", shunt_cal);
      }
      
      // Output both lines (newline between them for readability)
      snprintf(reply, maxlen, "%s\n%s", line1, line2);
    } else {
      snprintf(reply, maxlen, "INA228 not detected (ACK=%d)", ack);
    }
    return true;
  } else if (strcmp(cmd, "inafix") == 0) {
    // Force-fix INA228 ADC_CONFIG register to continuous mode
    // This is a diagnostic command to test if writes work
    Wire.beginTransmission(0x40);
    uint8_t ack = Wire.endTransmission();
    
    if (ack == 0) {
      // Try to write ADC_CONFIG = 0xF002 (Continuous All + 16avg)
      Wire.beginTransmission(0x40);
      Wire.write(0x01);  // ADC_CONFIG register
      Wire.write(0xF0);  // MSB
      Wire.write(0x02);  // LSB
      uint8_t write_result = Wire.endTransmission();
      
      delay(10);
      
      // Read back ADC_CONFIG
      Wire.beginTransmission(0x40);
      Wire.write(0x01);
      Wire.endTransmission(false);
      Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
      uint16_t adc_readback = 0;
      if (Wire.available() >= 2) {
        adc_readback = (Wire.read() << 8) | Wire.read();
      }
      
      snprintf(reply, maxlen, "Write:0xF002 result=%d Readback:0x%04X %s", 
               write_result, adc_readback, 
               (adc_readback == 0xF002) ? "OK" : "FAIL");
    } else {
      snprintf(reply, maxlen, "INA228 not detected (ACK=%d)", ack);
    }
    return true;
  }

  snprintf(reply, maxlen, "Err: Try board.<bat|frost|life|imax|telem|cinfo|diag|togglehiz|clearhiz|mppt|mpps|conf|wdtstatus|ibcal|learning|relearn|leds|i2cscan|inatest|inafix>");
  return true;
}

/// @brief Handles custom CLI setter commands for board configuration
/// @param setCommand Command string with value (without "board." prefix)
/// @return Status message ("OK" on success, error message on failure)
const char* InheroMr2Board::setCustomSetter(const char* setCommand) {

  static char ret[100];
  memset(ret, 0, sizeof(ret));  // Clear buffer to prevent garbage data

  if (strncmp(setCommand, "bat ", 4) == 0) {
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[4]));
    BoardConfigContainer::BatteryType bt = BoardConfigContainer::getBatteryTypeFromCommandString(value);
    if (bt != BoardConfigContainer::BatteryType::BAT_UNKNOWN) {
      boardConfig.setBatteryType(bt);
      snprintf(ret, sizeof(ret), "Bat set to %s", BoardConfigContainer::getBatteryTypeCommandString(boardConfig.getBatteryType()));
      return ret;
    } else {
      snprintf(ret, sizeof(ret), "Err: Try one of: %s", BoardConfigContainer::getAvailableBatOptions());
      return ret;
    }
  } else if (strncmp(setCommand, "frost ", 6) == 0) {
    // LTO batteries ignore JEITA temperature control - setting frost behavior is not applicable
    if (boardConfig.getBatteryType() == BoardConfigContainer::BatteryType::LTO_2S) {
      snprintf(ret, sizeof(ret), "Err: Frost setting N/A for LTO (JEITA disabled)");
      return ret;
    }
    
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[6]));
    BoardConfigContainer::FrostChargeBehaviour fcb =
        BoardConfigContainer::getFrostChargeBehaviourFromCommandString(value);
    if (fcb != BoardConfigContainer::FrostChargeBehaviour::REDUCE_UNKNOWN) {
      boardConfig.setFrostChargeBehaviour(fcb);
      snprintf(ret, sizeof(ret), "Frost charge current set to %s of imax",
               BoardConfigContainer::getFrostChargeBehaviourCommandString(boardConfig.getFrostChargeBehaviour()));
      return ret;
    } else {
      snprintf(ret, sizeof(ret), "Err: Try one of: %s",
               BoardConfigContainer::getAvailableFrostChargeBehaviourOptions());
      return ret;
    }
  } else if (strncmp(setCommand, "life ", 5) == 0) {
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    // Convert to lowercase for case-insensitive comparison
    char lowerValue[20];
    strncpy(lowerValue, value, sizeof(lowerValue) - 1);
    lowerValue[sizeof(lowerValue) - 1] = '\0';
    for (char* p = lowerValue; *p; ++p) *p = tolower(*p);
    
    if (strcmp(lowerValue, "true") == 0 || strcmp(lowerValue, "1") == 0) {
      boardConfig.setReducedChargeVoltage(true);
      snprintf(ret, sizeof(ret), "Charge cutoff voltage changed to %.2fV", boardConfig.getMaxChargeVoltage());
      return ret;
    } else if (strcmp(lowerValue, "false") == 0 || strcmp(lowerValue, "0") == 0) {
      boardConfig.setReducedChargeVoltage(false);
      snprintf(ret, sizeof(ret), "Charge cutoff voltage changed to %.2fV", boardConfig.getMaxChargeVoltage());
      return ret;
    } else {
      return "Err: Try true|false or 1|0";
    }
  } else if (strncmp(setCommand, "imax ", 5) == 0) {
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    int ma = atoi(value);
    if (ma >= 10 && ma <= 1000) {
      boardConfig.setMaxChargeCurrent_mA(ma);
      snprintf(ret, sizeof(ret), "Max charge current set to %s", boardConfig.getChargeCurrentAsStr());
      return ret;
    } else {
      return "Err: Try 10-1000";
    }
  } else if (strncmp(setCommand, "mppt ", 5) == 0) {
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    // Convert to lowercase for case-insensitive comparison
    char lowerValue[20];
    strncpy(lowerValue, value, sizeof(lowerValue) - 1);
    lowerValue[sizeof(lowerValue) - 1] = '\0';
    for (char* p = lowerValue; *p; ++p) *p = tolower(*p);
    
    if (strcmp(lowerValue, "true") == 0 || strcmp(lowerValue, "1") == 0) {
      boardConfig.setMPPTEnable(true);
      snprintf(ret, sizeof(ret), "MPPT enabled");
      return ret;
    } else if (strcmp(lowerValue, "false") == 0 || strcmp(lowerValue, "0") == 0) {
      boardConfig.setMPPTEnable(false);
      snprintf(ret, sizeof(ret), "MPPT disabled");
      return ret;
    } else {
      return "Err: Try true|false or 1|0";
    }
  } else if (strncmp(setCommand, "batcap ", 7) == 0) {
    // Set battery capacity (v0.2 feature)
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[7]));
    float capacity_mah = atof(value);
    
    if (boardConfig.setBatteryCapacity(capacity_mah)) {
      snprintf(ret, sizeof(ret), "Battery capacity set to %.0f mAh", capacity_mah);
    } else {
      snprintf(ret, sizeof(ret), "Err: Invalid capacity (100-100000 mAh)");
    }
    return ret;
  } else if (strncmp(setCommand, "ibcal ", 6) == 0) {
    // INA228 current calibration: set board.ibcal <actual_current_mA>
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[6]));
    float actual_current_ma = atof(value);
    
    // Validate reasonable current range (-2000 to +2000 mA)
    if (actual_current_ma < -2000.0f || actual_current_ma > 2000.0f) {
      snprintf(ret, sizeof(ret), "Err: Current out of range (-2000 to +2000 mA)");
      return ret;
    }
    
    // Perform calibration and store factor
    float new_factor = boardConfig.performIna228Calibration(actual_current_ma);
    
    if (new_factor > 0.0f) {
      snprintf(ret, sizeof(ret), "INA228 calibrated: factor=%.4f", new_factor);
    } else {
      snprintf(ret, sizeof(ret), "Err: Calibration failed (zero current?)");
    }
    return ret;
  } else if (strcmp(setCommand, "bqreset") == 0) {
    // Perform BQ25798 software reset and reload config from FS
    bool success = boardConfig.resetBQ();
    if (success) {
      snprintf(ret, sizeof(ret), "BQ25798 reset done - reconfigured from FS");
    } else {
      snprintf(ret, sizeof(ret), "Err: BQ reset failed");
    }
    return ret;
  } else if (strncmp(setCommand, "leds ", 5) == 0) {
    // Enable/disable heartbeat LED and BQ stat LED
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    bool enabled = (strcmp(value, "1") == 0 || strcmp(value, "on") == 0 || strcmp(value, "ON") == 0);
    bool disabled = (strcmp(value, "0") == 0 || strcmp(value, "off") == 0 || strcmp(value, "OFF") == 0);
    
    if (enabled || disabled) {
      boardConfig.setLEDsEnabled(enabled);
      snprintf(ret, sizeof(ret), "LEDs %s (Heartbeat + BQ Stat)", enabled ? "enabled" : "disabled");
      return ret;
    } else {
      snprintf(ret, sizeof(ret), "Err: Use 'on/1' or 'off/0'");
      return ret;
    }
  }

  snprintf(ret, sizeof(ret), "Err: Try board.<bat|imax|life|frost|mppt|batcap|ibcal|bqreset|leds>");
  return ret;
}

// ===== Power Management Methods (v0.2) =====

/// @brief Get voltage threshold for critical software shutdown (chemistry-specific)
/// @return Threshold in millivolts - Danger zone boundary and 0% SOC point
uint16_t InheroMr2Board::getVoltageCriticalThreshold() {
  BoardConfigContainer::BatteryType chemType = boardConfig.getBatteryType();
  return BoardConfigContainer::getVoltageCriticalThreshold(chemType);
}

/// @brief Get hardware UVLO voltage cutoff (chemistry-specific)
/// @return Hardware cutoff voltage in millivolts (INA228 Alert threshold)
uint16_t InheroMr2Board::getVoltageHardwareCutoff() {
  BoardConfigContainer::BatteryType chemType = boardConfig.getBatteryType();
  return BoardConfigContainer::getVoltageHardwareCutoff(chemType);
}

/// @brief Initiate controlled shutdown with filesystem protection (v0.2)
/// @param reason Shutdown reason code (stored in GPREGRET2 for next boot)
void InheroMr2Board::initiateShutdown(uint8_t reason) {
  MESH_DEBUG_PRINTLN("PWRMGT: Initiating shutdown (reason=0x%02X)", reason);
  
  // 1. Stop background tasks to prevent filesystem corruption
  BoardConfigContainer::stopBackgroundTasks();
  
  // 2. Put INA228 into shutdown mode (v0.2 hardware)
  // No need for Coulomb counting - we assume 0% SOC in danger zone
  if (boardConfig.getIna228Driver() != nullptr) {
    MESH_DEBUG_PRINTLN("PWRMGT: Shutting down INA228");
    boardConfig.getIna228Driver()->shutdown();
  }
  
  if (reason == SHUTDOWN_REASON_LOW_VOLTAGE) {
    MESH_DEBUG_PRINTLN("PWRMGT: Low voltage shutdown - syncing filesystem");
    
    // TODO: Explicit filesystem sync when LittleFS is integrated
    // For now, stopBackgroundTasks() should flush pending writes
    delay(100);  // Allow I/O to complete
    
    // 3. Configure RTC to wake us up in 1 hour
    configureRTCWake(1);
  }
  
  // 4. Store shutdown reason for next boot
  NRF_POWER->GPREGRET2 = reason;
  
  // 5. Enter SYSTEMOFF mode (1-5 µA)
  MESH_DEBUG_PRINTLN("PWRMGT: Entering SYSTEMOFF");
  delay(50);  // Let debug output complete
  
  sd_power_system_off();
  // Never returns
}

/// @brief Configure RV-3028 RTC countdown timer for periodic wake-up (v0.2)
/// @param hours Wake-up interval in hours (typically 1 = hourly checks)
/// @note TEST MODE: Using 1 minute (60 seconds) for lab testing
void InheroMr2Board::configureRTCWake(uint32_t hours) {
  // TEST MODE: Override to 1 minute instead of hours
  uint16_t countdown = 60;  // 1 minute for testing (normally: hours * 3600)
  MESH_DEBUG_PRINTLN("PWRMGT: Configuring RTC wake in 1 minute (TEST MODE)");
  
  // === RTC Timer Configuration per Manual Section 4.8.2 ===
  // Step 1: Stop Timer and clear flags
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL1);
  Wire.write(0x00);  // TE=0, TD=00 (stop timer)
  Wire.endTransmission();
  
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL2);
  Wire.write(0x00);  // TIE=0 (disable interrupt)
  Wire.endTransmission();
  
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_STATUS);
  Wire.write(0x00);  // Clear TF flag
  Wire.endTransmission();
  
  // Step 2: Set Timer Value (60 seconds)
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_TIMER_VALUE_0);
  Wire.write(countdown & 0xFF);        // Lower 8 bits
  Wire.write((countdown >> 8) & 0x0F); // Upper 4 bits
  Wire.endTransmission();
  
  // Step 3: Configure Timer (1 Hz clock, Single shot mode)
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL1);
  Wire.write(0x06);  // TE=1 (Enable), TD=10 (1 Hz), TRPT=0 (Single shot)
  Wire.endTransmission();
  
  // Step 4: Enable Timer Interrupt
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL2);
  Wire.write(0x10);  // TIE=1 (Timer Interrupt Enable, bit 4)
  Wire.endTransmission();
  
  MESH_DEBUG_PRINTLN("PWRMGT: RTC countdown configured (%d seconds at 1 Hz)", countdown);
}

/// @brief RTC interrupt handler - called when countdown timer expires (v0.2)
void InheroMr2Board::rtcInterruptHandler() {
  // RTC countdown elapsed - device woke from SYSTEMOFF
  // Clear Timer Flag (TF) bit in STATUS register to release INT pin
  
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_STATUS);
  Wire.endTransmission(false);
  Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)1);
  
  if (Wire.available()) {
    uint8_t status = Wire.read();
    
    // Clear TF bit (bit 3) by writing 0 to it
    status &= ~(1 << 3);  // Clear bit 3 (TF - Timer Flag)
    
    // Write back to clear the flag and release INT pin
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_STATUS);
    Wire.write(status);
    Wire.endTransmission();
  }
  
  // Note: Main logic for voltage/charge check happens in begin()
}

// ===== Helper Functions =====

/// @brief Find next available channel number in CayenneLPP packet
/// @param lpp CayenneLPP packet to analyze
/// @return Next free channel number (starts at 200 if no channels used yet)
/// @note This method parses the LPP buffer to find the highest channel number in use,
///       then returns the next available channel. Used by queryBoardTelemetry() to
///       append board-specific telemetry without channel conflicts.
uint8_t InheroMr2Board::findNextFreeChannel(CayenneLPP& lpp) {
  uint8_t max_channel = 0;
  uint8_t cursor = 0;
  uint8_t* buffer = lpp.getBuffer();
  uint8_t size = lpp.getSize();

  while (cursor < size) {
    if (cursor + 1 >= size) break;

    uint8_t channel = buffer[cursor];
    if (channel > max_channel) max_channel = channel;

    uint8_t type = buffer[cursor + 1];
    uint8_t data_len = 0;

    switch (type) {
    case LPP_DIGITAL_INPUT:
    case LPP_DIGITAL_OUTPUT:
    case LPP_PRESENCE:
    case LPP_RELATIVE_HUMIDITY:
    case LPP_PERCENTAGE:
    case LPP_SWITCH:
      data_len = 1;
      break;

    case LPP_ANALOG_INPUT:
    case LPP_ANALOG_OUTPUT:
    case LPP_LUMINOSITY:
    case LPP_TEMPERATURE:
    case LPP_BAROMETRIC_PRESSURE:
    case LPP_VOLTAGE:
    case LPP_CURRENT:
    case LPP_ALTITUDE:
    case LPP_POWER:
    case LPP_DIRECTION:
    case LPP_CONCENTRATION:
      data_len = 2;
      break;

    case LPP_COLOUR:
      data_len = 3;
      break;

    case LPP_FREQUENCY:
    case LPP_DISTANCE:
    case LPP_ENERGY:
    case LPP_UNIXTIME:
      data_len = 4;
      break;

    case LPP_ACCELEROMETER:
    case LPP_GYROMETER:
      data_len = 6;
      break;

    case LPP_GPS:
      data_len = 9;
      break;

    default:
      return (max_channel < 200) ? 200 : max_channel + 1;
    }

    cursor += (2 + data_len);
  }

  return max_channel + 1;
}
