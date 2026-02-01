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
#include "InheroMr1Board.h"

/// @brief Find next available channel number in CayenneLPP packet
/// @param lpp CayenneLPP packet to analyze
/// @return Next free channel number (starts at 200 if no channels used yet)
/// @note This method parses the LPP buffer to find the highest channel number in use,
///       then returns the next available channel. Used by queryBoardTelemetry() to
///       append board-specific telemetry without channel conflicts.
uint8_t InheroMr1Board::findNextFreeChannel(CayenneLPP& lpp) {
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

#include "BoardConfigContainer.h"

#include <Arduino.h>
#include <Wire.h>
#include <bluefruit.h>
#include <nrf_soc.h>

static BLEDfu bledfu;

static BoardConfigContainer boardConfig;

static void connect_callback(uint16_t conn_handle) {
  (void)conn_handle;
  MESH_DEBUG_PRINTLN("BLE client connected");
}

static void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  MESH_DEBUG_PRINTLN("BLE client disconnected");
}

/// @brief Collects board telemetry and appends to CayenneLPP packet
/// @param telemetry CayenneLPP packet to append data to
/// @return true if successful, false if telemetry data unavailable
bool InheroMr1Board::queryBoardTelemetry(CayenneLPP& telemetry) {
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

  return true;
}

/// @brief Handles custom CLI getter commands for board configuration
/// @param getCommand Command string (without "board." prefix)
/// @param reply Buffer to write response to
/// @param maxlen Maximum length of reply buffer
/// @return true if command was handled, false otherwise
bool InheroMr1Board::getCustomGetter(const char* getCommand, char* reply, uint32_t maxlen) {

  // Trim trailing whitespace from command
  char trimmedCommand[100];
  strncpy(trimmedCommand, getCommand, sizeof(trimmedCommand) - 1);
  trimmedCommand[sizeof(trimmedCommand) - 1] = '\0';
  char* cmd = BoardConfigContainer::trim(trimmedCommand);

  if (strcmp(cmd, "bat") == 0) {
    snprintf(reply, maxlen, "%s", BoardConfigContainer::getBatteryTypeCommandString(boardConfig.getBatteryType()));
    return true;
  } else if (strcmp(cmd, "hwver") == 0) {
    // Get hardware version (MR1 is always v0.1)
    snprintf(reply, maxlen, "v0.1 (MCP4652)");
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
  } else if (strcmp(cmd, "cinfo") == 0 || strcmp(cmd, "ci") == 0) {
    char infoBuffer[100];
    boardConfig.getChargerInfo(infoBuffer, sizeof(infoBuffer));
    snprintf(reply, maxlen, "%s", infoBuffer);
    return true;
  } else if (strcmp(cmd, "cstat") == 0 || strcmp(cmd, "cs") == 0) {
    // Alias for cinfo - charger status
    char infoBuffer[100];
    boardConfig.getChargerInfo(infoBuffer, sizeof(infoBuffer));
    snprintf(reply, maxlen, "%s", infoBuffer);
    return true;
  } else if (strcmp(cmd, "telem") == 0) {
    const Telemetry* telemetry = boardConfig.getTelemetryData();
    snprintf(reply, maxlen, "B:%.2fV/%imA/%.0fC S:%.2fV/%imA Y:%.2fV",
             telemetry->batterie.voltage / 1000.0f, telemetry->batterie.current,
             telemetry->batterie.temperature,
             telemetry->solar.voltage / 1000.0f, telemetry->solar.current,
             telemetry->system.voltage / 1000.0f);
    return true;
  } else if (strcmp(cmd, "soc") == 0) {
    // Not available in v0.1 hardware (use MR2 for SOC tracking)
    snprintf(reply, maxlen, "N/A (MR2 only)");
    return true;
  } else if (strcmp(cmd, "balance") == 0) {
    // Not available in v0.1 hardware (use MR2 for energy balance)
    snprintf(reply, maxlen, "N/A (MR2 only)");
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
  }

  snprintf(reply, maxlen, "Err: Try board.<bat|frost|life|imax|telem|cinfo|ci|cstat|cs|mppt|mpps|conf|wdtstatus|togglehiz>");
  return true;
}

/// @brief Handles custom CLI setter commands for board configuration
/// @param setCommand Command string with value (without "board." prefix)
/// @return Status message ("OK" on success, error message on failure)
const char* InheroMr1Board::setCustomSetter(const char* setCommand) {

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
  } else if (strcmp(setCommand, "togglehiz") == 0) {
    boardConfig.toggleHizAndCheck(ret, sizeof(ret));
    return ret;
  }

  snprintf(ret, sizeof(ret), "Err: Try board.<bat|imax|life|frost|mppt|togglehiz>");
  return ret;
}

void InheroMr1Board::begin() {
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

  // MR1 is v0.1 hardware (MCP4652 + TP2120 UVLO)
  MESH_DEBUG_PRINTLN("Inhero MR1 - Hardware v0.1");
  
  // Initialize board configuration (BQ25798, MCP4652, etc.)
  boardConfig.begin();
  
  // Enable DC/DC converter for improved power efficiency
  // Done after peripheral initialization to avoid voltage glitches
  NRF52BoardDCDC::begin();
  
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  pinMode(SX126X_POWER_EN, OUTPUT);
  digitalWrite(SX126X_POWER_EN, HIGH);
  delay(10); // give sx1262 some time to power up
  
  // Start hardware watchdog (600s timeout)
  // Must be last - after all initializations are complete
  BoardConfigContainer::setupWatchdog();
}

void InheroMr1Board::tick() {
  // Feed watchdog to prevent system reset
  // This ensures the main loop is running properly
  BoardConfigContainer::feedWatchdog();
}

uint16_t InheroMr1Board::getBattMilliVolts() {
  const Telemetry* telemetry = boardConfig.getTelemetryData();

  return telemetry->batterie.voltage;
}

bool InheroMr1Board::startOTAUpdate(const char* id, char reply[]) {
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
  Bluefruit.setName("InheroMR1_OTA");

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

