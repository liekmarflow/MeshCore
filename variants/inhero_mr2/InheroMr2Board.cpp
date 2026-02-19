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
#include "target.h"

#include <Arduino.h>
#include <Wire.h>
#include <nrf_soc.h>

// Static declarations
static BoardConfigContainer boardConfig;
volatile bool InheroMr2Board::rtc_irq_pending = false;
volatile uint32_t InheroMr2Board::ota_dfu_reset_at = 0;

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
  delay(50); // Give I2C bus time to stabilize

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
      (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos); // Wake on LOW (RTC interrupt is active-low)

  attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), rtcInterruptHandler, FALLING);

  // === CRITICAL: Early Boot Voltage Check ===
  // Prevents motorboating after Hardware-UVLO or Software-SHUTDOWN
  // When INA228 Alert cuts power via TPS62840 EN, RAK loses all RAM (GPREGRET2=0x00)
  // We must check voltage on EVERY ColdBoot, not just after Software-SHUTDOWN

  uint8_t shutdown_reason = NRF_POWER->GPREGRET2; // Read full register for Early Boot check

  // === High-Precision INA228 voltage measurement (24-bit ADC, ±0.1% accuracy) ===
  // RAK4630 cannot measure battery voltage - there's no voltage divider on GPIO!
  // Use static INA228 method with One-Shot ADC for fresh, accurate measurement
  // This is critical for wake/sleep decisions in danger zone
  MESH_DEBUG_PRINTLN("Early Boot: Reading VBAT from INA228 @ 0x40...");
  uint16_t vbat_mv = Ina228Driver::readVBATDirect(&Wire, 0x40);
  MESH_DEBUG_PRINTLN("Early Boot: readVBATDirect returned %dmV", vbat_mv);

  // CRITICAL: readVBATDirect() sets ADC_CONFIG to 0x1000 (Single-Shot mode)
  // Restore to continuous mode (0xF003) as interim config until ina228.begin()
  // sets the final high-accuracy config (0xFFCA with long conversion times)
  Wire.beginTransmission(0x40);
  Wire.write(0x01); // ADC_CONFIG register
  Wire.write(0xF0); // MSB: Continuous all channels (0xF)
  Wire.write(0x03); // LSB: 64 samples averaging (0x3) - interim, overridden by ina228.begin()
  uint8_t i2c_result = Wire.endTransmission();
  delay(50); // Longer delay for I2C bus to stabilize before BQ init
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
      if (boardConfig.getLEDsEnabled()) {
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_BLUE, HIGH);
          delay(150);
          digitalWrite(LED_BLUE, LOW);
          delay(150);
        }
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

        // Configure RTC wake-up (6h production)
        // voltageMonitorTask will send sleep command on next boot after init completes
        configureRTCWake(6);
        sd_power_system_off();
        // Never returns
      }

      // Voltage recovered above critical threshold  - resumed normal operation
      MESH_DEBUG_PRINTLN("Voltage recovered to %dmV, resuming normal operation", vbat_mv);
      // Clear ALL flags including Danger Zone - we're exiting!
      NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
    }
    // Case 2: ColdBoot after Hardware-UVLO (GPREGRET2 may have Danger Zone flag from previous session)
    // This is the critical case to prevent motorboating!
    else if (vbat_mv < critical_threshold) {
      MESH_DEBUG_PRINTLN("ColdBoot in danger zone detected (%dmV < %dmV)", vbat_mv, critical_threshold);
      MESH_DEBUG_PRINTLN("Likely Hardware-UVLO recovery at %dmV - voltage not stable yet", uvlo_threshold);
      MESH_DEBUG_PRINTLN("Going to sleep for 6h to avoid motorboating");

      delay(100);

      // Do NOT send sleep command here - would cause race condition with bootup
      // voltageMonitorTask will handle sleep after full system initialization

      // Configure RTC wake-up before shutdown (6h production)
      configureRTCWake(6);

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

  // Deferred OTA DFU reset: wait for CLI reply to be sent, then enter bootloader
  if (ota_dfu_reset_at != 0 && millis() >= ota_dfu_reset_at) {
    enterOTADfu();  // disables SoftDevice & interrupts, sets GPREGRET, resets — does not return
  }

  if (rtc_irq_pending) {
    rtc_irq_pending = false;

    // Clear TF here (not in ISR) to avoid I2C bus collisions with core RTC access.
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)1);

    if (Wire.available()) {
      uint8_t status = Wire.read();
      status &= ~(1 << 3); // Clear TF bit (bit 3)

      Wire.beginTransmission(RTC_I2C_ADDR);
      Wire.write(RV3028_REG_STATUS);
      Wire.write(status);
      Wire.endTransmission();
    }
  }
}

uint16_t InheroMr2Board::getBattMilliVolts() {
  // WORKAROUND: The MeshCore protocol currently only transmits battery voltage
  // (via getBattMilliVolts), not a direct SOC percentage. The companion app then
  // interprets this voltage using a hardcoded Li-Ion discharge curve to derive SOC%.
  // This gives wrong readings for LiFePO4/LTO chemistries whose voltage profiles
  // differ significantly from Li-Ion.
  //
  // Solution: When we have a valid Coulomb-counted SOC, we reverse-map it to
  // the Li-Ion 1S OCV (Open Circuit Voltage) that the app expects.
  // This way the app always displays our accurate chemistry-independent SOC.
  //
  // TODO: Remove this workaround once MeshCore supports transmitting the actual
  // SOC percentage alongside (or instead of) battery millivolts. At that point,
  // this function should return the real battery voltage again.

  const BatterySOCStats* socStats = boardConfig.getSOCStats();
  if (socStats && socStats->soc_valid) {
    return socToLiIonMilliVolts(boardConfig.getStateOfCharge());
  }

  // Fallback: no valid Coulomb-counting SOC yet — return real voltage
  const Telemetry* telemetry = boardConfig.getTelemetryData();
  if (!telemetry) {
    return 0;
  }
  return telemetry->batterie.voltage;
}

/// @brief Maps a SOC percentage (0-100%) to a fake Li-Ion 1S OCV in millivolts.
/// @details Uses a standard Li-Ion NMC/NCA OCV lookup table with piecewise-linear
///          interpolation. The companion app will reverse-map these voltages back
///          to the same SOC%, giving correct battery level display regardless of
///          the actual cell chemistry (Li-Ion, LiFePO4, LTO).
/// @param soc_percent State of Charge in percent (0.0 – 100.0)
/// @return Equivalent Li-Ion 1S voltage in millivolts (3000 – 4200)
uint16_t InheroMr2Board::socToLiIonMilliVolts(float soc_percent) {
  // Clamp input to valid range
  if (soc_percent <= 0.0f) return 3000;
  if (soc_percent >= 100.0f) return 4200;

  // Standard Li-Ion 1S OCV table (NMC/NCA, 10% steps)
  // Index 0 = 0% SOC, Index 10 = 100% SOC
  static const uint16_t LI_ION_OCV_TABLE[] = {
    3000,  // 0%
    3300,  // 10%
    3450,  // 20%
    3530,  // 30%
    3600,  // 40%
    3670,  // 50%
    3740,  // 60%
    3820,  // 70%
    3920,  // 80%
    4050,  // 90%
    4200   // 100%
  };

  // Piecewise-linear interpolation between 10% steps
  float index_f = soc_percent / 10.0f;       // 0.0 – 10.0
  uint8_t idx_lo = (uint8_t)index_f;          // lower table index
  if (idx_lo >= 10) idx_lo = 9;               // safety clamp
  uint8_t idx_hi = idx_lo + 1;

  float frac = index_f - (float)idx_lo;       // fractional part (0.0 – 1.0)
  float mv = (float)LI_ION_OCV_TABLE[idx_lo]
           + frac * (float)(LI_ION_OCV_TABLE[idx_hi] - LI_ION_OCV_TABLE[idx_lo]);

  return (uint16_t)(mv + 0.5f);               // round to nearest mV
}

bool InheroMr2Board::startOTAUpdate(const char* id, char reply[]) {
  // Skip the in-app BLE DFU (unstable on nRF52 in MeshCore environment) and
  // jump directly into the Adafruit bootloader's OTA DFU mode.
  // enterOTADfu() sets GPREGRET=0xA8, disables SoftDevice & interrupts, then resets.
  // The bootloader handles BLE advertising and firmware transfer natively.
  MESH_DEBUG_PRINTLN("OTA: Scheduling Adafruit bootloader DFU mode...");

  // Read BLE MAC address from nRF52 hardware registers (no Bluefruit needed)
  uint32_t addr0 = NRF_FICR->DEVICEADDR[0];
  uint32_t addr1 = NRF_FICR->DEVICEADDR[1];
  snprintf(reply, 64, "OK DFU - mac: %02X:%02X:%02X:%02X:%02X:%02X",
           (addr1 >> 8) & 0xFF, addr1 & 0xFF,
           (addr0 >> 24) & 0xFF, (addr0 >> 16) & 0xFF, (addr0 >> 8) & 0xFF, addr0 & 0xFF);

  // Schedule deferred reset into bootloader DFU mode.
  // Return immediately so the CLI handler can send the reply first.
  // tick() will handle cleanup (stop tasks, radio off) and reset after the delay.
  ota_dfu_reset_at = millis() + 3000;  // 3s delay to ensure reply is transmitted

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

  uint8_t batteryChannel = this->findNextFreeChannel(telemetry);
  uint8_t solarChannel = batteryChannel + 1;

  const BatterySOCStats* socStats = boardConfig.getSOCStats();
  bool hasValidSoc = (socStats && socStats->soc_valid);
  float socPercent = boardConfig.getStateOfCharge();
  // Requested precision: one decimal place
  socPercent = roundf(socPercent * 10.0f) / 10.0f;

  uint16_t ttlHours = boardConfig.getTTL_Hours();
  bool isInfiniteTtl = (socStats && socStats->soc_valid && !socStats->living_on_battery);
  constexpr float MAX_TTL_DAYS = 990.0f; // Max encodable LPP distance value

  // Battery channel
  // Field order:
  // 1) VBAT[V], 2) SOC[%] (optional), 3) IBAT[A], 4) TBAT[°C], 5) TTL[d] (optional)
  telemetry.addVoltage(batteryChannel, telemetryData->batterie.voltage / 1000.0f);
  if (hasValidSoc) {
    telemetry.addPercentage(batteryChannel, socPercent);
  }
  telemetry.addCurrent(batteryChannel, telemetryData->batterie.current / 1000.0f);
  telemetry.addTemperature(batteryChannel, telemetryData->batterie.temperature);

  // TTL handling:
  // - ttlHours > 0: send finite TTL in days
  // - surplus (infinite TTL): send max value
  // - unknown TTL (ttlHours == 0 and not surplus): send nothing
  if (ttlHours > 0) {
    telemetry.addDistance(batteryChannel, ttlHours / 24.0f);
  } else if (isInfiniteTtl) {
    telemetry.addDistance(batteryChannel, MAX_TTL_DAYS);
  }

  // Solar channel
  // Field order:
  // 1) VSOL[V], 2) ISOL[A], 3) MPPT_7D[%]
  telemetry.addVoltage(solarChannel, telemetryData->solar.voltage / 1000.0f);
  telemetry.addCurrent(solarChannel, telemetryData->solar.current / 1000.0f);
  telemetry.addPercentage(solarChannel, boardConfig.getMpptEnabledPercentage7Day());

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
    snprintf(reply, maxlen, "%s",
             BoardConfigContainer::getBatteryTypeCommandString(boardConfig.getBatteryType()));
    return true;
  } else if (strcmp(cmd, "hwver") == 0) {
    // MR2 is always v0.2 hardware
    snprintf(reply, maxlen, "v0.2 (INA228+RTC)");
    return true;
  } else if (strcmp(cmd, "fmax") == 0) {
    // LTO batteries ignore JEITA temperature control
    if (boardConfig.getBatteryType() == BoardConfigContainer::BatteryType::LTO_2S) {
      snprintf(reply, maxlen, "N/A");
    } else {
      snprintf(
          reply, maxlen, "%s",
          BoardConfigContainer::getFrostChargeBehaviourCommandString(boardConfig.getFrostChargeBehaviour()));
    }
    return true;
  } else if (strcmp(cmd, "imax") == 0) {
    snprintf(reply, maxlen, "%s", boardConfig.getChargeCurrentAsStr());
    return true;
  } else if (strcmp(cmd, "mppt") == 0) {
    snprintf(reply, maxlen, "MPPT=%s", boardConfig.getMPPTEnabled() ? "1" : "0");
    return true;
  } else if (strcmp(cmd, "stats") == 0) {
    // Combined energy statistics: balance + MPPT
    const BatterySOCStats* socStats = boardConfig.getSOCStats();
    if (!socStats) {
      float mppt_pct = boardConfig.getMpptEnabledPercentage7Day();
      snprintf(reply, maxlen, "N/A M:%.0f%%", mppt_pct);
      return true;
    }

    // Balance info (mAh) - rolling windows (no midnight reset)
    float last_24h_net = socStats->last_24h_net_mah;
    float last_24h_charged = socStats->last_24h_charged_mah;
    float last_24h_discharged = socStats->last_24h_discharged_mah;
    const char* status = socStats->living_on_battery ? "BAT" : "SOL";
    float avg3d = socStats->avg_3day_daily_net_mah;
    float avg3d_charged = socStats->avg_3day_daily_charged_mah;
    float avg3d_discharged = socStats->avg_3day_daily_discharged_mah;
    float avg7d = socStats->avg_7day_daily_net_mah;
    float avg7d_charged = socStats->avg_7day_daily_charged_mah;
    float avg7d_discharged = socStats->avg_7day_daily_discharged_mah;
    uint16_t ttl = boardConfig.getTTL_Hours();

    // MPPT info
    float mppt_pct = boardConfig.getMpptEnabledPercentage7Day();

    // Compact format: 24h/3d/7d Status MPPT% [TTL]
    if (ttl > 0) {
      char ttlBuf[16];
      if (ttl >= 24) {
        snprintf(ttlBuf, sizeof(ttlBuf), "%dd%dh", ttl / 24, ttl % 24);
      } else {
        snprintf(ttlBuf, sizeof(ttlBuf), "%dh", ttl);
      }
      snprintf(reply, maxlen, "%+.1f/%+.1f/%+.1fmAh C:%.1f D:%.1f 3dC:%.1f 3dD:%.1f 7dC:%.1f 7dD:%.1f %s M:%.0f%% TTL:%s",
               last_24h_net, avg3d, avg7d, last_24h_charged, last_24h_discharged, avg3d_charged, avg3d_discharged,
               avg7d_charged, avg7d_discharged, status, mppt_pct, ttlBuf);
    } else {
      snprintf(reply, maxlen, "%+.1f/%+.1f/%+.1fmAh C:%.1f D:%.1f 3dC:%.1f 3dD:%.1f 7dC:%.1f 7dD:%.1f %s M:%.0f%%",
               last_24h_net, avg3d, avg7d, last_24h_charged, last_24h_discharged, avg3d_charged, avg3d_discharged,
               avg7d_charged, avg7d_discharged, status, mppt_pct);
    }
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
    // Manual HIZ cycle to force input detection (like automatic task)
    char hizBuffer[100];
    boardConfig.toggleHizAndCheck(hizBuffer, sizeof(hizBuffer));
    snprintf(reply, maxlen, "%s", hizBuffer);
    return true;
  } else if (strcmp(cmd, "telem") == 0) {
    const Telemetry* telemetry = boardConfig.getTelemetryData();
    if (!telemetry) {
      snprintf(reply, maxlen, "Err: Telemetry unavailable");
      return true;
    }

    // Use precise battery current from telemetry (INA228)
    float precise_current_ma = telemetry->batterie.current;

    // Get SOC info
    float soc = boardConfig.getStateOfCharge();
    const BatterySOCStats* socStats = boardConfig.getSOCStats();

    // Format currents: battery current with 1 decimal, solar current without decimals
    // INA228 driver returns correctly signed values: positive=charging, negative=discharging
    char bat_current_str[16];
    snprintf(bat_current_str, sizeof(bat_current_str), "%.1fmA", precise_current_ma);

    char sol_current_str[16];
    snprintf(sol_current_str, sizeof(sol_current_str), "~%.0fmA", (float)telemetry->solar.current);

    if (socStats && socStats->soc_valid) {
      snprintf(reply, maxlen, "B:%.2fV/%s/%.0fC SOC:%.1f%% S:%.2fV/%s", telemetry->batterie.voltage / 1000.0f,
               bat_current_str, telemetry->batterie.temperature, soc, telemetry->solar.voltage / 1000.0f,
               sol_current_str);
    } else {
      snprintf(reply, maxlen, "B:%.2fV/%s/%.0fC SOC:N/A S:%.2fV/%s", telemetry->batterie.voltage / 1000.0f,
               bat_current_str, telemetry->batterie.temperature, telemetry->solar.voltage / 1000.0f,
               sol_current_str);
    }
    return true;
  } else if (strcmp(cmd, "conf") == 0) {
    // Display all configuration values
    const char* batType = BoardConfigContainer::getBatteryTypeCommandString(boardConfig.getBatteryType());
    const char* frostBehaviour;
    if (boardConfig.getBatteryType() == BoardConfigContainer::BatteryType::LTO_2S) {
      frostBehaviour = "N/A";
    } else {
      frostBehaviour =
          BoardConfigContainer::getFrostChargeBehaviourCommandString(boardConfig.getFrostChargeBehaviour());
    }
    float chargeVoltage = boardConfig.getMaxChargeVoltage();
    float voltage0Soc = getVoltageCriticalThreshold() / 1000.0f;
    const char* imax = boardConfig.getChargeCurrentAsStr();
    bool mpptEnabled = boardConfig.getMPPTEnabled();

    snprintf(reply, maxlen, "B:%s F:%s M:%s I:%s Vco:%.2f V0:%.2f", batType, frostBehaviour,
             mpptEnabled ? "1" : "0", imax, chargeVoltage, voltage0Soc);
    return true;
  } else if (strcmp(cmd, "ibcal") == 0) {
    // Get current INA228 calibration factor
    float factor = boardConfig.getIna228CalibrationFactor();
    snprintf(reply, maxlen, "INA228 calibration: %.4f (1.0=default)", factor);
    return true;
  } else if (strcmp(cmd, "iboffset") == 0) {
    // Get current INA228 current offset correction
    float offset = boardConfig.getIna228CurrentOffset();
    snprintf(reply, maxlen, "INA228 offset: %+.2f mA (0.00=default)", offset);
    return true;
  } else if (strcmp(cmd, "tccal") == 0) {
    // Get current NTC temperature calibration offset
    float offset = boardConfig.getTcCalOffset();
    snprintf(reply, maxlen, "TC offset: %+.2f C (0.00=default)", offset);
    return true;
  } else if (strcmp(cmd, "uvlo") == 0) {
    // Get INA228 UVLO enable state
    bool enabled = boardConfig.getUvloEnabled();
    snprintf(reply, maxlen, "UVLO: %s", enabled ? "ENABLED" : "DISABLED");
    return true;
  } else if (strcmp(cmd, "leds") == 0) {
    // Get LED enable state (heartbeat + BQ stat LED)
    bool enabled = boardConfig.getLEDsEnabled();
    snprintf(reply, maxlen, "LEDs: %s (Heartbeat + BQ Stat)", enabled ? "ON" : "OFF");
    return true;
  } else if (strcmp(cmd, "batcap") == 0) {
    // Get battery capacity in mAh - show if default or explicitly set
    float capacity_mah = boardConfig.getBatteryCapacity();
    bool explicitly_set = boardConfig.isBatteryCapacitySet();
    if (explicitly_set) {
      snprintf(reply, maxlen, "%.0f mAh (set)", capacity_mah);
    } else {
      snprintf(reply, maxlen, "%.0f mAh (default)", capacity_mah);
    }
    return true;
  } else if (strcmp(cmd, "energy") == 0) {
    // Read INA228 Charge Counter Register (already inverted in driver)
    Ina228Driver* ina = boardConfig.getIna228Driver();
    if (ina != nullptr) {
      float charge_mah = ina->readCharge_mAh();
      const BatterySOCStats* socStats = boardConfig.getSOCStats();

      if (socStats && socStats->soc_valid) {
        // Show charge and baseline (for debugging SOC calculations)
        float net_charge = charge_mah - socStats->ina228_baseline_mah;
        snprintf(reply, maxlen, "%.1fmAh (Base: %.1fmAh, Net: %+.1fmAh)", charge_mah,
                 socStats->ina228_baseline_mah, net_charge);
      } else {
        // SOC not yet synced, only show raw value
        snprintf(reply, maxlen, "%.1fmAh (SOC not synced)", charge_mah);
      }
    } else {
      snprintf(reply, maxlen, "Err: INA228 not initialized");
    }
    return true;
  }

  snprintf(reply, maxlen,
           "Err: Try "
           "board.<bat|hwver|fmax|imax|telem|stats|cinfo|diag|togglehiz|mppt|conf|ibcal|iboffset|tccal|uvlo|leds|batcap|"
           "energy>");
  return true;
}

/// @brief Handles custom CLI setter commands for board configuration
/// @param setCommand Command string with value (without "board." prefix)
/// @return Status message ("OK" on success, error message on failure)
const char* InheroMr2Board::setCustomSetter(const char* setCommand) {

  static char ret[100];
  memset(ret, 0, sizeof(ret)); // Clear buffer to prevent garbage data

  if (strncmp(setCommand, "bat ", 4) == 0) {
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[4]));
    BoardConfigContainer::BatteryType bt = BoardConfigContainer::getBatteryTypeFromCommandString(value);
    if (bt != BoardConfigContainer::BatteryType::BAT_UNKNOWN) {
      boardConfig.setBatteryType(bt);
      snprintf(ret, sizeof(ret), "Bat set to %s",
               BoardConfigContainer::getBatteryTypeCommandString(boardConfig.getBatteryType()));
      return ret;
    } else {
      snprintf(ret, sizeof(ret), "Err: Try one of: %s", BoardConfigContainer::getAvailableBatOptions());
      return ret;
    }
  } else if (strncmp(setCommand, "fmax ", 5) == 0) {
    // LTO batteries ignore JEITA temperature control - setting fmax behavior is not applicable
    if (boardConfig.getBatteryType() == BoardConfigContainer::BatteryType::LTO_2S) {
      snprintf(ret, sizeof(ret), "Err: Fmax setting N/A for LTO (JEITA disabled)");
      return ret;
    }

    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    BoardConfigContainer::FrostChargeBehaviour fcb =
        BoardConfigContainer::getFrostChargeBehaviourFromCommandString(value);
    if (fcb != BoardConfigContainer::FrostChargeBehaviour::REDUCE_UNKNOWN) {
      boardConfig.setFrostChargeBehaviour(fcb);
      snprintf(
          ret, sizeof(ret), "Fmax charge current set to %s of imax",
          BoardConfigContainer::getFrostChargeBehaviourCommandString(boardConfig.getFrostChargeBehaviour()));
      return ret;
    } else {
      snprintf(ret, sizeof(ret), "Err: Try one of: %s",
               BoardConfigContainer::getAvailableFrostChargeBehaviourOptions());
      return ret;
    }
  } else if (strncmp(setCommand, "imax ", 5) == 0) {
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    int ma = atoi(value);
    if (ma >= 50 && ma <= 1000) {
      boardConfig.setMaxChargeCurrent_mA(ma);
      snprintf(ret, sizeof(ret), "Max charge current set to %s", boardConfig.getChargeCurrentAsStr());
      return ret;
    } else {
      return "Err: Try 50-1000";
    }
  } else if (strncmp(setCommand, "mppt ", 5) == 0) {
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    // Convert to lowercase for case-insensitive comparison
    char lowerValue[20];
    strncpy(lowerValue, value, sizeof(lowerValue) - 1);
    lowerValue[sizeof(lowerValue) - 1] = '\0';
    for (char* p = lowerValue; *p; ++p)
      *p = tolower(*p);

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
    // INA228 current calibration: set board.ibcal <actual_current_mA> or set board.ibcal reset
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[6]));

    // Check for reset command
    if (strcmp(value, "reset") == 0 || strcmp(value, "RESET") == 0) {
      if (boardConfig.setIna228CalibrationFactor(1.0f)) {
        snprintf(ret, sizeof(ret), "INA228 calibration reset to 1.0000 (default)");
      } else {
        snprintf(ret, sizeof(ret), "Err: Failed to reset calibration");
      }
      return ret;
    }

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
  } else if (strncmp(setCommand, "iboffset ", 9) == 0) {
    // INA228 current offset calibration:
    //   set board.iboffset <actual_current_mA>  → calibrate offset using reference meter
    //   set board.iboffset reset                → reset offset to 0.00
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[9]));

    // Check for reset command
    if (strcmp(value, "reset") == 0 || strcmp(value, "RESET") == 0) {
      if (boardConfig.setIna228CurrentOffset(0.0f)) {
        snprintf(ret, sizeof(ret), "INA228 offset reset to +0.00 mA (default)");
      } else {
        snprintf(ret, sizeof(ret), "Err: Failed to reset offset");
      }
      return ret;
    }

    float actual_current_ma = atof(value);

    // Validate reasonable current range (-2000 to +2000 mA)
    if (actual_current_ma < -2000.0f || actual_current_ma > 2000.0f) {
      snprintf(ret, sizeof(ret), "Err: Current out of range (-2000 to +2000 mA)");
      return ret;
    }

    // Perform offset calibration and store
    float offset = boardConfig.performIna228OffsetCalibration(actual_current_ma);
    snprintf(ret, sizeof(ret), "INA228 offset: %+.2f mA", offset);
    return ret;
  } else if (strncmp(setCommand, "tccal", 5) == 0) {
    // NTC temperature calibration:
    //   set board.tccal          → auto-read BME280 as reference
    //   set board.tccal <temp_C>  → manual reference value
    //   set board.tccal reset     → reset to 0.00
    const char* rest = &setCommand[5];

    // Skip optional space
    if (*rest == ' ') rest++;

    const char* value = BoardConfigContainer::trim(const_cast<char*>(rest));

    // Check for reset command
    if (strcmp(value, "reset") == 0 || strcmp(value, "RESET") == 0) {
      if (boardConfig.setTcCalOffset(0.0f)) {
        snprintf(ret, sizeof(ret), "TC calibration reset to 0.00 (default)");
      } else {
        snprintf(ret, sizeof(ret), "Err: Failed to reset TC calibration");
      }
      return ret;
    }

    float new_offset;

    if (value[0] == '\0') {
      // No argument: auto-read BME280 as reference (averages 5 samples each)
      float bme_avg = 0.0f;
      new_offset = boardConfig.performTcCalibration(&bme_avg);
      if (new_offset > -900.0f) {
        snprintf(ret, sizeof(ret), "TC auto-cal: BME=%.1f offset=%+.2f C", bme_avg, new_offset);
      } else {
        snprintf(ret, sizeof(ret), "Err: Auto-cal failed (BME280/NTC error?)");
      }
    } else {
      // Manual reference value provided
      float actual_temp_c = atof(value);

      // Validate reasonable temperature range (-40 to +85 °C)
      if (actual_temp_c < -40.0f || actual_temp_c > 85.0f) {
        snprintf(ret, sizeof(ret), "Err: Temp out of range (-40 to +85 C)");
        return ret;
      }

      new_offset = boardConfig.performTcCalibration(actual_temp_c);
      if (new_offset > -900.0f) {
        snprintf(ret, sizeof(ret), "TC calibrated: offset=%+.2f C", new_offset);
      } else {
        snprintf(ret, sizeof(ret), "Err: TC calibration failed (NTC read error?)");
      }
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
  } else if (strncmp(setCommand, "uvlo ", 5) == 0) {
    // Enable/disable INA228 UVLO alert
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[5]));
    bool enabled = (strcmp(value, "1") == 0 || strcmp(value, "true") == 0 || strcmp(value, "TRUE") == 0);
    bool disabled = (strcmp(value, "0") == 0 || strcmp(value, "false") == 0 || strcmp(value, "FALSE") == 0);

    if (enabled || disabled) {
      if (boardConfig.setUvloEnabled(enabled)) {
        snprintf(ret, sizeof(ret), "UVLO %s (persistent)", enabled ? "ENABLED" : "DISABLED");
      } else {
        snprintf(ret, sizeof(ret), "Err: Failed to save UVLO setting");
      }
      return ret;
    } else {
      snprintf(ret, sizeof(ret), "Err: Use 'true/1', 'false/0'");
      return ret;
    }
  } else if (strncmp(setCommand, "soc ", 4) == 0) {
    // Manually set SOC percentage (e.g. after reboot with known SOC)
    const char* value = BoardConfigContainer::trim(const_cast<char*>(&setCommand[4]));
    float soc_percent = atof(value);

    if (BoardConfigContainer::setSOCManually(soc_percent)) {
      snprintf(ret, sizeof(ret), "SOC set to %.1f%%", soc_percent);
    } else {
      snprintf(ret, sizeof(ret), "Err: Invalid SOC (0-100) or INA228 not ready");
    }
    return ret;
  }

  snprintf(ret, sizeof(ret), "Err: Try board.<bat|imax|fmax|mppt|batcap|ibcal|iboffset|tccal|bqreset|leds|uvlo|soc>");
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
    delay(100); // Allow I/O to complete

    // 3. Configure RTC to wake us up (6 hours)
    configureRTCWake(6);
  }

  // 4. Store shutdown reason for next boot
  NRF_POWER->GPREGRET2 = reason;

  // 5. Enter SYSTEMOFF mode (1-5 µA)
  MESH_DEBUG_PRINTLN("PWRMGT: Entering SYSTEMOFF");
  delay(50); // Let debug output complete

  sd_power_system_off();
  // Never returns
}

/// @brief Configure RV-3028 RTC countdown timer for periodic wake-up (v0.2)
/// @param hours Wake-up interval in hours
void InheroMr2Board::configureRTCWake(uint32_t hours) {
#if defined(INHERO_MR2)
  rtc_clock.setLocked(true);
#endif
  uint32_t safe_hours = hours == 0 ? 6 : hours;
  uint32_t total_seconds = safe_hours * 3600UL;
  uint16_t countdown_ticks = static_cast<uint16_t>((total_seconds + 59UL) / 60UL);
  if (countdown_ticks == 0) {
    countdown_ticks = 1;
  }
  MESH_DEBUG_PRINTLN("PWRMGT: Configuring RTC wake in %lu hours (%lu seconds)",
                     static_cast<unsigned long>(safe_hours), static_cast<unsigned long>(total_seconds));

  // === RTC Timer Configuration per Manual Section 4.8.2 ===
  // Step 1: Stop Timer and clear flags
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL1);
  Wire.write(0x00); // TE=0, TD=00 (stop timer)
  Wire.endTransmission();

  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL2);
  Wire.write(0x00); // TIE=0 (disable interrupt)
  Wire.endTransmission();

  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_STATUS);
  Wire.write(0x00); // Clear TF flag
  Wire.endTransmission();

  // Step 2: Set Timer Value (ticks at 1/60 Hz)
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_TIMER_VALUE_0);
  Wire.write(countdown_ticks & 0xFF);        // Lower 8 bits
  Wire.write((countdown_ticks >> 8) & 0x0F); // Upper 4 bits
  Wire.endTransmission();

  // Step 3: Configure Timer (1/60 Hz clock, Single shot mode)
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL1);
  Wire.write(0x07); // TE=1 (Enable), TD=11 (1/60 Hz), TRPT=0 (Single shot)
  Wire.endTransmission();

  // Step 4: Enable Timer Interrupt
  Wire.beginTransmission(RTC_I2C_ADDR);
  Wire.write(RV3028_REG_CTRL2);
  Wire.write(0x10); // TIE=1 (Timer Interrupt Enable, bit 4)
  Wire.endTransmission();

  MESH_DEBUG_PRINTLN("PWRMGT: RTC countdown configured (%u ticks at 1/60 Hz)", countdown_ticks);

#if defined(INHERO_MR2)
  rtc_clock.setLocked(false);
#endif
}

/// @brief RTC interrupt handler - called when countdown timer expires (v0.2)
void InheroMr2Board::rtcInterruptHandler() {
  // RTC countdown elapsed - device woke from SYSTEMOFF
  // Defer I2C work to the main loop to avoid ISR I2C collisions.
  rtc_irq_pending = true;
}

// ===== Helper Functions =====

/// @brief Helper function to get LPP data length for a given type
/// @param type LPP data type
/// @return Data length in bytes, or 0 if unknown type
static uint8_t getLPPDataLength(uint8_t type) {
  switch (type) {
  case LPP_DIGITAL_INPUT:
  case LPP_DIGITAL_OUTPUT:
  case LPP_PRESENCE:
  case LPP_RELATIVE_HUMIDITY:
  case LPP_PERCENTAGE:
  case LPP_SWITCH:
    return 1;

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
    return 2;

  case LPP_COLOUR:
    return 3;

  case LPP_GENERIC_SENSOR:
  case LPP_FREQUENCY:
  case LPP_DISTANCE:
  case LPP_ENERGY:
  case LPP_UNIXTIME:
    return 4;

  case LPP_ACCELEROMETER:
  case LPP_GYROMETER:
    return 6;

  case LPP_GPS:
    return 9;

  case LPP_POLYLINE:
    return 8; // minimum size

  default:
    return 0; // Unknown type
  }
}

/// @brief Find next available channel number in CayenneLPP packet
/// @param lpp CayenneLPP packet to analyze
/// @return Next free channel number (highest used channel + 1)
/// @note This method parses the LPP buffer to find the highest channel number in use,
///       then returns the next available channel. Used by queryBoardTelemetry() to
///       append board-specific telemetry without channel conflicts.
uint8_t InheroMr2Board::findNextFreeChannel(CayenneLPP& lpp) {
  uint8_t max_channel = 0;
  uint8_t cursor = 0;
  uint8_t* buffer = lpp.getBuffer();
  uint8_t size = lpp.getSize();

  while (cursor < size) {
    // Need at least 2 bytes: channel + type
    if (cursor + 1 >= size) break;

    uint8_t channel = buffer[cursor];
    uint8_t type = buffer[cursor + 1];
    uint8_t data_len = getLPPDataLength(type);

    // Unknown type - can't determine length, stop parsing
    if (data_len == 0) break;

    // Update max channel
    if (channel > max_channel) max_channel = channel;

    // Move to next entry
    cursor += 2 + data_len;
  }

  // Return next channel after the highest parsed one.
  // If packet is empty, start at channel 1.
  return max_channel + 1;
}
