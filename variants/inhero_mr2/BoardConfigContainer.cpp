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
#include "BoardConfigContainer.h"
#include "GuardedRTCClock.h"
#include "InheroMr2Board.h"
#include "target.h"

#include "lib/BqDriver.h"
#include "lib/Ina228Driver.h"
#include "lib/SimplePreferences.h"

#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include <task.h>
#include <MeshCore.h>
#include <nrf_wdt.h>
#include <nrf_soc.h>  // For sd_power_system_off() and NRF_POWER

#if ENV_INCLUDE_BME280
#include <Adafruit_BME280.h>
#endif

#define RTC_SLEEP_SECONDS  (6 * 60 * 60)   // h * m * s
//#define RTC_SLEEP_SECONDS  (1 * 60)   //  m * s 1 min for testing

// rtc_clock is defined in target.cpp
extern GuardedRTCClock rtc_clock;

// Helper function to get RTC time safely
namespace {
  inline uint32_t getRTCTime() {
    return ((mesh::RTCClock&)rtc_clock).getCurrentTime();
  }

  bool isRtcPeriodicWakeConfigured(uint16_t expected_seconds);
  void configureRtcPeriodicWake(uint16_t seconds);

  bool isRtcPeriodicWakeConfigured(uint16_t expected_seconds) {
    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_CTRL1);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)1) != 1) return false;
    uint8_t ctrl1 = Wire.read();

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_CTRL2);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)1) != 1) return false;
    uint8_t ctrl2 = Wire.read();

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_TIMER_VALUE_0);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(RTC_I2C_ADDR, (uint8_t)2) != 2) return false;
    uint8_t val0 = Wire.read();
    uint8_t val1 = Wire.read();

    uint16_t countdown = (uint16_t)val0 | ((uint16_t)(val1 & 0x0F) << 8);

    bool timer_enabled = (ctrl1 & 0x04) != 0;    // TE
    bool repeat_enabled = (ctrl1 & 0x80) != 0;   // TRPT
    bool one_over_60_hz = (ctrl1 & 0x03) == 0x03; // TD=11 (1/60 Hz)
    bool interrupt_enabled = (ctrl2 & 0x10) != 0; // TIE
    uint16_t expected_ticks = static_cast<uint16_t>((expected_seconds + 59U) / 60U);

    return timer_enabled && repeat_enabled && one_over_60_hz && interrupt_enabled &&
         countdown == expected_ticks;
  }

  void configureRtcPeriodicWake(uint16_t seconds) {
    rtc_clock.setLocked(true);

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_CTRL1);
    Wire.write(0x00);  // TE=0, TD=00 (stop timer)
    Wire.endTransmission();

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_CTRL2);
    Wire.write(0x00);  // TIE=0
    Wire.endTransmission();

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_STATUS);
    Wire.write(0x00);  // Clear TF
    Wire.endTransmission();

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_TIMER_VALUE_0);
    uint16_t ticks = static_cast<uint16_t>((seconds + 59U) / 60U);
    if (ticks == 0) {
      ticks = 1;
    }
    Wire.write(ticks & 0xFF);
    Wire.write((ticks >> 8) & 0x0F);
    Wire.endTransmission();

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_CTRL1);
    Wire.write(0x87);  // TE=1, TD=11 (1/60 Hz), TRPT=1 (repeat)
    Wire.endTransmission();

    Wire.beginTransmission(RTC_I2C_ADDR);
    Wire.write(RV3028_REG_CTRL2);
    Wire.write(0x10);  // TIE=1
    Wire.endTransmission();

    rtc_clock.setLocked(false);
  }

  void blinkRed(uint8_t count, uint16_t on_ms, uint16_t off_ms, bool led_enabled) {
    if (!led_enabled) {
      return;
    }
    for (uint8_t i = 0; i < count; i++) {
      digitalWrite(LED_RED, HIGH);
      delay(on_ms);
      digitalWrite(LED_RED, LOW);
      delay(off_ms);
    }
  }
}

// Hardware drivers (v0.2 only)
static BqDriver bq;
static Ina228Driver ina228(0x40);  // A0=GND, A1=GND

static SimplePreferences prefs;

// Forward declare board instance
extern InheroMr2Board board;

// Initialize singleton pointer
BqDriver* BoardConfigContainer::bqDriverInstance = nullptr;
Ina228Driver* BoardConfigContainer::ina228DriverInstance = nullptr;
TaskHandle_t BoardConfigContainer::mpptTaskHandle = NULL;
TaskHandle_t BoardConfigContainer::heartbeatTaskHandle = NULL;
TaskHandle_t BoardConfigContainer::voltageMonitorTaskHandle = NULL;
TaskHandle_t BoardConfigContainer::socUpdateTaskHandle = NULL;
MpptStatistics BoardConfigContainer::mpptStats = {};
BatterySOCStats BoardConfigContainer::socStats = {};
bool BoardConfigContainer::leds_enabled = true;  // Default: enabled
float BoardConfigContainer::tcCalOffset = 0.0f;  // Default: no temperature calibration offset

// Solar charging thresholds
static const uint16_t MIN_VBUS_FOR_CHARGING = 3500; // 3.5V minimum for valid solar input

// Battery voltage thresholds moved to BatteryProperties structure (see .h file)
// SAFETY FEATURE: INA228 UVLO Alert provides latched hardware protection
// - Latched alert CANNOT be cleared by software (INA228 powered by battery)
// - Recovery requires PHYSICAL battery disconnect to reset INA228 registers
// - Or complete battery voltage collapse (INA loses power)
// This is intentional: Forces battery replacement/external charging for extreme deep discharge

// Temporary: disable INA228 UVLO alert during lab testing.
static const bool INA228_UVLO_ENABLED = false;

void BoardConfigContainer::runVoltageMonitor() {
  static bool init_done = false;
  static uint16_t critical_mv = 0;
  static uint32_t normal_interval_ms = 0;
  static uint32_t danger_interval_ms = 0;
  static uint32_t next_check_ms = 0;
  static uint32_t next_rtc_check_ms = 0;

  uint32_t now_ms = millis();

  if (!init_done) {
    BoardConfigContainer temp;
    float capacity_mah = 0.0f;
    bool capacity_loaded = false;
    if (bqDriverInstance) {
      capacity_loaded = temp.loadBatteryCapacity(capacity_mah);
    } else {
      capacity_mah = 2000.0f;
    }

    BatteryType battType = DEFAULT_BATTERY_TYPE;
    SimplePreferences prefs_bat;
    if (prefs_bat.begin("inheromr2")) {
      char buffer[10];
      if (prefs_bat.getString("batType", buffer, sizeof(buffer), "") > 0) {
        battType = getBatteryTypeFromCommandString(buffer);
        if (battType == BAT_UNKNOWN) {
          battType = DEFAULT_BATTERY_TYPE;
        }
      }
      prefs_bat.end();
    }

    socStats.capacity_mah = capacity_mah;
    socStats.nominal_voltage = getNominalVoltage(battType);
    socStats.current_soc_percent = 50.0f;
    socStats.lastHourUpdateTime = getRTCTime();

    MESH_DEBUG_PRINTLN("Battery capacity: %.0f mAh @ %.1fV %s",
                       capacity_mah, socStats.nominal_voltage,
                       capacity_loaded ? "manual" : "default");

    // Get danger zone threshold from battery properties
    const BatteryProperties* props = getBatteryProperties(battType);
    critical_mv = props ? props->danger_threshold : 2000;  // Fallback to 2000mV if lookup fails

    MESH_DEBUG_PRINTLN("Voltage Monitor: Critical=%dmV (Danger Zone boundary)", critical_mv);

  normal_interval_ms = 60UL * 1000UL;
  danger_interval_ms = 60UL * 1000UL;

    MESH_DEBUG_PRINTLN("PWRMGT: Waiting 2s for system stabilization before initial check");
    next_check_ms = now_ms + 2000;
    next_rtc_check_ms = now_ms + (60UL * 1000UL);
    init_done = true;
    return;
  }

  if (now_ms < next_check_ms) {
    if (now_ms >= next_rtc_check_ms) {
      if (!isRtcPeriodicWakeConfigured(RTC_SLEEP_SECONDS)) {
        MESH_DEBUG_PRINTLN("RTC: Periodic wake config missing - restoring %us", RTC_SLEEP_SECONDS);
        configureRtcPeriodicWake(RTC_SLEEP_SECONDS);
      }
      next_rtc_check_ms = now_ms + (60UL * 1000UL);
    }
    return;
  }

  if (now_ms >= next_rtc_check_ms) {
    if (!isRtcPeriodicWakeConfigured(RTC_SLEEP_SECONDS)) {
      MESH_DEBUG_PRINTLN("RTC: Periodic wake config missing - restoring %us", RTC_SLEEP_SECONDS);
      configureRtcPeriodicWake(RTC_SLEEP_SECONDS);
    }
    next_rtc_check_ms = now_ms + (60UL * 1000UL);
  }

  uint16_t vbat_mv = 0;
  if (ina228DriverInstance) {
    vbat_mv = ina228DriverInstance->readVoltage_mV();
  }

  bool in_danger_zone = (NRF_POWER->GPREGRET2 & GPREGRET2_IN_DANGER_ZONE) != 0;

  MESH_DEBUG_PRINTLN("PWRMGT: Voltage check - VBAT=%dmV, Critical=%dmV, DangerZone=%d, GPREGRET2=0x%02X",
                     vbat_mv, critical_mv, in_danger_zone, NRF_POWER->GPREGRET2);

  blinkRed(3, 100, 100, leds_enabled);

  if (vbat_mv > 0) {
    if (vbat_mv < critical_mv) {
      MESH_DEBUG_PRINTLN("PWRMGT: Danger Zone breach - voltage %dmV < %dmV - shutting down", vbat_mv, critical_mv);

      blinkRed(1, 100, 100, leds_enabled);
      blinkRed(3, 300, 300, leds_enabled);

      if (!in_danger_zone) {
        NRF_POWER->GPREGRET2 |= GPREGRET2_IN_DANGER_ZONE;
      }

      // Force SX1262 into sleep via RadioLib before SYSTEMOFF.
      radio_driver.powerOff();
      delay(5);

#ifndef DEBUG_MODE
      NRF_WDT->TASKS_START = 0;
#endif

      uint8_t rtc_result1 = 0, rtc_result2 = 0, rtc_result3 = 0, rtc_result4 = 0, rtc_result5 = 0;

      Wire.beginTransmission(0x52);
      Wire.write(0x0F);
      Wire.write(0x00);
      rtc_result1 = Wire.endTransmission();

      Wire.beginTransmission(0x52);
      Wire.write(0x10);
      Wire.write(0x00);
      rtc_result2 = Wire.endTransmission();

      Wire.beginTransmission(0x52);
      Wire.write(0x0E);
      Wire.write(0x00);
      rtc_result3 = Wire.endTransmission();

  uint16_t countdown = (RTC_SLEEP_SECONDS + 59U) / 60U;  // convert seconds to minutes (1/60 Hz ticks)
  Wire.beginTransmission(0x52);
  Wire.write(0x0A);
  Wire.write(countdown & 0xFF);
  Wire.write((countdown >> 8) & 0x0F);
  rtc_result4 = Wire.endTransmission();

  Wire.beginTransmission(0x52);
  Wire.write(0x0F);
  Wire.write(0x07);  // 1/60 Hz, single shot
  rtc_result5 = Wire.endTransmission();

  MESH_DEBUG_PRINTLN("RTC: Wake-up in %u minutes (%us)", countdown, RTC_SLEEP_SECONDS);

      Wire.beginTransmission(0x52);
      Wire.write(0x10);
      Wire.write(0x10);
      uint8_t rtc_result6 = Wire.endTransmission();

      NRF_POWER->GPREGRET2 = (NRF_POWER->GPREGRET2 & 0xFC) | SHUTDOWN_REASON_LOW_VOLTAGE;

      sd_power_system_off();
      delay(100);
      NRF_POWER->SYSTEMOFF = 1;
      delay(100);
      if (leds_enabled) {
        digitalWrite(LED_RED, HIGH);
      }
      while (1) { delay(1000); }
    } else {
      blinkRed(3, 100, 100, leds_enabled);

      if (in_danger_zone) {
        MESH_DEBUG_PRINTLN("PWRMGT: Voltage recovered to %dmV (Critical=%dmV)", vbat_mv, critical_mv);
        MESH_DEBUG_PRINTLN("PWRMGT: Exiting Danger Zone - initiating system reset to re-enable SX1262");

        NRF_POWER->GPREGRET2 = SHUTDOWN_REASON_NONE;
        delay(100);
        NVIC_SystemReset();
      }
    }
  }

  next_check_ms = now_ms + (in_danger_zone ? danger_interval_ms : normal_interval_ms);
}

// Watchdog state
static bool wdt_enabled = false;

// Cooldown timers to prevent interrupt loops and excessive toggling:
// - MPPT writes can trigger BQ25798 interrupts, which wake the task, creating a loop
// - HIZ toggles should be rare events only for stuck PGOOD conditions
static uint32_t lastMpptWriteTime = 0;      // 60-second cooldown for MPPT register writes
static uint32_t lastHizToggleTime = 0;      // 5-minute cooldown for HIZ toggles
#define HIZ_TOGGLE_COOLDOWN_MS (5 * 60 * 1000)  // 5 minutes

/// @brief Initialize and start the hardware watchdog timer
/// @details Configures nRF52 WDT with 600 second timeout for OTA compatibility. Only enabled in release builds.
///          Watchdog continues running during sleep and pauses during debug.
void BoardConfigContainer::setupWatchdog() {
  #ifndef DEBUG_MODE  // Only activate in release builds
    NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) |     // Run during sleep
                      (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);     // Pause during debug
    NRF_WDT->CRV = 32768 * 600;  // 600 seconds (10 min) @ 32.768 kHz - allows OTA updates
    NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;  // Enable reload register 0
    NRF_WDT->TASKS_START = 1;    // Start watchdog
    wdt_enabled = true;
    MESH_DEBUG_PRINTLN("Watchdog enabled: 600s timeout");
    
    // Visual feedback: blink LED 3 times to indicate WDT is active
    #ifdef LED_BLUE
      if (leds_enabled) {
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_BLUE, HIGH);
          delay(100);
          digitalWrite(LED_BLUE, LOW);
          delay(100);
        }
      }
    #endif
  #else
    MESH_DEBUG_PRINTLN("Watchdog disabled (DEBUG_MODE)");
  #endif
}

/// @brief Feed the watchdog timer to prevent system reset
/// @details Should be called regularly from main loop. No-op in debug builds.
void BoardConfigContainer::feedWatchdog() {
  #ifndef DEBUG_MODE
    if (wdt_enabled) {
      NRF_WDT->RR[0] = WDT_RR_RR_Reload;  // Reload watchdog
    }
  #endif
}

/// @brief Disable the watchdog timer (for OTA updates)
/// @details Note: nRF52 WDT cannot be stopped once started. This only sets flag to stop feeding.
void BoardConfigContainer::disableWatchdog() {
  #ifndef DEBUG_MODE
    wdt_enabled = false;  // Stop feeding the watchdog
  #endif
}

/// @brief Detects and fixes stuck PGOOD state by toggling HIZ mode
/// @details When solar voltage rises slowly (sunrise), BQ25798 may not detect input source properly.
///          This function forces input qualification by toggling HIZ mode to reset the input detection.
///          
///          Trigger conditions (all must be true):
///          - PG=0 (PowerGood stuck low)
///          - VBUS sufficient (>3.5V)
///          - PG_STAT flag NOT set (no recent PGOOD change detected)
///          
///          Cooldown mechanism:
///          - Only allows HIZ toggle once per 5 minutes to prevent excessive toggling
///          - When HIZ is toggled, also resets the MPPT write cooldown timer
///          - This allows immediate MPPT re-enable after HIZ toggle when PG goes high
void BoardConfigContainer::checkAndFixPgoodStuck() {
  if (!bqDriverInstance) return;

  bqDriverInstance->readReg(0x22); // Refresh status register

  uint8_t flag0 = bqDriverInstance->readReg(0x1B); // CHARGER_FLAG_0 register
  bool pgFlagSet = (flag0 & 0x08) != 0;            // Bit 3: PG_STAT flag (set when PGOOD changes)
  bool powerGood = bqDriverInstance->getChargerStatusPowerGood();

  // Cooldown: Don't toggle HIZ too frequently (max once per 5 minutes)
  // This prevents excessive toggling when PG remains stuck
  uint32_t currentTime = millis();
  if (lastHizToggleTime != 0 && (currentTime - lastHizToggleTime) < HIZ_TOGGLE_COOLDOWN_MS) {
    // Too soon since last toggle - skip
    return;
  }

  // Get VBUS voltage from telemetry data
  const Telemetry* telem = bqDriverInstance->getTelemetryData();
  uint16_t vbusVoltage = telem ? telem->solar.voltage : 0;

  // Detect stuck PGOOD state: VBUS voltage present but PGOOD not set
  // This can happen after slow solar voltage rise (sunrise over 10-20 minutes)
  // BQ expects fast VBUS edge (adapter plug), not gradual sunrise
  // Without edge detection, input qualification never runs → PGOOD stays low
  //
  // CRITICAL: Only toggle HIZ if:
  // 1. PGOOD is currently LOW (not yet set)
  // 2. VBUS voltage is sufficient for charging
  // 3. PG_STAT flag is NOT set (interrupt was NOT caused by PG change)
  //
  // If PG_STAT flag is set, it means BQ25798 just changed PGOOD itself
  // and we should NOT interfere by toggling HIZ!

  if (!powerGood && vbusVoltage > MIN_VBUS_FOR_CHARGING && !pgFlagSet) {
    MESH_DEBUG_PRINT("PGOOD stuck: VBUS=");
    MESH_DEBUG_PRINT("%d", vbusVoltage);
    MESH_DEBUG_PRINTLN("mV, PG=0 - forcing input detection");

    // Force input source detection by toggling HIZ
    // Per datasheet: Exiting HIZ triggers input source qualification
    bool wasHIZ = bq.getHIZMode();
    if (wasHIZ) {
      // EN_HIZ already set (poor source) - just clear it
      MESH_DEBUG_PRINTLN("EN_HIZ was set - clearing");
      bq.setHIZMode(false);
      delay(200); // Allow HIZ exit to settle
    } else {
      // EN_HIZ not set - toggle to force input re-detection
      MESH_DEBUG_PRINTLN("Toggling HIZ to force input scan");
      bq.setHIZMode(true);
      delay(250); // Give BQ time to process HIZ state
      bq.setHIZMode(false);
    }
    
    // Reset MPPT write cooldown so it can be written immediately when PG=1
    // Don't write MPPT=1 here - let BQ finish input qualification first
    // MPPT=1 will be set by checkAndFixSolarLogic() on next task run when PG=1
    lastMpptWriteTime = 0;
    
    // Set HIZ toggle cooldown timestamp
    lastHizToggleTime = currentTime;

    delay(100); // Allow input qualification to complete
    
    MESH_DEBUG_PRINTLN("Input detection triggered");
  }
}

/// @brief Re-enables MPPT if BQ25798 disabled it (e.g., during !PG state)
/// @details BQ25798 does not persist MPPT=1 and automatically sets MPPT=0 when PG=0.
///          This function restores MPPT=1 when PG returns to 1.
///          
///          CRITICAL: Only runs when PowerGood=1 to avoid false positives and interrupt loops
///          
///          Interrupt loop prevention:
///          - Writing to MPPT register triggers BQ25798 interrupts
///          - Interrupts wake the MPPT task, which may call this function again
///          - Implements 60-second cooldown between MPPT writes to break the loop
///          - Cooldown is reset by checkAndFixPgoodStuck() when HIZ is toggled
void BoardConfigContainer::checkAndFixSolarLogic() {
  if (!bqDriverInstance) return;

  // Check if MPPT is enabled in configuration
  bool mpptEnabled;
  BoardConfigContainer::loadMpptEnabled(mpptEnabled);

  if (!mpptEnabled) {
    // MPPT disabled in config - only disable if currently enabled (avoid unnecessary writes)
    uint8_t mpptVal = bqDriverInstance->readReg(0x15);
    if ((mpptVal & 0x01) != 0) {
      bqDriverInstance->writeReg(0x15, mpptVal & ~0x01);
      MESH_DEBUG_PRINTLN("MPPT disabled via config");
    }
    return;
  }

  // Check if PowerGood is currently set
  bool powerGood = bqDriverInstance->getChargerStatusPowerGood();

  // Only re-enable MPPT when PowerGood=1
  if (!powerGood) {
    // PG=0 - nothing to do, don't set cooldown
    return;
  }

  // Cooldown: Only run every 60 seconds to prevent interrupt loop
  // IMPORTANT: Cooldown only when PG=1, so MPPT is set immediately when PG goes high
  uint32_t currentTime = millis();
  
  if (lastMpptWriteTime != 0 && (currentTime - lastMpptWriteTime) < 60000) {
    // Less than 60 seconds since last run - skip
    return;
  }

  // Re-enable MPPT when PGOOD=1
  uint8_t mpptVal = bqDriverInstance->readReg(0x15);

  if ((mpptVal & 0x01) == 0) {
    bqDriverInstance->writeReg(0x15, mpptVal | 0x01);
    lastMpptWriteTime = currentTime; // Set cooldown timestamp AFTER write
    MESH_DEBUG_PRINTLN("MPPT re-enabled via register");
  }
}

void BoardConfigContainer::solarMpptTask(void* pvParameters) {
  (void)pvParameters;

  delay(2000);
  
  // Initialize MPPT statistics
  // Start with millis() as RTC might not be initialized yet
  memset(&mpptStats, 0, sizeof(MpptStatistics));
  mpptStats.lastUpdateTime = millis() / 1000; // Convert to seconds
  mpptStats.usingRTC = false;

  const TickType_t xBlockTime = pdMS_TO_TICKS(SOLAR_MPPT_TASK_INTERVAL_MS);

  while (true) {
    if (xSemaphoreTake(solarEventSem, xBlockTime) == pdTRUE) {
      // Interrupt triggered - solar event occurred
      // Clear BQ25798 interrupt flags early to de-assert INT line and allow new interrupts
      if (bqDriverInstance) {
        bqDriverInstance->readReg(0x1B); // Read CHARGER_FLAG_0 to clear flags
      }
      delay(100);
      checkAndFixPgoodStuck();  // Check for stuck PGOOD
      checkAndFixSolarLogic();   // Re-enable MPPT if needed
      
      // Update statistics on interrupt (status change) - only if MPPT enabled in config
      if (bqDriverInstance) {
        bool mpptEnabled;
        BoardConfigContainer::loadMpptEnabled(mpptEnabled);
        if (mpptEnabled) {
          updateMpptStats();
        }
      }
    } else {
      // Timeout - periodic check every 15 minutes
      checkAndFixPgoodStuck();  // Check for stuck PGOOD
      checkAndFixSolarLogic();   // Re-enable MPPT if needed
      
      // Update statistics for time accounting - only if MPPT enabled in config
      if (bqDriverInstance) {
        bool mpptEnabled;
        BoardConfigContainer::loadMpptEnabled(mpptEnabled);
        if (mpptEnabled) {
          updateMpptStats();
        }
      }
    }
    
    if (leds_enabled) {
      digitalWrite(LED_BLUE, LOW);
    }
  }
}

void BoardConfigContainer::onBqInterrupt() {
  if (leds_enabled) {
    digitalWrite(LED_BLUE, HIGH);
  }

  if (solarEventSem == NULL) return; // Safety check
  
  // NOTE: Interrupt flag clearing (readReg 0x1B) is deferred to solarMpptTask
  // to avoid I2C bus conflicts in ISR context. The INT line stays asserted until
  // cleared, but the task wakes immediately via semaphore.

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  xSemaphoreGiveFromISR(solarEventSem, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/// @brief Stops all background FreeRTOS tasks
/// @details This must be called before OTA update to prevent task interference
void BoardConfigContainer::stopBackgroundTasks() {
  MESH_DEBUG_PRINTLN("Stopping background tasks for OTA...");
  
  // Detach interrupt first to prevent new events
  detachInterrupt(digitalPinToInterrupt(BQ_INT_PIN));
  MESH_DEBUG_PRINTLN("BQ interrupt detached");
  
  // Delete MPPT task if running
  if (mpptTaskHandle != NULL) {
    vTaskDelete(mpptTaskHandle);
    mpptTaskHandle = NULL;
    MESH_DEBUG_PRINTLN("MPPT task stopped");
  }
  
  // Delete heartbeat task if running
  if (heartbeatTaskHandle != NULL) {
    vTaskDelete(heartbeatTaskHandle);
    heartbeatTaskHandle = NULL;
    MESH_DEBUG_PRINTLN("Heartbeat task stopped");
  }
  
  // Delete voltage monitor task if running (v0.2)
  if (voltageMonitorTaskHandle != NULL) {
    vTaskDelete(voltageMonitorTaskHandle);
    voltageMonitorTaskHandle = NULL;
    MESH_DEBUG_PRINTLN("Voltage monitor task stopped");
  }
  
  // Delete SOC update task if running
  if (socUpdateTaskHandle != NULL) {
    vTaskDelete(socUpdateTaskHandle);
    socUpdateTaskHandle = NULL;
    MESH_DEBUG_PRINTLN("SOC update task stopped");
  }
  
  // Clean up semaphore
  if (solarEventSem != NULL) {
    vSemaphoreDelete(solarEventSem);
    solarEventSem = NULL;
    MESH_DEBUG_PRINTLN("Semaphore deleted");
  }
  
  // NOTE: Don't call Wire.end() here - it can interfere with SoftDevice/BLE
  // The I2C peripheral will be reconfigured if needed after OTA
  
  // Longer delay to ensure all FreeRTOS resources are fully released
  delay(500);
  MESH_DEBUG_PRINTLN("Background cleanup complete");
}

void BoardConfigContainer::heartbeatTask(void* pvParameters) {
  (void)pvParameters;

  pinMode(LED_BLUE, OUTPUT);

  while (true) {
    if (leds_enabled) {
      digitalWrite(LED_BLUE, HIGH);
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms flash - well visible, minimal power
    if (leds_enabled) {
      digitalWrite(LED_BLUE, LOW);
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); // 5s interval - lower power consumption
  }
}

/// @brief Enable or disable heartbeat LED and BQ25798 stat LED
/// @param enabled true = LEDs enabled, false = LEDs disabled
/// @return true on success
bool BoardConfigContainer::setLEDsEnabled(bool enabled) {
  leds_enabled = enabled;
  
  // Save to filesystem
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  prefs.putString("leds_en", enabled ? "1" : "0");
  prefs.end();
  
  // Control heartbeat task
  if (enabled) {
    // Start heartbeat if not running
    if (heartbeatTaskHandle == NULL) {
      xTaskCreate(heartbeatTask, "Heartbeat", 512, NULL, 1, &heartbeatTaskHandle);
    }
  } else {
    // Stop heartbeat task
    if (heartbeatTaskHandle != NULL) {
      vTaskDelete(heartbeatTaskHandle);
      heartbeatTaskHandle = NULL;
      // Turn off LED
      pinMode(LED_BLUE, OUTPUT);
      digitalWrite(LED_BLUE, LOW);
    }
  }
  
  // Control BQ25798 STAT LED (only if BQ is initialized)
  if (BQ_INITIALIZED && bqDriverInstance) {
    bqDriverInstance->setStatPinEnable(enabled);
  }
  
  return true;
}

/// @brief Get current LED enable state
/// @return true if LEDs are enabled
bool BoardConfigContainer::getLEDsEnabled() const {
  return leds_enabled;
}

/// @brief Updates MPPT statistics based on elapsed time and current status
/// Should be called when MPPT status changes or periodically for time accounting
void BoardConfigContainer::updateMpptStats() {
  if (!bqDriverInstance) return;
  
  static bool lastMpptStatus = false;
  static bool initialized = false;
  
  // Get current time - prefer RTC, fallback to millis()
  uint32_t currentTime;
  uint32_t rtcTime = getRTCTime();
  
  // Check if RTC is initialized (returns > 0 if time was set)
  // AutoDiscoverRTCClock returns 0 if no RTC found and time not set
  if (rtcTime > 1000000000) { // Sanity check: After year 2001
    currentTime = rtcTime;
    if (!mpptStats.usingRTC) {
      // Switch from millis to RTC
      mpptStats.usingRTC = true;
      mpptStats.lastUpdateTime = currentTime;
      lastMpptStatus = bqDriverInstance->getMPPTenable();
      initialized = true;
      return; // Reset timing on switch
    }
  } else {
    // RTC not available or not set - use millis() in seconds
    currentTime = millis() / 1000;
  }
  
  bool currentMpptStatus = bqDriverInstance->getMPPTenable();
  
  // Initialize on first run
  if (!initialized) {
    mpptStats.lastUpdateTime = currentTime;
    lastMpptStatus = currentMpptStatus;
    initialized = true;
    return;
  }
  
  // Calculate elapsed time since last update
  uint32_t elapsedSeconds = currentTime - mpptStats.lastUpdateTime;
  
  // Sanity check: If more than 48 hours passed, reset
  const uint32_t MAX_INTERVAL_SEC = 48UL * 60UL * 60UL;
  if (elapsedSeconds > MAX_INTERVAL_SEC) {
    mpptStats.lastUpdateTime = currentTime;
    lastMpptStatus = currentMpptStatus;
    return;
  }
  
  uint32_t elapsedMinutes = elapsedSeconds / 60;
  
  if (elapsedMinutes == 0 && lastMpptStatus == currentMpptStatus) {
    return; // No time passed and no status change
  }
  
  // Add time to current hour accumulator if MPPT was enabled
  if (lastMpptStatus && elapsedMinutes > 0) {
    mpptStats.currentHourMinutes += elapsedMinutes;
    if (mpptStats.currentHourMinutes > 60) {
      mpptStats.currentHourMinutes = 60; // Cap at 60 minutes per hour
    }
  }
  
  // Calculate energy harvested since last update if MPPT was enabled
  if (lastMpptStatus && elapsedSeconds > 0) {
    // Use last measured power and integrate over time: E = P × t
    // Energy in mWh = Power in mW × Time in hours
    float hours = elapsedSeconds / 3600.0f;
    uint32_t energy_mWh = (uint32_t)(mpptStats.lastPower_mW * hours);
    mpptStats.currentHourEnergy_mWh += energy_mWh;
  }
  
  // Sample current solar power for next integration period
  if (currentMpptStatus) {
    const Telemetry* telem = bqDriverInstance->getTelemetryData();
    if (telem) {
      // Calculate power: P = U * I (both in mV and mA, result in mW)
      mpptStats.lastPower_mW = (int32_t)telem->solar.voltage * telem->solar.current / 1000;
    }
  } else {
    mpptStats.lastPower_mW = 0; // No power when MPPT disabled
  }
  
  mpptStats.lastUpdateTime = currentTime;
  lastMpptStatus = currentMpptStatus;
  
  // Check if we need to move to the next hour
  static uint32_t lastHourCheck = 0;
  uint32_t currentHour = currentTime / 3600;
  uint32_t lastHour = lastHourCheck / 3600;
  
  if (currentHour > lastHour) {
    // Store the completed hour's data
    mpptStats.hours[mpptStats.currentIndex].mpptEnabledMinutes = mpptStats.currentHourMinutes;
    mpptStats.hours[mpptStats.currentIndex].timestamp = currentTime;
    mpptStats.hours[mpptStats.currentIndex].harvestedEnergy_mWh = mpptStats.currentHourEnergy_mWh;
    
    // Move to next index (circular buffer)
    mpptStats.currentIndex = (mpptStats.currentIndex + 1) % MPPT_STATS_HOURS;
    
    // Reset for new hour
    mpptStats.currentHourMinutes = 0;
    mpptStats.currentHourEnergy_mWh = 0;
    lastHourCheck = currentTime;
  }
}

/// @brief Returns current max charge current as string
/// @return Static string buffer with charge current in mA
const char* BoardConfigContainer::getChargeCurrentAsStr() {
  static char buffer[16];
  snprintf(buffer, sizeof(buffer), "%dmA", this->getMaxChargeCurrent_mA());
  return buffer;
}

/// @brief Writes charger status information into provided buffer
/// @param buffer Destination buffer for status string
/// @param bufferSize Size of destination buffer
void BoardConfigContainer::getChargerInfo(char* buffer, uint32_t bufferSize) {
  // Check if buffer is valid
  if (!buffer || bufferSize == 0) {
    return;
  }
  
  // Clear buffer to prevent garbage data
  memset(buffer, 0, bufferSize);
  
  // Check if BQ25798 is initialized and responsive
  if (!BQ_INITIALIZED) {
    snprintf(buffer, bufferSize, "BQ25798 not initialized");
    return;
  }
  
  const char* powerGood = bq.getChargerStatusPowerGood() ? "PG" : "!PG";
  const char* statusString = "Unknown";  // Initialize with default value
  bq25798_charging_status status = bq.getChargingStatus();
  
  switch (status) {
  case bq25798_charging_status::BQ25798_CHARGER_STATE_NOT_CHARGING: {
    statusString = "!CHG";
    break;
  }
  case bq25798_charging_status::BQ25798_CHARGER_STATE_PRE_CHARGING: {
    statusString = "PRE";
    break;
  }
  case bq25798_charging_status::BQ25798_CHARGER_STATE_CC_CHARGING: {
    statusString = "CC";
    break;
  }
  case bq25798_charging_status::BQ25798_CHARGER_STATE_CV_CHARGING: {
    statusString = "CV";
    break;
  }
  case bq25798_charging_status::BQ25798_CHARGER_STATE_TRICKLE_CHARGING: {
    statusString = "TRICKLE";
    break;
  }
  case bq25798_charging_status::BQ25798_CHARGER_STATE_TOP_OF_TIMER_ACTIVE_CHARGING: {
    statusString = "TOP";
    break;
  }
  case bq25798_charging_status::BQ25798_CHARGER_STATE_DONE_CHARGING: {
    statusString = "DONE";
    break;
  }
  default:
    statusString = "Unknown";
    break;
  }
  
  // Show time since last automatic HIZ toggle (from checkAndFixPgoodStuck)
  if (lastHizToggleTime == 0) {
    snprintf(buffer, bufferSize, "%s / %s HIZ:never", powerGood, statusString);
  } else {
    uint32_t agoSec = (millis() - lastHizToggleTime) / 1000;
    if (agoSec < 60) {
      snprintf(buffer, bufferSize, "%s / %s HIZ:%ds ago", powerGood, statusString, agoSec);
    } else if (agoSec < 3600) {
      snprintf(buffer, bufferSize, "%s / %s HIZ:%dm ago", powerGood, statusString, agoSec / 60);
    } else {
      snprintf(buffer, bufferSize, "%s / %s HIZ:%dh%dm ago", powerGood, statusString, agoSec / 3600, (agoSec % 3600) / 60);
    }
  }
}

/// @brief Manual HIZ toggle with status report for debugging stuck PGOOD
/// @param buffer Destination buffer for status string
/// @param bufferSize Size of destination buffer
void BoardConfigContainer::toggleHizAndCheck(char* buffer, uint32_t bufferSize) {
  if (!buffer || bufferSize == 0 || !BQ_INITIALIZED || !bqDriverInstance) {
    return;
  }

  memset(buffer, 0, bufferSize);

  // Read current HIZ state (Register 0x0F Bit 2)
  uint8_t reg0F = bqDriverInstance->readReg(0x0F);
  bool hizBefore = (reg0F & 0x04) != 0;

  // Toggle HIZ
  if (hizBefore) {
    bqDriverInstance->writeReg(0x0F, reg0F & ~0x04); // Clear HIZ
  } else {
    bqDriverInstance->writeReg(0x0F, reg0F | 0x04); // Set HIZ
  }

  delay(100); // Allow registers to settle

  // Re-read HIZ state
  reg0F = bqDriverInstance->readReg(0x0F);
  bool hizAfter = (reg0F & 0x04) != 0;

  // Check PGOOD
  bool powerGood = bqDriverInstance->getChargerStatusPowerGood();
  const Telemetry* telem = getTelemetryData();
  float vbusVoltage = telem ? telem->solar.voltage : 0;

  // Check if toggle was successful
  bool success = (hizBefore != hizAfter);
  const char* result = success ? (powerGood ? "OK" : "!PG") : "FAIL";

  // Update HIZ toggle timestamp so board.cinfo reflects manual toggles too
  if (success) {
    lastHizToggleTime = millis();
  }

  snprintf(buffer, bufferSize, "HIZ toggled: VBUS=%.1fV PG=%s", vbusVoltage / 1000.0f, result);
}

/// @brief Get detailed BQ25798 diagnostics for debugging PG / !CHG issues
/// @param buffer Destination buffer for diagnostics string
/// @param bufferSize Size of destination buffer
void BoardConfigContainer::getDetailedDiagnostics(char* buffer, uint32_t bufferSize) {
  if (!buffer || bufferSize == 0) {
    return;
  }

  memset(buffer, 0, bufferSize);

  if (!BQ_INITIALIZED || !bqDriverInstance) {
    snprintf(buffer, bufferSize, "BQ25798 not initialized");
    return;
  }

  // Refresh ADC/status registers before reading (important for accurate readings)
  bqDriverInstance->readReg(0x22); // Trigger status refresh
  delay(10);                        // Allow registers to update

  // Read all relevant status and flag registers
  // Verified against BQ25798 datasheet (bq25798en.txt)
  uint8_t reg0F = bqDriverInstance->readReg(0x0F); // CHARGER_CONTROL_0 (EN_HIZ at bit 2, EN_CHG at bit 5)
  uint8_t reg15 = bqDriverInstance->readReg(0x15); // MPPT_CONTROL (EN_MPPT at bit 0, VOC config bits 7-1)
  uint8_t reg1B = bqDriverInstance->readReg(0x1B); // CHARGER_STATUS_0 (PG_STAT bit 3, VINDPM bit 6, IINDPM bit 7)
  uint8_t reg1C = bqDriverInstance->readReg(0x1C); // CHARGER_STATUS_1 (CHG_STAT bits 7:5, VBUS_STAT bits 4:1)
  uint8_t reg1F = bqDriverInstance->readReg(0x1F); // CHARGER_STATUS_4 (TS status bits 3:0)

  // Extract key status bits (verified against datasheet Section 9.5.1)
  bool hiz_enabled = (reg0F & 0x04) != 0;   // Bit 2: EN_HIZ in REG0F
  bool ce_disabled = (reg0F & 0x20) == 0;   // Bit 5: EN_CHG in REG0F (1=enabled, 0=disabled)
  bool mppt_enabled = (reg15 & 0x01) != 0;  // Bit 0: EN_MPPT in REG15
  uint8_t voc_pct = (reg15 >> 5) & 0x07;    // Bits 7-5: VOC_PCT in REG15
  uint8_t voc_dly = (reg15 >> 3) & 0x03;    // Bits 4-3: VOC_DLY in REG15
  uint8_t voc_rate = (reg15 >> 1) & 0x03;   // Bits 2-1: VOC_RATE in REG15
  bool powerGood = (reg1B & 0x08) != 0;     // Bit 3: PG_STAT in REG1B
  bool iindpm = (reg1B & 0x80) != 0;        // Bit 7: IINDPM_STAT in REG1B
  bool vindpm = (reg1B & 0x40) != 0;        // Bit 6: VINDPM_STAT in REG1B
  uint8_t chg_stat = (reg1C >> 5) & 0x07;   // Bits 7:5: CHG_STAT in REG1C
  uint8_t vbus_stat = (reg1C >> 1) & 0x0F;  // Bits 4:1: VBUS_STAT in REG1C
  bool ts_cold = (reg1F & 0x08) != 0;       // Bit 3: TS_COLD_STAT in REG1F
  bool ts_cool = (reg1F & 0x04) != 0;       // Bit 2: TS_COOL_STAT in REG1F
  bool ts_warm = (reg1F & 0x02) != 0;       // Bit 1: TS_WARM_STAT in REG1F
  bool ts_hot = (reg1F & 0x01) != 0;        // Bit 0: TS_HOT_STAT in REG1F

  // VBUS_STAT interpretation (4 bits: 0h-Fh from REG1C bits 4:1)
  const char* vbus_str;
  switch (vbus_stat) {
  case 0x0:
    vbus_str = "NoIn";
    break; // No input
  case 0x1:
    vbus_str = "SDP";
    break; // USB SDP (500mA)
  case 0x2:
    vbus_str = "CDP";
    break; // USB CDP (1.5A)
  case 0x3:
    vbus_str = "DCP";
    break; // USB DCP (3.25A)
  case 0x4:
    vbus_str = "HVDCP";
    break; // High Voltage DCP
  case 0x5:
    vbus_str = "UnkAdp";
    break; // Unknown adapter
  case 0x6:
    vbus_str = "NStd";
    break; // Non-standard adapter
  case 0x7:
    vbus_str = "OTG";
    break; // OTG mode
  case 0x8:
    vbus_str = "NotQual";
    break; // Not qualified adapter
  case 0xB:
    vbus_str = "DirPwr";
    break; // Direct VBUS power
  case 0xC:
    vbus_str = "Backup";
    break; // Backup mode
  default:
    vbus_str = "Rsv";
    break; // Reserved
  }

  // CHG_STAT interpretation
  const char* chg_str;
  switch (chg_stat) {
  case 0:
    chg_str = "!CHG";
    break;
  case 1:
    chg_str = "TRKL";
    break;
  case 2:
    chg_str = "PRE";
    break;
  case 3:
    chg_str = "CC";
    break;
  case 4:
    chg_str = "CV";
    break;
  case 6:
    chg_str = "TOP";
    break;
  case 7:
    chg_str = "DONE";
    break;
  default:
    chg_str = "???";
    break;
  }

  // VOC_PCT decoding (REG15 bits 7-5)
  const char* voc_pct_str;
  switch (voc_pct) {
  case 0: voc_pct_str = "56.25%"; break;
  case 1: voc_pct_str = "62.5%"; break;
  case 2: voc_pct_str = "68.75%"; break;
  case 3: voc_pct_str = "75%"; break;
  case 4: voc_pct_str = "81.25%"; break;
  case 5: voc_pct_str = "87.5%"; break;  // Default
  case 6: voc_pct_str = "93.75%"; break;
  case 7: voc_pct_str = "100%"; break;
  default: voc_pct_str = "???"; break;
  }
  
  // VOC_DLY decoding (REG15 bits 4-3)
  const char* voc_dly_str;
  switch (voc_dly) {
  case 0: voc_dly_str = "50ms"; break;
  case 1: voc_dly_str = "300ms"; break;  // Default
  case 2: voc_dly_str = "2s"; break;
  case 3: voc_dly_str = "5s"; break;
  default: voc_dly_str = "???"; break;
  }
  
  // VOC_RATE decoding (REG15 bits 2-1)
  const char* voc_rate_str;
  switch (voc_rate) {
  case 0: voc_rate_str = "30s"; break;
  case 1: voc_rate_str = "2min"; break;  // Default
  case 2: voc_rate_str = "10min"; break;
  case 3: voc_rate_str = "30min"; break;
  default: voc_rate_str = "???"; break;
  }

  // Get telemetry for voltage/current
  const Telemetry* telem = getTelemetryData();
  float vbus_v = telem ? telem->solar.voltage / 1000.0f : 0.0f;
  float vbat_v = telem ? telem->batterie.voltage / 1000.0f : 0.0f;
  float ibat_ma = telem ? telem->batterie.current : 0.0f;  // INA228 driver returns correctly signed values
  float temp_c = telem ? telem->batterie.temperature : 0.0f;
  int32_t ibat_ma_int = (int32_t)((ibat_ma >= 0.0f) ? (ibat_ma + 0.5f) : (ibat_ma - 0.5f));

  // Build compact diagnostic string (must fit in 100 bytes)
  const char* ts_str = ts_cold ? "COLD" : (ts_cool ? "COOL" : (ts_warm ? "WARM" : (ts_hot ? "HOT" : "OK")));
  snprintf(buffer, bufferSize,
           "PG%d CE%d HZ%d MP%d %s %s VD%d ID%d|%.1f/%.1fV %dmA %.0fC %s|%02X/%02X VOC%s/%s/%s",
           powerGood, !ce_disabled, hiz_enabled, mppt_enabled, chg_str, vbus_str, vindpm, iindpm, vbus_v,
           vbat_v, ibat_ma_int, temp_c, ts_str, reg0F, reg15, voc_pct_str, voc_dly_str, voc_rate_str);
}

/// @brief Initializes battery manager, potentiometer, preferences, and MPPT task
/// @return true if both BQ25798 and MCP4652 initialized successfully
bool BoardConfigContainer::begin() {
  // Initialize LEDs early for boot sequence visualization
  pinMode(LED_BLUE, OUTPUT);  // Blue LED (P1.03)
  pinMode(LED_RED, OUTPUT);   // Red LED (P1.04)
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_RED, LOW);
  
  // Load LED enable state from filesystem (default: enabled)
  SimplePreferences prefs_led;
  if (prefs_led.begin("inheromr2")) {
    char led_buffer[8];
    prefs_led.getString("leds_en", led_buffer, sizeof(led_buffer), "1");
    leds_enabled = (strcmp(led_buffer, "1") == 0);
    prefs_led.end();
  } else {
    leds_enabled = true;  // Default: enabled
  }

  bool skip_fs_writes = ((NRF_POWER->GPREGRET2 & 0x03) == SHUTDOWN_REASON_LOW_VOLTAGE);
  
  // === MR2 Hardware (v0.2): INA228 Power Monitor with UVLO ===
  // MR2 uses INA228 at 0x40 (A0=GND, A1=GND)
  MESH_DEBUG_PRINTLN("=== INA228 Detection @ 0x40 ===");
  delay(10);  // Let serial output flush
  
  // Visual indicator: Red LED on = INA228 detection in progress
  if (leds_enabled) {
    digitalWrite(LED_RED, HIGH);
    delay(50);
  }
  
  // First test I2C communication
  Wire.beginTransmission(0x40);
  uint8_t i2c_result = Wire.endTransmission();
  MESH_DEBUG_PRINTLN("INA228: I2C probe result = %d (0=OK)", i2c_result);
  delay(10);
  
  if (i2c_result == 0) {
    // Device responds, read ID registers
    Wire.beginTransmission(0x40);
    Wire.write(0x3E);  // Manufacturer ID register
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
    if (Wire.available() >= 2) {
      uint16_t mfg_id = (Wire.read() << 8) | Wire.read();
      MESH_DEBUG_PRINTLN("INA228: MFG_ID = 0x%04X (expect 0x5449)", mfg_id);
      delay(10);
    }
    
    Wire.beginTransmission(0x40);
    Wire.write(0x3F);  // Device ID register
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
    if (Wire.available() >= 2) {
      uint16_t dev_id = (Wire.read() << 8) | Wire.read();
      MESH_DEBUG_PRINTLN("INA228: DEV_ID = 0x%04X (expect 0x0228)", dev_id);
      delay(10);
    }
    
    // Try to initialize
    if (ina228.begin(100.0f)) {  // 100mΩ shunt resistor (optimal SNR for 10mA standby / 1A max)
      INA228_INITIALIZED = true;
      ina228DriverInstance = &ina228;
      
      // Turn off red LED (INA228 detection complete)
      if (leds_enabled) {
        digitalWrite(LED_RED, LOW);
        delay(10);
      }
      
      // Blue LED flash: INA228 initialized
      if (leds_enabled) {
        digitalWrite(LED_BLUE, HIGH);
        delay(150);
        digitalWrite(LED_BLUE, LOW);
        delay(100);
      }
      
      // Load and apply current calibration factor (gain)
      float calib_factor = 1.0f;
      if (loadIna228CalibrationFactor(calib_factor)) {
        ina228.setCalibrationFactor(calib_factor);
        MESH_DEBUG_PRINTLN("INA228 calibration factor loaded: %.4f", calib_factor);
      } else {
        MESH_DEBUG_PRINTLN("INA228 using default calibration (1.0)");
      }
      
      // Load and apply current offset correction (additive)
      float current_offset = 0.0f;
      if (loadIna228CurrentOffset(current_offset)) {
        ina228.setCurrentOffset(current_offset);
        MESH_DEBUG_PRINTLN("INA228 current offset loaded: %+.2f mA", current_offset);
      } else {
        MESH_DEBUG_PRINTLN("INA228 using default offset (0.0)");
      }
      delay(10);
      
      // Load battery type to set chemistry-specific UVLO threshold
      BatteryType bat;
      if (!loadBatType(bat)) {
        bat = DEFAULT_BATTERY_TYPE;
      }
      
      // INA228 UVLO Alert: Load from preferences
      // Purpose: Catastrophic battery protection - permanent shutdown if battery critically low
      // Latch mode: Alert stays LOW permanently (INA228 powered by battery, not 3.3V rail)
      // Recovery: Requires PHYSICAL battery disconnect to reset INA228 registers
      // Note: applyUvloSetting() will load from preferences and set chemistry-specific threshold
      applyUvloSetting();
    } else {
      MESH_DEBUG_PRINTLN("✗ INA228 begin() failed (check MFG_ID/DEV_ID above)");
      INA228_INITIALIZED = false;
    }
  } else {
    MESH_DEBUG_PRINTLN("✗ INA228 no I2C ACK @ 0x40");
    INA228_INITIALIZED = false;
  }
  delay(10);

  // Initialize BQ25798 (common for both v0.1 and v0.2)
  if (bq.begin()) {
    BQ_INITIALIZED = true;
    bqDriverInstance = &bq;
    MESH_DEBUG_PRINTLN("BQ25798 found. ");

    // Blue LED flash: BQ25798 initialized
    if (leds_enabled) {
      digitalWrite(LED_BLUE, HIGH);
      delay(150);
      digitalWrite(LED_BLUE, LOW);
      delay(100);
    }
  } else {
    MESH_DEBUG_PRINTLN("BQ25798 not found.");
    BQ_INITIALIZED = false;
  }
  
  // Load NTC temperature calibration offset (applies to all BQ temperature readings)
  float tc_offset = 0.0f;
  if (loadTcCalOffset(tc_offset)) {
    tcCalOffset = tc_offset;
    MESH_DEBUG_PRINTLN("TC calibration offset loaded: %+.2f C", tc_offset);
  } else {
    MESH_DEBUG_PRINTLN("TC using default calibration (0.0)");
  }
  
  // === RV-3028 RTC Initialization (v0.2) ===
  bool rtc_initialized = false;
  Wire.beginTransmission(0x52);  // RV-3028 I2C address
  if (Wire.endTransmission() == 0) {
    rtc_initialized = true;
    MESH_DEBUG_PRINTLN("RV-3028 RTC found @ 0x52");
    
    // Blue LED flash: RTC initialized
    if (leds_enabled) {
      digitalWrite(LED_BLUE, HIGH);
      delay(150);
      digitalWrite(LED_BLUE, LOW);
      delay(100);
    }
  } else {
    MESH_DEBUG_PRINTLN("RV-3028 RTC not found @ 0x52");
  }
  
  // === MR2 Configuration (v0.2 only) ===
  SimplePreferences prefs_init;
  prefs_init.begin(PREFS_NAMESPACE);
  
  BatteryType bat = DEFAULT_BATTERY_TYPE;
  FrostChargeBehaviour frost = DEFAULT_FROST_BEHAVIOUR;
  uint16_t maxChargeCurrent_mA = DEFAULT_MAX_CHARGE_CURRENT_MA;

  if (!loadBatType(bat)) {
    if (!skip_fs_writes) {
      prefs_init.putString(BATTKEY, getBatteryTypeCommandString(bat));
    }
  }
  if (!loadFrost(frost)) {
    if (!skip_fs_writes) {
      prefs_init.putString(FROSTKEY, getFrostChargeBehaviourCommandString(frost));
    }
  }
  if (!loadMaxChrgI(maxChargeCurrent_mA)) {
    if (!skip_fs_writes) {
      prefs_init.putInt(MAXCHARGECURRENTKEY, maxChargeCurrent_mA);
    }
  }

  this->configureBaseBQ();
  this->configureChemistry(bat);
  
  // MR2 (v0.2) doesn't use MCP4652 - only v0.1 hardware
  // this->configureMCP(bat);  // Commented out for MR2
  
  this->setFrostChargeBehaviour(frost);
  this->setMaxChargeCurrent_mA(maxChargeCurrent_mA);

  pinMode(BQ_INT_PIN, INPUT_PULLUP);
  if (solarEventSem == NULL) {
    solarEventSem = xSemaphoreCreateBinary();
    if (solarEventSem == NULL) {
      MESH_DEBUG_PRINTLN("Failed to create solarEventSem!");
      return false;
    }
  }

  this->configureSolarOnlyInterrupts();
  
  // Clear any latched fault status from previous operation/boot
  // This ensures we start with clean state after full configuration
  bq.readReg(0x20); // FAULT_STATUS_0
  bq.readReg(0x21); // FAULT_STATUS_1

  if (mpptTaskHandle == NULL) {
    BaseType_t taskCreated = xTaskCreate(BoardConfigContainer::solarMpptTask, "SolarDaemon", 4096, NULL, 1, &mpptTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create MPPT task!");
      return false;
    }
  }

  if (heartbeatTaskHandle == NULL && leds_enabled) {
    BaseType_t taskCreated = xTaskCreate(BoardConfigContainer::heartbeatTask, "Heartbeat", 1024, NULL, 1, &heartbeatTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create Heartbeat task!");
      return false;
    }
  }
  
  // NOTE: Voltage Monitor Task is started AFTER INA228 initialization (below)
  // to avoid I2C bus conflicts during hardware setup

  attachInterrupt(digitalPinToInterrupt(BQ_INT_PIN), onBqInterrupt, FALLING);

  // Check if all critical components initialized
  bool all_components_ok = BQ_INITIALIZED && INA228_INITIALIZED && rtc_initialized;
  
  if (!all_components_ok) {
    // Start permanent slow red LED blink to indicate missing component
    MESH_DEBUG_PRINTLN("⚠️ Missing components - starting error LED");
    if (!BQ_INITIALIZED) MESH_DEBUG_PRINTLN("  - BQ25798 missing");
    if (!INA228_INITIALIZED) MESH_DEBUG_PRINTLN("  - INA228 missing");
    if (!rtc_initialized) MESH_DEBUG_PRINTLN("  - RV-3028 RTC missing");
    
    // Create error LED blink task
    if (leds_enabled) {
      xTaskCreate([](void* param) {
        while (1) {
          digitalWrite(LED_RED, HIGH);  // Red LED on
          vTaskDelay(pdMS_TO_TICKS(500));
          digitalWrite(LED_RED, LOW);   // Red LED off
          vTaskDelay(pdMS_TO_TICKS(500));
        }
      }, "ErrorLED", 512, NULL, 1, NULL);
    }
  }
  
  // Voltage monitoring is handled inside socUpdateTask (single periodic loop).
  
  // Start SOC Update Task (runs every minute)
  if (socUpdateTaskHandle == NULL && INA228_INITIALIZED) {
    BaseType_t taskCreated = xTaskCreate(BoardConfigContainer::socUpdateTask, "SOCUpdate", 2048, NULL, 2, &socUpdateTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create SOC Update task!");
      // Non-critical, continue
    } else {
      MESH_DEBUG_PRINTLN("SOC Update task started - running every minute");
    }
  }
  
  // CRITICAL: Verify and fix ADC_CONFIG after all tasks are created
  // The voltageMonitorTask may have preempted and overwritten ADC_CONFIG
  if (INA228_INITIALIZED) {
    delay(50);  // Let tasks settle
    
    Wire.beginTransmission(0x40);
    Wire.write(0x01);  // ADC_CONFIG register
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)0x40, (uint8_t)2);
    uint16_t adc_cfg_check = 0;
    if (Wire.available() >= 2) {
      adc_cfg_check = (Wire.read() << 8) | Wire.read();
    }
    
    if (adc_cfg_check != 0xFFCA) {
      MESH_DEBUG_PRINTLN("⚠ INA228 ADC_CONFIG=0x%04X after task start - fixing...", adc_cfg_check);
      
      // Visual: Rapid red blinks = ADC_CONFIG corrupted by task race
      if (leds_enabled) {
        for (int i = 0; i < 3; i++) {
          digitalWrite(LED_RED, HIGH);
          delay(100);
          digitalWrite(LED_RED, LOW);
          delay(100);
        }
      }
      
      // Force-write ADC_CONFIG again
      Wire.beginTransmission(0x40);
      Wire.write(0x01);
      Wire.write(0xFF);
      Wire.write(0xCA);  // 0xFFCA: Long conv times + 16 avg
      Wire.endTransmission();
      delay(10);
      
      MESH_DEBUG_PRINTLN("INA228 ADC_CONFIG restored to 0xFFCA");
    }
  }
  
  // MR2 requires BQ25798 + INA228 (RTC is optional for basic operation)
  return BQ_INITIALIZED && INA228_INITIALIZED;
}

/// @brief Loads battery type from preferences
/// @param type Reference to store loaded battery type
/// @return true if preference found and valid, false if default used
bool BoardConfigContainer::loadBatType(BatteryType& type) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[10];
  if (prefs.getString(BATTKEY, buffer, sizeof(buffer), "") > 0) {
    type = this->getBatteryTypeFromCommandString(buffer);
    if (type != BAT_UNKNOWN) {
      return true;
    } else {
      type = DEFAULT_BATTERY_TYPE;
      return false;
    }
  }
  
  // No preference found - use default
  type = DEFAULT_BATTERY_TYPE;
  return false;
}

/// @brief Loads frost charge behavior from preferences
/// @param behaviour Reference to store loaded behavior
/// @return true if preference found and valid, false if default used
bool BoardConfigContainer::loadFrost(FrostChargeBehaviour& behaviour) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[10];
  if (prefs.getString(FROSTKEY, buffer, sizeof(buffer), "") > 0) {
    behaviour = this->getFrostChargeBehaviourFromCommandString(buffer);
    if (behaviour != REDUCE_UNKNOWN) {
      return true;
    } else {
      behaviour = DEFAULT_FROST_BEHAVIOUR;
      return false;
    }
  }
  
  // No preference found - use default
  behaviour = DEFAULT_FROST_BEHAVIOUR;
  return false;
}

/// @brief Loads maximum charge current from preferences
/// @param maxCharge_mA Reference to store loaded current in mA
/// @return true if preference found and valid (1-3000mA), false if default used
bool BoardConfigContainer::loadMaxChrgI(uint16_t& maxCharge_mA) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[10];

  if (prefs.getString(MAXCHARGECURRENTKEY, buffer, sizeof(buffer), "") > 0) {

    int val = atoi(buffer);
    // Bounds check: Reasonable charge current range
    if (val > 0 && val <= 3000) {  // Max 3A for safety
      maxCharge_mA = val;
      return true;
    } else {
      maxCharge_mA = DEFAULT_MAX_CHARGE_CURRENT_MA;
      return false;
    }
  }
  
  // No preference found - use default
  maxCharge_mA = DEFAULT_MAX_CHARGE_CURRENT_MA;
  return false;
}

/// @brief Loads MPPT enabled setting from preferences
/// @param enabled Reference to store loaded setting
/// @return true if preference found, false if default used
bool BoardConfigContainer::loadMpptEnabled(bool& enabled) {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[10];

  if (prefs.getString("mpptEn", buffer, sizeof(buffer), "") > 0) {
    if (buffer[0] != '\0') {
      enabled = buffer[0] == '1' ? true : false;
      return true;
    } else {
      enabled = DEFAULT_MPPT_ENABLED;
      return false;
    }
  }
  
  // No preference found - use default
  enabled = DEFAULT_MPPT_ENABLED;
  return false;
}

/// @brief Returns pointer to current telemetry data (MR2: INA228 for VBAT/IBAT, BQ25798 for Solar)
/// @return Pointer to Telemetry struct with combined data from INA228 and BQ25798
/// @note MR2 v0.2: Battery voltage/current from INA228 (24-bit ADC, ±0.1% accuracy)
///                  Solar data and battery temperature from BQ25798 ADC
const Telemetry* BoardConfigContainer::getTelemetryData() {
  static Telemetry telemetry;
  
  // Battery voltage/current ALWAYS from INA228 (no fallback to BQ25798)
  // MR2 v0.2 hardware uses INA228 for precise battery monitoring
  uint16_t batt_voltage = 0;
  float batt_current = 0.0f;
  int32_t batt_power = 0;
  if (ina228DriverInstance != nullptr) {
    batt_voltage = ina228DriverInstance->readVoltage_mV();
    batt_current = ina228DriverInstance->readCurrent_mA_precise();
    batt_power = (int32_t)((batt_voltage * batt_current) / 1000.0f);
  }

  // Get base telemetry from BQ25798 (solar data + temperature)
  const Telemetry* bqData = bq.getTelemetryData();
  if (!bqData) {
    memset(&telemetry, 0, sizeof(Telemetry));
    return &telemetry;
  }

  // Copy BQ25798 data (solar, system, temperature with calibration offset)
  telemetry.solar = bqData->solar;
  telemetry.system = bqData->system;
  telemetry.batterie.temperature = bqData->batterie.temperature + tcCalOffset;

  telemetry.batterie.voltage = batt_voltage;
  telemetry.batterie.current = batt_current;
  telemetry.batterie.power = batt_power;

  return &telemetry;
}

/// @brief Resets BQ25798 to default register values and reconfigures
/// @return true if reset and reconfiguration successful
bool BoardConfigContainer::resetBQ() {
  if (!BQ_INITIALIZED) {
    return false;
  }
  
  MESH_DEBUG_PRINTLN("Resetting BQ25798 to defaults...");
  
  // Software reset via REG_RST bit - resets all registers to default
  bool success = bq.reset();
  if (!success) {
    MESH_DEBUG_PRINTLN("BQ reset failed");
    return false;
  }
  
  delay(100); // Allow reset to complete
  
  MESH_DEBUG_PRINTLN("Reconfiguring BQ25798 from stored preferences...");
  
  // Reload configuration from preferences (not current values!)
  prefs.begin(PREFS_NAMESPACE);
  BatteryType bat;
  FrostChargeBehaviour frost;
  uint16_t maxChargeCurrent_mA;
  
  if (!loadBatType(bat)) {
    bat = DEFAULT_BATTERY_TYPE;  // Safe: no charging until user configures
  }
  if (!loadFrost(frost)) {
    frost = DEFAULT_FROST_BEHAVIOUR;  // Safe: no frost charging
  }
  if (!loadMaxChrgI(maxChargeCurrent_mA)) {
    maxChargeCurrent_mA = DEFAULT_MAX_CHARGE_CURRENT_MA;
  }
  
  // Apply configuration
  configureBaseBQ();
  configureChemistry(bat);
  // MR2 doesn't use MCP4652 (v0.1 only)
  if (bat != BatteryType::LTO_2S) {
    setFrostChargeBehaviour(frost);
  }
  setMaxChargeCurrent_mA(maxChargeCurrent_mA);
  
  // Re-enable interrupts
  bq.configureSolarOnlyInterrupts();
  
  MESH_DEBUG_PRINTLN("BQ25798 reset and reconfiguration complete");
  return true;
}

/// @brief Configures base BQ25798 settings (timers, watchdog, input limits, MPPT)
/// @return true if BQ initialized and configuration successful
bool BoardConfigContainer::configureBaseBQ() {
  if (!BQ_INITIALIZED) {
    return false;
  }

  bq.setRechargeThreshOffsetV(.2);
  bq.setPrechargeTimerEnable(false);
  bq.setFastChargeTimerEnable(false);
  bq.setTsIgnore(false);
  bq.setWDT(BQ25798_WDT_DISABLE);
  bq.setInputLimitA(1);

  bq.setVOCdelay(BQ25798_VOC_DLY_2S);
  bq.setVOCrate(BQ25798_VOC_RATE_2MIN);
  bq.setAutoDPinsDetection(false);
  bq.setMPPTenable(true);

  bq.setMinSystemV(2.75);  // 2.75V = next valid step above 2.7V (250mV steps: 2.5, 2.75, 3.0...)
  bq.setStatPinEnable(leds_enabled);  // Configure STAT LED based on user preference
  bq.setTsCool(BQ25798_TS_COOL_5C);
  return true;
}

/// @brief Configures battery chemistry-specific parameters (cell count, charge voltage)
/// @param type Battery chemistry type (LIION_1S, LIFEPO4_1S, LTO_2S, BAT_UNKNOWN)
/// @return true if configuration successful
bool BoardConfigContainer::configureChemistry(BatteryType type) {
  if (!BQ_INITIALIZED) {
    return false;
  }

  // Get battery properties from lookup table
  const BatteryProperties* props = getBatteryProperties(type);
  if (!props) {
    MESH_DEBUG_PRINTLN("ERROR: Invalid battery type");
    return false;
  }

  // Apply charge enable/disable based on battery type
  bq.setChargeEnable(props->charge_enable);
  
  if (!props->charge_enable) {
    MESH_DEBUG_PRINTLN("WARNING: Battery type UNKNOWN - Charging DISABLED for safety!");
    return true;  // No further configuration needed for unknown battery
  }

  // Configure chemistry-specific parameters
  switch (type) {
  case BoardConfigContainer::BatteryType::LIION_1S:
    bq.setCellCount(BQ25798_CELL_COUNT_1S);
    bq.setTsIgnore(false);
    bq.setChargeLimitV(props->charge_voltage);
    break;
  case BoardConfigContainer::BatteryType::LIFEPO4_1S:
    bq.setCellCount(BQ25798_CELL_COUNT_1S);
    bq.setTsIgnore(false);
    bq.setChargeLimitV(props->charge_voltage);
    break;
  case BoardConfigContainer::BatteryType::LTO_2S:
    bq.setCellCount(BQ25798_CELL_COUNT_2S);
    bq.setTsIgnore(true);
    // Explicitly set JEITA current limits to UNCHANGED for LTO
    // Even though TS_IGNORE disables temperature monitoring, ensure JEITA registers don't interfere
    bq.setJeitaISetC(BQ25798_JEITA_ISETC_UNCHANGED);  // Cold region - no current reduction
    bq.setJeitaISetH(BQ25798_JEITA_ISETH_UNCHANGED);  // Warm region - no current reduction
    bq.setChargeLimitV(props->charge_voltage);
    break;
  case BoardConfigContainer::BatteryType::BAT_UNKNOWN:
    // Already handled above via charge_enable flag
    break;
  }

  return true;
}

/// @brief Configures solar-only interrupt masks on BQ25798
/// @return true if configuration successful
bool BoardConfigContainer::configureSolarOnlyInterrupts() {
  return bq.configureSolarOnlyInterrupts();
}

/// @brief Gets current battery type from preferences
/// @return Current battery chemistry type, defaults to DEFAULT_BATTERY_TYPE if read fails
BoardConfigContainer::BatteryType BoardConfigContainer::getBatteryType() const {
  BatteryType bat;
  if (loadBatType(bat)) {
    return bat;
  } else {
    return DEFAULT_BATTERY_TYPE;
  }
}

/// @brief Gets current frost charge behavior from preferences
/// @return Frost charge behavior, defaults to NO_CHARGE if read fails
BoardConfigContainer::FrostChargeBehaviour BoardConfigContainer::getFrostChargeBehaviour() const {
  FrostChargeBehaviour frost;
  if (loadFrost(frost)) {
    return frost;
  } else {
    return NO_CHARGE;
  }
}

/// @brief Gets maximum charge current from preferences
/// @return Maximum charge current in mA, defaults to 100mA if read fails
uint16_t BoardConfigContainer::getMaxChargeCurrent_mA() const {
  uint16_t maxI = 100;
  loadMaxChrgI(maxI);
  return maxI;
}

/// @brief Gets current MPPT enable status from preferences
/// @return true if MPPT enabled in configuration
bool BoardConfigContainer::getMPPTEnabled() const {
  bool enabled;
  loadMpptEnabled(enabled);
  return enabled;
}

/// @brief Enables or disables MPPT
/// @param enableMPPT true to enable MPPT
/// @return true if successful
bool BoardConfigContainer::setMPPTEnable(bool enableMPPT) {
  // Save to preferences first
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  if (!prefs.putString(MPPTENABLEKEY, enableMPPT ? "1" : "0")) {
    return false;
  }
  
  // Set the hardware register
  if (!enableMPPT) {
    // Disable MPPT in hardware
    bq.setMPPTenable(false);
  } else {
    // Enable MPPT - will be set by solarMpptTask when appropriate
    bq.setMPPTenable(true);
  }
  
  return true;
}

/// @brief Gets current maximum charge voltage
/// @return Charge voltage limit in V
float BoardConfigContainer::getMaxChargeVoltage() const {
  return bq.getChargeLimitV();
};

/// @brief Sets battery type and reconfigures BQ accordingly
/// @param type Battery chemistry type
/// @return true if all configurations successful
bool BoardConfigContainer::setBatteryType(BatteryType type) {
  bool bqBaseConfigured = this->configureBaseBQ();
  bool bqConfigured = this->configureChemistry(type);
  
  // === CRITICAL: Update INA228 UVLO threshold when battery type changes ===
  if (ina228DriverInstance) {
    // INA228 UVLO Alert: Update threshold for new battery chemistry
    // Threshold is read from battery properties in applyUvloSetting()
    applyUvloSetting();
    delay(10);
  }
  
  // MR2 (v0.2) doesn't use MCP4652
  
  // Store battery type in preferences
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  prefs.putString(BATTKEY, getBatteryTypeCommandString(type));
  
  // Safety: When switching to Li-Ion or LiFePO4, reset frost charge to NO_CHARGE
  // These chemistries should not be charged at low temperatures
  if (type == BatteryType::LIION_1S || type == BatteryType::LIFEPO4_1S) {
    setFrostChargeBehaviour(FrostChargeBehaviour::NO_CHARGE);
  }
  
  return bqBaseConfigured && bqConfigured;
}

/// @brief Sets frost charge behavior (JEITA cold region)
/// @param behaviour Charging behavior at low temperature
/// @return true if successful
bool BoardConfigContainer::setFrostChargeBehaviour(FrostChargeBehaviour behaviour) {
  switch (behaviour) {
  case BoardConfigContainer::FrostChargeBehaviour::NO_CHARGE:
    bq.setJeitaISetC(BQ25798_JEITA_ISETC_SUSPEND);
    break;
  case BoardConfigContainer::FrostChargeBehaviour::NO_REDUCE:
    bq.setJeitaISetC(BQ25798_JEITA_ISETC_UNCHANGED);
    break;
  case BoardConfigContainer::FrostChargeBehaviour::I_REDUCE_TO_40:
    bq.setJeitaISetC(BQ25798_JEITA_ISETC_40_PERCENT);
    break;
  case BoardConfigContainer::FrostChargeBehaviour::I_REDUCE_TO_20:
    bq.setJeitaISetC(BQ25798_JEITA_ISETC_20_PERCENT);
    break;
  }
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  prefs.putString(FROSTKEY, getFrostChargeBehaviourCommandString(behaviour));
  return true;
}

/// @brief Sets maximum charge current
/// @param maxChrgI Maximum charge current in mA
/// @return true if successful
bool BoardConfigContainer::setMaxChargeCurrent_mA(uint16_t maxChrgI) {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  prefs.putInt(MAXCHARGECURRENTKEY, maxChrgI);
  return bq.setChargeLimitA(maxChrgI / 1000.0f);
}

/// @brief Calculates 7-day moving average of MPPT enabled percentage
/// @return Percentage (0.0-100.0) of time MPPT was enabled over last 7 days
float BoardConfigContainer::getMpptEnabledPercentage7Day() const {
  // Return 0 if MPPT is disabled in config
  bool mpptEnabled;
  loadMpptEnabled(mpptEnabled);
  if (!mpptEnabled) {
    return 0.0f;
  }
  
  uint32_t totalMinutes = 0;
  uint32_t enabledMinutes = 0;
  uint32_t validHours = 0;
  
  // Count backwards through the circular buffer
  for (int i = 0; i < MPPT_STATS_HOURS; i++) {
    int index = (mpptStats.currentIndex - 1 - i + MPPT_STATS_HOURS) % MPPT_STATS_HOURS;
    
    // Skip entries that haven't been filled yet (timestamp == 0)
    if (mpptStats.hours[index].timestamp == 0) {
      continue;
    }
    
    validHours++;
    enabledMinutes += mpptStats.hours[index].mpptEnabledMinutes;
  }
  
  if (validHours == 0) {
    return 0.0f; // No data yet
  }
  
  totalMinutes = validHours * 60; // Each hour has 60 minutes
  
  return (enabledMinutes * 100.0f) / totalMinutes;
}

/// @brief Calculates average daily energy over last 3 days (72 hours)
/// @return Average daily harvested energy in mWh
uint32_t BoardConfigContainer::getAvgDailyEnergy3Day() const {
  // Return 0 if MPPT is disabled in config
  bool mpptEnabled;
  loadMpptEnabled(mpptEnabled);
  if (!mpptEnabled) {
    return 0;
  }
  
  uint32_t totalEnergy = 0;
  uint32_t validHours = 0;
  
  // Look back 72 hours (3 days)
  const int HOURS_72 = 72;
  
  for (int i = 0; i < HOURS_72 && i < MPPT_STATS_HOURS; i++) {
    int index = (mpptStats.currentIndex - 1 - i + MPPT_STATS_HOURS) % MPPT_STATS_HOURS;
    
    // Skip entries that haven't been filled yet
    if (mpptStats.hours[index].timestamp == 0) {
      continue;
    }
    
    totalEnergy += mpptStats.hours[index].harvestedEnergy_mWh;
    validHours++;
  }
  
  if (validHours == 0) {
    return 0; // No data
  }
  
  // Calculate average daily energy
  // Total energy over X hours, normalized to 24h
  return (totalEnergy * 24) / validHours;
}

/// @brief Formats MPPT statistics into a string buffer
/// @param buffer Destination buffer for formatted string
/// @param bufferSize Size of destination buffer
void BoardConfigContainer::getMpptStatsString(char* buffer, uint32_t bufferSize) const {
  if (!buffer || bufferSize == 0) {
    return;
  }
  
  // Check if MPPT is disabled in config
  bool mpptEnabled;
  loadMpptEnabled(mpptEnabled);
  if (!mpptEnabled) {
    snprintf(buffer, bufferSize, "MPPT disabled in configuration");
    return;
  }
  
  float percentage = getMpptEnabledPercentage7Day();
  uint32_t avgDailyEnergy = getAvgDailyEnergy3Day();
  
  // Count how many hours of data we actually have
  uint32_t validHours = 0;
  for (int i = 0; i < MPPT_STATS_HOURS; i++) {
    if (mpptStats.hours[i].timestamp != 0) {
      validHours++;
    }
  }
  
  float days = validHours / 24.0f;
  
  snprintf(buffer, bufferSize, "7d:%.1f%% 3d:%dmWh (%.1fd)", 
           percentage, avgDailyEnergy, days);
}

// ===== Battery SOC & Coulomb Counter Methods (v0.2) =====

/// @brief Get current State of Charge in percent
/// @return SOC in % (0-100)
float BoardConfigContainer::getStateOfCharge() const {
  return socStats.current_soc_percent;
}

/// @brief Get nominal voltage for battery chemistry type
/// @param type Battery chemistry type
/// @return Nominal voltage in V (used for mAh → mWh conversion)
float BoardConfigContainer::getNominalVoltage(BatteryType type) {
  switch (type) {
    case BatteryType::LTO_2S:
      return LTO_2S_NOMINAL;        // 5.0V (2.5V/cell)
    case BatteryType::LIFEPO4_1S:
      return LIFEPO4_1S_NOMINAL;    // 3.2V
    case BatteryType::LIION_1S:
    default:
      return LIION_1S_NOMINAL;      // 3.7V
  }
}

/// @brief Get battery capacity in mAh
/// @return Battery capacity (user-configured)
float BoardConfigContainer::getBatteryCapacity() const {
  return socStats.capacity_mah;
}

/// @brief Check if battery capacity was explicitly set via CLI
/// @return true if capacity was set in preferences, false if using default
bool BoardConfigContainer::isBatteryCapacitySet() const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  size_t len = prefs.getString(BATTERY_CAPACITY_KEY, buffer, sizeof(buffer), "");
  return (len > 0 && buffer[0] != '\0');
}

/// @brief Set battery capacity manually via CLI (converts to mWh internally)
/// @param capacity_mah Capacity in mAh (user input)
/// @return true if successful
bool BoardConfigContainer::setBatteryCapacity(float capacity_mah) {
  if (capacity_mah < 100.0f || capacity_mah > 100000.0f) {
    return false;  // Sanity check
  }
  
  // Store user-configured capacity in mAh
  socStats.capacity_mah = capacity_mah;
  
  // Get nominal voltage for current chemistry
  BatteryType batType = getBatteryType();
  float v_nominal = getNominalVoltage(batType);
  socStats.nominal_voltage = v_nominal;
  
  // Invalidate SOC until next "Charging Done" sync
  socStats.soc_valid = false;
  
  // Save to preferences
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%.1f", capacity_mah);
  prefs.putString(BATTERY_CAPACITY_KEY, buffer);
  
  MESH_DEBUG_PRINTLN("Battery capacity set to %.0f mAh @ %.1fV", 
                     capacity_mah, v_nominal);
  return true;
}

/// @brief Get formatted SOC string
/// @param buffer Output buffer
/// @param bufferSize Buffer size
void BoardConfigContainer::getBatterySOCString(char* buffer, uint32_t bufferSize) const {
  if (!socStats.soc_valid) {
    snprintf(buffer, bufferSize, "SOC:N/A (load bat fully to sync)");
  } else {
    snprintf(buffer, bufferSize, "SOC:%.1f%% Cap:%.0fmAh", 
             socStats.current_soc_percent, 
             socStats.capacity_mah);
  }
}

/// @brief Get formatted daily balance string (mWh-based)
/// @param buffer Output buffer
/// @param bufferSize Buffer size
void BoardConfigContainer::getDailyBalanceString(char* buffer, uint32_t bufferSize) const {
  // Last 24h net balance (mAh)
  float last_24h_net = socStats.last_24h_net_mah;
  float last_24h_charged = socStats.last_24h_charged_mah;
  float last_24h_discharged = socStats.last_24h_discharged_mah;
  float avg3d_charged = socStats.avg_3day_daily_charged_mah;
  float avg3d_discharged = socStats.avg_3day_daily_discharged_mah;
  const char* status = socStats.living_on_battery ? "BATTERY" : "SOLAR";
  
  snprintf(buffer, bufferSize, "24h:%+.1fmAh C:%.1f D:%.1f %s 3d:%+.1fmAh C:%.1f D:%.1f",
           last_24h_net,
           last_24h_charged,
           last_24h_discharged,
           status,
           socStats.avg_3day_daily_net_mah,
           avg3d_charged,
           avg3d_discharged);
}

/// @brief Get Time To Live in hours
/// @details Based on the 7-day rolling average of daily net energy deficit (avg_7day_daily_net_mah),
///          calculated from hourly INA228 Coulomb-counter samples in a 168h ring buffer.
///          Formula: TTL = (current_soc% * capacity_mah) / abs(avg_7day_daily_net_mah) * 24h
/// @return Hours until battery empty (0 = not calculated, charging, or insufficient data)
uint16_t BoardConfigContainer::getTTL_Hours() const {
  return socStats.ttl_hours;
}

/// @brief Check if living on battery (net deficit)
/// @return true if using more than charging
bool BoardConfigContainer::isLivingOnBattery() const {
  return socStats.living_on_battery;
}

/// @brief Sync SOC to 100% after "Charging Done" event from BQ25798
/// @details Resets INA228 Coulomb Counter baseline and marks SOC as valid
void BoardConfigContainer::syncSOCToFull() {
  if (!ina228DriverInstance) {
    return;
  }
  
  // Reset INA228 Coulomb Counter (clears ENERGY and CHARGE registers)
  ina228DriverInstance->resetCoulombCounter();
  
  // Set baseline to 0 (we just reset the counter)
  socStats.ina228_baseline_mah = 0;
  socStats.offset_accumulated_mah = 0.0f;  // Reset offset compensation accumulator
  socStats.last_soc_update_ms = millis();   // Reset time reference
  
  // Mark as fully charged
  socStats.current_soc_percent = 100.0f;
  socStats.soc_valid = true;
  
  MESH_DEBUG_PRINTLN("SOC: Synced to 100%% (Charging Done) - INA228 baseline reset");
}

/// @brief Manually set SOC to specific percentage (e.g. after reboot with known SOC)
/// @param soc_percent Desired SOC value (0-100)
/// @return true if successful, false if invalid parameters
bool BoardConfigContainer::setSOCManually(float soc_percent) {
  if (!ina228DriverInstance) {
    MESH_DEBUG_PRINTLN("SOC: Cannot set - INA228 not initialized");
    return false;
  }
  
  // Validate SOC range
  if (soc_percent < 0.0f || soc_percent > 100.0f) {
    MESH_DEBUG_PRINTLN("SOC: Invalid value %.1f%% (must be 0-100)", soc_percent);
    return false;
  }
  
  if (socStats.capacity_mah <= 0) {
    MESH_DEBUG_PRINTLN("SOC: Cannot set - battery capacity unknown");
    return false;
  }
  
  // Read current CHARGE register value
  float current_charge_mah = ina228DriverInstance->readCharge_mAh();
  
  // Calculate remaining capacity at desired SOC
  float remaining_mah = (soc_percent / 100.0f) * socStats.capacity_mah;
  
  // Calculate baseline: charge_mah = baseline + net_charge
  // We want: remaining_mah = capacity + net_charge = capacity + (charge - baseline + offset_accum)
  // Therefore: baseline = charge - (remaining - capacity) + offset_accum
  // But since we reset offset_accum to 0, baseline absorbs it:
  socStats.ina228_baseline_mah = current_charge_mah - (remaining_mah - socStats.capacity_mah);
  socStats.offset_accumulated_mah = 0.0f;  // Reset offset compensation accumulator
  socStats.last_soc_update_ms = millis();   // Reset time reference
  
  // Set SOC and mark as valid
  socStats.current_soc_percent = soc_percent;
  socStats.soc_valid = true;
  
  MESH_DEBUG_PRINTLN("SOC: Manually set to %.1f%% (CHARGE=%.1fmAh, Baseline=%.1fmAh)",
                     soc_percent, current_charge_mah, socStats.ina228_baseline_mah);
  
  return true;
}

/// @brief Load battery capacity from preferences
/// @param capacity_mah Output parameter
/// @return true if loaded successfully
bool BoardConfigContainer::loadBatteryCapacity(float& capacity_mah) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  if (prefs.getString(BATTERY_CAPACITY_KEY, buffer, sizeof(buffer), "") > 0) {
    if (buffer[0] != '\0') {
      capacity_mah = atof(buffer);
      return (capacity_mah > 0.0f);
    }
  }
  
  // Default capacity based on battery type (estimate)
  BatteryType type;
  if (loadBatType(type)) {
    switch (type) {
      case BatteryType::LTO_2S:
        capacity_mah = 2000.0f;  // Typical LTO capacity
        break;
      case BatteryType::LIFEPO4_1S:
        capacity_mah = 1500.0f;  // Typical LiFePO4 capacity
        break;
      case BatteryType::LIION_1S:
      default:
        capacity_mah = 2000.0f;  // Typical Li-Ion capacity
        break;
    }
  } else {
    capacity_mah = 2000.0f;  // Default fallback
  }
  
  return false;  // Not loaded from prefs
}

/// @brief Load INA228 calibration factor from preferences (v0.2)
/// @param factor Output parameter
/// @return true if loaded successfully, false if using default
bool BoardConfigContainer::loadIna228CalibrationFactor(float& factor) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  if (prefs.getString(INA228_CALIB_KEY, buffer, sizeof(buffer), "") > 0) {
    if (buffer[0] != '\0') {
      factor = atof(buffer);
      
      // Validate factor is in reasonable range
      if (factor >= 0.5f && factor <= 2.0f) {
        return true;
      }
    }
  }
  
  // Default: no calibration
  factor = 1.0f;
  return false;
}

/// @brief Set INA228 calibration factor and save to preferences (v0.2)
/// @param factor Calibration factor (0.5 to 2.0)
/// @return true if saved successfully
bool BoardConfigContainer::setIna228CalibrationFactor(float factor) {
  // Clamp to reasonable range
  if (factor < 0.5f) factor = 0.5f;
  if (factor > 2.0f) factor = 2.0f;
  
  // Apply to INA228 driver
  if (ina228DriverInstance) {
    ina228DriverInstance->setCalibrationFactor(factor);
  }
  
  // Save to preferences
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%.4f", factor);
  
  if (prefs.putString(INA228_CALIB_KEY, buffer)) {
    MESH_DEBUG_PRINTLN("INA228 calibration factor saved: %.4f", factor);
    return true;
  }
  
  return false;
}

/// @brief Get current INA228 calibration factor (v0.2)
/// @return Current calibration factor
float BoardConfigContainer::getIna228CalibrationFactor() const {
  if (ina228DriverInstance) {
    return ina228DriverInstance->getCalibrationFactor();
  }
  return 1.0f;  // Default if INA228 not available
}

/// @brief Perform INA228 current calibration and store result (v0.2)
/// @param actual_current_ma Actual measured battery current in mA (from reference meter)
/// @return Calculated and applied calibration factor, or 0.0 on error
float BoardConfigContainer::performIna228Calibration(float actual_current_ma) {
  if (!ina228DriverInstance) {
    return 0.0f;
  }
  
  // Perform calibration
  float new_factor = ina228DriverInstance->calibrateCurrent(actual_current_ma);
  
  if (new_factor <= 0.0f) {
    return 0.0f;  // Failed
  }
  
  // Store persistently
  if (!setIna228CalibrationFactor(new_factor)) {
    return 0.0f;
  }
  
  return new_factor;
}

/// @brief Get INA228 driver instance (v0.2)
/// @return Pointer to INA228 driver or nullptr if not initialized
Ina228Driver* BoardConfigContainer::getIna228Driver() {
  return ina228DriverInstance;
}

// ===== INA228 Current Offset Calibration =====

/// @brief Load INA228 current offset from preferences (v0.2)
/// @param offset Output parameter in mA
/// @return true if loaded successfully, false if using default (0.0)
bool BoardConfigContainer::loadIna228CurrentOffset(float& offset) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  if (prefs.getString(INA228_OFFSET_KEY, buffer, sizeof(buffer), "") > 0) {
    if (buffer[0] != '\0') {
      offset = atof(buffer);
      
      // Validate offset is in reasonable range (±50mA)
      if (offset >= -50.0f && offset <= 50.0f) {
        return true;
      }
    }
  }
  
  // Default: no offset
  offset = 0.0f;
  return false;
}

/// @brief Set INA228 current offset and save to preferences (v0.2)
/// @param offset_mA Offset in mA (-50 to +50)
/// @return true if saved successfully
bool BoardConfigContainer::setIna228CurrentOffset(float offset_mA) {
  // Clamp to reasonable range
  if (offset_mA < -50.0f) offset_mA = -50.0f;
  if (offset_mA > 50.0f) offset_mA = 50.0f;
  
  // Apply to INA228 driver
  if (ina228DriverInstance) {
    ina228DriverInstance->setCurrentOffset(offset_mA);
  }
  
  // Save to preferences
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%.2f", offset_mA);
  
  if (prefs.putString(INA228_OFFSET_KEY, buffer)) {
    MESH_DEBUG_PRINTLN("INA228 current offset saved: %+.2f mA", offset_mA);
    return true;
  }
  
  return false;
}

/// @brief Get current INA228 offset correction (v0.2)
/// @return Current offset in mA (0.0 = no correction)
float BoardConfigContainer::getIna228CurrentOffset() const {
  if (ina228DriverInstance) {
    return ina228DriverInstance->getCurrentOffset();
  }
  return 0.0f;
}

/// @brief Perform INA228 current offset calibration and store result (v0.2)
/// @param actual_current_ma Actual battery current in mA (from reference meter, signed)
/// @return Calculated offset in mA, or 0.0 on error
/// @note Best performed at LOW current where offset error dominates (e.g., idle/sleep current)
float BoardConfigContainer::performIna228OffsetCalibration(float actual_current_ma) {
  if (!ina228DriverInstance) {
    return 0.0f;
  }
  
  // Perform offset calibration
  float offset = ina228DriverInstance->calibrateCurrentOffset(actual_current_ma);
  
  // Store persistently
  if (!setIna228CurrentOffset(offset)) {
    return 0.0f;
  }
  
  return offset;
}

// ===== NTC Temperature Calibration =====

/// @brief Load NTC temperature calibration offset from preferences
/// @param offset Output parameter (°C)
/// @return true if loaded successfully, false if using default (0.0)
bool BoardConfigContainer::loadTcCalOffset(float& offset) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  if (prefs.getString(TCCAL_KEY, buffer, sizeof(buffer), "") > 0) {
    if (buffer[0] != '\0') {
      offset = atof(buffer);
      
      // Validate offset is in reasonable range (±20°C)
      if (offset >= -20.0f && offset <= 20.0f) {
        return true;
      }
    }
  }
  
  // Default: no offset
  offset = 0.0f;
  return false;
}

/// @brief Set NTC temperature calibration offset and save to preferences
/// @param offset_c Calibration offset in °C (-20 to +20)
/// @return true if saved successfully
bool BoardConfigContainer::setTcCalOffset(float offset_c) {
  // Clamp to reasonable range
  if (offset_c < -20.0f) offset_c = -20.0f;
  if (offset_c > 20.0f) offset_c = 20.0f;
  
  // Apply to runtime variable
  tcCalOffset = offset_c;
  
  // Save to preferences
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[20];
  snprintf(buffer, sizeof(buffer), "%.2f", offset_c);
  
  if (prefs.putString(TCCAL_KEY, buffer)) {
    MESH_DEBUG_PRINTLN("TC calibration offset saved: %.2f °C", offset_c);
    return true;
  }
  
  return false;
}

/// @brief Get current NTC temperature calibration offset
/// @return Current offset in °C (0.0 = no calibration)
float BoardConfigContainer::getTcCalOffset() const {
  return tcCalOffset;
}

/// @brief Perform NTC temperature calibration using a reference temperature
/// Averages 5 NTC readings to reduce ADC noise, computes offset = reference - avg, stores it.
/// @param actual_temp_c Reference temperature in °C (e.g. from BME280)
/// @return Computed offset in °C, or -999.0 on error
float BoardConfigContainer::performTcCalibration(float actual_temp_c) {
  if (!bqDriverInstance) {
    return -999.0f;
  }
  
  // Temporarily remove any existing offset to get raw NTC readings
  float old_offset = tcCalOffset;
  tcCalOffset = 0.0f;
  
  // Average multiple NTC readings to reduce ADC noise
  const int NUM_SAMPLES = 5;
  const int SAMPLE_DELAY_MS = 200;
  float ntc_sum = 0.0f;
  int valid_count = 0;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if (i > 0) delay(SAMPLE_DELAY_MS);
    
    const Telemetry* bqData = bqDriverInstance->getTelemetryData();
    if (!bqData) continue;
    
    float raw = bqData->batterie.temperature;
    // Skip error codes
    if (raw <= -800.0f || raw >= 98.0f) continue;
    
    ntc_sum += raw;
    valid_count++;
  }
  
  if (valid_count < 3) {
    tcCalOffset = old_offset;  // Restore old offset
    MESH_DEBUG_PRINTLN("TC Cal: Only %d/%d valid NTC readings", valid_count, NUM_SAMPLES);
    return -999.0f;
  }
  
  float raw_ntc_avg = ntc_sum / valid_count;
  
  // Compute offset: calibrated = raw + offset  →  offset = reference - raw
  float new_offset = actual_temp_c - raw_ntc_avg;
  
  MESH_DEBUG_PRINTLN("TC Cal: ref=%.2f NTC_avg=%.2f (%d samples) offset=%.2f", 
                     actual_temp_c, raw_ntc_avg, valid_count, new_offset);
  
  // Store persistently
  if (!setTcCalOffset(new_offset)) {
    tcCalOffset = old_offset;  // Restore on failure
    return -999.0f;
  }
  
  return new_offset;
}

/// @brief Perform NTC temperature calibration using on-board BME280 as reference
/// Averages 5 BME280 readings, then delegates to performTcCalibration(float).
/// @param bme_temp_out Optional: receives the averaged BME temperature used for calibration
/// @return Computed offset in °C, or -999.0 on error
float BoardConfigContainer::performTcCalibration(float* bme_temp_out) {
  // Average multiple BME280 readings to reduce noise
  const int NUM_SAMPLES = 5;
  const int SAMPLE_DELAY_MS = 200;
  float bme_sum = 0.0f;
  int valid_count = 0;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    float t = readBmeTemperature();
    if (t <= -900.0f) continue;
    bme_sum += t;
    valid_count++;
    if (i < NUM_SAMPLES - 1) delay(SAMPLE_DELAY_MS);
  }
  
  if (valid_count < 3) {
    MESH_DEBUG_PRINTLN("TC Cal: Only %d/%d valid BME readings", valid_count, NUM_SAMPLES);
    return -999.0f;
  }
  
  float bme_avg = bme_sum / valid_count;
  MESH_DEBUG_PRINTLN("TC Cal: BME avg=%.2f (%d samples)", bme_avg, valid_count);
  
  if (bme_temp_out) {
    *bme_temp_out = bme_avg;
  }
  
  return performTcCalibration(bme_avg);
}

/// @brief Read BME280 temperature directly via I2C (temporary instance, no core code changes)
/// @return Temperature in °C, or -999.0 if BME280 not available
float BoardConfigContainer::readBmeTemperature() {
#if ENV_INCLUDE_BME280
  Adafruit_BME280 bme;
  if (!bme.begin(0x76, &Wire)) {
    MESH_DEBUG_PRINTLN("TC Cal: BME280 not found at 0x76");
    return -999.0f;
  }
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::SAMPLING_X1,
                  Adafruit_BME280::FILTER_OFF,
                  Adafruit_BME280::STANDBY_MS_1000);
  if (!bme.takeForcedMeasurement()) {
    MESH_DEBUG_PRINTLN("TC Cal: BME280 forced measurement failed");
    return -999.0f;
  }
  float temp = bme.readTemperature();
  MESH_DEBUG_PRINTLN("TC Cal: BME280 reads %.2f C", temp);
  return temp;
#else
  MESH_DEBUG_PRINTLN("TC Cal: BME280 not compiled in (ENV_INCLUDE_BME280=0)");
  return -999.0f;
#endif
}

/// @brief Load UVLO enabled setting from preferences
/// @param enabled Reference to store loaded setting
/// @return true if preference found, false if default used
bool BoardConfigContainer::loadUvloEnabled(bool& enabled) const {
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  
  char buffer[10];
  if (prefs.getString(UVLO_ENABLE_KEY, buffer, sizeof(buffer), "") > 0) {
    if (buffer[0] != '\0') {
      enabled = buffer[0] == '1' ? true : false;
      return true;
    } else {
      enabled = false;  // Default: disabled
      return false;
    }
  }
  
  // No preference found - use default
  enabled = false;  // Default: UVLO disabled for field testing
  return false;
}

/// @brief Enable or disable INA228 UVLO alert
/// @param enabled true = enable UVLO, false = disable UVLO
/// @return true if successful
bool BoardConfigContainer::setUvloEnabled(bool enabled) {
  // Save to preferences
  SimplePreferences prefs;
  prefs.begin(PREFS_NAMESPACE);
  bool success = prefs.putString(UVLO_ENABLE_KEY, enabled ? "1" : "0");
  prefs.end();
  
  if (success) {
    // Apply immediately to hardware
    applyUvloSetting();
    MESH_DEBUG_PRINTLN("INA228 UVLO: %s (persistent)", enabled ? "ENABLED" : "DISABLED");
    return true;
  }
  return false;
}

/// @brief Get current UVLO enable state
/// @return true if UVLO is enabled
bool BoardConfigContainer::getUvloEnabled() const {
  bool enabled = false;
  loadUvloEnabled(enabled);
  return enabled;
}

/// @brief Apply current UVLO setting to INA228 hardware
void BoardConfigContainer::applyUvloSetting() {
  if (!ina228DriverInstance) {
    return;  // INA228 not initialized
  }
  
  bool uvlo_enabled = false;
  loadUvloEnabled(uvlo_enabled);
  
  // Get chemistry type and UVLO threshold from battery properties
  BatteryType bat_type = getBatteryType();
  const BatteryProperties* props = getBatteryProperties(bat_type);
  uint16_t uvlo_mv = props ? props->uvlo_threshold : 2000;  // Fallback to 2000mV if lookup fails
  
  // Apply to hardware
  ina228DriverInstance->setUnderVoltageAlert(uvlo_mv);
  ina228DriverInstance->enableAlert(uvlo_enabled, false, true);  // UVLO enabled/disabled, active-LOW, LATCHED
  
  MESH_DEBUG_PRINTLN("INA228 UVLO: %s @ %dmV (Latched)", uvlo_enabled ? "ENABLED" : "DISABLED", uvlo_mv);
}

/// @brief Get voltage threshold for critical software shutdown (chemistry-specific)
/// @details This is the danger zone boundary and 0% SOC point. Below this: danger zone (no TX, RTC wake).
///          Above this: normal operation. Hysteresis above UVLO prevents motorboating.
/// @param type Battery chemistry type
/// @return Threshold in millivolts - Danger zone boundary (100-200mV above hardware UVLO)
uint16_t BoardConfigContainer::getVoltageCriticalThreshold(BatteryType type) {
  const BatteryProperties* props = getBatteryProperties(type);
  return props ? props->danger_threshold : 2000;  // Fallback to 2000mV
}

/// @brief Get hardware UVLO voltage cutoff (chemistry-specific)
/// @param type Battery chemistry type
/// @return Hardware cutoff voltage in millivolts (INA228 Alert threshold)
uint16_t BoardConfigContainer::getVoltageHardwareCutoff(BatteryType type) {
  const BatteryProperties* props = getBatteryProperties(type);
  return props ? props->uvlo_threshold : 2000;  // Fallback to 2000mV
}

/// @brief Update battery SOC from INA228 Hardware Coulomb Counter (v0.2)
/// @details Uses INA228 CHARGE register (mAh) for accurate charge tracking
void BoardConfigContainer::updateBatterySOC() {
  if (!ina228DriverInstance) {
    return;
  }
  
  // Read INA228 Hardware Coulomb Counter (mAh) - TWO'S COMPLEMENT, has correct sign!
  // Positive = charging (into battery), Negative = discharging (from battery)
  float charge_mah = ina228DriverInstance->readCharge_mAh();
  
  // === Current Offset Compensation ===
  // The INA228 hardware CHARGE register accumulates the raw ADC current including
  // any constant input offset voltage (Vos). This offset causes a phantom current
  // that accumulates over time, drifting the Coulomb count.
  // We compensate by tracking the offset × time and applying it to all delta/net calculations.
  uint32_t now_ms = millis();
  float offset_delta_mah = 0.0f;
  
  if (socStats.last_soc_update_ms > 0) {
    uint32_t elapsed_ms = now_ms - socStats.last_soc_update_ms;
    // offset_mA × dt_hours = offset_mA × (elapsed_ms / 3600000)
    float offset_mA = ina228DriverInstance->getCurrentOffset();
    offset_delta_mah = offset_mA * ((float)elapsed_ms / 3600000.0f);
    socStats.offset_accumulated_mah += offset_delta_mah;
  }
  socStats.last_soc_update_ms = now_ms;
  
  // Update current hour statistics (track charged/discharged charge in mAh)
  // This runs ALWAYS, independent of SOC validity
  static float last_charge_mah = 0.0f;
  static bool first_read = true;
  
  if (first_read) {
    // Initialize baseline on first read, don't count initial value as delta
    last_charge_mah = charge_mah;
    socStats.last_charge_reading_mah = charge_mah;
    first_read = false;
  } else {
    float delta_mah = charge_mah - last_charge_mah;
    last_charge_mah = charge_mah;
    
    // Apply offset compensation to delta
    // The hardware delta is offset-biased; add the software offset correction
    delta_mah += offset_delta_mah;
    
    // Handle potential counter wrap or reset (ignore huge jumps > 10Ah)
    if (delta_mah > 10000.0f || delta_mah < -10000.0f) {
      MESH_DEBUG_PRINTLN("SOC: Large charge delta %.0fmAh - ignoring (counter reset?)", delta_mah);
    } else {
      // CHARGE register inverted in driver: positive delta = charging, negative delta = discharging
      if (delta_mah > 0.0f) {
        // Charging (positive delta)
        socStats.current_hour_charged_mah += delta_mah;
        socStats.current_hour_solar_mah += delta_mah;  // Assume solar (BQ tracks this)
      } else if (delta_mah < 0.0f) {
        // Discharging (negative delta)
        socStats.current_hour_discharged_mah += (-delta_mah);
      }
    }
  }
  
  // Check if BQ reports charging done → auto-sync
  if (bqDriverInstance) {
    bq25798_charging_status status = bqDriverInstance->getChargingStatus();
    if (status == BQ25798_CHARGER_STATE_DONE_CHARGING) {
      if (!socStats.soc_valid) {
        MESH_DEBUG_PRINTLN("SOC: First \"Charging Done\" detected - syncing to 100%%");
        syncSOCToFull();
      } else if (socStats.current_soc_percent < 99.0f) {
        MESH_DEBUG_PRINTLN("SOC: \"Charging Done\" detected - re-syncing to 100%%");
        syncSOCToFull();
      }
    }
  }
  
  // SOC calculation is only valid after first "Charging Done" sync via syncSOCToFull()
  if (!socStats.soc_valid) {
    return;  // Wait for first sync
  }
  
  // Net charge since last baseline reset (using CHARGE register in mAh)
  // Driver inverted: positive = charged into battery, negative = discharged from battery
  // Apply accumulated offset compensation to correct hardware CHARGE register drift
  float net_charge_mah = (charge_mah - socStats.ina228_baseline_mah) + socStats.offset_accumulated_mah;
  
  // Remaining capacity = Initial capacity + net charge (positive=charged adds, negative=discharged subtracts)
  float remaining_mah = socStats.capacity_mah + net_charge_mah;
  
  // Calculate SOC percentage
  if (socStats.capacity_mah > 0) {
    socStats.current_soc_percent = (remaining_mah / socStats.capacity_mah) * 100.0f;
    
    // Clamp to 0-100%
    if (socStats.current_soc_percent > 100.0f) socStats.current_soc_percent = 100.0f;
    if (socStats.current_soc_percent < 0.0f) socStats.current_soc_percent = 0.0f;
  }
}

/// @brief Update daily balance statistics (mAh-based)
/// @brief Update hourly battery statistics and advance rolling window
void BoardConfigContainer::updateHourlyStats() {
  uint32_t currentTime = getRTCTime();
  
  // Calculate hour boundary (align to full hours)
  uint32_t currentHour = (currentTime / 3600) * 3600;  // Truncate to hour boundary
  
  // Check if hour has changed
  if (socStats.lastHourUpdateTime == 0) {
    // First run - initialize
    socStats.lastHourUpdateTime = currentHour;
    MESH_DEBUG_PRINTLN("SOC: Hourly stats initialized at timestamp %u", currentHour);
    return;
  }
  
  uint32_t lastHour = (socStats.lastHourUpdateTime / 3600) * 3600;
  
  if (currentHour > lastHour) {
    // Hour boundary crossed - save current hour stats
    MESH_DEBUG_PRINTLN("SOC: Hour changed (%u -> %u) - saving stats: C:%.1f D:%.1f S:%.1f mAh", 
                       lastHour, currentHour,
                       socStats.current_hour_charged_mah,
                       socStats.current_hour_discharged_mah,
                       socStats.current_hour_solar_mah);
    
    // Move to next hour slot in circular buffer
    uint8_t nextIndex = (socStats.currentIndex + 1) % HOURLY_STATS_HOURS;
    
    // Save completed hour's stats
    HourlyBatteryStats& completedHour = socStats.hours[socStats.currentIndex];
    completedHour.timestamp = lastHour;
    completedHour.charged_mah = socStats.current_hour_charged_mah;
    completedHour.discharged_mah = socStats.current_hour_discharged_mah;
    completedHour.solar_mah = socStats.current_hour_solar_mah;
    
    // Reset accumulators for new hour
    socStats.currentIndex = nextIndex;
    socStats.current_hour_charged_mah = 0.0f;
    socStats.current_hour_discharged_mah = 0.0f;
    socStats.current_hour_solar_mah = 0.0f;
    socStats.lastHourUpdateTime = currentHour;
    
    // Recalculate rolling window statistics (24h and 3-day averages)
    calculateRollingStats();
  }
}

/// @brief Calculate 24h and 3-day rolling averages from hourly buffer
void BoardConfigContainer::calculateRollingStats() {
  // Calculate last 24 hours net balance
  float sum_24h_charged = 0.0f;
  float sum_24h_discharged = 0.0f;
  float sum_24h_solar = 0.0f;
  int valid_hours_24h = 0;
  
  // Sum up last 24 hours (most recent 24 entries)
  for (int i = 0; i < 24 && i < HOURLY_STATS_HOURS; i++) {
    int idx = (socStats.currentIndex - 1 - i + HOURLY_STATS_HOURS) % HOURLY_STATS_HOURS;
    if (socStats.hours[idx].timestamp != 0) {
      sum_24h_charged += socStats.hours[idx].charged_mah;
      sum_24h_discharged += socStats.hours[idx].discharged_mah;
      sum_24h_solar += socStats.hours[idx].solar_mah;
      valid_hours_24h++;
    }
  }
  
  // Last 24h net: solar - discharged (positive = surplus, negative = deficit)
  socStats.last_24h_net_mah = sum_24h_solar - sum_24h_discharged;
  socStats.last_24h_charged_mah = sum_24h_charged;
  socStats.last_24h_discharged_mah = sum_24h_discharged;
  socStats.living_on_battery = (socStats.last_24h_net_mah < 0.0f);
  
  // Calculate 3-day average daily net (72 hours)
  float sum_72h_charged = 0.0f;
  float sum_72h_discharged = 0.0f;
  float sum_72h_solar = 0.0f;
  int valid_hours_72h = 0;
  
  for (int i = 0; i < 72 && i < HOURLY_STATS_HOURS; i++) {
    int idx = (socStats.currentIndex - 1 - i + HOURLY_STATS_HOURS) % HOURLY_STATS_HOURS;
    if (socStats.hours[idx].timestamp != 0) {
      sum_72h_charged += socStats.hours[idx].charged_mah;
      sum_72h_discharged += socStats.hours[idx].discharged_mah;
      sum_72h_solar += socStats.hours[idx].solar_mah;
      valid_hours_72h++;
    }
  }
  
  // Average daily net over 3 days (divide 72h sum by 3)
  if (valid_hours_72h >= 24) {  // Need at least 24h of data
    float net_72h = sum_72h_solar - sum_72h_discharged;
    socStats.avg_3day_daily_net_mah = net_72h / 3.0f;  // Divide by 3 days
    socStats.avg_3day_daily_charged_mah = sum_72h_charged / 3.0f;
    socStats.avg_3day_daily_discharged_mah = sum_72h_discharged / 3.0f;
  } else {
    socStats.avg_3day_daily_net_mah = 0.0f;
    socStats.avg_3day_daily_charged_mah = 0.0f;
    socStats.avg_3day_daily_discharged_mah = 0.0f;
  }
  
  // Calculate 7-day average daily net (168 hours)
  float sum_168h_charged = 0.0f;
  float sum_168h_discharged = 0.0f;
  float sum_168h_solar = 0.0f;
  int valid_hours_168h = 0;
  
  for (int i = 0; i < 168 && i < HOURLY_STATS_HOURS; i++) {
    int idx = (socStats.currentIndex - 1 - i + HOURLY_STATS_HOURS) % HOURLY_STATS_HOURS;
    if (socStats.hours[idx].timestamp != 0) {
      sum_168h_charged += socStats.hours[idx].charged_mah;
      sum_168h_discharged += socStats.hours[idx].discharged_mah;
      sum_168h_solar += socStats.hours[idx].solar_mah;
      valid_hours_168h++;
    }
  }
  
  // Average daily net over 7 days (divide 168h sum by 7)
  if (valid_hours_168h >= 24) {  // Need at least 24h of data
    float net_168h = sum_168h_solar - sum_168h_discharged;
    socStats.avg_7day_daily_net_mah = net_168h / 7.0f;  // Divide by 7 days
    socStats.avg_7day_daily_charged_mah = sum_168h_charged / 7.0f;
    socStats.avg_7day_daily_discharged_mah = sum_168h_discharged / 7.0f;
  } else {
    socStats.avg_7day_daily_net_mah = 0.0f;
    socStats.avg_7day_daily_charged_mah = 0.0f;
    socStats.avg_7day_daily_discharged_mah = 0.0f;
  }
  
  MESH_DEBUG_PRINTLN("SOC: Rolling stats - 24h net: %+.1fmAh, 3d avg: %+.1fmAh/day, 7d avg: %+.1fmAh/day",
                     socStats.last_24h_net_mah, socStats.avg_3day_daily_net_mah, socStats.avg_7day_daily_net_mah);
  
  // Calculate TTL
  calculateTTL();
}

/// @brief Calculate Time To Live (hours until battery empty)
/// @details TTL is based on the **7-day rolling average** of daily net energy consumption
///          (avg_7day_daily_net_mah). This average is computed from a 168-hour (7-day)
///          ring buffer of hourly INA228 Coulomb-counter measurements (charged/discharged/solar mAh).
///
///          Data flow:
///          1. INA228 hardware Coulomb counter measures charge flow continuously (24-bit ADC)
///          2. updateHourlyStats() samples the counter every hour, storing per-hour deltas
///             (charged_mah, discharged_mah, solar_mah) in hours[168] ring buffer
///          3. calculateRollingStats() sums the last 168 hours and divides by 7 to get
///             avg_7day_daily_net_mah (= solar - discharged per day)
///          4. This method extrapolates: remaining_mah / deficit_per_day * 24 = TTL hours
///
///          Preconditions for TTL > 0:
///          - living_on_battery == true (24h net is negative, i.e. energy deficit)
///          - avg_7day_daily_net_mah < 0 (7-day average shows net discharge)
///          - capacity_mah > 0 (battery capacity is known)
///          - At least 24 hours of valid hourly data exist in the ring buffer
///
///          When the device is solar-powered with energy surplus (net >= 0),
///          TTL is 0 and callers interpret this as "infinite" via living_on_battery flag.
void BoardConfigContainer::calculateTTL() {
  if (!socStats.living_on_battery || socStats.avg_7day_daily_net_mah >= 0) {
    socStats.ttl_hours = 0;  // Not draining or charging
    return;
  }
  
  if (socStats.capacity_mah <= 0) {
    socStats.ttl_hours = 0;  // Capacity unknown
    return;
  }
  
  // Current remaining capacity in mAh
  float remaining_mah = (socStats.current_soc_percent / 100.0f) * socStats.capacity_mah;
  
  // Daily deficit (negative value)
  float deficit_per_day = -socStats.avg_7day_daily_net_mah;
  
  if (deficit_per_day <= 0) {
    socStats.ttl_hours = 0;
    return;
  }
  
  // Days until empty
  float days_remaining = remaining_mah / deficit_per_day;
  
  // Convert to hours
  socStats.ttl_hours = (uint16_t)(days_remaining * 24.0f);
  
  MESH_DEBUG_PRINTLN("TTL: %.1f days (%.0f mAh remaining, -%.0f mAh/day)",
                     days_remaining, remaining_mah, deficit_per_day);
}

/// @brief Estimate SOC from voltage (fallback method)
/// @param voltage_mv Battery voltage in mV
/// @param type Battery chemistry type
/// @return Estimated SOC in %
float BoardConfigContainer::estimateSOCFromVoltage(uint16_t voltage_mv, BatteryType type) {
  float voltage_v = voltage_mv / 1000.0f;
  float soc = 0.0f;
  
  switch (type) {
    case BatteryType::LTO_2S:
      // LTO discharge curve (2S): 5.6V (100%) → 4.0V (0%)
      soc = ((voltage_v - 4.0f) / (5.6f - 4.0f)) * 100.0f;
      break;
      
    case BatteryType::LIFEPO4_1S:
      // LiFePO4 discharge curve (1S): 3.6V (100%) → 2.8V (0%)
      soc = ((voltage_v - 2.8f) / (3.6f - 2.8f)) * 100.0f;
      break;
      
    case BatteryType::LIION_1S:
    default:
      // Li-Ion discharge curve (1S): 4.2V (100%) → 3.2V (0%)
      // More accurate: use piecewise linear approximation
      if (voltage_v >= 4.1f) {
        soc = 90.0f + ((voltage_v - 4.1f) / 0.1f) * 10.0f;  // 4.1-4.2V = 90-100%
      } else if (voltage_v >= 3.9f) {
        soc = 70.0f + ((voltage_v - 3.9f) / 0.2f) * 20.0f;  // 3.9-4.1V = 70-90%
      } else if (voltage_v >= 3.7f) {
        soc = 40.0f + ((voltage_v - 3.7f) / 0.2f) * 30.0f;  // 3.7-3.9V = 40-70%
      } else if (voltage_v >= 3.5f) {
        soc = 20.0f + ((voltage_v - 3.5f) / 0.2f) * 20.0f;  // 3.5-3.7V = 20-40%
      } else {
        soc = ((voltage_v - 3.2f) / 0.3f) * 20.0f;          // 3.2-3.5V = 0-20%
      }
      break;
  }
  
  // Clamp to 0-100%
  if (soc > 100.0f) soc = 100.0f;
  if (soc < 0.0f) soc = 0.0f;
  
  return soc;
}

/// @brief SOC Update Task - runs every minute to update battery statistics
/// @details Updates SOC from INA228 Coulomb Counter and daily balance statistics.
///          Separated from voltageMonitorTask to allow frequent SOC updates (1 minute)
///          while voltageMonitorTask handles voltage checks at longer intervals (1 hour).
/// @param pvParameters Task parameters (unused)
void BoardConfigContainer::socUpdateTask(void* pvParameters) {
  (void)pvParameters;
  
  MESH_DEBUG_PRINTLN("SOC Update Task started - running every minute");
  
  const uint32_t UPDATE_INTERVAL_MS = 60UL * 1000UL;  // 1 minute
  static uint8_t minute_counter = 0;  // Count minutes until hourly update
  static bool first_loop = true;
  
  while (true) {
    // Run an immediate voltage/RTC check on task start to avoid 60s wait in Danger Zone.
    runVoltageMonitor();
    if (first_loop) {
      // Allow the short stabilization window to pass, then re-check before the 60s cadence.
      vTaskDelay(pdMS_TO_TICKS(2000));
      runVoltageMonitor();
      first_loop = false;
    }

    // Wait for 1 minute
    vTaskDelay(pdMS_TO_TICKS(UPDATE_INTERVAL_MS));
    
    // Update SOC from Coulomb Counter (runs every minute)
    updateBatterySOC();

    // Run periodic undervoltage/RTC checks on the same cadence
    runVoltageMonitor();
    
    // Increment minute counter
    minute_counter++;
    
    // Every 60 minutes: Update hourly statistics
    if (minute_counter >= 60) {
      MESH_DEBUG_PRINTLN("SOC: 60 minutes elapsed - updating hourly stats");
      updateHourlyStats();
      minute_counter = 0;
    }
  }
}

/// @brief Voltage Monitor Task with SOC tracking (v0.2)
/// @param pvParameters Task parameters (unused)
void BoardConfigContainer::voltageMonitorTask(void* pvParameters) {
  (void)pvParameters;
  MESH_DEBUG_PRINTLN("Voltage Monitor Task disabled (SOC task handles UV checks)");
  vTaskDelete(NULL);
}

// ===== Helper Functions =====

/// @brief Trim whitespace from string
char* BoardConfigContainer::trim(char* str) {
  char* end;

  while (isspace((unsigned char)*str))
    str++;

  if (*str == 0) {
    return str;
  }

  end = str + strlen(str) - 1;

  while (end > str && isspace((unsigned char)*end))
    end--;

  *(end + 1) = 0;

  return str;
}

/// @brief Convert command string to battery type enum
BoardConfigContainer::BatteryType BoardConfigContainer::getBatteryTypeFromCommandString(const char* cmdStr) {
  for (const auto& entry : bat_map) {
    if (entry.command_string == nullptr) break;
    if (strcmp(entry.command_string, cmdStr) == 0) {
      return entry.type;
    }
  }
  return BatteryType::BAT_UNKNOWN;
}

/// @brief Get battery properties for a given battery type
/// @param type Battery type
/// @return Pointer to BatteryProperties structure, or nullptr if not found
const BoardConfigContainer::BatteryProperties* BoardConfigContainer::getBatteryProperties(BatteryType type) {
  for (const auto& props : battery_properties) {
    if (props.type == type) {
      return &props;
    }
  }
  return nullptr;  // Should never happen if battery_properties is complete
}

/// @brief Convert battery type enum to command string
const char* BoardConfigContainer::getBatteryTypeCommandString(BatteryType type) {
  for (const auto& entry : bat_map) {
    if (entry.command_string == nullptr) break;
    if (entry.type == type) {
      return entry.command_string;
    }
  }
  return "unknown";
}

/// @brief Convert frost charge behaviour enum to command string
const char* BoardConfigContainer::getFrostChargeBehaviourCommandString(FrostChargeBehaviour type) {
  for (const auto& entry : frostchargebehaviour_map) {
    if (entry.command_string == nullptr) break;
    if (entry.type == type) {
      return entry.command_string;
    }
  }
  return "unknown";
}

/// @brief Convert command string to frost charge behaviour enum
BoardConfigContainer::FrostChargeBehaviour BoardConfigContainer::getFrostChargeBehaviourFromCommandString(const char* cmdStr) {
  for (const auto& entry : frostchargebehaviour_map) {
    if (entry.command_string == nullptr) break;
    if (strcmp(entry.command_string, cmdStr) == 0) {
      return entry.type;
    }
  }
  return FrostChargeBehaviour::REDUCE_UNKNOWN;
}

/// @brief Get available frost charge behaviour option strings
const char* BoardConfigContainer::getAvailableFrostChargeBehaviourOptions() {
  static char buffer[64];

  if (buffer[0] != '\0') return buffer;

  buffer[0] = '\0';

  for (const auto& entry : frostchargebehaviour_map) {
    if (entry.command_string == nullptr) break;

    size_t space_needed = strlen(buffer) + 1 + strlen(entry.command_string) + 1;

    if (space_needed >= sizeof(buffer)) {
      break;
    }

    if (buffer[0] != '\0') {
      strcat(buffer, "|");
    }
    strcat(buffer, entry.command_string);
  }

  return buffer;
}

/// @brief Get available battery type option strings
const char* BoardConfigContainer::getAvailableBatOptions() {
  static char buffer[64];

  if (buffer[0] != '\0') return buffer;

  buffer[0] = '\0';

  for (const auto& entry : bat_map) {
    if (entry.command_string == nullptr) break;

    size_t space_needed = strlen(buffer) + 1 + strlen(entry.command_string) + 1;

    if (space_needed >= sizeof(buffer)) {
      break;
    }

    if (buffer[0] != '\0') {
      strcat(buffer, "|");
    }
    strcat(buffer, entry.command_string);
  }

  return buffer;
}
