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
#include "InheroMr2Board.h"

#include "lib/BqDriver.h"
#include "lib/Ina228Driver.h"
#include "lib/SimplePreferences.h"

#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include <task.h>
#include <MeshCore.h>
#include <nrf_wdt.h>

// Forward declaration - rtc_clock is defined in target.cpp
// Use base class to avoid including AutoDiscoverRTCClock header
class AutoDiscoverRTCClock;
extern AutoDiscoverRTCClock rtc_clock;

// Helper function to get RTC time safely
namespace {
  inline uint32_t getRTCTime() {
    return ((mesh::RTCClock&)rtc_clock).getCurrentTime();
  }
}

// Hardware drivers (v0.2 only)
static BqDriver bq;
static Ina228Driver ina228;

static SimplePreferences prefs;

// Forward declare board instance
extern InheroMr2Board board;

// Initialize singleton pointer
BqDriver* BoardConfigContainer::bqDriverInstance = nullptr;
Ina228Driver* BoardConfigContainer::ina228DriverInstance = nullptr;
TaskHandle_t BoardConfigContainer::mpptTaskHandle = NULL;
TaskHandle_t BoardConfigContainer::heartbeatTaskHandle = NULL;
TaskHandle_t BoardConfigContainer::voltageMonitorTaskHandle = NULL;
MpptStatistics BoardConfigContainer::mpptStats = {};
BatterySOCStats BoardConfigContainer::socStats = {};

// Solar charging thresholds
static const uint16_t MIN_VBUS_FOR_CHARGING = 3500; // 3.5V minimum for valid solar input

// Watchdog state
static bool wdt_enabled = false;

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
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BLUE, HIGH);
        delay(100);
        digitalWrite(LED_BLUE, LOW);
        delay(100);
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

BoardConfigContainer::BatteryType BoardConfigContainer::getBatteryTypeFromCommandString(const char* cmdStr) {
  for (const auto& entry : bat_map) {
    if (entry.command_string == nullptr) break;
    if (strcmp(entry.command_string, cmdStr) == 0) {
      return entry.type;
    }
  }
  return BatteryType::BAT_UNKNOWN;
}

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

const char* BoardConfigContainer::getBatteryTypeCommandString(BatteryType type) {
  for (const auto& entry : bat_map) {
    if (entry.command_string == nullptr) break;
    if (entry.type == type) {
      return entry.command_string;
    }
  }
  return "unknown";
}

const char* BoardConfigContainer::getFrostChargeBehaviourCommandString(FrostChargeBehaviour type) {
  for (const auto& entry : frostchargebehaviour_map) {
    if (entry.command_string == nullptr) break;
    if (entry.type == type) {
      return entry.command_string;
    }
  }
  return "unknown";
}

BoardConfigContainer::FrostChargeBehaviour BoardConfigContainer::getFrostChargeBehaviourFromCommandString(const char* cmdStr) {
  for (const auto& entry : frostchargebehaviour_map) {
    if (entry.command_string == nullptr) break;
    if (strcmp(entry.command_string, cmdStr) == 0) {
      return entry.type;
    }
  }
  return FrostChargeBehaviour::REDUCE_UNKNOWN;
}

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

/// @brief Detects and fixes stuck PGOOD state by toggling HIZ mode
/// @details When solar voltage rises slowly (sunrise), BQ25798 may not detect input source.
///          This function forces input qualification by toggling HIZ mode.
///          Only triggers if: PG=0, VBUS sufficient, and PG_STAT flag NOT set.
void BoardConfigContainer::checkAndFixPgoodStuck() {
  if (!bqDriverInstance) return;

  bqDriverInstance->readReg(0x22);  // Refresh status register

  uint8_t flag0 = bqDriverInstance->readReg(0x1B);  // CHARGER_FLAG_0 register
  bool pgFlagSet = (flag0 & 0x08) != 0;  // Bit 3: PG_STAT flag (set when PGOOD changes)
  bool powerGood = bqDriverInstance->getChargerStatusPowerGood();
  
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
    MESH_DEBUG_PRINT(vbusVoltage);
    MESH_DEBUG_PRINTLN("mV, PG=0 - forcing input detection");
    
    // Force input source detection by toggling HIZ
    // Per datasheet: Exiting HIZ triggers input source qualification
    bool wasHIZ = bq.getHIZMode();
    if (wasHIZ) {
      // EN_HIZ already set (poor source) - just clear it
      MESH_DEBUG_PRINTLN("EN_HIZ was set - clearing");
      bq.setHIZMode(false);
    } else {
      // EN_HIZ not set - toggle to force input re-detection
      MESH_DEBUG_PRINTLN("Toggling HIZ to force input scan");
      bq.setHIZMode(true);
      delay(50);
      bq.setHIZMode(false);
    }
    delay(100); // Allow input qualification to complete
    bq.setMPPTenable(true); // Ensure MPPT remains enabled
    MESH_DEBUG_PRINTLN("Input detection triggered");
  }
}

/// @brief Re-enables MPPT if BQ25798 disabled it (e.g., during !PG state)
/// @details BQ25798 does not persist MPPT=1 and sets MPPT=0 when PG=0.
///          This function restores MPPT=1 when PG returns to 1.
void BoardConfigContainer::checkAndFixSolarLogic() {
  if (!bqDriverInstance) return;

  // Check if MPPT is enabled in configuration
  bool mpptEnabled;
  BoardConfigContainer::loadMpptEnabled(mpptEnabled);

  if (!mpptEnabled) {
    // MPPT disabled in config - ensure register is cleared
    uint8_t mpptVal = bqDriverInstance->readReg(0x15);
    if ((mpptVal & 0x01) != 0) {
      bqDriverInstance->writeReg(0x15, mpptVal & ~0x01);
    }
    return;
  }

  // Read flag register to check if PG changed
  uint8_t flag0 = bqDriverInstance->readReg(0x1B);
  bool pgFlagSet = (flag0 & 0x08) != 0;  // Bit 3: PG_STAT flag
  bool powerGood = bqDriverInstance->getChargerStatusPowerGood();

  // Only re-enable MPPT if PG changed AND PGOOD is now set (PG=1)
  // When PGOOD goes 1→0, BQ automatically disables MPPT (which is correct)
  if (pgFlagSet && powerGood) {
    uint8_t mpptVal = bqDriverInstance->readReg(0x15);

    if ((mpptVal & 0x01) == 0) {
      bqDriverInstance->writeReg(0x15, mpptVal | 0x01);
      MESH_DEBUG_PRINTLN("MPPT re-enabled via register");
    }
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
    
    digitalWrite(LED_BLUE, LOW);
  }
}

void BoardConfigContainer::onBqInterrupt() {
  digitalWrite(LED_BLUE, HIGH);

  if (solarEventSem == NULL) return; // Safety check

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

  pinMode(LED_GREEN, OUTPUT);

  while (true) {
    digitalWrite(LED_GREEN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(10));  // 10ms flash - well visible, minimal power
    digitalWrite(LED_GREEN, LOW);
    vTaskDelay(pdMS_TO_TICKS(5000)); // 5s interval - lower power consumption
  }
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
  
  snprintf(buffer, bufferSize, "%s / %s", powerGood, statusString);
}

/// @brief Initializes battery manager, potentiometer, preferences, and MPPT task
/// @return true if both BQ25798 and MCP4652 initialized successfully
bool BoardConfigContainer::begin() {
  // Initialize BQ25798 (common for both v0.1 and v0.2)
  if (bq.begin()) {
    BQ_INITIALIZED = true;
    bqDriverInstance = &bq;
    MESH_DEBUG_PRINTLN("BQ25798 found. ");
  } else {
    MESH_DEBUG_PRINTLN("BQ25798 not found.");
    BQ_INITIALIZED = false;
  }
  
  // === MR2 Hardware (v0.2): INA228 Power Monitor with UVLO ===
  if (ina228.begin(20.0f)) {  // 20mΩ shunt resistor
    INA228_INITIALIZED = true;
    ina228DriverInstance = &ina228;
    MESH_DEBUG_PRINTLN("INA228 found @ 0x45");
      
      // Load and apply current calibration factor
      float calib_factor = 1.0f;
      if (loadIna228CalibrationFactor(calib_factor)) {
        ina228.setCalibrationFactor(calib_factor);
        MESH_DEBUG_PRINTLN("INA228 calibration factor loaded: %.4f", calib_factor);
      } else {
        MESH_DEBUG_PRINTLN("INA228 using default calibration (1.0)");
      }
      
      // Load battery type to set chemistry-specific UVLO threshold
      BatteryType bat;
      if (!loadBatType(bat)) {
        bat = DEFAULT_BATTERY_TYPE;
      }
      
      // Set hardware UVLO threshold based on chemistry
      uint16_t uvlo_mv = 0;
      switch (bat) {
        case BatteryType::LTO_2S:
          uvlo_mv = 4000;  // 4.0V
          break;
        case BatteryType::LIFEPO4_1S:
          uvlo_mv = 2800;  // 2.8V
          break;
        case BatteryType::LIION_1S:
        default:
          uvlo_mv = 3200;  // 3.2V
          break;
      }
      
      ina228.setUnderVoltageAlert(uvlo_mv);
      ina228.enableAlert(true, false, true);  // UVLO only, active-high
      MESH_DEBUG_PRINTLN("INA228 UVLO threshold: %dmV", uvlo_mv);
  } else {
    MESH_DEBUG_PRINTLN("INA228 not found @ 0x45");
    INA228_INITIALIZED = false;
  }
  
  // === MR2 Configuration (v0.2 only) ===
  prefs.begin(PREFS_NAMESPACE);
  BatteryType bat;
  FrostChargeBehaviour frost;
  uint16_t maxChargeCurrent_mA;
  bool reducedBattVoltage;

  if (!loadBatType(bat)) {
    prefs.putString(BATTKEY, getBatteryTypeCommandString(bat));
  }
  if (!loadFrost(frost)) {
    prefs.putString(FROSTKEY, getFrostChargeBehaviourCommandString(frost));
  }
  if (!loadMaxChrgI(maxChargeCurrent_mA)) {
    prefs.putInt(MAXCHARGECURRENTKEY, maxChargeCurrent_mA);
  }
  if (!loadReduceChrgU(reducedBattVoltage)) {
    prefs.putString(REDUCEDBATTVOLTAGE, reducedBattVoltage ? "1" : "0");
  }

  this->configureBaseBQ();
  this->configureChemistry(bat, reducedBattVoltage);
  
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

  if (mpptTaskHandle == NULL) {
    BaseType_t taskCreated = xTaskCreate(BoardConfigContainer::solarMpptTask, "SolarDaemon", 4096, NULL, 1, &mpptTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create MPPT task!");
      return false;
    }
  }

  if (heartbeatTaskHandle == NULL) {
    BaseType_t taskCreated = xTaskCreate(BoardConfigContainer::heartbeatTask, "Heartbeat", 1024, NULL, 1, &heartbeatTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create Heartbeat task!");
      return false;
    }
  }
  
  // Start Voltage Monitor Task (MR2 v0.2 feature)
  if (voltageMonitorTaskHandle == NULL) {
    BaseType_t taskCreated = xTaskCreate(BoardConfigContainer::voltageMonitorTask, "VoltMon", 2048, NULL, 2, &voltageMonitorTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create Voltage Monitor task!");
      // Non-critical, continue
    } else {
      MESH_DEBUG_PRINTLN("Voltage Monitor task started (v0.2)");
    }
  }

  attachInterrupt(digitalPinToInterrupt(BQ_INT_PIN), onBqInterrupt, FALLING);

  // MR2 always has BQ25798 + INA228
  return BQ_INITIALIZED && INA228_INITIALIZED;
}

/// @brief Loads battery type from preferences
/// @param type Reference to store loaded battery type
/// @return true if preference found and valid, false if default used
bool BoardConfigContainer::loadBatType(BatteryType& type) const {
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

/// @brief Loads reduced charge voltage setting from preferences
/// @param reduce Reference to store loaded setting
/// @return true if preference found, false if default used
bool BoardConfigContainer::loadReduceChrgU(bool& reduce) const {
  char buffer[10];

  if (prefs.getString(REDUCEDBATTVOLTAGE, buffer, sizeof(buffer), "") > 0) {
    if (buffer[0] != '\0') {
      reduce = buffer[0] == '1' ? true : false;
      return true;
    } else {
      reduce = DEFAULT_REDUCED_CHARGE_VOLTAGE;
      return false;
    }
  }
  
  // No preference found - use default
  reduce = DEFAULT_REDUCED_CHARGE_VOLTAGE;
  return false;
}

/// @brief Loads MPPT enabled setting from preferences
/// @param enabled Reference to store loaded setting
/// @return true if preference found, false if default used
bool BoardConfigContainer::loadMpptEnabled(bool& enabled) {
  SimplePreferences prefs;
  prefs.begin("inheromr1");
  
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
  
  // Get base telemetry from BQ25798 (solar data + temperature)
  const Telemetry* bqData = bq.getTelemetryData();
  if (!bqData) {
    memset(&telemetry, 0, sizeof(Telemetry));
    return &telemetry;
  }
  
  // Copy BQ25798 data (solar, system, temperature)
  telemetry.solar = bqData->solar;
  telemetry.system = bqData->system;
  telemetry.batterie.temperature = bqData->batterie.temperature;
  
  // Override battery voltage/current with INA228 data (v0.2 hardware)
  if (INA228_INITIALIZED && ina228DriverInstance != nullptr) {
    telemetry.batterie.voltage = ina228DriverInstance->readVoltage_mV();
    telemetry.batterie.current = ina228DriverInstance->readCurrent_mA();
    telemetry.batterie.power = ((int32_t)telemetry.batterie.voltage * telemetry.batterie.current) / 1000;
  } else {
    // Fallback to BQ25798 values if INA228 not available
    telemetry.batterie.voltage = bqData->batterie.voltage;
    telemetry.batterie.current = bqData->batterie.current;
    telemetry.batterie.power = bqData->batterie.power;
  }
  
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
  bool reducedBattVoltage;
  
  if (!loadBatType(bat)) {
    bat = BatteryType::LIION_1S;  // Default
  }
  if (!loadFrost(frost)) {
    frost = FrostChargeBehaviour::NO_REDUCE;  // Default (1)
  }
  if (!loadMaxChrgI(maxChargeCurrent_mA)) {
    maxChargeCurrent_mA = 200;  // Default
  }
  if (!loadReduceChrgU(reducedBattVoltage)) {
    reducedBattVoltage = false;  // Default
  }
  
  // Apply configuration
  configureBaseBQ();
  configureChemistry(bat, reducedBattVoltage);
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

  bq.setMinSystemV(2.7);
  bq.setStatPinEnable(true);
  bq.setTsCool(BQ25798_TS_COOL_5C);
  return true;
}

/// @brief Configures battery chemistry-specific parameters (cell count, charge voltage)
/// @param type Battery chemistry type (LIION_1S, LIFEPO4_1S, LTO_2S)
/// @param reduceMaxChrgU If true, uses reduced max voltage for extended life
/// @return true if configuration successful
bool BoardConfigContainer::configureChemistry(BatteryType type, bool reduceMaxChrgU) {
  if (!BQ_INITIALIZED) {
    return false;
  }

  switch (type) {
  case BoardConfigContainer::BatteryType::LIION_1S:
    bq.setCellCount(BQ25798_CELL_COUNT_1S);
    bq.setTsIgnore(false);
    if (reduceMaxChrgU) {
      bq.setChargeLimitV(LIION_1S_VOLTAGE_REDUCED);
    } else {
      bq.setChargeLimitV(LIION_1S_VOLTAGE_NORMAL);
    }
    break;
  case BoardConfigContainer::BatteryType::LIFEPO4_1S:
    bq.setCellCount(BQ25798_CELL_COUNT_1S);
    bq.setTsIgnore(false);
    if (reduceMaxChrgU) {
      bq.setChargeLimitV(LIFEPO4_1S_VOLTAGE_REDUCED);
    } else {
      bq.setChargeLimitV(LIFEPO4_1S_VOLTAGE_NORMAL);
    }
    break;
  case BoardConfigContainer::BatteryType::LTO_2S:
    bq.setCellCount(BQ25798_CELL_COUNT_2S);
    bq.setTsIgnore(true);
    // Explicitly set JEITA current limits to UNCHANGED for LTO
    // Even though TS_IGNORE disables temperature monitoring, ensure JEITA registers don't interfere
    bq.setJeitaISetC(BQ25798_JEITA_ISETC_UNCHANGED);  // Cold region - no current reduction
    bq.setJeitaISetH(BQ25798_JEITA_ISETH_UNCHANGED);  // Warm region - no current reduction
    if (reduceMaxChrgU) {
      bq.setChargeLimitV(LTO_2S_VOLTAGE_REDUCED);
    } else {
      bq.setChargeLimitV(LTO_2S_VOLTAGE_NORMAL);
    }
  }

  return true;
}

/// @brief Configures solar-only interrupt masks on BQ25798
/// @return true if configuration successful
bool BoardConfigContainer::configureSolarOnlyInterrupts() {
  return bq.configureSolarOnlyInterrupts();
}

/// @brief Gets current battery type from preferences
/// @return Current battery chemistry type, defaults to LIFEPO4_1S if read fails
BoardConfigContainer::BatteryType BoardConfigContainer::getBatteryType() const {
  BatteryType bat;
  if (loadBatType(bat)) {
    return bat;
  } else {
    return BatteryType::LIFEPO4_1S;
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

/// @brief Gets reduced charge voltage setting from preferences
/// @return true if reduced voltage enabled, false otherwise
bool BoardConfigContainer::getReduceChargeVoltage() const {
  bool reduce = false;
  loadReduceChrgU(reduce);
  return reduce;
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

/// @brief Sets battery type and reconfigures BQ and MCP accordingly
/// @param type Battery chemistry type
/// @param reducedChargeVoltage Enable reduced voltage for extended life
/// @return true if all configurations successful
bool BoardConfigContainer::setBatteryType(BatteryType type, bool reducedChargeVoltage) {
  bool bqBaseConfigured = this->configureBaseBQ();
  bool bqConfigured = this->configureChemistry(type, reducedChargeVoltage);
  // MR2 (v0.2) doesn't use MCP4652
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
  prefs.putString(FROSTKEY, getFrostChargeBehaviourCommandString(behaviour));
}

/// @brief Sets maximum charge current
/// @param maxChrgI Maximum charge current in mA
/// @return true if successful
bool BoardConfigContainer::setMaxChargeCurrent_mA(uint16_t maxChrgI) {
  prefs.putInt(MAXCHARGECURRENTKEY, maxChrgI);
  return bq.setChargeLimitA(maxChrgI / 1000.0f);
}

/// @brief Sets battery type with current reduced voltage setting
/// @param type Battery chemistry type
/// @return true if successful
bool BoardConfigContainer::setBatteryType(BatteryType type) {
  bool reduce = false;
  loadReduceChrgU(reduce);
  setBatteryType(type, reduce);
  prefs.putString(BATTKEY, getBatteryTypeCommandString(type));
  
  // Safety: When switching to Li-Ion or LiFePO4, reset frost charge to NO_CHARGE
  // These chemistries should not be charged at low temperatures
  if (type == BatteryType::LIION_1S || type == BatteryType::LIFEPO4_1S) {
    setFrostChargeBehaviour(FrostChargeBehaviour::NO_CHARGE);
  }
}

/// @brief Sets reduced charge voltage mode
/// @param reduce true to enable reduced voltage
/// @return true if successful
bool BoardConfigContainer::setReducedChargeVoltage(bool reduce) {
  BatteryType type = LIFEPO4_1S;
  loadBatType(type);
  setBatteryType(type, reduce);
  prefs.putString(REDUCEDBATTVOLTAGE, reduce ? "1" : "0");
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

/// @brief Get battery capacity in mAh
/// @return Battery capacity (learned or configured)
float BoardConfigContainer::getBatteryCapacity() const {
  return socStats.battery_capacity_mah;
}

/// @brief Set battery capacity manually via CLI
/// @param capacity_mah Capacity in mAh
/// @return true if successful
bool BoardConfigContainer::setBatteryCapacity(float capacity_mah) {
  if (capacity_mah < 100.0f || capacity_mah > 100000.0f) {
    return false;  // Sanity check
  }
  
  socStats.battery_capacity_mah = capacity_mah;
  socStats.capacity_learned = false;  // Manual override
  
  // Save to preferences (as integer mAh)
  prefs.putInt(BATTERY_CAPACITY_KEY, (uint16_t)capacity_mah);
  
  MESH_DEBUG_PRINTLN("Battery capacity set to %.0f mAh", capacity_mah);
  return true;
}

/// @brief Get formatted SOC string
/// @param buffer Output buffer
/// @param bufferSize Buffer size
void BoardConfigContainer::getBatterySOCString(char* buffer, uint32_t bufferSize) const {
  const char* capacity_source = socStats.capacity_learned ? "learned" : "config";
  snprintf(buffer, bufferSize, "SOC:%.1f%% Cap:%.0fmAh(%s)", 
           socStats.current_soc_percent, 
           socStats.battery_capacity_mah,
           capacity_source);
}

/// @brief Get formatted daily balance string
/// @param buffer Output buffer
/// @param bufferSize Buffer size
void BoardConfigContainer::getDailyBalanceString(char* buffer, uint32_t bufferSize) const {
  // Get today's stats
  int32_t today_net = socStats.today_solar_mah - socStats.today_discharge_mah;
  const char* status = socStats.living_on_battery ? "BATTERY" : "SOLAR";
  
  snprintf(buffer, bufferSize, "Today:%+dmAh %s 3dAvg:%+.0fmAh", 
           today_net,
           status,
           socStats.avg_daily_deficit_mah);
}

/// @brief Get Time To Live in hours
/// @return Hours until battery empty (0 = not calculated or charging)
uint16_t BoardConfigContainer::getTTL_Hours() const {
  return socStats.ttl_hours;
}

/// @brief Check if living on battery (net deficit)
/// @return true if using more than charging
bool BoardConfigContainer::isLivingOnBattery() const {
  return socStats.living_on_battery;
}

/// @brief Load battery capacity from preferences
/// @param capacity_mah Output parameter
/// @return true if loaded successfully
bool BoardConfigContainer::loadBatteryCapacity(float& capacity_mah) const {
  // Check if key exists by building file path manually
  String path = String("/") + PREFS_NAMESPACE + "/" + BATTERY_CAPACITY_KEY + ".txt";
  if (InternalFS.exists(path.c_str())) {
    char buffer[20];
    prefs.getString(BATTERY_CAPACITY_KEY, buffer, sizeof(buffer), "0");
    capacity_mah = atof(buffer);
    return (capacity_mah > 0.0f);
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
  String path = String("/") + PREFS_NAMESPACE + "/" + INA228_CALIB_KEY + ".txt";
  if (InternalFS.exists(path.c_str())) {
    char buffer[20];
    prefs.getString(INA228_CALIB_KEY, buffer, sizeof(buffer), "1.0");
    factor = atof(buffer);
    
    // Validate factor is in reasonable range
    if (factor >= 0.5f && factor <= 2.0f) {
      return true;
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

/// @brief Update battery SOC from INA228 Coulomb Counter (MR2 v0.2)
void BoardConfigContainer::updateBatterySOC() {
  if (!ina228DriverInstance) {
    return;
  }
  
  Ina228BatteryData data;
  if (!ina228DriverInstance->readAll(&data)) {
    return;
  }
  
  // Update today's accumulators
  float charge_delta_mah = data.charge_mah - socStats.learning_accumulated_mah;
  socStats.learning_accumulated_mah = data.charge_mah;
  
  if (charge_delta_mah > 0) {
    // Charging
    socStats.today_charge_mah += (int32_t)charge_delta_mah;
    socStats.today_solar_mah += (int32_t)charge_delta_mah;
  } else {
    // Discharging
    socStats.today_discharge_mah += (int32_t)(-charge_delta_mah);
  }
  
  // Update SOC
  if (socStats.battery_capacity_mah > 0) {
    // Coulomb-counting based SOC
    float soc_delta = (charge_delta_mah / socStats.battery_capacity_mah) * 100.0f;
    socStats.current_soc_percent += soc_delta;
    
    // Clamp to 0-100%
    if (socStats.current_soc_percent > 100.0f) socStats.current_soc_percent = 100.0f;
    if (socStats.current_soc_percent < 0.0f) socStats.current_soc_percent = 0.0f;
  } else {
    // Fallback: Voltage-based SOC estimation
    BatteryType type;
    if (bqDriverInstance) {
      // Get battery type from config
      BoardConfigContainer temp;
      temp.loadBatType(type);
      socStats.current_soc_percent = estimateSOCFromVoltage(data.voltage_mv, type);
    }
  }
  
  // TODO: Auto-learning capacity from full charge cycle
  // Detect "Charge Done" from BQ25798 → Start learning
  // Accumulate discharge until "Danger Zone" → Calculate capacity
}

/// @brief Update daily balance statistics
void BoardConfigContainer::updateDailyBalance() {
  uint32_t currentTime = getRTCTime();
  
  // Check if day has changed (86400 seconds = 1 day)
  if (currentTime - socStats.lastUpdateTime >= 86400) {
    // Move to next day
    uint8_t nextIndex = (socStats.currentIndex + 1) % DAILY_STATS_DAYS;
    
    // Save today's stats
    DailyBatteryStats& today = socStats.days[socStats.currentIndex];
    today.timestamp = socStats.lastUpdateTime;
    today.charge_mah = socStats.today_charge_mah;
    today.discharge_mah = socStats.today_discharge_mah;
    today.solar_charge_mah = socStats.today_solar_mah;
    today.net_balance_mah = socStats.today_solar_mah - socStats.today_discharge_mah;
    
    // Reset accumulators for new day
    socStats.currentIndex = nextIndex;
    socStats.today_charge_mah = 0;
    socStats.today_discharge_mah = 0;
    socStats.today_solar_mah = 0;
    socStats.lastUpdateTime = currentTime;
    
    // Calculate 3-day average deficit
    int32_t total_deficit = 0;
    int32_t valid_days = 0;
    for (int i = 0; i < 3 && i < DAILY_STATS_DAYS; i++) {
      int idx = (socStats.currentIndex - 1 - i + DAILY_STATS_DAYS) % DAILY_STATS_DAYS;
      if (socStats.days[idx].timestamp != 0) {
        total_deficit += socStats.days[idx].net_balance_mah;
        valid_days++;
      }
    }
    
    if (valid_days > 0) {
      socStats.avg_daily_deficit_mah = (float)total_deficit / valid_days;
      socStats.living_on_battery = (socStats.avg_daily_deficit_mah < 0);
    }
    
    // Calculate TTL
    calculateTTL();
  }
}

/// @brief Calculate Time To Live (hours until battery empty)
void BoardConfigContainer::calculateTTL() {
  if (!socStats.living_on_battery || socStats.avg_daily_deficit_mah >= 0) {
    socStats.ttl_hours = 0;  // Not draining or charging
    return;
  }
  
  if (socStats.battery_capacity_mah <= 0) {
    socStats.ttl_hours = 0;  // Capacity unknown
    return;
  }
  
  // Current remaining capacity in mAh
  float remaining_mah = (socStats.current_soc_percent / 100.0f) * socStats.battery_capacity_mah;
  
  // Daily deficit (negative value)
  float deficit_per_day = -socStats.avg_daily_deficit_mah;
  
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

/// @brief Voltage Monitor Task with SOC tracking (v0.2)
/// @param pvParameters Task parameters (unused)
void BoardConfigContainer::voltageMonitorTask(void* pvParameters) {
  (void)pvParameters;
  
  MESH_DEBUG_PRINTLN("Voltage Monitor Task started (v0.2)");
  
  // Load or estimate battery capacity
  float capacity_mah;
  bool capacity_loaded = false;
  if (bqDriverInstance) {
    BoardConfigContainer temp;
    capacity_loaded = temp.loadBatteryCapacity(capacity_mah);
  } else {
    capacity_mah = 2000.0f;  // Default
  }
  
  socStats.battery_capacity_mah = capacity_mah;
  socStats.capacity_learned = capacity_loaded;
  socStats.current_soc_percent = 50.0f;  // Initial estimate
  socStats.lastUpdateTime = getRTCTime();
  
  MESH_DEBUG_PRINTLN("Battery capacity: %.0f mAh (%s)", 
                     capacity_mah, 
                     capacity_loaded ? "loaded" : "default");
  
  // Adaptive monitoring intervals
  uint32_t checkInterval_ms = 60000;  // Start with 60s
  
  // Get thresholds
  uint16_t critical_mv = 3400;  // Default, will be updated
  uint16_t warning_mv = critical_mv + 100;
  uint16_t normal_mv = 3600;
  
  while (true) {
    // Update SOC from Coulomb Counter
    updateBatterySOC();
    
    // Update daily balance (checks if day changed)
    updateDailyBalance();
    
    // Read current voltage
    uint16_t vbat_mv = 0;
    if (ina228DriverInstance) {
      vbat_mv = ina228DriverInstance->readVoltage_mV();
    }
    
    // Check voltage thresholds
    if (vbat_mv > 0) {
      if (vbat_mv < critical_mv) {
        // Critical - initiate shutdown
        MESH_DEBUG_PRINTLN("PWRMGT: Critical voltage %dmV - shutting down", vbat_mv);
        
        // TODO: Call initiateShutdown from InheroMr2Board
        // For now, just log
        checkInterval_ms = 10000;  // 10s
      } else if (vbat_mv < warning_mv) {
        // Warning - check more frequently
        checkInterval_ms = 10000;  // 10s
      } else if (vbat_mv < normal_mv) {
        // Near warning - moderate frequency
        checkInterval_ms = 30000;  // 30s
      } else {
        // Normal - slow monitoring
        checkInterval_ms = 60000;  // 60s
      }
    }
    
    vTaskDelay(pdMS_TO_TICKS(checkInterval_ms));
  }
}
