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

#include "InheroMr1Board.h"
#include "lib/BqDriver.h"
#include "lib/McpDriver.h"
#include "lib/SimplePreferences.h"
#include "target.h"

#include <ArduinoJson.h>
#include <FreeRTOS.h>
#include <MeshCore.h>
#include <RTClib.h>
#include <nrf_wdt.h>
#include <task.h>

// Hardware drivers (v0.1 only)
static BqDriver bq;
static McpDriver mcp;

static SimplePreferences prefs;

// Forward declare board instance
extern InheroMr1Board board;

// Initialize singleton pointer
BqDriver* BoardConfigContainer::bqDriverInstance = nullptr;
TaskHandle_t BoardConfigContainer::mpptTaskHandle = NULL;
TaskHandle_t BoardConfigContainer::heartbeatTaskHandle = NULL;
MpptStatistics BoardConfigContainer::mpptStats = {};
uint32_t BoardConfigContainer::lastHizToggleTimestamp = 0;
uint16_t BoardConfigContainer::lastHizToggleVbus = 0;
bool BoardConfigContainer::lastHizToggleSuccess = false;

// Watchdog state
static bool wdt_enabled = false;

// Cooldown timers to prevent interrupt loops and excessive toggling:
// - MPPT writes can trigger BQ25798 interrupts, which wake the task, creating a loop
// - HIZ toggles should be rare events only for stuck PGOOD conditions
static uint32_t lastMpptWriteTime = 0;      // 60-second cooldown for MPPT register writes
static uint32_t lastHizToggleTime = 0;      // 5-minute cooldown for HIZ toggles
#define HIZ_TOGGLE_COOLDOWN_MS (5 * 60 * 1000)  // 5 minutes

// Solar charging thresholds
static const uint16_t MIN_VBUS_FOR_CHARGING = 3500; // 3.5V minimum for valid solar input

/// @brief Initialize and start the hardware watchdog timer
/// @details Configures nRF52 WDT with 600 second timeout for OTA compatibility. Only enabled in release
/// builds.
///          Watchdog continues running during sleep and pauses during debug.
void BoardConfigContainer::setupWatchdog() {
#ifndef DEBUG_MODE                                                   // Only activate in release builds
  NRF_WDT->CONFIG = (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) | // Run during sleep
                    (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos);  // Pause during debug
  NRF_WDT->CRV = 32768 * 600; // 600 seconds (10 min) @ 32.768 kHz - allows OTA updates
  NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos; // Enable reload register 0
  NRF_WDT->TASKS_START = 1;                                 // Start watchdog
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
    NRF_WDT->RR[0] = WDT_RR_RR_Reload; // Reload watchdog
  }
#endif
}

/// @brief Disable the watchdog timer (for OTA updates)
/// @details Note: nRF52 WDT cannot be stopped once started. This only sets flag to stop feeding.
void BoardConfigContainer::disableWatchdog() {
#ifndef DEBUG_MODE
  wdt_enabled = false; // Stop feeding the watchdog
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

BoardConfigContainer::FrostChargeBehaviour
BoardConfigContainer::getFrostChargeBehaviourFromCommandString(const char* cmdStr) {
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
    MESH_DEBUG_PRINT(vbusVoltage);
    MESH_DEBUG_PRINTLN("mV, PG=0 - forcing input detection");

    // Record VBUS voltage before toggle
    lastHizToggleVbus = vbusVoltage;

    // Force input source detection by toggling HIZ
    // Per datasheet: Exiting HIZ triggers input source qualification
    bool wasHIZ = bq.getHIZMode();
    if (wasHIZ) {
      // EN_HIZ already set (poor source) - just clear it
      MESH_DEBUG_PRINTLN("EN_HIZ was set - clearing");
      bq.setHIZMode(false);
      delay(200);                                          // Allow HIZ exit to settle
      lastHizToggleTimestamp = rtc_clock.getCurrentTime(); // Record HIZ toggle event
    } else {
      // EN_HIZ not set - toggle to force input re-detection
      MESH_DEBUG_PRINTLN("Toggling HIZ to force input scan");
      bq.setHIZMode(true);
      delay(250); // Give BQ time to process HIZ state
      bq.setHIZMode(false);
      lastHizToggleTimestamp = rtc_clock.getCurrentTime(); // Record HIZ toggle event
    }
    
    // Reset MPPT write cooldown so it can be written immediately when PG=1
    // Don't write MPPT=1 here - let BQ finish input qualification first
    // MPPT=1 will be set by checkAndFixSolarLogic() on next task run when PG=1
    lastMpptWriteTime = 0;
    
    // Set HIZ toggle cooldown timestamp
    lastHizToggleTime = currentTime;

    // Check if toggle was successful (PGOOD now set)
    bqDriverInstance->readReg(0x22); // Refresh status
    lastHizToggleSuccess = bqDriverInstance->getChargerStatusPowerGood();

    MESH_DEBUG_PRINT("Input detection triggered - PGOOD=");
    MESH_DEBUG_PRINTLN(lastHizToggleSuccess ? "OK" : "FAIL");
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
    // PG=0 - nichts zu tun, kein Cooldown setzen
    return;
  }

  // Cooldown: Nur alle 60 Sekunden ausführen um Interrupt-Loop zu vermeiden
  // WICHTIG: Cooldown nur wenn PG=1, damit MPPT sofort gesetzt wird wenn PG high geht
  uint32_t currentTime = millis();
  
  if (lastMpptWriteTime != 0 && (currentTime - lastMpptWriteTime) < 60000) {
    // Weniger als 60 Sekunden seit letztem Durchlauf - skip
    return;
  }

  // Re-enable MPPT only when PowerGood is set (PG=1)
  uint8_t mpptVal = bqDriverInstance->readReg(0x15);

  if ((mpptVal & 0x01) == 0) {
    bqDriverInstance->writeReg(0x15, mpptVal | 0x01);
    lastMpptWriteTime = currentTime; // Cooldown NUR setzen wenn tatsächlich geschrieben
    MESH_DEBUG_PRINTLN("MPPT re-enabled via register (PG=1)");
  }
}

void BoardConfigContainer::solarMpptTask(void* pvParameters) {
  (void)pvParameters;

  delay(2000);

  // Initialize MPPT statistics (v0.1: millis-based only, no RTC)
  memset(&mpptStats, 0, sizeof(MpptStatistics));
  mpptStats.lastUpdateTime = millis() / 1000; // Convert to seconds

  const TickType_t xBlockTime = pdMS_TO_TICKS(SOLAR_MPPT_TASK_INTERVAL_MS);

  while (true) {
    if (xSemaphoreTake(solarEventSem, xBlockTime) == pdTRUE) {
      // Interrupt triggered - solar event occurred
      delay(100);
      checkAndFixPgoodStuck(); // Check for stuck PGOOD
      checkAndFixSolarLogic(); // Re-enable MPPT if needed

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
      checkAndFixPgoodStuck(); // Check for stuck PGOOD
      checkAndFixSolarLogic(); // Re-enable MPPT if needed

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
  
  // CRITICAL: Always clear interrupt flags by reading CHARGER_STATUS_0 register
  // The BQ25798 requires reading register 0x1B to acknowledge interrupts and clear flags.
  // Without this, the interrupt line stays asserted and no new interrupts will be generated.
  // This must happen on EVERY interrupt, regardless of whether we process the event or not.
  if (bqDriverInstance) {
    bqDriverInstance->readReg(0x1B); // Read CHARGER_STATUS_0 (0x1B) to clear interrupt flags
  }

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
    vTaskDelay(pdMS_TO_TICKS(10)); // 10ms flash - well visible, minimal power
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

  // v0.1 hardware: use millis() in seconds (no RTC available)
  uint32_t currentTime = millis() / 1000;

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

    // Move to next index (circular buffer)
    mpptStats.currentIndex = (mpptStats.currentIndex + 1) % MPPT_STATS_HOURS;

    // Reset for new hour
    mpptStats.currentHourMinutes = 0;
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

/// @brief Manually triggers HIZ toggle and reports result
/// @param buffer Destination buffer for status string
/// @param bufferSize Size of destination buffer
/// @details Performs manual HIZ toggle sequence and reports VBUS voltage and success status.
///          Format: "HIZ toggled: VBUS=4.2V PG=OK" or "HIZ toggled: VBUS=3.8V PG=FAIL"
///          Skips toggle if PGOOD already set: "Power already good"
void BoardConfigContainer::toggleHizAndCheck(char* buffer, uint32_t bufferSize) {
  if (!buffer || bufferSize == 0) {
    return;
  }

  memset(buffer, 0, bufferSize);

  if (!BQ_INITIALIZED || !bqDriverInstance) {
    snprintf(buffer, bufferSize, "BQ25798 not initialized");
    return;
  }

  // Refresh status before reading
  bqDriverInstance->readReg(0x22);

  // Skip PGOOD check - allow toggle even when flag register shows PG=1
  // (flag register may be latched while actual status is PG=0)
  
  // Get VBUS voltage
  const Telemetry* telem = bqDriverInstance->getTelemetryData();
  uint16_t vbusVoltage = telem ? telem->solar.voltage : 0;

  // Perform HIZ toggle
  bool wasHIZ = bq.getHIZMode();
  if (wasHIZ) {
    // HIZ already set - just clear it
    bq.setHIZMode(false);
    delay(200);
  } else {
    // Toggle HIZ to force input re-detection
    bq.setHIZMode(true);
    delay(250);
    bq.setHIZMode(false);
  }
  delay(250);             // Allow input qualification
  bq.setMPPTenable(true); // Ensure MPPT remains enabled

  // Record event
  lastHizToggleVbus = vbusVoltage;
  lastHizToggleTimestamp = rtc_clock.getCurrentTime();

  // Check success
  bqDriverInstance->readReg(0x22); // Refresh status
  bool pgoodNow = bqDriverInstance->getChargerStatusPowerGood();
  lastHizToggleSuccess = pgoodNow;

  // Format response
  const char* success = pgoodNow ? "OK" : "FAIL";
  snprintf(buffer, bufferSize, "HIZ toggled: VBUS=%.1fV PG=%s", vbusVoltage / 1000.0f, success);
}

/// @brief Force clear HIZ mode without PGOOD check (for debugging HIZ:1 / PG:0 states)
/// @param buffer Destination buffer for status string
/// @param bufferSize Size of destination buffer
void BoardConfigContainer::clearHiz(char* buffer, uint32_t bufferSize) {
  if (!buffer || bufferSize == 0) {
    return;
  }

  memset(buffer, 0, bufferSize);

  if (!BQ_INITIALIZED || !bqDriverInstance) {
    snprintf(buffer, bufferSize, "BQ25798 not initialized");
    return;
  }

  // Get current HIZ state
  bool wasHIZ = bq.getHIZMode();
  
  // Get VBUS voltage before clearing HIZ
  const Telemetry* telem = bqDriverInstance->getTelemetryData();
  uint16_t vbusVoltage = telem ? telem->solar.voltage : 0;

  // Force clear HIZ mode
  if (wasHIZ) {
    bq.setHIZMode(false);
    delay(250);             // Allow input qualification
    bq.setMPPTenable(true); // Ensure MPPT remains enabled

    // Record event
    lastHizToggleVbus = vbusVoltage;
    lastHizToggleTimestamp = rtc_clock.getCurrentTime();

    // Check if PGOOD is now set
    bqDriverInstance->readReg(0x22); // Refresh status
    bool pgoodNow = bqDriverInstance->getChargerStatusPowerGood();
    lastHizToggleSuccess = pgoodNow;

    const char* success = pgoodNow ? "OK" : "FAIL";
    snprintf(buffer, bufferSize, "HIZ cleared: VBUS=%.1fV PG=%s", vbusVoltage / 1000.0f, success);
  } else {
    snprintf(buffer, bufferSize, "HIZ was already clear (HIZ=0)");
  }
}

/// @brief Get detailed BQ25798 diagnostics for debugging PG / !CHG issues
/// @param buffer Destination buffer for diagnostics string
/// @param bufferSize Size of destination buffer
void BoardConfigContainer::getDetailedDiagnostics(char* buffer, uint32_t bufferSize, int part) {
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
  uint8_t reg15 = bqDriverInstance->readReg(0x15); // MPPT_CONTROL (EN_MPPT at bit 0)
  uint8_t reg1B = bqDriverInstance->readReg(0x1B); // CHARGER_FLAG_0 (PG_STAT bit 3, VINDPM bit 6, IINDPM bit 7)
  uint8_t reg1C = bqDriverInstance->readReg(0x1C); // CHARGER_FLAG_1 (CHG_STAT bits 7:5, VBUS_STAT bits 4:1)
  uint8_t reg20 = bqDriverInstance->readReg(0x20); // FAULT_STATUS_0 (VBUS faults, VBAT faults)
  uint8_t reg21 = bqDriverInstance->readReg(0x21); // FAULT_STATUS_1 (TSBUS, TSBAT, TDIE faults)
  uint8_t reg1F = bqDriverInstance->readReg(0x1F); // CHARGER_FLAG_3 (TS status bits 3:0)

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

  // Get telemetry for voltage/current
  const Telemetry* telem = bqDriverInstance->getTelemetryData();
  float vbus_v = telem ? telem->solar.voltage / 1000.0f : 0.0f;
  float vbat_v = telem ? telem->batterie.voltage / 1000.0f : 0.0f;
  int16_t ibat_ma = telem ? telem->batterie.current : 0;
  float temp_c = telem ? telem->batterie.temperature : 0.0f;

  // VBUS_STAT interpretation (4 bits: 0h-Fh from REG1C bits 4:1)
  const char* vbus_str;
  switch (vbus_stat) {
  case 0x0: vbus_str = "NoIn"; break;
  case 0x1: vbus_str = "SDP"; break;
  case 0x2: vbus_str = "CDP"; break;
  case 0x3: vbus_str = "DCP"; break;
  case 0x4: vbus_str = "HVDCP"; break;
  case 0x5: vbus_str = "UnkAdp"; break;
  case 0x6: vbus_str = "NStd"; break;
  case 0x7: vbus_str = "OTG"; break;
  case 0x8: vbus_str = "NotQual"; break;
  case 0xB: vbus_str = "DirPwr"; break;
  case 0xC: vbus_str = "Backup"; break;
  default: vbus_str = "Rsv"; break;
  }

  // CHG_STAT interpretation
  const char* chg_str;
  switch (chg_stat) {
  case 0: chg_str = "!CHG"; break;
  case 1: chg_str = "TRKL"; break;
  case 2: chg_str = "PRE"; break;
  case 3: chg_str = "CC"; break;
  case 4: chg_str = "CV"; break;
  case 6: chg_str = "TOP"; break;
  case 7: chg_str = "DONE"; break;
  default: chg_str = "???"; break;
  }

  if (part == 1) {
    // DIAG1: Status flags and voltages
    snprintf(buffer, bufferSize,
             "PG:%d CE:%d HIZ:%d MPPT:%d CHG:%s VBUS:%s VINDPM:%d IINDPM:%d | "
             "Vbus:%.2fV Vbat:%.2fV Ibat:%dmA Temp:%.1fC",
             powerGood, !ce_disabled, hiz_enabled, mppt_enabled, chg_str, vbus_str, vindpm, iindpm,
             vbus_v, vbat_v, ibat_ma, temp_c);
  } else {
    // DIAG2: Faults and additional details
    // Extract fault bits from REG20 (FAULT_STATUS_0) per BQ25798 datasheet Table 9-41
    bool ibat_reg = (reg20 & 0x80) != 0;    // Bit 7: IBAT_REG_STAT
    bool vbus_ovp = (reg20 & 0x40) != 0;    // Bit 6: VBUS_OVP_STAT
    bool vbat_ovp = (reg20 & 0x20) != 0;    // Bit 5: VBAT_OVP_STAT
    bool ibus_ocp = (reg20 & 0x10) != 0;    // Bit 4: IBUS_OCP_STAT
    bool ibat_ocp = (reg20 & 0x08) != 0;    // Bit 3: IBAT_OCP_STAT
    bool conv_ocp = (reg20 & 0x04) != 0;    // Bit 2: CONV_OCP_STAT
    bool vac2_ovp = (reg20 & 0x02) != 0;    // Bit 1: VAC2_OVP_STAT
    bool vac1_ovp = (reg20 & 0x01) != 0;    // Bit 0: VAC1_OVP_STAT
    
    // Extract fault bits from REG21 (FAULT_STATUS_1) per BQ25798 datasheet Table 9-42
    bool vsys_short = (reg21 & 0x80) != 0;  // Bit 7: VSYS_SHORT_STAT
    bool vsys_ovp = (reg21 & 0x40) != 0;    // Bit 6: VSYS_OVP_STAT
    bool otg_ovp = (reg21 & 0x20) != 0;     // Bit 5: OTG_OVP_STAT
    bool otg_uvp = (reg21 & 0x10) != 0;     // Bit 4: OTG_UVP_STAT
    bool tshut = (reg21 & 0x04) != 0;       // Bit 2: TSHUT_STAT
    
    // Read VREG for all cases (faults and no faults)
    uint8_t reg01_high = bqDriverInstance->readReg(0x01);
    uint8_t reg01_low = bqDriverInstance->readReg(0x02);
    uint16_t vreg_raw = ((uint16_t)(reg01_high & 0x07) << 8) | reg01_low;
    float vreg_v = (vreg_raw * 10.0f) / 1000.0f;
    
    bool faults = ibat_reg || vbus_ovp || vbat_ovp || ibus_ocp || ibat_ocp || conv_ocp || 
                  vac2_ovp || vac1_ovp || vsys_short || vsys_ovp || otg_ovp || otg_uvp || tshut;
    
    if (faults) {
      snprintf(buffer, bufferSize,
               "FAULTS: %s%s%s%s%s%s%s%s%s%s%s%s%s | VREG:%.2fV R20:0x%02X R21:0x%02X",
               ibat_reg ? "IBAT_REG " : "", vbus_ovp ? "VBUS_OVP " : "", vbat_ovp ? "VBAT_OVP " : "",
               ibus_ocp ? "IBUS_OCP " : "", ibat_ocp ? "IBAT_OCP " : "", conv_ocp ? "CONV_OCP " : "",
               vac2_ovp ? "VAC2_OVP " : "", vac1_ovp ? "VAC1_OVP " : "",
               vsys_short ? "VSYS_SHORT " : "", vsys_ovp ? "VSYS_OVP " : "",
               otg_ovp ? "OTG_OVP " : "", otg_uvp ? "OTG_UVP " : "", tshut ? "TSHUT" : "",
               vreg_v, reg20, reg21);
    } else {
      // VOC_PCT decoding
      const char* voc_pct_str;
      switch (voc_pct) {
      case 0: voc_pct_str = "56.25%"; break;
      case 1: voc_pct_str = "62.5%"; break;
      case 2: voc_pct_str = "68.75%"; break;
      case 3: voc_pct_str = "75%"; break;
      case 4: voc_pct_str = "81.25%"; break;
      case 5: voc_pct_str = "87.5%"; break;
      case 6: voc_pct_str = "93.75%"; break;
      case 7: voc_pct_str = "100%"; break;
      default: voc_pct_str = "???"; break;
      }
      
      // VOC_DLY decoding
      const char* voc_dly_str;
      switch (voc_dly) {
      case 0: voc_dly_str = "50ms"; break;
      case 1: voc_dly_str = "300ms"; break;
      case 2: voc_dly_str = "2s"; break;
      case 3: voc_dly_str = "5s"; break;
      default: voc_dly_str = "???"; break;
      }
      
      // VOC_RATE decoding
      const char* voc_rate_str;
      switch (voc_rate) {
      case 0: voc_rate_str = "30s"; break;
      case 1: voc_rate_str = "2min"; break;
      case 2: voc_rate_str = "10min"; break;
      case 3: voc_rate_str = "30min"; break;
      default: voc_rate_str = "???"; break;
      }
      
      // VREG already read above for fault case
      snprintf(buffer, bufferSize,
               "No faults | TS: %s%s%s%s | VREG:%.2fV | VOC:%s/%s/%s",
               ts_cold ? "COLD " : "", ts_cool ? "COOL " : "", ts_warm ? "WARM " : "",
               ts_hot ? "HOT" : "OK", vreg_v, voc_pct_str, voc_dly_str, voc_rate_str);
    }
  }
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
  const char* statusString = "Unknown"; // Initialize with default value
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

  // Include last HIZ toggle timestamp if available
  if (lastHizToggleTimestamp > 0) {
    DateTime dt = DateTime(lastHizToggleTimestamp);
    const char* success = lastHizToggleSuccess ? "OK" : "FAIL";
    snprintf(buffer, bufferSize, "%s / %s [HIZ:%02d:%02d %d/%d %.1fV %s]", powerGood, statusString, dt.hour(),
             dt.minute(), dt.day(), dt.month(), lastHizToggleVbus / 1000.0f, success);
  } else {
    snprintf(buffer, bufferSize, "%s / %s", powerGood, statusString);
  }
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

  // === v0.1 Hardware: MCP4652 Digital Potentiometer ===
  // Note: INA228 is not available in v0.1 (use MR2 for power monitoring)
  if (mcp.begin()) {
    MCP_INITIALIZED = true;
    MESH_DEBUG_PRINTLN("MCP4652 found @ 0x2F");
  } else {
    MESH_DEBUG_PRINTLN("MCP4652 not found.");
    MCP_INITIALIZED = false;
  }

  // === Common initialization ===
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

  // Configure MCP4652 (MR1 v0.1 hardware)
  this->configureMCP(bat);

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
    BaseType_t taskCreated =
        xTaskCreate(BoardConfigContainer::solarMpptTask, "SolarDaemon", 4096, NULL, 1, &mpptTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create MPPT task!");
      return false;
    }
  }

  if (heartbeatTaskHandle == NULL) {
    BaseType_t taskCreated =
        xTaskCreate(BoardConfigContainer::heartbeatTask, "Heartbeat", 1024, NULL, 1, &heartbeatTaskHandle);
    if (taskCreated != pdPASS) {
      MESH_DEBUG_PRINTLN("Failed to create Heartbeat task!");
      return false;
    }
  }

  // Note: Voltage monitoring not available in v0.1 (use MR2 for advanced power management)

  attachInterrupt(digitalPinToInterrupt(BQ_INT_PIN), onBqInterrupt, FALLING);

  // v0.1 hardware check
  return BQ_INITIALIZED && MCP_INITIALIZED;
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
    if (val > 0 && val <= 3000) { // Max 3A for safety
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

/// @brief Returns pointer to current telemetry data from battery manager
/// @return Pointer to Telemetry struct from BqDriver
const Telemetry* BoardConfigContainer::getTelemetryData() {
  return bq.getTelemetryData();
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
    bat = BatteryType::LIION_1S; // Default
  }
  if (!loadFrost(frost)) {
    frost = FrostChargeBehaviour::NO_REDUCE; // Default (1)
  }
  if (!loadMaxChrgI(maxChargeCurrent_mA)) {
    maxChargeCurrent_mA = 200; // Default
  }
  if (!loadReduceChrgU(reducedBattVoltage)) {
    reducedBattVoltage = false; // Default
  }

  // Apply configuration
  configureBaseBQ();
  configureChemistry(bat, reducedBattVoltage);
  configureMCP(bat);
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
    bq.setJeitaISetC(BQ25798_JEITA_ISETC_UNCHANGED); // Cold region - no current reduction
    bq.setJeitaISetH(BQ25798_JEITA_ISETH_UNCHANGED); // Warm region - no current reduction
    if (reduceMaxChrgU) {
      bq.setChargeLimitV(LTO_2S_VOLTAGE_REDUCED);
    } else {
      bq.setChargeLimitV(LTO_2S_VOLTAGE_NORMAL);
    }
  }

  return true;
}

/// @brief Configures MCP4652 potentiometer for battery chemistry voltage adjustment
/// @param type Battery chemistry type
/// @return true if configuration successful
bool BoardConfigContainer::configureMCP(BatteryType type) {
  if (!MCP_INITIALIZED) {
    return false;
  }

  mcp.setTerminal(MCP4652_CHANNEL_0, MCP4652_TERMINAL_B, true);
  mcp.setTerminal(MCP4652_CHANNEL_0, MCP4652_TERMINAL_W, true);

  switch (type) {
  case BoardConfigContainer::BatteryType::LIION_1S:
    mcp.setWiper(MCP4652_CHANNEL_0, 102);
    mcp.setWiper(MCP4652_CHANNEL_1, 255);
    mcp.setTerminal(MCP4652_CHANNEL_1, MCP4652_TERMINAL_B, true);
    mcp.setTerminal(MCP4652_CHANNEL_1, MCP4652_TERMINAL_W, true);
    break;
  case BoardConfigContainer::BatteryType::LIFEPO4_1S:
    mcp.setWiper(MCP4652_CHANNEL_0, 128);
    mcp.setTerminal(MCP4652_CHANNEL_1, MCP4652_TERMINAL_B, false);
    mcp.setTerminal(MCP4652_CHANNEL_1, MCP4652_TERMINAL_W, false);
    break;
  case BoardConfigContainer::BatteryType::LTO_2S:
    mcp.setWiper(MCP4652_CHANNEL_0, 80);
    mcp.setWiper(MCP4652_CHANNEL_1, 255);
    mcp.setTerminal(MCP4652_CHANNEL_1, MCP4652_TERMINAL_B, true);
    mcp.setTerminal(MCP4652_CHANNEL_1, MCP4652_TERMINAL_W, true);
    break;
  default:
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
  bool mcpConfigured = this->configureMCP(type);
  return bqBaseConfigured && bqConfigured && mcpConfigured;
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

  uint32_t enabledMinutes = 0;

  // Count backwards through the circular buffer (all 168 hours = 7 days)
  for (int i = 0; i < MPPT_STATS_HOURS; i++) {
    int index = (mpptStats.currentIndex - 1 - i + MPPT_STATS_HOURS) % MPPT_STATS_HOURS;

    // Count all hours, treating unfilled entries (timestamp == 0) as 0 minutes
    if (mpptStats.hours[index].timestamp != 0) {
      enabledMinutes += mpptStats.hours[index].mpptEnabledMinutes;
    }
    // Else: hour counts as 0 minutes enabled (missing data treated as 0%)
  }

  // Total minutes in 7 days
  uint32_t totalMinutes = MPPT_STATS_HOURS * 60; // 168 hours * 60 minutes

  return (enabledMinutes * 100.0f) / totalMinutes;
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

  // Count how many hours of data we actually have
  uint32_t validHours = 0;
  for (int i = 0; i < MPPT_STATS_HOURS; i++) {
    if (mpptStats.hours[i].timestamp != 0) {
      validHours++;
    }
  }

  float days = validHours / 24.0f;

  // v0.1: Only show percentage (no mAh due to inaccurate current sensing)
  snprintf(buffer, bufferSize, "7d:%.1f%% (%.1fd)", percentage, days);
}

// ===== End of BoardConfigContainer Implementation =====
