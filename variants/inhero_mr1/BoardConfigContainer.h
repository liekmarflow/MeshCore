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
#pragma once
// #include "lib/SimplePreferences.h"
#include "lib/BqDriver.h"
#include "lib/McpDriver.h"

#include <Arduino.h>

static SemaphoreHandle_t solarEventSem = NULL;

// Solar MPPT task interval
#define SOLAR_MPPT_TASK_INTERVAL_MS (15 * 60 * 1000)  // 15 minutes

// MPPT Statistics tracking for 7-day moving average
#define MPPT_STATS_HOURS 168  // 7 days * 24 hours

typedef struct {
  uint8_t mpptEnabledMinutes;  ///< Minutes MPPT was enabled in this hour (0-60)
  uint32_t timestamp;          ///< Unix timestamp (seconds) for this hour
} MpptHourlyStats;

typedef struct {
  MpptHourlyStats hours[MPPT_STATS_HOURS]; ///< Rolling buffer of hourly stats
  uint8_t currentIndex;                    ///< Current position in circular buffer
  uint32_t lastUpdateTime;                 ///< Last update time (millis)
  uint16_t currentHourMinutes;             ///< Accumulated minutes for current hour
} MpptStatistics;


class BoardConfigContainer {

public:
  // static constexpr char* boardCommands[] = { "bat", "frost", "life", "imax" };

  enum BatteryType : uint8_t { BAT_UNKNOWN = 0, LTO_2S = 1, LIFEPO4_1S = 2, LIION_1S = 3 };
  typedef struct {
    const char* command_string;
    BatteryType type;
  } BatteryMapping;

  // Charge voltage limits for different battery types
  static constexpr float LIION_1S_VOLTAGE_NORMAL = 4.2f;
  static constexpr float LIION_1S_VOLTAGE_REDUCED = 4.05f;
  static constexpr float LIFEPO4_1S_VOLTAGE_NORMAL = 3.6f;
  static constexpr float LIFEPO4_1S_VOLTAGE_REDUCED = 3.45f;
  static constexpr float LTO_2S_VOLTAGE_NORMAL = 5.6f;
  static constexpr float LTO_2S_VOLTAGE_REDUCED = 5.4f;

  static inline constexpr BatteryMapping bat_map[] = { { "lto2s", LTO_2S },
                                                       { "lifepo1s", LIFEPO4_1S },
                                                       { "liion1s", LIION_1S },
                                                       { nullptr, BAT_UNKNOWN } };

  enum FrostChargeBehaviour : uint8_t {
    NO_CHARGE = 4,
    I_REDUCE_TO_20 = 3,
    I_REDUCE_TO_40 = 2,
    NO_REDUCE = 1,
    REDUCE_UNKNOWN = 0
  };
  typedef struct {
    const char* command_string;
    FrostChargeBehaviour type;
  } FrostChargeBehaviourMapping;

  static inline constexpr FrostChargeBehaviourMapping frostchargebehaviour_map[] = {
    { "0%", NO_CHARGE },
    { "20%", I_REDUCE_TO_20 },
    { "40%", I_REDUCE_TO_40 },
    { "100%", NO_REDUCE },
    { nullptr, REDUCE_UNKNOWN }
  };

  // Default values for newly flashed boards
  static constexpr BatteryType DEFAULT_BATTERY_TYPE = LIION_1S;
  static constexpr FrostChargeBehaviour DEFAULT_FROST_BEHAVIOUR = NO_CHARGE;
  static constexpr uint16_t DEFAULT_MAX_CHARGE_CURRENT_MA = 200;
  static constexpr bool DEFAULT_REDUCED_CHARGE_VOLTAGE = false;
  static constexpr bool DEFAULT_MPPT_ENABLED = false;

  static BatteryType getBatteryTypeFromCommandString(const char* cmdStr);
  static char* trim(char* str);
  static const char* getBatteryTypeCommandString(BatteryType type);
  static const char* getFrostChargeBehaviourCommandString(FrostChargeBehaviour type);
  static FrostChargeBehaviour getFrostChargeBehaviourFromCommandString(const char* cmdStr);
  static const char* getAvailableFrostChargeBehaviourOptions();
  static const char* getAvailableBatOptions();
  static void checkAndFixPgoodStuck();  ///< Check for stuck PGOOD and toggle HIZ if needed
  static void checkAndFixSolarLogic();  ///< Re-enable MPPT if BQ disabled it
  static void solarMpptTask(void* pvParameters);
  static void heartbeatTask(void* pvParameters);
  static void onBqInterrupt();
  static bool loadMpptEnabled(bool& enabled);
  static void stopBackgroundTasks(); ///< Stop all background tasks before OTA

  bool setBatteryType(BatteryType type);
  bool setReducedChargeVoltage(bool reduce);

  BatteryType getBatteryType() const;

  bool setFrostChargeBehaviour(FrostChargeBehaviour behaviour);
  FrostChargeBehaviour getFrostChargeBehaviour() const;

  bool getReduceChargeVoltage() const;

  bool setMaxChargeCurrent_mA(uint16_t maxChrgI);
  uint16_t getMaxChargeCurrent_mA() const;

  bool getMPPTEnabled() const;
  bool setMPPTEnable(bool enableMPPT);

  float getMaxChargeVoltage() const;

  bool begin();

  const Telemetry* getTelemetryData();
  bool resetBQ(); ///< Reset BQ25798 to defaults and reconfigure

  const char* getChargeCurrentAsStr();
  void getChargerInfo(char* buffer, uint32_t bufferSize);
  void toggleHizAndCheck(char* buffer, uint32_t bufferSize); ///< Manual HIZ toggle with status report
  void clearHiz(char* buffer, uint32_t bufferSize); ///< Force clear HIZ mode (bypass PGOOD check)
  void getDetailedDiagnostics(char* buffer, uint32_t bufferSize); ///< Get detailed BQ25798 diagnostics for debugging
  
  // MPPT Statistics methods
  float getMpptEnabledPercentage7Day() const;  ///< Get 7-day moving average of MPPT enabled %
  void getMpptStatsString(char* buffer, uint32_t bufferSize) const; ///< Get formatted stats string
  
  // Watchdog methods
  static void setupWatchdog();   ///< Initialize and start hardware watchdog (120s timeout)
  static void feedWatchdog();    ///< Feed the watchdog to prevent reset
  static void disableWatchdog(); ///< Disable watchdog before OTA (cannot truly disable nRF52 WDT)

private:
  static BqDriver* bqDriverInstance; ///< Singleton reference for static methods
  static TaskHandle_t mpptTaskHandle;  ///< Handle for MPPT task cleanup
  static TaskHandle_t heartbeatTaskHandle; ///< Handle for heartbeat task
  static MpptStatistics mpptStats; ///< MPPT statistics data
  static uint32_t lastHizToggleTimestamp; ///< Unix timestamp of last HIZ toggle event (RTC)
  static uint16_t lastHizToggleVbus; ///< VBUS voltage in mV when HIZ toggle occurred
  static bool lastHizToggleSuccess; ///< Whether HIZ toggle successfully restored PGOOD
  
  bool BQ_INITIALIZED = false;
  bool MCP_INITIALIZED = false;  // v0.1 only

  bool setBatteryType(BatteryType type, bool reducedChargeVoltage);

  bool configureBaseBQ();
  bool configureChemistry(BatteryType type, bool reduceMaxChrgU);
  bool configureMCP(BatteryType type);
  bool configureSolarOnlyInterrupts();
  const char* PREFS_NAMESPACE = "inheromr1";
  char* BATTKEY = "batType";
  char* FROSTKEY = "frost";
  char* MAXCHARGECURRENTKEY = "maxChrg";
  char* REDUCEDBATTVOLTAGE = "reduce";
  char* MPPTENABLEKEY = "mpptEn";

  bool loadBatType(BatteryType& type) const;
  bool loadFrost(FrostChargeBehaviour& behaviour) const;
  bool loadMaxChrgI(uint16_t& maxCharge_mA) const;
  bool loadReduceChrgU(bool& reduce) const;
  
  // MPPT Statistics helper
  static void updateMpptStats();
};