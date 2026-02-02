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
#include "lib/Ina228Driver.h"  // v0.2 power monitor

#include <Arduino.h>

static SemaphoreHandle_t solarEventSem = NULL;

// Solar MPPT task interval
#define SOLAR_MPPT_TASK_INTERVAL_MS (15 * 60 * 1000)  // 15 minutes

// MPPT Statistics tracking for 7-day moving average
#define MPPT_STATS_HOURS 168  // 7 days * 24 hours

typedef struct {
  uint8_t mpptEnabledMinutes;  ///< Minutes MPPT was enabled in this hour (0-60)
  uint32_t timestamp;          ///< Unix timestamp (seconds) for this hour
  uint32_t harvestedEnergy_mWh; ///< Harvested solar energy (mWh) during this hour
} MpptHourlyStats;

typedef struct {
  MpptHourlyStats hours[MPPT_STATS_HOURS]; ///< Rolling buffer of hourly stats
  uint8_t currentIndex;                    ///< Current position in circular buffer
  uint32_t lastUpdateTime;                 ///< Last update time (Unix seconds or millis if RTC unavailable)
  uint16_t currentHourMinutes;             ///< Accumulated minutes for current hour
  bool usingRTC;                           ///< True if using RTC, false if using millis fallback
  uint32_t currentHourEnergy_mWh;          ///< Accumulated energy for current hour (mWh)
  int32_t lastPower_mW;                    ///< Last measured power for energy calculation
} MpptStatistics;

// Battery Coulomb Counter & SOC Tracking (v0.2)
#define DAILY_STATS_DAYS 7  // 7 days of daily statistics

typedef struct {
  uint32_t timestamp;           ///< Unix timestamp (start of day, seconds)
  int32_t charge_mah;          ///< Net charge for this day (+ = gained, - = lost)
  int32_t discharge_mah;       ///< Total discharge for this day (always positive)
  int32_t solar_charge_mah;    ///< Solar contribution for this day
  int32_t net_balance_mah;     ///< Net balance (solar - discharge)
} DailyBatteryStats;

typedef struct {
  DailyBatteryStats days[DAILY_STATS_DAYS]; ///< Rolling buffer of daily stats
  uint8_t currentIndex;                     ///< Current position in circular buffer
  uint32_t lastUpdateTime;                  ///< Last update timestamp
  
  // Current day accumulators
  int32_t today_charge_mah;
  int32_t today_discharge_mah;
  int32_t today_solar_mah;
  
  // Battery capacity tracking
  float battery_capacity_mah;  ///< Total battery capacity (learned or configured)
  float current_soc_percent;   ///< Current State of Charge in %
  bool capacity_learned;       ///< True if capacity was learned from full cycle
  
  // Auto-learning state
  bool learning_active;        ///< Currently in learning cycle
  float learning_start_soc;    ///< SOC at start of learning
  float learning_accumulated_mah; ///< Accumulated charge during learning
  
  // Forecast
  float avg_daily_deficit_mah; ///< 3-day average deficit (negative = using battery)
  uint16_t ttl_hours;          ///< Time To Live - hours until battery empty (0 = not calculated)
  bool living_on_battery;      ///< True if net deficit over last 24h
} BatterySOCStats;

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
  static void checkAndFixSolarLogic();
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
  
  // MPPT Statistics methods
  float getMpptEnabledPercentage7Day() const;  ///< Get 7-day moving average of MPPT enabled %
  uint32_t getAvgDailyEnergy3Day() const;      ///< Get average daily energy over last 3 days (mWh)
  void getMpptStatsString(char* buffer, uint32_t bufferSize) const; ///< Get formatted stats string
  
  // Battery SOC & Coulomb Counter methods (v0.2)
  float getStateOfCharge() const;              ///< Get current SOC in % (0-100)
  float getBatteryCapacity() const;            ///< Get battery capacity in mAh
  bool setBatteryCapacity(float capacity_mah); ///< Set battery capacity manually via CLI
  void getBatterySOCString(char* buffer, uint32_t bufferSize) const; ///< Get formatted SOC string
  void getDailyBalanceString(char* buffer, uint32_t bufferSize) const; ///< Get daily balance stats
  uint16_t getTTL_Hours() const;               ///< Get Time To Live in hours (0 = not calculated)
  bool isLivingOnBattery() const;              ///< True if net deficit over last 24h
  static void voltageMonitorTask(void* pvParameters); ///< Voltage monitor with SOC tracking (v0.2)
  static void updateBatterySOC();              ///< Update SOC from INA228 Coulomb Counter
  Ina228Driver* getIna228Driver();             ///< Get INA228 driver instance (v0.2)
  
  // INA228 Calibration methods (v0.2)
  bool setIna228CalibrationFactor(float factor); ///< Store INA228 current calibration factor
  float getIna228CalibrationFactor() const;      ///< Get current INA228 calibration factor
  float performIna228Calibration(float actual_current_ma); ///< Perform calibration and store factor
  
  // Watchdog methods
  static void setupWatchdog();   ///< Initialize and start hardware watchdog (120s timeout)
  static void feedWatchdog();    ///< Feed the watchdog to prevent reset
  static void disableWatchdog(); ///< Disable watchdog before OTA (cannot truly disable nRF52 WDT)

private:
  static BqDriver* bqDriverInstance; ///< Singleton reference for static methods
  static Ina228Driver* ina228DriverInstance; ///< Singleton reference for INA228 (v0.2 hardware) (v0.2)
  static TaskHandle_t mpptTaskHandle;  ///< Handle for MPPT task cleanup
  static TaskHandle_t heartbeatTaskHandle; ///< Handle for heartbeat task
  static TaskHandle_t voltageMonitorTaskHandle; ///< Handle for voltage monitor task (v0.2)
  static MpptStatistics mpptStats; ///< MPPT statistics data
  static BatterySOCStats socStats; ///< Battery SOC statistics (v0.2)
  
  bool BQ_INITIALIZED = false;
  bool INA228_INITIALIZED = false;  // v0.2 only (MR2)

  bool setBatteryType(BatteryType type, bool reducedChargeVoltage);

  bool configureBaseBQ();
  bool configureChemistry(BatteryType type, bool reduceMaxChrgU);
  // configureMCP() removed - v0.1 only, MR2 doesn't have MCP4652
  bool configureSolarOnlyInterrupts();
  const char* PREFS_NAMESPACE = "inheromr1";
  char* BATTKEY = "batType";
  char* FROSTKEY = "frost";
  char* MAXCHARGECURRENTKEY = "maxChrg";
  char* REDUCEDBATTVOLTAGE = "reduce";
  char* MPPTENABLEKEY = "mpptEn";
  char* BATTERY_CAPACITY_KEY = "batCap";  // v0.2: Battery capacity in mAh
  char* INA228_CALIB_KEY = "ina228Cal";   // v0.2: INA228 current calibration factor

  bool loadBatType(BatteryType& type) const;
  bool loadFrost(FrostChargeBehaviour& behaviour) const;
  bool loadMaxChrgI(uint16_t& maxCharge_mA) const;
  bool loadReduceChrgU(bool& reduce) const;
  bool loadBatteryCapacity(float& capacity_mah) const; // v0.2
  bool loadIna228CalibrationFactor(float& factor) const; // v0.2
  
  // MPPT Statistics helper
  static void updateMpptStats();
  
  // Battery SOC helpers (v0.2)
  static void updateDailyBalance();
  static void calculateTTL();
  static float estimateSOCFromVoltage(uint16_t voltage_mv, BatteryType type);
};