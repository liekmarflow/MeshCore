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
#include "lib/BqDriver.h"
#include "lib/Ina228Driver.h"

#include <Arduino.h>

// Solar MPPT polling interval (no interrupt — pure polling)
#define SOLAR_MPPT_TASK_INTERVAL_MS (1 * 60 * 1000)  // 1 minute

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

// Battery SOC Tracking - mAh-based using INA228 Hardware Coulomb Counter (CHARGE register)
#define HOURLY_STATS_HOURS 168  // 7 days * 24 hours = 168 hours

// Hourly battery statistics for rolling window
typedef struct {
  uint32_t timestamp;           ///< Unix timestamp (start of hour, seconds)
  float charged_mah;            ///< Charge added this hour (mAh)
  float discharged_mah;         ///< Charge removed this hour (mAh)
  float solar_mah;              ///< Solar charge contribution this hour (mAh)
} HourlyBatteryStats;

typedef struct {
  // Battery configuration
  float capacity_mah;          ///< Total battery capacity in mAh
  float nominal_voltage;       ///< Nominal voltage for current chemistry (V)
  
  // SOC tracking using INA228 hardware counter (CHARGE register in mAh)
  float current_soc_percent;   ///< Current State of Charge in % (0-100)
  bool soc_valid;              ///< True after first "Charging Done" sync
  float ina228_baseline_mah;   ///< INA228 CHARGE reading at last 100% sync (mAh)

  uint32_t last_soc_update_ms;  ///< millis() of last updateBatterySOC() call (for offset dt)
  
  // Hourly statistics (168-hour rolling buffer for 7 days)
  HourlyBatteryStats hours[HOURLY_STATS_HOURS];
  uint8_t currentIndex;
  uint32_t lastHourUpdateTime;  ///< Last hour boundary timestamp
  
  // Current hour accumulators (reset every hour)
  float current_hour_charged_mah;
  float current_hour_discharged_mah;
  float current_hour_solar_mah;
  float last_charge_reading_mah;  ///< Last INA228 CHARGE reading for delta calculation
  
  // Rolling window statistics (calculated from hourly buffer)
  float last_24h_net_mah;        ///< Net balance over last 24 hours
  float last_24h_charged_mah;    ///< Total charged over last 24 hours
  float last_24h_discharged_mah; ///< Total discharged over last 24 hours
  float avg_3day_daily_net_mah;  ///< Average daily net over last 3 days (72h)
  float avg_3day_daily_charged_mah;    ///< Average daily charged over last 3 days
  float avg_3day_daily_discharged_mah; ///< Average daily discharged over last 3 days
  float avg_7day_daily_net_mah;  ///< Average daily net over last 7 days (168h)
  float avg_7day_daily_charged_mah;    ///< Average daily charged over last 7 days
  float avg_7day_daily_discharged_mah; ///< Average daily discharged over last 7 days
  uint16_t ttl_hours;            ///< Time To Live - hours until battery empty (0 = not calculated).
                                 ///< Based on 7-day rolling avg of daily net deficit (avg_7day_daily_net_mah)
                                 ///< from hourly INA228 Coulomb-counter samples in 168h ring buffer.
  bool living_on_battery;        ///< True if net deficit over last 24h
} BatterySOCStats;

class BoardConfigContainer {

public:
  enum BatteryType : uint8_t { BAT_UNKNOWN = 0, LTO_2S = 1, LIFEPO4_1S = 2, LIION_1S = 3 };
  typedef struct {
    const char* command_string;
    BatteryType type;
  } BatteryMapping;

  // Battery type properties
  typedef struct {
    BatteryType type;
    float charge_voltage;       // Max charge voltage in V
    float nominal_voltage;      // Nominal voltage for energy calculations
    uint16_t lowv_sleep_mv;     // Low-voltage sleep threshold (INA228 ALERT triggers System-Off) in mV
    uint16_t lowv_wake_mv;      // Low-voltage wake threshold (0% SOC marker, RTC wake decision) in mV
    bool charge_enable;         // Enable/disable charging (false for BAT_UNKNOWN)
  } BatteryProperties;

  // Battery properties lookup table
  // All battery-specific thresholds in one central location
  static inline constexpr BatteryProperties battery_properties[] = {
    // Type         ChgV  NomV  SleepMv WakeMv  ChgEn
    { BAT_UNKNOWN,  0.0f, 0.0f, 2000,  2200,   false }, // SAFETY: Safe low thresholds, no charging
    { LTO_2S,       5.4f, 5.0f, 3900,  4100,   true  }, // LTO 2S: 200mV hysteresis
    { LIFEPO4_1S,   3.5f, 3.2f, 2700,  2900,   true  }, // LiFePO4: 200mV hysteresis
    { LIION_1S,     4.1f, 3.7f, 3100,  3300,   true  }  // Li-Ion: 200mV hysteresis
  };

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
  static constexpr BatteryType DEFAULT_BATTERY_TYPE = BAT_UNKNOWN;  // Safe: low thresholds, user must configure
  static constexpr FrostChargeBehaviour DEFAULT_FROST_BEHAVIOUR = NO_CHARGE;
  static constexpr uint16_t DEFAULT_MAX_CHARGE_CURRENT_MA = 200;
  static constexpr bool DEFAULT_MPPT_ENABLED = false;

  static BatteryType getBatteryTypeFromCommandString(const char* cmdStr);
  static char* trim(char* str);
  static const char* getBatteryTypeCommandString(BatteryType type);
  static const char* getFrostChargeBehaviourCommandString(FrostChargeBehaviour type);
  static FrostChargeBehaviour getFrostChargeBehaviourFromCommandString(const char* cmdStr);
  static const char* getAvailableFrostChargeBehaviourOptions();
  static const char* getAvailableBatOptions();
  static const BatteryProperties* getBatteryProperties(BatteryType type);
  
  // Solar Power Management Functions
  // These functions work together to handle stuck PGOOD conditions and MPPT recovery:
  
  static void heartbeatTask(void* pvParameters);

  /// Re-enable MPPT if BQ disabled it (when PG=1).
  static void checkAndFixSolarLogic();

  static bool loadMpptEnabled(bool& enabled);
  void tickPeriodic();              ///< Called from tick() — dispatches all periodic I2C work (MPPT, SOC, hourly stats)
  static void stopBackgroundTasks(); ///< Stop heartbeat task and disarm alerts before OTA

  bool setBatteryType(BatteryType type);

  BatteryType getBatteryType() const;

  bool setFrostChargeBehaviour(FrostChargeBehaviour behaviour);
  FrostChargeBehaviour getFrostChargeBehaviour() const;

  bool setMaxChargeCurrent_mA(uint16_t maxChrgI);
  uint16_t getMaxChargeCurrent_mA() const;

  bool getMPPTEnabled() const;
  bool setMPPTEnable(bool enableMPPT);

  float getMaxChargeVoltage() const;

  bool begin();

  const Telemetry* getTelemetryData(); ///< Get combined telemetry (INA228 for VBAT/IBAT, BQ25798 for Solar)

  const char* getChargeCurrentAsStr();
  void getChargerInfo(char* buffer, uint32_t bufferSize);
  void getDetailedDiagnostics(char* buffer, uint32_t bufferSize); ///< Get detailed BQ25798 diagnostics for debugging
  
  // MPPT Statistics methods
  float getMpptEnabledPercentage7Day() const;  ///< Get 7-day moving average of MPPT enabled %
  uint32_t getAvgDailyEnergy3Day() const;      ///< Get average daily energy over last 3 days (mWh)
  void getMpptStatsString(char* buffer, uint32_t bufferSize) const; ///< Get formatted stats string
  
  // Battery SOC & Coulomb Counter methods
  float getStateOfCharge() const;              ///< Get current SOC in % (0-100)
  float getBatteryCapacity() const;            ///< Get battery capacity in mAh
  bool setBatteryCapacity(float capacity_mah); ///< Set battery capacity manually via CLI (converts to mWh internally)
  bool isBatteryCapacitySet() const;           ///< Check if battery capacity was explicitly set (vs default)
  void getBatterySOCString(char* buffer, uint32_t bufferSize) const; ///< Get formatted SOC string
  void getDailyBalanceString(char* buffer, uint32_t bufferSize) const; ///< Get daily balance stats
  uint16_t getTTL_Hours() const;               ///< Get Time To Live in hours (0 = not calculated)
  bool isLivingOnBattery() const;              ///< True if net deficit over last 24h
  static void syncSOCToFull();                 ///< Sync SOC to 100% after "Charging Done" (resets INA228 baseline)
  static bool setSOCManually(float soc_percent); ///< Manually set SOC to specific value (e.g. after reboot)
  const BatterySOCStats* getSOCStats() const { return &socStats; } ///< Get SOC stats for CLI
  const MpptStatistics* getMpptStats() const { return &mpptStats; } ///< Get MPPT stats for CLI
  static void updateBatterySOC();              ///< Update SOC from INA228 Coulomb Counter

  static float getNominalVoltage(BatteryType type); ///< Get nominal voltage for chemistry type
  void setLowVoltageRecovery() { lowVoltageRecovery = true; } ///< Mark as low-voltage recovery boot
  Ina228Driver* getIna228Driver();             ///< Get INA228 driver instance
  
  // NTC Temperature Calibration methods
  bool setTcCalOffset(float offset_c);           ///< Store temperature calibration offset in °C (persistent)
  float getTcCalOffset() const;                  ///< Get current temperature calibration offset
  float performTcCalibration(float* bme_temp_out = nullptr); ///< Calibrate NTC using BME280 as auto-reference
  static float readBmeTemperature();               ///< Read BME280 temperature directly via I2C
  
  // Low-voltage alert methods (Rev 1.0 — INA228 ALERT on P1.02)
  void armLowVoltageAlert();    ///< Arm INA228 BUVL alert at lowv_sleep_mv (called on battery config)
  static void disarmLowVoltageAlert(); ///< Disarm INA228 BUVL alert and detach ISR
  static void lowVoltageAlertISR(); ///< ISR for INA228 ALERT pin — sets flag (checked in tickPeriodic)
  
  // Voltage threshold helpers (chemistry-specific)
  static uint16_t getLowVoltageSleepThreshold(BatteryType type);   ///< Get sleep threshold (INA228 ALERT)
  static uint16_t getLowVoltageWakeThreshold(BatteryType type);    ///< Get wake threshold (0% SOC marker)
  
  // Watchdog methods
  static void setupWatchdog();   ///< Initialize and start hardware watchdog (120s timeout)
  static void feedWatchdog();    ///< Feed the watchdog to prevent reset
  static void disableWatchdog(); ///< Disable watchdog before OTA (cannot truly disable nRF52 WDT)
  
  // LED control methods
  bool setLEDsEnabled(bool enabled); ///< Enable/disable heartbeat LED and BQ stat LED (persistent)
  bool getLEDsEnabled() const;       ///< Get current LED enable state

private:
  static BqDriver* bqDriverInstance; ///< Singleton reference for static methods
  static Ina228Driver* ina228DriverInstance; ///< Singleton reference for INA228
  static TaskHandle_t heartbeatTaskHandle; ///< Handle for heartbeat task
  static volatile bool lowVoltageAlertFired; ///< ISR flag: INA228 ALERT fired (checked in tickPeriodic)

  // Tick-based scheduling state (millis()-based, overflow-safe)
  uint32_t lastMpptMs = 0;         ///< Last runMpptCycle() execution
  uint32_t lastSocMs = 0;          ///< Last updateBatterySOC() execution
  uint32_t lastHourlyMs = 0;       ///< Last updateHourlyStats() execution
  bool tickInitialized = false;    ///< First-call init flag for MPPT stats

  void runMpptCycle();             ///< Single MPPT cycle (extracted from old solarMpptTask body)
  static MpptStatistics mpptStats; ///< MPPT statistics data
  static BatterySOCStats socStats; ///< Battery SOC statistics
  
  bool BQ_INITIALIZED = false;
  bool INA228_INITIALIZED = false;
  bool lowVoltageRecovery = false;  ///< Set in begin() if booting from low-voltage sleep (GPREGRET2)
  static bool leds_enabled;  // Heartbeat and BQ stat LED control (static for ISR access)
  static float tcCalOffset;   // NTC temperature calibration offset in °C (0.0 = no calibration)

  bool configureBaseBQ();
  bool configureChemistry(BatteryType type);
  bool configureSolarOnlyInterrupts();
  float performTcCalibration(float actual_temp_c); ///< Internal: calibrate NTC given reference temp (called by BME auto-cal)
  static constexpr const char* PREFS_NAMESPACE = "inheromr2";
  static constexpr const char* BATTKEY = "batType";
  static constexpr const char* FROSTKEY = "frost";
  static constexpr const char* MAXCHARGECURRENTKEY = "maxChrg";
  static constexpr const char* MPPTENABLEKEY = "mpptEn";
  static constexpr const char* BATTERY_CAPACITY_KEY = "batCap";
  static constexpr const char* TCCAL_KEY = "tcCal";              // NTC temperature calibration offset


  bool loadBatType(BatteryType& type) const;
  bool loadFrost(FrostChargeBehaviour& behaviour) const;
  bool loadMaxChrgI(uint16_t& maxCharge_mA) const;
  bool loadBatteryCapacity(float& capacity_mah) const;
  bool loadTcCalOffset(float& offset) const;  // NTC temperature calibration

  
  // MPPT Statistics helper
  static void updateMpptStats();

  // Battery SOC helpers
  static void updateHourlyStats();   ///< Update hourly statistics (called every 60 minutes)
  static void calculateRollingStats(); ///< Calculate 24h and 3-day averages from rolling buffer
  static void calculateTTL();    ///< Calculate TTL from 7-day avg net deficit and remaining SOC capacity
  static float estimateSOCFromVoltage(uint16_t voltage_mv, BatteryType type);
};