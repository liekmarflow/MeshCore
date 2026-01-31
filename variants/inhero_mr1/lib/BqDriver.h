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
 *
 * -------------------------------------------------------------------------
 * Dependencies:
 * This driver extends the Adafruit_BQ25798 library (BSD License).
 * Requires <Adafruit_BQ25798.h> to be installed/available.
 * -------------------------------------------------------------------------
 */

#pragma once

#include <Adafruit_BQ25798.h>
#include <Wire.h>
#include <math.h>

#define R_PULLUP    5600.0f  ///< Upper resistor RT1 in Ohms
#define R_PARALLEL  27000.0f ///< Lower parallel resistor RT2 in Ohms
#define R_NTC_25    10000.0f ///< NTC nominal value at 25°C in Ohms
#define BETA_VAL    3380.0f  ///< Beta value of NCP15XH103F03RC NTC
#define T_25_KELVIN 298.15f  ///< 25°C in Kelvin (273.15 + 25)
// Temperature correction lookup table (Beta error compensation)
// Calibrated from actual measurements with BME280 reference
// Format: {Beta_calculated_temp, offset_to_apply}
static const float TEMP_CORRECTION[][2] = {
  {-30.0f, -7.0f},  // Extrapolated for extreme cold
  {-15.0f, -6.6f},  // At -18.3°C real: measured -17.5, need -0.8 more
  {5.0f,   -3.6f},  // At 1.4°C real: worked well before
  {17.0f,  -2.8f},  // At 18.5°C real: measured 17.1, need +1.4 less negative
  {25.0f,  -2.5f},  // Nominal, offset reduces
  {50.0f,  -2.0f}   // Extrapolated for warm temps
};
#define IBUS_ADC_OFFSET_MA 27    ///< ADC calibration offset for solar current in mA

/// Solar input telemetry data
typedef struct {
  uint16_t voltage; ///< Solar voltage in mV
  int16_t current;  ///< Solar current in mA
  int32_t power;    ///< Solar power in mW
  bool mppt;        ///< MPPT enabled status
} SolarData;

/// Battery telemetry data
typedef struct {
  uint16_t voltage; ///< Battery voltage in mV
  int16_t current;  ///< Battery current in mA (positive = charging, negative = discharging)
  int32_t power;    ///< Battery power in mW
  float temperature; ///< Battery temperature in °C
} BattData;

/// System voltage telemetry
typedef struct {
  uint16_t voltage; ///< System voltage in mV
} SysData;

/// Main telemetry container aggregating all data sources
typedef struct {
  SysData system;     ///< System voltage data
  SolarData solar;    ///< Solar input data
  BattData batterie;  ///< Battery data
} Telemetry;

/// JEITA voltage setting for warm/cool regions (NTC Control 0 Register 0x17)
typedef enum {
  BQ25798_JEITA_VSET_SUSPEND = 0x00,     ///< Charge Suspend
  BQ25798_JEITA_VSET_MINUS_800MV = 0x01, ///< Set VREG to VREG-800mV
  BQ25798_JEITA_VSET_MINUS_600MV = 0x02, ///< Set VREG to VREG-600mV
  BQ25798_JEITA_VSET_MINUS_400MV = 0x03, ///< Set VREG to VREG-400mV (default)
  BQ25798_JEITA_VSET_MINUS_300MV = 0x04, ///< Set VREG to VREG-300mV
  BQ25798_JEITA_VSET_MINUS_200MV = 0x05, ///< Set VREG to VREG-200mV
  BQ25798_JEITA_VSET_MINUS_100MV = 0x06, ///< Set VREG to VREG-100mV
  BQ25798_JEITA_VSET_UNCHANGED = 0x07    ///< VREG unchanged
} bq25798_jeita_vset_t;

typedef enum {
  BQ25798_JEITA_ISETH_SUSPEND = 0x00,    ///< Charge Suspend
  BQ25798_JEITA_ISETH_20_PERCENT = 0x01, ///< Set ICHG to 20% * ICHG
  BQ25798_JEITA_ISETH_40_PERCENT = 0x02, ///< Set ICHG to 40% * ICHG
  BQ25798_JEITA_ISETH_UNCHANGED = 0x03   ///< ICHG unchanged (default)
} bq25798_jeita_iseth_t;

typedef enum {
  BQ25798_JEITA_ISETC_SUSPEND = 0x00,    ///< Charge Suspend
  BQ25798_JEITA_ISETC_20_PERCENT = 0x01, ///< Set ICHG to 20% * ICHG (default)
  BQ25798_JEITA_ISETC_40_PERCENT = 0x02, ///< Set ICHG to 40% * ICHG
  BQ25798_JEITA_ISETC_UNCHANGED = 0x03   ///< ICHG unchanged
} bq25798_jeita_isetc_t;

typedef enum {
  BQ25798_CHARGER_STATE_NOT_CHARGING = 0x00,
  BQ25798_CHARGER_STATE_TRICKLE_CHARGING = 0x01,
  BQ25798_CHARGER_STATE_PRE_CHARGING = 0x02,
  BQ25798_CHARGER_STATE_CC_CHARGING = 0x03,
  BQ25798_CHARGER_STATE_CV_CHARGING = 0x04,
  BQ25798_CHARGER_STATE_TOP_OF_TIMER_ACTIVE_CHARGING = 0x06,
  BQ25798_CHARGER_STATE_DONE_CHARGING = 0x07

} bq25798_charging_status;

/// New enums for NTC Control 1 (Register 0x18)
typedef enum {
  BQ25798_TS_COOL_5C = 0x00,  ///< 71.1% of REGN (5°C)
  BQ25798_TS_COOL_10C = 0x01, ///< 68.4% of REGN (10°C, default)
  BQ25798_TS_COOL_15C = 0x02, ///< 65.5% of REGN (15°C)
  BQ25798_TS_COOL_20C = 0x03  ///< 62.4% of REGN (20°C)
} bq25798_ts_cool_t;

typedef enum {
  BQ25798_TS_WARM_40C = 0x00, ///< 48.4% of REGN (40°C)
  BQ25798_TS_WARM_45C = 0x01, ///< 44.8% of REGN (45°C, default)
  BQ25798_TS_WARM_50C = 0x02, ///< 41.2% of REGN (50°C)
  BQ25798_TS_WARM_55C = 0x03  ///< 37.7% of REGN (55°C)
} bq25798_ts_warm_t;

/// BHOT threshold - upper temperature limit for charging
typedef enum {
  BQ25798_BHOT_55C = 0x00,    ///< 55°C
  BQ25798_BHOT_60C = 0x01,    ///< 60°C (default)
  BQ25798_BHOT_65C = 0x02,    ///< 65°C
  BQ25798_BHOT_DISABLE = 0x03 ///< Disable BHOT protection
} bq25798_bhot_t;

/// BCOLD threshold - lower temperature limit for charging
typedef enum {
  BQ25798_BCOLD_MINUS_10C = 0x00, ///< -10°C (default)
  BQ25798_BCOLD_MINUS_20C = 0x01  ///< -20°C
} bq25798_bcold_t;

/// ADC resolution setting
typedef enum {
  BQ25798_ADC_SAMPLE_15BIT = 0b00, ///< 15-bit resolution (default, ~24ms conversion)
  BQ25798_ADC_SAMPLE_14BIT = 0b01, ///< 14-bit resolution
  BQ25798_ADC_SAMPLE_13BIT = 0b10, ///< 13-bit resolution
  BQ25798_ADC_SAMPLE_12BIT = 0b11  ///< 12-bit resolution (not recommended)
} bq25798_adc_sample_t;

/**
 * @brief Extended BQ25798 driver with NTC support and comprehensive telemetry
 * 
 * Extends Adafruit_BQ25798 with:
 * - JEITA temperature control (VSET, ISETH, ISETC)
 * - NTC thermistor temperature calculation
 * - Complete ADC telemetry (solar, battery, system)
 * - One-shot ADC conversion management
 */
class BqDriver : public Adafruit_BQ25798 {
public:
  BqDriver();
  ~BqDriver();

  bool begin(uint8_t i2c_addr = BQ25798_DEFAULT_ADDR, TwoWire* wire = &Wire);

  // In der Klasse Adafruit_BQ25798 (unter den bestehenden Funktionsprototypen):
  bq25798_jeita_vset_t getJeitaVSet();
  bool setJeitaVSet(bq25798_jeita_vset_t setting);

  bq25798_jeita_iseth_t getJeitaISetH();
  bool setJeitaISetH(bq25798_jeita_iseth_t setting);

  bq25798_jeita_isetc_t getJeitaISetC();
  bool setJeitaISetC(bq25798_jeita_isetc_t setting);

  bq25798_ts_cool_t getTsCool();
  bool setTsCool(bq25798_ts_cool_t threshold);

  bq25798_ts_warm_t getTsWarm();
  bool setTsWarm(bq25798_ts_warm_t threshold);

  bq25798_bhot_t getBHot();
  bool setBHot(bq25798_bhot_t threshold);

  bq25798_bcold_t getBCold();
  bool setBCold(bq25798_bcold_t threshold);

  bool getTsIgnore();
  bool setTsIgnore(bool ignore);

  const Telemetry* const getTelemetryData();
  
  /// @brief Read VBAT directly via I2C (no initialization required)
  /// @param wire Pointer to TwoWire instance (default: &Wire)
  /// @return Battery voltage in mV, or 0 if read fails
  /// @note Static method for early boot use before driver initialization
  static uint16_t readVBATDirect(TwoWire* wire = &Wire);

  bool stopIbatADC();
  bool startIbatADC();

  // Charger Status
  bool getChargerStatusPowerGood();
  bq25798_charging_status getChargingStatus();

  bool configureSolarOnlyInterrupts();

  bool checkAndClearPgFlag();

  // Non-static register access methods (use instance I2C config)
  bool writeReg(uint8_t reg, uint8_t val);
  uint8_t readReg(uint8_t reg);

protected:
  Adafruit_I2CDevice* ih_i2c_dev = nullptr; ///< Dedicated I2C device for NTC access

private:
  // In MyBQ25798 class (under NTC functions):
  // ADC Control
  bool startADCOneShot();

  bool getADCEnabled();
  bool setADCEnabled(bool enabled);

  bool getADCRate(); // false = continuous, true = one-shot
  bool setADCRate(bool oneshot);

  bq25798_adc_sample_t getADCSample();
  bool setADCSample(bq25798_adc_sample_t sample);

  bool getADCAvg();
  bool setADCAvg(bool avg);

  bool getADCAvgInit();
  bool setADCAvgInit(bool init);

  // ADC Function Disable 0 (0x2F)
  bool getIBUSADCDisable();
  bool setIBUSADCDisable(bool disable);

  bool getIBATADCDisable();
  bool setIBATADCDisable(bool disable);

  bool getVBUSADCDisable();
  bool setVBUSADCDisable(bool disable);

  bool getVSYSADCDisable();
  bool setVSYSADCDisable(bool disable);

  bool getTSADCDisable();
  bool setTSADCDisable(bool disable);

  bool getTDIEADCDisable();
  bool setTDIEADCDisable(bool disable);

  // ADC Function Disable 1 (0x30)
  bool getDPADCDisable();
  bool setDPADCDisable(bool disable);

  bool getDMADCDisable();
  bool setDMADCDisable(bool disable);

  bool getVAC2ADCDisable();
  bool setVAC2ADCDisable(bool disable);

  bool getVAC1ADCDisable();
  bool setVAC1ADCDisable(bool disable);

  int16_t getIBUS();
  int16_t getIBAT();
  uint16_t getVBUS();
  uint16_t getVBAT();
  uint16_t getVSYS();

  float getTS();   // TS voltage in % of REGN
  float getTDIE(); // Die temperature in °C (signed)

  float calculateBatteryTemp(float ts_pct);
  Telemetry telemetryData = { 0 };
};