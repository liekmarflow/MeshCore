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
#include "BqDriver.h"

#include <MeshCore.h>

/// @brief Default constructor
BqDriver::BqDriver() {}

/// @brief Destructor - cleans up I2C device
BqDriver::~BqDriver() {
  if (ih_i2c_dev) {
    delete ih_i2c_dev;
    ih_i2c_dev = nullptr;
  }
}

/// @brief Initializes BQ25798 charger and creates dedicated I2C device for NTC access
/// @param i2c_addr I2C address of BQ25798 (default: 0x6B)
/// @param wire Pointer to TwoWire instance (default: &Wire)
/// @return true if initialization successful
bool BqDriver::begin(uint8_t i2c_addr, TwoWire* wire) {
  if (!Adafruit_BQ25798::begin(i2c_addr, wire)) {
    // Cleanup any existing device before returning
    if (ih_i2c_dev) {
      delete ih_i2c_dev;
      ih_i2c_dev = nullptr;
    }
    return false;
  }
  if (ih_i2c_dev) {
    delete ih_i2c_dev;
  }
  ih_i2c_dev = new Adafruit_I2CDevice(i2c_addr, wire);
  if (!ih_i2c_dev->begin()) {
    // Cleanup on failure
    delete ih_i2c_dev;
    ih_i2c_dev = nullptr;
    return false;
  }
  return true;
}

/// @brief Reads Power Good status from charger
/// @return true if input power is good (sufficient for charging)
bool BqDriver::getChargerStatusPowerGood() {
  Adafruit_BusIO_Register chrg_stat_0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_0);
  Adafruit_BusIO_RegisterBits chrg_stat_0_bits = Adafruit_BusIO_RegisterBits(&chrg_stat_0_reg, 1, 3);

  uint8_t reg_value = chrg_stat_0_bits.read();

  return (bool)reg_value;
}

/// @brief Reads current charging state from charger
/// @return Charging status enum (NOT_CHARGING, PRE_CHARGING, CC, CV, etc.)
bq25798_charging_status BqDriver::getChargingStatus() {
  Adafruit_BusIO_Register chrg_stat_1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_1);
  Adafruit_BusIO_RegisterBits chrg_stat_1_bits = Adafruit_BusIO_RegisterBits(&chrg_stat_1_reg, 3, 5);

  uint8_t reg_value = chrg_stat_1_bits.read();

  return (bq25798_charging_status)reg_value;
}

/// @brief Configures interrupt masks to only trigger on Power Good changes (solar events)
/// @return true if all mask registers configured successfully
bool BqDriver::configureSolarOnlyInterrupts() {
  Adafruit_BusIO_Register mask0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_0);
  if (!mask0.write(0xF7)) return false;

  Adafruit_BusIO_Register mask1 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_1);
  if (!mask1.write(0xFF)) return false;

  Adafruit_BusIO_Register mask2 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_2);
  if (!mask2.write(0xFF)) return false;

  Adafruit_BusIO_Register mask3 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_MASK_3);
  if (!mask3.write(0xFF)) return false;

  // 5. Fault Mask 0 (0x2C) - Mask everything (0xFF)
  Adafruit_BusIO_Register fault0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_FAULT_MASK_0);
  if (!fault0.write(0xFF)) return false;

  // 6. Fault Mask 1 (0x2D) - Mask everything (0xFF)
  Adafruit_BusIO_Register fault1 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_FAULT_MASK_1);
  if (!fault1.write(0xFF)) return false;

  // 7. Clear existing interrupts ("Flush")
  // We read Charger Flag 0 (0x22) where PG_FLAG resides.
  // Reading resets the INT pin on the chip back to HIGH.
  Adafruit_BusIO_Register flag0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_FLAG_0);
  uint8_t dummy;
  flag0.read(&dummy); // Dummy Read zum Clearen

  return true;
}

/// @brief Checks and clears Power Good flag register
/// @return true if PG_FLAG bit is set
bool BqDriver::checkAndClearPgFlag() {
  Adafruit_BusIO_Register flag0 = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_FLAG_0);
  uint8_t val;
  if (!flag0.read(&val)) return false;
  return (val & 0x08);
}

/// @brief Reads all telemetry data (solar, battery, system) via ADC one-shot
/// @return Pointer to internal Telemetry struct (valid until next call)
const Telemetry* const BqDriver::getTelemetryData() {
  telemetryData = { 0 };
  this->startIbatADC();
  bool success = this->startADCOneShot();

  if (!success) {
    return &telemetryData;
  }

  delay(170);
  
  telemetryData.solar.voltage = getVBUS();
  telemetryData.solar.current = getIBUS() - IBUS_ADC_OFFSET_MA;
  if (telemetryData.solar.current < 0) {
    telemetryData.solar.current = 0;
  }
  telemetryData.solar.power = ((int32_t)telemetryData.solar.voltage * telemetryData.solar.current) / 1000;
  telemetryData.solar.mppt = getMPPTenable();

  telemetryData.batterie.voltage = getVBAT();
  telemetryData.batterie.current = getIBAT();
  telemetryData.batterie.power =
      ((int32_t)telemetryData.batterie.voltage * telemetryData.batterie.current) / 1000;
  telemetryData.batterie.temperature = this->calculateBatteryTemp(getTS());

  telemetryData.system.voltage = getVSYS();
  this->stopIbatADC();
  return &telemetryData;
}

/**
 * Calculates battery temperature in °C
 * Per BQ25798 datasheet Figure 9-12: REGN → RT1 → TS → (RT2||NTC) → GND
 * @param ts_pct Voltage at TS pin in percentage of REGN (e.g., 70.5 for 70.5%)
 * @return Temperature in °C
 */
float BqDriver::calculateBatteryTemp(float ts_pct) {
  // Convert TS percentage to ratio (0.0 to 1.0)
  // TS% = 100 × R_bottom / (R_top + R_bottom)
  // where R_bottom = RT2 || NTC
  float k = ts_pct / 100.0f;

  // Plausibility check
  if (k > 0.99f) return -99.0f; // NTC open/disconnected
  if (k < 0.01f) return 99.0f;  // NTC short circuit

  // Calculate total resistance of bottom network (RT2 || NTC)
  // From: k = R_bottom / (RT1 + R_bottom)
  // Rearranged: R_bottom = RT1 × k / (1 - k)
  float r_bottom_total = R_PULLUP * (k / (1.0f - k));

  // Extract NTC resistance from parallel combination with RT2
  // For parallel resistors: 1/R_total = 1/R_NTC + 1/RT2
  // Therefore: 1/R_NTC = 1/R_total - 1/RT2
  float g_total = 1.0f / r_bottom_total;
  float g_rt2 = 1.0f / R_PARALLEL;

  if (g_total <= g_rt2) {
    return -99.0f; // Invalid measurement
  }

  float r_ntc = 1.0f / (g_total - g_rt2);

  // Apply Beta equation: R = R0 × exp(B × (1/T - 1/T0))
  // Inverted: 1/T = 1/T0 + (1/B) × ln(R/R0)
  float inv_T = (1.0f / T_25_KELVIN) + (1.0f / BETA_VAL) * logf(r_ntc / R_NTC_25);

  // Convert Kelvin to Celsius
  float temp_c = (1.0f / inv_T) - 273.15f;
  
  // Apply temperature-dependent offset correction using linear interpolation
  // Compensates for Beta equation non-linearity
  float offset = -4.0f;  // Default
  
  const int table_size = sizeof(TEMP_CORRECTION) / sizeof(TEMP_CORRECTION[0]);
  
  if (temp_c <= TEMP_CORRECTION[0][0]) {
    offset = TEMP_CORRECTION[0][1];  // Below lowest temp
  } else if (temp_c >= TEMP_CORRECTION[table_size-1][0]) {
    offset = TEMP_CORRECTION[table_size-1][1];  // Above highest temp
  } else {
    // Linear interpolation between table points
    for (int i = 0; i < table_size - 1; i++) {
      if (temp_c >= TEMP_CORRECTION[i][0] && temp_c <= TEMP_CORRECTION[i+1][0]) {
        float t1 = TEMP_CORRECTION[i][0];
        float t2 = TEMP_CORRECTION[i+1][0];
        float o1 = TEMP_CORRECTION[i][1];
        float o2 = TEMP_CORRECTION[i+1][1];
        float ratio = (temp_c - t1) / (t2 - t1);
        offset = o1 + ratio * (o2 - o1);
        break;
      }
    }
  }
  
  temp_c += offset;

  return temp_c;
}

// Getter/Setter for NTC Control 0 (0x17)
/// @brief Gets JEITA voltage setting for warm/cool regions
/// @return JEITA VSET enum value
bq25798_jeita_vset_t BqDriver::getJeitaVSet() {
  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_vset_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 3, 5);

  uint8_t reg_value = jeita_vset_bits.read();

  return (bq25798_jeita_vset_t)reg_value;
}

/// @brief Sets JEITA voltage setting for warm/cool temperature regions
/// @param setting JEITA VSET enum (suspend, or VREG offset)
/// @return true if successful
bool BqDriver::setJeitaVSet(bq25798_jeita_vset_t setting) {
  if (setting > BQ25798_JEITA_VSET_UNCHANGED) {
    return false;
  }

  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_vset_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 3, 5);

  jeita_vset_bits.write((uint8_t)setting);

  return true;
}

/// @brief Gets JEITA current setting for hot region
/// @return JEITA ISETH enum value
bq25798_jeita_iseth_t BqDriver::getJeitaISetH() {
  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_iseth_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 3);

  uint8_t reg_value = jeita_iseth_bits.read();

  return (bq25798_jeita_iseth_t)reg_value;
}

/// @brief Sets JEITA current setting for hot temperature region
/// @param setting JEITA ISETH enum (suspend or percentage)
/// @return true if successful
bool BqDriver::setJeitaISetH(bq25798_jeita_iseth_t setting) {
  if (setting > BQ25798_JEITA_ISETH_UNCHANGED) {
    return false;
  }

  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_iseth_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 3);

  jeita_iseth_bits.write((uint8_t)setting);

  return true;
}

/// @brief Gets JEITA current setting for cold region
/// @return JEITA ISETC enum value
bq25798_jeita_isetc_t BqDriver::getJeitaISetC() {
  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_isetc_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 1);

  uint8_t reg_value = jeita_isetc_bits.read();

  return (bq25798_jeita_isetc_t)reg_value;
}

/// @brief Sets JEITA current setting for cold temperature region
/// @param setting JEITA ISETC enum (suspend or percentage)
/// @return true if successful
bool BqDriver::setJeitaISetC(bq25798_jeita_isetc_t setting) {
  if (setting > BQ25798_JEITA_ISETC_UNCHANGED) {
    return false;
  }

  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_isetc_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 1);

  jeita_isetc_bits.write((uint8_t)setting);

  return true;
}

/// @brief Gets TS Cool threshold (lower boundary of COOL region)
/// @return TS COOL enum value
bq25798_ts_cool_t BqDriver::getTsCool() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_cool_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 6);

  uint8_t reg_value = ts_cool_bits.read();

  return (bq25798_ts_cool_t)reg_value;
}

/// @brief Sets TS Cool threshold (lower boundary of COOL region)
/// @param threshold TS COOL enum (0°C to 20°C)
/// @return true if successful
bool BqDriver::setTsCool(bq25798_ts_cool_t threshold) {
  if (threshold > BQ25798_TS_COOL_20C) {
    return false;
  }

  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_cool_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 6);

  ts_cool_bits.write((uint8_t)threshold);

  return true;
}

/// @brief Gets TS Warm threshold (upper boundary of WARM region)
/// @return TS WARM enum value
bq25798_ts_warm_t BqDriver::getTsWarm() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_warm_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 4);

  uint8_t reg_value = ts_warm_bits.read();

  return (bq25798_ts_warm_t)reg_value;
}

/// @brief Sets TS Warm threshold (upper boundary of WARM region)
/// @param threshold TS WARM enum (40°C to 55°C)
/// @return true if successful
bool BqDriver::setTsWarm(bq25798_ts_warm_t threshold) {
  if (threshold > BQ25798_TS_WARM_55C) {
    return false;
  }

  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_warm_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 4);

  ts_warm_bits.write((uint8_t)threshold);

  return true;
}

/// @brief Gets BHOT threshold (upper limit for charging)
/// @return BHOT enum value
bq25798_bhot_t BqDriver::getBHot() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bhot_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 2);

  uint8_t reg_value = bhot_bits.read();

  return (bq25798_bhot_t)reg_value;
}

/// @brief Sets BHOT threshold (upper limit for charging)
/// @param threshold BHOT enum (55°C to 65°C, or disable)
/// @return true if successful
bool BqDriver::setBHot(bq25798_bhot_t threshold) {
  if (threshold > BQ25798_BHOT_DISABLE) {
    return false;
  }

  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bhot_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 2);

  bhot_bits.write((uint8_t)threshold);

  return true;
}

/// @brief Gets BCOLD threshold (lower limit for charging)
/// @return BCOLD enum value
bq25798_bcold_t BqDriver::getBCold() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bcold_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 1);

  uint8_t reg_value = bcold_bits.read();

  return (bq25798_bcold_t)reg_value;
}

/// @brief Sets BCOLD threshold (lower limit for charging)
/// @param threshold BCOLD enum (-10°C or -20°C)
/// @return true if successful
bool BqDriver::setBCold(bq25798_bcold_t threshold) {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bcold_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 1);

  bcold_bits.write((uint8_t)threshold);

  return true;
}

/// @brief Gets TS ignore status (disables all temperature monitoring)
/// @return true if temperature monitoring disabled
bool BqDriver::getTsIgnore() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_ignore_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 0);

  return (bool)ts_ignore_bits.read();
}

/// @brief Sets TS ignore status (disables all temperature monitoring)
/// @param ignore true to ignore temperature monitoring
/// @return true if successful
bool BqDriver::setTsIgnore(bool ignore) {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_ignore_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 0);

  ts_ignore_bits.write((uint8_t)ignore);

  return true;
}

/// @brief Starts ADC one-shot conversion for all enabled channels
/// @return true if successful
bool BqDriver::startADCOneShot() {
  Adafruit_BusIO_Register disable_reg_0 = Adafruit_BusIO_Register(ih_i2c_dev, 0x2F);
  Adafruit_BusIO_Register disable_reg_1 = Adafruit_BusIO_Register(ih_i2c_dev, 0x30);

  if (!disable_reg_0.write(0x00)) return false;
  if (!disable_reg_1.write(0x00)) return false;

  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  return adc_ctrl_reg.write(0xC0);
}

/// @brief Disables IBAT ADC discharge current measurement
/// @return true if successful
bool BqDriver::stopIbatADC() {
  Adafruit_BusIO_Register ctrl5_reg = Adafruit_BusIO_Register(ih_i2c_dev, 0x14);
  Adafruit_BusIO_RegisterBits en_ibat_bit = Adafruit_BusIO_RegisterBits(&ctrl5_reg, 1, 5);
  return en_ibat_bit.write(0);
}

/// @brief Enables IBAT ADC discharge current measurement
/// @return true if successful
bool BqDriver::startIbatADC() {
  Adafruit_BusIO_Register ctrl5_reg = Adafruit_BusIO_Register(ih_i2c_dev, 0x14);
  Adafruit_BusIO_RegisterBits en_ibat_bit = Adafruit_BusIO_RegisterBits(&ctrl5_reg, 1, 5);
  return en_ibat_bit.write(1);
}

// Implementierungen für ADC Control (0x2E)
bool BqDriver::getADCEnabled() {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_en_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 7);
  return (bool)adc_en_bits.read();
}

bool BqDriver::setADCEnabled(bool enabled) {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_en_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 7);
  return adc_en_bits.write((uint8_t)enabled);
}

bool BqDriver::getADCRate() {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_rate_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 6);
  return (bool)adc_rate_bits.read();
}

bool BqDriver::setADCRate(bool oneshot) {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_rate_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 6);
  return adc_rate_bits.write((uint8_t)oneshot);
}

bq25798_adc_sample_t BqDriver::getADCSample() {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_sample_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 2, 4);
  return (bq25798_adc_sample_t)adc_sample_bits.read();
}

bool BqDriver::setADCSample(bq25798_adc_sample_t sample) {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_sample_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 2, 4);
  return adc_sample_bits.write((uint8_t)sample);
}

bool BqDriver::getADCAvg() {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_avg_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 3);
  return (bool)adc_avg_bits.read();
}

bool BqDriver::setADCAvg(bool avg) {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_avg_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 3);
  return adc_avg_bits.write((uint8_t)avg);
}

bool BqDriver::getADCAvgInit() {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_avg_init_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 2);
  return (bool)adc_avg_init_bits.read();
}

bool BqDriver::setADCAvgInit(bool init) {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_avg_init_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 2);
  return adc_avg_init_bits.write((uint8_t)init);
}

// Implementierungen für ADC Function Disable 0 (0x2F)
bool BqDriver::getIBUSADCDisable() {
  Adafruit_BusIO_Register disable0_reg =
      Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_FUNCTION_DISABLE_0);
  Adafruit_BusIO_RegisterBits ibus_dis_bits = Adafruit_BusIO_RegisterBits(&disable0_reg, 1, 7);
  return (bool)ibus_dis_bits.read();
}

bool BqDriver::setIBUSADCDisable(bool disable) {
  Adafruit_BusIO_Register disable0_reg =
      Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_FUNCTION_DISABLE_0);
  Adafruit_BusIO_RegisterBits ibus_dis_bits = Adafruit_BusIO_RegisterBits(&disable0_reg, 1, 7);
  return ibus_dis_bits.write((uint8_t)disable);
}

// Ähnlich für die anderen Disable-Bits in 0x2F (Bit 6: IBAT, 5: VBUS, 4: VBAT, 3: VSYS, 2: TS, 1: TDIE)
// Kopiere den Pattern und passe Bit-Position an, z.B. für getIBATADCDisable(): Bits(1, 6)

// Für ADC Function Disable 1 (0x30): Bit 7: DP, 6: DM, 5: VAC2, 4: VAC1
// Ähnlich implementieren.

// Implementierungen für ADC Readings (Beispiel für getIBUS() - signed float in A)
int16_t BqDriver::getIBUS() {
  Adafruit_BusIO_Register ibus_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_IBUS_ADC, 2, MSBFIRST);
  uint16_t raw;
  if (!ibus_reg.read(&raw)) { // MSB first
    return 0;
  }
  int16_t val = (int16_t)raw; // 2's complement for signed
  return val;                 // in mA
}

int16_t BqDriver::getIBAT() {
  Adafruit_BusIO_Register ibat_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_IBAT_ADC, 2, MSBFIRST);
  uint16_t raw;
  if (!ibat_reg.read(&raw)) { // MSB first
    return 0;
  }
  int16_t val = (int16_t)raw; // 2's complement for signed
  return val;                 // in mA
}

// Ähnlich für getIBAT(): Gleiches wie oben.

uint16_t BqDriver::getVBUS() {
  Adafruit_BusIO_Register vbus_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_VBUS_ADC, 2, MSBFIRST);
  uint16_t val;
  if (!vbus_reg.read(&val)) {
    return 0.0f;
  }
  return val; // in mV
}

uint16_t BqDriver::getVBAT() {
  Adafruit_BusIO_Register vbat_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_VBAT_ADC, 2, MSBFIRST);
  uint16_t val;
  if (!vbat_reg.read(&val)) {
    return 0.0f;
  }
  return val; // in mV
}

/// @brief Read VBAT directly via I2C without requiring driver initialization
/// @param wire Pointer to TwoWire instance
/// @return Battery voltage in millivolts, or 0 if read fails
/// @note Static method for early boot use before BqDriver::begin() is called
uint16_t BqDriver::readVBATDirect(TwoWire* wire) {
  const uint8_t BQ_I2C_ADDR = 0x6B;
  // Use BQ25798_REG_VBAT_ADC from Adafruit library (via #include)
  
  wire->beginTransmission(BQ_I2C_ADDR);
  wire->write(BQ25798_REG_VBAT_ADC);  // 0x3B
  if (wire->endTransmission(false) != 0) {
    return 0;  // I2C communication failed
  }
  
  wire->requestFrom(BQ_I2C_ADDR, (uint8_t)2);
  if (wire->available() < 2) {
    return 0;  // Not enough data available
  }
  
  uint8_t msb = wire->read();
  uint8_t lsb = wire->read();
  
  return (msb << 8) | lsb;  // BQ25798 returns voltage directly in mV
}

uint16_t BqDriver::getVSYS() {
  Adafruit_BusIO_Register vsys_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_VSYS_ADC, 2, MSBFIRST);
  uint16_t val;
  if (!vsys_reg.read(&val)) {
    return 0.0f;
  }
  return val; // in mV
}

// Für getTS(): % of REGN
float BqDriver::getTS() {
  Adafruit_BusIO_Register ts_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_TS_ADC, 2, MSBFIRST);
  uint16_t val;
  if (!ts_reg.read(&val)) {
    return 0.0f;
  }
  return val * 0.09765625f; // 0.09765625 %/LSB (exact: 1/1024)
}

// Für getTDIE(): signed °C
float BqDriver::getTDIE() {
  Adafruit_BusIO_Register tdie_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_TDIE_ADC, 2, MSBFIRST);
  uint16_t raw;
  if (!tdie_reg.read(&raw)) {
    return 0.0f;
  }
  int16_t val = (int16_t)raw;
  return val * 0.5f; // 0.5 °C/LSB
}

// Non-static register access methods (use instance I2C config)
bool BqDriver::writeReg(uint8_t reg, uint8_t val) {
  if (!ih_i2c_dev) return false;
  
  uint8_t buffer[2] = {reg, val};
  return ih_i2c_dev->write(buffer, 2);
}

uint8_t BqDriver::readReg(uint8_t reg) {
  if (!ih_i2c_dev) return 0;
  
  uint8_t buffer[1] = {reg};
  if (!ih_i2c_dev->write_then_read(buffer, 1, buffer, 1)) {
    return 0;
  }
  return buffer[0];
}

// Kopiere für die anderen Readings und passe Skalierung an (z.B. getDP()/getDM(): *0.001f für V).