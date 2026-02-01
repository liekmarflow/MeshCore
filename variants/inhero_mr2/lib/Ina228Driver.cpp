/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * INA228 Power Monitor Driver Implementation
 */

#include "Ina228Driver.h"

Ina228Driver::Ina228Driver(uint8_t i2c_addr) : _i2c_addr(i2c_addr), _shunt_mohm(10.0f), _current_lsb(0.0f) {}

bool Ina228Driver::begin(float shunt_resistor_mohm) {
  _shunt_mohm = shunt_resistor_mohm;

  // Check if device is present
  if (!isConnected()) {
    return false;
  }

  // Reset device
  reset();
  delay(10);

  // Configure ADC: Continuous mode, all channels, 16 samples averaging
  uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) |  // Continuous all
                        (INA228_ADC_AVG_16 << 0);             // 16 samples average
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);

  // Calculate current LSB: Max expected current / 2^19 (20-bit ADC)
  // Max 1A through 20mΩ shunt, LSB = 1A / 524288 ≈ 1.91 µA
  _current_lsb = 1.0f / 524288.0f;  // in Amperes

  // Calculate shunt calibration value
  // SHUNT_CAL = 13107.2 × 10^6 × CURRENT_LSB × R_SHUNT
  // R_SHUNT in Ohms, CURRENT_LSB in A
  float shunt_ohm = _shunt_mohm / 1000.0f;
  uint16_t shunt_cal = (uint16_t)(13107.2e6 * _current_lsb * shunt_ohm);
  writeRegister16(INA228_REG_SHUNT_CAL, shunt_cal);

  // Configure INA228: ADC range ±40.96mV for better resolution with 20mΩ shunt
  // At 1A: V_shunt = 1A × 0.02Ω = 20mV (fits in ±40.96mV range)
  uint16_t config = INA228_CONFIG_ADCRANGE;  // Use ±40.96mV range
  writeRegister16(INA228_REG_CONFIG, config);

  return true;
}

bool Ina228Driver::isConnected() {
  // Read Manufacturer ID (should be 0x5449 = "TI")
  uint16_t mfg_id = readRegister16(INA228_REG_MANUFACTURER);
  if (mfg_id != 0x5449) {
    return false;
  }

  // Read Device ID (should be 0x228 in lower 12 bits)
  uint16_t dev_id = readRegister16(INA228_REG_DEVICE_ID);
  if ((dev_id & 0x0FFF) != 0x228) {
    return false;
  }

  return true;
}

void Ina228Driver::reset() {
  writeRegister16(INA228_REG_CONFIG, INA228_CONFIG_RST);
}

uint16_t Ina228Driver::readVoltage_mV() {
  int32_t vbus_raw = readRegister24(INA228_REG_VBUS);
  // VBUS LSB = 195.3125 µV
  float vbus_v = vbus_raw * 195.3125e-6;
  return (uint16_t)(vbus_v * 1000.0f);  // Convert to mV
}

int16_t Ina228Driver::readCurrent_mA() {
  int32_t current_raw = readRegister24(INA228_REG_CURRENT);
  // Current = raw × CURRENT_LSB
  float current_a = current_raw * _current_lsb;
  return (int16_t)(current_a * 1000.0f);  // Convert to mA
}

void Ina228Driver::shutdown() {
  // Set operating mode to Shutdown (MODE = 0x0)
  // This disables all conversions and Coulomb Counter
  uint16_t adc_config = 0x0000;  // MODE = 0x0 (Shutdown)
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}

void Ina228Driver::wakeup() {
  // Re-enable continuous measurement mode
  uint16_t adc_config = (INA228_ADC_MODE_CONT_ALL << 12) |  // Continuous all
                        (INA228_ADC_AVG_16 << 0);             // 16 samples average
  writeRegister16(INA228_REG_ADC_CONFIG, adc_config);
}

uint16_t Ina228Driver::readVBATDirect(TwoWire* wire, uint8_t i2c_addr) {
  // === One-Shot ADC Trigger ===
  // Configure ADC for single-shot bus voltage measurement
  // MODE = 0x1 (Single-shot bus voltage only)
  uint16_t adc_config = (0x1 << 12);  // MODE = 0x1, no averaging for speed
  
  wire->beginTransmission(i2c_addr);
  wire->write(INA228_REG_ADC_CONFIG);
  wire->write((adc_config >> 8) & 0xFF);
  wire->write(adc_config & 0xFF);
  if (wire->endTransmission() != 0) {
    return 0;  // I2C communication failed
  }
  
  // Wait for conversion to complete (~200µs typical)
  delay(1);
  
  // Read VBUS register (24-bit)
  wire->beginTransmission(i2c_addr);
  wire->write(INA228_REG_VBUS);
  if (wire->endTransmission(false) != 0) {
    return 0;
  }
  
  wire->requestFrom(i2c_addr, (uint8_t)3);
  if (wire->available() < 3) {
    return 0;
  }
  
  int32_t vbus_raw = wire->read() << 16;  // MSB
  vbus_raw |= wire->read() << 8;          // Mid
  vbus_raw |= wire->read();               // LSB
  
  // Sign-extend 24-bit to 32-bit
  if (vbus_raw & 0x800000) {
    vbus_raw |= 0xFF000000;
  }
  
  // VBUS LSB = 195.3125 µV (24-bit ADC)
  float vbus_v = vbus_raw * 195.3125e-6;
  return (uint16_t)(vbus_v * 1000.0f);  // Convert to mV
}

int32_t Ina228Driver::readPower_mW() {
  int32_t power_raw = readRegister24(INA228_REG_POWER);
  // Power LSB = 3.2 × CURRENT_LSB
  float power_w = power_raw * (3.2f * _current_lsb);
  return (int32_t)(power_w * 1000.0f);  // Convert to mW
}

int32_t Ina228Driver::readEnergy_mWh() {
  int64_t energy_raw = readRegister40(INA228_REG_ENERGY);
  // Energy LSB = 16 × 3.2 × CURRENT_LSB (in J)
  // Convert to Wh: / 3600
  float energy_j = energy_raw * (16.0f * 3.2f * _current_lsb);
  float energy_wh = energy_j / 3600.0f;
  return (int32_t)(energy_wh * 1000.0f);  // Convert to mWh
}

float Ina228Driver::readCharge_mAh() {
  int64_t charge_raw = readRegister40(INA228_REG_CHARGE);
  // Charge LSB = CURRENT_LSB (in C = A·s)
  // Convert to Ah: / 3600
  float charge_c = charge_raw * _current_lsb;
  float charge_ah = charge_c / 3600.0f;
  return charge_ah * 1000.0f;  // Convert to mAh
}

float Ina228Driver::readDieTemperature_C() {
  int32_t temp_raw = readRegister24(INA228_REG_DIETEMP);
  // Temperature LSB = 7.8125 m°C
  float temp_c = temp_raw * 7.8125e-3;
  return temp_c;
}

bool Ina228Driver::readAll(Ina228BatteryData* data) {
  if (!isConnected()) {
    return false;
  }

  data->voltage_mv = readVoltage_mV();
  data->current_ma = readCurrent_mA();
  data->power_mw = readPower_mW();
  data->energy_mwh = readEnergy_mWh();
  data->charge_mah = readCharge_mAh();
  data->die_temp_c = readDieTemperature_C();

  return true;
}

void Ina228Driver::resetCoulombCounter() {
  // Reading energy/charge registers clears them
  readRegister40(INA228_REG_ENERGY);
  readRegister40(INA228_REG_CHARGE);
}

bool Ina228Driver::setUnderVoltageAlert(uint16_t voltage_mv) {
  // BUVL = voltage / 195.3125 µV
  float voltage_v = voltage_mv / 1000.0f;
  uint16_t buvl_value = (uint16_t)(voltage_v / 195.3125e-6);
  return writeRegister16(INA228_REG_BUVL, buvl_value);
}

bool Ina228Driver::setOverVoltageAlert(uint16_t voltage_mv) {
  // BOVL = voltage / 195.3125 µV
  float voltage_v = voltage_mv / 1000.0f;
  uint16_t bovl_value = (uint16_t)(voltage_v / 195.3125e-6);
  return writeRegister16(INA228_REG_BOVL, bovl_value);
}

void Ina228Driver::enableAlert(bool enable_uvlo, bool enable_ovlo, bool active_high) {
  uint16_t diag_alrt = 0;

  if (enable_uvlo) {
    diag_alrt |= INA228_DIAG_ALRT_BUSUL;  // Enable bus under-voltage alert
  }

  if (enable_ovlo) {
    diag_alrt |= INA228_DIAG_ALRT_BUSOL;  // Enable bus over-voltage alert
  }

  if (active_high) {
    diag_alrt |= INA228_DIAG_ALRT_APOL;   // Active-high polarity
  }

  // Enable alert latch (must be cleared manually)
  diag_alrt |= INA228_DIAG_ALRT_ALATCH;

  writeRegister16(INA228_REG_DIAG_ALRT, diag_alrt);
}

bool Ina228Driver::isAlertActive() {
  uint16_t diag_flags = getDiagnosticFlags();
  return (diag_flags & (INA228_DIAG_ALRT_BUSUL | INA228_DIAG_ALRT_BUSOL)) != 0;
}

void Ina228Driver::clearAlert() {
  // Read diagnostic register to clear latched alerts
  getDiagnosticFlags();
}

uint16_t Ina228Driver::getDiagnosticFlags() {
  return readRegister16(INA228_REG_DIAG_ALRT);
}

// ===== Private Methods =====

bool Ina228Driver::writeRegister16(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);  // MSB
  Wire.write(value & 0xFF);         // LSB
  return (Wire.endTransmission() == 0);
}

uint16_t Ina228Driver::readRegister16(uint8_t reg) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg);
  Wire.endTransmission(false);  // Repeated start

  Wire.requestFrom(_i2c_addr, (uint8_t)2);
  if (Wire.available() < 2) {
    return 0;
  }

  uint16_t value = Wire.read() << 8;  // MSB
  value |= Wire.read();               // LSB
  return value;
}

int32_t Ina228Driver::readRegister24(uint8_t reg) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(_i2c_addr, (uint8_t)3);
  if (Wire.available() < 3) {
    return 0;
  }

  int32_t value = Wire.read() << 16;  // MSB
  value |= Wire.read() << 8;          // Mid
  value |= Wire.read();               // LSB

  // Sign-extend 24-bit to 32-bit
  if (value & 0x800000) {
    value |= 0xFF000000;
  }

  return value;
}

int64_t Ina228Driver::readRegister40(uint8_t reg) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(_i2c_addr, (uint8_t)5);
  if (Wire.available() < 5) {
    return 0;
  }

  int64_t value = (int64_t)Wire.read() << 32;  // MSB
  value |= (int64_t)Wire.read() << 24;
  value |= (int64_t)Wire.read() << 16;
  value |= (int64_t)Wire.read() << 8;
  value |= (int64_t)Wire.read();               // LSB

  // Sign-extend 40-bit to 64-bit
  if (value & 0x8000000000LL) {
    value |= 0xFFFFFF0000000000LL;
  }

  return value;
}
