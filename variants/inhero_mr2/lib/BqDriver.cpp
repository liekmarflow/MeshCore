/*
 * Copyright (c) 2026 Inhero GmbH
 *
 * SPDX-License-Identifier: MIT
 *
 * BQ25798 Charger Driver Implementation
 */
#include "BqDriver.h"

#include <MeshCore.h>

BqDriver::BqDriver() {}

BqDriver::~BqDriver() {
  if (ih_i2c_dev) {
    delete ih_i2c_dev;
    ih_i2c_dev = nullptr;
  }
}

// Initializes BQ25798 charger and creates dedicated I2C device for NTC access
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

// Reads Power Good status from charger — true if input power is sufficient for charging
bool BqDriver::getChargerStatusPowerGood() {
  Adafruit_BusIO_Register chrg_stat_0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_0);
  Adafruit_BusIO_RegisterBits chrg_stat_0_bits = Adafruit_BusIO_RegisterBits(&chrg_stat_0_reg, 1, 3);

  uint8_t reg_value = chrg_stat_0_bits.read();

  return (bool)reg_value;
}

// Reads current charging state from charger
bq25798_charging_status BqDriver::getChargingStatus() {
  Adafruit_BusIO_Register chrg_stat_1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_1);

  // Read with explicit error check. RegisterBits::read() would return bits of -1
  // on a failed I2C read — CHG_STAT[7:5] of 0xFF decodes as 0x07 = DONE_CHARGING,
  // which the SOC logic treats as "battery full".
  uint8_t reg_value = 0;
  if (!chrg_stat_1_reg.read(&reg_value, 1)) {
    return BQ25798_CHARGER_STATE_UNKNOWN;
  }

  return (bq25798_charging_status)((reg_value >> 5) & 0x07);
}

// Reads solar and temperature telemetry via BQ25798 ADC one-shot
//
// BQ25798 ADC Operating Conditions (SLUSDV2B 9.3.10 / SLUSDV2C 7.3.10):
//   "The ADC is allowed to operate if either VBUS > 3.4V or VBAT > 2.9V is valid.
//    At battery only condition, if the TS_ADC channel is enabled, the ADC only
//    works when battery voltage is higher than 3.2V, otherwise, the ADC works
//    when the battery voltage is higher than 2.9V."
//
// This means:
//   VBUS > 3.4V              → ADC runs, all channels available
//   VBAT >= 3.2V (no VBUS)   → ADC runs, all channels including TS
//   VBAT 2.9-3.2V (no VBUS)  → ADC runs ONLY if TS channel is DISABLED
//   VBAT < 2.9V (no VBUS)    → ADC cannot run at all
//
// Strategy:
//   1. If VBAT < 3.2V: disable TS channel to lower threshold to 2.9V
//      → Solar data (VBUS/IBUS) still readable, temperature returns N/A
//   2. If VBAT < 2.9V and no VBUS: ADC times out, all values zero/N/A
//   3. Only channels used on MR2 are enabled (IBUS, VBUS, TDIE, optional TS).
//      Other channels are disabled to avoid unnecessary conversion time.
//
// ADC_EN auto-clear behavior:
//   In one-shot mode, ADC_EN resets to 0 only when ALL enabled channels
//   have completed conversion. However, a rejected or interrupted conversion
//   can also leave ADC_EN=0 without fresh data. Require ADC_DONE_FLAG as well.
//
// vbat_mv: battery voltage in mV from INA228 (0 = unknown, assume sufficient).
// Returns pointer to internal Telemetry struct (valid until next call).
const Telemetry* BqDriver::getTelemetryData(uint16_t vbat_mv) {
  return readTelemetry(vbat_mv, -1);
}

const Telemetry* BqDriver::readTelemetry(uint16_t vbat_mv, int8_t tsOverride) {
  telemetryData = { 0 };
  telemetryData.battery.temperature = -888.0f;
  adcDiagnostics = {};
  adcDiagnostics.result = "NOT-INIT";
  if (!ih_i2c_dev) return &telemetryData;

  Adafruit_BusIO_Register inputStatus(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_0);
  Adafruit_BusIO_Register chargerControl(ih_i2c_dev, BQ25798_REG_CHARGER_CONTROL_0);
  adcDiagnostics.result = "STATE-I2C";
  if (!inputStatus.read(&adcDiagnostics.inputBefore, 1) ||
      !chargerControl.read(&adcDiagnostics.chargerBefore, 1)) return &telemetryData;

  // The TS channel runs regardless of chemistry. A missing NTC then decodes
  // through the RT2-only pole to a bogus ≈-46°C that slips past the open-pin
  // check; the BME280 plausibility filter in
  // BoardConfigContainer::getTelemetryData() discards such readings.
  //
  // The VBAT >= 3.2V requirement applies to battery-only operation (datasheet
  // quote above). With an input source qualified the ADC runs off VBUS, so the
  // channel stays on — that is exactly the case worth measuring: a cold, nearly
  // empty cell being charged. Below the threshold and without an input the TS
  // channel would stall the whole conversion, costing the solar readings too,
  // so it is switched off there.
  bool ts_enabled = true;
  if (vbat_mv > 0 && vbat_mv < 3200 && !(adcDiagnostics.inputBefore & 0x08)) {
    ts_enabled = false;  // Disable TS → ADC threshold drops to 2.9V
  }
  if (tsOverride >= 0) ts_enabled = tsOverride != 0;

  bool success = this->startADCOneShot(ts_enabled);

  if (!success) {
    setADCEnabled(false);
    return &telemetryData;
  }

  // ADC_EN=0 alone does not prove that a conversion happened (e.g. low supply).
  // Require a fresh ADC_DONE_FLAG, cleared before this one-shot was started.
  // Channels: IBUS + VBUS + TDIE (+ TS if enabled) → ~72-96ms typical.
  const uint32_t ADC_TIMEOUT_MS = 250;
  uint32_t start = millis();
  bool conversion_done = false;
  bool doneSeen = false;
  Adafruit_BusIO_Register adc_flags(ih_i2c_dev, 0x24);
  Adafruit_BusIO_Register adc_control(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  while ((millis() - start) < ADC_TIMEOUT_MS) {
    uint8_t control = 0, flags = 0;
    if (!adc_control.read(&control, 1) || !adc_flags.read(&flags, 1)) {
      adcDiagnostics.result = "POLL-I2C";
      break;
    }
    adcDiagnostics.endControl = control;
    adcDiagnostics.flags |= flags;
    doneSeen = doneSeen || (flags & 0x20);
    if (!(control & 0x80) && doneSeen) {
      conversion_done = true;
      break;
    }
    delay(10);
  }
  adcDiagnostics.elapsedMs = millis() - start;
  Adafruit_BusIO_Register adc_status(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_3);
  adc_status.read(&adcDiagnostics.status, 1);
  inputStatus.read(&adcDiagnostics.inputAfter, 1);
  chargerControl.read(&adcDiagnostics.chargerAfter, 1);

  if (!conversion_done) {
    if (strcmp(adcDiagnostics.result, "WAIT") == 0) {
      adcDiagnostics.result = (adcDiagnostics.endControl & 0x80) ? "TIMEOUT" : "NO-DONE";
    }
    this->setADCEnabled(false);
  }

  if (conversion_done) {
    adcDiagnostics.result = "DATA-I2C";
    Adafruit_BusIO_Register vbus(ih_i2c_dev, BQ25798_REG_VBUS_ADC, 2, MSBFIRST);
    Adafruit_BusIO_Register ibus(ih_i2c_dev, BQ25798_REG_IBUS_ADC, 2, MSBFIRST);
    uint16_t voltage = 0, current = 0;
    if (vbus.read(&voltage) && ibus.read(&current)) {
      telemetryData.solar.voltage = voltage;
      telemetryData.solar.current = (int16_t)current;
      telemetryData.solar.valid = true;
      adcDiagnostics.result = "OK";
    }
    if (telemetryData.solar.current < 0) {
      telemetryData.solar.current = 0;
    }
    telemetryData.solar.power = ((int32_t)telemetryData.solar.voltage * telemetryData.solar.current) / 1000;

    if (ts_enabled) {
      telemetryData.battery.temperature = this->calculateBatteryTemp(getTS());
    } else {
      // TS channel off — battery-only below 3.2V. No reading.
      telemetryData.battery.temperature = -888.0f;
    }
  } else {
    // ADC didn't complete — VBAT < 2.9V and no VBUS, or I2C issue
    telemetryData.battery.temperature = -888.0f;
  }

  telemetryData.solar.mppt = getMPPTenable();

  return &telemetryData;
}

void BqDriver::getAdcDiagnostics(char* buffer, uint32_t bufferSize, uint16_t vbat_mv, int8_t tsOverride) {
  if (!buffer || bufferSize == 0) return;
  readTelemetry(vbat_mv, tsOverride);
  snprintf(buffer, bufferSize, "ADC:%s %lums C:%02X>%02X S:%02X F:%02X M:%02X/%02X VB:%u P:%02X>%02X H:%02X>%02X",
           adcDiagnostics.result, (unsigned long)adcDiagnostics.elapsedMs,
           adcDiagnostics.startControl, adcDiagnostics.endControl,
           adcDiagnostics.status, adcDiagnostics.flags,
           adcDiagnostics.disable0, adcDiagnostics.disable1, vbat_mv,
           adcDiagnostics.inputBefore, adcDiagnostics.inputAfter,
           adcDiagnostics.chargerBefore, adcDiagnostics.chargerAfter);
}

void BqDriver::captureAdcSequence(AdcSequenceTrace& trace, bool legacy, bool tsEnabled) {
  trace = {};
  trace.captured = true;
  trace.result = "NOT-INIT";
  if (!ih_i2c_dev) return;

  Adafruit_BusIO_Register control(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_Register status(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_3);
  Adafruit_BusIO_Register flags(ih_i2c_dev, 0x24);
  Adafruit_BusIO_Register input(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_0);
  Adafruit_BusIO_Register charger(ih_i2c_dev, BQ25798_REG_CHARGER_CONTROL_0);
  Adafruit_BusIO_Register voltage(ih_i2c_dev, BQ25798_REG_VBUS_ADC, 2, MSBFIRST);
  Adafruit_BusIO_Register current(ih_i2c_dev, BQ25798_REG_IBUS_ADC, 2, MSBFIRST);

  trace.result = "BEFORE-I2C";
  if (!control.read(&trace.before.control, 1)) return;
  // Do not stop an existing conversion to prepare OLD: that would defeat the
  // comparison. All board I2C work runs synchronously in the main loop.
  if (trace.before.control & 0x80) {
    trace.result = "BUSY";
    return;
  }
  if (!status.read(&trace.before.status, 1) ||
      !input.read(&trace.inputBefore, 1) || !charger.read(&trace.chargerBefore, 1) ||
      !voltage.read(&trace.voltageBefore) || !current.read(&trace.currentBefore) ||
      !flags.read(&trace.before.flags, 1)) return;
  // The baseline FLAG read above clears retained events in BOTH cases. OLD
  // keeps the historical write sequence, but never treats retained data as fresh.
  trace.inputAfter = trace.inputBefore;
  trace.chargerAfter = trace.chargerBefore;
  trace.stopped = false;
  bool started;
  if (legacy) {
    Adafruit_BusIO_Register disable0(ih_i2c_dev, 0x2F);
    Adafruit_BusIO_Register disable1(ih_i2c_dev, 0x30);
    uint8_t discardedFlags;
    // Historical ADC start: channel masks, then C0. No preceding ADC disable and
    // no mask readback. Both variants now release HIZ immediately before C0.
    started = false;
    trace.result = "START-I2C";
    if (flags.read(&discardedFlags, 1) && disable0.write(tsEnabled ? 0x58 : 0x5C) &&
        disable1.write(0xF0)) {
      if (prepareADCInput()) started = control.write(0xC0);
      else trace.result = adcDiagnostics.result;
    }
  } else {
    started = startADCOneShot(tsEnabled);
    trace.result = adcDiagnostics.result;
  }

  if (started) {
    trace.result = "POLL-I2C";
    const uint16_t sampleTimes[] = {0, 10, 25, 50, 100, 150, 250};
    const uint32_t start = millis();
    bool completed = false;
    bool doneSeen = false;
    bool pollsOk = true;
    for (uint8_t i = 0; i < 7; ++i) {
      uint32_t elapsed = millis() - start;
      if (elapsed < sampleTimes[i]) delay(sampleTimes[i] - elapsed);
      AdcTracePoint& point = trace.points[trace.count];
      point.ms = (uint16_t)(millis() - start);
      if (!control.read(&point.control, 1) || !status.read(&point.status, 1) ||
          !flags.read(&point.flags, 1) || !input.read(&trace.inputAfter, 1) ||
          !charger.read(&trace.chargerAfter, 1)) {
        pollsOk = false;
        break;
      }
      ++trace.count;
      trace.stateChanged |= trace.inputAfter != trace.inputBefore ||
                            trace.chargerAfter != trace.chargerBefore;
      doneSeen |= (point.flags & 0x20) != 0;
      completed |= !(point.control & 0x80) && doneSeen;
      // Keep sampling after completion: capture the distinction between the
      // persistent DONE_STAT and the read-to-clear DONE_FLAG.
    }
    if (pollsOk) {
      trace.result = completed ? "OK" :
          ((trace.points[trace.count - 1].control & 0x80) ? "TIMEOUT" : "NO-DONE");
    }
    // Always show raw register contents, even after NO-DONE. These diagnostic
    // values are NOT published through Telemetry or used for PG recovery.
    bool voltageOk = voltage.read(&trace.voltageAfter);
    bool currentOk = current.read(&trace.currentAfter);
    if ((!voltageOk || !currentOk) && pollsOk) trace.result = "DATA-I2C";
  }

  // No extra disable write if the ADC has already stopped. This preserves the
  // end state of OLD for NEW and vice versa. Bound a hung/failed start and verify
  // cleanup before allowing a second test. Do not restore HIZ after conversion.
  uint8_t endControl = 0;
  if (control.read(&endControl, 1) && !(endControl & 0x80)) {
    trace.stopped = true;
  } else if (setADCEnabled(false) && control.read(&endControl, 1) && !(endControl & 0x80)) {
    trace.stopped = true;
  } else {
    trace.result = "STOP-I2C";
  }
}

void BqDriver::compareAdcSequences(char* buffer, uint32_t bufferSize, uint16_t vbat_mv, bool reverse) {
  if (!buffer || bufferSize == 0) return;
  adcSequenceTraces[0] = {};
  adcSequenceTraces[1] = {};
  uint8_t initialInput = 0;
  if (!ih_i2c_dev) {
    snprintf(buffer, bufferSize, "ADC A/B:NOT-INIT");
    return;
  }
  Adafruit_BusIO_Register input(ih_i2c_dev, BQ25798_REG_CHARGER_STATUS_0);
  if (!input.read(&initialInput, 1)) {
    snprintf(buffer, bufferSize, "ADC A/B:STATE-I2C");
    return;
  }
  // Freeze the channel choice for BOTH runs, even if PG changes meanwhile.
  const bool tsEnabled = !(vbat_mv > 0 && vbat_mv < 3200 && !(initialInput & 0x08));
  const uint8_t first = reverse ? 1 : 0;
  const uint8_t second = 1 - first;
  captureAdcSequence(adcSequenceTraces[first], first == 0, tsEnabled);
  const AdcSequenceTrace& a = adcSequenceTraces[first];
  if (a.stopped && (strcmp(a.result, "OK") == 0 || strcmp(a.result, "NO-DONE") == 0 ||
                    strcmp(a.result, "TIMEOUT") == 0)) {
    captureAdcSequence(adcSequenceTraces[second], second == 0, tsEnabled);
  }
  const AdcSequenceTrace& b = adcSequenceTraces[second];
  bool changed = a.stateChanged || b.stateChanged || a.inputBefore != initialInput ||
                 a.inputAfter != b.inputBefore || a.chargerAfter != b.chargerBefore;
  const bool bothRan = a.count == 7 && b.count == 7;
  snprintf(buffer, bufferSize,
           "ADC A/B:%s VBAT:%u M:%02X/F0\nOLD:%s NEW:%s\nP:%02X>%02X H:%02X>%02X %s",
           reverse ? "NEW>OLD" : "OLD>NEW", vbat_mv, tsEnabled ? 0x58 : 0x5C,
           adcSequenceTraces[0].result, adcSequenceTraces[1].result,
           a.inputBefore, b.inputAfter, a.chargerBefore, b.chargerAfter,
           !bothRan ? "INCOMPLETE" : (changed ? "CHANGED" : "STABLE"));
}

void BqDriver::getAdcSequenceTrace(char* buffer, uint32_t bufferSize, bool legacy) const {
  if (!buffer || bufferSize == 0) return;
  const AdcSequenceTrace& trace = adcSequenceTraces[legacy ? 0 : 1];
  if (!trace.captured) {
    snprintf(buffer, bufferSize, "No capture: get board.adc compare");
    return;
  }
  // U/I are raw hexadecimal register words (mV / signed mA). C/S/F are raw
  // ADC_CONTROL / STATUS_3 / FLAG_2. Seven samples fit in the 160-byte CLI reply.
  snprintf(buffer, bufferSize, "%s U:%04X>%04X I:%04X>%04X\npre:%02X/%02X/%02X\n",
           legacy ? "OLD" : "NEW", trace.voltageBefore, trace.voltageAfter,
           trace.currentBefore, trace.currentAfter,
           trace.before.control, trace.before.status, trace.before.flags);
  for (uint8_t i = 0; i < trace.count; ++i) {
    const AdcTracePoint& point = trace.points[i];
    size_t used = strlen(buffer);
    if (used >= bufferSize - 1) break;
    snprintf(buffer + used, bufferSize - used, "%s%u:%02X/%02X/%02X",
             i ? " " : "", point.ms, point.control, point.status, point.flags);
  }
}

// Calculates battery temperature in °C using Steinhart-Hart equation.
// Uses coefficients derived from Murata NCP15XH103F03RC datasheet R-T table.
// Max error vs. datasheet: ±0.36°C over -40..+125°C range.
//
// Per BQ25798 datasheet Figure 9-12: REGN → RT1 → TS → (RT2||NTC) → GND
// ts_pct: voltage at TS pin in percentage of REGN (e.g., 70.5 for 70.5%).
//         Special input values: -1.0 = I2C error, -2.0 = ADC not ready/invalid.
// Returns temperature in °C, or error codes:
//   -999.0 = I2C communication error
//   -888.0 = ADC not ready (read 0 or 0xFFFF)
//    -99.0 = NTC open/disconnected (k > 0.99)
//     99.0 = NTC short circuit (k < 0.01)
float BqDriver::calculateBatteryTemp(float ts_pct) {
  // Check for I2C read error
  if (ts_pct == -1.0f) return -999.0f; // I2C error
  if (ts_pct == -2.0f) return -888.0f; // ADC not ready or invalid value
  
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

  // Apply Steinhart-Hart equation: 1/T = A + B·ln(R) + C·(ln(R))³
  float ln_r = logf(r_ntc);
  float inv_T = SH_A + SH_B * ln_r + SH_C * ln_r * ln_r * ln_r;

  // Convert Kelvin to Celsius
  return (1.0f / inv_T) - 273.15f;
}

// Getter/Setter for NTC Control 0 (0x17)
// Gets JEITA voltage setting for warm/cool regions
bq25798_jeita_vset_t BqDriver::getJeitaVSet() {
  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_vset_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 3, 5);

  uint8_t reg_value = jeita_vset_bits.read();

  return (bq25798_jeita_vset_t)reg_value;
}

// Sets JEITA voltage setting for warm/cool temperature regions
bool BqDriver::setJeitaVSet(bq25798_jeita_vset_t setting) {
  if (setting > BQ25798_JEITA_VSET_UNCHANGED) {
    return false;
  }

  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_vset_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 3, 5);

  jeita_vset_bits.write((uint8_t)setting);

  return true;
}

// Gets JEITA current setting for hot region
bq25798_jeita_iseth_t BqDriver::getJeitaISetH() {
  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_iseth_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 3);

  uint8_t reg_value = jeita_iseth_bits.read();

  return (bq25798_jeita_iseth_t)reg_value;
}

// Sets JEITA current setting for hot temperature region
bool BqDriver::setJeitaISetH(bq25798_jeita_iseth_t setting) {
  if (setting > BQ25798_JEITA_ISETH_UNCHANGED) {
    return false;
  }

  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_iseth_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 3);

  jeita_iseth_bits.write((uint8_t)setting);

  return true;
}

// Gets JEITA current setting for cold region
bq25798_jeita_isetc_t BqDriver::getJeitaISetC() {
  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_isetc_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 1);

  uint8_t reg_value = jeita_isetc_bits.read();

  return (bq25798_jeita_isetc_t)reg_value;
}

// Sets JEITA current setting for cold temperature region
bool BqDriver::setJeitaISetC(bq25798_jeita_isetc_t setting) {
  if (setting > BQ25798_JEITA_ISETC_UNCHANGED) {
    return false;
  }

  Adafruit_BusIO_Register ntc0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_0);
  Adafruit_BusIO_RegisterBits jeita_isetc_bits = Adafruit_BusIO_RegisterBits(&ntc0_reg, 2, 1);

  jeita_isetc_bits.write((uint8_t)setting);

  return true;
}

// Gets TS Cool threshold (lower boundary of COOL region)
bq25798_ts_cool_t BqDriver::getTsCool() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_cool_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 6);

  uint8_t reg_value = ts_cool_bits.read();

  return (bq25798_ts_cool_t)reg_value;
}

// Sets TS Cool threshold (lower boundary of COOL region)
bool BqDriver::setTsCool(bq25798_ts_cool_t threshold) {
  if (threshold > BQ25798_TS_COOL_20C) {
    return false;
  }

  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_cool_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 6);

  ts_cool_bits.write((uint8_t)threshold);

  return true;
}

// Gets TS Warm threshold (upper boundary of WARM region)
bq25798_ts_warm_t BqDriver::getTsWarm() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_warm_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 4);

  uint8_t reg_value = ts_warm_bits.read();

  return (bq25798_ts_warm_t)reg_value;
}

// Sets TS Warm threshold (upper boundary of WARM region)
bool BqDriver::setTsWarm(bq25798_ts_warm_t threshold) {
  if (threshold > BQ25798_TS_WARM_55C) {
    return false;
  }

  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_warm_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 4);

  ts_warm_bits.write((uint8_t)threshold);

  return true;
}

// Gets BHOT threshold (upper limit for charging)
bq25798_bhot_t BqDriver::getBHot() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bhot_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 2);

  uint8_t reg_value = bhot_bits.read();

  return (bq25798_bhot_t)reg_value;
}

// Sets BHOT threshold (upper limit for charging)
bool BqDriver::setBHot(bq25798_bhot_t threshold) {
  if (threshold > BQ25798_BHOT_DISABLE) {
    return false;
  }

  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bhot_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 2, 2);

  bhot_bits.write((uint8_t)threshold);

  return true;
}

// Gets BCOLD threshold (lower limit for charging)
bq25798_bcold_t BqDriver::getBCold() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bcold_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 1);

  uint8_t reg_value = bcold_bits.read();

  return (bq25798_bcold_t)reg_value;
}

// Sets BCOLD threshold (lower limit for charging)
bool BqDriver::setBCold(bq25798_bcold_t threshold) {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits bcold_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 1);

  bcold_bits.write((uint8_t)threshold);

  return true;
}

// Gets TS ignore status (disables all temperature monitoring)
bool BqDriver::getTsIgnore() {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_ignore_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 0);

  return (bool)ts_ignore_bits.read();
}

// Sets TS ignore status (disables all temperature monitoring)
bool BqDriver::setTsIgnore(bool ignore) {
  Adafruit_BusIO_Register ntc1_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_NTC_CONTROL_1);
  Adafruit_BusIO_RegisterBits ts_ignore_bits = Adafruit_BusIO_RegisterBits(&ntc1_reg, 1, 0);

  ts_ignore_bits.write((uint8_t)ignore);

  return true;
}

// Starts ADC one-shot conversion for selected channels
//
// MR2 ADC Channel Map:
//   Reg 0x2F (ADC_FUNCTION_DISABLE_0): bit=1 means DISABLED
//     Bit 7: IBUS  → ENABLED  (solar current)
//     Bit 6: IBAT  → disabled (INA228 measures battery current)
//     Bit 5: VBUS  → ENABLED  (solar voltage)
//     Bit 4: VBAT  → disabled (INA228 measures battery voltage)
//     Bit 3: VSYS  → disabled (not used)
//     Bit 2: TS    → ENABLED or disabled depending on VBAT level
//     Bit 1: TDIE  → ENABLED (charger die temperature)
//     Bit 0: reserved
//
//   Reg 0x30 (ADC_FUNCTION_DISABLE_1): all disabled on MR2
//     Bit 7: D+   → disabled (AutoDPinsDetection=false, pin not connected)
//     Bit 6: D-   → disabled (pin not connected)
//     Bit 5: VAC2 → disabled (not routed on PCB)
//     Bit 4: VAC1 → disabled (not routed on PCB)
//
// Unused channels stay disabled to reduce conversion time. Supply availability
// still limits ADC operation; channel masks cannot compensate for an invalid supply.
//
// Release the input before a one-shot, without changing charge enable.
bool BqDriver::prepareADCInput() {
  adcDiagnostics.result = "HIZ-I2C";
  if (!ih_i2c_dev) return false;
  Adafruit_BusIO_Register charger(ih_i2c_dev, BQ25798_REG_CHARGER_CONTROL_0);
  uint8_t control;
  if (!charger.read(&control, 1)) return false;
  if (control & 0x04) {
    // Preserve EN_CHG and all other settings; CE is controlled by board config.
    if (!charger.write(control & ~0x04)) return false;
  }
  return true;
}

// ts_enabled: true = enable TS (requires VBAT > 3.2V in battery-only operation).
// Success confirms setup/start register accesses, not conversion completion.
bool BqDriver::startADCOneShot(bool ts_enabled) {
  adcDiagnostics.result = "SETUP-I2C";
  // Stop any previous conversion, then discard its read-to-clear done flag.
  if (!setADCEnabled(false)) return false;
  Adafruit_BusIO_Register adc_flags(ih_i2c_dev, 0x24);
  uint8_t flags;
  if (!adc_flags.read(&flags, 1)) return false;
  Adafruit_BusIO_Register disable_reg_0 = Adafruit_BusIO_Register(ih_i2c_dev, 0x2F);
  Adafruit_BusIO_Register disable_reg_1 = Adafruit_BusIO_Register(ih_i2c_dev, 0x30);

  // Reg 0x2F bit map: IBUS(7) IBAT(6) VBUS(5) VBAT(4) VSYS(3) TS(2) TDIE(1) reserved(0)
  // 1 = disabled, 0 = enabled
  uint8_t disable0 = 0x58;  // Enable IBUS(7), VBUS(5), TS(2), TDIE(1) — disable rest
  if (!ts_enabled) {
    disable0 |= 0x04;       // Also disable TS(2) → 0x5C
  }
  if (!disable_reg_0.write(disable0)) { return false; }

  // Reg 0x30: Disable all — D+(7), D-(6), VAC2(5), VAC1(4) not connected on MR2
  if (!disable_reg_1.write(0xF0)) { return false; }

  if (!disable_reg_0.read(&adcDiagnostics.disable0, 1) ||
      !disable_reg_1.read(&adcDiagnostics.disable1, 1)) return false;
  if (adcDiagnostics.disable0 != disable0 || adcDiagnostics.disable1 != 0xF0) {
    adcDiagnostics.result = "MASK-MISMATCH";
    return false;
  }

  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  // Release HIZ only after setup, immediately before ADC_EN. Waiting here can
  // let source qualification reassert HIZ before our conversion even starts.
  if (!prepareADCInput()) return false;
  adcDiagnostics.result = "START-I2C";
  if (!adc_ctrl_reg.write(0xC0) || !adc_ctrl_reg.read(&adcDiagnostics.startControl, 1)) return false;
  adcDiagnostics.result = "WAIT";
  return true;
}

// ADC Control register (0x2E) implementations
bool BqDriver::getADCEnabled() {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  Adafruit_BusIO_RegisterBits adc_en_bits = Adafruit_BusIO_RegisterBits(&adc_ctrl_reg, 1, 7);
  bool result = (bool)adc_en_bits.read();
  return result;
}

bool BqDriver::setADCEnabled(bool enabled) {
  Adafruit_BusIO_Register adc_ctrl_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_ADC_CONTROL);
  uint8_t control;
  if (!adc_ctrl_reg.read(&control, 1)) return false;
  return adc_ctrl_reg.write(enabled ? (control | 0x80) : (control & ~0x80));
}

// ADC Reading implementations
int16_t BqDriver::getIBUS() {
  Adafruit_BusIO_Register ibus_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_IBUS_ADC, 2, MSBFIRST);
  uint16_t raw;
  if (!ibus_reg.read(&raw)) { // MSB first
    return 0;
  }
  int16_t val = (int16_t)raw; // 2's complement for signed
  return val;                 // in mA
}

uint16_t BqDriver::getVBUS() {
  Adafruit_BusIO_Register vbus_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_VBUS_ADC, 2, MSBFIRST);
  uint16_t val;
  if (!vbus_reg.read(&val)) {
    return 0;
  }
  return val; // in mV
}

float BqDriver::getDieTemperature_C() {
  Adafruit_BusIO_Register tdie_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_TDIE_ADC, 2, MSBFIRST);
  uint16_t raw;
  if (!tdie_reg.read(&raw)) {
    return -999.0f;
  }
  return (int16_t)raw * 0.5f; // 2's complement, 0.5°C/LSB
}

float BqDriver::getTS() {
  Adafruit_BusIO_Register ts_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_TS_ADC, 2, MSBFIRST);
  uint16_t val;
  
  // Try up to 3 times with small delays if we get invalid values
  for (int retry = 0; retry < 3; retry++) {
    if (!ts_reg.read(&val)) {
      delay(20);
      continue; // I2C read error, retry
    }
    // Check for invalid/uninitialized ADC value (0 or 0xFFFF)
    if (val == 0 || val == 0xFFFF) {
      if (retry < 2) {
        delay(50); // Wait a bit longer for ADC to settle
        continue;
      }
      return -2.0f; // ADC not ready / invalid value after retries
    }
    // Valid value
    return val * 0.09765625f; // 0.09765625 %/LSB (exact: 1/1024)
  }
  
  return -1.0f; // I2C read error after all retries
}

bool BqDriver::setVOCpercent(bq25798_voc_pct_t pct) {
  uint8_t reg15 = readReg(0x15);
  reg15 = (reg15 & 0x1F) | ((uint8_t)pct << 5);  // Bits [7:5] = VOC_PCT
  return writeReg(0x15, reg15);
}

bq25798_voc_pct_t BqDriver::getVOCpercent() {
  uint8_t reg15 = readReg(0x15);
  return (bq25798_voc_pct_t)((reg15 >> 5) & 0x07);
}


// Gets EN_AUTO_IBATDIS state (auto battery discharge during VBAT_OVP; POR default = enabled)
bool BqDriver::getAutoIBATDIS() {
  Adafruit_BusIO_Register ctrl0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_CONTROL_0);
  Adafruit_BusIO_RegisterBits auto_ibatdis_bit = Adafruit_BusIO_RegisterBits(&ctrl0_reg, 1, 7);
  return (bool)auto_ibatdis_bit.read();
}

// Sets EN_AUTO_IBATDIS (auto battery discharge during VBAT_OVP).
// enable: true = BQ sinks 30mA from BAT during OVP, false = no active discharge.
bool BqDriver::setAutoIBATDIS(bool enable) {
  Adafruit_BusIO_Register ctrl0_reg = Adafruit_BusIO_Register(ih_i2c_dev, BQ25798_REG_CHARGER_CONTROL_0);
  Adafruit_BusIO_RegisterBits auto_ibatdis_bit = Adafruit_BusIO_RegisterBits(&ctrl0_reg, 1, 7);
  return auto_ibatdis_bit.write(enable ? 1 : 0);
}

// Non-static register access methods (use instance I2C config)
bool BqDriver::writeReg(uint8_t reg, uint8_t val) {
  if (!ih_i2c_dev) return false;
  
  uint8_t buffer[2] = {reg, val};
  bool ok = ih_i2c_dev->write(buffer, 2);
  return ok;
}

uint8_t BqDriver::readReg(uint8_t reg) {
  uint8_t value = 0;
  return readReg(reg, value) ? value : 0;
}

bool BqDriver::readReg(uint8_t reg, uint8_t& val) {
  if (!ih_i2c_dev) return false;
  return ih_i2c_dev->write_then_read(&reg, 1, &val, 1);
}

// Static, raw-Wire helpers — safe pre-begin().
void BqDriver::maskAllInterrupts(TwoWire& wire, uint8_t addr) {
  static const uint8_t mask_regs[] = {0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
  for (uint8_t r : mask_regs) {
    wire.beginTransmission(addr);
    wire.write(r);
    wire.write(0xFF);
    wire.endTransmission();
  }
}

void BqDriver::clearInterruptFlags(TwoWire& wire, uint8_t addr) {
  static const uint8_t flag_regs[] = {0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
  for (uint8_t r : flag_regs) {
    wire.beginTransmission(addr);
    wire.write(r);
    wire.endTransmission(false);
    wire.requestFrom(addr, (uint8_t)1);
    while (wire.available()) wire.read();
  }
}

void BqDriver::disableAdc(TwoWire& wire, uint8_t addr) {
  wire.beginTransmission(addr);
  wire.write(0x2E);  // ADC_CONTROL
  wire.write(0x00);
  wire.endTransmission();
}
