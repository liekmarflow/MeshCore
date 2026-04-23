/*
 * Copyright (c) 2026 Inhero GmbH
 * SPDX-License-Identifier: MIT
 */
#include "BqLowPowerSetup.h"

#include <Arduino.h>
#include <Wire.h>

#include "../InheroMr2Board.h"

namespace inhero {

static constexpr uint8_t INA228_ADDR = 0x40;

void prepareIcsForSystemOff() {
  // INA228 -> shutdown mode (~3.5uA vs ~350uA continuous).
  // I2C writes can fail silently -> retry with readback verification.
  for (int retry = 0; retry < 3; retry++) {
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(0x01);  // ADC_CONFIG
    Wire.write(0x00);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) {
      delay(10);
      continue;
    }
    delay(2);
    Wire.beginTransmission(INA228_ADDR);
    Wire.write(0x01);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)INA228_ADDR, (uint8_t)2);
    uint16_t rb = 0;
    if (Wire.available() >= 2) {
      rb = (Wire.read() << 8) | Wire.read();
    }
    if ((rb & 0xF000) == 0x0000) break;
    delay(10);
  }

  // INA228 -> release latched ALERT (under-voltage alert is ALATCH=1 -> ALERT stays LOW
  // -> RAK4630 internal pull-up wastes ~330uA). Switch to transparent mode and
  // zero the threshold so no condition can re-assert.
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(0x0B);  // DIAG_ALRT
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
  Wire.beginTransmission(INA228_ADDR);
  Wire.write(0x08);  // BUVL
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  // BQ25798 -> disable ADC (saves ~500uA continuous draw)
  Wire.beginTransmission(BQ25798_I2C_ADDR);
  Wire.write(0x2E);  // ADC_CONTROL
  Wire.write(0x00);
  Wire.endTransmission();

  // BQ25798 -> mask all interrupts + clear flags so INT pin de-asserts.
  // Otherwise any pending flag holds INT LOW and INPUT_PULLUP wastes ~254uA.
  static const uint8_t mask_regs[] = {0x28, 0x29, 0x2A, 0x2B, 0x2C, 0x2D};
  for (uint8_t r : mask_regs) {
    Wire.beginTransmission(BQ25798_I2C_ADDR);
    Wire.write(r);
    Wire.write(0xFF);
    Wire.endTransmission();
  }
  static const uint8_t flag_regs[] = {0x22, 0x23, 0x24, 0x25, 0x26, 0x27};
  for (uint8_t r : flag_regs) {
    Wire.beginTransmission(BQ25798_I2C_ADDR);
    Wire.write(r);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)BQ25798_I2C_ADDR, (uint8_t)1);
    while (Wire.available()) Wire.read();
  }
}

} // namespace inhero
