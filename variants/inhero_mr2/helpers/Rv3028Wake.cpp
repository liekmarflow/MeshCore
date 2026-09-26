/*
 * Copyright (c) 2026 Inhero GmbH
 * SPDX-License-Identifier: MIT
 */
#include "Rv3028Wake.h"
#include "I2cBusRecovery.h"

#include <MeshCore.h>
#include <Wire.h>

namespace inhero {
namespace {

// MR2 hardware: RV-3028-C7 at 0x52, INT on WB_IO1/P0.17.
constexpr uint8_t kAddress = 0x52;
constexpr uint8_t kIntPin = 17;
constexpr uint8_t kTimerLow = 0x0A;
constexpr uint8_t kTimerHigh = 0x0B;
constexpr uint8_t kStatus = 0x0E;
constexpr uint8_t kCtrl1 = 0x0F;
constexpr uint8_t kCtrl2 = 0x10;
constexpr uint8_t kClkout = 0x35;
constexpr uint8_t kBackup = 0x37;
constexpr uint8_t kTf = 0x08;
constexpr uint8_t kEeBusy = 0x80;
constexpr uint8_t kEerd = 0x08;
constexpr uint8_t kTie = 0x10;
constexpr uint8_t kTimerMask = 0x87; // TRPT, TE, TD

bool busIdle() {
  // Avoid entering the nRF52 Wire driver's wait loop on an already stuck bus.
  // This cannot provide a timeout for a fault that starts during a transfer.
  return digitalRead(PIN_BOARD_SDA) == HIGH && digitalRead(PIN_BOARD_SCL) == HIGH;
}

bool readRegister(uint8_t reg, uint8_t& value) {
  if (!busIdle()) return false;
  Wire.beginTransmission(kAddress);
  const bool queued = Wire.write(reg) == 1;
  // STOP is intentional: it also terminates a failed register-pointer write.
  if (Wire.endTransmission() != 0 || !queued) return false;
  if (!busIdle()) return false;
  const uint8_t received = Wire.requestFrom(kAddress, static_cast<uint8_t>(1));
  if (received != 1 || Wire.available() != 1) {
    while (Wire.available()) Wire.read();
    return false;
  }
  value = static_cast<uint8_t>(Wire.read());
  return true;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  if (!busIdle()) return false;
  Wire.beginTransmission(kAddress);
  bool queued = Wire.write(reg) == 1;
  queued = (Wire.write(value) == 1) && queued;
  return Wire.endTransmission() == 0 && queued;
}

bool verifyRegister(uint8_t reg, uint8_t value, uint8_t mask = 0xFF) {
  uint8_t actual;
  return readRegister(reg, actual) && (actual & mask) == (value & mask);
}

bool writeVerified(uint8_t reg, uint8_t value, uint8_t mask = 0xFF) {
  return writeRegister(reg, value) && verifyRegister(reg, value, mask);
}

bool waitForRefresh() {
  // EERD prevents new automatic refreshes, but an already running refresh
  // (about 66 ms at POR) must finish before changing the configuration RAM.
  for (unsigned elapsed = 0; elapsed <= 100; ++elapsed) {
    uint8_t status;
    if (!readRegister(kStatus, status)) return false;
    if ((status & kEeBusy) == 0) return true;
    if (elapsed < 100) delay(1);
  }
  return false;
}

bool initializeOnce() {
  uint8_t ctrl1;
  if (!readRegister(kCtrl1, ctrl1) ||
      !writeVerified(kCtrl1, ctrl1 | kEerd, 0xBF) ||
      !waitForRefresh()) return false;

  uint8_t backup, clkout, ctrl2;
  if (!readRegister(kBackup, backup) || !readRegister(kClkout, clkout) ||
      !readRegister(kCtrl2, ctrl2)) return false;

  // VDD and VBACKUP use the same 3.3 V rail on MR2. Disable DSM/LSM and
  // charging, preserving the current calibration LSB and resistor selection.
  // Keep EERD set deliberately: a daily refresh must not restore an old DSM
  // setting during operation or sleep. POR still loads EEPROM normally.
  backup = static_cast<uint8_t>((backup & 0x83) | 0x10);
  // FD=111 holds CLKOUT LOW even if an old CLKF is set. Keep CLKSY/PORIE.
  clkout = static_cast<uint8_t>((clkout & 0x48) | 0x07);
  ctrl2 &= static_cast<uint8_t>(~0x03); // 24-hour mode; never assert RESET

  return writeVerified(kBackup, backup) &&
         writeVerified(kClkout, clkout, 0xCF) &&
         writeVerified(kCtrl2, ctrl2, 0xFE);
}

bool clearTimerFlagOnce() {
  uint8_t status;
  if (!readRegister(kStatus, status)) return false;
  // Status flags are cleared by writing zero. Ones preserve other flags,
  // including an event that arrived after the status read above.
  if ((status & kTf) != 0 &&
      !writeRegister(kStatus, static_cast<uint8_t>(~kTf))) return false;
  return verifyRegister(kStatus, 0, kTf);
}

bool configureOnce(uint16_t ticks) {
  if (!initializeOnce()) return false;

  uint8_t ctrl1, ctrl2;
  if (!readRegister(kCtrl1, ctrl1) || !readRegister(kCtrl2, ctrl2)) return false;
  // Keep EERD and unrelated Control 1 settings. Select the countdown as the
  // only INT source so a later update/alarm/event cannot wake System OFF.
  const uint8_t stopped = ctrl1 & static_cast<uint8_t>(~kTimerMask);
  const uint8_t interruptOff = ctrl2 & 0x80; // keep timestamp enable only
  const uint8_t running = stopped | 0x07; // single-shot, TE=1, TD=1/60 Hz
  const uint8_t interruptOn = interruptOff | kTie;
  uint8_t clkout;
  if (!readRegister(kClkout, clkout)) return false;
  clkout &= static_cast<uint8_t>(~0x08); // PORIE is another INT source

  if (!writeVerified(kCtrl1, stopped, 0xBF) ||
      !writeVerified(kCtrl2, interruptOff, 0xFE) ||
      !writeVerified(kClkout, clkout, 0xCF) ||
      !clearTimerFlagOnce() ||
      !writeVerified(kTimerLow, static_cast<uint8_t>(ticks)) ||
      !writeVerified(kTimerHigh, static_cast<uint8_t>(ticks >> 8), 0x0F) ||
      !writeVerified(kCtrl1, running, 0xBF) ||
      !writeVerified(kCtrl2, interruptOn, 0xFE)) return false;

  // Verify the complete final state, not just the last successful transfer.
  uint8_t backup;
  if (!verifyRegister(kCtrl1, running, 0xBF) ||
      !verifyRegister(kCtrl2, interruptOn, 0xFE) ||
      !verifyRegister(kTimerLow, static_cast<uint8_t>(ticks)) ||
      !verifyRegister(kTimerHigh, static_cast<uint8_t>(ticks >> 8), 0x0F) ||
      !verifyRegister(kStatus, 0, kTf) ||
      !readRegister(kBackup, backup) || (backup & 0x7C) != 0x10 ||
      !readRegister(kClkout, clkout) || (clkout & 0x8F) != 0x07) return false;

  // The wake input must be released before System OFF can be armed.
  return digitalRead(kIntPin) == HIGH;
}

} // namespace

bool recoverRtcBus() {
  Wire.end();
  const bool recovered = recoverI2cBus(PIN_BOARD_SDA, PIN_BOARD_SCL);
  Wire.begin(); // restore the shared bus even when a device remains stuck
  return recovered;
}

bool initializeRtc() {
  if (initializeOnce()) return true;
  MESH_DEBUG_PRINTLN("RTC: initialization failed, recovering I2C bus");
  return recoverRtcBus() && initializeOnce();
}

bool configurePeriodicWake(uint16_t minutes) {
  const uint16_t ticks = minutes == 0 ? 1 : (minutes > 4095 ? 4095 : minutes);
  if (configureOnce(ticks)) return true;
  MESH_DEBUG_PRINTLN("RTC: wake setup failed, recovering I2C bus");
  return recoverRtcBus() && configureOnce(ticks);
}

bool clearTimerFlag() {
  if (clearTimerFlagOnce()) return true;
  MESH_DEBUG_PRINTLN("RTC: clearing TF failed, recovering I2C bus");
  return recoverRtcBus() && clearTimerFlagOnce();
}

} // namespace inhero
