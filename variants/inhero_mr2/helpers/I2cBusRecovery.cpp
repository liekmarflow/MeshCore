/*
 * Copyright (c) 2026 Inhero GmbH
 * SPDX-License-Identifier: MIT
 */
#include "I2cBusRecovery.h"

#include <Arduino.h>

namespace inhero {
namespace {

void pullLow(uint8_t pin) {
  // Set the output latch before enabling the driver. HIGH is always obtained
  // by releasing the pin, never by driving against another bus participant.
  digitalWrite(pin, LOW);
  pinMode(pin, OUTPUT);
}

bool waitHigh(uint8_t pin) {
  for (unsigned elapsed = 0; elapsed < 1000; elapsed += 5) {
    if (digitalRead(pin) == HIGH) return true;
    delayMicroseconds(5);
  }
  return digitalRead(pin) == HIGH;
}

bool releaseBus(uint8_t sda, uint8_t scl, bool success) {
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  return success;
}

} // namespace

bool recoverI2cBus(uint8_t sda, uint8_t scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  if (!waitHigh(scl)) return releaseBus(sda, scl, false);

  for (unsigned clock = 0; clock < 9 && digitalRead(sda) == LOW; ++clock) {
    pullLow(scl);
    delayMicroseconds(5);
    pinMode(scl, INPUT_PULLUP);
    if (!waitHigh(scl)) return releaseBus(sda, scl, false);
    delayMicroseconds(5);
  }

  // Bring SDA LOW only while SCL is LOW, avoiding an accidental START.
  // This also supplies a STOP after interrupted transfers with SDA HIGH.
  pullLow(scl);
  delayMicroseconds(5);
  pullLow(sda);
  delayMicroseconds(5);
  pinMode(scl, INPUT_PULLUP);
  if (!waitHigh(scl)) return releaseBus(sda, scl, false);
  delayMicroseconds(5);
  pinMode(sda, INPUT_PULLUP);
  delayMicroseconds(5);

  const bool idle = digitalRead(sda) == HIGH && digitalRead(scl) == HIGH;
  return releaseBus(sda, scl, idle);
}

} // namespace inhero
