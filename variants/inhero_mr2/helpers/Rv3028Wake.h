/*
 * Copyright (c) 2026 Inhero GmbH
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <Arduino.h>

namespace inhero {

// Reinitializes Wire and clears an interrupted bus transaction. This is not
// a chip reset and cannot repair a RTC that remains electrically unavailable.
bool recoverRtcBus();

// Applies and verifies MR2's single-supply configuration in the RAM mirrors:
// backup switching/charging off, CLKOUT off and 24-hour mode. Preserves the
// calibration and deliberately sets EERD: automatic refresh must not restore
// an old backup mode while running/asleep. POR still loads EEPROM normally.
// No EEPROM is programmed. This is also run before programming a wake.
bool initializeRtc();

// Configures the RV-3028-C7 periodic countdown timer to fire after `minutes`
// at 1/60 Hz, single-shot, with TIE=1 and other INT sources disabled.
// `minutes` is clamped to [1, 4095] (12-bit timer register).
// True requires verified registers, a cleared TF and an inactive INT pin.
// On failure the caller must abort sleep. Retries the whole setup once after
// bus recovery, rather than trusting partially completed writes.
bool configurePeriodicWake(uint16_t minutes);

// Clears the RV-3028 Timer Flag (TF, status bit 3) without touching other bits.
// Other status flags receive ones (write-zero-to-clear), preserving events
// that arrive during the operation. System Sleep wake resets the MCU, so the
// FALLING-edge ISR never sees the RTC event and TF can stay latched.
// True means TF was read back clear. Other interrupt flags are preserved.
bool clearTimerFlag();

} // namespace inhero
