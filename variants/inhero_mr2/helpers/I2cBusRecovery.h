/*
 * Copyright (c) 2026 Inhero GmbH
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>

namespace inhero {

// Call with the I2C peripheral disabled. Uses only LOW outputs and released
// inputs, up to nine clocks, and a bounded wait for SCL. Always attempts a
// STOP, including when SDA was already HIGH. Returns true only for an idle
// bus; both pins are restored to INPUT on every return path.
bool recoverI2cBus(uint8_t sda, uint8_t scl);

} // namespace inhero
