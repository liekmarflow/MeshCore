/*
 * Copyright (c) 2026 Inhero GmbH
 * SPDX-License-Identifier: MIT
 *
 * Global I2C bus mutex for thread-safe Wire access.
 *
 * The nRF52 Wire (TWI) peripheral is shared between:
 *   - Main loop (board.tick, sensors.loop, CLI commands)
 *   - solarMpptTask (BQ25798 + INA228 polling)
 *   - socUpdateTask (INA228 Coulomb counter + voltage monitor + RTC)
 *
 * Without this mutex, a higher-priority task (socUpdateTask at prio 2) can
 * preempt the main loop or solarMpptTask mid-I2C-transaction, corrupting
 * the Wire peripheral state.  This causes:
 *   - I2C bus stuck (SDA held low after interrupted repeated-start)
 *   - Wrong data returned from INA228/BQ25798/RTC registers
 *   - Eventual system hang: LoRa stops responding while heartbeat LED blinks
 *
 * Usage:
 *   Include this header, then wrap I2C operations:
 *     I2C_MUTEX_TAKE();
 *     Wire.beginTransmission(...);
 *     ...
 *     Wire.endTransmission();
 *     I2C_MUTEX_GIVE();
 *
 *   The mutex is recursive, so nested takes from the same task are safe.
 */
#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

extern SemaphoreHandle_t g_i2c_mutex;

/// Must be called once before any task uses I2C (typically in BoardConfigContainer::begin())
void i2c_mutex_init();

/// Take the I2C mutex (blocks up to 2 s, then returns false)
inline bool i2c_mutex_take() {
  if (g_i2c_mutex == NULL) return true;  // Not yet initialised (pre-RTOS)
  return xSemaphoreTakeRecursive(g_i2c_mutex, pdMS_TO_TICKS(2000)) == pdTRUE;
}

/// Give (release) the I2C mutex
inline void i2c_mutex_give() {
  if (g_i2c_mutex == NULL) return;
  xSemaphoreGiveRecursive(g_i2c_mutex);
}

#define I2C_MUTEX_TAKE()  i2c_mutex_take()
#define I2C_MUTEX_GIVE()  i2c_mutex_give()
