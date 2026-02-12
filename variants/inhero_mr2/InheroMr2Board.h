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
 */
#pragma once

#include <Arduino.h>
#include <CayenneLPP.h>
#include <MeshCore.h>
#include <helpers/NRF52Board.h>

// Power Management Mode Configuration
// Set to true for lab testing (fast intervals), false for production deployment
#define TESTING_MODE false  // true = 60s intervals, false = 1h normal / 12h danger zone

// LoRa radio module pins for Inhero MR-2
#define P_LORA_DIO_1             47
#define P_LORA_NSS               42
#define P_LORA_RESET             RADIOLIB_NC // 38
#define P_LORA_BUSY              46
#define P_LORA_SCLK              43
#define P_LORA_MISO              45
#define P_LORA_MOSI              44
#define SX126X_POWER_EN          37

// GPS module support (future expansion)
// Note: GPS pins not yet configured in MR2 hardware
// GPS_BAUD_RATE would be 9600, GPS_ADDRESS would be 0x42 (I2C)

#define SX126X_DIO2_AS_RF_SWITCH true
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

// built-ins
#define PIN_VBAT_READ            5
#define ADC_MULTIPLIER           (3 * 1.73 * 1.187 * 1000)

// Power Management Configuration (v0.2 - INA228 + RTC)
// Note: GPIO17 (WB_IO1) is used for RTC_INT, not available for GPS_1PPS
#define RTC_INT_PIN              17  // GPIO17 (WB_IO1) - RTC Interrupt from RV-3028
#define RTC_I2C_ADDR             0x52  // RV-3028-C7 I2C address
#define INA228_I2C_ADDR          0x40  // INA228 I2C address (A0=GND, A1=GND)
// Note: INA228 ALERT pin controls TPS62840 EN directly (hardware UVLO), not connected to RAK4630

// RV-3028-C7 RTC Register Addresses (Per Application Manual Section 3.2)
#define RV3028_REG_STATUS        0x0E  // Status register (TF flag at bit 3)
#define RV3028_REG_CTRL1         0x0F  // Control 1 (TE at bit 2, TD at bits 1:0)
#define RV3028_REG_CTRL2         0x10  // Control 2 (TIE at bit 4)
#define RV3028_REG_TIMER_VALUE_0 0x0A  // Timer Value 0 (lower 8 bits)
#define RV3028_REG_TIMER_VALUE_1 0x0B  // Timer Value 1 (upper 4 bits)

// Shutdown reason codes (stored in GPREGRET2 bits [1:0])
#define SHUTDOWN_REASON_NONE          0x00
#define SHUTDOWN_REASON_LOW_VOLTAGE   0x01
#define SHUTDOWN_REASON_USER_REQUEST  0x02
#define SHUTDOWN_REASON_THERMAL       0x03

// Power management state flags (stored in GPREGRET2 bits [7:2])
#define GPREGRET2_IN_DANGER_ZONE      0x04  // Bit 2: In Danger Zone (SX1262 disabled)

class InheroMr2Board : public NRF52BoardDCDC {
public:
  InheroMr2Board() : NRF52Board("InheroMR2_OTA") {}
  void begin();
  void tick() override;  // Feed watchdog and perform board-specific tasks

  uint16_t getBattMilliVolts() override;
  
  // Power Management Methods (v0.2 - INA228 + RTC)
  /// @brief Initiate controlled shutdown with filesystem protection
  /// @param reason Shutdown reason code (stored in GPREGRET2 for next boot)
  void initiateShutdown(uint8_t reason);
  
  /// @brief Configure RV-3028 RTC countdown timer for periodic wake-up
  /// @param hours Wake-up interval in hours (typically 1 = hourly checks)
  void configureRTCWake(uint32_t hours);
  
  /// @brief Get voltage threshold for critical shutdown (chemistry-specific)
  /// @return Threshold in millivolts - Danger zone boundary and 0% SOC point
  uint16_t getVoltageCriticalThreshold();
  
  /// @brief Get hardware UVLO voltage cutoff (chemistry-specific)
  /// @return Hardware cutoff voltage in millivolts
  uint16_t getVoltageHardwareCutoff();
  
  /// @brief RTC interrupt handler (called by hardware interrupt)
  static void rtcInterruptHandler();

  const char* getManufacturerName() const override { return "Inhero MR-2"; }

  void reboot() override { NVIC_SystemReset(); }

  bool startOTAUpdate(const char* id, char reply[]) override;

  bool getCustomGetter(const char* getCommand, char* reply, uint32_t maxlen) override;
  const char* setCustomSetter(const char* setCommand) override;
  bool queryBoardTelemetry(CayenneLPP& telemetry) override;

private:
  uint8_t findNextFreeChannel(CayenneLPP& lpp);
  static volatile bool rtc_irq_pending;
};
