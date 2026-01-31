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

// LoRa radio module pins for Inhero MR-1
#define P_LORA_DIO_1             47
#define P_LORA_NSS               42
#define P_LORA_RESET             RADIOLIB_NC // 38
#define P_LORA_BUSY              46
#define P_LORA_SCLK              43
#define P_LORA_MISO              45
#define P_LORA_MOSI              44
#define SX126X_POWER_EN          37

// #define PIN_GPS_SDA       13  //GPS SDA pin (output option)
// #define PIN_GPS_SCL       14  //GPS SCL pin (output option)
// #define PIN_GPS_TX        16  //GPS TX pin
// #define PIN_GPS_RX        15  //GPS RX pin
#define PIN_GPS_1PPS             17 // GPS PPS pin
#define GPS_BAUD_RATE            9600
#define GPS_ADDRESS              0x42 // i2c address for GPS

#define SX126X_DIO2_AS_RF_SWITCH true
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

// built-ins
#define PIN_VBAT_READ            5
#define ADC_MULTIPLIER           (3 * 1.73 * 1.187 * 1000)

// Hardware version detection
enum HardwareVersion : uint8_t {
  HW_UNKNOWN = 0,
  HW_V0_1 = 1,  // MCP4652 + TP2120 UVLO
  HW_V0_2 = 2   // INA228 + RV-3028 RTC (active)
};

// Power Management Configuration (v0.2)
#define RTC_INT_PIN              17  // GPIO17 (WB_IO1) - RTC Interrupt
#define RTC_I2C_ADDR             0x52  // RV-3028-C7 I2C address
#define INA228_I2C_ADDR          0x45  // INA228 I2C address (A0=GND, A1=GND)
#define MCP4652_I2C_ADDR         0x2F  // MCP4652 I2C address (v0.1 hardware)
// Note: INA228 ALERT pin controls TPS62840 EN directly (hardware UVLO), not connected to RAK

// RV-3028-C7 RTC Register Addresses
#define RV3028_REG_CTRL1         0x00  // Control 1 (TE bit)
#define RV3028_REG_CTRL2         0x01  // Control 2 (TIE, TF bits)
#define RV3028_REG_COUNTDOWN_LSB 0x09  // Countdown Timer LSB
#define RV3028_REG_COUNTDOWN_MSB 0x0A  // Countdown Timer MSB

// Shutdown reason codes (stored in GPREGRET2)
#define SHUTDOWN_REASON_NONE          0x00
#define SHUTDOWN_REASON_LOW_VOLTAGE   0x01
#define SHUTDOWN_REASON_USER_REQUEST  0x02
#define SHUTDOWN_REASON_THERMAL       0x03

class InheroMr1Board : public NRF52BoardDCDC {
public:
  InheroMr1Board() : NRF52Board("InheroMR1_OTA") {}
  void begin();
  void tick() override;  // Feed watchdog and perform board-specific tasks
  // void configure();

  uint16_t getBattMilliVolts() override;
  
  /// @brief Detect hardware version by probing I2C devices
  /// @return Hardware version enum (HW_V0_1 or HW_V0_2)
  HardwareVersion detectHardwareVersion();
  
  /// @brief Get current hardware version (cached after first detection)
  /// @return Hardware version enum
  HardwareVersion getHardwareVersion() const { return hwVersion; }
  
  // Power Management Methods (v0.2 only)
  /// @brief Initiate controlled shutdown with filesystem protection
  /// @param reason Shutdown reason code (stored in GPREGRET2 for next boot)
  void initiateShutdown(uint8_t reason);
  
  /// @brief Configure RV-3028 RTC countdown timer for periodic wake-up
  /// @param hours Wake-up interval in hours (typically 1 = hourly checks)
  void configureRTCWake(uint32_t hours);
  
  /// @brief Get voltage threshold for critical shutdown (chemistry-specific)
  /// @return Threshold in millivolts
  uint16_t getVoltageCriticalThreshold();
  
  /// @brief Get voltage threshold for wake-up with hysteresis (chemistry-specific)
  /// @return Threshold in millivolts (typically critical + 100-200mV)
  uint16_t getVoltageWakeThreshold();
  
  /// @brief RTC interrupt handler (called by hardware interrupt)
  static void rtcInterruptHandler();

  const char* getManufacturerName() const override { return "Inhero MR-1"; }

  void reboot() override { NVIC_SystemReset(); }

  bool startOTAUpdate(const char* id, char reply[]) override;

  bool getCustomGetter(const char* getCommand, char* reply, uint32_t maxlen) override;
  const char* setCustomSetter(const char* setCommand) override;
  bool queryBoardTelemetry(CayenneLPP& telemetry) override;

private:
  uint8_t findNextFreeChannel(CayenneLPP& lpp);
  HardwareVersion hwVersion = HW_UNKNOWN;  // Cached hardware version
};
