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

class InheroMr1Board : public mesh::MainBoard {
protected:
  uint8_t startup_reason;

public:
  void begin();
  // void configure();

  uint8_t getStartupReason() const override { return startup_reason; }

  uint16_t getBattMilliVolts() override;

  const char* getManufacturerName() const override { return "Inhero MR-1"; }

  void reboot() override { NVIC_SystemReset(); }

  bool startOTAUpdate(const char* id, char reply[]) override;

  bool getCustomGetter(const char* getCommand, char* reply, uint32_t maxlen) override;
  const char* setCustomSetter(const char* setCommand) override;
  bool queryBoardTelemetry(CayenneLPP& telemetry) override;

private:
  uint8_t findNextFreeChannel(CayenneLPP& lpp);
};
