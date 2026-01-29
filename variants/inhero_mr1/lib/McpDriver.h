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
#ifndef MCPDRIVER
#define MCPDRIVER

#include <Arduino.h>
#include <Wire.h>

// Enums for channels and terminals
typedef enum {
  MCP4652_CHANNEL_0,
  MCP4652_CHANNEL_1
} mcp4652_channel_t;

typedef enum {
  MCP4652_TERMINAL_W,
  MCP4652_TERMINAL_B
} mcp4652_terminal_t;

class McpDriver {
public:
  McpDriver(uint8_t i2c_address = 0x2F);  // Default I2C address
  bool begin(TwoWire *wire = &Wire);    // Initialize with optional Wire instance

  // Set wiper value (0-256) for the specified channel
  bool setWiper(mcp4652_channel_t channel, uint16_t value);

  // Connect (true) or disconnect (false) a terminal for the specified channel
  bool setTerminal(mcp4652_channel_t channel, mcp4652_terminal_t terminal, bool connect);

  // Read current wiper value for the specified channel
  uint16_t readWiper(mcp4652_channel_t channel);

  // Read current TCON register value
  uint16_t readTCON();

private:
  uint8_t _i2c_address;
  TwoWire *_wire;

  // Helper to write command
  bool writeCommand(uint8_t cmd_byte, uint8_t data_byte);

  // Helper to read register (returns 0xFFFF on error)
  uint16_t readRegister(uint8_t reg_addr);
};

#endif  // MCP4652_H