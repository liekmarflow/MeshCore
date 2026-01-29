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
#include "McpDriver.h"

// Register addresses
#define MCP4652_REG_WIPER0 0x00
#define MCP4652_REG_WIPER1 0x01
#define MCP4652_REG_TCON   0x04

// Commands
#define MCP4652_CMD_WRITE 0x00
#define MCP4652_CMD_READ  0x03  // 11 in bits 3:2

McpDriver::McpDriver(uint8_t i2c_address) : _i2c_address(i2c_address), _wire(&Wire) {}

bool McpDriver::begin(TwoWire *wire) {
  _wire = wire;
  _wire->begin();
  // Optional: Check if device responds
  _wire->beginTransmission(_i2c_address);
  return (_wire->endTransmission() == 0);
}

bool McpDriver::setWiper(mcp4652_channel_t channel, uint16_t value) {
  if (value > 256) return false;
  uint8_t reg_addr = (channel == MCP4652_CHANNEL_0) ? MCP4652_REG_WIPER0 : MCP4652_REG_WIPER1;
  uint8_t cmd_byte = (reg_addr << 4) | (MCP4652_CMD_WRITE << 2) | ((value >> 8) & 0x01);  // D8 in bit 0, D9=0
  uint8_t data_byte = value & 0xFF;
  return writeCommand(cmd_byte, data_byte);
}

bool McpDriver::setTerminal(mcp4652_channel_t channel, mcp4652_terminal_t terminal, bool connect) {
  uint16_t tcon = readTCON();
  if (tcon == 0xFFFF) return false;

  // Bits: For Channel 0: R0W=bit1, R0B=bit0, R0HW=bit3
  // For Channel 1: offset by 4 bits
  uint8_t base_bit = (channel == MCP4652_CHANNEL_0) ? 0 : 4;
  uint8_t bit_pos;
  switch (terminal) {
    case MCP4652_TERMINAL_W: bit_pos = base_bit + 1; break;
    case MCP4652_TERMINAL_B: bit_pos = base_bit + 0; break;
    default: return false;
  }

  if (connect) {
    tcon |= (1 << bit_pos);
  } else {
    tcon &= ~(1 << bit_pos);
  }

  // Write TCON: D8 is part of the value (MSB)
  uint8_t cmd_byte = (MCP4652_REG_TCON << 4) | (MCP4652_CMD_WRITE << 2) | ((tcon >> 8) & 0x01);  // D8
  uint8_t data_byte = tcon & 0xFF;
  return writeCommand(cmd_byte, data_byte);
}

uint16_t McpDriver::readWiper(mcp4652_channel_t channel) {
  uint8_t reg_addr = (channel == MCP4652_CHANNEL_0) ? MCP4652_REG_WIPER0 : MCP4652_REG_WIPER1;
  return readRegister(reg_addr);
}

uint16_t McpDriver::readTCON() {
  return readRegister(MCP4652_REG_TCON);
}

bool McpDriver::writeCommand(uint8_t cmd_byte, uint8_t data_byte) {
  _wire->beginTransmission(_i2c_address);
  _wire->write(cmd_byte);
  _wire->write(data_byte);
  return (_wire->endTransmission() == 0);
}

uint16_t McpDriver::readRegister(uint8_t reg_addr) {
  uint8_t cmd_byte = (reg_addr << 4) | (MCP4652_CMD_READ << 2);  // D9/D8=0 for read
  _wire->beginTransmission(_i2c_address);
  _wire->write(cmd_byte);
  if (_wire->endTransmission() != 0) return 0xFFFF;

  _wire->requestFrom(_i2c_address, (uint8_t)2);
  if (_wire->available() < 2) return 0xFFFF;

  // According to datasheet (Section 7.3): For read, the device sends two bytes.
  // First byte: AD3:AD0 (bits 7-4), CMD (bits 3-2 = 11), D9 (bit 1), D8 (bit 0)
  // Second byte: D7:D0
  // For volatile wiper registers, D9 is always 0.
  // So we extract D8 from first byte bit 0, and ignore D9/other bits.
  uint8_t high_byte = _wire->read();
  uint8_t low_byte = _wire->read();
  return ((high_byte & 0x01) << 8) | low_byte;  // D8 << 8 | D7:D0
}