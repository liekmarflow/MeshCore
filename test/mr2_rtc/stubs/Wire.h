#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

struct Transfer {
  enum Kind { Write, ReadPointer, Read } kind;
  uint8_t reg;
  size_t length;
};

class WireMock {
public:
  std::array<uint8_t, 256> registers{};
  std::vector<Transfer> transfers;
  enum Fault { None, Nack, ShortRead } fault = None;
  int failAt = -1;
  bool persistent = false;
  bool faultStarted = false;
  int ignoredWriteReg = -1;
  uint8_t statusRaisedBeforeWrite = 0;
  int failQueueAt = -1;
  int queueCalls = 0;
  bool persistentQueueFailure = false;
  int emptyReadBufferReg = -1;
  int beginCount = 0;
  int endCount = 0;

  void begin();
  void end();
  void beginTransmission(uint8_t address);
  size_t write(uint8_t value);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t address, uint8_t size);
  int available();
  int read();

private:
  uint8_t pointer = 0;
  std::vector<uint8_t> tx;
  std::vector<uint8_t> rx;
  size_t readIndex = 0;
  bool record(Transfer transfer);
};

extern WireMock Wire;
