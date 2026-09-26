# MR2 RTC and low-voltage sleep regressions

Run `python test/mr2_rtc/run_test.py` with a C++17 `g++` on PATH, or pass
`--cxx path/to/g++`. No PlatformIO packages are needed. The runner builds and
executes two temporary host binaries with warnings treated as errors.

The first links the production `Rv3028Wake.cpp` and `I2cBusRecovery.cpp` against
register-based Wire and GPIO mocks. It injects a transient and persistent fault
at every transfer and every queued byte in initialization, timer setup and TF
clearing. Additional cases cover short/empty reads, ACK without persisted data,
timer clamping, calibration preservation, EERD preventing automatic refresh
without EEPROM writes, exclusive timer interrupts for sleep, a concurrent status event,
permanently busy EEPROM, stuck INT, and bounded open-drain bus recovery. The
production self-test method is extracted unchanged to verify user-RAM restore
after failures and rejection of failed restore/readback.

The second compiles unchanged production board methods and the complete Low-V
portion of the periodic dispatcher against side-effect-recording stubs. It checks
all three Low-V entry points: an unavailable RTC continues normal boot or aborts
before shutdown side effects; a verified RTC permits sleep. It also covers user
and thermal shutdown, a 60-second retry interval across the `millis()` rollover,
and voltage recovery or failed voltage reads during that interval.
The retained charger-fault interlock prevents both early boot sleep paths;
runtime sleep proceeds only after successful checked charger recovery, with no
shutdown side effects when recovery fails. The actual interlock persistence and
charger recovery implementations are tested separately in `test/mr2_jeita`.

These tests verify control flow and modeled I2C/GPIO behavior. Actual RTC reset
recovery, shared-bus electrical timing, and timer wake after SYSTEMOFF still need
an MR2 hardware test.
