# MR2 JEITA and battery configuration regression tests

Run with Python 3 and a C++17 compiler:

```powershell
python test/mr2_jeita/run.py --cxx C:/Tools/mingw64/bin/g++.exe
```

The runner extracts production configuration methods, persistence helpers,
chemistry tables, constants and the relevant CLI command branches on every run.
Unrelated telemetry/calibration command branches are omitted; the original CLI
unknown-command response remains. The test explicitly rejects a returned
`board.saved` dispatcher. No generated source or executable is retained in the
repository. The `.inc` fixture is not a PlatformIO test translation unit.

The host model supplies preference storage, CE GPIO, BQ register state and an INA
dependency. Tests cover:

- Strict `imax < 0.05C`, including equality, invalid/NaN/infinite values and
  whole-mAh capacity normalization before the comparison.
- Rejected enable/current/capacity changes leave preferences, GPIO, charger,
  statistics and monitoring unchanged; CLI messages match the specified strings.
- Accepted override survives reboot even before the capacity RAM cache is filled;
  invalid legacy requests are removed and never reactivate after a later change.
- Enabling discards custom frost behavior; disabling returns to zero; repeated
  disabling preserves a subsequently configured frost value.
- All 20 different chemistry transitions reset charge settings, capacity, SOC and
  battery history while retaining LED, altitude and temperature calibration.
  Repeating the current chemistry is a no-op in a healthy configuration.
- LTO and Na-ion temperature behavior is chemistry-defined and has no user gate.
- Real preference writes/readbacks, failed removals/updates, ignored register
  writes, JEITA read/write failures and every charger write/checked register read
  in a chemistry change report failure and keep CE low.
- After a hardware error, accepted settings remain visible. A successful retry
  restores the entire checked charger configuration before re-enabling CE;
  retrying the current chemistry preserves settings while recovering hardware.
- The persistent charger-fault interlock survives an MCU reset, including after
  partial JEITA register programming. Only a complete verified recovery clears
  it. Unreadable storage or a malformed retained marker also blocks early sleep.
  Recovery via a different setter resynchronizes a capacity saved before an error.

The BQ model includes register encodings relevant to these checks and CELL reset
behavior. It does not run the real I2C driver, LittleFS, thermal regulation, SOC
integration or hardware interrupts. Firmware build and physical-device tests
remain separate checks; a passing host suite is not hardware validation.
