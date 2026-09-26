#!/usr/bin/env python3
"""Compile the production MR2 RTC/recovery helpers against fault-injection mocks.

Run with: python test/mr2_rtc/run_test.py [--cxx path/to/g++]
No PlatformIO or board libraries are needed. Build products are temporary.
"""

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


def method(source, signature):
    """Extract a complete production method without changing its implementation."""
    start = source.index(signature)
    cursor = source.index("{", start) + 1
    depth = 1
    while depth:
        depth += (source[cursor] == "{") - (source[cursor] == "}")
        cursor += 1
    return source[start:cursor]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=os.environ.get("CXX") or shutil.which("g++"))
    args = parser.parse_args()
    if not args.cxx:
        parser.error("g++ unavailable; pass --cxx with a C++17 compiler")

    tests = Path(__file__).resolve().parent
    root = tests.parents[1]
    helpers = root / "variants/inhero_mr2/helpers"
    config_source = (helpers.parent / "BoardConfigContainer.cpp").read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="mr2-rtc-") as directory:
        executable = Path(directory) / ("rtc.exe" if os.name == "nt" else "rtc")
        (Path(directory) / "probe.inc").write_text(
            method(config_source, "bool BoardConfigContainer::probeRtc()"), encoding="utf-8")
        rtc_cpp = Path(directory) / "rtc.cpp"
        rtc_cpp.write_text((tests / "rtc_test.cpp.in").read_text(encoding="utf-8"), encoding="utf-8")
        subprocess.run([
            args.cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-DPIN_BOARD_SDA=13", "-DPIN_BOARD_SCL=14",
            "-I", str(tests / "stubs"), "-I", str(helpers), "-I", directory,
            str(rtc_cpp), str(helpers / "Rv3028Wake.cpp"),
            str(helpers / "I2cBusRecovery.cpp"), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)

        board_source = (helpers.parent / "InheroMr2Board.cpp").read_text(encoding="utf-8")
        board_header = (helpers.parent / "InheroMr2Board.h").read_text(encoding="utf-8")
        constants = "\n".join(line for line in board_header.splitlines() if line.startswith("#define "))
        methods = "\n\n".join(method(board_source, signature) for signature in [
            "static bool restoreConfiguredChargeEnable()",
            "void InheroMr2Board::begin()",
            "void InheroMr2Board::initiateShutdown(",
            "bool InheroMr2Board::configureRTCWake(",
        ])
        # Isolate the complete Low-V decision within the periodic dispatcher;
        # unrelated MPPT, SOC and hourly scheduling require hardware drivers.
        tick = method(config_source, "void BoardConfigContainer::tickPeriodic()")
        tick = tick[tick.index("  uint32_t now = millis();"):tick.index("  // Every ~60s: MPPT cycle")]
        harness = (tests / "board_test.cpp.in").read_text(encoding="utf-8")
        harness = harness.replace("@CONSTANTS@", constants).replace("@METHODS@", methods).replace("@LOW_VOLTAGE_TICK@", tick)
        cpp = Path(directory) / "board.cpp"
        cpp.write_text(harness, encoding="utf-8")
        subprocess.run([
            args.cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror",
            str(cpp), "-o", str(executable),
        ], check=True)
        subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
