#!/usr/bin/env python3
"""Run MR2 configuration regression tests against extracted production methods.

Only platform dependencies are replaced by host fakes. Production configuration
functions and constants are read afresh for each run; no copies are tested.
"""

import argparse
import importlib.util
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location("mr2_vsysmin", HERE.parent / "mr2_vsysmin/run.py")
extractor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(extractor)


def extract_production():
    header = (ROOT / "variants/inhero_mr2/BoardConfigContainer.h").read_text(encoding="utf-8")
    source = (ROOT / "variants/inhero_mr2/BoardConfigContainer.cpp").read_text(encoding="utf-8")
    global_types = header[header.index("#define MPPT_STATS_HOURS"):header.index("class BoardConfigContainer")]
    start = header.index("  enum BatteryType")
    end = header.index("  static BatteryType getBatteryTypeFromCommandString")
    declarations = header[start:end]
    declarations += "\n" + "\n".join(re.findall(r"  static constexpr const char\* [A-Z_]+ = [^;]+;", header))
    functions = []
    names = (
        "getBatteryTypeFromCommandString", "getBatteryTypeCommandString",
        "getFrostChargeBehaviourFromCommandString", "getFrostChargeBehaviourCommandString",
        "getBatteryProperties", "getNominalVoltage", "getBatteryType", "getFrostChargeBehaviour",
        "getMaxChargeCurrent_mA", "getBatteryCapacity", "getMPPTEnabled",
        "loadBatType", "loadFrost", "loadMaxChrgI", "loadMpptEnabled", "loadBatteryCapacity",
        "isBatteryCapacitySet", "loadJeitaIgnoreEnabled", "getJeitaIgnoreEnabled",
        "isJeitaIgnoreCurrentAllowed", "jeitaIgnoreGateOk", "setJeitaIgnore",
        "applyJeitaIgnore", "normalizeJeitaIgnore", "disableCharging",
        "storeSetting", "removeSetting", "recoverChargeConfiguration", "hasChargeConfigurationFault",
        "verifyChargeConfiguration", "configureBaseBQ", "configureChemistry",
        "resetBatteryStatistics", "setBatteryType", "setFrostChargeBehaviour",
        "setMaxChargeCurrent_mA", "setBatteryCapacity", "trim", "getChargeCurrentAsStr",
        "getAvailableBatOptions", "getAvailableFrostChargeBehaviourOptions",
    )
    for name in names:
        matches = list(re.finditer(r"^\S[^\n]*BoardConfigContainer::" + name + r"\([^\n]*", source, re.MULTILINE))
        if not matches:
            raise ValueError(f"Production method missing: {name}")
        functions.extend(extractor.function_body(source, match.group()) for match in matches)
    return global_types, declarations, "\n\n".join(functions)


def extract_cli():
    """Retain tested command branches verbatim and the real unknown fallback.

    Unrelated telemetry/calibration branches are omitted to keep the fake I/O
    surface small. Branch extraction follows their top-level indentation, while
    balanced-body extraction accounts for nested conditionals and strings.
    """
    source = (ROOT / "variants/inhero_mr2/helpers/CliCommands.cpp").read_text(encoding="utf-8")
    if re.search(r'(?:strcmp|strncmp)\([^\n]*"saved(?: |")', source):
        raise ValueError("Removed board.saved command has returned")
    selected = {"bat", "fmax", "imax", "batcap", "jeitaignore"}
    functions = []
    for signature in ("bool handleGet(", "const char* handleSet("):
        function = extractor.function_body(source, signature)
        branches = list(re.finditer(r"^  (?:} else )?(if \([^\n]+)", function, re.MULTILINE))
        parts = []
        for branch in branches:
            first_line = branch.group(1)
            command = re.search(r'"([a-z]+) ?"', first_line)
            if command and command.group(1) in selected:
                parts.append(extractor.function_body(function, first_line))
        last = branches[-1]
        last_body = extractor.function_body(function, last.group(1))
        trailing = function[last.start(1) + len(last_body):]
        functions.append(function[:branches[0].start()] + "  " + " else ".join(parts) + trailing)
    return "\n\n".join(functions)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cxx", default=shutil.which("g++"))
    args = parser.parse_args()
    if not args.cxx:
        parser.error("g++ not found; supply --cxx /path/to/g++")
    global_types, declarations, functions = extract_production()
    with tempfile.TemporaryDirectory(prefix="mr2-jeita-") as temporary:
        directory = Path(temporary)
        for filename, content in (("production_types.inc", global_types),
                                  ("production_declarations.inc", declarations),
                                  ("production_functions.inc", functions),
                                  ("production_cli.inc", extract_cli())):
            (directory / filename).write_text(content, encoding="utf-8")
        executable = directory / "regression.exe"
        subprocess.run([
            args.cxx, "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
            "-x", "c++", str(HERE / "fixture.inc"), "-I", str(directory), "-o", str(executable),
        ], check=True)
        return subprocess.run([str(executable)]).returncode


if __name__ == "__main__":
    raise SystemExit(main())
