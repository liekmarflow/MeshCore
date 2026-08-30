# MeshCore — Inhero MR2 Fork

This repository is a downstream fork of [meshcore-dev/MeshCore](https://github.com/meshcore-dev/MeshCore)
maintained for the **Inhero MR2** LoRa repeater hardware. It carries
hardware-specific firmware code, a tested release branch, and complete
operator documentation for the MR2 board.

The upstream MeshCore project README is preserved as
[`README.upstream.md`](README.upstream.md).

---

## Inhero MR2 Variant

- **Hardware:** RAK4630(H) (nRF52840 + SX1262, high-band variant), BQ25798 buck/boost charger,
  INA228 coulomb counter, RV-3028 RTC, BME280, TPS62840 3.3 V rail,
  universal 3.6–24 V solar input with autonomous MPPT.
- **LoRa bands:** IN865, EU868, US915, AU915, KR920, AS923.
  The low-band module RAK4630(L) (EU433 / CN470) is not fitted — see
  [LoRa Frequency Bands](variants/inhero_mr2/docs/DATASHEET.md#lora-frequency-bands).
- **Form factor:** 45 × 40 mm.
- **Idle consumption:** 6.0 mA @ 4.2 V / 7.7 mA @ 3.3 V (USB off, no TX).
- **Battery chemistries:** Li-Ion, LiFePO4, LTO, Na-Ion.
- **Hardware revision:** Rev 1.1.

### Documentation

All variant documentation lives under [`variants/inhero_mr2/docs/`](variants/inhero_mr2/docs/):

| Document | English | Deutsch |
|---|---|---|
| Variant overview | [README.md](variants/inhero_mr2/docs/README.md) | [de/README.md](variants/inhero_mr2/docs/de/README.md) |
| Quick start | [QUICK_START.md](variants/inhero_mr2/docs/QUICK_START.md) | [de/QUICK_START.md](variants/inhero_mr2/docs/de/QUICK_START.md) |
| Datasheet | [DATASHEET.md](variants/inhero_mr2/docs/DATASHEET.md) | [de/DATASHEET.md](variants/inhero_mr2/docs/de/DATASHEET.md) |
| Battery guide | [BATTERY_GUIDE.md](variants/inhero_mr2/docs/BATTERY_GUIDE.md) | [de/BATTERY_GUIDE.md](variants/inhero_mr2/docs/de/BATTERY_GUIDE.md) |
| CLI cheat sheet | [CLI_CHEAT_SHEET.md](variants/inhero_mr2/docs/CLI_CHEAT_SHEET.md) | [de/CLI_CHEAT_SHEET.md](variants/inhero_mr2/docs/de/CLI_CHEAT_SHEET.md) |
| Telemetry | [TELEMETRY.md](variants/inhero_mr2/docs/TELEMETRY.md) | [de/TELEMETRY.md](variants/inhero_mr2/docs/de/TELEMETRY.md) |
| FAQ | [FAQ.md](variants/inhero_mr2/docs/FAQ.md) | [de/FAQ.md](variants/inhero_mr2/docs/de/FAQ.md) |
| Power management | [POWER_MANAGEMENT.md](variants/inhero_mr2/docs/POWER_MANAGEMENT.md) | [de/POWER_MANAGEMENT.md](variants/inhero_mr2/docs/de/POWER_MANAGEMENT.md) |

### Building the firmware

PlatformIO environment: `Inhero_MR2_repeater`.

```bash
pio run -e Inhero_MR2_repeater
```

Build artifacts are written to `.pio/build/Inhero_MR2_repeater/`. Pre-built
binaries are attached to GitHub Releases.

### Flashing

The board exposes the standard nRF52 UF2 bootloader. Drag-and-drop the UF2
from a release asset onto the `RAK4630`/`FTHR840` mass-storage device that
appears after a double tap on the reset button. See
[QUICK_START.md](variants/inhero_mr2/docs/QUICK_START.md) for the full procedure.

---

## Relation to upstream

- This fork tracks selected upstream changes manually; it is **not** a
  rolling mirror. The MR2 variant lives in `variants/inhero_mr2/` and
  does not affect any other board.
- The board was first proposed upstream in April 2026
  ([#2316](https://github.com/meshcore-dev/MeshCore/pull/2316)) and withdrawn
  at the time: the hardware was not yet available and CE certification was
  still in progress. Both are settled now — the board is in production and
  CE-certified (RED 2014/53/EU) — and it has been proposed again, split into
  [#3131](https://github.com/meshcore-dev/MeshCore/pull/3131) (board hooks)
  and [#3132](https://github.com/meshcore-dev/MeshCore/pull/3132) (the
  variant itself). Until those land, firmware for this board is released
  from this fork.
- For the upstream MeshCore project, ecosystem documentation, and other
  supported boards, see [meshcore-dev/MeshCore](https://github.com/meshcore-dev/MeshCore).

## License

Same as upstream MeshCore — see [`license.txt`](license.txt).
