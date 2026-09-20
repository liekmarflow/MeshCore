---
description: >-
  Technical documentation for Inhero LoRa mesh infrastructure hardware —
  commissioning, battery chemistry, power management, CLI and telemetry.
---

# Inhero Documentation

Technical documentation for Inhero hardware built for autonomous, long-term
operation of LoRa mesh infrastructure. All figures are verifiable and refer
to the hardware revision as shipped.

## Hardware

### Inhero MR2 — solar mesh repeater board

Application-specific platform for repeater and sensor nodes at hard-to-reach
sites. Built around a RAK4630 (nRF52840 + SX1262), with a BQ25798 buck-boost
charger with MPPT, an INA228 coulomb counter for precise SOC tracking, an
RV-3028 RTC and a BME280. Supports Li-Ion, LiFePO4, LTO and Na-Ion.
Form factor 45 × 40 mm.

[**Go to the MR2 documentation →**](mr2/index.md)

| Chapter | Contents |
| --- | --- |
| [Overview](mr2/index.md) | Feature matrix, power management, CLI commands, CE notes |
| [Quick Start](mr2/quick-start.md) | First commissioning and basic CLI setup |
| [Datasheet](mr2/datasheet.md) | Hardware specifications and pinout |
| [Battery Guide](mr2/battery-guide.md) | Chemistry comparison, cold behaviour, cell selection |
| [Power Management](mr2/power-management.md) | Full technical documentation on charging, sleep and MPPT |
| [CLI Cheat Sheet](mr2/cli-cheat-sheet.md) | All board-specific commands at a glance |
| [Telemetry](mr2/telemetry.md) | Which channels are transmitted and what the app displays |
| [FAQ](mr2/faq.md) | Common questions on operation, charging and troubleshooting |

## Common starting points

- **Which battery chemistry for winter operation?** The
  [Battery Guide](mr2/battery-guide.md) compares Li-Ion, LiFePO4, LTO and
  Na-Ion including cold behaviour and charge inhibition below 0 °C.
- **Why does the app show an unexpected SOC?** See
  [Telemetry](mr2/telemetry.md) and the SOC→Li-Ion mapping section in the
  [Overview](mr2/index.md).
- **Red LED blinking, battery not charging.** See the
  [FAQ](mr2/faq.md) and the register verification section in the
  [Overview](mr2/index.md).
- **Which antenna and TX power are permitted?** The regulatory limits are
  listed at the end of the [Overview](mr2/index.md).

## Where to buy

Boards, kits and accessories are available from the
[Inhero shop](https://shop.inhero.de/en/) — including the
[MR2 board](https://shop.inhero.de/en/products/inhero-mr-2-solar-mesh-repeater-board-rak4630-sx1262-mppt-red-ce-gepruft),
the [Hilltop S](https://shop.inhero.de/en/products/hilltop-mesh-repeater-s-bausatz-mr-2)
and [Hilltop L](https://shop.inhero.de/en/products/hilltop-mesh-repeater-l-bausatz-mr-2)
kits, and [868 MHz antennas](https://shop.inhero.de/en/collections/antennen).

## Sources and licence

The firmware documentation is maintained in the
[MeshCore fork with MR2 support](https://github.com/liekmarflow/MeshCore).
This site is the authoritative, up-to-date version.
