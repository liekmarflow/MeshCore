---
description: >-
  Technische Dokumentation der Inhero-Hardware für LoRa-Mesh-Infrastruktur —
  Inbetriebnahme, Akkuchemie, Energieverwaltung, CLI und Telemetrie.
---

# Inhero Dokumentation

Technische Dokumentation der Inhero-Hardware für den autarken Dauerbetrieb
von LoRa-Mesh-Infrastruktur. Alle Angaben sind nachprüfbar und beziehen sich
auf die jeweils ausgelieferte Hardware-Revision.

## Hardware

### Inhero MR2 — Solar-Mesh-Repeater-Board

Anwendungsspezifische Plattform für Repeater- und Sensorknoten an
wartungsintensiven Standorten. Kern ist ein RAK4630 (nRF52840 + SX1262),
ergänzt um einen BQ25798-Buck-Boost-Lader mit MPPT, einen INA228
Coulomb-Counter für präzises SOC-Tracking, RV-3028-RTC und BME280.
Unterstützt Li-Ion, LiFePO4, LTO und Na-Ion. Formfaktor 45 × 40 mm.

[**Zur MR2-Dokumentation →**](mr2/index.md)

| Kapitel | Inhalt |
| --- | --- |
| [Übersicht](mr2/index.md) | Feature-Matrix, Energieverwaltung, CLI-Befehle, CE-Hinweise |
| [Schnellstart](mr2/quick-start.md) | Erste Inbetriebnahme und CLI-Grundkonfiguration |
| [Datenblatt](mr2/datasheet.md) | Hardware-Spezifikationen und Pinout |
| [Akkuchemie-Ratgeber](mr2/battery-guide.md) | Vergleich der Chemien, Kälteverhalten, Zellauswahl |
| [Energieverwaltung](mr2/power-management.md) | Vollständige technische Dokumentation zu Laden, Sleep und MPPT |
| [CLI-Referenz](mr2/cli-cheat-sheet.md) | Alle board-spezifischen Befehle auf einen Blick |
| [Telemetrie](mr2/telemetry.md) | Welche Kanäle übertragen werden und was die App anzeigt |
| [FAQ](mr2/faq.md) | Häufige Fragen zu Betrieb, Laden und Fehlersuche |

## Häufige Einstiegsfragen

- **Welche Akkuchemie für den Winterbetrieb?** Der
  [Akkuchemie-Ratgeber](mr2/battery-guide.md) vergleicht Li-Ion, LiFePO4,
  LTO und Na-Ion inklusive Kälteverhalten und Ladefreigabe unter 0 °C.
- **Warum zeigt die App einen anderen SOC als erwartet?** Siehe
  [Telemetrie](mr2/telemetry.md) sowie den Abschnitt zum
  SOC→Li-Ion-Mapping in der [Übersicht](mr2/index.md).
- **Rote LED blinkt, Akku lädt nicht.** Siehe
  [FAQ](mr2/faq.md) und die Registerverifikation in der
  [Übersicht](mr2/index.md).
- **Welche Antenne und welche Sendeleistung sind zulässig?** Die
  regulatorischen Grenzwerte stehen am Ende der
  [Übersicht](mr2/index.md).

## Hardware beziehen

Boards, Bausätze und Zubehör sind im
[Inhero Shop](https://shop.inhero.de/) erhältlich — darunter das
[MR2-Board](https://shop.inhero.de/products/inhero-mr-2-solar-mesh-repeater-board-rak4630-sx1262-mppt-red-ce-gepruft),
die Bausätze [Hilltop S](https://shop.inhero.de/products/hilltop-mesh-repeater-s-bausatz-mr-2)
und [Hilltop L](https://shop.inhero.de/products/hilltop-mesh-repeater-l-bausatz-mr-2)
sowie [868-MHz-Antennen](https://shop.inhero.de/collections/antennen).

## Quellen und Lizenz

Die Firmware-Dokumentation wird aus dem
[MeshCore-Fork mit MR2-Support](https://github.com/liekmarflow/MeshCore)
gepflegt. Diese Seite ist die maßgebliche, jeweils aktuelle Fassung.
