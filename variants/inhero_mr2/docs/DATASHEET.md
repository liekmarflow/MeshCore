# Inhero MR2 — Datasheet

> **Inhero MR2 – Smart Solar Mesh Board**
> Hardware Revision 1.1

---

## Board Overview

The Inhero MR2 is a LoRa mesh repeater board based on the **RAK4630** module (nRF52840 + SX1262) with integrated smart solar charging, power monitoring, and low-voltage protection.

| Parameter | Value |
|---|---|
| **MCU** | nRF52840 (ARM Cortex-M4, 64 MHz) |
| **Radio** | Semtech SX1262 (via RAK4630) |
| **Frequency** | LoRa Sub-GHz (region-dependent) |
| **Connectivity** | LoRa, BLE 5.0, USB-C |
| **Supply Voltage** | 1S Li-Ion / 1S LiFePO4 / 2S LTO (via firmware config) |
| **Solar Input** | 3.6 V – 24 V (MPPT) |
| **Max. Solar Voc** | 25 V |
| **Charger** | BQ25798 (MPPT, JEITA) |
| **Max. Charge Current** | 50 – 1500 mA (configurable) |
| **Power Monitor** | INA228 (Coulomb Counter, ALERT) |
| **RTC** | RV-3028-C7 (wake-up timer) |
| **Buck Converter** | TPS62840 (750 mA, always on) |
| **System-Off Current** | ~15 µA |
| **PCB Size** | ~42 × 38 mm (4 × M2.5 mounting holes) |
| **Operating Temperature** | –40 °C to +85 °C (MCU spec) |

---

## PCB – Front Side (Component Side)

![Inhero MR2 Front](img/front.jpg)

### Component Map – Front

```
  ┌──────────────────────────────────────────────────────┐
  │  (BLE)              RAK4630            (LoRa)        │
  │   ○                 Module               ○       ┌──┤ ← USB-C
  │  U.FL            ┌──────────┐          U.FL      └──┤
  │                   │ RAK4630  │                       │
  │                   │ nRF52840 │                       │
  │                   │ +SX1262  │              BME280   │
  │                   └──────────┘                       │
  │                                    SS34       LED1 ○ │
  │  ┌──┐                                        LED2 ○ │
  │  │J8│                    ICs                         │
  │  └──┘              (BQ25798 etc.)                    │
  │                                                      │
  │  R100                                                │
  │ (Shunt)           TPS62840                  C222     │
  │                                            (Elko)    │
  │  3.3V_OFF                                   220µF    │
  │  (Schalter)                                 25V      │
  │             ┌──┬──┬──┬──┬──┐                         │
  │             │B+│B-│TS│S+│S-│                         │
  │             └──┴──┴──┴──┴──┘                         │
  │              Battery / Solar Connector                │
  └──────────────────────────────────────────────────────┘
```

### Components – Front Side

| Nr. | Bezeichnung | Beschreibung |
|-----|-------------|--------------|
| 1 | **RAK4630** | Core-Modul: nRF52840 SoC + SX1262 LoRa-Transceiver |
| 2 | **U.FL – BLE** | Antennenanschluss für Bluetooth Low Energy (links oben am RAK4630) |
| 3 | **U.FL – LoRa** | Antennenanschluss für LoRa Sub-GHz (rechts oben am RAK4630) |
| 4 | **USB-C** | USB-Schnittstelle für Stromversorgung, Firmware-Flash und CLI-Zugang |
| 5 | **BME280** | Umweltsensor (Temperatur, Luftfeuchtigkeit, Luftdruck) – I2C, oben rechts (Markierung „38P UP") |
| 6 | **SS34** | Schottky-Diode (3A, 40V) – Verpolungsschutz / Sperrdiode |
| 7 | **LED1** | Status-LED 1 – Heartbeat / Boot-Indikator (rechte Seite) |
| 8 | **LED2** | Status-LED 2 – BQ25798 STAT-Ausgang / Ladezustand (rechte Seite) |
| 9 | **J8** | Stecker / Anschluss (linke Seite) |
| 10 | **R100** | 100 mΩ Shunt-Widerstand für INA228 Strommessung (max. 1,6 A) |
| 11 | **BQ25798** | Akku-Laderegler mit MPPT, JEITA-Temperaturschutz und 15-Bit-ADC (I2C: 0x6B) |
| 12 | **INA228** | Power Monitor / Coulomb Counter mit ALERT-Interrupt (I2C: 0x40, ALERT → P1.02) |
| 13 | **TPS62840** | DC/DC Buck Converter, 750 mA, EN an VDD (immer aktiv) |
| 14 | **DMN2004TK-7** | N-FET für BQ CE-Pin (invertierte Logik: GPIO HIGH = Laden aktiv) |
| 15 | **C222 (220 µF / 25 V)** | Elektrolytkondensator – Pufferung des Solareingangs (SM534-Bauform) |
| 16 | **3.3V_OFF (Schalter)** | Schiebeschalter zum Trennen der 3,3 V-Versorgung (Board-Abschaltung) |
| 17 | **Battery/Solar Connector** | 5-poliger JST-Stecker (unten) – Pinout: **Batt+**, **Batt−**, **TS**, **Solar+**, **Solar−** |

### Steckerbelegung – Battery/Solar Connector (von links nach rechts)

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | **Batt +** | Batterie-Pluspol |
| 2 | **Batt −** | Batterie-Minuspol (GND) |
| 3 | **TS** | Temperatursensor (NTC) für JEITA-Ladeschutz |
| 4 | **Solar +** | Solarpanel-Pluspol (3,6 V – 24 V, max. Voc 25 V) |
| 5 | **Solar −** | Solarpanel-Minuspol (GND) |

---

## PCB – Back Side (Rückseite)

![Inhero MR2 Back](img/back.jpg)

### Component Map – Back

```
  ┌──────────────────────────────────────────────────────┐
  │  Rev. 1.1                                    QR Code │
  │                                                      │
  │  ○ (M2.5)       Header Row 1             ○ (M2.5)   │
  │          GND  RX  TX  SDA  SCL  3.3V                 │
  │           ●    ●   ●    ●    ●    ●                  │
  │                                                      │
  │          RESET GND SWCLK SWDIO 3.3V                  │
  │           ●    ●    ●     ●     ●                    │
  │              Header Row 2 (SWD/Debug)                │
  │                                                      │
  │            Inhero  MR2                               │
  │         Smart Solar Mesh Board                       │
  │                                                      │
  │  ┌────────────────────────────────────┐              │
  │  │  🔋 1S-LiFePO4 / Li-Ion           │    QR Code   │
  │  │     2S-LTO                         │              │
  │  │     via Firmware →                 │              │
  │  │  ☀  3.6V – 24V (MPPT)            │              │
  │  └────────────────────────────────────┘              │
  │                                                      │
  │  ○ (M2.5)                             ○ (M2.5)      │
  │             Close for Onboard →  (○)                 │
  │           Bat Temperature Sensor (TS)                │
  └──────────────────────────────────────────────────────┘
```

### Header & Pads – Back Side

#### Header Row 1 – UART / I2C (obere Reihe, Castellated Pads)

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | **GND** | Masse |
| 2 | **RX** | UART Receive |
| 3 | **TX** | UART Transmit |
| 4 | **SDA** | I2C Data |
| 5 | **SCL** | I2C Clock |
| 6 | **3.3V** | 3,3 V Ausgang |

#### Header Row 2 – SWD / Debug (untere Reihe, Castellated Pads)

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | **RESET** | nRF52840 Reset |
| 2 | **GND** | Masse |
| 3 | **SWCLK** | SWD Clock (Debug-Interface) |
| 4 | **SWDIO** | SWD Data (Debug-Interface) |
| 5 | **3.3V** | 3,3 V Ausgang |

#### TS Solder Bridge (unten rechts)

| Element | Beschreibung |
|---------|-------------|
| **TS Solder Bridge** | Lötbrücke für den Onboard-NTC-Temperatursensor. **Geschlossen** = Onboard-NTC aktiv (NCP15XH103F03RC, 10 kΩ @ 25 °C, Beta 3380). **Offen** = externer NTC über TS-Pin des Battery-Connectors. |

#### Mounting Holes

4× M2.5 Montagebohrungen in den Ecken der Platine.

---

## I2C Bus – Adressübersicht

| Adresse | Bauteil | Funktion |
|---------|---------|----------|
| 0x40 | INA228 | Power Monitor / Coulomb Counter |
| 0x52 | RV-3028-C7 | Echtzeituhr (RTC) |
| 0x6B | BQ25798 | Akku-Laderegler (MPPT, JEITA) |
| 0x76/0x77 | BME280 | Umweltsensor (T, H, P) |

---

## Pin-Zuordnung (Key GPIOs)

| GPIO | nRF52840 Pin | Funktion |
|------|-------------|----------|
| P0.04 | WB_IO4 | BQ CE Pin (via DMN2004TK-7 N-FET, invertiert) |
| P1.02 | — | INA228 ALERT (Low-Voltage Interrupt) |
| GPIO17 | WB_IO1 | RV-3028 RTC Interrupt |
| GPIO21 | — | BQ25798 INT |
| P1.05 | — | PE4259 RF Switch VDD (SX126X Power Enable) |

---

## Supported Battery Chemistries

| Typ | Nennspannung | Ladeendspannung | Low-V Sleep | Low-V Wake | Hysterese |
|-----|-------------|----------------|-------------|------------|-----------|
| **Li-Ion 1S** | 3,7 V | 4,2 V | 3100 mV | 3300 mV | 200 mV |
| **LiFePO4 1S** | 3,2 V | 3,6 V | 2700 mV | 2900 mV | 200 mV |
| **LTO 2S** | 4,6 V (2× 2,3 V) | 5,6 V | 3900 mV | 4100 mV | 200 mV |
| **none** | — | — | — | — | — |

---

## Firmware Environments

| Build Target | Beschreibung |
|---|---|
| `Inhero_MR2_repeater` | Standard-Repeater |
| `Inhero_MR2_repeater_bridge_rs232` | Repeater mit RS232-Brücke (Serial2 auf P0.19/P0.20) |
| `Inhero_MR2_room_server` | Room Server |
| `Inhero_MR2_companion_radio_usb` | Companion Radio (USB, extra Filesystem) |
| `Inhero_MR2_terminal_chat` | Terminal Chat |
| `Inhero_MR2_sensor` | Sensor-Firmware |
| `Inhero_MR2_kiss_modem` | KISS Modem |

---

## Absolute Maximum Ratings

| Parameter | Min | Max | Einheit |
|---|---|---|---|
| Solar-Eingangsspannung (Voc) | — | 25 | V |
| Ladestrom (konfigurierbar) | 50 | 1500 | mA |
| Shunt-Strom (INA228, 100 mΩ) | — | 1600 | mA |
| Umgebungstemperatur (Betrieb) | –40 | +85 | °C |

---

## See Also

- [README.md](README.md) – Übersicht, Feature-Matrix und Diagnose
- [QUICK_START.md](QUICK_START.md) – Schnelleinstieg und CLI-Konfiguration
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) – Alle board-spezifischen CLI-Kommandos
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) – Vollständige technische Dokumentation
