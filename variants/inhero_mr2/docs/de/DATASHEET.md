# Inhero MR2 — Datenblatt

> **Inhero MR2 – Smart Solar Mesh Board**
> Hardware-Revision 1.1

> 🇬🇧 [English Version](../DATASHEET.md)

---

## Board-Übersicht

Das Inhero MR2 ist ein LoRa-Mesh-Repeater-Board auf Basis des **RAK4630**-Moduls (nRF52840 + SX1262) mit integriertem intelligentem Solar-Laderegler, Leistungsüberwachung und Tiefentladeschutz. Unterstützte Akku-Konfigurationen sind 1S-Li-Ion, 1S-LiFePO4 und 2S-LTO. Das Board wurde speziell für den autarken Langzeiteinsatz an abgelegenen oder schwer erreichbaren Standorten entwickelt. In Mitteleuropa ist ein ununterbrochener Repeater-Dauerbetrieb mit unverschatteten Solarmodulen ≥ 1 W und Akkukapazitäten ≥ 9 Ah möglich.

Die Lade- und Entladeschlussspannungen (siehe Tabelle [Unterstützte Akkuchemien](#unterstützte-akkuchemien)) sind so gewählt, dass die Akkus im Sommer nicht übermäßig belastet werden und gleichzeitig der Sleep-Betrieb bei Energiemangel sicher eingeleitet werden kann.

Im Low-Voltage-Sleep beträgt die Stromaufnahme < 500 µA. Sobald die Akkuspannung durch Solarladung wieder über die jeweilige Low-V-Wake-Schwelle (siehe Tabelle [Unterstützte Akkuchemien](#unterstützte-akkuchemien)) gestiegen ist, bootet das Board normal. Die 200 mV Hysterese zwischen Sleep- und Wake-Schwelle verhindert Motorboating – ein unkontrolliertes, schnelles Ein- und Ausschalten des Systems, das auftreten würde, wenn Sleep- und Wake-Schwelle zu dicht beieinander lägen.

### Sicherheits- und Schutzfunktionen

| Feature | Beschreibung |
|---------|-------------|
| **Watchdog Timer (WDT)** | Hardware-Watchdog des nRF52840. Startet das Board automatisch neu, wenn die Firmware hängt – wichtig für den unbeaufsichtigten Dauerbetrieb. |
| **Low-Voltage-Protection** | INA228 ALERT-Interrupt bei Unterschreitung der chemie-spezifischen Schwelle → kontrollierter System-Off mit RTC-Wake. Solarladung bleibt im Sleep aktiv (CE-Pin latched). |
| **Laderegler nur bei aktiver Firmware** | Der BQ25798 lädt ausschließlich, wenn die Firmware aktiv läuft. Ohne geflashte Firmware oder bei ausgeschaltetem 3.3V_off-Schalter bleibt die Ladung deaktiviert. Der nRF52840 muss als Host den Laderegler jederzeit überwachen können. |
| **JEITA-Temperaturschutz** | Temperaturabhängige Ladestromreduktion über den NTC-Sensor (TS-Pin). Frostladeschutz konfigurierbar per `set board.fmax`. Bei LTO ist JEITA deaktiviert. |

### Solar-Energiemanagement

| Feature | Beschreibung |
|---------|-------------|
| **MPPT (Maximum Power Point Tracking)** | Der BQ25798 optimiert die Solarernte per MPPT (VOC_PCT = 81,25 %, passend für kristalline Silizium-Solarzellen). Automatische Recovery bei Power-Good-Verlust und Stuck-PGOOD-Erkennung mit HIZ-Toggle. |
| **PFM Forward Mode** | Permanent aktiviert – verbessert die Effizienz bei niedrigen Solarströmen. |

### Technische Daten

| Parameter | Wert |
|---|---|
| **MCU** | nRF52840 (ARM Cortex-M4, 64 MHz) |
| **Funk** | Semtech SX1262 (via RAK4630) |
| **Frequenz** | LoRa Sub-GHz (regionsabhängig) |
| **Konnektivität** | LoRa, BLE 5.0, USB-C |
| **Versorgungsspannung** | 1S Li-Ion / 1S LiFePO4 / 2S LTO (per Firmware konfigurierbar) |
| **Solareingang** | 3,6 V – 24 V (MPPT) |
| **Max. Solar-Voc** | 25 V |
| **USB-Laden** | 5 V über USB-C (SS34-Diode auf VBUS-BQ, gleicher Ladepfad wie Solar) |
| **Laderegler** | BQ25798 (MPPT, JEITA) |
| **Max. Ladestrom** | 50 – 1500 mA (konfigurierbar) |
| **Leistungsmonitor** | INA228 (Coulomb Counter, ALERT) |
| **RTC** | RV-3028-C7 (Zeitbasis/Aufweck-Timer) |
| **Buck Converter** | TPS62840 (3.3V-Rail, max. 750 mA) |
| **System-Off-Strom** | via 3.3V off Switch ~15 µA |
| **System-Sleep-Strom** | < 500 µA (Firmware-Sleep mit GPIO-Latch, CE aktiv, RTC-Wake) |
| **Platinengröße** | 45 × 40 mm |
| **Montagebohrungen** | 4× M2.5, Lochabstand 40 × 35 mm |
| **Betriebstemperatur** | –40 °C bis +85 °C (MCU-Spezifikation) |
| **Bootloader** | Adafruit nRF52 OTA-Fix Bootloader (ab Werk), UF2-fähig |

---

## PCB – Vorderseite (Bestückungsseite)

![Inhero MR2 Vorderseite](../img/front.jpg)

![Inhero MR2 Vorderseite – Beschriftet](../img/front-annotated_.png)

### Anschlüsse, Taster & LEDs – Vorderseite

| Label (→ Bild) | Bezeichnung | Beschreibung |
|----------------|-------------|--------------|
| **Ble-Conn** | U.FL – BLE | Antennenanschluss für Bluetooth Low Energy (links oben am RAK4630) |
| **LoRa-Conn** | U.FL – LoRa | Antennenanschluss für LoRa Sub-GHz (links mittig am RAK4630) |
| **USB-C** | USB-C-Anschluss | USB-Schnittstelle für Stromversorgung, Laden, Firmware-Flash und CLI-Zugang (rechts oben). CC1/CC2 über 4,7 kΩ auf GND (USB-Sink). VBUS-USB ist über SS34-Schottky-Diode mit VBUS-BQ (Solareingang) verbunden — USB-Strom speist denselben Charger-Eingang wie das Solarpanel. |
| **Reset** | Reset-Taster | Einfachklick: Neustart des nRF52840. Doppelklick: USB-Mass-Storage-Mode für UF2-Firmware-Updates (rechts, unterhalb USB-C) |
| **Led 1+2** | Status-LEDs | LED1 + LED2 = RAK4630 User-LEDs (Heartbeat / Boot-Indikator, rechte Seite, übereinander) |
| **Chrg. Led** | Lade-LED | BQ25798 STAT-Ausgang – zeigt Ladezustand an (rechts unten, neben Solar-Connector) |
| **3.3V off** | Power-Schalter | Schiebeschalter zum Trennen der 3,3 V-Versorgung (links unten). **⚠ Achtung: Invertierte Logik!** Schalterstellung „ON" = EN-Pin auf Low = Board **aus**. Schalterstellung „OFF" = EN-Pin auf High = Board **ein**. |
| **Bat-Conn** (JST PH2.0-3P) | Batterie-Stecker | 3-poliger JST PH2.0 Stecker: **Batt+**, **Batt−**, **TS** (unten links) |
| **Solar-Conn** (JST PH2.0-2P) | Solar-Stecker | 2-poliger JST PH2.0 Stecker: **Solar+**, **Solar−** (unten rechts) |
| **Ø 2.5mm** | Montagebohrungen | 4× M2.5 Befestigungslöcher in den Ecken |

### Wichtige Bauteile – Vorderseite

| Bauteil | Bezeichnung | Beschreibung |
|---------|-------------|--------------|
| **RAK4630** | Core-Modul | nRF52840 SoC + SX1262 LoRa-Transceiver (Mitte, mit Abschirmung) |
| **BME280** | Umweltsensor | Temperatur, Luftfeuchtigkeit, Luftdruck |
| **BQ25798** | Akku-Laderegler | MPPT, JEITA-Temperaturschutz, 15-Bit-ADC |
| **INA228** | Leistungsmonitor | Coulomb Counter mit ALERT-Interrupt |
| **TPS62840** | Buck Converter | DC/DC, 750 mA, EN geschaltet über 3.3V_off Switch |

### Steckerbelegung – Batterie-Stecker (JST PH2.0-3P, von links nach rechts)

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | **Batt +** | Batterie-Pluspol |
| 2 | **Batt −** | Batterie-Minuspol (GND) |
| 3 | **TS** | Temperatursensor (NTC) für JEITA-Ladeschutz. Erforderlicher Typ: NCP15XH103F03RC (10 kΩ @ 25 °C, Beta 3380) oder kompatibel |

### Steckerbelegung – Solar-Stecker (JST PH2.0-2P, von links nach rechts)

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | **Solar +** | Solarpanel-Pluspol (3,6 V – 24 V, max. Voc 25 V) |
| 2 | **Solar −** | Solarpanel-Minuspol (GND) |

### USB-Ladepfad

USB-C VBUS ist über eine **SS34-Schottky-Diode** mit dem BQ25798-VBUS-Eingang (derselbe einzelne Eingang wie Solar) verbunden. Der BQ25798 hat nur einen VBUS-Eingang und unterscheidet nicht zwischen USB und Solar. CC1 und CC2 sind über 4,7 kΩ Widerstände auf GND gezogen und melden das Board als USB-Power-Sink (5 V Standard). Die SS34-Diode verhindert einen Rückfluss vom Solarpanel zum USB-Bus, allerdings **kann** Strom von USB-VBUS über den Solarstecker abfließen.

> **⚠ Warnung:** Da VBUS-USB und VBUS-BQ (Solareingang) über die SS34-Diode verbunden sind, führt ein **Kurzschluss am Solarstecker** auch zum Kurzschluss von VBUS-USB. Den Solareingang niemals kurzschließen, während USB angeschlossen ist.

---

## PCB – Rückseite

![Inhero MR2 Rückseite](../img/back.jpg)

![Inhero MR2 Rückseite – Beschriftet](../img/back-annotated_.png)

### Header & Pads – Rückseite

#### UART/I2C – Header-Reihe 1 (obere Reihe, Castellated Pads)

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | **GND** | Masse |
| 2 | **RX** | UART Receive |
| 3 | **TX** | UART Transmit |
| 4 | **SDA** | I2C Data |
| 5 | **SCL** | I2C Clock |
| 6 | **3.3V** | 3,3 V Ausgang (max. 500 mA, gemeinsam mit Board-Verbrauch) |

#### SWD – Header-Reihe 2 (untere Reihe, Castellated Pads)

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | **RESET** | nRF52840 Reset |
| 2 | **GND** | Masse |
| 3 | **SWCLK** | SWD Clock (Debug-Interface) |
| 4 | **SWDIO** | SWD Data (Debug-Interface) |
| 5 | **3.3V** | 3,3 V Ausgang (max. 500 mA, gemeinsam mit Board-Verbrauch) |

#### Lötbrücke – Onboard-Temperatursensor (unten rechts)

| Label (→ Bild) | Beschreibung |
|----------------|-------------|
| **Solder-Bridge** (close for onboard Temp-Sensor) | Lötbrücke für den Onboard-NTC-Temperatursensor (NCP15XH103F03RC, 10 kΩ @ 25 °C, Beta 3380). **Geschlossen** = Onboard-NTC aktiv. **Offen** = externer NTC vom Typ NCP15XH103F03RC (10 kΩ @ 25 °C, Beta 3380) oder kompatibel über TS-Pin des Batterie-Steckers erforderlich. |

---

## I2C-Bus – Adressübersicht

| Adresse | Bauteil | Funktion |
|---------|---------|----------|
| 0x40 | INA228 | Leistungsmonitor / Coulomb Counter |
| 0x52 | RV-3028-C7 | Echtzeituhr (RTC) |
| 0x6B | BQ25798 | Akku-Laderegler (MPPT, JEITA) |
| 0x76/0x77 | BME280 | Umweltsensor (T, H, P) |

---

## Pin-Zuordnung (wichtige GPIOs)

| GPIO | nRF52840 Pin | Funktion |
|------|-------------|----------|
| P0.04 | WB_IO4 | BQ CE-Pin (via DMN2004TK-7 N-FET, invertiert) |
| P1.02 | — | INA228 ALERT (Tiefentlade-Interrupt) |
| GPIO17 | WB_IO1 | RV-3028 RTC-Interrupt |
| GPIO21 | — | BQ25798 INT |

---

## Unterstützte Akkuchemien

| Typ | Nennspannung | Ladeendspannung | Low-V Sleep | Low-V Wake | Hysterese |
|-----|-------------|----------------|-------------|------------|-----------|
| **Li-Ion 1S** | 3,7 V | 4,2 V | 3100 mV | 3300 mV | 200 mV |
| **LiFePO4 1S** | 3,2 V | 3,6 V | 2700 mV | 2900 mV | 200 mV |
| **LTO 2S** | 4,6 V (2× 2,3 V) | 5,6 V | 3900 mV | 4100 mV | 200 mV |
| **none** | — | — | — | — | — |

---


## Absolute Maximalwerte

| Parameter | Min | Max | Einheit |
|---|---|---|---|
| Solar-Eingangsspannung (Voc) | — | 25 | V |
| Ladestrom (konfigurierbar) | 50 | 1500 | mA |
| Shunt-Strom (INA228, 100 mΩ) | — | 1600 | mA |
| Umgebungstemperatur (Betrieb) | –40 | +85 | °C |

---

## Siehe auch

- [README.md](README.md) – Übersicht, Feature-Matrix und Diagnose
- [QUICK_START.md](QUICK_START.md) – Schnelleinstieg und CLI-Konfiguration
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) – Alle board-spezifischen CLI-Kommandos
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) – Vollständige technische Dokumentation
