# Inhero MR-1 Datenblatt
## Solarbetriebener Mesh-Netzwerk-Knoten mit intelligentem Batteriemanagement

**Revision:** 2.0 (v0.2 Hardware)  
**Datum:** 31. Januar 2026  
**Hersteller:** Inhero GmbH  
**Website:** https://inhero.de

> **Hardware-Versionen:**  
> - **v0.1**: MCP4652 Digitalpotentiometer + TP2120 Komparator für UVLO (Legacy)  
> - **v0.2**: INA228 Power Monitor + RV-3028-C7 RTC für erweiterte Funktionen (Aktuell)  
> - **Auto-Detection**: Firmware erkennt Hardware-Version automatisch via I²C

---

## Inhaltsverzeichnis

1. [Produktübersicht](#produktübersicht)
2. [Hauptmerkmale](#hauptmerkmale)
3. [Technische Spezifikationen](#technische-spezifikationen)
4. [Blockdiagramm](#blockdiagramm)
5. [Pin-Belegung](#pin-belegung)
6. [Elektrische Eigenschaften](#elektrische-eigenschaften)
7. [Mechanische Abmessungen](#mechanische-abmessungen)
8. [Funktionsbeschreibung](#funktionsbeschreibung)
9. [Betriebsmodi](#betriebsmodi)
10. [Schnittstellen](#schnittstellen)
11. [Software-Konfiguration](#software-konfiguration)
12. [Anwendungsbeispiele](#anwendungsbeispiele)
13. [Sicherheitshinweise](#sicherheitshinweise)
14. [Bestellinformationen](#bestellinformationen)
15. [Zertifizierungen](#zertifizierungen)

---

## 1. Produktübersicht

Der **Inhero MR-1** ist ein hocheffizienter, solarbetriebener Mesh-Netzwerk-Knoten für LoRa-basierte Kommunikation. Das Board kombiniert modernste Nordic nRF52840 MCU mit einem SX1262 LoRa-Transceiver und einem intelligenten BQ25798 Batteriemanagement-System mit MPPT (Maximum Power Point Tracking). 

### Zielgruppen
- IoT-Entwickler für energieautonome Systeme
- Mesh-Netzwerk-Applikationen (Meshtastic, MeshCore)
- Umwelt- und Wetterüberwachung
- Smart Agriculture und Remote Sensing
- Off-Grid Kommunikationslösungen

> **Firmware-Hinweis:**  
> Die aktuelle Firmware ist ausschließlich für **MeshCore** entwickelt und getestet. Unterstützung für **Meshtastic** ist geplant und wird in zukünftigen Firmware-Versionen implementiert.

### Alleinstellungsmerkmale
- **Multi-Chemie-Unterstützung**: LTO 2S, LiFePO4 1S, Li-Ion 1S
- **Intelligentes MPPT**: Maximale Energieausbeute aus Solarpanels
- **Power Path Management**: Solarenergie wird auch bei Frostschutz direkt zum Betrieb genutzt (Akku wird geschont)
- **Temperaturgesteuertes Laden**: JEITA-konform mit NTC-Sensor
- **Coulomb Counter (v0.2)**: Echtzeit-SOC-Tracking mit INA228 Power Monitor
- **Daily Energy Balance (v0.2)**: 7-Tage-Analyse von Solar vs. Batterie
- **TTL Forecast (v0.2)**: Vorhersage der Batterie-Laufzeit
- **Hardware UVLO (v0.2)**: INA228 Alert → TPS62840 EN für ultimativen Schutz
- **RTC Wake-up (v0.2)**: RV-3028-C7 für periodische Wiederherstellung
- **Erweiterbar**: Multiple GPIO-Slots für Sensoren und Module
- **Open Source**: Vollständig dokumentierte Firmware

---

## 2. Hauptmerkmale

### Prozessor & Speicher
- **MCU**: Nordic Semiconductor nRF52840
  - ARM® Cortex®-M4F @ 64 MHz
  - 243 KB RAM
  - 796 KB Flash (815.104 Bytes verfügbar)
  - DSP-Befehle und FPU

### Drahtlose Kommunikation
- **LoRa-Modul**: RAK Wireless RAK4630
  - **Transceiver**: Semtech SX1262
  - **MCU**: Nordic nRF52840 (integriert im Modul)
  - Frequenzbänder: 150 MHz - 960 MHz (regional abhängig)
  - Leistung: +22 dBm (max)
  - Empfindlichkeit: -148 dBm
  - DIO2-basierte RF-Umschaltung
  - TCXO: 1.8V, integriert

- **Bluetooth**: nRF52840 (im RAK4630)
  - Bluetooth 5.3 / BLE
  - IEEE 802.15.4 (Thread, Zigbee fähig)

### Stromversorgung
- **Solarpanel-Eingang**: 3,6V - 24V DC
- **Batteriemanagement**: Texas Instruments BQ25798
  - MPPT mit automatischer Tracking-Steuerung
  - **Power Path Management**: Direkte Solarspeisung bei Frostschutz
  - Ladestrom (IBAT): Hardware bis 3000 mA, **Software-Limit: 1000 mA (1A)**
  - Eingangsstrom (IBUS): **Software-Limit: 1000 mA (1A)**
  - Multi-Chemie: LTO, LiFePO4, Li-Ion
  - 15-Bit ADC für Strom-/Spannungsmessung (bei niedrigen Strömen ungenau)
  - **Ab v0.2**: Ungenaue Strommessung wird durch hochpräzisen INA228 kompensiert
  - Hardware-Interrupts für Zustandsänderungen
  
- **Batterie-Überwachung (v0.2)**:
  - **INA228 Power Monitor** @ I²C 0x45
    - Echtzeit-Spannungsmessung (20-bit ADC)
    - Echtzeit-Strommessung (20mΩ Shunt, 1A max)
    - Coulomb Counter (Ladungsmessung in mAh)
    - Energiemessung (mWh)
    - Hardware-UVLO via Alert-Pin → TPS62840 EN
    - Shutdown-Modus (~1µA während SYSTEMOFF)
  - Temperaturüberwachung (NTC-Thermistor NCP15XH103F03RC)
    - **Primäre Funktion**: Frostschutz (Ladeverbot unter 0°C)
    - Misst Batterietemperatur für JEITA-konformes Laden
    - Genauigkeit außerhalb des Gefrierpunkts ist für Frostschutz unkritisch
    - **Umgebungstemperatur**: Wird durch on-board BME280-Sensor erfasst

### Power Management (v0.2)
- **INA228 Hardware UVLO**:
  - Alert-Pin steuert direkt TPS62840 Enable
  - Chemie-spezifische Schwellenwerte:
    - Li-Ion: 3.2V (Hardware), 3.4V (Software Dangerzone), 3.6V (Wake-up)
    - LiFePO4: 2.8V (Hardware), 2.9V (Software Dangerzone), 3.0V (Wake-up)
    - LTO 2S: 4.0V (Hardware), 4.2V (Software Dangerzone), 4.4V (Wake-up)
  - 3-Schichten Schutz: Software → Hardware Alert → Zero Power (0µA)

### Spannungsanpassung & Brownout-Schutz (v0.1 Legacy)
- **Digitales Potentiometer**: Microchip MCP4652
  - Dual-Kanal, 257 Stufen (8-bit)
  - I²C-Schnittstelle (0x2F)
  - Dynamische Spannungsanpassung für verschiedene Batteriechemien

- **Unterspannungsschutz**: TP2120 (Komparator)
  - Zellchemieabhängige Abschaltschwelle (konfigurierbar via MCP4652)
  - Wiedereinschalthysterese zur Motorboating-Prävention
  - Deaktiviert TPS62840DLCR Buck-Converter bei Unterspannung
  - Stromlose Schaltung des RAK4630 bei kritischem Ladezustand
  - Automatische Reaktivierung bei ausreichender Akkuladung

### Echtzeituhr (RTC)
- **RTC-Chip**: Micro Crystal RV-3028-C7
  - Hochpräzise: ±1 ppm Genauigkeit
  - 32.768 kHz Quarzoszillator (integriert)
  - I²C-Schnittstelle (0x52)
  - Batterie-Backup (optional)
  - Stromverbrauch: < 45 nA (Backup-Modus)
  - **INT-Pin**: GPIO17 (WB_IO1) für Wake-up

**Verwendungszweck**:
- ✅ **Zeit-Synchronisation**: Verhindert "Weglaufen" der Software-RTC im RAK4630
- ✅ **Nach Brownout**: RAK hat nach Wiedereinschaltung sofort die korrekte Zeit
- ✅ **Nach manuellem Ausschalten**: Zeit bleibt nach SW1-Betrieb erhalten
- ✅ **Hardware-Wake-ups (v0.2)**: 
  - Countdown-Timer für periodische Aufwachzyklen

### Umgebungssensor
- **BME280**: Bosch Sensortec BME280
  - Temperaturmessung: -40°C bis +85°C (±1°C Genauigkeit)
  - Luftfeuchtigkeitsmessung: 0-100% RH (±3% Genauigkeit)
  - Luftdruckmessung: 300-1100 hPa (±1 hPa Genauigkeit)
  - I²C-Schnittstelle (0x76)
  - Stromverbrauch: ~3.6 µA @ 1 Hz
  - **Verwendung**: Umgebungstemperatur, Wetterüberwachung, Höhenmessung
  - Wake from SYSTEMOFF (1-5µA Tiefschlaf)
  - Konfigurierbar: 1-18 Stunden Intervall
  - Anwendung: Low-Voltage Recovery, periodisches Monitoring

### Physische Schnittstellen
> **Hinweis:** Das Board hat **keine bestückten Erweiterungsheader**. Nur Debug- und Kommunikationspads auf der Unterseite.

**Verfügbare Pads (siehe PCB-Unterseite):**
- Debug-Schnittstelle (SWD): RESET, SWCLK, SWDIO, GND, 3.3V
- I²C1 + UART: GND, RX, TX, SDA (GPIO13), SCL (GPIO14), 3.3V

**Alle Pads im 2.54mm Raster** für Pogo-Pin-Programmierung.

### Indikatoren & Steuerung
- **LED1 (Grün)**: GPIO35 (Funktion nicht definiert, standardmäßig aus)
- **LED2 (Rot)**: GPIO36 (Funktion nicht definiert, standardmäßig aus)
- **STAT-LED (Rot)**: BQ25798 STAT-Pin (Ladestatus, aktuell aktiviert)
  - 🔋 **Funktion**: Zeigt Ladezustand der Batterie an
  - 💡 **Status**: Hardware-gesteuert durch BQ25798
- **Reset Button**: Hardware-Reset (RST, TS-1187F-1526)
- **SW1 (Manueller Schalter)**: RAK4630 Deaktivierung
  - 🛡️ **Zweck**: Manuelle Abschaltung des RAK-Moduls (z.B. für Antennenmontage)
  - ⚡ **Funktion**: Unterbricht Stromversorgung zum RAK4630
  - 🔄 **Wiedereinschalten**: RTC stellt sofort korrekte Zeit wieder her

### PCB-Unterseite: Lötpads & Debugging
> **Hinweis:** Auf der Unterseite des PCB sind spezielle Debug- und Kommunikations-Pads zugänglich.  
> **Rastermaß:** Alle Pads im **2.54mm (0.1") Standard-Raster** für Pogo-Pin-Programmierung.

#### Debug-Schnittstelle (SWD)
- **RESET**: Reset-Pin
- **SWCLK**: Serial Wire Clock
- **SWDIO**: Serial Wire Data I/O
- **GND**: Ground
- **3.3V**: Stromversorgung

**Verwendung**: Firmware-Debugging, Programmierung über SWD (z.B. mit J-Link, ST-Link)  
**Programmierung**: Pogo-Pin-Adapter kompatibel (2.54mm Rastermaß)

#### I²C1 + UART Kommunikation
- **GND**: Ground
- **RX**: UART Receive
- **TX**: UART Transmit
- **SDA**: I²C1 Data (GPIO13)
- **SCL**: I²C1 Clock (GPIO14)
- **3.3V**: Stromversorgung

**Verwendung**: Serielle Kommunikation, I²C-Sensoren, externe Module  
**Anschluss**: Pogo-Pin-Adapter oder Direktlötung möglich (2.54mm Rastermaß)

#### Lötbrücke JP1
- **JP1 (TS-Int-enable)**: On-Board NTC Aktivierung
  - 🔧 **Position**: Solder Bridge 2P (ungebrided/maskiert)
  - 🔥 **Funktion**: Aktiviert den On-Board NTC-Temperatursensor (NCP15XH103F03RC)
  - ⚠️ **Anwendung**: Ermöglicht Temperaturüberwachung bei Akkus ohne eigenen NTC-Sensor
  - 🖌️ **Aktivierung**: Lötbrücke schließen (standardmäßig offen)

---

## 3. Technische Spezifikationen

### Elektrische Eigenschaften

| Parameter | Min | Typ | Max | Einheit | Bedingung |
|-----------|-----|-----|-----|---------|-----------|
| **Stromversorgung** |
| Solar-Eingangsspannung | 3.6 | - | 24 | V | |
| Batteriespannung (Li-Ion 1S) | 3.0 (Voff) / 3.45 (Von) | 3.7 | 4.2 | V | Brownout-Schutz |
| Batteriespannung (LiFePO4 1S) | 2.9 (Voff) / 3.15 (Von) | 3.2 | 3.6 | V | Brownout-Schutz |
| Batteriespannung (LTO 2S) | 4.0 (Voff) / 4.3 (Von) | 5.0 | 5.6 | V | Brownout-Schutz |
| Ladestrom (IBAT) | 10 | 500 | 1000 | mA | Software-Limit (INA228 20mΩ max. 1A) |
| Eingangsstrom (IBUS) | - | 500 | 1000 | mA | Software-Limit (INA228 20mΩ max. 1A) |
| Ruhestrom (Sleep v0.2) | - | 5 | 10 | µA | INA228 Shutdown, LoRa Sleep |
| Ruhestrom (Sleep v0.1) | - | 10 | 50 | µA | BLE aus, LoRa Sleep |
| Betriebsstrom (RX) | - | 15 | 25 | mA | LoRa RX-Modus |
| Betriebsstrom (TX @22dBm) | - | 120 | 150 | mA | LoRa TX-Modus |
| **LoRa-Modul (RAK4630 mit SX1262)** |
| Frequenzbereich | 150 | - | 960 | MHz | Regional |
| Ausgangsleistung | -9 | +14 | +22 | dBm | Konfigurierbar |
| RX-Empfindlichkeit | - | -140 | -148 | dBm | SF12, BW125 |
| **Prozessor (nRF52840)** |
| CPU-Taktfrequenz | - | 64 | - | MHz | |
| RAM | - | 243 | - | KB | |
| Flash-Speicher | - | 796 | - | KB | User verfügbar |
| **Umgebungsbedingungen** |
| Betriebstemperatur | -20 | 25 | +60 | °C | |
| Lagertemperatur | -40 | - | +85 | °C | |
| Luftfeuchtigkeit | 0 | - | 95 | % | Nicht kondensierend |

### Speicherauslastung (Firmware Typical)

| Ressource | Verwendet | Verfügbar | Auslastung |
|-----------|-----------|-----------|------------|
| RAM | 28.772 Bytes | 248.832 Bytes | 11,6% |
| Flash | 302.080 Bytes | 815.104 Bytes | 37,1% |

---

## 4. Blockdiagramm

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Inhero MR-1 System                              │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌──────────────┐                                                       │
│  │ Solar Panel  │─────────┐                                             │
│  │  (3.6-24V)   │         │                                             │
│  └──────────────┘         │                                             │
│                           ▼                                             │
│  ┌──────────────┐   ┌──────────────────────┐                           │
│  │   Battery    │◄──►  BQ25798 Power Mgmt │                           │
│  │ (LTO/LiFePO4/│   │  ┌────────────────┐  │                           │
│  │   Li-Ion)    │   │  │ MPPT Charging  │  │                           │
│  └──────────────┘   │  │ 15-bit ADC/NTC │  │                           │
│                     │  └────────────────┘  │                           │
│                     └──────────┬───────────┘                           │
│                                │ VSYS (~VBAT + 100mV)                  │
│                                │                                       │
│                    ┌───────────┴───────────┐                           │
│                    │                       │                           │
│          ┌─────────▼────────────┐  ┌───────▼──────────────────┐       │
│          │  TPS7A0233 LDO       │  │  TPS62840DLCR Buck      │       │
│          │  3.3V Always-On      │  │  3.3V Switchable        │       │
│          │  ┌────────────────┐  │  │  ┌───────────────────┐  │       │
│          │  │ • RV-3028 RTC  │  │  │  │ Enable ◄─ TP2120  │  │       │
│          │  │ • MCP4652      │  │  │  │   (Brownout)      │  │       │
│          │  │ • TP2120       │  │  │  └───────────────────┘  │       │
│          │  └────────────────┘  │  └──────┬───────────────────┘       │
│          └──────────────────────┘         │ 3.3V                      │
│                                            │                           │
│                                ┌───────────▼──────────────┐            │
│                                │   RAK4630 LoRa Module    │            │
│                                │  ┌────────────────────┐  │   ┌────┐  │
│                                │  │ nRF52840 + SX1262  │  ├───►Ant │  │
│                                │  │ BLE 5.3 / LoRa     │  │   └────┘  │
│                                │  └────────────────────┘  │            │
│                                └──────────────────────────┘            │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 5. Pin-Belegung

### LoRa-Modul (RAK4630 mit SX1262)

| Funktionsname | GPIO | Richtung | Beschreibung |
|---------------|------|----------|--------------|
| P_LORA_DIO_1 | GPIO47 | Input | DIO1 Interrupt-Pin |
| P_LORA_NSS | GPIO42 | Output | SPI Chip Select |
| P_LORA_RESET | NC (GPIO38) | Output | Reset (nicht verwendet) |
| P_LORA_BUSY | GPIO46 | Input | Busy-Status |
| P_LORA_SCLK | GPIO43 | Output | SPI Clock |
| P_LORA_MISO | GPIO45 | Input | SPI Master In |
| P_LORA_MOSI | GPIO44 | Output | SPI Master Out |
| SX126X_POWER_EN | GPIO37 | Output | LoRa Power Enable |

**Hinweise:**
- DIO2 wird als RF-Switch verwendet (automatisch)
- DIO3 steuert den 1.8V TCXO

### Erweiterungs-Slots

#### SLOT A/B (Shared)
| Pin | GPIO | Beschreibung |
|-----|------|--------------|
| WB_IO1 | GPIO17 | General Purpose I/O (auch GPS 1PPS) |
| WB_IO2 | GPIO34 | General Purpose I/O |

#### SLOT C
| Pin | GPIO | Beschreibung |
|-----|------|--------------|
| WB_IO3 | GPIO21 | General Purpose I/O (BQ_INT_PIN) |
| WB_IO4 | GPIO4 | General Purpose I/O |

#### SLOT D
| Pin | GPIO | Beschreibung |
|-----|------|--------------|
| WB_IO5 | GPIO9 | General Purpose I/O |
| WB_IO6 | GPIO10 | General Purpose I/O |

### I²C-Busse

#### I²C Bus 1 (Sensor-Slot)
| Funktionsname | GPIO | Beschreibung |
|---------------|------|--------------|
| WB_I2C1_SDA | GPIO13 | I²C Data (GPS, Sensoren) |
| WB_I2C1_SCL | GPIO14 | I²C Clock |

#### I²C Bus 2 (IO-Slot)
| Funktionsname | GPIO | Beschreibung |
|---------------|------|--------------|
| WB_I2C2_SDA | GPIO24 | I²C Data (Expansion) |
| WB_I2C2_SCL | GPIO25 | I²C Clock |

### SPI-Bus (IO-Slot)

| Funktionsname | GPIO | Beschreibung |
|---------------|------|--------------|
| WB_SPI_CS | GPIO26 | Chip Select |
| WB_SPI_CLK | GPIO3 | SPI Clock |
| WB_SPI_MISO | GPIO29 | Master In Slave Out |
| WB_SPI_MOSI | GPIO30 | Master Out Slave In |

### Analog-Eingänge

| Pin | GPIO | Beschreibung |
|-----|------|--------------|
| WB_A0 | GPIO5 | Analog Input 0 (auch VBAT-Messung) |
| WB_A1 | GPIO31 | Analog Input 1 |

### Benutzerschnittstelle

| Funktionsname | GPIO | Typ | Beschreibung |
|---------------|------|-----|--------------|
| LED1 (Grün) | GPIO35 | Output | User-definierbar |
| LED2 (Rot) | GPIO36 | Output | User-definierbar |
| STAT-LED (Rot) | BQ25798 STAT | Output | Ladestatus (Hardware) |
| WB_SW1 | GPIO33 | Input | Interner GPIO (nicht als Button bestückt) |

### GPS (Optional)

| Funktionsname | GPIO/Adresse | Beschreibung |
|---------------|--------------|--------------|
| PIN_GPS_1PPS | GPIO17 | 1 Pulse Per Second |
| GPS_ADDRESS | 0x42 (I²C) | I²C-Bus-Adresse |
| Baudrate | 9600 | Seriell (alternativ) |

### Interne Sensoren

| Komponente | Schnittstelle | Adresse | Beschreibung | Hardware-Version |
|------------|---------------|---------|--------------|------------------|
| BQ25798 PMIC | I²C | 0x6B | Batteriemanagement | Alle |
| RV-3028-C7 RTC | I²C | 0x52 | Echtzeituhr | Alle |
| BME280 | I²C | 0x76 | Umgebungstemperatur, Luftfeuchtigkeit, Luftdruck | Alle |
| MCP4652 DigiPot | I²C | 0x2F | Spannungsanpassung (LEGACY) | **v0.1 nur** |
| TP2120 | Analog | - | Unterspannungsüberwachung (LEGACY) | **v0.1 nur** |
| INA228 | I²C | 0x45 | Power Monitor, Coulomb Counter | **v0.2 nur** |
| NTC-Thermistor | Analog (BQ25798) | - | Batterietemperatur | Alle |

---

## 6. Elektrische Eigenschaften

### 6.1 Stromversorgung

#### Solar-Eingang
- **Spannungsbereich**: 3,6V - 24V DC
- **Empfohlener Bereich**: 5V - 12V (optimal für MPPT)
- **Maximaler Strom**: 5A (BQ25798-Limit)
- **Schutz**: 
  - Verpolungsschutz
  - Überspannungsschutz (VINDPM)
  - Überstromschutz

#### Batterie-Ausgang
| Batterie-Typ | Zellen | Nominalspannung | Ladespannung Normal | Ladespannung Reduziert | Entladeschluss |
|--------------|--------|-----------------|---------------------|------------------------|----------------|
| Li-Ion | 1S | 3,7V | 4,20V | 4,05V | 2,8V |
| LiFePO4 | 1S | 3,2V | 3,60V | 3,45V | 2,5V |
| LTO | 2S | 5,0V | 5,60V | 5,40V | 3,8V |

**Reduzierte Ladespannung**: Verlängert Batterielebensdauer (empfohlen für 24/7-Betrieb)

#### NTC-Thermistor-Netzwerk
```
R_PULLUP (RT1)    = 5.600 Ω
R_PARALLEL (RT2)  = 27.000 Ω
R_NTC_25          = 10.000 Ω (NCP15XH103F03RC)
Beta-Wert         = 3.380
Temperaturoffset  = -2,5°C (Kalibrierung)
```

**Funktionszweck**: 
- **Primär**: Frostschutz (Erkennung von Temperaturen < 0°C für Ladeabschaltung)
- Misst Batterietemperatur für JEITA-konformes Laden
- Genauigkeit außerhalb des Gefrierpunkts ist für Frostschutzfunktion unkritisch
- **Umgebungstemperatur**: Wird präzise durch on-board BME280-Sensor gemessen

### 6.2 Stromaufnahme

| Betriebsmodus | Typisch | Maximum | Bedingung |
|---------------|---------|---------|-----------|
| Deep Sleep (v0.2) | 5 µA | 10 µA | CPU Sleep, INA228 Shutdown, LoRa Sleep |
| Deep Sleep (v0.1) | 10 µA | 50 µA | CPU Sleep, BLE aus, LoRa Sleep |
| CPU Aktiv (Idle) | 2 mA | 5 mA | CPU wach, kein LoRa |
| LoRa RX | 15 mA | 25 mA | Continuous RX-Modus |
| LoRa TX (+14dBm) | 45 mA | 60 mA | Mittlere Sendeleistung |
| LoRa TX (+22dBm) | 120 mA | 150 mA | Maximale Sendeleistung |
| BLE Advertising | 8 mA | 15 mA | BLE aktiv |
| Laden (Solar) | +500 mA | +1000 mA | Software-Limit IBAT (INA228 20mΩ max. 1A, ±0.5% Genauigkeit) |

**Hinweis**: Stromaufnahme kann durch Software-Konfiguration optimiert werden.

### 6.3 LoRa-Funkparameter

#### Frequenzbänder (Regional)
- **Europa**: 863-870 MHz (EU868)
- **Nordamerika**: 902-928 MHz (US915)
- **Asien**: 920-923 MHz (AS923)
- **Andere**: Konfigurierbar durch Software

#### Sendeleistung
| Einstellung | Leistung (dBm) | Stromverbrauch |
|-------------|----------------|----------------|
| Low | +2 | ~20 mA |
| Medium | +14 | ~45 mA |
| High | +22 | ~120 mA |

#### Empfindlichkeit
| Spreading Factor | Bandbreite | Empfindlichkeit |
|------------------|-----------|-----------------|
| SF7 | 125 kHz | -124 dBm |
| SF9 | 125 kHz | -134 dBm |
| SF12 | 125 kHz | -148 dBm |

---

## 7. Mechanische Abmessungen

### PCB-Abmessungen
- **Platinengröße**: 40,0 mm × 45,0 mm
- **PCB-Dicke**: 1,6 mm (Standard FR-4)
- **Lagen**: 4-lagig (Top Layer, Inner Layer 1, Inner Layer 2, Bottom Layer)
- **Material**: FR-4, Lead-Free HASL oder ENIG (je nach Bestellung)

### Montagelöcher
- **Anzahl**: 4× Ø 2,50 mm (durchkontaktiert)
- **Lochabstand**: 35,0 mm × 40,0 mm (Lochbild-Zentrierung)
- **Position**: An den vier Ecken der Platine
- **Verwendung**: M2.5 Schrauben oder Abstandsbolzen empfohlen

### Bohrlöcher (Komponenten)
- **Via-Durchmesser**: Ø 0,305 mm (innere Verbindungen)
- **Standard-Pads**: Ø 0,40 mm / Ø 0,50 mm (Komponenten-Anschlüsse)
- **Alignment-Pin**: 1× Ø 0,65 mm NPTH (nicht durchkontaktiert)

### Steckverbinder-Positionen
- **USB Type-C**: Mittig an Längsseite (24-Pin Connector)
- **Antennenanschluss**: U.FL/IPEX-Stecker für LoRa-Antenne
- **Erweiterungsleisten**: GPIO-Header (optional bestückbar)

### Bauhöhe
- **Maximale Höhe**: ca. 12 mm (mit USB-Buchse als höchstes Bauteil)
- **Flachbauteile**: < 3 mm (BME280-Sensor, ICs, Passives)
- **Elektrolytkondensator**: ca. 7 mm (220µF Panasonic 6TPE220MAZB)
- **Komponenten-Seite**: Top/Bottom
- **Kritische Abstände**: Antennen-Keep-Out-Zone beachten

---

## 8. Funktionsbeschreibung

### 8.1 Batteriemanagement-System (BQ25798)

Der Texas Instruments BQ25798 ist ein hochintegrierter Single-Chip-Batterielader mit integriertem MPPT-Controller.

#### MPPT (Maximum Power Point Tracking)
- **Funktion**: Optimiert automatisch die Solar-Panel-Spannung für maximale Leistungsausbeute
- **Algorithmus**: Periodische Leerlaufspannungsmessung (VOC) mit Arbeitspunktberechnung
- **Methode**: Der BQ25798 unterbricht in definierten Zeitabständen kurz den Ladevorgang, misst die Leerlaufspannung des Solar-Panels und berechnet daraus den optimalen Arbeitspunkt (typisch ~70-80% von VOC)
- **Update-Rate**: Hardware-gesteuert, konfigurierbar
- **Überwachung**: FreeRTOS-Task (`solarMpptTask`) mit 15-Minuten-Periode
- **Status-Tracking**: 7-Tage-Statistik mit Betriebszeit und Energiegewinnung

#### Ladefunktionen
- **Pre-Charge**: Trickle-Charge für tiefentladene Batterien
- **Fast-Charge**: Constant-Current (CC) Modus
- **Top-Off**: Constant-Voltage (CV) Modus
- **Termination**: Automatischer Ladeabschluss bei Vollladung

#### JEITA-Temperaturmanagement
| Temperaturbereich | Bezeichnung | Ladeverhalten |
|-------------------|-------------|---------------|
| < 0°C | COLD | Kein Laden (oder reduziert je nach Konfiguration) |
| 0°C - 10°C | COOL | Reduzierter Strom (20% / 40% je nach Einstellung) |
| 10°C - 45°C | NORMAL | Voller Ladestrom |
| 45°C - 55°C | WARM | Reduzierte Spannung |
| > 55°C | HOT | Laden gestoppt |

**Frost-Charge-Modus**:
- `0%`: Kein Laden unter 0°C (sicher für alle Batterien)
- `20%`: Ladestrom auf 20% reduziert bei COOL
- `40%`: Ladestrom auf 40% reduziert bei COOL
- `100%`: Keine Reduzierung (nur für spezielle LTO-Batterien)

> **⚡ Power Path Vorteil:**  
> Wenn das Laden bei Frost gestoppt wird, nutzt der BQ25798 die **Solarenergie direkt zur Speisung des RAK4630** (Power Path).  
> **Vorteil gegenüber anderen Boards**: Der Akku wird geschont und muss trotz verfügbarer Solarenergie nicht entladen werden.

> **⚠️ Known Issue - NTC-Kalibrierung:**  
> Der NTC-Sensor (NCP15XH103F03RC) ist für den **JEITA-Schutz um den Gefrierpunkt (-10°C bis +10°C)** optimiert kalibriert.  
> **Außerhalb dieses Bereichs** (>10°C oder <-10°C) kann eine **Temperaturdrift von mehreren Grad** auftreten.  
> **Auswirkung**: JEITA-Schutz funktioniert zuverlässig im kritischen Temperaturbereich, aber Telemetriewerte können bei extremeren Temperaturen ungenau sein.

#### ADC-Messungen (15-Bit-Auflösung)
- Solar-Spannung (±10mV Genauigkeit)
- Solar-Strom (±27mA Offset-Kalibrierung)
- Batterie-Spannung
- Batterie-Strom (Laden/Entladen)
- Batterie-Temperatur (NTC-Thermistor, kalibriert für -10°C bis +10°C)

### 8.2 Multi-Chemie-Unterstützung

#### Hardware v0.1 (Legacy): MCP4652 Digitales Potentiometer & TP2120 Brownout-Schutz
> **⚠️ Nur Hardware v0.1**: Diese Komponenten werden in v0.2 durch INA228 Power Monitor ersetzt.

Das Dual-Channel Digital Potentiometer ermöglicht dynamische Spannungsanpassungen für verschiedene Batteriechemien ohne Hardware-Modifikation.

**Funktionsweise (v0.1)**:
1. Firmware erkennt Batterietyp aus Konfiguration
2. MCP4652 stellt die entsprechenden Wiper-Werte ein:
   - **Kanal 1**: BQ25798 Ladespannungsanpassung
   - **Kanal 2**: TP2120 Unterspannungsschwelle und Hysterese
3. Einstellungen werden in Flash gespeichert

**Wiper-Auflösung**: 257 Stufen (0-256), 8-Bit-Präzision

#### Zellchemieabhängiger Brownout-Schutz
**Hardware v0.1 (Legacy)**: TP2120 Komparator arbeitet mit dem MCP4652 zusammen, um einen intelligenten Brownout-Schutz zu realisieren.

**Funktionsprinzip (v0.1)**:
1. **Unterspannung erkannt**: TP2120 deaktiviert den TPS62840DLCR Buck-Converter
2. **RAK4630 stromlos**: LoRa-Modul wird komplett abgeschaltet
3. **Board arbeitet weiter**: BQ25798 lädt Batterie über Solarpanel
4. **Hysterese**: Wiedereinschaltung erst bei ausreichender Ladung über Schwelle
5. **Automatische Reaktivierung**: RAK4630 startet neu, wenn Spannung stabil ist

**Hardware v0.2 (Aktuell)**: INA228 Power Monitor bietet verbesserten Schutz:
- **Echtzeit-Strommessung**: 20mΩ Shunt, max. **1000mA** (1A)
- **Hardware-UVLO**: Alert-Pin steuert direkt TPS62840 Enable
- **Chemie-spezifische Schwellen**: Li-Ion 3.2V, LiFePO4 2.8V, LTO 4.0V
- **3-Schichten-Schutz**: Software → Hardware Alert → Zero Power (0µA)
- **Coulomb Counter**: Präzise SOC-Tracking, Daily Energy Balance, TTL Forecast

**Vorteile**:
- ✅ **Verhindert Motorboating**: Kein zyklisches Ein-/Ausschalten bei niedrigem Ladezustand
- ✅ **Batterieschonung**: Tiefentladung wird sicher vermieden
- ✅ **Zuverlässiger Solarbetrieb**: Typisches Problem bei LoRa-Repeatern gelöst
- ✅ **Chemie-spezifisch**: Angepasste Schwellwerte für LTO/LiFePO4/Li-Ion

#### Unterstützte Batterie-Typen

##### 1. Lithium-Ionen (Li-Ion 1S)
- **Chemie**: LiCoO₂, LiNMC, LiNCA
- **Nennspannung**: 3,7V
- **Ladespannung**: 4,20V (Normal), 4,05V (Reduziert)
- **Entladeschluss**: 2,8V
- **Empfohlener Ladestrom**: 0,5C - 1C
- **Anwendung**: Standard IoT-Geräte, hohe Energiedichte

##### 2. Lithium-Eisenphosphat (LiFePO4 1S)
- **Chemie**: LiFePO₄
- **Nennspannung**: 3,2V
- **Ladespannung**: 3,60V (Normal), 3,45V (Reduziert)
- **Entladeschluss**: 2,5V
- **Empfohlener Ladestrom**: 0,5C - 1C
- **Anwendung**: Langlebigkeit, sicher, geringe Temperaturdrift

##### 3. Lithium-Titanat (LTO 2S)
- **Chemie**: Li₄Ti₅O₁₂ (2 Zellen in Serie)
- **Nennspannung**: 5,0V (2 × 2,5V)
- **Ladespannung**: 5,60V (Normal), 5,40V (Reduziert)
- **Entladeschluss**: 3,8V
- **Empfohlener Ladestrom**: 1C - 3C (schnelles Laden möglich)
- **Anwendung**: Extreme Temperaturen (-30°C bis +60°C), 20.000+ Zyklen

### 8.3 Telemetrie-System

Das Board exportiert Telemetriedaten im **CayenneLPP**-Format über zwei Kanäle:

#### Kanal N: Solar-Eingang
| Parameter | Datentyp | Einheit | Beschreibung |
|-----------|----------|---------|--------------|
| Voltage | Analog Input | Volt (V) | Solar-Panel-Spannung |
| Current | Analog Input | Ampere (A) | Solar-Panel-Strom |
| Power | Analog Output | Watt (W) | V × I |
| MPPT Status | Digital Input | Boolean | MPPT aktiv (1) / inaktiv (0) |

#### Kanal N+1: Batterie
| Parameter | Datentyp | Einheit | Beschreibung |
|-----------|----------|---------|--------------|
| Voltage | Analog Input | Volt (V) | Batterie-Spannung |
| Current | Analog Input | Ampere (A) | Lade-(+) / Entladestrom (-) |
| Power | Analog Output | Watt (W) | V × I |
| Temperature | Temperature | °C | Batterie-Temperatur (NTC) |

**Übertragung**: Integriert in MeshCore-Telemetrie-Pakete

> **⚠️ Known Issue - Strommessung-Genauigkeit (v0.1 Hardware):**  
> Die ADC-Strommessungen des BQ25798 können bei niedrigen Strömen ungenau sein (Abweichung bis 100%).  
> **Ursache**: Der BQ25798 ist für höhere Ströme (bis 3A+) optimiert, arbeitet aber typischerweise bei:
> - ISYS: 13-200 mA (Peak)
> - IBUS/IBAT: ~200 mA (typisch)
> 
> **Auswirkung**: Strom-Telemetriewerte (Solar-Strom, Lade-/Entladestrom) können signifikant von tatsächlichen Werten abweichen.  
> **Lösung ab v0.2**: INA228 Power Monitor mit 20mΩ Shunt liefert hochpräzise Strommessung (±0.5% Genauigkeit) und ersetzt BQ25798-ADC für Batteriestrom. Coulomb Counter ermöglicht akkurate SOC-Berechnung.

### 8.4 FreeRTOS-Architektur

#### Solar MPPT Task
```c
Task-Name:      solarMpptTask
Priorität:      1 (Low)
Stack-Größe:    4096 Bytes
Trigger:        Semaphore-basiert (Interrupt) + 15-Min-Timeout
```

**Ablauf**:
1. Task wartet auf Semaphore (Solar-Interrupt) oder 15-Minuten-Timeout
2. BQ25798-Status wird gelesen (Power Good, MPPT-Status)
3. Solar- und Batterie-Telemetrie wird erfasst
4. MPPT-Statistiken werden aktualisiert (7-Tage-Rolling-Buffer)
5. Energiegewinnung wird berechnet (E = P × Δt)
6. Bei Solar-Event: Blaues LED blinkt
7. Bei Chip-Störung: Automatischer MPPT-Neustart

#### Interrupt-System
- **BQ25798 INT-Pin**: GPIO21
- **Interrupt-Quellen**: 
  - Power Good Status-Änderung
  - Charger Status-Änderung (optional)
- **Handler**: Gibt `solarEventSem` frei → Weckt solarMpptTask

---

## 9. Betriebsmodi

### 9.1 Solar-Betrieb (Autonomous)
- **Beschreibung**: Kontinuierlicher Betrieb mit Solar-Panel und Batterie
- **Stromquelle**: Solar → Batterie → System
- **MPPT**: Aktiv (empfohlen)
- **Energiemanagement**: Automatisch

**Typische Konfiguration**:
```bash
set board.bat lifepo1s
set board.mppt true
set board.imax 500
set board.frost 20%
set board.life true
```

### 9.2 Nur-Batterie-Betrieb
- **Beschreibung**: Betrieb ohne Solar-Panel
- **Stromquelle**: Batterie → System
- **MPPT**: Inaktiv
- **Energiemanagement**: Manuell

### 9.3 USB-Betrieb (Entwicklung)
- **Beschreibung**: Stromversorgung über USB
- **Stromquelle**: USB 5V → System
- **MPPT**: Nicht relevant
- **Verwendung**: Firmware-Entwicklung, Debugging

### 9.4 Low-Power-Modus
- **Beschreibung**: Minimale Stromaufnahme
- **CPU**: Sleep-Modus
- **LoRa**: RX-Modus mit preamble detection
- **BLE**: Aus
- **Stromverbrauch**: ~10-50 µA

---

## 10. Schnittstellen

### 10.1 CLI (Command Line Interface)

Das Board bietet eine umfangreiche CLI über serielle Schnittstelle (USB/UART).

#### Get-Befehle (Lesezugriff)
| Befehl | Rückgabe | Beschreibung |
|--------|----------|--------------|
| `board.bat` | String | Aktueller Batterietyp (lto2s / lifepo1s / liion1s) |
| `board.frost` | String | Frost-Charge-Verhalten (0% / 20% / 40% / 100%) |
| `board.life` | Boolean | Reduzierte Ladespannung (true/false) |
| `board.imax` | Integer | Maximaler Ladestrom (mA) |
| `board.mppt` | Boolean | MPPT-Status (true/false) |
| `board.mpptstat` | String | 7-Tage-MPPT-Statistik und 3-Tage-Energie |
| `board.info` | String | Charger-Status (Power Good, Charging State) |
| `board.tele` | JSON | Vollständige Telemetrie-Momentaufnahme |

#### Set-Befehle (Schreibzugriff)
| Befehl | Parameter | Beschreibung |
|--------|-----------|--------------|
| `set board.bat <typ>` | lto2s / lifepo1s / liion1s | Batterietyp festlegen |
| `set board.frost <verhalten>` | 0% / 20% / 40% / 100% | Frost-Charge-Modus |
| `set board.life <flag>` | true / false | Reduzierte Spannung aktivieren |
| `set board.imax <strom>` | 1-1000 (mA) | Maximalen Ladestrom setzen (Software-Limit, max. 1000mA) |
| `set board.mppt <flag>` | true / false | MPPT aktivieren/deaktivieren |

**Beispiel-Ausgabe**:
```bash
> board.tele
Solar: 5.23V, 287mA, 1.50W, MPPT: ON
Battery: 4.08V, 312mA (charging), 1.27W, 23.5°C
Status: Power Good, Fast Charging

> board.mpptstat
MPPT 7d avg: 87.3%, E_daily 3d: 1250mWh (3.0d data)
```

### 10.2 I²C-Schnittstellen

#### I²C Bus 1 (Primary - Sensoren/GPS)
- **Pins**: SDA=GPIO13, SCL=GPIO14
- **Taktfrequenz**: 100 kHz (Standard) / 400 kHz (Fast)
- **Pull-Ups**: On-Board 4,7 kΩ
- **Geräte**:
  - GPS-Modul (0x42)
  - Externe Sensoren (BME280, etc.)

#### I²C Bus 2 (Secondary - Expansion)
- **Pins**: SDA=GPIO24, SCL=GPIO25
- **Taktfrequenz**: 100 kHz (Standard) / 400 kHz (Fast)
- **Pull-Ups**: On-Board 4,7 kΩ
- **Geräte**: Erweiterungsmodule

#### Interne I²C-Geräte (Shared Bus)
| Gerät | Adresse | Bus | Beschreibung | Hardware-Version |
|-------|---------|-----|--------------|------------------|
| BQ25798 | 0x6B | Internal | Battery Charger | Alle |
| RV-3028-C7 | 0x52 | Internal | Real-Time Clock | Alle |
| MCP4652 | 0x2F | Internal | Digital Potentiometer | **v0.1 nur** |
| INA228 | 0x45 | Internal | Power Monitor (20mΩ, 1A max) | **v0.2 nur** |

### 10.3 SPI-Schnittstelle

- **Pins**: CS=GPIO26, CLK=GPIO3, MISO=GPIO29, MOSI=GPIO30
- **Taktfrequenz**: Bis zu 8 MHz
- **Modus**: Mode 0 (CPOL=0, CPHA=0)
- **Verwendung**: Externe Speicher, Displays, Sensoren

**Hinweis**: SX1262 LoRa-Transceiver nutzt eigenen dedizierten SPI-Bus.

### 10.4 UART-Schnittstellen

#### USB-CDC (Primary)
- **Schnittstelle**: USB 2.0 Full-Speed
- **Baudrate**: Automatisch (USB-Device)
- **Verwendung**: Firmware-Upload, CLI, Debugging

#### GPS-UART (Optional)
- **Pins**: TX=GPIO16, RX=GPIO15
- **Baudrate**: 9600 bps (Standard)
- **Protokoll**: NMEA 0183

### 10.5 GPIO-Schnittstellen

Alle GPIO-Pins sind 3,3V-Logikpegel. **Wichtig**: Keine 5V-toleranten Eingänge!

#### Konfigurierbare Modi
- Digital Input (mit/ohne Pull-Up/Pull-Down)
- Digital Output (Push-Pull)
- Analog Input (12-bit ADC, 0-3,3V)
- PWM-Output (Timer-basiert)
- Interrupt-Eingang (Rising/Falling/Both Edge)

---

## 11. Software-Konfiguration

> **Wichtig:**  
> Das Inhero MR-1 Board wird derzeit ausschließlich mit **MeshCore Firmware** unterstützt. Die Integration für **Meshtastic Firmware** ist in Entwicklung und wird zu einem späteren Zeitpunkt verfügbar sein.

### 11.1 Build-Umgebung

#### PlatformIO-Konfiguration
```ini
[env:Inhero_MR1_repeater]
platform = nordicnrf52 @ 10.10.0
board = inhero_mr1
framework = arduino

upload_protocol = nrfutil
upload_port = COM[X]  ; Automatisch erkannt

build_flags = 
    -DINHERO_MR1
    -DARDUINO_NRF52840_FEATHER
    -DNRF52840_XXAA
    -DS140
    
lib_deps = 
    RadioLib @ 7.5.0
    Adafruit BQ25798
    CayenneLPP @ 1.6.1
    ArduinoJson @ 7.4.2
```

#### Firmware-Upload
```bash
# Via PlatformIO CLI
pio run -e Inhero_MR1_repeater -t upload

# Via Task (VS Code)
Task: "Build Inhero MR1"
```

### 11.2 Bootloader & Firmware-Update

#### Adafruit nRF52 Bootloader
- **Version**: 0.7.0+
- **Features**: 
  - USB-CDC-Unterstützung
  - UF2-Dateiformat
  - Doppelklick-Reset für Bootloader-Modus

#### UF2-Update-Prozess
1. Doppelklick auf Reset-Button (falls vorhanden) oder programmatisch
2. Board erscheint als USB-Massenspeicher (`NRF52BOOT`)
3. UF2-Datei auf Laufwerk kopieren
4. Board bootet automatisch mit neuer Firmware

#### Over-The-Air (OTA) Update
- **Unterstützt**: Ja (via `board.startOTAUpdate()`)
- **Protokoll**: Nordic DFU über BLE
- **Partition-Schema**: Dual-Bank (Application + Update)

### 11.3 Persistente Einstellungen

#### LittleFS-Dateisystem
- **Partition**: 0x3C000 - 0xF4000 (752 KB)
- **Namespace**: `inhero_mr1`
- **Library**: SimplePreferences (Wrapper)

#### Gespeicherte Parameter
| Schlüssel | Typ | Default | Beschreibung |
|-----------|-----|---------|--------------|
| `battery_type` | uint8_t | 3 (LIION_1S) | Batterie-Chemie |
| `frost_behavior` | uint8_t | 4 (NO_CHARGE) | Frost-Charge-Modus |
| `max_charge_current_ma` | uint16_t | 200 | Ladestrom (mA) |
| `reduced_voltage` | bool | false | Reduzierte Spannung |
| `mppt_enabled` | bool | true | MPPT-Status |

**Zugriff via Code**:
```cpp
#include "lib/SimplePreferences.h"

SimplePreferences prefs("inhero_mr1");
uint8_t batType = prefs.getUChar("battery_type", LIION_1S);
prefs.putUChar("battery_type", LTO_2S);
```

### 11.4 Debug & Logging

#### Serial-Logging
```cpp
Serial.begin(115200);
Serial.println("Inhero MR-1 Initialized");
```

#### Log-Level
- `DEBUG`: Detaillierte Diagnoseinformationen
- `INFO`: Allgemeine Informationen
- `WARNING`: Warnungen (z.B. Temperatur außerhalb Bereich)
- `ERROR`: Fehler (z.B. I²C-Kommunikation fehlgeschlagen)

---

## 12. Anwendungsbeispiele

### 12.1 Solar-Repeater-Knoten

**Beschreibung**: Autonomer LoRa-Mesh-Repeater mit Solar-Power.

**Hardware**:
- Inhero MR-1 Board
- 6V / 1W Solar-Panel
- LiFePO4 1S / 1500mAh Batterie
- LoRa-Antenne 868 MHz (3 dBi)

**Konfiguration**:
```bash
set board.bat lifepo1s
set board.frost 20%
set board.life true
set board.imax 300
set board.mppt true
```

**Erwartete Laufzeit**: 
- Solare Energiegewinnung: ~1000-1500 mWh/Tag (bei 4h Sonnenlicht)
- Durchschnittlicher Verbrauch: ~600 mWh/Tag (LoRa RX + gelegentlich TX)
- **Autonomie**: Unbegrenzt bei ausreichend Sonnenlicht

### 12.2 Wetterstation mit BME280

**Beschreibung**: Solarbetriebene Wetterstation mit Temperatur, Luftfeuchtigkeit, Luftdruck.

**Hardware**:
- Inhero MR-1 Board
- BME280 Sensor (I²C Bus 1)
- 6V / 2W Solar-Panel
- Li-Ion 1S / 2000mAh Batterie

**Firmware**:
```cpp
#include <Adafruit_BME280.h>
#include <CayenneLPP.h>

Adafruit_BME280 bme;
CayenneLPP lpp(64);

void setup() {
  Wire.begin();
  bme.begin(0x76, &Wire);
}

void loop() {
  float temp = bme.readTemperature();
  float hum = bme.readHumidity();
  float press = bme.readPressure() / 100.0F;
  
  lpp.reset();
  lpp.addTemperature(1, temp);
  lpp.addRelativeHumidity(2, hum);
  lpp.addBarometricPressure(3, press);
  
  // Über LoRa senden
  mesh.sendTelemetry(lpp.getBuffer(), lpp.getSize());
  
  // Sleep 10 Minuten
  delay(600000);
}
```

**Konfiguration**:
```bash
set board.bat liion1s
set board.frost 40%
set board.imax 400
```

### 12.3 GPS-Tracker

**Beschreibung**: Mobiler GPS-Tracker mit LoRa-Übertragung.

**Hardware**:
- Inhero MR-1 Board
- GPS-Modul (I²C, Adresse 0x42)
- Li-Ion 1S / 1000mAh Batterie
- Optional: 5V / 0,5W Solar-Panel

**Features**:
- 1PPS-Signal für präzises Timing (GPIO17)
- Position alle 5 Minuten
- Low-Power zwischen Messungen

### 12.4 Kaltwetter-Betrieb

**Beschreibung**: Outdoor-Sensor in kalten Umgebungen (-20°C bis +10°C).

**Hardware**:
- Inhero MR-1 Board
- LTO 2S / 1000mAh Batterie (kälteresistent)
- 12V / 3W Solar-Panel
- Isoliertes Gehäuse

**Konfiguration**:
```bash
set board.bat lto2s
set board.frost 100%     # LTO erlaubt Laden bei Kälte
set board.imax 1000      # LTO kann höhere Ströme
set board.life false     # Volle Spannung
```

**Vorteile von LTO**:
- Funktioniert von -30°C bis +60°C
- Schnelles Laden möglich
- 20.000+ Zyklen Lebensdauer
- Keine Degradation bei Kälte

---

## 13. Sicherheitshinweise

### 13.1 Elektrische Sicherheit

#### ⚠️ Spannungsgrenzen
- **Solar-Eingang**: Maximale Spannung 24V DC. Überspannung kann Schäden verursachen.
- **GPIO**: Alle Pins sind **3,3V-Logikpegel**. Keine 5V anlegen!
- **Batterie**: Richtige Polung beachten. Verpolungsschutz vorhanden, aber Vorsicht geboten.

#### ⚠️ Strombegrenzung
- **Ladestrom (v0.2)**: Software-limitiert auf **1000mA (1A)** durch INA228 20mΩ Shunt. Hardware (BQ25798) unterstützt bis 5000mA.
- **Ladestrom (v0.1)**: Software-limitiert auf 3000mA (3A). Hardware (BQ25798) unterstützt bis 5000mA.
- **GPIO-Strom**: Maximaler Strom pro Pin: 15 mA. Gesamtstrom: 200 mA.
- **Solar-Panel**: Kurzschlussstrom sollte < 5A sein.

#### ⚠️ ESD-Schutz
- Board ist ESD-empfindlich (MIL-STD-883, Klasse 1A)
- ESD-Schutzmaßnahmen bei Handhabung beachten
- Antistatisches Armband empfohlen

### 13.2 Thermisches Management

#### Betriebstemperatur
- **Normal**: -20°C bis +60°C
- **Kritisch**: > +60°C → Automatische Abschaltung des Ladens
- **Unterkühlt**: < 0°C → Frost-Charge-Modus aktiv

#### Kühlung
- **Natural Convection**: Ausreichend für normale Betriebsbedingungen
- **Forced Airflow**: Empfohlen bei:
  - Hohem Ladestrom (> 2A)
  - Hoher Umgebungstemperatur (> 40°C)
  - Kontinuierlicher TX-Betrieb bei +22 dBm

#### Kritische Komponenten
- **BQ25798**: Kann bei hohem Ladestrom warm werden (bis 85°C intern ist normal)
- **nRF52840**: Bei kontinuierlichem CPU-Betrieb auf Temperatur achten

### 13.3 Batterie-Sicherheit

#### ⚠️ Batterietyp-Kompatibilität
- **IMMER** den korrekten Batterietyp konfigurieren (`set board.bat`)
- Falsche Einstellung kann zu Über-/Unterladung führen
- Bei Unsicherheit: Hersteller-Datenblatt prüfen

#### ⚠️ Lithium-Batterien
- **Li-Ion**: Nicht über 4,2V laden (Explosionsgefahr)
- **LiFePO4**: Nicht über 3,6V laden
- **LTO**: Nicht über 5,6V laden (2S)

#### ⚠️ Temperaturüberwachung
- NTC-Sensor muss thermischen Kontakt zur Batterie haben
- Bei fehlerhaftem Sensor: Laden wird unsicher
- Regelmäßig `board.tele` prüfen für Temperaturwerte

#### ⚠️ Mechanische Sicherheit
- Batterie vor mechanischer Beschädigung schützen
- Keine scharfen Gegenstände in der Nähe
- Gehäuse mit Belüftung verwenden

### 13.4 Antennen-Sicherheit

#### ⚠️ LoRa-Antenne
- **NIEMALS** ohne Antenne senden!
- SX1262 kann beschädigt werden durch Reflexion
- Antenne vor Einschalten anbringen
- SWR < 2:1 empfohlen

#### Antennen-Platzierung
- Mindestabstand zu Körper: 20 cm (bei +22 dBm)
- Keep-Out-Zone auf PCB beachten (keine Metallteile)
- Ground Plane für optimale Performance

### 13.5 Software-Sicherheit

#### ⚠️ Firmware-Updates
- Nur offizielle Firmware von Inhero GmbH oder verifizierten Quellen
- Backup der Konfiguration vor Update
- Nicht während Update-Prozess unterbrechen (Bricken möglich)

#### ⚠️ Konfigurationsänderungen
- Parameter außerhalb erlaubter Bereiche werden abgewiesen
- Nach Änderungen Telemetrie prüfen (`board.tele`)
- **Strommesswerte**: Bei niedrigen Strömen (<500mA) können ADC-Werte um bis zu 100% abweichen
- Dokumentation änderungen in Log führen

---

## 14. Bestellinformationen

### 14.1 Teilenummern

> **Hinweis**: Dieser Abschnitt sollte mit Ihren tatsächlichen Teilenummern ausgefüllt werden.

| Teilenummer | Beschreibung | Lieferumfang |
|-------------|--------------|--------------|
| INHERO-MR1-BASE | Inhero MR-1 Board (ohne Batterie/Solar) | PCB, LoRa-Antenne |
| INHERO-MR1-KIT-LIION | Starter-Kit Li-Ion | Board, 2000mAh Batterie, 6V/2W Solar |
| INHERO-MR1-KIT-LIFEPO | Starter-Kit LiFePO4 | Board, 1500mAh Batterie, 6V/2W Solar |
| INHERO-MR1-KIT-LTO | Starter-Kit LTO | Board, 1000mAh 2S Batterie, 12V/3W Solar |

### 14.2 Zubehör

| Artikel | Beschreibung | Kompatibilität |
|---------|--------------|----------------|
| INHERO-ANT-868-3DBI | 868 MHz LoRa-Antenne (3 dBi) | Europa |
| INHERO-ANT-915-3DBI | 915 MHz LoRa-Antenne (3 dBi) | USA |
| INHERO-SOLAR-6V2W | Solar-Panel 6V / 2W | Alle Kits |
| INHERO-SOLAR-12V3W | Solar-Panel 12V / 3W | LTO-Kit |
| INHERO-BAT-LIION-2000 | Li-Ion 1S 2000mAh | MR-1 Board |
| INHERO-BAT-LIFEPO-1500 | LiFePO4 1S 1500mAh | MR-1 Board |
| INHERO-BAT-LTO-1000-2S | LTO 2S 1000mAh | MR-1 Board |
| INHERO-CASE-OUTDOOR | IP65 Outdoor-Gehäuse | MR-1 Board |
| INHERO-GPS-MODULE | GPS-Modul I²C | Alle Kits |

### 14.3 Regionale Varianten

| Region | Frequenzband | Teilenummer-Suffix | Antennen |
|--------|--------------|-------------------|----------|
| Europa | 863-870 MHz | -EU | 868 MHz |
| USA/Kanada | 902-928 MHz | -US | 915 MHz |
| Asien | 920-923 MHz | -AS | 923 MHz |
| Global | Konfigurierbar | -GL | Kundenspezifisch |

**Hinweis**: Firmware ist identisch, nur Antennen und Standard-Konfiguration variieren.

---

## 15. Zertifizierungen

> **Hinweis**: Dieser Abschnitt sollte mit tatsächlichen Zertifizierungen ausgefüllt werden.

### 15.1 Geplante/Erforderliche Zertifizierungen

#### Europa (CE)
- **RED 2014/53/EU**: Radio Equipment Directive
  - EN 300 220 (LoRa, Short Range Devices)
  - EN 301 489 (EMC für Funkgeräte)
  - EN 62368-1 (Sicherheit)
  
- **RoHS 2011/65/EU**: Restriction of Hazardous Substances
- **WEEE 2012/19/EU**: Elektroschrott-Richtlinie

#### USA (FCC)
- **FCC Part 15, Subpart C**: LoRa-Transmitter
- **FCC Part 15, Subpart B**: EMI/EMC

#### International
- **Bluetooth SIG**: Bluetooth 5.3 Qualified Design
- **LoRa Alliance**: Certified LoRaWAN® Device (optional)

### 15.2 Konformitätserklärung

> Die finale Konformitätserklärung sollte von einem akkreditierten Testlabor erstellt werden.

**Erklärung**:  
Inhero GmbH erklärt, dass der Inhero MR-1 den grundlegenden Anforderungen und anderen relevanten Bestimmungen der Richtlinie 2014/53/EU entspricht.

**Testlabor**: [Name des Testlabors]  
**Zertifikat-Nr.**: [Nummer]  
**Datum**: [Datum]

---

## 16. Support & Dokumentation

### 16.1 Online-Ressourcen

- **Hauptwebsite**: https://inhero.de
- **GitHub-Repository**: https://github.com/[username]/MeshCore
- **Dokumentation**: https://docs.inhero.de/mr1
- **Community-Forum**: https://forum.inhero.de
- **Discord**: https://discord.gg/[invite]

### 16.2 Firmware-Quellen

- **MeshCore Framework**: Open Source (MIT License)
- **Beispiele**: `examples/` Verzeichnis im Repository
- **OTA-Updates**: Über BLE oder Web-Interface

### 16.3 Technischer Support

**E-Mail**: support@inhero.de  
**Antwortzeit**: 24-48 Stunden (Werktage)

**Informationen bei Supportanfragen**:
1. Teilenummer und Revisionsnummer
2. Firmware-Version (`board.info`)
3. Konfiguration (`board.bat`, `board.frost`, etc.)
4. Telemetrie-Ausgabe (`board.tele`)
5. Fehlerbeschreibung mit Schritten zur Reproduktion

### 16.4 Entwickler-Community

- **Beiträge willkommen**: Pull Requests auf GitHub
- **Fehlermeldungen**: GitHub Issues
- **Feature-Anfragen**: GitHub Discussions

---

## 17. Garantie & Haftungsausschluss

### 17.1 Garantie

Inhero GmbH gewährt eine **12-monatige beschränkte Garantie** ab Kaufdatum gegen Material- und Verarbeitungsfehler.

**Garantieausschlüsse**:
- Physische Beschädigung (Fall, Wasser, etc.)
- Unsachgemäße Verwendung (Überspannung, falsche Konfiguration)
- Modifikationen der Hardware
- Verschleißteile (Batterie)

### 17.2 Haftungsausschluss

Die Software wird "wie besehen" bereitgestellt, ohne Gewährleistung jeglicher Art. Siehe MIT-Lizenz für Details.

**Inhero GmbH haftet nicht für**:
- Indirekte Schäden
- Datenverlust
- Betriebsausfall
- Folgeschäden

---

## 18. Revision History

| Version | Datum | Autor | Änderungen |
|---------|-------|-------|------------|
| 1.0 | 29.01.2026 | Inhero GmbH | Erstveröffentlichung Datenblatt |

---

## 19. Kontakt

**Inhero GmbH**  
[Adresse]  
[PLZ] [Stadt]  
Deutschland

**Telefon**: +49 [Nummer]  
**E-Mail**: info@inhero.de  
**Web**: https://inhero.de

---

## 20. Lizenzen

### Hardware
- **PCB-Design**: © 2026 Inhero GmbH (Proprietär, EasyEDA-Pro)
- **Schaltplan**: Verfügbar auf Anfrage für Entwickler

### Software
- **Firmware**: MIT License (Open Source)
- **MeshCore Framework**: MIT License
- **Bibliotheken**: Siehe jeweilige Lizenz (meist BSD/MIT)

```
Copyright (c) 2026 Inhero GmbH

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

**Ende des Datenblatts**

*Dieses Dokument wurde automatisch generiert basierend auf Firmware-Analyse und technischen Spezifikationen. Für produktionsbereite Veröffentlichung sollten alle Platzhalter-Abschnitte (mechanische Abmessungen, Bestellnummern, Zertifizierungen) mit tatsächlichen Daten ausgefüllt werden.*
