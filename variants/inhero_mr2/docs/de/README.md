# Inhero MR-2

<img src="../img/front.jpg" alt="Inhero MR2" width="400">

> 🇬🇧 [English version](../README.md)

## Inhaltsverzeichnis

- [Übersicht](#übersicht)
- [Aktuelle Feature-Matrix](#aktuelle-feature-matrix)
- [Energieverwaltungsfunktionen](#energieverwaltungsfunktionen)
- [Firmware-Build](#firmware-build)
- [CLI-Befehle](#cli-befehle)
- [Diagnose & Fehlersuche](#diagnose--fehlersuche)
- [Siehe auch](#siehe-auch)

## Übersicht

Das Inhero MR-2 ist eine anwendungsspezifische Hardware-Plattform für den autarken Dauerbetrieb von Mesh-Infrastruktur, die im Gegensatz zu herkömmlichen Universallösungen auf maximale Zuverlässigkeit an wartungsintensiven Standorten optimiert ist. Ein universeller Solareingang mit aktivem MPPT maximiert die Energieausbeute, was kompakte, unauffällige Installationen ermöglicht und teure Überdimensionierungen der Peripherie vermeidet. Dank nativer Unterstützung für Li-Ion, LiFePO4 sowie LTO und einer autonomen Recovery-Logik via RTC-Wakeup wird ein konsequenter „Install & Forget“-Ansatz auch unter extremen Umweltbedingungen realisiert. Das Design minimiert so die langfristigen Betriebskosten an Orten, an denen manuelle Wartungseinsätze aufgrund schwieriger Erreichbarkeit unverhältnismäßig aufwändig wären.

**Hardware-Version:** Rev 1.1  
**Hauptmerkmale:**
- **Core:** Basierend auf RAK4630 (nRF52840 + SX1262).
- **Power-Path:** BQ25798 Buck/Boost Charger. Ermöglicht Energiegewinnung auch dann, wenn die Solarspannung unter der Akkuspannung liegt (wichtig für Schwachlicht-Phasen).
- **High-Efficiency Rail:** 3.3V-Rail via TPS62840 für maximale Effizienz.
- **Robustes Monitoring:** INA228 Coulomb-Counter für präzises SOC-Tracking (essenziell für LiFePO4-Chemie) und Langzeit-Energiestatistiken.
- **Universeller Solareingang:** 3.6V – 24V mit autonomem MPPT-Tracking und integriertem Schutz gegen „Stuck-States“ (Hardware-Watchdog-Logik).
- **Umweltsensorik & Zeit:** Integrierter BME280 und RV-3028 RTC für autonomes Wake-up Management und präzise Zeitbasis.
- **Formfaktor:** Nur 45 × 40 mm – optimiert für unauffällige Gehäuse und geringe mechanische Belastung.

## Aktuelle Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| INA228 ALERT → Low-Voltage System Sleep | Aktiv | ISR auf P1.02 → Task-Notification → System Sleep mit GPIO-Latch + RTC-Wake |
| RTC-Wakeup (Low-Voltage-Recovery) | Aktiv | 60 min (periodisch) |
| BQ CE-Pin Safety (FET-invertiert) | Aktiv | GPIO HIGH → FET ON → CE LOW → Laden an (BQ25798 CE active-low), Dual-Layer: GPIO + I2C |
| System Sleep mit gelatchtem CE | Aktiv | < 500µA, GPIO4-Latch HIGH erhalten → FET ON → CE LOW → Solar-Laden möglich |
| SOC 0% nach Low-Voltage-Recovery | Aktiv | SOC wird bei Recovery auf 0% initialisiert, auto-sync bei "Charging Done" |
| SOC via INA228 + manuelle Batteriekapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-Ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |
| PFM Forward Mode | Permanent aktiv | Immer aktiviert (verbessert Effizienz bei niedrigen Solarströmen) |

## Energieverwaltungsfunktionen

### Low-Voltage-Handling (Flag/Tick-Architektur)

1. **INA228 ALERT** feuert bei `lowv_sleep_mv` (Hardware-Interrupt auf P1.02)
2. **ISR** setzt `lowVoltageAlertFired = true` (nur volatile Flag, kein FreeRTOS-Aufruf)
3. **`tickPeriodic()`** (Main-Loop, nächster `tick()`) prüft Flag → Shutdown:
   - CE-Pin → HIGH (FET ON → CE LOW → Laden aktiv)
   - P0.04 von `disconnectLeakyPullups()` ausgeschlossen → GPIO-Latch bleibt im Sleep erhalten
   - RTC-Wake konfiguriert (`LOW_VOLTAGE_SLEEP_MINUTES` = 60 min)
   - SOC auf 0% gesetzt
   - `sd_power_system_off()` → **System Sleep mit GPIO-Latch** (< 500µA)
4. **RTC-Wake** (stündlich) → System bootet, Early-Boot prüft VBAT:
   - Unter `lowv_wake_mv` → sofort wieder System Sleep (CE bleibt gelatcht LOW)
   - Über `lowv_wake_mv` → normaler Boot, SOC startet bei 0%

> **Hinweis**: Alle I2C-Operationen (MPPT, SOC, Hourly Stats) laufen im Main-Loop-Kontext
> über `tickPeriodic()` — keine FreeRTOS-Tasks für I2C, kein Mutex nötig.

### BQ CE-Pin (Rev 1.1 — FET-invertiert)
- **DMN2004TK-7 N-FET**: Gate ← GPIO4 (ext. Pull-Down), Drain → CE, Source → GND
- **GPIO HIGH** → FET ON → CE LOW → **Laden AN** (BQ25798 CE active-low)
- **GPIO LOW / High-Z** → ext. Pull-Down am Gate → FET OFF → Pull-Up am CE → CE HIGH → **Laden AUS**
- **System Sleep**: GPIO4-Latch HIGH erhalten (von `disconnectLeakyPullups()` ausgeschlossen) → FET ON → CE LOW → **Solar-Laden aktiv**
- **Safety-Default**: RAK stromlos/ungeflasht → Pull-Down am Gate → FET OFF → CE HIGH → **Laden deaktiviert**
- **Dual-Layer**: CE-Pin (Hardware-FET) + `setChargeEnable()` (I2C Register)

### Spannungsschwellen (alle Chemien)

| Chemie | lowv_sleep_mv | lowv_wake_mv | Hysterese |
|--------|--------------|-------------|-----------|
| Li-Ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |

- **lowv_sleep_mv**: INA228 ALERT-Schwelle → löst System Sleep mit GPIO-Latch aus
- **lowv_wake_mv**: RTC-Wake-Schwelle → Boot nur wenn VBAT darüber liegt, gleichzeitig 0% SOC-Marker

### Stromverbrauch im System Sleep
- **< 500µA** Gesamtverbrauch (nRF52840 System-Off + RTC + quiescent currents aller Komponenten)
- GPIO4-Latch HIGH erhalten → FET ON → CE LOW → Solar-Laden aktiv

### Coulomb Counter & SOC-Tracking
- **Echtzeit-SOC-Tracking** via INA228 (±0.1% Genauigkeit)
- **100mΩ Shunt-Widerstand** (1.6A max Strom)
- **200mV einheitliche Hysterese** für alle Chemien (lowv_sleep_mv → lowv_wake_mv)
- **Manuelle Kapazität:** `set board.batcap` für feste Kapazität

### SOC→Li-Ion mV Mapping (Workaround)
- **Problem**: MeshCore überträgt nur `getBattMilliVolts()`, keinen SOC%. Die Companion App nutzt eine Li-Ion-Kurve zur SOC-Berechnung — falsche Anzeige bei LiFePO4/LTO.
- **Lösung**: Bei validem Coulomb-Counting-SOC wird eine äquivalente Li-Ion 1S OCV (3000–4200 mV) zurückgegeben, sodass die App den korrekten SOC% anzeigt.
- **TODO**: Entfernen, sobald MeshCore die native Übertragung des SOC% unterstützt.

### Time-To-Live (TTL) Prognose
- **Zeitbasis:** 7-Tage gleitender Durchschnitt (`avg_7day_daily_net_mah`) des täglichen Netto-Energieverbrauchs
- **Datenbasis:** 168-Stunden-Ringpuffer (7 Tage) mit stündlichen INA228-Coulomb-Counter-Samples (charged/discharged/solar mAh)
- **Formel:** `TTL_hours = (SOC% × capacity_mah / 100) / |avg_7day_daily_net_mah| × 24`
- **Voraussetzungen:** `living_on_battery == true` (24h-Defizit), mind. 24h Daten, Kapazität bekannt
- **TTL = 0:** Solar-Überschuss, keine 24h Daten vorhanden, oder Kapazität unbekannt
- **CLI:** TTL wird in `get board.stats` angezeigt (nur im BAT-Modus, z.B. `T:12d0h`)
- **Telemetrie:** Wird als Tage via CayenneLPP Distance-Feld übertragen (max. 990 Tage für "unendlich")

### Solar-Energieverwaltung 🆕

- **Solarstrom-Anzeige:** Der BQ25798 IBUS-ADC ist bei niedrigen Strömen ungenau (~±30mA Fehler). Daher wird der Solarstrom abgestuft angezeigt:
  - `0mA` — ADC meldet exakt 0 (kein Solarstrom)
  - `<50mA` — 1–49mA (ADC in diesem Bereich unzuverlässig)
  - `~72mA` — 50–100mA mit Rundungszeichen `~` (eingeschränkte Genauigkeit)
  - `385mA` — >100mA ohne Rundungszeichen (hinreichend genau)
  - Immer ganzzahlig ohne Dezimalstellen (keine Pseudopräzision)
- **PFM Forward Mode:** Permanent aktiviert. Verbessert Effizienz bei niedrigen Strömen.
- **MPPT VOC_PCT 81.25%:** Der BQ25798-MPPT ist auf VOC_PCT=81.25% konfiguriert (statt Chip-Default 87.5% oder vormals 75%). Dieser Wert entspricht dem typischen Vmp/Voc-Verhältnis kristalliner Silizium-Solarzellen (~80-83%).
- **MPPT-Recovery:** Aktiviert MPPT wieder bei PowerGood=1 (Readback-Check: nur bei tatsächlicher Änderung)
- **BQ INT-Pin nicht genutzt:** Kein Interrupt — reines Polling alle 60s in `runMpptCycle()`
- **Fehlerüberwachung:** Diagnosebefehle zeigen FAULT_STATUS-Register (0x20, 0x21) für detaillierte Analyse inkl. VBAT_OVP, VBUS_OVP und Temperaturbedingungen
- **VREG-Anzeige:** Zeigt die tatsächlich konfigurierte Battery-Regulation-Spannung in der Diagnose zur Schwellenwert-Prüfung

## Firmware-Build

```bash
# Repeater (Standard)
platformio run -e Inhero_MR2_repeater

# Repeater mit RS232-Bridge (Serial2 an P0.19/P0.20)
platformio run -e Inhero_MR2_repeater_bridge_rs232

# Room Server
platformio run -e Inhero_MR2_room_server

# Companion Radio (USB, mit Extra-Filesystem)
platformio run -e Inhero_MR2_companion_radio_usb

# Terminal Chat
platformio run -e Inhero_MR2_terminal_chat

# Sensor
platformio run -e Inhero_MR2_sensor

# KISS Modem
platformio run -e Inhero_MR2_kiss_modem
```

## CLI-Befehle

### Get-Befehle
```bash
get board.bat       # Aktuellen Batterietyp abfragen
                    # Ausgabe: liion1s | lifepo1s | lto2s | none

get board.fmax      # Frost-Ladeverhalten abfragen
                    # Ausgabe: 0% | 20% | 40% | 100%
                    # Wert = maximaler Ladestrom im T-Cool-Bereich (0°C bis -5°C),
                    # relativ zu board.imax
                    # 40% bei imax=500mA → max. 200mA Ladestrom bei 0°C bis -5°C
                    # 0% = Laden im T-Cool-Bereich gesperrt
                    # 100% = keine Reduktion (voller Strom auch bei Kälte)
                    # Unter -5°C (T-Cold): Laden immer komplett gesperrt (JEITA)
                    # Hinweis: Nur das Laden wird eingeschränkt. Bei ausreichend
                    # Solar wird das Board weiterhin mit Solarstrom betrieben —
                    # der Akku wird weder ge- noch entladen.
                    # LTO batteries: N/A (JEITA disabled, lädt auch bei Frost)

get board.imax      # Maximalen Ladestrom abfragen
                    # Ausgabe: <current>mA (z.B. 200mA)

get board.mppt      # MPPT-Status abfragen
                    # Ausgabe: MPPT=1 (aktiviert) | MPPT=0 (deaktiviert)

get board.telem     # Echtzeit-Telemetrie mit SOC abfragen 🆕
                    # Ausgabe: B:<V>V/<I>mA/<T>C SOC:<Prozent>% S:<V>V/<SolarStrom>
                    # Beispiele:
                    #   B:3.85V/125.4mA/22C SOC:68.5% S:5.12V/385mA      (>100mA: genau)
                    #   B:3.85V/-8.2mA/18C SOC:72.0% S:4.90V/~72mA      (50-100mA: ~Schätzwert)
                    #   B:3.30V/-45.0mA/5C SOC:40.1% S:0.00V/<50mA      (<50mA: ADC ungenau)
                    # Komponenten:
                    # - B: Battery (Voltage/Current/Temperature/SOC)
                    # - S: Solar (Voltage/Strom — Genauigkeit abhängig vom BQ25798 IBUS-ADC)

get board.stats     # Energie-Statistiken (Bilanz + MPPT) abfragen 🆕
                    # Ausgabe: <24h>/<3d>/<7d>mAh C:<24h> D:<24h> 3C:<3d> 3D:<3d> 7C:<7d> 7D:<7d> <SOL|BAT> M:<mppt>% T:<ttl>
                    # Beispiel: +125/+45/+38mAh C:200 D:75 3C:150 3D:105 7C:140 7D:102 SOL M:85% T:N/A
                    # Beispiel: -30/-45/-40mAh C:10 D:40 3C:5 3D:50 7C:8 7D:48 BAT M:45% T:72h
                    # Komponenten:
                    # - +125: Last 24h net balance (charge - discharge) in mAh
                    # - +45: 3-day average net balance in mAh
                    # - +38: 7-day average net balance in mAh
                    # - C/D: Charged/Discharged mAh (24h)
                    # - 3C/3D: 3-day average charged/discharged mAh
                    # - 7C/7D: 7-day average charged/discharged mAh
                    # - SOL: Running on solar (self-sufficient)
                    # - BAT: Living on battery (deficit mode)
                    # - M:85%: MPPT enabled percentage (7-day average)
                    # - T:72h: Time To Live (only shown if BAT mode, 7d-avg basis)
                    #   Format: T:12d5h (≥24h) oder T:72h (<24h) oder T:N/A

get board.cinfo     # Ladegerät-Info + letzter PG-Stuck HIZ-Toggle
                    # Ausgabe: <state> + flags
                    # States: !CHG, PRE, CC, CV, TRICKLE, TOP, DONE

get board.conf      # Alle Konfigurationswerte abfragen
                    # Ausgabe: B:<bat> F:<fmax> M:<mppt> I:<imax> Vco:<voltage> V0:<0%SOC>

get board.batcap    # Batteriekapazität abfragen
                    # Ausgabe: <capacity> mAh (set) oder <capacity> mAh (default)
                    # Zeigt ob Kapazität manuell gesetzt oder Chemie-Default

get board.tccal     # NTC-Temperatur-Kalibrieroffset abfragen
                    # Ausgabe: TC offset: <+/-offset> C (0.00=default)

get board.leds      # LED-Aktivstatus abfragen
                    # Ausgabe: "LEDs: ON (Heartbeat + BQ Stat)" oder "LEDs: OFF (Heartbeat + BQ Stat)"
                    # Shows whether heartbeat LED and BQ25798 stat LED are enabled
```

### Set-Befehle
```bash
set board.bat <type>           # Batterietyp setzen
                               # Options: lto2s | lifepo1s | liion1s | none
                               # none = kein Akku / unbekannt (Laden deaktiviert)

set board.fmax <behavior>      # Frost-Ladeverhalten setzen
                               # Options: 0% | 20% | 40% | 100%
                               # Begrenzt Ladestrom im T-Cool-Bereich (0°C bis -5°C)
                               # auf X% von board.imax
                               # 0% = Laden im T-Cool-Bereich gesperrt
                               # 20% = max. 20% von imax bei 0°C bis -5°C
                               # 40% = max. 40% von imax bei 0°C bis -5°C
                               # 100% = keine Reduktion
                               # Unter -5°C (T-Cold): Laden immer gesperrt (JEITA)
                               # Hinweis: Nur das Laden wird eingeschränkt. Bei
                               # ausreichend Solar läuft das Board weiterhin auf
                               # Solarstrom — der Akku wird weder ge- noch entladen.
                               # N/A for LTO batteries (JEITA disabled)

set board.imax <current>       # Maximalen Ladestrom in mA setzen
                               # Range: 50-1500mA (BQ25798-Minimum: 50mA)

set board.mppt <1|0>           # Enable/disable MPPT
                               # 1 = enabled, 0 = disabled

set board.batcap <capacity>    # Batteriekapazität in mAh setzen
                               # Range: 100-100000 mAh
                               # Used for accurate SOC calculation

set board.tccal                # NTC-Temperatur kalibrieren
                               # Zwei Modi:
                               # 1) set board.tccal         → Auto-Kalibrierung via BME280
                               #    Ausgabe: TC auto-cal: BME=<temp> offset=<+/-offset> C
                               # 2) set board.tccal reset    → Offset auf 0.00 zurücksetzen
                               #    Ausgabe: TC calibration reset to 0.00 (default)

set board.leds <on|off>        # Enable/disable heartbeat + BQ stat LED
                               # on/1 = enable, off/0 = disable
                               # Boot-LEDs (3 blaue Blinks) immer aktiv

set board.soc <percent>        # SOC manuell setzen
                               # Bereich: 0-100
                               # Hinweis: INA228 muss initialisiert sein
```

## Diagnose & Fehlersuche

### BQ25798-Registerverifikation
Die Diagnosefunktionen ermöglichen präzise Verifikation der BQ25798-Register gegen das Datenblatt:

**Wichtige Register:**
- **0x0F (CHARGER_CONTROL_0)**: EN_CHG (Bit 5)
- **0x15 (MPPT_CONTROL)**: EN_MPPT (Bit 0), VOC_PCT (Bits 7-5), VOC_DLY (Bits 4-3), VOC_RATE (Bits 2-1)
- **0x1B (CHARGER_STATUS_0)**: PG_STAT (Bit 3), VINDPM (Bit 6), IINDPM (Bit 7)
- **0x1C (CHARGER_STATUS_1)**: CHG_STAT (Bits 7-5), VBUS_STAT (Bits 4-1)
- **0x1F (CHARGER_STATUS_4)**: Temperature status (Bits 3-0)

**Bekannte Probleme:**
1. **MPPT deaktiviert**: BQ25798 setzt MPPT=0 automatisch bei PG=0
   - Lösung: `checkAndFixSolarLogic()` reaktiviert MPPT bei PG=1
2. **PG-Stuck bei Sonnenaufgang**: VBUS steigt langsam, BQ qualifiziert die Quelle nicht
   - Lösung: `checkAndFixSolarLogic()` toggled HIZ bei VBUS ≥ 4.5V + PG=0 (5min Cooldown)

## Siehe auch

- [QUICK_START.md](QUICK_START.md) - Schnellstart fuer Inbetriebnahme und CLI-Setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) - Alle board-spezifischen CLI-Befehle auf einen Blick
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
