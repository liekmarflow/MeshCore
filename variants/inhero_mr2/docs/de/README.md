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
- [Regulatorische Hinweise & CE-Konformität](#regulatorische-hinweise--ce-konformität-red-201453eu)
- [Siehe auch](#siehe-auch)

## Übersicht

Das Inhero MR-2 ist eine anwendungsspezifische Hardware-Plattform für den autarken Dauerbetrieb von Mesh-Infrastruktur, die im Gegensatz zu herkömmlichen Universallösungen auf maximale Zuverlässigkeit an wartungsintensiven Standorten optimiert ist. Mit einem aktiven Idle-Verbrauch von nur 6,0 mA bei 4,2 V bzw. 7,7 mA bei 3,3 V (USB aus, kein Radio-TX) ist das Board für einen voll ausgestatteten Repeater außergewöhnlich sparsam — das ermöglicht lange Laufzeiten selbst mit kompakten Akkus und kleinen Solarpanels. Ein universeller Solareingang mit aktivem MPPT maximiert die Energieausbeute, was kompakte, unauffällige Installationen ermöglicht und teure Überdimensionierungen der Peripherie vermeidet. Dank nativer Unterstützung für Li-Ion, LiFePO4, LTO sowie Na-Ion und einer autonomen Recovery-Logik via RTC-Wakeup wird ein konsequenter „Install & Forget"-Ansatz auch unter extremen Umweltbedingungen realisiert. Das Design minimiert so die langfristigen Betriebskosten an Orten, an denen manuelle Wartungseinsätze aufgrund schwieriger Erreichbarkeit unverhältnismäßig aufwändig wären.

**Hardware-Version:** Rev 1.1  
**Hauptmerkmale:**
- **Core:** Basierend auf RAK4630 (nRF52840 + SX1262).
- **Power-Path:** BQ25798 Buck/Boost Charger. Ermöglicht Energiegewinnung auch dann, wenn die Solarspannung unter der Akkuspannung liegt (wichtig für Schwachlicht-Phasen).
- **High-Efficiency Rail:** 3.3V-Rail via TPS62840 für maximale Effizienz.
- **Robustes Monitoring:** INA228 Coulomb-Counter für präzises SOC-Tracking (essenziell für LiFePO4-Chemie) und Langzeit-Energiestatistiken.
- **Universeller Solareingang:** 3.6V – 24V mit autonomem MPPT-Tracking und integriertem Schutz gegen „Stuck-States“ (Hardware-Watchdog-Logik).
- **Umweltsensorik & Zeit:** Integrierter BME280 und RV-3028 RTC für autonomes Wake-up Management und präzise Zeitbasis.
- **Formfaktor:** Nur 45 × 40 mm – optimiert für unauffällige Gehäuse und geringe mechanische Belastung.

> **⚠ WARNUNG — Kein Verpolschutz:** Das Board verfügt über keinen Hardware-Verpolschutz an Akku- oder Solareingang. Ein verpolter Anschluss führt zu sofortiger, irreversibler Beschädigung. Vor dem Anschließen immer die Polarität prüfen.

## Aktuelle Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| INA228 ALERT → Low-Voltage System Sleep | Aktiv | ISR auf P1.02 → volatile Flag → tickPeriodic() → System Sleep mit GPIO-Latch + RTC-Wake |
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
| Na-Ion 1S | 2500 | 2700 | 200mV |

- **lowv_sleep_mv**: INA228 ALERT-Schwelle → löst System Sleep mit GPIO-Latch aus
- **lowv_wake_mv**: RTC-Wake-Schwelle → Boot nur wenn VBAT darüber liegt, gleichzeitig 0% SOC-Marker

### Stromverbrauch im System Sleep
- **< 500µA** Gesamtverbrauch (nRF52840 System-Off + RTC + quiescent currents aller Komponenten)
- GPIO4-Latch HIGH erhalten → FET ON → CE LOW → Solar-Laden aktiv

### Stromverbrauch im Active Idle
- **6,0 mA** @ VBAT 4,2 V (USB aus, kein Radio-TX)
- **7,7 mA** @ VBAT 3,3 V (USB aus, kein Radio-TX)
- **+0,8–1,0 mA** mit USB-Peripherie aktiviert
- USB wird automatisch verwaltet: aktiviert bei VBUS-Erkennung, deaktiviert bei Entfernung

### Stromsparmaßnahmen
- **WFE Idle** (`board.sleep(0)`): CPU geht zwischen Loop-Iterationen in den Wait-For-Event-Modus. Aufwachen bei jedem Interrupt (Radio, SysTick, USB, I2C) — typisch innerhalb 1 ms. Reduziert den nRF52840-CPU-Strom von ~3 mA (Busy-Loop) auf ~0,5–0,8 mA.
- **USB Auto-Disable**: Die nRF52840-USB-Peripherie wird automatisch deaktiviert wenn kein VBUS erkannt wird, spart ~0,8–1,0 mA. Wird automatisch reaktiviert wenn ein USB-Kabel angeschlossen wird.

### Coulomb Counter & SOC-Tracking
- **Echtzeit-SOC-Tracking** via INA228 (±0.1% Genauigkeit)
- **100mΩ Shunt-Widerstand** (1.6A max Strom)
- **200mV einheitliche Hysterese** für alle Chemien (lowv_sleep_mv → lowv_wake_mv)
- **Manuelle Kapazität:** `set board.batcap` für feste Kapazität

### SOC→Li-Ion mV Mapping (Workaround)
- **Problem**: MeshCore überträgt nur `getBattMilliVolts()`, keinen SOC%. Die Companion App nutzt eine Li-Ion-Kurve zur SOC-Berechnung — falsche Anzeige bei LiFePO4/LTO.
- **Lösung**: Bei validem Coulomb-Counting-SOC wird eine äquivalente Li-Ion 1S OCV (3000–4200 mV) zurückgegeben, sodass die App den korrekten SOC% anzeigt. Siehe [TELEMETRY.md](TELEMETRY.md) für Details zur App-Anzeige.
- **TODO**: Entfernen, sobald MeshCore die native Übertragung des SOC% unterstützt.

### Time-To-Live (TTL) Prognose
- **Zeitbasis:** 7-Tage gleitender Durchschnitt (`avg_7day_daily_net_mah`) des täglichen Netto-Energieverbrauchs
- **Datenbasis:** 168-Stunden-Ringpuffer (7 Tage) mit stündlichen INA228-Coulomb-Counter-Samples (charged/discharged/solar mAh)
- **Formel:** `TTL_hours = (SOC% × capacity_mah / 100) / |avg_7day_daily_net_mah| × 24`
- **Voraussetzungen:** `living_on_battery == true` (24h-Defizit), mind. 24h Daten, Kapazität bekannt
- **TTL = 0:** Solar-Überschuss, keine 24h Daten vorhanden, oder Kapazität unbekannt
- **CLI:** TTL wird in `get board.stats` angezeigt (nur im BAT-Modus, z.B. `T:12d0h`)
- **Telemetrie:** Wird als Tage via CayenneLPP Distance-Feld übertragen (max. 990 Tage für "unendlich"). Siehe [TELEMETRY.md](TELEMETRY.md) für Kanal-Details.

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

### JEITA-Temperaturzonen-Konfiguration

Der BQ25798 nutzt den TS-Pin (NTC-Thermistor) für JEITA-konforme temperaturabhängige Laderegelung. Das Inhero MR2 verwendet einen Spannungsteiler (RT1=5,6 kΩ Pullup an REGN, RT2=27 kΩ parallel zu GND), der TS-Schwellen nach unten verschiebt gegenüber dem TI-Referenzdesign (5,24 kΩ / 30,31 kΩ). Die Verschiebung ist **temperaturabhängig**: ~5–6 °C im Kaltbereich, ~2–3 °C im Warm-/Hot-Bereich (da bei niedrigen Temperaturen der NTC-Widerstand groß ist relativ zu RT2 und den Teiler-Unterschied verstärkt).

| JEITA-Zone | BQ25798-Schwelle | TI-Referenz | Inhero MR2 (real) | Shift | Firmware-Konfig |
|------------|------------------|-------------|--------------------|---------|-----------------|
| T-Cold (Ladung gesperrt) | VT1 = 72,0% REGN | +3,7 °C | −2,0 °C | −5,7 °C | — (nicht konfigurierbar) |
| T-Cool (reduzierter Strom) | VT2 = 69,8% REGN | +7,9 °C | +2,8 °C | −5,1 °C | `set board.fmax` |
| T-Warm Start | VT3 = 37,7% REGN | +54,5 °C | +52,2 °C | −2,3 °C | `TS_WARM = 55°C` Register-Einstellung |
| T-Hot (Ladung gesperrt) | VT5 = 34,2% REGN | +59,9 °C | +57,7 °C | −2,2 °C | — (nicht konfigurierbar) |

> Berechnung basiert auf: NTC 103AT (B25/50=3435) für TI-Referenz, NCP15XH103F03RC (B25/85=3380) für Inhero. Typische %REGN-Werte aus BQ25798-Datenblatt.

**Wichtige Firmware-Einstellungen in `configureBaseBQ()`:**

- **`TS_WARM = 55°C`** (BQ-Registerwert): Verschiebt die WARM-Zonen-Schwelle vom Default 45 °C (44,8% REGN, ~41,8 °C mit Inhero-Teiler) auf 37,7% REGN (~52,2 °C mit Inhero-Teiler). Verhindert vorzeitigen WARM-Zonen-Eintritt bei moderaten Temperaturen.
- **`JEITA_VSET = UNCHANGED`**: Keine Reduktion der Battery-Regulation-Spannung in der WARM-Zone. Der POR-Default (VREG−400 mV) würde VREG auf 3,1 V für LiFePO4 reduzieren und VBAT_OVP bei normalen Batteriespannungen (3,3–3,5 V) auslösen.
- **`JEITA_ISETH = ICHG unchanged`** (POR-Default, beibehalten): Keine Ladestrom-Reduktion in der WARM-Zone. Zusammen mit JEITA_VSET=UNCHANGED ist die WARM-Zone effektiv neutralisiert — Ladung läuft mit voller Spannung und vollem Strom weiter.
- **`AUTO_IBATDIS = deaktiviert`**: Deaktiviert die automatische 30-mA-Batterieentladung des BQ25798 während VBAT_OVP. Der POR-Default entlädt die Batterie aktiv mit ~30 mA (IBAT_LOAD) bei OVP — kontraproduktiv für solarbetriebene Systeme.

> **Hintergrund:** Mit den BQ25798-Standardeinstellungen verursachte die Kombination aus Inhero-Teiler-Offset und LiFePO4-Chemie eine Fehlerkette bei ~42 °C: WARM-Zonen-Eintritt → VREG auf 3,1 V reduziert → VBAT_OVP (Batterie bei 3,47 V > 104% × 3,1 V) → aktive 30-mA-Entladung → netto −45 mA Verbrauch trotz Solareinspeisung. Die obigen Einstellungen verhindern dies vollständig. Die WARM-Zone (52–58 °C mit Inhero-Teiler) hat nun keinen Einfluss auf das Ladeverhalten.

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
                    # Ausgabe: liion1s | lifepo1s | lto2s | naion1s | none

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
                    # LTO / Na-Ion batteries: N/A (JEITA disabled, lädt auch bei Frost)

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

get board.selftest  # I²C-Hardware-Probe (alle Onboard-Komponenten)
                    # Ausgabe: INA:<state> BQ:<state> RTC:<state> BME:<state>
                    # States: OK | NACK | WR_FAIL (nur RTC, Write-Verify-Mismatch)

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
                               # Options: lto2s | lifepo1s | liion1s | naion1s | none
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
                               # N/A for LTO / Na-Ion batteries (JEITA disabled)

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

### I²C-Hardware-Selbsttest

```bash
get board.selftest
```

Prüft alle I²C-Komponenten auf dem Board und liefert den Status in einer Zeile:

```
INA:OK BQ:OK RTC:OK BME:OK
```

| Komponente | Adresse | Test |
|---|---|---|
| `INA` | `0x40` | INA228 Power-Monitor — Adress-ACK |
| `BQ`  | `0x6B` | BQ25798 Charger — Adress-ACK |
| `RTC` | `0x52` | RV-3028 RTC — Adress-ACK **plus** User-RAM (`0x1F`) Write/Readback-Verifikation mit zwei Mustern (`0xA5`, `0x5A`); das ursprüngliche Byte wird wiederhergestellt |
| `BME` | `0x76` | BME280 Umweltsensor — Adress-ACK |

Mögliche Status-Werte je Gerät:

- **`OK`** — Gerät antwortet (RTC: Writes werden korrekt persistiert)
- **`NACK`** — Gerät antwortet nicht auf dem I²C-Bus
- **`WR_FAIL`** — *nur RTC* — Chip ACKt, aber Write/Read stimmen nicht überein. Dieselbe Write-Verifikation läuft in `BoardConfigContainer::begin()` und löst bei Fehler die langsam blinkende rote Error-LED aus, sodass das Board vor dem Deployment als fehlerhaft markiert wird.

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

## Regulatorische Hinweise & CE-Konformität (RED 2014/53/EU)

Das Inhero MR-2 wird als Hardware-Plattform (Entwicklungsmodul) mit vorinstalliertem Bootloader ausgeliefert. Die Hardware wurde durch ein akkreditiertes Prüflabor auf Konformität gemäß der europäischen Funkanlagenrichtlinie (RED 2014/53/EU) getestet. Die Zertifizierung der abgestrahlten Leistung erfolgte unter Verwendung der vorgesehenen Referenzantennen (RAK FPCB-Antenne 863–870 MHz, MHF1-Anschluss, Antennengewinn: 0,7 dBi).

**Vorgaben für den gesetzeskonformen Betrieb der Funk-Firmware:**
Da die finale Sendecharakteristik (Sendeleistung, Frequenz, Duty Cycle) maßgeblich von der durch den Anwender installierten Software (z. B. MeshCore) und der gewählten Antenne abhängt, muss sichergestellt werden, dass die folgenden europäischen Grenzwerte (gemäß EN 300 220 und EN 300 328, ERC/REC 70-03 Annex 1) zwingend eingehalten werden:

1. **Standard LoRa (868 MHz Band):**
   * Frequenzbereich: 865,0 – 868,6 MHz
   * Max. abgestrahlte Sendeleistung (ERP): 25 mW (14 dBm)
   * Max. Duty Cycle: 1 % (oder LBT+AFA gemäß EN 300 220)
   * *Hinweis: Im Sub-Band 863,0 – 865,0 MHz gelten abweichende Duty-Cycle-Anforderungen gemäß ERC/REC 70-03.*

2. **High-Power LoRa (Sonderbereich 869,5 MHz):**
   * Frequenzbereich: 869,40 – 869,65 MHz (MeshCore Standard-Kanal)
   * Max. abgestrahlte Sendeleistung (ERP): **500 mW (27 dBm)**
   * Max. Duty Cycle: 10 %
   * *Hinweis zur Hardware:* Der verbaute LoRa-Transceiver (SX1262) liefert eine maximale leitungsgebundene Sendeleistung von 22 dBm. Um das gesetzliche Limit von 500 mW ERP (entspricht 29,15 dBm EIRP) vollständig auszuschöpfen, ist die Verwendung einer entsprechenden Antenne mit einem Antennengewinn von ca. +7 dBi erforderlich (abzüglich etwaiger Kabelverluste). Mit der mitgelieferten FPCB-Antenne (0,7 dBi) werden max. ca. 114 mW ERP erreicht.

3. **Bluetooth Low Energy (2,4 GHz):**
   * Max. abgestrahlte Sendeleistung (EIRP): 100 mW (20 dBm)

**Antennen & Verantwortung des Betreibers (EIRP/ERP Limit):**
Der Anwender ist verpflichtet, die konfigurierte Sendeleistung (TX Power) im Chip und den Antennengewinn aufeinander abzustimmen. Wird eine Antenne verwendet, deren Gewinn in Kombination mit der eingestellten Sendeleistung die oben genannten gesetzlichen EIRP/ERP-Limits überschreitet, muss die Sendeleistung softwareseitig zwingend reduziert werden.

**Haftungsausschluss:**
Das Inhero MR-2 ist ein Modul für professionelle Entwickler und qualifizierte Anwender. Werden durch die Wahl der Firmware, der Antenne oder durch manuelle Konfiguration die gesetzlichen Parameter außerhalb der EU-Normen betrieben, erlischt die CE-Konformität des Geräts. In diesem Fall geht die gesamte rechtliche Verantwortung für den Betrieb auf den Integrator bzw. Anwender über.

## Siehe auch

- [TELEMETRY.md](TELEMETRY.md) — Telemetrie-Kanäle erklärt (was die App anzeigt)
- [QUICK_START.md](QUICK_START.md) — Schnellstart für Inbetriebnahme und CLI-Setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle board-spezifischen CLI-Befehle auf einen Blick
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Vollständige technische Dokumentation
