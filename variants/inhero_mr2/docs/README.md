# Inhero MR-2 (Hardware Rev 1.0)

## Inhaltsverzeichnis

- [Übersicht](#übersicht)
- [Aktuelle Feature-Matrix](#aktuelle-feature-matrix)
- [Energieverwaltungsfunktionen (Rev 1.0)](#energieverwaltungsfunktionen-rev-10)
- [Firmware-Build](#firmware-build)
- [CLI-Befehle](#cli-befehle)
- [Diagnose & Fehlersuche](#diagnose--fehlersuche)
- [Siehe auch](#siehe-auch)

## Übersicht

Das Inhero MR-2 ist die zweite Generation des Mesh-Repeaters mit verbessertem Power-Management.

**Hardware-Version:** Rev 1.0  
**Hauptmerkmale:**
- INA228 Power Monitor mit Coulomb Counter + ALERT-Interrupt auf P1.02
- RV-3028-C7 RTC für Wake-up Management
- TPS62840 Buck Converter (EN an VDD — immer an)
- BQ25798 Battery Charger mit MPPT
- BQ CE-Pin (P0.04/WB_IO4) via DMN2004TK-7 N-FET (invertierte Logik: HIGH=Laden an)

## Aktuelle Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| INA228 ALERT → Low-Voltage System-Off | Aktiv | ISR auf P1.02 → Task-Notification → System-Off + RTC-Wake |
| RTC-Wakeup (Low-Voltage-Recovery) | Aktiv | 15 min (periodisch) |
| BQ CE-Pin Safety (FET-invertiert) | Aktiv | HIGH=Laden an (via DMN2004TK-7), Dual-Layer: GPIO + I2C |
| System-Off mit gelatchtem CE | Aktiv | ~15µA, CE-Pin bleibt aktiv → Solar-Laden möglich |
| SOC 0% nach Low-Voltage-Recovery | Aktiv | SOC wird bei Recovery auf 0% initialisiert, auto-sync bei "Charging Done" |
| SOC via INA228 + manuelle Batteriekapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-Ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |
| HIZ-Gated Charging + Parasitic-Drain-Schutz | Aktiv | 2-State-Machine (HIZ_IDLE / CHARGE_ACTIVE) |
| PFM Forward Mode | Konfigurierbar | `set board.pfm 0\|1` (persistent) |
| Auto-Learning (Methode 1/2) | Veraltet | Aktuell nicht umgesetzt/aktiv |

## Energieverwaltungsfunktionen (Rev 1.0)

### Architektur-Vergleich v0.2 → Rev 1.0

| Aspekt | v0.2 | Rev 1.0 |
|--------|------|---------|
| Low-Voltage-Erkennung | Software-Polling (60s) + Hardware-UVLO (INA228→TPS EN) | INA228 ALERT ISR auf P1.02 (Hardware-Interrupt) |
| Shutdown-Modus | System ON Idle (__WFI-Loop, ~0.6mA) | System-Off (~15µA) |
| CE-Pin Logik | Direkt (LOW=enable) | FET-invertiert via DMN2004TK-7 (HIGH=enable) |
| TPS62840 EN | Software-steuerbar (UVLO kann EN abschalten) | An VDD gebunden (immer an) |
| Schwellen-Modell | 2 Stufen (Danger Zone + UVLO) | 1 Stufe (lowv_sleep_mv / lowv_wake_mv) |
| GPIO-Latching | Erforderlich (System ON für CE-Pin) | Nicht nötig (CE-FET hält Zustand in System-Off) |

### Low-Voltage-Handling (Rev 1.0 — Flag/Tick-Architektur)

1. **INA228 ALERT** feuert bei `lowv_sleep_mv` (Hardware-Interrupt auf P1.02)
2. **ISR** setzt `lowVoltageAlertFired = true` (nur volatile Flag, kein FreeRTOS-Aufruf)
3. **`tickPeriodic()`** (Main-Loop, nächster `tick()`) prüft Flag → Shutdown:
   - CE-Pin → HIGH (Laden an, da FET-invertiert — Solar-Laden bleibt möglich)
   - RTC-Wake konfiguriert (`LOW_VOLTAGE_SLEEP_MINUTES` = 60 min)
   - SOC auf 0% gesetzt
   - `sd_power_system_off()` → **System-Off** (~15µA)
4. **RTC-Wake** (stündlich) → System bootet, Early-Boot prüft VBAT:
   - Unter `lowv_wake_mv` → sofort wieder System-Off (CE bleibt gelatcht)
   - Über `lowv_wake_mv` → normaler Boot, SOC startet bei 0%

> **Hinweis**: Alle I2C-Operationen (MPPT, SOC, Hourly Stats) laufen im Main-Loop-Kontext
> über `tickPeriodic()` — keine FreeRTOS-Tasks für I2C, kein Mutex nötig.

### BQ CE-Pin (Rev 1.0 — FET-invertiert)
- **DMN2004TK-7 N-FET**: GPIO HIGH → FET ON → CE an GND → Laden aktiv
- **GPIO LOW** → FET OFF → CE floatet → ext. Pull-Up → CE HIGH → Laden gesperrt
- **Vorteil**: In System-Off (alle GPIOs High-Z) floatet CE → ext. Pull-Up → CE HIGH → **Laden bleibt aktiv** → Solar-Recovery möglich
- **Dual-Layer**: CE-Pin (Hardware-FET) + `setChargeEnable()` (I2C Register)

### Spannungsschwellen (alle Chemien)

| Chemie | lowv_sleep_mv | lowv_wake_mv | Hysterese |
|--------|--------------|-------------|-----------|
| Li-Ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |

- **lowv_sleep_mv**: INA228 ALERT-Schwelle → löst System-Off aus
- **lowv_wake_mv**: RTC-Wake-Schwelle → Boot nur wenn VBAT darüber liegt, gleichzeitig 0% SOC-Marker

### Stromverbrauch im System-Off
- **~15µA** Gesamtverbrauch (nRF52840 System-Off + RTC + quiescent currents)
- CE-Pin bleibt über FET-Schaltung aktiv → Solar-Laden möglich
- Verglichen mit v0.2 System ON Idle (~0.6mA): **40× effizienter**

### Coulomb Counter & Auto-Learning (veraltet)
- **Echtzeit-SOC-Tracking** via INA228 (±0.1% Genauigkeit)
- **100mΩ Shunt-Widerstand** (1.6A max Strom)
- **200mV einheitliche Hysterese** für alle Chemien (lowv_sleep_mv → lowv_wake_mv)
- **Auto-Learning-Status:** veraltet / aktuell nicht aktiv in dieser Firmware

### SOC→Li-Ion mV Mapping (Workaround)
- **Problem**: MeshCore überträgt nur `getBattMilliVolts()`, keinen SOC%. Die Companion App nutzt eine Li-Ion-Kurve zur SOC-Berechnung — falsche Anzeige bei LiFePO4/LTO.
- **Lösung**: Bei validem Coulomb-Counting-SOC wird eine äquivalente Li-Ion 1S OCV (3000–4200 mV) zurückgegeben, sodass die App den korrekten SOC% anzeigt.
- **TODO**: Entfernen, sobald MeshCore die native Übertragung des SOC% unterstützt.
- **Zwei-Methoden-Auto-Learning** zur Kapazitätskalibrierung (historisches Konzept):
   * **Methode 1:** Voller Entladezyklus (100% → 10% Danger Zone, ~29 Tage @ 13mA)
   * **Methode 2:** USB-C-Ladung aus der Danger Zone (0% → 100%, ~Stunden)
- **Learning gate / persistence:** dokumentiert, aber derzeit nicht aktiv
- **Manuelle Kapazität:** `set board.batcap` für feste Kapazität
- **7-Tage-Energiebilanz** für TTL-Prognosen

### Time-To-Live (TTL) Prognose
- **Zeitbasis:** 7-Tage gleitender Durchschnitt (`avg_7day_daily_net_mah`) des täglichen Netto-Energieverbrauchs
- **Datenbasis:** 168-Stunden-Ringpuffer (7 Tage) mit stündlichen INA228-Coulomb-Counter-Samples (charged/discharged/solar mAh)
- **Formel:** `TTL_hours = (SOC% × capacity_mah / 100) / |avg_7day_daily_net_mah| × 24`
- **Voraussetzungen:** `living_on_battery == true` (24h-Defizit), mind. 24h Daten, Kapazität bekannt
- **TTL = 0:** Solar-Überschuss, keine 24h Daten vorhanden, oder Kapazität unbekannt
- **CLI:** TTL wird in `get board.stats` angezeigt (nur im BAT-Modus, z.B. `TTL:288h`)
- **Telemetrie:** Wird als Tage via CayenneLPP Distance-Feld übertragen (max. 990 Tage für "unendlich")

### Solar-Energieverwaltung 🆕

> **⚠️ Solarpanel-Empfehlung: 5-6V Panels verwenden, KEINE 12V Panels!**
>
> Kleine 12V-Solarpanels (z.B. 5W/12V) sind für den BQ25798-Charger **schlecht geeignet**.
> Bei schwachem Licht (Sonnenaufgang, Bewölkung, Dämmerung) liefern sie hohe Leerlaufspannung
> (Voc ~17V) aber kaum Strom. Der Buck-Konverter muss von 17V auf ~3.7V herunterregeln
> (Verhältnis 4.6:1, Effizienz ~75%), wobei der Eigenverbrauch des BQ25798 (~10-20mA)
> plus Schaltverluste die gesamte Panelleistung auffressen. **Ergebnis: Der Akku wird
> ENTLADEN trotz aktivem Ladevorgang.**
>
> Ein **5-6V Panel gleicher Fläche** liefert bei gleicher Einstrahlung den **3-fachen Strom**
> bei 1/3 der Spannung. Der Buck-Ratio sinkt auf ~1.6:1 (Effizienz ~90%) — das Panel
> liefert selbst bei schwachem Licht nützliche Leistung.
>
> | Panel | Voc | Isc (schwach) | Buck-Ratio | Effizienz | BQ-Verlust |
> |-------|-----|---------------|------------|-----------|------------|
> | 6V/5W | ~7V | ~150mA | 1.9:1 | ~90% | ~3mA |
> | 12V/5W | ~17V | ~50mA | 4.6:1 | ~75% | ~12mA |

- **Solarstrom-Anzeige:** Der BQ25798 IBUS-ADC ist bei niedrigen Strömen ungenau (~±30mA Fehler). Daher wird der Solarstrom abgestuft angezeigt:
  - `0mA` — ADC meldet exakt 0 (kein Solarstrom)
  - `<50mA` — 1–49mA (ADC in diesem Bereich unzuverlässig)
  - `~72mA` — 50–100mA mit Rundungszeichen `~` (eingeschränkte Genauigkeit)
  - `385mA` — >100mA ohne Rundungszeichen (hinreichend genau)
  - Immer ganzzahlig ohne Dezimalstellen (keine Pseudopräzision)
- **HIZ-Gated Charging:** Default-Zustand = HIZ (charger aus, kein Batterie-Drain). BQ entscheidet via PG ob Eingang nutzbar. Bei PG=1 startet Coulomb-Counter-Monitoring (55s-Fenster). Erkennt parasitären Drain (ΔCharge < -0.05 mAh) → HIZ + 5 Minuten Cooldown.
- **PFM Forward Mode:** Manuell steuerbar via `set board.pfm 0|1` (persistent). Verbessert Effizienz bei niedrigen Strömen. Default: aus.
- **Stuck-PGOOD-Erkennung:** Erkennt automatisch hängendes PGOOD und triggert Input-Qualifizierung via HIZ-Toggle (5-Minuten-Cooldown, nur No-Battery-Fallback)
- **MPPT VOC_PCT 81.25%:** Der BQ25798-MPPT ist auf VOC_PCT=81.25% konfiguriert (statt Chip-Default 87.5% oder vormals 75%). Dieser Wert entspricht dem typischen Vmp/Voc-Verhältnis kristalliner Silizium-Solarzellen (~80-83%) und passt sowohl für 5-6V als auch 12V Panels.
- **MPPT-Recovery:** Aktiviert MPPT wieder bei PowerGood=1 (Readback-Check: nur bei tatsächlicher Änderung)
- **BQ INT-Pin nicht genutzt:** Kein Interrupt — reines Polling alle 60s in `runMpptCycle()`
- **Fehlerüberwachung:** Diagnosebefehle zeigen FAULT_STATUS-Register (0x20, 0x21) für detaillierte Analyse inkl. VBAT_OVP, VBUS_OVP und Temperaturbedingungen
- **VREG-Anzeige:** Zeigt die tatsächlich konfigurierte Battery-Regulation-Spannung in der Diagnose zur Schwellenwert-Prüfung

## Firmware-Build

```bash
platformio run -e Inhero_MR2_repeater
```

## CLI-Befehle

### Get-Befehle
```bash
get board.bat       # Aktuellen Batterietyp abfragen
                    # Ausgabe: liion1s | lifepo1s | lto2s

get board.hwver     # Hardware-Version abfragen
                    # Ausgabe: Rev 1.0 (INA228+RTC)
                    # Hinweis: MR2 ist immer Rev 1.0-Hardware

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
                    # Ausgabe: <24h>/<3d>/<7d>mAh C:<24h> D:<24h> 3dC:<3d> 3dD:<3d> 7dC:<7d> 7dD:<7d> <SOL|BAT> M:<mppt>% [TTL:<Stunden>h]
                    # Beispiel: +125.0/+45.0/+38.0mAh C:200.0 D:75.0 3dC:150.0 3dD:105.0 7dC:140.0 7dD:102.0 SOL M:85%
                    # Beispiel: -30.0/-45.0/-40.0mAh C:10.0 D:40.0 3dC:5.0 3dD:50.0 7dC:8.0 7dD:48.0 BAT M:45% TTL:72h
                    # Komponenten:
                    # - +125.0: Last 24h net balance (charge - discharge) in mAh
                    # - +45.0: 3-day average net balance in mAh
                    # - +38.0: 7-day average net balance in mAh
                    # - C/D: Charged/Discharged mAh (24h)
                    # - 3dC/3dD: 3-day average charged/discharged mAh
                    # - 7dC/7dD: 7-day average charged/discharged mAh
                    # - SOL: Running on solar (self-sufficient)
                    # - BAT: Living on battery (deficit mode)
                    # - M:85%: MPPT enabled percentage (7-day average)
                    # - TTL:72h: Time To Live (hours until empty, only shown if BAT mode, 7d-avg basis)

get board.cinfo     # Ladegerät-Info (BQ25798-Status) abfragen
                    # Ausgabe: <state> + flags
                    # States: !CHG, PRE, CC, CV, TRICKLE, TOP, DONE

get board.diag      # Detaillierte BQ25798-Diagnose abfragen
                    # Ausgabe: PG CE HIZ MPPT CHG VBUS VINDPM IINDPM | Spannungen | Temperaturen | Register | VOC-Konfig
                    # Beispiel: PG:1 CE:1 HIZ:0 MPPT:1 CHG:CC VBUS:UnkAdp VINDPM:1 IINDPM:0 | 
                    #          Vbus:6.22V Vbat:3.35V Ibat:0mA Temp:31C | 
                    #          TS: OK | R0F:0x23 R15:0xAB | VOC:87.5%/300ms/2min
                    # Key diagnostics for debugging charging issues:
                    # - PG: Power Good status (1=good, 0=no power)
                    # - CE: Charge Enable (1=enabled, 0=disabled)
                    # - HIZ: High Impedance mode (0=normal, 1=input disabled)
                    # - MPPT: Maximum Power Point Tracking (1=aktiv, 0=inaktiv)
                    # - CHG: Charge state (!CHG|TRKL|PRE|CC|CV|TOP|DONE)
                    # - VBUS: Input source type (NoIn|SDP|CDP|DCP|UnkAdp|NStd|NotQual|DirPwr)
                    # - VINDPM: Eingangs-Spannungs-DPM aktiv (1=limitierend, 0=ok)
                    # - IINDPM: Eingangs-Strom-DPM aktiv (1=limitierend, 0=ok)
                    # - VOC: MPPT VOC configuration (percentage/delay/rate)

get board.togglehiz # Force input detection via HIZ cycle 🆕
                    # Ausgabe: HIZ-Zyklus <war gesetzt|erzwungen>: VBUS=<V>V PG=<status>
                    # Same logic as automatic task in checkAndFixPgoodStuck()
                    # If HIZ=1: Clears HIZ → input qualification
                    # If HIZ=0: Set HIZ briefly, then clear → triggers input detection
                    # Always ends with HIZ=0
                    # Useful for manually triggering stuck PGOOD recovery

get board.conf      # Alle Konfigurationswerte abfragen
                    # Ausgabe: B:<bat> F:<fmax> M:<mppt> I:<imax> Vco:<voltage> V0:<0%SOC>

get board.iboffset  # INA228-Strom-Offset abfragen (v0.2-Feature)
                    # Ausgabe: INA228 offset: <+/-offset> mA (0.00=default)
                    # Korrigiert einen konstanten Offset der Strommessung

get board.batcap    # Batteriekapazität abfragen (v0.2-Feature)
                    # Ausgabe: <capacity> mAh (set) oder <capacity> mAh (default)
                    # Zeigt ob Kapazität manuell gesetzt oder Chemie-Default

get board.energy    # INA228 Coulomb Counter abfragen (v0.2-Feature)
                    # Ausgabe bei validem SOC: <charge>mAh (Base: <baseline>mAh, Net: <net>mAh)
                    # Ausgabe ohne SOC-Sync: <charge>mAh (SOC not synced)
                    # Fehler: Err: INA228 not initialized

get board.tccal     # NTC-Temperatur-Kalibrieroffset abfragen (v0.2-Feature)
                    # Ausgabe: TC offset: <+/-offset> C (0.00=default)

get board.leds      # LED-Aktivstatus abfragen (v0.2-Feature)
                    # Ausgabe: "LEDs: ON (Heartbeat + BQ Stat)" oder "LEDs: OFF (Heartbeat + BQ Stat)"
                    # Shows whether heartbeat LED and BQ25798 stat LED are enabled

get board.pfm       # PFM Forward Mode Status + HIZ-Gate-State abfragen
                    # Ausgabe: PFM: on [CHG] | PFM: off [HIZ]
                    # States: [HIZ] = HIZ-Idle, [CHG] = Charge-Active
```

### Set-Befehle
```bash
set board.bat <type>           # Batterietyp setzen
                               # Options: lto2s | lifepo1s | liion1s

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

set board.batcap <capacity>    # Batteriekapazität in mAh setzen (v0.2-Feature)
                               # Range: 100-100000 mAh
                               # Used for accurate SOC calculation

set board.iboffset <current_mA> # INA228-Strom-Offset kalibrieren (v0.2-Feature)
                               # Range: -2000 to +2000 mA
                               # Berechnet Offset: offset = actual - ina228_reading
                               # Beispiel: set board.iboffset 0.0
                               # (bei bekanntem 0mA-Zustand, korrigiert Nullpunktfehler)
                               # Ausgabe: INA228 offset: +1.25 mA
                               # Oder: set board.iboffset reset
                               # Setzt Offset auf +0.00 mA zurück

set board.tccal                # NTC-Temperatur kalibrieren (v0.2-Feature)
                               # Zwei Modi:
                               # 1) set board.tccal         → Auto-Kalibrierung via BME280
                               #    Ausgabe: TC auto-cal: BME=<temp> offset=<+/-offset> C
                               # 2) set board.tccal reset    → Offset auf 0.00 zurücksetzen
                               #    Ausgabe: TC calibration reset to 0.00 (default)

set board.leds <on|off>        # Enable/disable heartbeat + BQ stat LED (v0.2)
                               # on/1 = enable, off/0 = disable
                               # Boot-LEDs (3 blaue Blinks) immer aktiv

set board.pfm <0|1|on|off>     # PFM Forward Mode ein-/ausschalten (persistent)
                               # on/1 = PFM aktiv (besser für 5-6V Panels)
                               # off/0 = PFM deaktiviert (sicher für 12V Panels)

set board.soc <percent>        # SOC manuell setzen (v0.2-Feature)
                               # Bereich: 0-100
                               # Hinweis: INA228 muss initialisiert sein
```

## Diagnose & Fehlersuche

### BQ25798-Registerverifikation
Die Diagnosefunktionen ermöglichen präzise Verifikation der BQ25798-Register gegen das Datenblatt:

**Wichtige Register:**
- **0x0F (CHARGER_CONTROL_0)**: EN_HIZ (Bit 2), EN_CHG (Bit 5)
- **0x15 (MPPT_CONTROL)**: EN_MPPT (Bit 0), VOC_PCT (Bits 7-5), VOC_DLY (Bits 4-3), VOC_RATE (Bits 2-1)
- **0x1B (CHARGER_STATUS_0)**: PG_STAT (Bit 3), VINDPM (Bit 6), IINDPM (Bit 7)
- **0x1C (CHARGER_STATUS_1)**: CHG_STAT (Bits 7-5), VBUS_STAT (Bits 4-1)
- **0x1F (CHARGER_STATUS_4)**: Temperature status (Bits 3-0)

**Bekannte Probleme:**
1. **Stuck PGOOD**: Langsamer Sonnenaufgang kann Input-Qualifikation verhindern
   - Symptom: VBUS >3.5V aber PG=0
   - Lösung: `board.togglehiz` oder automatisch via `checkAndFixPgoodStuck()` (nur No-Battery-Fallback)
   - Mechanismus: HIZ-Toggle triggert Input-Qualifikation (per BQ25798 Datasheet)
2. **MPPT deaktiviert**: BQ25798 setzt MPPT=0 automatisch bei PG=0
   - Lösung: `checkAndFixSolarLogic()` reaktiviert MPPT bei PG=1
3. **Parasitärer Drain**: Panel liefert PG=1 aber Netto-Entladung (typisch für 12V Panels bei schwachem Licht)
   - Erkennung: Coulomb-Counter ΔCharge < -0.05 mAh über 55s-Fenster
   - Reaktion: HIZ + 5 Minuten Cooldown
   - CLI: `get board.cinfo` zeigt `HIZ-COOL` während Cooldown

### INA228-Kalibrierung (v0.2)

Die INA228-Strommessung kann einen **Offset-Fehler** (additiv, konstante Abweichung) aufweisen.
Die Rev 1.0 PCB-Kelvin-Traces minimieren Skalierungsfehler, daher ist nur die Offset-Kalibrierung nötig.

#### Offset-Kalibrierung (iboffset)

Im Ruhezustand des Repeaters (kein USB-Kabel, kein Solar) ein Multimeter in den Batteriekreis hängen und den Ruhestrom vergleichen:

```bash
# Multimeter zeigt: -9.5 mA (Entladestrom im Ruhezustand)
# INA228 zeigt:     -8.0 mA
# → Konstanter Offset von +1.5 mA

# Offset kalibrieren: den tatsächlichen DMM-Wert angeben
set board.iboffset -9.5
# Ausgabe: INA228 offset: -1.50 mA

# Prüfen:
get board.iboffset
# Ausgabe: INA228 offset: -1.50 mA (0.00=default)
```

Der Wert wird persistent gespeichert und bei jedem Boot geladen.
Zurücksetzen mit `set board.iboffset reset`.

## Siehe auch

- [QUICK_START.md](QUICK_START.md) - Schnellstart fuer Inbetriebnahme und CLI-Setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) - Alle board-spezifischen CLI-Befehle auf einen Blick
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
- [BATTERY_AUTO_LEARNING.md](BATTERY_AUTO_LEARNING.md) - Veraltet: historisches Auto-Learning-Konzept
