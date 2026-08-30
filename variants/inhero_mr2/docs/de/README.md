# Inhero MR2

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

Das Inhero MR2 ist eine anwendungsspezifische Hardware-Plattform für den autarken Dauerbetrieb von Mesh-Infrastruktur an schwer erreichbaren Standorten. Der aktive Idle-Verbrauch beträgt 6,0 mA bei 4,2 V bzw. 7,7 mA bei 3,3 V (USB aus, kein Radio-TX) — das gibt einem voll ausgestatteten Repeater lange Akkulaufzeit mit einer kompakten Zelle und einem kleinen Solarpanel. Ein universeller Solareingang mit aktivem MPPT erntet über einen weiten Eingangsbereich, sodass eine kompakte, unauffällige Installation in der Regel genügt und die Peripherie nicht überdimensioniert werden muss. Li-ion, LiFePO4, LTO und Na-ion werden nativ unterstützt, und die RTC-Recovery holt das Board nach einer Unterspannungsabschaltung selbsttätig zurück, sodass ein leerer Akku keinen Einsatz vor Ort erzwingt. Das senkt die langfristigen Betriebskosten an Orten, an denen ein manueller Wartungseinsatz teuer ist, weil der Standort schwer erreichbar ist.

**Hardware-Version:** Rev 1.1  
**Hauptmerkmale:**
- **Core:** Basierend auf RAK4630**(H)** (nRF52840 + SX1262) — Hochband-Variante für IN865, EU868, US915, AU915, KR920 und AS923. Die Niederband-Variante RAK4630(L) (EU433 / CN470) ist nicht verbaut, ein Betrieb auf 433 MHz / 470 MHz ist damit nicht möglich. Siehe [DATASHEET.md — LoRa-Frequenzbänder](DATASHEET.md#lora-frequenzbänder).
- **Power-Path:** BQ25798 Buck/Boost Charger. Ermöglicht Energiegewinnung auch dann, wenn die Solarspannung unter der Akkuspannung liegt (wichtig für Schwachlicht-Phasen).
- **High-Efficiency Rail:** 3.3V-Rail via TPS62840 für maximale Effizienz.
- **Robustes Monitoring:** INA228 Coulomb-Counter für präzises SOC-Tracking (essenziell für LiFePO4-Chemie) und Langzeit-Energiestatistiken.
- **Universeller Solareingang:** 3.6V – 24V mit autonomem MPPT-Tracking und integriertem Schutz gegen „Stuck-States“ (Hardware-Watchdog-Logik).
- **Umweltsensorik & Zeit:** Integrierter BME280 und RV-3028 RTC für autonomes Wake-up Management und präzise Zeitbasis. Siehe [FAQ #23](FAQ.md#23-warum-braucht-das-repeater-board-eine-korrekte-uhrzeit) warum eine korrekte Uhrzeit wichtig ist.
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
| SOC via INA228 + manuelle Akkukapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |
| PFM Forward Mode | Aktiv (Chip-Default) | Ab Werk im BQ25798 aktiv (PFM_FWD_DIS=0, REG0x12); die Firmware ändert ihn nicht. Verbessert Effizienz bei niedrigen Solarströmen |
| JEITA-Override (`set board.jeitaignore`) | Verfügbar, standardmäßig aus | Nur Li-ion 1S / LiFePO4 1S; wird aktiv, solange `board.batcap` gesetzt ist und `board.imax` höchstens 0,05C beträgt |

## Energieverwaltungsfunktionen

### Low-Voltage-Handling (Flag/Tick-Architektur)

1. **INA228 ALERT** feuert bei `lowv_sleep_mv` (Hardware-Interrupt auf P1.02)
2. **ISR** setzt `lowVoltageAlertFired = true` (nur volatile Flag, kein FreeRTOS-Aufruf)
3. **`tickPeriodic()`** (Main-Loop, nächster `tick()`) prüft Flag → Shutdown:
   - CE-Pin → HIGH (FET ON → CE LOW → Laden aktiv)
   - P0.04 von `disconnectLeakyPullups()` ausgeschlossen → GPIO-Latch bleibt im Sleep erhalten
   - RTC-Wake konfiguriert (`LOW_VOLTAGE_SLEEP_MINUTES` = 60 min)
   - `sd_power_system_off()` → **System Sleep mit GPIO-Latch** (< 500µA)
4. **RTC-Wake** (stündlich) → System bootet, Early-Boot prüft VBAT:
   - Unter `lowv_wake_mv` → sofort wieder System Sleep (CE bleibt gelatcht LOW)
   - Über `lowv_wake_mv` → normaler Boot, SOC startet bei 0%

> **Hinweis**: Alle I2C-Operationen (MPPT, SOC, Hourly Stats) laufen im Main-Loop-Kontext
> über `tickPeriodic()` — keine FreeRTOS-Tasks für I2C, kein Mutex nötig.

### BQ CE-Pin (Rev 1.1 — FET-invertiert)
- **CE-N-FET**: Gate ← GPIO4 (ext. Pull-Down), Drain → CE, Source → GND
- **GPIO HIGH** → FET ON → CE LOW → **Laden AN** (BQ25798 CE active-low)
- **GPIO LOW / High-Z** → ext. Pull-Down am Gate → FET OFF → Pull-Up am CE → CE HIGH → **Laden AUS**
- **System Sleep**: GPIO4-Latch HIGH erhalten (von `disconnectLeakyPullups()` ausgeschlossen) → FET ON → CE LOW → **Solar-Laden aktiv**
- **Safety-Default**: RAK stromlos/ungeflasht → Pull-Down am Gate → FET OFF → CE HIGH → **Laden deaktiviert**
- **Dual-Layer**: CE-Pin (Hardware-FET) + `setChargeEnable()` (I2C Register)

### Spannungsschwellen (alle Chemien)

| Chemie | lowv_sleep_mv | lowv_wake_mv | Hysterese |
|--------|--------------|-------------|-----------|
| Li-ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |
| Na-ion 1S | 2500 | 2700 | 200mV |

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

### SOC→Li-ion mV Mapping (Workaround)
- **Problem**: MeshCore überträgt nur `getBattMilliVolts()`, keinen SOC%. Die MeshCore-App nutzt eine Li-ion-Kurve zur SOC-Berechnung — falsche Anzeige bei LiFePO4/LTO.
- **Lösung**: Bei validem Coulomb-Counting-SOC wird eine äquivalente Li-ion 1S OCV (3000–4200 mV) zurückgegeben, sodass die App den korrekten SOC% anzeigt. Siehe [TELEMETRY.md](TELEMETRY.md) für Details zur App-Anzeige.
- Dieser Workaround wird entfernt, sobald MeshCore die native Übertragung des SOC% unterstützt.
- → [FAQ #11 — SOC zeigt 0% oder N/A?](FAQ.md#11-warum-zeigt-der-soc-0-oder-na-an)

### Batt-TTL-Prognose

> **Batt-TTL** steht für *Battery Time-To-Live* — die geschätzte Restlaufzeit im Akkubetrieb. Gemeint ist nicht das Hop-Limit, das „TTL“ im Mesh-Netz bezeichnet.
- **Zeitbasis:** 7-Tage gleitender Durchschnitt (`avg_7day_daily_net_mah`) des täglichen Netto-Energieverbrauchs
- **Datenbasis:** 168-Stunden-Ringpuffer (7 Tage) mit stündlichen INA228-Coulomb-Counter-Samples (charged/discharged/solar mAh)
- **Formel:** `TTL_hours = max(0, SOC% × capacity_mah / 100 − capacity_mah × (1 − f(T))) / |avg_7day_daily_net_mah| × 24`
- **f(T):** Kälte-Derating-Faktor (Trapped-Charge-Modell) — bei niedrigen Temperaturen ist ein Teil der gespeicherten Ladung nicht nutzbar und wird vorab abgezogen; f(T) = 1 bei warmen Temperaturen
- **Voraussetzungen:** `living_on_battery == true` (24h-Defizit), mind. 24h Daten, Kapazität bekannt
- **Batt-TTL = 0:** Solar-Überschuss, keine 24h Daten vorhanden, oder Kapazität unbekannt
- **CLI:** Batt-TTL wird in `get board.stats` angezeigt (nur im BAT-Modus, z.B. `BT:12d0h`)
- **Telemetrie:** Wird als Tage via CayenneLPP Distance-Feld übertragen (max. 990 Tage für "unendlich"). Siehe [TELEMETRY.md](TELEMETRY.md) für Kanal-Details.

### Solar-Energieverwaltung

- **Solarstrom-Anzeige:** Der BQ25798 IBUS-ADC ist bei niedrigen Strömen ungenau (~±30mA Fehler). Daher wird der Solarstrom abgestuft angezeigt:
  - `0mA` — ADC meldet exakt 0 (kein Solarstrom)
  - `<50mA` — 1–49mA (ADC in diesem Bereich unzuverlässig)
  - `~72mA` — 50–100mA mit Rundungszeichen `~` (eingeschränkte Genauigkeit)
  - `385mA` — >100mA ohne Rundungszeichen (hinreichend genau)
  - Immer ganzzahlig ohne Dezimalstellen (keine Pseudopräzision)
- **PFM Forward Mode:** PFM-Forward-Modus ist ab Werk im BQ25798 aktiv (PFM_FWD_DIS=0, REG0x12); die Firmware ändert ihn nicht. Verbessert Effizienz bei niedrigen Strömen.
- **MPPT VOC_PCT 81.25%:** Der BQ25798-MPPT ist auf VOC_PCT=81.25% konfiguriert (statt Chip-Default 87.5%). Dieser Wert entspricht dem typischen Vmp/Voc-Verhältnis kristalliner Silizium-Solarzellen (~80-83%).
- **MPPT-Recovery:** Aktiviert MPPT wieder bei PowerGood=1 (Readback-Check: nur bei tatsächlicher Änderung)
- **BQ INT-Pin nicht genutzt:** Kein Interrupt — reines Polling alle 60s in `runMpptCycle()`
- **Fehlerüberwachung:** Diagnosebefehle zeigen FAULT_STATUS-Register (0x20, 0x21) für detaillierte Analyse inkl. VBAT_OVP, VBUS_OVP und Temperaturbedingungen
- **VREG-Anzeige:** Zeigt die tatsächlich konfigurierte Battery-Regulation-Spannung in der Diagnose zur Schwellenwert-Prüfung

### JEITA-Temperaturzonen-Konfiguration

Der BQ25798 nutzt den TS-Pin (NTC-Thermistor) für JEITA-konforme temperaturabhängige Laderegelung. Das Inhero MR2 verwendet einen Spannungsteiler (RT1=5,6 kΩ Pullup an REGN, RT2=27 kΩ parallel zu GND), der TS-Schwellen nach unten verschiebt gegenüber dem TI-Referenzdesign (5,24 kΩ / 30,31 kΩ). Die Verschiebung ist **temperaturabhängig**: ~5–6 °C im Kaltbereich, ~2–3 °C im Warm-/Hot-Bereich (da bei niedrigen Temperaturen der NTC-Widerstand groß ist relativ zu RT2 und den Teiler-Unterschied verstärkt).

| JEITA-Zone | BQ25798-Schwelle | TI-Referenz | Inhero MR2 (real) | Shift | Firmware-Konfig |
|------------|------------------|-------------|--------------------|---------|-----------------|
| T-Cold (Ladung gesperrt) | VT1 = 72,0% REGN | +3,7 °C | −2,0 °C | −5,7 °C | Schwelle fest |
| T-Cool (reduzierter Strom) | VT2 = 69,8% REGN | +7,9 °C | +2,8 °C | −5,1 °C | `set board.fmax` |
| T-Warm Start | VT3 = 37,7% REGN | +54,5 °C | +52,2 °C | −2,3 °C | `TS_WARM = 55°C` Register-Einstellung |
| T-Hot (Ladung gesperrt) | VT5 = 34,2% REGN | +59,9 °C | +57,7 °C | −2,2 °C | Schwelle fest |

> Solange der JEITA-Override aktiv ist, zwingt TS_IGNORE = 1 alle vier TS-Statusbits auf 000: Jede Zone dieser Tabelle wird übergangen, und die `board.fmax`-Reduktion bleibt wirkungslos.

> Berechnung basiert auf: NTC 103AT (B25/50=3435) für TI-Referenz, NCP15XH103F03RC (B25/85=3380) für Inhero. Typische %REGN-Werte aus BQ25798-Datenblatt.

**Wichtige Firmware-Einstellungen in `configureBaseBQ()`:**

- **`TS_WARM = 55°C`** (BQ-Registerwert): Verschiebt die WARM-Zonen-Schwelle vom Default 45 °C (44,8% REGN, ~41,8 °C mit Inhero-Teiler) auf 37,7% REGN (~52,2 °C mit Inhero-Teiler). Verhindert vorzeitigen WARM-Zonen-Eintritt bei moderaten Temperaturen.
- **`JEITA_VSET = UNCHANGED`**: Keine Reduktion der Battery-Regulation-Spannung in der WARM-Zone. Der POR-Default (VREG−400 mV) würde VREG auf 3,1 V für LiFePO4 reduzieren und VBAT_OVP bei normalen Akkuspannungen (3,3–3,5 V) auslösen.
- **`JEITA_ISETH = ICHG unchanged`** (POR-Default, beibehalten): Keine Ladestrom-Reduktion in der WARM-Zone. Zusammen mit JEITA_VSET=UNCHANGED ist die WARM-Zone effektiv neutralisiert — Ladung läuft mit voller Spannung und vollem Strom weiter.
- **`AUTO_IBATDIS = deaktiviert`**: Deaktiviert die automatische 30-mA-Akkuentladung des BQ25798 während VBAT_OVP. Der POR-Default entlädt den Akku aktiv mit ~30 mA (IBAT_LOAD) bei OVP — kontraproduktiv für solarbetriebene Systeme.

> **Hintergrund:** Mit den BQ25798-Standardeinstellungen verursachte die Kombination aus Inhero-Teiler-Offset und LiFePO4-Chemie eine Fehlerkette bei ~42 °C: WARM-Zonen-Eintritt → VREG auf 3,1 V reduziert → VBAT_OVP (Akku bei 3,47 V > 104% × 3,1 V) → aktive 30-mA-Entladung → netto −45 mA Verbrauch trotz Solareinspeisung. Die obigen Einstellungen verhindern dies vollständig. Die WARM-Zone (52–58 °C mit Inhero-Teiler) hat nun keinen Einfluss auf das Ladeverhalten.

**JEITA-Override (`set board.jeitaignore`)**

`set board.jeitaignore 1` setzt das TS_IGNORE-Bit des BQ25798 (NTC Control 1, Register 0x18, Bit 0). Der Lader behandelt den TS-Pin dann durchgehend als ladefreundlich, sodass auch unterhalb der T-Cold-Schwelle weitergeladen wird. Standardmäßig aus.

- **Chemie:** akzeptiert für Li-ion 1S und LiFePO4 1S. LTO 2S und Na-ion 1S brauchen keine JEITA-Überwachung (`needs_jeita = false` in der Chemie-Tabelle) und laufen ohnehin mit gesetztem TS_IGNORE; dort antwortet der Befehl `Err: This chemistry runs without JEITA (always 1)`.
- **Gate:** Der Override wird nur aktiv, solange `board.batcap` ausdrücklich geschrieben wurde und `board.imax` höchstens 0,05C dieser Kapazität beträgt. Schreibzugriffe auf `imax` oder `batcap` leiten den Zustand neu ab, und die Antwort benennt die Änderung.
- **Gespeichert wird der Wunsch:** Das Flag liegt im NVS, der wirksame Zustand wird bei jeder Chemie-Anwendung neu abgeleitet und nie gespeichert. Ein nicht bestandenes Gate verwirft das Flag nicht — der Override rastet von selbst wieder ein, sobald `imax`/`batcap` wieder passen. Vom Einschalten bis zum Anwenden der gespeicherten Konfiguration ist TS_IGNORE gelöscht und der Hardware-Temperaturschutz zuständig.
- **Reichweite des Schalters:** Mit gesetztem TS_IGNORE melden alle vier TS-Statusbits 000, damit entfällt auch die Ladesperre bei T-Hot (~+57,7 °C mit dem Inhero-Teiler). Für keine der beiden Grenzen bietet die Firmware einen Ersatz — das Board schläft im SYSTEMOFF mit aktiviertem Lader, dort läuft keine Regelschleife; die 0,05C-Grenze ist das gesamte Sicherheitsargument.
- **Risiko:** Das Laden von Li-ion oder LiFePO4 bei Frost erfolgt auf eigenes Risiko des Betreibers. Die 0,05C-Grenze begrenzt die Rate; Lithium-Plating an der Graphitanode bleibt kumulativ und dauerhaft und zeigt sich als still verschwundene Kapazität. Feldbelege, Temperaturbereich und die vollständige Abwägung stehen im [BATTERY_GUIDE.md](BATTERY_GUIDE.md).

## Firmware-Build

```bash
# Repeater (Standard)
platformio run -e Inhero_MR2_repeater

# Repeater mit RS232-Bridge (Serial2 an P0.19/P0.20)
platformio run -e Inhero_MR2_repeater_bridge_rs232

# Sensor
platformio run -e Inhero_MR2_sensor
```

## CLI-Befehle

### Get-Befehle
```bash
get board.bat       # Aktuellen Akkutyp abfragen
                    # Ausgabe: liion1s | lifepo1s | lto2s | naion1s | none

get board.fmax      # Frost-Ladeverhalten abfragen
                    # Ausgabe: 0% | 20% | 40% | 100%
                    # Wert = maximaler Ladestrom im T-Cool-Bereich
                    # (ca. -2 °C bis +3 °C, siehe JEITA-Tabelle im README),
                    # relativ zu board.imax
                    # 40% bei imax=500mA → max. 200mA Ladestrom im T-Cool-Bereich
                    # 0% = Laden im T-Cool-Bereich gesperrt
                    # 100% = keine Reduktion (voller Strom auch bei Kälte)
                    # Unter ca. -2 °C (T-Cold): Laden gesperrt (JEITA), außer
                    # der JEITA-Override ist aktiv — siehe
                    # set board.jeitaignore
                    # Hinweis: Nur das Laden wird eingeschränkt. Bei ausreichend
                    # Solar wird das Board weiterhin mit Solarstrom betrieben —
                    # der Akku wird weder ge- noch entladen.
                    # Ausgabe ist N/A, solange der JEITA-Override aktiv ist:
                    # Chemien ohne JEITA (LTO / Na-ion / none) oder ein
                    # eingerastetes set board.jeitaignore

get board.imax      # Maximalen Ladestrom abfragen
                    # Ausgabe: <current>mA (z.B. 200mA)

get board.mppt      # MPPT-Status abfragen
                    # Ausgabe: MPPT=1 (aktiviert) | MPPT=0 (deaktiviert)

get board.telem     # Echtzeit-Telemetrie mit SOC abfragen
                    # Ausgabe: B:<V>V/<I>mA/<T>C SOC:<Prozent>% S:<V>V/<SolarStrom>
                    # Beispiele:
                    #   B:3.85V/125.4mA/22C SOC:68.5% S:5.12V/385mA      (>100mA: genau)
                    #   B:3.85V/-8.2mA/18C SOC:72.0% S:4.90V/~72mA      (50-100mA: ~Schätzwert)
                    #   B:3.30V/-45.0mA/5C SOC:40.1% S:0.00V/<50mA      (<50mA: ADC ungenau)
                    # Ausgabevarianten:
                    # - SOC:N/A — kein gültiger Coulomb-Counting-SOC vorhanden
                    # - SOC:68.5% (52%) — zweiter Wert = kälte-deratierter SOC,
                    #   erscheint wenn der Temperatur-Derating-Faktor < 1 ist
                    # - <T>C wird zu N/A wenn keine NTC-Temperatur verfügbar ist
                    # Komponenten:
                    # - B: Battery (Voltage/Current/Temperature/SOC)
                    # - S: Solar (Voltage/Strom — Genauigkeit abhängig vom BQ25798 IBUS-ADC)

get board.stats     # Energie-Statistiken (Bilanz + MPPT) abfragen
                    # Ausgabe: <24h>/<3d>/<7d>mAh C:<24h> D:<24h> 3C:<3d> 3D:<3d> 7C:<7d> 7D:<7d> <SOL|BAT> M:<mppt>% BT:<ttl>
                    # Beispiel: +125/+45/+38mAh C:200 D:75 3C:150 3D:105 7C:140 7D:102 SOL M:85% BT:N/A
                    # Beispiel: -30/-45/-40mAh C:10 D:40 3C:5 3D:50 7C:8 7D:48 BAT M:45% BT:72h
                    # Komponenten:
                    # - +125: Netto-Bilanz der letzten 24h (Ladung - Entladung) in mAh
                    # - +45: 3-Tage-Durchschnitt der Netto-Bilanz in mAh
                    # - +38: 7-Tage-Durchschnitt der Netto-Bilanz in mAh
                    # - C/D: Geladene/Entladene mAh (24h)
                    # - 3C/3D: 3-Tage-Durchschnitt geladene/entladene mAh
                    # - 7C/7D: 7-Tage-Durchschnitt geladene/entladene mAh
                    # - SOL: Läuft auf Solar (selbstversorgend)
                    # - BAT: Lebt vom Akku (Defizit-Modus)
                    # - M:85%: MPPT-Aktivquote in Prozent (7-Tage-Durchschnitt)
                    # - BT:72h: Batt-TTL (nur im BAT-Modus, Basis 7-Tage-Durchschnitt)
                    #   Format: BT:12d5h (≥24h) oder BT:72h (<24h) oder BT:N/A

get board.cinfo     # Ladegerät-Info + letzter PG-Stuck HIZ-Toggle
                    # Ausgabe: <state> + flags
                    # States: !CHG, PRE, CC, CV, TRICKLE, TOP, DONE

get board.selftest  # I²C-Hardware-Probe (alle Onboard-Komponenten)
                    # Ausgabe: INA:<state> BQ:<state> RTC:<state> BME:<state>
                    # States: OK | NACK | WR_FAIL (nur RTC, Write-Verify-Mismatch)

get board.conf      # Alle Konfigurationswerte abfragen
                    # Ausgabe: B:<bat> F:<fmax> M:<mppt> I:<imax> Vco:<voltage> V0:<0%SOC>
                    # F: zeigt N/A, solange der JEITA-Override aktiv ist
                    # Ein angehängtes J:1 erscheint, solange der Benutzer-
                    # Override eingerastet ist (nur Li-ion / LiFePO4)

get board.batcap    # Akkukapazität abfragen
                    # Ausgabe: <capacity> mAh (set) oder <capacity> mAh (default)
                    # Zeigt ob Kapazität manuell gesetzt oder Chemie-Default

get board.tccal     # NTC-Temperatur-Kalibrieroffset abfragen
                    # Ausgabe: TC offset: <+/-offset> C (0.00=default)

get board.leds      # LED-Aktivstatus abfragen
                    # Ausgabe: "LEDs: ON (Heartbeat + BQ Stat)" oder "LEDs: OFF (Heartbeat + BQ Stat)"
                    # Zeigt ob Heartbeat-LED und BQ25798-Status-LED aktiviert sind

get board.jeitaignore  # Zustand des JEITA-Overrides abfragen
                       # Ausgabe: jeitaignore 0
                       #   → Override aus
                       # Ausgabe: jeitaignore 1
                       #   → Override aktiv
                       # Ausgabe: jeitaignore 1 (chemistry)
                       #   → LTO / Na-ion laufen ohne JEITA
                       # Ausgabe: jeitaignore 1, N/A, C>0.05
                       #   → Flag gespeichert, imax über 0,05C von batcap
                       # Ausgabe: jeitaignore 1, N/A, batcap not set
                       #   → Flag gespeichert, batcap nie geschrieben
```

### Set-Befehle
```bash
set board.bat <type>           # Akkutyp setzen
                               # Optionen: lto2s | lifepo1s | liion1s | naion1s | none
                               # none = kein Akku / unbekannt (Laden deaktiviert)

set board.fmax <behavior>      # Frost-Ladeverhalten setzen
                               # Optionen: 0% | 20% | 40% | 100%
                               # Begrenzt Ladestrom im T-Cool-Bereich
                               # (ca. -2 °C bis +3 °C, siehe JEITA-Tabelle im README)
                               # auf X% von board.imax
                               # 0% = Laden im T-Cool-Bereich gesperrt
                               # 20% = max. 20% von imax im T-Cool-Bereich
                               # 40% = max. 40% von imax im T-Cool-Bereich
                               # 100% = keine Reduktion
                               # Unter ca. -2 °C (T-Cold): Laden gesperrt (JEITA),
                               # außer der JEITA-Override ist aktiv —
                               # siehe set board.jeitaignore
                               # Hinweis: Nur das Laden wird eingeschränkt. Bei
                               # ausreichend Solar läuft das Board weiterhin auf
                               # Solarstrom — der Akku wird weder ge- noch entladen.
                               # Abgelehnt bei Chemien ohne JEITA
                               # (LTO / Na-ion):
                               #   Err: Fmax setting N/A for this chemistry (JEITA disabled)
                               # Abgelehnt, solange der Override aktiv ist:
                               #   Err: Fmax N/A while jeitaignore is on

set board.imax <current>       # Maximalen Ladestrom in mA setzen
                               # Bereich: 50-1500mA (BQ25798-Minimum: 50mA)
                               # imax gehört zum jeitaignore-Gate: ändert der
                               # Schreibzugriff den Override-Zustand, hängt die
                               # Antwort "; jeitaignore 1" oder
                               # "; jeitaignore N/A, C>0.05" an

set board.mppt <1|0>           # MPPT aktivieren/deaktivieren
                               # 1 = aktiviert, 0 = deaktiviert

set board.batcap <capacity>    # Akkukapazität in mAh setzen
                               # Bereich: 100-100000 mAh
                               # Wird für genaue SOC-Berechnung verwendet
                               # batcap gehört zum jeitaignore-Gate: ändert der
                               # Schreibzugriff den Override-Zustand, hängt die
                               # Antwort "; jeitaignore 1" oder
                               # "; jeitaignore N/A, C>0.05" an

set board.tccal                # NTC-Temperatur kalibrieren
                               # Zwei Modi:
                               # 1) set board.tccal         → Auto-Kalibrierung via BME280
                               #    Ausgabe: TC auto-cal: BME=<temp> offset=<+/-offset> C
                               # 2) set board.tccal reset    → Offset auf 0.00 zurücksetzen
                               #    Ausgabe: TC calibration reset to 0.00 (default)

set board.leds <on|off>        # Heartbeat- + BQ-Status-LED aktivieren/deaktivieren
                               # on/1 = aktivieren, off/0 = deaktivieren
                               # Boot-LEDs folgen dieser Einstellung; nur der
                               # Low-Voltage-Recovery-Blitz (3 blaue Blinks)
                               # ist immer aktiv

set board.soc <percent>        # SOC manuell setzen
                               # Bereich: 0-100
                               # Hinweis: INA228 muss initialisiert sein

set board.jeitaignore <1|0>    # Laden unterhalb der JEITA-T-Cold-Schwelle erlauben
                               # 1 = Lader ignoriert den TS-Pin (TS_IGNORE)
                               # 0 = JEITA-Temperaturregelung aktiv (Standard)
                               # Nur Li-ion 1S / LiFePO4 1S. LTO / Na-ion:
                               #   Err: This chemistry runs without JEITA (always 1)
                               # Gate: board.batcap muss ausdrücklich gesetzt sein
                               # und board.imax höchstens 0,05C davon betragen
                               # Antworten: jeitaignore set to 1
                               #            jeitaignore set to 1, N/A, C>0.05
                               #            jeitaignore set to 1, N/A, batcap not set
                               #            jeitaignore set to 0
                               # Das Flag bleibt gespeichert, wenn das Gate nicht
                               # passt, und rastet wieder ein, sobald imax/batcap
                               # passen
                               # Der Override entfernt auch die Ladesperre auf der
                               # heißen Seite (~ +57,7 °C). Das Laden von Li-ion
                               # oder LiFePO4 bei Frost erfolgt auf eigenes Risiko
                               # des Betreibers — siehe BATTERY_GUIDE.md
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

→ [FAQ #9 — Rote LED blinkt / Akku wird nicht geladen](FAQ.md#9-die-rote-led-bq-status-led-blinkt-langsam-und-der-akku-wird-nicht-geladen)

## Regulatorische Hinweise & CE-Konformität (RED 2014/53/EU)

Das Inhero MR2 wird als Hardware-Plattform (Entwicklungsmodul) mit vorinstalliertem Bootloader ausgeliefert.

**Was das Board festlegt und was der Betreiber wählt:** Die Funkhardware kann alles, was das verbaute Modul RAK4630**(H)** hergibt — IN865, EU868, US915, AU915, KR920, AS923 (siehe [DATASHEET.md — LoRa-Frequenzbänder](DATASHEET.md#lora-frequenzbänder)). Ausgeschlossen ist einzig das Niederband: RAK4630(L) (EU433, CN470) ist nicht verbaut, 433 MHz und 470 MHz stehen damit nicht zur Verfügung. Alles Übrige — **Frequenz, Sendeleistung und Duty Cycle — wird in der Firmware eingestellt, nicht vom Board vorgegeben**. Welche Grenzwerte gelten, ergibt sich daher daraus, wo das Board betrieben wird, und ihre Einhaltung liegt beim Betreiber.

Firmware-Vorgabewerte sind eine Entscheidung der Firmware und können sich von Version zu Version ändern. MeshCore sendet auf diesem Board derzeit auf 869,618 MHz (`LORA_FREQ=869.618`), einem Wert innerhalb des EU-g3-Subbands. Das ist ein Startwert, den man gegen die am Standort geltenden Regeln prüft — keine Eigenschaft der Hardware.

### Betrieb in der EU (CE / RED 2014/53/EU)

Die Hardware ist **CE-gekennzeichnet und konform zur europäischen Funkanlagenrichtlinie (RED 2014/53/EU)**; die zugehörigen Prüfungen wurden durch ein akkreditiertes Prüflabor durchgeführt, und eine EU-Konformitätserklärung liegt vor. Die Zertifizierung der abgestrahlten Leistung erfolgte unter Verwendung der vorgesehenen Referenzantennen (RAK FPCB-Antenne 863–870 MHz, MHF1-Anschluss, Antennengewinn: 0,7 dBi).

Da die finale Sendecharakteristik (Sendeleistung, Frequenz, Duty Cycle) maßgeblich von der durch den Anwender installierten Software (z. B. MeshCore) und der gewählten Antenne abhängt, muss jeder, der das Board in der EU betreibt, es so konfigurieren, dass die folgenden europäischen Grenzwerte (gemäß EN 300 220 und EN 300 328, ERC/REC 70-03 Annex 1) eingehalten werden:

1. **Standard LoRa (868 MHz Band):**
   * Frequenzbereich: 865,0 – 868,6 MHz
   * Max. abgestrahlte Sendeleistung (ERP): 25 mW (14 dBm)
   * Max. Duty Cycle: 1 % (oder LBT+AFA gemäß EN 300 220)
   * *Hinweis: Im Sub-Band 863,0 – 865,0 MHz gelten abweichende Duty-Cycle-Anforderungen gemäß ERC/REC 70-03.*

2. **High-Power LoRa (g3-Subband, Bereich 869,5 MHz):**
   * Frequenzbereich: 869,40 – 869,65 MHz — jeder Kanal innerhalb von g3 erfüllt das; die konkrete Frequenz ist eine Firmware-Einstellung
   * Max. abgestrahlte Sendeleistung (ERP): **500 mW (27 dBm)**
   * Max. Duty Cycle: 10 %
   * *Hinweis zur Hardware:* Der verbaute LoRa-Transceiver (SX1262) liefert eine maximale leitungsgebundene Sendeleistung von 22 dBm. Um das gesetzliche Limit von 500 mW ERP (entspricht 29,15 dBm EIRP) vollständig auszuschöpfen, ist die Verwendung einer entsprechenden Antenne mit einem Antennengewinn von ca. +7 dBi erforderlich (abzüglich etwaiger Kabelverluste). Mit der mitgelieferten FPCB-Antenne (0,7 dBi) werden max. ca. 114 mW ERP erreicht.

3. **Bluetooth Low Energy (2,4 GHz):**
   * Max. abgestrahlte Sendeleistung (EIRP): 100 mW (20 dBm)

**Antennen & Verantwortung des Betreibers (EIRP/ERP Limit):**
Der Anwender ist verpflichtet, die konfigurierte Sendeleistung (TX Power) im Chip und den Antennengewinn aufeinander abzustimmen. Wird eine Antenne verwendet, deren Gewinn in Kombination mit der eingestellten Sendeleistung die oben genannten gesetzlichen EIRP/ERP-Limits überschreitet, muss die Sendeleistung softwareseitig zwingend reduziert werden.

### Betrieb außerhalb der EU

Das verbaute (H)-Modul deckt die zu Beginn dieses Abschnitts genannten regionalen Bänder ab, das Board ist also auch außerhalb Europas einsetzbar. Die Werte im vorangehenden Abschnitt sind die **europäischen** Grenzwerte und gelten dort nicht. Die vorliegende EU-Konformitätserklärung deckt den Betrieb innerhalb der EU ab; in jeder anderen Region gelten die dortigen nationalen Funkvorschriften — Frequenzplan, abgestrahlte Leistung sowie Duty Cycle bzw. Dwell Time. Ihre Einhaltung einschließlich einer örtlich erforderlichen Zulassung liegt beim Integrator bzw. Anwender.

**USA (FCC):** Das verbaute Modul RAK4630**(H)** besitzt eine modulare Zulassung nach FCC 15.212 (47 CFR Part 15 Subpart C) — FCC ID `2AF6B-RAK4630`, erteilt am 27.11.2020 an Shenzhen RAKwireless Technology Co., Ltd. für die Modelle RAK4630 und RAK4631. Die Zulassung hat das Modul; das MR2-Board selbst hat keine eigene Zulassung. Sie deckt das Modul in der Konfiguration ab, die im Grant festgehalten ist — die dort genannten Frequenzbereiche (LoRa 902,3–914,9 MHz sowie 903,0–914,2 MHz, BLE 2402,0–2480,0 MHz; siehe [DATASHEET.md — LoRa-Frequenzbänder](DATASHEET.md#lora-frequenzbänder)) und die Antennen seiner Antennenliste. Alles außerhalb dieser Konfiguration ist davon nicht abgedeckt; dafür ist der Host-Hersteller bzw. der Betreiber verantwortlich. Ob ein konkreter Einsatz zulässig ist, folgt nicht allein aus der Modulzulassung. Die Auflagen, die das OEM-Manual an die Zulassung knüpft, gelten für jedes Host-Gerät, das das Modul trägt:

* **Kennzeichnung des Host-Geräts:** Das Host-Gerät muss die Kennzeichnung `Contains FCC ID: 2AF6B-RAK4630` tragen.
* **RF-Exposition:** Es ist ein Mindestabstand von 20 cm zum Körper einzuhalten.
* **Antennen:** Der Grant führt die Antennen auf, mit denen das Modul zugelassen wurde, jeweils mit einem maximalen Gewinn. Dieser Wert steht hier nicht — die eigene Antennenauswahl gegen die Antennenliste zu prüfen, ist Sache des Betreibers.

**Kanada (ISED):** Dasselbe Modul ist in Kanada nach RSS-247, Issue 2 (Februar 2017) zertifiziert — Zertifizierungsnummer `25908-RAK4630`, erteilt am 16.08.2021 von Bay Area Compliance Laboratories an Shenzhen RAKwireless Technology Co., Ltd. (HVIN RAK4630). Das Zertifikat hält dieselben Frequenzbereiche fest wie der FCC-Grant: LoRa 902,3–914,9 MHz sowie 903,0–914,2 MHz, BLE 2402–2480 MHz. Wie bei der FCC-Zulassung hat das Modul die Zertifizierung und nicht das Board. Sie deckt das Modul in der im Zertifikat festgehaltenen Konfiguration ab; für alles außerhalb davon ist der Host-Hersteller bzw. der Betreiber verantwortlich. Ein Host-Gerät, das das Modul trägt, muss die Kennzeichnung `Contains IC: 25908-RAK4630` tragen.

**Australien (RCM):** RAK hält für das Modul eine RCM-Lieferantenerklärung. Diese berechtigt Inhero nicht, das RCM auf dem MR2 anzubringen oder Konformität für Australien zu erklären. Nach den ACMA-Regeln ist der Supplier ein australischer Hersteller, Importeur oder Agent, und er muss vor dem Anbringen des Zeichens in der nationalen Datenbank registriert sein — eine Rolle, die ein deutscher Hersteller nicht selbst ausfüllen kann. Das MR2 trägt daher kein RCM, und zur australischen Konformität wird hier nichts ausgesagt.

**Antennen und die getestete Konfiguration:** Annex A des ISED-Zertifikats nennt die Antennen, mit denen die kanadische Zertifizierung erteilt wurde: für LoRa einen Dipol mit 3,0 dBi, für BLE eine PCB-Antenne mit 2,23 dBi. Eine Antenne mit höherem Gewinn führt aus der getesteten Konfiguration heraus; die Bewertung dieses Falls liegt dann beim Betreiber. Der Wert stammt aus dem ISED-Zertifikat und ist keine FCC-Auflage — der FCC-Grant führt eine eigene Antennenliste in Abschnitt 2.7 des OEM-Manuals, die hier nicht verifiziert ist und separat geprüft werden muss. Der U.FL-Anschluss des MR2 lässt jede Antenne zu: Das ist eine Eigenschaft des Boards, keine Erlaubnis.

Welche Zulassungen das Modul hat und was jede davon für das MR2 bedeutet, steht in [DATASHEET.md — Zulassungen des Funkmoduls](DATASHEET.md#zulassungen-des-funkmoduls).

**Haftungsausschluss:**
Das Inhero MR2 ist ein Modul für professionelle Entwickler und qualifizierte Anwender. Wird das Gerät durch die Wahl der Firmware, der Antenne oder durch manuelle Konfiguration außerhalb der oben genannten EU-Grenzwerte betrieben, erlischt seine CE-Konformität. In diesem Fall geht die gesamte rechtliche Verantwortung für den Betrieb auf den Integrator bzw. Anwender über.

## Siehe auch

- [DATASHEET.md](DATASHEET.md) — Hardware-Spezifikationen und Pinout
- [TELEMETRY.md](TELEMETRY.md) — Telemetrie-Kanäle erklärt (was die App anzeigt)
- [QUICK_START.md](QUICK_START.md) — Schnellstart für Inbetriebnahme und CLI-Setup
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Akkuchemie-Vergleich und Einsatzempfehlungen
- [FAQ.md](FAQ.md) — Häufig gestellte Fragen
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle board-spezifischen CLI-Befehle auf einen Blick
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Vollständige technische Dokumentation
