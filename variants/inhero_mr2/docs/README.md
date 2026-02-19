# Inhero MR-2 (Hardware v0.2)

## Inhaltsverzeichnis

- [Übersicht](#übersicht)
- [Aktuelle Feature-Matrix](#aktuelle-feature-matrix)
- [Energieverwaltungsfunktionen (v0.2)](#energieverwaltungsfunktionen-v02)
- [Firmware-Build](#firmware-build)
- [CLI-Befehle](#cli-befehle)
- [Diagnose & Fehlersuche](#diagnose--fehlersuche)
- [Siehe auch](#siehe-auch)

## Übersicht

Das Inhero MR-2 ist die zweite Generation des Mesh-Repeaters mit verbessertem Power-Management.

**Hardware-Version:** v0.2  
**Hauptmerkmale:**
- INA228 Power Monitor mit Coulomb Counter
- RV-3028-C7 RTC für Wake-up Management
- TPS62840 Buck Converter mit Hardware-UVLO
- BQ25798 Battery Charger mit MPPT

## Aktuelle Feature-Matrix

| Funktion | Status | Hinweis |
|---------|--------|---------|
| Spannungsüberwachung + Danger-Zone-Shutdown | Aktiv | Produktiv im Betrieb |
| Hardware-UVLO (INA228 Alert → TPS62840 EN) | Aktiv | Hardware-Schutz aktiv |
| RTC-Wakeup (SYSTEMOFF-Recovery) | Aktiv | 6h (Produktion) |
| SOC via INA228 + manuelle Batteriekapazität | Aktiv | `set board.batcap` verfügbar |
| SOC→Li-Ion mV Mapping (Workaround) | Aktiv | Wird entfernt wenn MeshCore SOC% nativ übermittelt |
| MPPT-Recovery + Stuck-PGOOD-Handling | Aktiv | Cooldown-Logik aktiv |
| Auto-Learning (Methode 1/2) | Veraltet | Aktuell nicht umgesetzt/aktiv |
| Erweiterte Auto-Learning-Reaktivierung | Geplant | Nur als zukünftige Aufgabe dokumentiert |

## Energieverwaltungsfunktionen (v0.2)

### Monitoring-Intervalle
Die Firmware nutzt feste Intervalle:
- **VBAT-Check**: 60s
- **RTC-Wake**: 6 Stunden (periodisch)

### 2-stufiges Schutzsystem
1. **Software-Spannungsüberwachung** - feste Strategie:
   - **Normalbetrieb**: 60s Checks (System läuft bereits)
   - **Danger Zone**: 6-Stunden-RTC-Wakeups (SYSTEMOFF, teurer Bootvorgang)
2. **Hardware-UVLO** - INA228 Alert → TPS62840 EN (absoluter Schutz auf Hardware-Ebene)
3. **SX1262 Power Control** - RadioLib `powerOff()` vor SYSTEMOFF (richtige Sleep-Methode)

**Kernpunkt:** Normale Checks kosten ~1µAh (INA228 I²C-Read), Danger-Zone-Wakeups kosten ~0.03mAh (vollständiger Systemstart ~10s @ ~10mA). Die Strategie maximiert die Batterielaufzeit.

### Spannungsschwellen (Li-Ion 1S)
- **Hardware-Cutoff (UVLO):** 3.1V (INA228 Alert → TPS62840 EN, 64-Sample-Averaging filtert TX-Spitzen)
- **Kritische Schwelle (0% SOC):** 3.4V (Danger-Zone-Grenze, Software-Shutdown)
- **Hysterese:** 300mV verhindert Motorboating und schafft Sicherheitsreserve bei Spannungseinbrüchen

### Coulomb Counter & Auto-Learning (veraltet)
- **Echtzeit-SOC-Tracking** via INA228 (±0.1% Genauigkeit)
- **100mΩ Shunt-Widerstand** (1.6A max Strom)
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

### Solar-Energieverwaltung 🆕
- **Stuck-PGOOD-Erkennung:** Erkennt automatisch langsame Sonnenaufgänge mit hängendem PGOOD und triggert Input-Qualifizierung via HIZ-Toggle (5-Minuten-Cooldown gegen übermäßiges Toggeln)
- **MPPT-Recovery:** Aktiviert MPPT wieder bei PowerGood=1 mit 60-Sekunden-Cooldown, um Interrupt-Loops zwischen Solarlogik und BQ25798-Interrupts zu vermeiden
- **Interrupt-Clearing:** Löscht stets BQ25798-Interrupt-Flags (CHARGER_STATUS_0 Register 0x1B), damit der Betrieb stabil bleibt und kein Interrupt-Lockup entsteht
- **Fehlerüberwachung:** Diagnosebefehle zeigen FAULT_STATUS-Register (0x20, 0x21) für detaillierte Analyse inkl. VBAT_OVP, VBUS_OVP und Temperaturbedingungen
- **VREG-Anzeige:** Zeigt die tatsächlich konfigurierte Battery-Regulation-Spannung in der Diagnose zur Schwellenwert-Prüfung

## Firmware-Build

```bash
platformio run -e Inhero_MR2_repeater
```

## CLI-Befehle

### Get-Befehle
```bash
board.bat       # Aktuellen Batterietyp abfragen
                # Ausgabe: liion1s | lifepo1s | lto2s

board.hwver     # Hardware-Version abfragen
                # Ausgabe: v0.2 (INA228+RTC)
                # Hinweis: MR2 ist immer v0.2-Hardware

board.fmax      # Frost-Ladeverhalten abfragen
                # Ausgabe: 0% | 20% | 40% | 100%
                # LTO batteries: N/A (JEITA disabled)

board.imax      # Maximalen Ladestrom abfragen
                # Ausgabe: <current>mA (z.B. 200mA)

board.mppt      # MPPT-Status abfragen
                # Ausgabe: MPPT=1 (aktiviert) | MPPT=0 (deaktiviert)

board.telem     # Echtzeit-Telemetrie mit SOC abfragen 🆕
                # Ausgabe: B:<V>V/<I>mA/<T>C SOC:<Prozent>% S:<V>V/~<I>mA
                # Beispiel: B:3.85V/125.4mA/22C SOC:68.5% S:5.12V/~245mA
                # Falls SOC nicht synchronisiert: B:3.85V/125.4mA/22C SOC:N/A S:5.12V/~245mA
                # Komponenten:
                # - B: Battery (Voltage/Current/Temperature/SOC)
                # - S: Solar (Voltage/Current, ~ = Schätzwert)

board.stats     # Energie-Statistiken (Bilanz + MPPT) abfragen 🆕
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
                # - TTL:72h: Time To Live (hours until empty, only shown if BAT mode)

board.cinfo     # Ladegerät-Info (BQ25798-Status) abfragen
                # Ausgabe: <state> + flags
                # States: !CHG, PRE, CC, CV, TRICKLE, TOP, DONE

board.diag      # Detaillierte BQ25798-Diagnose abfragen
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

board.togglehiz # Force input detection via HIZ cycle 🆕
                # Ausgabe: HIZ-Zyklus <war gesetzt|erzwungen>: VBUS=<V>V PG=<status>
                # Same logic as automatic task in checkAndFixPgoodStuck()
                # If HIZ=1: Clears HIZ → input qualification
                # If HIZ=0: Set HIZ briefly, then clear → triggers input detection
                # Always ends with HIZ=0
                # Useful for manually triggering stuck PGOOD recovery

board.uvlo      # UVLO-Einstellung abfragen (v0.2-Feature) 🆕
                # Ausgabe: ENABLED | DISABLED
                # Zeigt die Persistente UVLO-Einstellung
                # Chemie-spezifische UVLO-Schwellen:
                # - Li-Ion 1S: 3.1V
                # - LiFePO4 1S: 2.7V
                # - LTO 2S: 3.9V

board.conf      # Alle Konfigurationswerte abfragen
                # Ausgabe: B:<bat> F:<fmax> M:<mppt> I:<imax> Vco:<voltage> V0:<0%SOC>

board.ibcal     # INA228-Kalibrierfaktor abfragen (v0.2-Feature)
                # Ausgabe: INA228 calibration: <factor> (1.0=default)
                # Used to correct current measurement errors

board.iboffset  # INA228-Strom-Offset abfragen (v0.2-Feature)
                # Ausgabe: INA228 offset: <+/-offset> mA (0.00=default)
                # Korrigiert einen konstanten Offset der Strommessung

board.batcap    # Batteriekapazität abfragen (v0.2-Feature)
                # Ausgabe: <capacity> mAh (set) oder <capacity> mAh (default)
                # Zeigt ob Kapazität manuell gesetzt oder Chemie-Default

board.energy    # INA228 Coulomb Counter abfragen (v0.2-Feature)
                # Ausgabe bei validem SOC: <charge>mAh (Base: <baseline>mAh, Net: <net>mAh)
                # Ausgabe ohne SOC-Sync: <charge>mAh (SOC not synced)
                # Fehler: Err: INA228 not initialized

board.tccal     # NTC-Temperatur-Kalibrieroffset abfragen (v0.2-Feature)
                # Ausgabe: TC offset: <+/-offset> C (0.00=default)

board.leds      # LED-Aktivstatus abfragen (v0.2-Feature)
                # Ausgabe: "LEDs: ON (Heartbeat + BQ Stat)" oder "LEDs: OFF (Heartbeat + BQ Stat)"
                # Shows whether heartbeat LED and BQ25798 stat LED are enabled
```

### Set-Befehle
```bash
set board.bat <type>           # Batterietyp setzen
                               # Options: lto2s | lifepo1s | liion1s

set board.fmax <behavior>      # Frost-Ladeverhalten setzen
                               # Options: 0% | 20% | 40% | 100%
                               # N/A for LTO batteries

set board.imax <current>       # Maximalen Ladestrom in mA setzen
                               # Range: 10-1000mA

set board.mppt <1|0>           # Enable/disable MPPT
                               # 1 = enabled, 0 = disabled

set board.batcap <capacity>    # Batteriekapazität in mAh setzen (v0.2-Feature)
                               # Range: 100-100000 mAh
                               # Used for accurate SOC calculation

set board.ibcal <current_mA>   # INA228-Stromsensor kalibrieren (v0.2-Feature)
                               # Range: -2000 to +2000 mA
                               # Measures actual current, calculates correction factor
                               # Beispiel: set board.ibcal 100.5
                               # Ausgabe: INA228 calibrated: factor=0.9850
                               # Oder: set board.ibcal reset
                               # Setzt Kalibrierung auf Standardwert 1.0 zurück

set board.iboffset <current_mA> # INA228-Strom-Offset kalibrieren (v0.2-Feature)
                               # Range: -2000 to +2000 mA
                               # Berechnet Offset: offset = actual - ina228_reading
                               # Beispiel: set board.iboffset 0.0
                               # (bei bekanntem 0mA-Zustand, korrigiert Nullpunktfehler)
                               # Ausgabe: INA228 offset: +1.25 mA
                               # Oder: set board.iboffset reset
                               # Setzt Offset auf +0.00 mA zurück

set board.tccal [<temp_C>]     # NTC-Temperatur kalibrieren (v0.2-Feature)
                               # Drei Modi:
                               # 1) set board.tccal         → Auto-Kalibrierung via BME280
                               #    Ausgabe: TC auto-cal: BME=<temp> offset=<+/-offset> C
                               # 2) set board.tccal <temp_C> → Manueller Referenzwert (-40 bis +85°C)
                               #    Ausgabe: TC calibrated: offset=<+/-offset> C
                               # 3) set board.tccal reset    → Offset auf 0.00 zurücksetzen
                               #    Ausgabe: TC calibration reset to 0.00 (default)

set board.bqreset              # BQ25798 zurücksetzen und Konfiguration aus FS neu laden
                               # Performs software reset and reconfigures
                               # all settings from stored preferences

set board.leds <on|off>        # Enable/disable heartbeat + BQ stat LED (v0.2)
                               # on/1 = enable, off/0 = disable
                               # Boot-LEDs (3 blaue Blinks) immer aktiv

set board.uvlo <0|1|true|false> # UVLO-Einstellung setzen (v0.2-Feature) 🆕
                               # 0/false = DISABLED (Standard für Feldtests)
                               # 1/true = ENABLED (für kritische Anwendungen)
                               # Wird persistent gespeichert und bei nächstem Boot geladen
                               # Chemie-spezifische Schwellen werden automatisch angewendet

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
   - Lösung: `board.togglehiz` oder automatisch via `checkAndFixPgoodStuck()` (15min Intervall)
   - Mechanismus: HIZ-Toggle triggert Input-Qualifikation (per BQ25798 Datasheet)
2. **MPPT deaktiviert**: BQ25798 setzt MPPT=0 automatisch bei PG=0
   - Lösung: `checkAndFixSolarLogic()` reaktiviert MPPT bei PG=1 (60s Cooldown)

### INA228-Kalibrierung (v0.2)

Die INA228-Strommessung kann zwei Arten von Fehlern aufweisen:
- **Offset-Fehler** (additiv): Konstante Abweichung, unabhängig vom Strom
- **Skalierungsfehler** (multiplikativ): Proportionale Abweichung, wächst mit dem Strom

Für eine saubere Kalibrierung wird **zuerst der Offset** und **dann der Skalierungsfaktor** korrigiert.

#### Schritt 1: Offset-Kalibrierung (iboffset)

Im Ruhezustand des Repeaters (kein USB-Kabel, kein Solar) ein Multimeter in den Batteriekreis hängen und den Ruhestrom vergleichen:

```bash
# Multimeter zeigt: -9.5 mA (Entladestrom im Ruhezustand)
# INA228 zeigt:     -8.0 mA
# → Konstanter Offset von +1.5 mA

# Offset kalibrieren: den tatsächlichen DMM-Wert angeben
set board.iboffset -9.5
# Ausgabe: INA228 offset: -1.50 mA

# Prüfen:
board.iboffset
# Ausgabe: INA228 offset: -1.50 mA (0.00=default)
```

#### Schritt 2: Skalierungsfaktor-Kalibrierung (ibcal)

Nun ein USB-Ladekabel anschließen und den Ladestrom auf einen bekannten Wert begrenzen:

```bash
# Ladestrom begrenzen für saubere Messung:
set board.imax 100

# Multimeter zeigt: 95 mA (Ladestrom)
# INA228 zeigt:     93 mA (nach Offset-Korrektur)
# → Proportionaler Fehler von ~2%

# Skalierungsfaktor kalibrieren: den tatsächlichen DMM-Wert angeben
set board.ibcal 95
# Ausgabe: INA228 calibrated: factor=1.0215

# Prüfen:
board.ibcal
# Ausgabe: INA228 calibration: 1.0215 (1.0=default)
```

#### Zusammenfassung

| Kalibrierung | Korrigiert | Formel | Reihenfolge |
|---|---|---|---|
| `iboffset` | Konstanter Nullpunktfehler | `corrected = raw + offset` | **Zuerst** |
| `ibcal` | Proportionaler Skalierungsfehler | `corrected = raw × factor` | **Danach** |

Beide Werte werden persistent gespeichert und bei jedem Boot geladen.
Zurücksetzen mit `set board.iboffset reset` bzw. `set board.ibcal reset`.

## Siehe auch

- [QUICK_START.md](QUICK_START.md) - Schnellstart fuer Inbetriebnahme und CLI-Setup
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
- [BATTERY_AUTO_LEARNING.md](BATTERY_AUTO_LEARNING.md) - Veraltet: historisches Auto-Learning-Konzept
