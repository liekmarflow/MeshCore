# Inhero MR2 – CLI Cheat-Sheet

> 🇬🇧 [English version](../CLI_CHEAT_SHEET.md)

Alle board-spezifischen CLI-Befehle auf einen Blick.
Präfix ist immer `board.` – also `get board.<cmd>` bzw. `set board.<cmd> <wert>`.

Siehe [FAQ.md](FAQ.md) für Erläuterungen der wichtigsten Parameter (`imax`, `fmax`, `batcap`) und [DATASHEET.md](DATASHEET.md#unterstützte-akkuchemien) für Chemie-Details.

---

## Setter (Konfiguration ändern)

```bash
# Akkuchemie
set board.bat liion1s          # Li-ion 1S (3.7V nominal)
set board.bat lifepo1s         # LiFePO4 1S (3.2V nominal)
set board.bat lto2s            # LTO 2S (2x 2.3V nominal)
set board.bat naion1s          # Na-ion 1S (3.1V nominal)
set board.bat none             # Kein Akku / unbekannt (Laden deaktiviert)

# Akkukapazität (100–100000 mAh)
# Faustregel: 90% der Nennkapazität (siehe FAQ #4)
set board.batcap 10000

# Maximaler Ladestrom (50–1500 mA)
set board.imax 500

# Frost-Ladestromabsenkung (T-Cool ca. -2 °C bis +3 °C, siehe JEITA-Tabelle im README)
set board.fmax 0%              # Laden gesperrt
set board.fmax 20%             # max. 20% von imax
set board.fmax 40%             # max. 40% von imax
set board.fmax 100%            # keine Reduktion
# Bei LTO / Na-ion abgelehnt:  "Err: Fmax setting N/A for this chemistry (JEITA disabled)"
# Bei aktivem JEITA-Override abgelehnt:  "Err: Fmax N/A while jeitaignore is on"
# Ohne gesetzte Chemie abgelehnt:  "Err: Set board.bat first"

# JEITA-Override — Laden unterhalb der T-Cold-Schwelle (ca. -2 °C) fortsetzen
# Nur Li-ion / LiFePO4; standardmäßig aus; auf eigenes Risiko (siehe Abschnitt unten)
set board.jeitaignore 1        # TS-Pin ignorieren (1/true)
set board.jeitaignore 0        # zurück zum Hardware-JEITA (0/false)

# MPPT ein/aus
set board.mppt 1               # MPPT aktivieren
set board.mppt 0               # MPPT deaktivieren

# LEDs ein/aus (Heartbeat + BQ-Stat)
set board.leds on              # LEDs aktivieren  (on/1)
set board.leds off             # LEDs deaktivieren (off/0)

# SOC manuell setzen (0–100%)
set board.soc 85.0

# Unbekannter Setter:  "Err: bat|imax|fmax|mppt|batcap|tccal|leds|soc|jeitaignore"
```

### Kalibrierung

```bash
# NTC-Temperatur-Kalibrierung
# Best Practice: Am frühen Morgen vor Sonnenaufgang ausführen,
# wenn sich die Akkutemperatur an die Umgebung angeglichen hat (siehe FAQ #12).
set board.tccal                # Auto-Kalibrierung via BME280
set board.tccal reset          # Offset auf 0.00 zurücksetzen
```

---

## Getter (Status abfragen)

```bash
# Konfiguration & Hardware
get board.bat                  # Aktueller Akkutyp
get board.batcap               # Akkukapazität in mAh (set/default)
get board.imax                 # Maximaler Ladestrom in mA
get board.fmax                 # Frost-Ladeverhalten (0%/20%/40%/100%, oder N/A bei
                               #   aktivem JEITA-Override)
get board.jeitaignore          # Zustand des JEITA-Overrides (siehe Abschnitt unten)
get board.mppt                 # MPPT-Status (0/1)
get board.leds                 # LED-Status (ON/OFF)
get board.conf                 # Kurzübersicht aller Konfigs (B, F, M, I, Vco, V0)
                               #   zusätzlich " J:1" bei aktivem JEITA-Override
                               #   auf einer Chemie, die JEITA nutzt

# Echtzeit-Telemetrie
get board.telem                # Battery+Solar: V, I, T, SOC

# Energie & Statistik
get board.stats                # Energie-Bilanz (24h/3d/7d), C/D, MPPT%, Batt-TTL
                               #   Batt-TTL = Battery Time-To-Live (Stunden bis Akku leer),
                               #   kein Hop-Limit
                               #   Basis: 7-Tage-Durchschnitt des tägl. Netto-Defizits
                               #   aus stündlichen INA228-Coulomb-Counter-Samples (168h-Ringpuffer)
                               #   Formel: entnehmbare Ladung / |7d-Avg-Defizit| × 24, wobei
#   entnehmbar = SOC% × Kapazität − eingeschlossene Ladung
#   (Kälte-Derating; bei Normaltemperatur 0)
                               #   Batt-TTL erscheint nur im BAT-Modus (Netto-Defizit)
                               #   Voraussetzung: mind. 24h Daten + Kapazität bekannt

# Ladegerät & Diagnose
get board.cinfo                # Charger-Status + letzter PG-Stuck HIZ-Toggle
get board.bqdiag               # Diagnose/Debug: kompakter BQ25798-Register-Dump
                               #   PG-/Ladezustand, TS-Region (COLD/COOL/WARM/HOT),
                               #   aktive Status-/Fehler-Flags (z.B. VINDPM, VBAT_OVP)
get board.selftest             # Alle I2C-Komponenten prüfen (INA228/BQ25798/RV-3028/BME280)
                               #   Ausgabe: "INA:OK BQ:OK RTC:OK BME:OK"
                               #   RTC inkl. User-RAM Write/Readback-Verifikation
                               #   — erkennt kalte Lötstellen (Chip ACKt, akzeptiert
                               #   aber keine Writes). Mögliche Werte je Gerät:
                               #     OK      — antwortet (RTC: Write persistiert)
                               #     NACK    — keine I2C-Antwort
                               #     WR_FAIL — (nur RTC) ACKt, aber Write/Read stimmen nicht überein
get board.socdebug             # Diagnose/Debug: SOC-Tracking-Interna
                               #   SHUNT_CAL, präziser Strom, CHARGE-Register (mAh),
                               #   Stundenzähler Laden/Entladen, Update-Zähler,
                               #   RTC-Zeit, Temperatur-Derating-Faktor

# Kalibrierung
get board.tccal                # NTC-Temperatur-Offset in °C (0.00 = default)

# Unbekannter Getter:
#   "Err: bat|fmax|imax|mppt|telem|stats|cinfo|conf|tccal|leds|batcap|jeitaignore"
#   bqdiag, selftest und socdebug funktionieren, stehen aber nicht in dieser Liste
```

---

## Getter-Kurzinfos

| Befehl | Beschreibung |
|---|---|
| `get board.bat` | Akkutyp (`liion1s`, `lifepo1s`, `lto2s`, `naion1s`, `none`) |
| `get board.batcap` | Akkukapazität in mAh (set/default) |
| `get board.imax` | Maximaler Ladestrom in mA |
| `get board.fmax` | Frost-Ladeverhalten (`0%`/`20%`/`40%`/`100%`; `N/A` bei aktivem JEITA-Override) |
| `get board.jeitaignore` | Zustand des JEITA-Overrides — `jeitaignore 1`, `jeitaignore 0`, `jeitaignore 1 (chemistry)`, eine blockierte Variante oder `N/A`, solange keine Chemie gesetzt ist |
| `get board.mppt` | MPPT-Status (`0`/`1`) |
| `get board.leds` | LED-Status Heartbeat + BQ-Stat (`ON`/`OFF`) |
| `get board.conf` | Kurzübersicht: B(at) F(max) M(ppt) I(max) Vco V0, zusätzlich `J:1` bei Li-ion / LiFePO4 mit aktivem JEITA-Override; bei `none` lautet die ganze Antwort `B:none (no battery, charging disabled)` |
| `get board.telem` | Echtzeit-Telemetrie: Battery/Solar V, I, T, SOC — siehe [TELEMETRY.md](TELEMETRY.md) |
| `get board.stats` | Energie-Bilanz (24h/3d/7d), C/D, MPPT%, Batt-TTL (7d-Avg-basiert) |
| `get board.cinfo` | Charger-Status + PG-Stuck HIZ-Toggle (z.B. "PG / CC HIZ:3m ago") |
| `get board.bqdiag` | Diagnose/Debug: BQ25798-Register-Dump — PG-/Ladezustand, TS-Region, aktive Fehler-Flags |
| `get board.selftest` | I2C-Komponenten-Probe — `INA:OK BQ:OK RTC:OK BME:OK` (RTC inkl. Write-Verify) |
| `get board.socdebug` | Diagnose/Debug: SOC-Interna — SHUNT_CAL, Strom, CHARGE, Stundenzähler, Derating-Faktor |
| `get board.tccal` | NTC-Temperatur-Offset in °C (`0.00` = default) |

---

## Setter-Kurzinfos

| Befehl | Wertebereich | Beschreibung |
|---|---|---|
| `set board.bat` | `liion1s` · `lifepo1s` · `lto2s` · `naion1s` · `none` | Akkuchemie wählen — leitet den JEITA-Override neu ab und setzt `fmax` bei Li-ion / LiFePO4 auf `0%` zurück |
| `set board.batcap` | `100`–`100000` (mAh) | Akkukapazität setzen — zugleich Bezugsgröße für das `jeitaignore`-Gate |
| `set board.imax` | `50`–`1500` (mA) | Max. Ladestrom setzen — zugleich Gate-Größe für `jeitaignore` |
| `set board.fmax` | `0%` · `20%` · `40%` · `100%` | Frost-Ladestromabsenkung (abgelehnt bei LTO/Na-ion und bei aktivem `jeitaignore`) |
| `set board.jeitaignore` | `1`/`0` · `true`/`false` | JEITA-Override, nur Li-ion/LiFePO4 — Gate: `batcap` gesetzt und `imax` ≤ 0,05C |
| `set board.mppt` | `0`/`1` · `true`/`false` | MPPT ein-/ausschalten |
| `set board.leds` | `on`/`off` · `1`/`0` | LEDs ein-/ausschalten |
| `set board.soc` | `0`–`100` (%) | SOC manuell setzen |
| `set board.tccal` | `reset` · *(leer = auto)* | NTC-Temperatur kalibrieren oder zurücksetzen |

---

## JEITA-Override (`board.jeitaignore`)

`set board.jeitaignore 1` setzt das TS_IGNORE-Bit des BQ25798. Der Laderegler behandelt den
TS-Pin damit als immer in Ordnung und lädt auch unterhalb der T-Cold-Schwelle von ca. -2 °C
weiter. Damit der Override wirkt, müssen zwei Bedingungen erfüllt sein:

- `set board.batcap` wurde ausdrücklich geschrieben, und
- `imax` liegt bei höchstens 0,05C dieser Kapazität (300 mA bei 6000 mAh, 500 mA bei 10000 mAh).

Die Einstellung bleibt auch dann gespeichert, wenn das Gate nicht erfüllt ist; ein kleineres
`imax` oder eine größere `batcap` schaltet sie von selbst wieder scharf, und die Antwort dieser
Befehle sagt es. LTO 2S und Na-ion 1S laufen ohnehin ohne JEITA und lehnen den Befehl ab.

TS_IGNORE übergeht alle vier TS-Bereiche, damit entfällt auch die Ladeabschaltung auf der warmen
Seite bei rund +58 °C, und `fmax` bleibt wirkungslos, solange der Override an ist. Unter dem
Gefrierpunkt führt das Laden von Li-ion oder LiFePO4 zu kumulativem, dauerhaftem Lithium-Plating
an der Anode. Das 0,05C-Gate begrenzt die Rate; den Mechanismus beseitigt es nicht, der Override
läuft also auf eigenes Risiko. Die vollständige Darstellung — Felderfahrung, Gegenargumente und
Quellen — steht im [BATTERY_GUIDE.md](BATTERY_GUIDE.md).

```bash
set board.jeitaignore 1
#  "jeitaignore set to 1"                              — Gate erfüllt, Override aktiv
#  "jeitaignore set to 1, N/A, C>0.05"                 — imax über 0,05C; gespeichert, wirkt später
#  "jeitaignore set to 1, N/A, batcap not set"         — keine batcap; gespeichert, wirkt später
#  "Err: This chemistry runs without JEITA (always 1)" — lto2s / naion1s
#  "Err: Set board.bat first"                          — noch keine Chemie gesetzt
#  "Err: Use 1|0"                                      — Argument ist nicht 1|0|true|false
#  "Err: Failed to store setting"                      — Schreiben in den Flash fehlgeschlagen

set board.jeitaignore 0
#  "jeitaignore set to 0"                              — gespeichertes fmax-Verhalten gilt wieder

get board.jeitaignore
#  "jeitaignore 1 (chemistry)"                         — lto2s / naion1s
#  "N/A"                                               — noch keine Chemie gesetzt
#  "jeitaignore 1"                                     — Override aktiv
#  "jeitaignore 1, N/A, C>0.05"                        — gesetzt, durch imax blockiert
#  "jeitaignore 1, N/A, batcap not set"                — gesetzt, blockiert mangels batcap
#  "jeitaignore 0"                                     — nicht gesetzt

# imax und batcap sind Gate-Größen — ihre Antwort nennt einen Zustandswechsel:
set board.imax 500
#  "Max charge current set to 500mA"                          — keine Änderung
#  "Max charge current set to 500mA; jeitaignore 1"           — Override wieder scharf
#  "Max charge current set to 500mA; jeitaignore N/A, C>0.05" — Override verloren

set board.batcap 10000
#  "Battery capacity set to 10000 mAh"                          — keine Änderung
#  "Battery capacity set to 10000 mAh; jeitaignore 1"           — Override wieder scharf
#  "Battery capacity set to 10000 mAh; jeitaignore N/A, C>0.05" — Override verloren
```

Das tatsächlich gesetzte Bit lässt sich mit `get board.bqdiag` nachlesen: die Antwort endet mit
`N:<hex>`, dem rohen NTC_CONTROL_1-Register — ein ungerader Wert bedeutet, dass TS_IGNORE
programmiert ist.

---

## Schnellstart-Rezepte

### Li-ion 1S mit 10Ah und Solar
```bash
set board.bat liion1s
set board.batcap 10000
set board.imax 500
set board.fmax 20%
set board.mppt 1
set board.leds off
```

### LiFePO4 1S mit 6Ah und Solar
```bash
set board.bat lifepo1s
set board.batcap 6000
set board.imax 300
set board.fmax 40%
set board.mppt 1
set board.leds off
```

### LTO 2S mit 18Ah und Solar
```bash
set board.bat lto2s
set board.batcap 18000
set board.imax 700
set board.mppt 1
set board.leds off
```

### Na-ion 1S mit 10Ah und Solar
```bash
set board.bat naion1s
set board.batcap 10000
set board.imax 500
set board.mppt 1
set board.leds off
```

### Li-ion 1S mit 10Ah, Solar und JEITA-Override
```bash
set board.bat liion1s
set board.batcap 10000         # Bezugsgröße — vor jeitaignore setzen
set board.imax 500             # 0,05C von 10000 mAh — genau auf der Gate-Grenze
set board.jeitaignore 1        # "jeitaignore set to 1"
set board.mppt 1
set board.leds off
# set board.fmax wird bei aktivem Override abgelehnt; get board.fmax liefert N/A
```

### Status-Check (alles auf einen Blick)
```bash
get board.conf
get board.telem
get board.stats
get board.cinfo
```

---

## Siehe auch

- [README.md](README.md) — Übersicht, Feature-Matrix und Diagnose
- [DATASHEET.md](DATASHEET.md) — Hardware-Spezifikationen und Pinout
- [TELEMETRY.md](TELEMETRY.md) — Telemetrie-Kanäle erklärt (was die App anzeigt)
- [QUICK_START.md](QUICK_START.md) — Schnellstart für Inbetriebnahme und CLI-Setup
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Akkuchemie-Vergleich und Einsatzempfehlungen
- [FAQ.md](FAQ.md) — Häufig gestellte Fragen
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Vollständige technische Dokumentation
