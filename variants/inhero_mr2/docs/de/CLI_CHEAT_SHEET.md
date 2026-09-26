# Inhero MR2 – CLI Cheat-Sheet

> 🇬🇧 [English version](../CLI_CHEAT_SHEET.md)

Alle board-spezifischen CLI-Befehle auf einen Blick.
Präfix ist immer `board.` – also `get board.<cmd>` bzw. `set board.<cmd> <wert>`.

Siehe [FAQ.md](FAQ.md) für Erläuterungen der wichtigsten Parameter (`imax`, `fmax`, `batcap`) und [DATASHEET.md](DATASHEET.md#unterstützte-akkuchemien) für Chemie-Details.

---

## Setter (Konfiguration ändern)

Ein Wechsel zu einer **anderen Akkuchemie**, auch nach oder von `none`, setzt die Akku- und Ladekonfiguration zurück: `imax` auf 200 mA, MPPT aus, `fmax` auf 0% und den Benutzer-JEITA-Override aus. Die Kapazität fällt auf den Chemie-Default (1500 mAh bei LiFePO4, sonst 2000 mAh) zurück und gilt nicht mehr als ausdrücklich gesetzt. Der SOC wird ungültig und bleibt bis zu einer neuen Referenz unbekannt. Ladespannung und Low-V-Schwellen folgen der neuen Chemie. Deshalb zuerst die Chemie, danach Kapazität und Ladeparameter setzen. Das erneute Setzen derselben Chemie und normale Neustarts erhalten gültige Einstellungen. LEDs, Aufstellhöhe und NTC-Kalibrierung bleiben erhalten.

```bash
# Akkuchemie
set board.bat liion1s          # Li-ion 1S (3.7V nominal)
set board.bat lifepo1s         # LiFePO4 1S (3.2V nominal)
set board.bat lto2s            # LTO 2S (2x 2.3V nominal)
set board.bat naion1s          # Na-ion 1S (3.1V nominal)
set board.bat none             # Kein Akku / unbekannt (Laden deaktiviert)

# Akkukapazität (100–100000 mAh)
# Speicherung und Gate-Prüfung verwenden eine Nachkommastelle (mAh).
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

# Aufstellhöhe (-500 bis 9000 m); Druck auf Kanal 2 wird dadurch zu QNH
set board.altitude 312
set board.altitude clear       # Höhe löschen; wieder Stationsdruck ausgeben

# LEDs ein/aus (Heartbeat + BQ-Stat)
set board.leds on              # LEDs aktivieren  (on/1)
set board.leds off             # LEDs deaktivieren (off/0)

# SOC manuell setzen (0–100%)
set board.soc 85.0

# Unbekannter Setter:  "Err: bat|imax|fmax|mppt|altitude|batcap|tccal|leds|soc|jeitaignore"
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
get board.mppt                 # Gespeicherte MPPT-Einstellung (0/1)
get board.mpptdiag             # MPPT Soll/Ist, VSYSMIN, VINDPM, VSYS_MIN,
                               # PG, CELL, ICHG, VREG (Spannungen mV, Strom mA)
get board.altitude             # Aufstellhöhe für die BME280-QNH-Korrektur
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
#   "Err: bat|fmax|imax|mppt|altitude|telem|stats|cinfo|conf|tccal|leds|batcap|jeitaignore"
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
| `get board.jeitaignore` | Zustand des JEITA-Overrides — `jeitaignore 1`, `jeitaignore 0`, `jeitaignore 1 (chemistry)`, `N/A`, solange keine Chemie gesetzt ist |
| `get board.mppt` | MPPT-Status (`0`/`1`) |
| `get board.altitude` | Aufstellhöhe in Metern oder `N/A (station pressure)`, solange die QNH-Korrektur nicht konfiguriert ist |
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
| `set board.bat` | `liion1s` · `lifepo1s` · `lto2s` · `naion1s` · `none` | Andere Chemie setzt Akku-/Ladewerte auf Default; gleiche Chemie bleibt unverändert |
| `set board.batcap` | `100`–`100000` (mAh) | Akkukapazität setzen — zugleich Bezugsgröße für das `jeitaignore`-Gate |
| `set board.imax` | `50`–`1500` (mA) | Max. Lade- und Vorladestrom für alle ladbaren Akkuchemien setzen — zugleich Gate-Größe für `jeitaignore` |
| `set board.fmax` | `0%` · `20%` · `40%` · `100%` | Frost-Ladestromabsenkung (abgelehnt bei LTO/Na-ion und bei aktivem `jeitaignore`) |
| `set board.jeitaignore` | `1`/`0` · `true`/`false` | JEITA-Override, nur Li-ion/LiFePO4 — Gate: `batcap` gesetzt und `imax` < 0,05C |
| `set board.mppt` | `0`/`1` · `true`/`false` | MPPT ein-/ausschalten |
| `set board.altitude` | `-500`–`9000` (m) · `clear` | Aufstellhöhe speichern und den BME280-Druck auf Kanal 2 als QNH ausgeben oder die Höhe löschen und zum Stationsdruck zurückkehren |
| `set board.leds` | `on`/`off` · `1`/`0` | LEDs ein-/ausschalten |
| `set board.soc` | `0`–`100` (%) | SOC manuell setzen |
| `set board.tccal` | `reset` · *(leer = auto)* | NTC-Temperatur kalibrieren oder zurücksetzen |

Das Vorladeregister des BQ25798 hat 40-mA-Schritte; der Vorladestrom wird auf den nächsten Schritt abgerundet (`imax 200` ergibt 200 mA, `imax 500` ergibt 480 mA). Die separate Trickle-Charge-Phase unterhalb der Tiefentladeschwelle bleibt unverändert.

---

## JEITA-Override (`board.jeitaignore`)

Der Override erfordert eine ausdrücklich gesetzte `board.batcap` und **`imax < 0,05C`**. Gleichheit wird abgelehnt: Bei 10000 mAh bestehen 450 mA das Gate, 500 mA nicht. `set board.jeitaignore 1` antwortet bei fehlender Kapazitätsangabe mit `N/A, batcap not set`, bei zu hohem Strom mit `N/A, imax >=0,05C`. Diese Gate-Abweisungen (`N/A, …`) ändern weder Einstellungen noch Hardware und speichern keinen vorgemerkten Wunsch. Solange der Benutzer-Override an ist, wird eine Änderung von `imax` oder `batcap`, die das Gate verletzen würde, mit `N/A, jeitaignore=1` abgelehnt; alle bisherigen Werte bleiben erhalten. Für eine solche Änderung zuerst den Override ausschalten. Eine spätere Parameteränderung aktiviert ihn niemals von selbst wieder.

Das Einschalten des Overrides verwirft einen individuellen `fmax` und speichert den Default 0%. Solange er an ist, liefert `get board.fmax` `N/A`; `set board.fmax` wird mit `Err: Fmax N/A while jeitaignore is on` abgelehnt. Beim Wechsel von `jeitaignore 1` auf `0` gilt wieder Hardware-JEITA mit `fmax=0%`; kein alter Frostwert kehrt zurück. Ein erneutes `set board.jeitaignore 0` bei bereits ausgeschaltetem Override erhält einen inzwischen neu eingestellten `fmax`. `get board.conf` hängt bei aktivem Benutzer-Override ` J:1` an.

TS_IGNORE übergeht alle vier TS-Bereiche, damit entfällt auch die Ladeabschaltung auf der warmen
Seite bei rund +58 °C, und `fmax` bleibt wirkungslos, solange der Override an ist. Unter dem
Gefrierpunkt führt das Laden von Li-ion oder LiFePO4 zu kumulativem, dauerhaftem Lithium-Plating
an der Anode. Das 0,05C-Gate begrenzt die Rate; den Mechanismus beseitigt es nicht, der Override
läuft also auf eigenes Risiko. Die vollständige Darstellung — Felderfahrung, Gegenargumente und
Quellen — steht im [BATTERY_GUIDE.md](BATTERY_GUIDE.md).

```bash
set board.jeitaignore 1
#  "jeitaignore set to 1"                              — angenommen, Override an
#  "N/A, imax >=0,05C"                                 — abgewiesen, Strom zu hoch
#  "N/A, batcap not set"                              — abgewiesen, Kapazität nicht gesetzt
#  "Err: This chemistry runs without JEITA (always 1)" — lto2s / naion1s
#  "Err: Set board.bat first"                          — keine Chemie gesetzt
#  "Err: Use 1|0"                                     — ungültiges Argument
#  "Err: Failed to store setting"                 — Speicherung oder Hardware-Anwendung fehlgeschlagen

set board.jeitaignore 0
#  "jeitaignore set to 0"                              — 1 -> 0 setzt fmax auf 0%
#  Wiederholtes 0 bei bereits ausgeschaltetem Override erhält einen neuen fmax.

get board.jeitaignore
#  "jeitaignore 1 (chemistry)"                         — lto2s / naion1s
#  "N/A"                                              — none
#  "jeitaignore 1"                                    — akzeptierte Benutzereinstellung: ein
#  "jeitaignore 0"                                    — akzeptierte Benutzereinstellung: aus

# Beispiel: Override an, batcap 10000 mAh, imax 450 mA
set board.imax 500
#  "N/A, jeitaignore=1"                               — unverändert: 450 mA
set board.batcap 9000
#  "N/A, jeitaignore=1"                               — unverändert: 10000 mAh
set board.imax 400
#  "Max charge current set to 400mA"                  — zulässig
set board.batcap 12000
#  "Battery capacity set to 12000 mAh"                — zulässig
```

`get board.jeitaignore` zeigt mit `0`/`1` die akzeptierte gültige Benutzereinstellung. `get board.fmax` und `get board.conf` folgen derselben Konfiguration. Den tatsächlichen Registerzustand zeigt `get board.bqdiag`; es gibt keinen vorgemerkten Override außerhalb des Gates.

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
set board.imax 450             # Unter 0,05C von 10000 mAh; 500 mA würden abgelehnt
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

`get board.saved` entfällt. Verwende die einzelnen Getter; es gibt keine zusätzliche Versionskennung.
