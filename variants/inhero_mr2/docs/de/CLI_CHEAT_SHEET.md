# Inhero MR2 – CLI Cheat-Sheet

> 🇬🇧 [English version](../CLI_CHEAT_SHEET.md)

Alle board-spezifischen CLI-Befehle auf einen Blick.
Praefix ist immer `board.` – also `get board.<cmd>` bzw. `set board.<cmd> <wert>`.

---

## Setter (Konfiguration aendern)

```bash
# Akkuchemie
set board.bat liion1s          # Li-Ion 1S (3.7V nominal)
set board.bat lifepo1s         # LiFePO4 1S (3.2V nominal)
set board.bat lto2s            # LTO 2S (2x 2.3V nominal)
set board.bat naion1s          # Na-Ion 1S (3.1V nominal)
set board.bat none             # Kein Akku / unbekannt (Laden deaktiviert)

# Akkukapazitaet (100–100000 mAh)
set board.batcap 10000

# Maximaler Ladestrom (50–1500 mA)
set board.imax 500

# Frost-Ladestromabsenkung (T-Cool 0°C bis -5°C)
set board.fmax 0%              # Laden gesperrt
set board.fmax 20%             # max. 20% von imax
set board.fmax 40%             # max. 40% von imax
set board.fmax 100%            # keine Reduktion
# Hinweis: Bei LTO / Na-Ion ohne Wirkung (JEITA deaktiviert)

# MPPT ein/aus
set board.mppt 1               # MPPT aktivieren
set board.mppt 0               # MPPT deaktivieren

# LEDs ein/aus (Heartbeat + BQ-Stat)
set board.leds on              # LEDs aktivieren  (on/1)
set board.leds off             # LEDs deaktivieren (off/0)

# SOC manuell setzen (0–100%)
set board.soc 85.0
```

### Kalibrierung

```bash
# NTC-Temperatur-Kalibrierung
set board.tccal                # Auto-Kalibrierung via BME280
set board.tccal reset          # Offset auf 0.00 zuruecksetzen
```

---

## Getter (Status abfragen)

```bash
# Konfiguration & Hardware
get board.bat                  # Aktueller Batterietyp
get board.batcap               # Batteriekapazitaet in mAh (set/default)
get board.imax                 # Maximaler Ladestrom in mA
get board.fmax                 # Frost-Ladeverhalten (0%/20%/40%/100% oder N/A)
get board.mppt                 # MPPT-Status (0/1)
get board.leds                 # LED-Status (ON/OFF)
get board.conf                 # Kurzuebersicht aller Konfigs (B, F, M, I, Vco, V0)

# Echtzeit-Telemetrie
get board.telem                # Battery+Solar: V, I, T, SOC

# Energie & Statistik
get board.stats                # Energie-Bilanz (24h/3d/7d), C/D, MPPT%, TTL
                               #   TTL = Time To Live (Stunden bis Akku leer)
                               #   Basis: 7-Tage-Durchschnitt des taegl. Netto-Defizits
                               #   aus stuendlichen INA228-Coulomb-Counter-Samples (168h-Ringpuffer)
                               #   Formel: (SOC% × Kapazitaet) / |7d-Avg-Deficit| × 24
                               #   TTL erscheint nur im BAT-Modus (Netto-Defizit)
                               #   Voraussetzung: mind. 24h Daten + Kapazitaet bekannt

# Ladegeraet & Diagnose
get board.cinfo                # Charger-Status + letzter PG-Stuck HIZ-Toggle
get board.selftest             # Alle I2C-Komponenten pruefen (INA228/BQ25798/RV-3028/BME280)
                               #   Ausgabe: "INA:OK BQ:OK RTC:OK BME:OK"
                               #   RTC inkl. User-RAM Write/Readback-Verifikation
                               #   — erkennt kalte Loetstellen (Chip ACKt, akzeptiert
                               #   aber keine Writes). Moegliche Werte je Geraet:
                               #     OK      — antwortet (RTC: Write persistiert)
                               #     NACK    — keine I2C-Antwort
                               #     WR_FAIL — (nur RTC) ACKt, aber Write/Read stimmen nicht ueberein

# Kalibrierung
get board.tccal                # NTC-Temperatur-Offset in °C (0.00 = default)
```

---

## Getter-Kurzinfos

| Befehl | Beschreibung |
|---|---|
| `get board.bat` | Batterietyp (`liion1s`, `lifepo1s`, `lto2s`, `naion1s`, `none`) |
| `get board.batcap` | Batteriekapazitaet in mAh (set/default) |
| `get board.imax` | Maximaler Ladestrom in mA |
| `get board.fmax` | Frost-Ladeverhalten (`0%`/`20%`/`40%`/`100%`, bei LTO/Na-Ion: `N/A`) |
| `get board.mppt` | MPPT-Status (`0`/`1`) |
| `get board.leds` | LED-Status Heartbeat + BQ-Stat (`ON`/`OFF`) |
| `get board.conf` | Kurzuebersicht: B(at) F(max) M(ppt) I(max) Vco V0 |
| `get board.telem` | Echtzeit-Telemetrie: Battery/Solar V, I, T, SOC — siehe [TELEMETRY.md](TELEMETRY.md) |
| `get board.stats` | Energie-Bilanz (24h/3d/7d), C/D, MPPT%, TTL (7d-Avg-basiert) |
| `get board.cinfo` | Charger-Status + PG-Stuck HIZ-Toggle (z.B. "PG / CC HIZ:3m ago") |
| `get board.selftest` | I2C-Komponenten-Probe — `INA:OK BQ:OK RTC:OK BME:OK` (RTC inkl. Write-Verify) |
| `get board.tccal` | NTC-Temperatur-Offset in °C (`0.00` = default) |

---

## Setter-Kurzinfos

| Befehl | Wertebereich | Beschreibung |
|---|---|---|
| `set board.bat` | `liion1s` · `lifepo1s` · `lto2s` · `naion1s` · `none` | Akkuchemie waehlen |
| `set board.batcap` | `100`–`100000` (mAh) | Akkukapazitaet setzen |
| `set board.imax` | `50`–`1500` (mA) | Max. Ladestrom setzen |
| `set board.fmax` | `0%` · `20%` · `40%` · `100%` | Frost-Ladestromabsenkung (nicht bei LTO/Na-Ion) |
| `set board.mppt` | `0`/`1` · `true`/`false` | MPPT ein-/ausschalten |
| `set board.leds` | `on`/`off` · `1`/`0` | LEDs ein-/ausschalten |
| `set board.soc` | `0`–`100` (%) | SOC manuell setzen |
| `set board.tccal` | `reset` · *(leer = auto)* | NTC-Temperatur kalibrieren oder zuruecksetzen |

---

## Schnellstart-Rezepte

### Li-Ion 1S mit 10Ah und Solar
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

### Na-Ion 1S mit 10Ah und Solar
```bash
set board.bat naion1s
set board.batcap 10000
set board.imax 500
set board.mppt 1
set board.leds off
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
- [TELEMETRY.md](TELEMETRY.md) — Telemetrie-Kanäle erklärt (was die App anzeigt)
- [QUICK_START.md](QUICK_START.md) — Schnellstart für Inbetriebnahme und CLI-Setup
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Vollständige technische Dokumentation
