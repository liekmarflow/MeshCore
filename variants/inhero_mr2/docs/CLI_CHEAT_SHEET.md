# Inhero MR2 – CLI Cheat-Sheet

Alle board-spezifischen CLI-Befehle auf einen Blick.
Praefix ist immer `board.` – also `get board.<cmd>` bzw. `set board.<cmd> <wert>`.

---

## Setter (Konfiguration aendern)

```bash
# Akkuchemie
set board.bat liion1s          # Li-Ion 1S (3.7V nominal)
set board.bat lifepo1s         # LiFePO4 1S (3.2V nominal)
set board.bat lto2s            # LTO 2S (2x 2.3V nominal)

# Akkukapazitaet (100–100000 mAh)
set board.batcap 10000

# Maximaler Ladestrom (50–1000 mA)
set board.imax 500

# Frost-Ladestromabsenkung (T-Cool 0°C bis -5°C)
set board.fmax 0%              # Laden gesperrt
set board.fmax 20%             # max. 20% von imax
set board.fmax 40%             # max. 40% von imax
set board.fmax 100%            # keine Reduktion
# Hinweis: Bei LTO ohne Wirkung (JEITA deaktiviert)

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
# INA228-Strom-Offset-Korrektur
set board.iboffset <mA>        # Offset mit Referenzstrom kalibrieren
set board.iboffset reset       # Offset auf 0.00 zuruecksetzen

# NTC-Temperatur-Kalibrierung
set board.tccal                # Auto-Kalibrierung via BME280
set board.tccal reset          # Offset auf 0.00 zuruecksetzen
```

---

## Getter (Status abfragen)

```bash
# Konfiguration & Hardware
get board.bat                  # Aktueller Batterietyp
get board.hwver                # Hardware-Version (immer Rev 1.0)
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
get board.energy               # INA228 Coulomb Counter (Raw, Base, Net)

# Ladegeraet & Diagnose
get board.cinfo                # Charger-Status (State + Flags)
get board.diag                 # Detail-Diagnose BQ25798 (kompakt: PG, HZ, MP, CHG, VBUS, TS, VOC)
get board.hiz                  # Manuelles Input-Qualify via HIZ-Toggle (Alias: togglehiz)

# Kalibrierung
get board.iboffset             # INA228-Strom-Offset in mA (0.00 = default)
get board.tccal                # NTC-Temperatur-Offset in °C (0.00 = default)

# Solar / PFM
get board.pfm                  # PFM Forward Mode Status + HIZ-Gate-State
```

---

## Getter-Kurzinfos

| Befehl | Beschreibung |
|---|---|
| `get board.bat` | Batterietyp (`liion1s`, `lifepo1s`, `lto2s`) |
| `get board.hwver` | Hardware-Version (MR2 immer `Rev 1.0`) |
| `get board.batcap` | Batteriekapazitaet in mAh (set/default) |
| `get board.imax` | Maximaler Ladestrom in mA |
| `get board.fmax` | Frost-Ladeverhalten (`0%`/`20%`/`40%`/`100%`, bei LTO: `N/A`) |
| `get board.mppt` | MPPT-Status (`0`/`1`) |
| `get board.leds` | LED-Status Heartbeat + BQ-Stat (`ON`/`OFF`) |
| `get board.conf` | Kurzuebersicht: B(at) F(max) M(ppt) I(max) Vco V0 |
| `get board.telem` | Echtzeit-Telemetrie: Battery/Solar V, I, T, SOC |
| `get board.stats` | Energie-Bilanz (24h/3d/7d), C/D, MPPT%, TTL (7d-Avg-basiert) |
| `get board.energy` | INA228 Coulomb Counter (Raw, Base, Net) |
| `get board.cinfo` | Ladegeraet-Status (Charger State + Flags) |
| `get board.diag` | Detail-Diagnose BQ25798 (kompakt: PG, HZ, MP, CHG, VBUS, TS, VOC) |
| `get board.hiz` | Manuelles Input-Qualify via HIZ-Toggle (Alias: `togglehiz`) |
| `get board.pfm` | PFM Forward Mode Status + HIZ-Gate-State (`on`/`off` + `[HIZ]`/`[CHG]`) |
| `get board.iboffset` | INA228-Strom-Offset in mA (`0.00` = default) |
| `get board.tccal` | NTC-Temperatur-Offset in °C (`0.00` = default) |

---

## Setter-Kurzinfos

| Befehl | Wertebereich | Beschreibung |
|---|---|---|
| `set board.bat` | `liion1s` · `lifepo1s` · `lto2s` | Akkuchemie waehlen |
| `set board.batcap` | `100`–`100000` (mAh) | Akkukapazitaet setzen |
| `set board.imax` | `50`–`1000` (mA) | Max. Ladestrom setzen |
| `set board.fmax` | `0%` · `20%` · `40%` · `100%` | Frost-Ladestromabsenkung (nicht bei LTO) |
| `set board.mppt` | `0`/`1` · `true`/`false` | MPPT ein-/ausschalten |
| `set board.leds` | `on`/`off` · `1`/`0` | LEDs ein-/ausschalten |
| `set board.pfm` | `0`/`1` · `on`/`off` | PFM Forward Mode (besser für 5-6V Panels) |
| `set board.soc` | `0`–`100` (%) | SOC manuell setzen |
| `set board.iboffset` | `<mA>` · `reset` | INA228-Strom-Offset kalibrieren oder zuruecksetzen |
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

### Status-Check (alles auf einen Blick)
```bash
get board.conf
get board.telem
get board.stats
get board.cinfo
```

---

## Siehe auch

- [README.md](README.md) - Übersicht, Feature-Matrix und Diagnose
- [QUICK_START.md](QUICK_START.md) - Schnellstart fuer Inbetriebnahme und CLI-Setup
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Vollständige technische Dokumentation
- [BATTERY_AUTO_LEARNING.md](BATTERY_AUTO_LEARNING.md) - Veraltet: historisches Auto-Learning-Konzept
