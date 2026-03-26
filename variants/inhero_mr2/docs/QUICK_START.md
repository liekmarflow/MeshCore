# Inhero MR2 Quick-Start (Rev 1.0)

Diese Anleitung fuehrt Sie durch die Inbetriebnahme und die wichtigsten CLI-Commands.

## 1) Temperaturfuehler (TS/NTC) vorbereiten
- Entweder den 3-poligen Akkuanschluss mit TS/NTC nutzen oder die Onboard-NTC-Loetbruecke auf der Rueckseite schliessen.
- Firmware-NTC-Typ: NCP15XH103F03RC (10k @ 25C, Beta 3380).
- Zweck: Der Charger nutzt den TS-Pin fuer JEITA/Frost-Logik.

## 2) Antennen anschliessen
- Nie ohne Antenne betreiben, sonst Gefahr fuer das RF-Frontend.

## 3) Akku anschliessen
- Ein Ladestand >90% wird empfohlen, damit die SOC-Berechnung stabil startet.

## 4) Repeater per USB konfigurieren
- Repeater per USB-Kabel mit dem Rechner verbinden.
- Auf https://flasher.meshcore.co.uk/ -> Repeater-Setup konfigurieren (LoRa-Settings, Name, Admin-Passwort usw.).
- Dadurch werden die Grundparameter im Geraet gesetzt.

## 5) CLI oeffnen
- https://flasher.meshcore.co.uk/ -> Console
- oder MeshCore-App -> Manage -> Command-Line
- Hier werden die Board-spezifischen Commands gesetzt.

## 6) Akkuchemie setzen
- Command:
  - set board.bat liion1s
  - oder set board.bat lifepo1s
  - oder set board.bat lto2s
- Legt Ladeparameter und Low-Voltage-Schwellen fest.

## 7) Akkukapazitaet setzen
- Command: set board.batcap <mAh>
- Beispiel: set board.batcap 10000
- Wichtig fuer korrekte SOC-Berechnung.

## 8) Maximalen Ladestrom setzen
- Command: set board.imax <mA>
- Bereich laut Firmware: 50 bis 1500 mA (BQ25798-Minimum: 50mA).
- Passend zum Solar-Setup waehlen, damit die Stroeme zum PG-Check passen.
- Faustformel: Panelleistung / Panelspannung * 1.2

## 9) Frost-Ladestromabsenkung einstellen
- Command: set board.fmax <0%|20%|40%|100%>
- Begrenzt den maximalen Ladestrom im T-Cool-Bereich (0°C bis -5°C) auf X% von board.imax.
- 0% = Laden im T-Cool-Bereich gesperrt.
- 20% = max. 20% von imax (z.B. 500mA → 100mA bei 0°C bis -5°C).
- 40% = max. 40% von imax (z.B. 500mA → 200mA bei 0°C bis -5°C).
- 100% = keine Reduktion, voller Ladestrom auch bei Kaelte.
- Unter -5°C (T-Cold): Laden immer komplett gesperrt durch JEITA.
- Wichtig: Nur das Laden wird eingeschraenkt. Bei ausreichend Solar wird das Board weiterhin mit Solarstrom betrieben — der Akku wird weder ge- noch entladen.
- Hinweis: Bei LTO ist JEITA deaktiviert (fmax ohne Wirkung, laedt auch bei Frost).

## 10) MPPT aktivieren
- Command: set board.mppt <0|1>
- 1 = MPPT an, 0 = MPPT aus.
- Fuer Solar-Eingang typischerweise aktivieren.

## 11) LEDs aktivieren/deaktivieren
- Command: set board.leds <on|off> oder set board.leds <1|0>
- Steuert Heartbeat-LED und BQ-Status-LED (Boot-LEDs bleiben aktiv).

## 12) Akku voll laden (SOC-Sync)
- Den Akku einmal komplett ueber USB aufladen, damit der SOC sauber synchronisiert.

## Zusatzhinweise (Praxis)
- Nach dem Setzen der Akkuchemie lohnt ein kurzer Check mit `get board.bat`, ob die Einstellung gespeichert wurde.
- Bei Solarbetrieb ist `set board.mppt 1` empfehlenswert; bei reinem USB-Betrieb kann MPPT aus bleiben.
- Wenn die Eingangserkennung haengt (PGOOD/USB), hilft `get board.togglehiz` fuer eine manuelle Neuqualifikation.
- Bei konstantem Strom-Offset (z.B. INA228 zeigt immer 2mA zu viel) hilft `set board.iboffset <mA>` fuer eine Offset-Korrektur.

## Beispielwerte je Akkuchemie (Startpunkt)
Diese Werte sind sichere Startpunkte und sollten an Akku, Panel und Einsatzprofil angepasst werden.

### Li-Ion 1S (3.7V nominal)
```bash
set board.bat liion1s
set board.imax 500
set board.fmax 20%
```

### LiFePO4 1S (3.2V nominal)
```bash
set board.bat lifepo1s
set board.imax 300
set board.fmax 40%
```

### LTO 2S (2x 2.3V nominal)
```bash
set board.bat lto2s
set board.imax 700
set board.fmax 0%
```

Hinweis: `set board.fmax` hat bei LTO keine Wirkung (JEITA deaktiviert).

## Solarpanel-Hinweise
- Maximale Leerlaufspannung (Voc) fuer den Eingang: 25V.
- Typische Panels sind 5V oder 6V (MPP darunter); 12V-Panels sind nicht notwendig.
- Das Board hat Buck/Boost und kann auch mit niedrigerer Panelspannung hoehere Akkuspannungen laden.
- 24V-Panels oder Serienverschaltung koennen die 25V-Voc-Grenze ueberschreiten und sind nicht geeignet.
- Wattklasse: mindestens 1W, typisch 2W.
- Bei 1W-Panels wird eine Akkukapazitaet von >7Ah empfohlen.
- Gilt nur bei Suedausrichtung, vertikaler Montage und unverschattetem Standort.
- Bei schlechteren Solarbedingungen entweder auf 2W gehen oder die Akkukapazitaet fuer "Winterueberleben" erhoehen.

## Spannungsschwellen je Akkuchemie
Die Schwellen sind auf maximale Lebensdauer und stabilen Betrieb optimiert.

| Akkuchemie | lowv_sleep_mv (System-Off) | lowv_wake_mv (0% SOC) | Hysterese |
|---|---|---|---|
| Li-Ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |

## Verhalten bei Low-Voltage
- **Low-Voltage System-Off:** Wenn VBAT unter `lowv_sleep_mv` fällt, feuert der INA228 ALERT-Interrupt (P1.02). Die Firmware setzt CE HIGH (Laden bleibt aktiv über FET-Schaltung), konfiguriert den RTC-Wake-Timer und geht in System-Off (~15µA). Periodische RTC-Wakes (stündlich) prüfen die Spannung — erst bei Erholung über `lowv_wake_mv` wird normal gebootet.
- **Solar-Recovery:** Im System-Off bleibt der CE-Pin über den DMN2004TK-7 FET aktiv (GPIO High-Z → ext. Pull-Up → CE HIGH → Laden an). Solar-Laden läuft autonom weiter bis die Batterie über `lowv_wake_mv` geladen ist.

## CLI-Beispiele (kompakt)
```bash
# Akkuchemie und Kapazitaet
set board.bat liion1s
set board.batcap 10000

# Ladeparameter
set board.imax 500
set board.fmax 20%
set board.mppt 1

# LEDs
set board.leds off

# Statuschecks
get board.bat
get board.imax
get board.fmax
get board.mppt
get board.leds
get board.batcap
get board.telem
get board.stats
get board.energy
get board.cinfo
get board.diag
get board.conf
```

## Getter-Kurzinfos (alle relevanten Board-Getter)
- `get board.bat` - Aktueller Batterietyp (liion1s, lifepo1s, lto2s).
- `get board.hwver` - Hardware-Version (MR2 immer Rev 1.0).
- `get board.fmax` - Aktuelles Frost-Ladeverhalten (0%/20%/40%/100%).
- `get board.imax` - Maximaler Ladestrom in mA.
- `get board.mppt` - MPPT-Status (0/1).
- `get board.leds` - LED-Status (Heartbeat + BQ-Stat).
- `get board.batcap` - Batteriekapazität in mAh (set/default).
- `get board.telem` - Echtzeit-Telemetrie (Battery/Solar inkl. SOC, V/I/T).
- `get board.stats` - Energie-Bilanz (24h/3d/7d), Charge/Discharge-Breakdown und MPPT-Anteil.
- `get board.energy` - INA228 Coulomb Counter (Rohdaten, Base, Net).
- `get board.cinfo` - Ladegeraet-Status (Charger State + Flags).
- `get board.diag` - Detaildiagnose des BQ25798 (PG, HIZ, MPPT, VBUS, Temp, Register).
- `get board.togglehiz` - Manuelles Input-Qualify via HIZ-Toggle.
- `get board.conf` - Kurzuebersicht aller Konfigs (B, F, M, I, Vco, V0).
- `get board.iboffset` - INA228-Strom-Offset in mA (0.00 = default).
- `get board.tccal` - NTC-Temperatur-Kalibrieroffset in °C (0.00 = default).
