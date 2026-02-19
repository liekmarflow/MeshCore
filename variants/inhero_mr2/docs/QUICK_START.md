# Inhero MR2 Quick-Start (v0.2)

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
- Legt Ladeparameter und UVLO-Schwellen fest.

## 7) Akkukapazitaet setzen
- Command: set board.batcap <mAh>
- Beispiel: set board.batcap 10000
- Wichtig fuer korrekte SOC-Berechnung.

## 8) Maximalen Ladestrom setzen
- Command: set board.imax <mA>
- Bereich laut Firmware: 50 bis 1000 mA (BQ25798-Minimum: 50mA).
- Passend zum Solar-Setup waehlen, damit die Stroeme zum PG-Check passen.
- Faustformel: Panelleistung / Panelspannung * 1.2

## 9) Frost-Ladestromabsenkung einstellen
- Command: set board.fmax <0%|20%|40%|100%>
- Steuert die JEITA-Reduktion bei Kaelte.
- Hinweis: Bei LTO ist JEITA deaktiviert (Frost ohne Wirkung).

## 10) MPPT aktivieren
- Command: set board.mppt <0|1>
- 1 = MPPT an, 0 = MPPT aus.
- Fuer Solar-Eingang typischerweise aktivieren.

## 11) UVLO latched aktivieren/deaktivieren
- Command: set board.uvlo <0|1|true|false>
- 1/true = aktiv, 0/false = deaktiviert.
- Die Einstellung ist persistent und nutzt chemie-spezifische Schwellen.

## 12) LEDs aktivieren/deaktivieren
- Command: set board.leds <on|off> oder set board.leds <1|0>
- Steuert Heartbeat-LED und BQ-Status-LED (Boot-LEDs bleiben aktiv).

## 13) Akku voll laden (SOC-Sync)
- Den Akku einmal komplett ueber USB aufladen, damit der SOC sauber synchronisiert.

## Zusatzhinweise (Praxis)
- Nach dem Setzen der Akkuchemie lohnt ein kurzer Check mit `get board.bat`, ob die Einstellung gespeichert wurde.
- Bei Solarbetrieb ist `set board.mppt 1` empfehlenswert; bei reinem USB-Betrieb kann MPPT aus bleiben.
- Wenn die Eingangserkennung haengt (PGOOD/USB), hilft `get board.togglehiz` fuer eine manuelle Neuqualifikation.
- Bei falschen Stromwerten kann `set board.ibcal <mA>` die INA228-Strommessung kalibrieren.
- Bei konstantem Strom-Offset (z.B. INA228 zeigt immer 2mA zu viel) hilft `set board.iboffset <mA>` fuer eine Offset-Korrektur.
- `get board.uvlo` zeigt, ob UVLO latched aktiv ist; fuer Feldtests ist oft DISABLED gesetzt.

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

| Akkuchemie | Hardware-UVLO (Alert) | Kritisch (0% SOC) | Hysterese |
|---|---|---|---|
| Li-Ion 1S | 3.1V | 3.4V | +0.3V |
| LiFePO4 1S | 2.7V | 2.9V | +0.2V |
| LTO 2S | 3.9V | 4.2V | +0.3V |

## Verhalten bei Danger-Zone und UVLO
- **Danger-Zone (Critical / 0% SOC):** Die Firmware geht in den Schutzmodus und plant seltene RTC-Wakes, um die Zelle zu schonen und Selbstentladung zu minimieren.
- **Unter UVLO-Schwelle (latched):** Die Hardware schaltet hart ab (INA228 Alert -> TPS62840 EN). Das Board bleibt dann aus und kommt nie wieder von selbst hoch. Das ist ein reiner Akkuschutz und kein normaler Sleep-Modus.

## CLI-Beispiele (kompakt)
```bash
# Akkuchemie und Kapazitaet
set board.bat liion1s
set board.batcap 10000

# Ladeparameter
set board.imax 500
set board.fmax 20%
set board.mppt 1

# UVLO und LEDs
set board.uvlo 1
set board.leds off

# Statuschecks
get board.bat
get board.imax
get board.fmax
get board.mppt
get board.uvlo
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
- `get board.hwver` - Hardware-Version (MR2 immer v0.2).
- `get board.fmax` - Aktuelles Frost-Ladeverhalten (0%/20%/40%/100%).
- `get board.imax` - Maximaler Ladestrom in mA.
- `get board.mppt` - MPPT-Status (0/1).
- `get board.uvlo` - UVLO latched Status (ENABLED/DISABLED).
- `get board.leds` - LED-Status (Heartbeat + BQ-Stat).
- `get board.batcap` - Batteriekapazität in mAh (set/default).
- `get board.telem` - Echtzeit-Telemetrie (Battery/Solar inkl. SOC, V/I/T).
- `get board.stats` - Energie-Bilanz (24h/3d/7d), Charge/Discharge-Breakdown und MPPT-Anteil.
- `get board.energy` - INA228 Coulomb Counter (Rohdaten, Base, Net).
- `get board.cinfo` - Ladegeraet-Status (Charger State + Flags).
- `get board.diag` - Detaildiagnose des BQ25798 (PG, HIZ, MPPT, VBUS, Temp, Register).
- `get board.togglehiz` - Manuelles Input-Qualify via HIZ-Toggle.
- `get board.conf` - Kurzuebersicht aller Konfigs (B, F, M, I, Vco, V0).
- `get board.ibcal` - INA228-Kalibrierfaktor (1.0 = default).
- `get board.iboffset` - INA228-Strom-Offset in mA (0.00 = default).
- `get board.tccal` - NTC-Temperatur-Kalibrieroffset in °C (0.00 = default).
