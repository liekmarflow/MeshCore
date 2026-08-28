# Inhero MR2 Quick-Start

> 🇬🇧 [English version](../QUICK_START.md)

Diese Anleitung führt Sie durch die Inbetriebnahme und die wichtigsten CLI-Commands.

## 1) Temperaturfühler (TS/NTC) vorbereiten
- Entweder den 3-poligen Akkuanschluss mit TS/NTC nutzen oder die Onboard-NTC-Lötbrücke auf der Rückseite schließen.
- Firmware-NTC-Typ: NCP15XH103F03RC (10k @ 25C, Beta 3380).
- Zweck: Der Charger nutzt den TS-Pin für JEITA/Frost-Logik.
- Ohne NTC liest der offene TS-Pin als Frost, und der Charger sperrt das Laden bei Li-ion und LiFePO4. Ein NTC ist die saubere Lösung; für eine Installation ohne NTC nimmt der Override aus Schritt 10 den TS-Pin aus der Entscheidung heraus.
- → [FAQ #2 — Akkupacks ohne NTC](FAQ.md#2-kann-ich-auch-akkupacks-ohne-eingebauten-ntc-nutzen)

## 2) Antennen anschließen
- Nie ohne Antenne betreiben, sonst Gefahr für das RF-Frontend.

## 3) Akku anschließen
- Ein Ladestand >90% wird empfohlen, damit der Akku per USB voll geladen werden kann und damit die SOC-Berechnung stabil startet.

> **⚠ WARNUNG — Kein Verpolschutz:** Das Board hat keinen Hardware-Verpolschutz. Ein verpolter Akkuanschluss führt zu sofortiger, irreversibler Beschädigung. Vor dem Anschließen immer die Polarität prüfen.

## 4) Firmware flashen und Repeater konfigurieren
- Board per USB-Kabel mit dem Rechner verbinden.
- Ausgeliefert wird das Board nur mit Bootloader. Die Firmware für den MR2 wird aus dem Inhero-Fork veröffentlicht — flasher.meshcore.io führt dieses Board noch nicht (die Upstream-PRs #3131 / #3132 stehen aus).
- UF2 von https://github.com/liekmarflow/MeshCore/releases laden.
- Reset-Taster doppelt tippen (rechte Seite, unter USB-C). Es erscheint ein Massenspeicher-Laufwerk namens `RAK4630` oder `FTHR840`.
- UF2 auf dieses Laufwerk ziehen; das Board startet in die Firmware.
- Der MR2 fährt die **Repeater**-Rolle von MeshCore (Build `Inhero_MR2_repeater`); ein **Sensor**-Build (`Inhero_MR2_sensor`) steht ebenfalls bereit.
- Danach auf https://flasher.meshcore.io -> Repeater-Setup die LoRa-Settings, den Namen und das Admin-Passwort konfigurieren.

## 5) CLI öffnen
- https://flasher.meshcore.io -> Console
- oder MeshCore-App -> Manage -> Command-Line
- Hier werden die Board-spezifischen Commands gesetzt.

## 6) Akkuchemie setzen
- Command:
  - set board.bat liion1s
  - oder set board.bat lifepo1s
  - oder set board.bat lto2s
  - oder set board.bat naion1s
- Legt Ladeparameter und Low-Voltage-Schwellen fest.
- → [FAQ #1](FAQ.md#1-welche-akkuchemie-soll-ich-einsetzen) | [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Welche Akkuchemie soll ich einsetzen?

## 7) Akkukapazität setzen
- Command: set board.batcap <mAh>
- Beispiel: set board.batcap 10000
- Wichtig für korrekte SOC-Berechnung und Voraussetzung für den Frost-Lade-Override in Schritt 10.
- → [FAQ #4 — Welcher mAh-Wert?](FAQ.md#4-welchen-mah-wert-gebe-ich-bei-set-boardbatcap-ein)

## 8) Maximalen Ladestrom setzen
- Command: set board.imax <mA>
- Bereich laut Firmware: 50 bis 1500 mA (BQ25798-Minimum: 50mA).
- Passend zum Solar-Setup wählen, damit die Ströme zum PG-Check passen.
- Faustformel: Panelleistung / Panelspannung * 1.2
- → [FAQ #5 — Warum imax setzen?](FAQ.md#5-warum-ist-es-wichtig-den-maximalen-ladestrom-set-boardimax-einzustellen)

## 9) Frost-Ladestromabsenkung einstellen
- Command: set board.fmax <0%|20%|40%|100%>
- Begrenzt den maximalen Ladestrom im T-Cool-Bereich (ca. -2 °C bis +3 °C, siehe JEITA-Tabelle im README) auf X% von board.imax.
- 0% = Laden im T-Cool-Bereich gesperrt.
- 20% = max. 20% von imax (z.B. 500mA → 100mA bei ca. -2 °C bis +3 °C).
- 40% = max. 40% von imax (z.B. 500mA → 200mA bei ca. -2 °C bis +3 °C).
- 100% = keine Reduktion, voller Ladestrom auch bei Kälte.
- Unter ca. -2 °C (T-Cold): Laden komplett gesperrt durch JEITA, sofern nicht der Override aus Schritt 10 scharf ist.
- Wichtig: Nur das Laden wird eingeschränkt. Bei ausreichend Solar wird das Board weiterhin mit Solarstrom betrieben — der Akku wird weder ge- noch entladen.
- Hinweis: Bei LTO und Na-ion ist JEITA deaktiviert (`set board.fmax` wird mit Fehler abgelehnt, lädt auch bei Frost).
- → [FAQ #6 — Was steuert fmax?](FAQ.md#6-was-wird-durch-set-boardfmax-beeinflusst)

## 10) Frost-Lade-Override (optional)
- Command: set board.jeitaignore <1|0> — Default 0.
- Nur für Li-ion und LiFePO4. LTO und Na-ion laden ohnehin bei Frost und lehnen den Command mit `Err: This chemistry runs without JEITA (always 1)` ab.
- Mit 1 ignoriert der Charger den TS-Pin: Das Laden läuft unter -2 °C weiter, und die obere Abschaltung des Chargers bei ca. +58 °C entfällt ebenfalls.
- Voraussetzung: `board.batcap` muss gesetzt sein (Schritt 7) und `board.imax` höchstens 0,05C dieser Kapazität betragen (10000 mAh → 500 mA). Sonst benennt die Antwort den Blocker — `jeitaignore set to 1, N/A, batcap not set` oder `jeitaignore set to 1, N/A, C>0.05`. Die Einstellung bleibt in beiden Fällen gespeichert und greift von selbst, sobald imax oder batcap passen.
- Die 0,05C-Grenze gilt, solange der Override an ist, und sie kann deutlich unter dem liegen, was das Panel liefert: Ein 4000-mAh-Akku erlaubt 200 mA; das 2-W-Panel aus Schritt 8 gibt rund 480 mA.
- Laden von Li-ion oder LiFePO4 bei Frost scheidet metallisches Lithium an der Anode ab, kumulativ und dauerhaft; das zeigt sich später als verlorene Kapazität.
- Solange der Override an ist, zeigt `get board.fmax` N/A, und `set board.fmax` wird mit `Err: Fmax N/A while jeitaignore is on` abgelehnt.
- → [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Laden bei Kälte, Felderfahrung und die vollständige Abwägung

## 11) MPPT aktivieren
- Command: set board.mppt <0|1>
- 1 = MPPT an, 0 = MPPT aus.
- Für Solar-Eingang typischerweise aktivieren.

## 12) LEDs aktivieren/deaktivieren
- Command: set board.leds <on|off> oder set board.leds <1|0>
- Steuert Heartbeat-LED und BQ-Status-LED (Bootloader-LED-Muster bleiben unberührt).
- → [FAQ #17 — Was bedeuten die LEDs?](FAQ.md#17-was-bedeuten-die-leds)

## 13) Akku voll laden (SOC-Sync)
- Den Akku einmal komplett über USB aufladen, damit der SOC sauber synchronisiert.
- → [FAQ #11 — SOC zeigt 0% oder N/A?](FAQ.md#11-warum-zeigt-der-soc-0-oder-na-an)

> **Hinweis Kältebetrieb:** SOC% ist rein Coulomb-basiert und ändert sich nur durch reale Ladungsflüsse. Bei Kälte sperrt das Trapped-Charge-Modell den Boden der Entladekurve — bei niedrigem SOC fällt die entnehmbare Kapazität steil, und `get board.telem` zeigt den derateten Wert in Klammern: `SOC:95.0% (78%)`. Siehe [FAQ #13](FAQ.md#13-wie-funktioniert-das-temperatur-derating) für Details.

## Zusatzhinweise (Praxis)
- Nach dem Setzen der Akkuchemie lohnt ein kurzer Check mit `get board.bat`, ob die Einstellung gespeichert wurde.
- Bei Solarbetrieb ist `set board.mppt 1` empfehlenswert; bei reinem USB-Betrieb kann MPPT aus bleiben.

## Beispielwerte je Akkuchemie (Startpunkt)
Diese Werte sind sichere Startpunkte und sollten an Akku, Panel und Einsatzprofil angepasst werden.

Die `imax`-Werte unten leiten sich aus der Faustformel aus Abschnitt 8 ab:
**`imax ≈ Panelleistung ÷ Panelspannung × 1.2`** (z.B. 2 W ÷ 5 V × 1.2 ≈ 480 mA → aufgerundet auf 500).
`fmax` ist ein Prozentwert von `imax` und wirkt nur in der T-Cool-Zone (ca. -2 °C bis +3 °C, siehe JEITA-Tabelle im README).

### Li-ion 1S (3.7V nominal)
```bash
set board.bat liion1s    # Chemie: 1S Li-ion (setzt Ladeprofil + Low-V-Schwellen)
set board.batcap 10000   # Akkukapazität — SOC und die 0,05C-Grenze für Schritt 10 (→ 500 mA)
set board.imax 500       # max. Ladestrom — ≈ 2 W Panel @ 5 V (2 W ÷ 5 V × 1.2 ≈ 480 mA)
set board.fmax 20%       # T-Cool (ca. -2…+3 °C): begrenzt auf 20 % × 500 mA = 100 mA
```

### LiFePO4 1S (3.2V nominal)
```bash
set board.bat lifepo1s   # Chemie: 1S LiFePO4 (setzt Ladeprofil + Low-V-Schwellen)
set board.batcap 9000    # Akkukapazität — SOC und die 0,05C-Grenze für Schritt 10 (→ 450 mA)
set board.imax 300       # max. Ladestrom — ≈ 1 W Panel @ 5 V (1 W ÷ 5 V × 1.2 ≈ 240 mA, aufgerundet als Reserve)
set board.fmax 40%       # T-Cool (ca. -2…+3 °C): begrenzt auf 40 % × 300 mA = 120 mA
```

### LTO 2S (2x 2.3V nominal)
```bash
set board.bat lto2s      # Chemie: 2S LTO (setzt Ladeprofil + Low-V-Schwellen)
set board.batcap 10000   # Akkukapazität — für die SOC-Berechnung
set board.imax 700       # max. Ladestrom — ≈ 3 W Panel @ 5 V (3 W ÷ 5 V × 1.2 = 720 mA → 700)
                         # fmax entfällt: wird bei LTO abgelehnt (JEITA deaktiviert — LTO lädt auch bei Frost)
```

### Na-ion 1S (3.1V nominal)
```bash
set board.bat naion1s    # Chemie: 1S Na-ion (setzt Ladeprofil + Low-V-Schwellen)
set board.batcap 10000   # Akkukapazität — für die SOC-Berechnung
set board.imax 500       # max. Ladestrom — ≈ 2 W Panel @ 5 V (2 W ÷ 5 V × 1.2 ≈ 480 mA)
                         # fmax entfällt: wird bei Na-ion abgelehnt (JEITA deaktiviert)
```

Hinweis: `set board.fmax` wird bei LTO und Na-ion mit Fehler abgelehnt (JEITA deaktiviert); `get board.fmax` zeigt N/A. Dasselbe gilt bei Li-ion und LiFePO4, solange der Frost-Lade-Override aus Schritt 10 an ist.

## Solarpanel-Hinweise
- Maximale Leerlaufspannung (Voc) für den Eingang: 25V.
- Typische Panels haben 5V oder 6V (MPP darunter).
- Das Board hat Buck/Boost und kann auch mit niedrigerer Panelspannung höhere Akkuspannungen laden.
- 24V-Panels oder Serienverschaltung können die 25V-Voc-Grenze überschreiten und sind nicht geeignet.
- Wattklasse: mindestens 1W, typisch 2W.
- Bei 1W-Panels wird eine Akkukapazität von >7Ah empfohlen.
- Gilt nur bei Südausrichtung, vertikaler Montage und unverschattetem Standort.
- Bei schlechteren Solarbedingungen entweder auf 2W gehen oder die Akkukapazität für "Winterüberleben" erhöhen.

→ [FAQ #8 — Welche Solarpanels?](FAQ.md#8-welche-solarpanels-kann-ich-anschließen)

## USB-Laden
- Das Board kann auch über USB-C (5V) geladen werden.
- USB-C VBUS wird über eine **Schottky-Diode** auf den BQ25798 VBUS-Eingang geführt — derselbe einzelne Eingang wie das Solarpanel. Der BQ25798 hat nur einen VBUS-Eingang und unterscheidet nicht zwischen USB und Solar.
- Die Schottky-Diode verhindert einen Rückfluss vom Solarpanel zum USB-Bus. Allerdings **kann** Strom von USB-VBUS über den Solarstecker abfließen.
- CC1/CC2 sind über 4,7kΩ auf GND gezogen (USB-Sink, 5V Standard).
- **⚠ Warnung:** Da VBUS-USB und VBUS-BQ denselben Bus teilen (über die Schottky-Diode), führt ein **Kurzschluss am Solarstecker auch zum Kurzschluss von VBUS-USB**. Den Solareingang niemals kurzschließen, während USB angeschlossen ist.

## Spannungsschwellen je Akkuchemie
Die Schwellen sind auf lange Lebensdauer und stabilen Betrieb ausgelegt.

| Akkuchemie | lowv_sleep_mv (System Sleep) | lowv_wake_mv (0% SOC) | Hysterese |
|---|---|---|---|
| Li-ion 1S | 3100 | 3300 | 200mV |
| LiFePO4 1S | 2700 | 2900 | 200mV |
| LTO 2S | 3900 | 4100 | 200mV |
| Na-ion 1S | 2500 | 2700 | 200mV |

## Verhalten bei Low-Voltage
- **Low-Voltage System Sleep:** Wenn VBAT unter `lowv_sleep_mv` fällt, feuert der INA228 ALERT-Interrupt (P1.02). Die Firmware latcht CE HIGH (`digitalWrite(BQ_CE_PIN, HIGH)` → FET ON → CE LOW → Laden aktiv), konfiguriert den RTC-Wake-Timer und geht in System Sleep mit GPIO-Latch (< 500µA). P0.04 wird von `disconnectLeakyPullups()` ausgeschlossen, damit der GPIO-Latch HIGH bleibt. Periodische RTC-Wakes (stündlich) prüfen die Spannung — erst bei Erholung über `lowv_wake_mv` wird normal gebootet.
- **Solar-Recovery:** Im System Sleep bleibt der GPIO4-Latch HIGH erhalten → CE-FET ON → CE LOW → Laden aktiv. Solar-Laden läuft autonom weiter bis der Akku über `lowv_wake_mv` geladen ist. Ohne GPIO-Latch (RAK stromlos): ext. Pull-Down am Gate → FET OFF → CE HIGH → Laden AUS (Safety-Default).

## CLI-Beispiele (kompakt)
```bash
# Akkuchemie und Kapazität
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
get board.jeitaignore
get board.telem
get board.stats
get board.cinfo
get board.selftest
get board.conf
```

## Getter-Kurzinfos (alle relevanten Board-Getter)
- `get board.bat` - Aktueller Akkutyp (liion1s, lifepo1s, lto2s, naion1s, none).
- `get board.fmax` - Aktuelles Frost-Ladeverhalten (0%/20%/40%/100%; N/A immer dann, wenn der JEITA-Override aktiv ist).
- `get board.imax` - Maximaler Ladestrom in mA.
- `get board.mppt` - MPPT-Status (0/1).
- `get board.leds` - LED-Status (Heartbeat + BQ-Stat).
- `get board.batcap` - Akkukapazität in mAh (set/default).
- `get board.jeitaignore` - Frost-Lade-Override: `jeitaignore 0`, `jeitaignore 1`, `jeitaignore 1 (chemistry)` bei LTO/Na-ion oder die gespeicherte Einstellung mit ihrem Blocker (`jeitaignore 1, N/A, batcap not set` / `jeitaignore 1, N/A, C>0.05`).
- `get board.telem` - Echtzeit-Telemetrie (Battery/Solar inkl. SOC, V/I/T). Siehe [TELEMETRY.md](TELEMETRY.md) für die App-Anzeige.
- `get board.stats` - Energie-Bilanz (24h/3d/7d), Charge/Discharge-Breakdown und MPPT-Anteil.
- `get board.cinfo` - Ladegerät-Status (Charger State + Flags).
- `get board.selftest` - I²C-Hardware-Probe (`INA:OK BQ:OK RTC:OK BME:OK`). RTC inkl. Write/Readback-Verifikation (Zustand `WR_FAIL` bei Mismatch).
- `get board.conf` - Kurzübersicht aller Konfigs (B, F, M, I, Vco, V0; zusätzlich `J:1`, solange der Frost-Lade-Override bei Li-ion/LiFePO4 an ist).
- `get board.tccal` - NTC-Temperatur-Kalibrieroffset in °C (0.00 = default).
  - → [FAQ #12 — Wann tccal ausführen?](FAQ.md#12-wann-sollte-ich-set-boardtccal-ausführen)

---

## Siehe auch

- [README.md](README.md) — Übersicht, Feature-Matrix und Diagnose
- [DATASHEET.md](DATASHEET.md) — Hardware-Spezifikationen und Pinout
- [TELEMETRY.md](TELEMETRY.md) — Telemetrie-Kanäle erklärt (was die App anzeigt)
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Akkuchemie-Vergleich und Einsatzempfehlungen
- [FAQ.md](FAQ.md) — Häufig gestellte Fragen
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle board-spezifischen CLI-Befehle auf einen Blick
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Vollständige technische Dokumentation
