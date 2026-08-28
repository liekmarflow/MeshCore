# Telemetrie-Kanäle

> 🇬🇧 [English version](../TELEMETRY.md)

Das Inhero MR2 sendet Telemetriedaten im [CayenneLPP](https://docs.mydevices.com/docs/lorawan/cayenne-lpp)-Format über vier Kanäle. Die MeshCore-App zeigt diese als **Kanal 1–4** an.

---

## Kanal 1 — Gerätestatus

Basisdaten des Nodes.

| Feld | Einheit | Quelle | Beschreibung |
|------|---------|--------|--------------|
| Akkustand | % / V | INA228 | Siehe Hinweis zum SOC-Workaround unten |
| Temperatur | °C / °F | nRF52840-Chip | Chiptemperatur des Prozessors, gelesen über `getMCUTemperature()` und vom allgemeinen MeshCore-Telemetriepfad angehängt. Siehe [Welche Temperatur ist welche](#welche-temperatur-ist-welche) |

### Akkustand & SOC-Workaround

MeshCore überträgt auf Kanal 1 aktuell nur die Akku-**Spannung** — es gibt kein natives SOC%-Feld. Die MeshCore-App rechnet diese Spannung über eine fest hinterlegte **Li-ion-Entladekurve** in Prozent um. Das funktioniert gut für Li-ion-Zellen, liefert aber falsche Werte für LiFePO4, LTO oder Na-ion (deren Spannungskurve deutlich flacher verläuft).

Das MR2 umgeht diese Einschränkung:

| SOC-Status | Was `getBattMilliVolts()` liefert | App zeigt |
|------------|-----------------------------------|-----------|
| **SOC noch nicht gültig** | Echte Akkuspannung vom INA228 | Prozent basierend auf Li-ion-Kurve (kann bei Nicht-Li-ion ungenau sein) |
| **SOC gültig** (Coulomb-Counter kalibriert) | Fake Li-ion OCV, rückgerechnet aus echtem SOC% (`socToLiIonMilliVolts()`) | Korrekter Prozentwert — die Li-ion-Kurve der App dekodiert zurück zum ursprünglichen SOC% |

> **OCV** = Open Circuit Voltage (Leerlaufspannung) — die Ruhespannung des Akkus ohne Last. Die OCV-Kurve (Spannung vs. SOC%) ist charakteristisch für jede Akkuchemie und wird hier als Lookup-Tabelle verwendet, um SOC% zurück in eine Spannung umzurechnen, die die App interpretieren kann.

Der SOC wird gültig, sobald ein **Referenzpunkt** existiert — entweder manuell via `set board.soc <Prozent>` oder automatisch bei einem „Charging Done"-Event (setzt SOC auf 100 %).

Ohne `set board.batcap <mAh>` wird eine chemie-typische Default-Kapazität (1500–2000 mAh) angenommen. Das Setzen der echten Kapazität macht den angezeigten Prozentwert und die Batt-TTL genau.

Die Rückrechnung nutzt eine stückweise lineare Li-ion-OCV-Tabelle (3000 mV bei 0 % → 4200 mV bei 100 %). So zeigt die App den korrekten Coulomb-gezählten SOC unabhängig von der tatsächlichen Akkuchemie an.

---

## Kanal 2 — Umgebung (BME280)

Daten des BME280-Umgebungssensors (immer auf dem MR2 vorhanden).

| Feld | Einheit | Quelle | Beschreibung |
|------|---------|--------|--------------|
| Temperatur | °C / °F | BME280 | Umgebungstemperatur |
| Relative Luftfeuchtigkeit | % | BME280 | Relative Luftfeuchtigkeit |
| Luftdruck | hPa | BME280 | Barometrischer Druck |
| Höhe | m / ft | BME280 | Aus dem Luftdruck berechnete Höhe (Referenz: Meereshöhe) |

> **Hinweis:** Die Höhenberechnung basiert auf dem Standard-Meeresspiegeldruck (1013,25 hPa) und kann je nach Wetterlage abweichen.

---

## Kanal 3 — Akku (INA228 / BQ25798)

Hochpräzise Akkudaten vom INA228-Coulomb-Counter und BQ25798-Ladecontroller.

| Feld | LPP-Typ | Einheit | Quelle | Beschreibung |
|------|---------|---------|--------|--------------|
| Spannung | Voltage | V | INA228 | Akkuspannung (20-Bit-ADC, ±0,1 % Genauigkeit) |
| SOC | Percentage | % | INA228 | Ladezustand per Coulomb-Counting — *optional, nur wenn kalibriert* |
| Strom | Current | A | INA228 | Akkustrom. Negativ = Entladung, positiv = Ladung |
| Temperatur | Temperature | °C / °F | NTC am TS-Pin des BQ25798 | Akkutemperatur — *optional, entfällt wenn nicht verfügbar* |
| Batt-TTL | Distance | Tage | berechnet | Geschätzte Restlaufzeit — *optional, nur bei gültigem SOC* |

### SOC & Batt-TTL

SOC und Batt-TTL erscheinen nur, wenn der Coulomb-Counter einen gültigen Referenzpunkt hat — entweder ein manuell gesetzter SOC (`set board.soc`) oder ein „Charging Done"-Event. Der Prozentwert basiert auf der konfigurierten Akkukapazität (`set board.batcap`; sonst wird eine chemie-typische Default-Kapazität von 1500–2000 mAh angenommen). Solange der SOC nicht gültig ist, werden diese Felder weggelassen.

### Batt-TTL-Kodierung

Der Batt-TTL-Wert wird als **CayenneLPP-Distance-Wert** in Tagen übertragen, da CayenneLPP keinen nativen „Dauer"-Typ hat. Die MeshCore-App zeigt ihn als Entfernung an (z. B. „42 m"), aber der Wert repräsentiert **Tage Restlaufzeit**.

| Bedingung | Übertragener Wert | Bedeutung |
|-----------|-------------------|-----------|
| Endliche Batt-TTL | `ttlHours / 24.0` | Geschätzte verbleibende Tage im Akkubetrieb |
| Überschuss (Ladung > Verbrauch) | `990.0` (Sentinel-Wert) | Praktisch unendlich — Gerät gewinnt Ladung |
| Unbekannt (SOC noch nicht gültig) | *nicht gesendet* | Batt-TTL kann noch nicht berechnet werden |

### Akkutemperatur

Die Temperatur auf diesem Kanal wird am Akku gemessen, vom NTC am TS-Pin des BQ25798. Der BQ25798 meldet die Spannung am TS-Pin als Prozentwert von REGN; die Firmware dekodiert sie über den Inhero-Teiler (RT1 = 5,6 kΩ, RT2 = 27 kΩ) mit einer Steinhart-Hart-Gleichung, addiert den Offset aus `set board.tccal` und führt vor dem Senden eine Plausibilitätsprüfung durch.

Damit eine Akkutemperatur erscheint, müssen vier Bedingungen erfüllt sein:

1. Der ADC-One-Shot des BQ25798 läuft durch.
2. Der TS-ADC-Kanal ist aktiv. Er wird abgeschaltet, solange der INA228 eine Akkuspannung über 0 und unter 3200 mV meldet und keine Eingangsquelle qualifiziert ist — mit aktivem TS braucht der ADC des BQ25798 im reinen Akkubetrieb VBAT ≥ 3,2 V, und das Abschalten von TS senkt diese Schwelle auf 2,9 V, damit die Solarwerte weiter funktionieren. Mit qualifizierter Eingangsquelle bleibt der Kanal an, eine kalte und fast leere Zelle meldet ihre Temperatur also während des Ladens.
3. Der dekodierte Wert liegt innerhalb von −50 … +90 °C.
4. Der Wert besteht die BME280-Plausibilitätsprüfung (siehe unten).

Die Akkuchemie spielt bei dieser Entscheidung keine Rolle. Der TS-Kanal läuft für jede Chemie, daher melden LiFePO4, Li-ion, LTO und Na-ion alle eine Akkutemperatur, sobald ein NTC bestückt ist und VBAT den Kanal zulässt.

Bedingung 2 ist die, die im Feld auffällt: bei nominal 3,2 V (LiFePO4) oder 3,1 V (Na-ion) liegen diese Chemien über weite Teile ihrer Entladekurve unter 3200 mV, und ohne Solareingang steht die Akkutemperatur dort auch mit bestücktem NTC auf N/A — bei einem Solarknoten also nachts. LTO 2S (4,6–5,4 V) bleibt durchgehend über der Schwelle, Li-ion 1S liegt über den größten Teil seiner Kurve darüber.

#### BME280-Plausibilitätsprüfung

Ein fehlender oder offener NTC kann zu einem Wert dekodieren, den die Fensterprüfung durchlässt. Mit RT1 = 5,6 kΩ und RT2 = 27 kΩ sitzt der Teiler auf dem Pol, an dem nur noch RT2 wirkt. Genau auf dem Pol liefert die Dekodierung −99 °C, was das Fenster −50 … +90 °C abfängt. Einen TS-ADC-Schritt (0,09765625 % von REGN) daneben landet der Wert bei etwa −46 °C, innerhalb dieses Fensters und von einem echten Tiefkältewert nicht zu unterscheiden. Jeder angenommene Messwert wird deshalb mit einer frischen BME280-Messung verglichen:

| BME280-Messwert | Differenz (kalibrierter NTC-Wert vs. BME280) | Ergebnis |
|-----------------|----------------------------------------------|----------|
| Über −100 °C und unter +100 °C | ≤ 15,0 °C | Wert wird gesendet |
| Über −100 °C und unter +100 °C | > 15,0 °C | Wird durch −999 ersetzt → N/A |
| Nicht lesbar (liefert −999) | wird nicht ausgewertet | Prüfung entfällt, Wert wird unverändert gesendet |

Eine Differenz von exakt 15,0 °C besteht die Prüfung. Verglichen wird der kalibrierte Wert (Rohmesswert + Offset aus `set board.tccal`).

Ein verworfener Messwert frischt außerdem den zwischengespeicherten Wert für das SOC-Derating nicht auf; nach 5 Minuten ohne angenommenen NTC-Messwert fällt das Derating auf den BME280 zurück. Die Auto-Kalibrierung `set board.tccal` ist davon nicht betroffen — sie liest den BQ25798-Treiber direkt mit genulltem Offset und überspringt nur dessen eigene Fehlercodes.

### Sentinel-Werte Temperatur

Der BQ25798-Treiber erzeugt diese Werte:

| Wert | Bedeutung |
|------|-----------|
| −999 °C | I²C-Kommunikationsfehler |
| −888 °C | ADC nicht bereit: One-Shot nicht abgeschlossen, TS-Register lieferte auch nach drei Versuchen 0 oder 0xFFFF, oder TS-Kanal abgeschaltet |
| −99 °C | NTC offen (nicht angeschlossen), oder die Dekodierung landete auf dem Pol, an dem nur noch RT2 wirkt |
| +99 °C | NTC kurzgeschlossen |

Alles außerhalb von −50 … +90 °C wird in der Board-Schicht in −999 umgesetzt, und ein von der BME280-Prüfung verworfener Messwert wird ebenfalls zu −999. Damit ist −999 der einzige Sentinel-Wert, der diese Schicht verlässt, und er trägt gleich drei Ursachen: I²C-Fehler, Treiber-Fehlercode oder unplausibler Messwert. Die drei sind von außen nicht unterscheidbar.

Werte ≤ −100 °C werden nicht gesendet: Das CayenneLPP-Temperaturfeld auf dem Akku-Kanal entfällt vollständig, und `get board.telem` gibt an dieser Stelle `N/A` aus.

---

## Kanal 4 — Solar (BQ25798)

Solareingangsdaten vom BQ25798-Ladecontroller.

| Feld | LPP-Typ | Einheit | Quelle | Beschreibung |
|------|---------|---------|--------|--------------|
| Spannung | Voltage | V | BQ25798 | Spannung am Solareingang (VBUS) |
| Strom | Current | A | BQ25798 | Solarstrom (IBUS) |
| MPPT 7-Tage | Percentage | % | Firmware | MPPT-Aktivierung der letzten 7 Tage. Zeigt, wie viel Prozent der Zeit der MPPT-Regler aktiv Solar-Energie eingespeist hat. |

> **Hinweis — Genauigkeit Solarstrom:** Der BQ25798 IBUS-ADC hat eine Auflösung von 1 mA (15-Bit-Modus), zeigt jedoch bei niedrigen Strömen einen erheblichen Messfehler (~±30 mA). Werte unter ca. 150 mA sollten nur als grobe Schätzungen betrachtet werden. Für präzise Strommessung nutzt die Akku-Seite stattdessen den INA228.

> **Hinweis:** Der MPPT-Prozentwert ist ein gleitender 7-Tage-Durchschnitt. Ein niedriger Wert (z. B. 1 %) bedeutet, dass das Panel nur selten genug Leistung liefert, um den MPPT-Regler zu aktivieren — z. B. bei bedecktem Himmel oder ungünstigem Panelwinkel.

---

## Welche Temperatur ist welche

Das MR2 meldet vier Temperaturen von vier verschiedenen Sensoren:

| Wo sie erscheint | Wert | Sensor | Anmerkungen |
|------------------|------|--------|-------------|
| Kanal 1 | MCU-Chiptemperatur | Sensor im nRF52840 | Wird vom allgemeinen MeshCore-Telemetriepfad angehängt (`getMCUTemperature()`), zusammen mit der Basis-Akkuspannung. Läuft unter Last wärmer als die Umgebung |
| Kanal 2 | Umgebungs-/Boardtemperatur | BME280 | Zugleich Referenz für `set board.tccal`, für die Plausibilitätsprüfung und als Rückfallebene für das SOC-Derating |
| Kanal 3 | Akkutemperatur | NTC am TS-Pin des BQ25798 | Steinhart-Hart-Dekodierung + `tccal`-Offset + Plausibilitätsprüfung; entfällt, wenn nicht verfügbar |
| `get board.cinfo`, Feld `TDIE:` | Chiptemperatur des Laderegler-ICs | Sensor im BQ25798 | Sperrschichttemperatur des Ladecontrollers, aus dem letzten abgeschlossenen ADC-One-Shot, also bis zu eine Telemetrieperiode alt. Läuft beim Laden wärmer als die Umgebung. Liegt auf keinem LPP-Kanal |

Nur der Wert auf Kanal 3 wird am Akku gemessen.

---

## Kanalzuordnung im Code

Die Kanäle werden dynamisch zugewiesen:

1. **Kanal 1** (`TELEM_CHANNEL_SELF`) ist fest definiert und enthält die MeshCore-Basisdaten (Akkuspannung und MCU-Chiptemperatur).
2. `querySensors()` weist jedem aktiven Sensor einen eigenen Kanal direkt nach Kanal 1 zu — der BME280 landet daher auf **Kanal 2**.
3. Der **Akku-Kanal** wird von `queryBoardTelemetry()` als nächster freier Kanal ermittelt (`findNextFreeLppChannel`).
4. Der **Solar-Kanal** = Akku-Kanal + 1.

`querySensors()` belegt Kanal 2 mit dem BME280, bevor `queryBoardTelemetry()` läuft — in der Praxis landen die Akkudaten daher auf Kanal 3 und Solar auf Kanal 4.

```
Reihenfolge im CayenneLPP-Paket:
┌────────────────────────────────────────────────┐
│ Kanal 1: Spannung (INA228 / SOC-Fake)          │  ← MyMesh.cpp (getBattMilliVolts)
│ Kanal 2: Temp., Luftfeuchte, Luftdruck, Höhe   │  ← BME280 (querySensors)
│ Kanal 3: VBAT, [SOC], IBAT, [TBAT], [Batt-TTL] │  ← queryBoardTelemetry()
│ Kanal 4: VSOL, ISOL, MPPT%                     │  ← queryBoardTelemetry()
│ Kanal 1: MCU-Chiptemperatur                    │  ← MyMesh.cpp (getMCUTemperature)
└────────────────────────────────────────────────┘
```

Felder in eckigen Klammern sind optional: `[SOC]` und `[Batt-TTL]` brauchen einen gültigen Referenzpunkt des Coulomb-Counters, `[TBAT]` eine verfügbare Akkutemperatur.

> **Berechtigungen:** Kanal 2 bis 4 werden nur gesendet, wenn der anfragende Client die Berechtigung `TELEM_PERM_ENVIRONMENT` besitzt. Gäste (Guest-Rolle) erhalten ausschließlich Kanal 1 mit Basisspannung und MCU-Temperatur.

## Siehe auch

- [README.md](README.md) — Übersicht, Feature-Matrix und Diagnose
- [DATASHEET.md](DATASHEET.md) — Hardware-Spezifikationen und Pinout
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle board-spezifischen CLI-Kommandos
- [QUICK_START.md](QUICK_START.md) — Schnelleinstieg und CLI-Konfiguration
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Akkuchemie-Vergleich und Einsatzempfehlungen
- [FAQ.md](FAQ.md) — Häufig gestellte Fragen
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Vollständige technische Dokumentation
