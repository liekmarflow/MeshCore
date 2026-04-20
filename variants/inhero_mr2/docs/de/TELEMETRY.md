# Telemetrie-Kanäle

> 🇬🇧 [English version](../TELEMETRY.md)

Das Inhero MR-2 sendet Telemetriedaten im [CayenneLPP](https://docs.mydevices.com/docs/lorawan/cayenne-lpp)-Format über drei Kanäle. Die Companion-App zeigt diese als **Kanal 1–3** an.

---

## Kanal 1 — Gerätestatus & Umgebung

Basisdaten des Nodes plus BME280-Umgebungssensor (immer auf dem MR-2 vorhanden).

| Feld | Einheit | Quelle | Beschreibung |
|------|---------|--------|--------------|
| Akkustand | % / V | INA228 | Siehe Hinweis zum SOC-Workaround unten |
| Temperatur | °C / °F | BME280 | Umgebungstemperatur |
| Relative Luftfeuchtigkeit | % | BME280 | Relative Luftfeuchtigkeit |
| Luftdruck | hPa | BME280 | Barometrischer Druck |
| Höhe | m / ft | BME280 | Aus dem Luftdruck berechnete Höhe (Referenz: Meereshöhe) |

### Akkustand & SOC-Workaround

MeshCore überträgt auf Kanal 1 aktuell nur die Batterie-**Spannung** — es gibt kein natives SOC%-Feld. Die Companion-App rechnet diese Spannung über eine fest hinterlegte **Li-Ion-Entladekurve** in Prozent um. Das funktioniert gut für Li-Ion-Zellen, liefert aber falsche Werte für LiFePO₄, LTO oder Na-Ion (deren Spannungskurve deutlich flacher verläuft).

Das MR-2 umgeht diese Einschränkung:

| SOC-Status | Was `getBattMilliVolts()` liefert | App zeigt |
|------------|-----------------------------------|-----------|
| **SOC noch nicht gültig** | Echte Batteriespannung vom INA228 | Prozent basierend auf Li-Ion-Kurve (kann bei Nicht-Li-Ion ungenau sein) |
| **SOC gültig** (Coulomb-Counter kalibriert) | Fake Li-Ion OCV, rückgerechnet aus echtem SOC% (`socToLiIonMilliVolts()`) | Korrekter Prozentwert — die Li-Ion-Kurve der App dekodiert zurück zum ursprünglichen SOC% |

> **OCV** = Open Circuit Voltage (Leerlaufspannung) — die Ruhespannung der Batterie ohne Last. Die OCV-Kurve (Spannung vs. SOC%) ist charakteristisch für jede Batteriechemie und wird hier als Lookup-Tabelle verwendet, um SOC% zurück in eine Spannung umzurechnen, die die App interpretieren kann.

Der SOC wird gültig, wenn **beide** Bedingungen erfüllt sind:

1. **Batteriekapazität gesetzt** — via `set board.batcap <mAh>`
2. **SOC-Referenzpunkt vorhanden** — entweder manuell via `set board.soc <Prozent>` oder automatisch bei einem „Charging Done"-Event (setzt SOC auf 100 %)

Die Rückrechnung nutzt eine stückweise lineare Li-Ion-OCV-Tabelle (3000 mV bei 0 % → 4200 mV bei 100 %). So zeigt die App den korrekten Coulomb-gezählten SOC unabhängig von der tatsächlichen Batteriechemie an.

> **Hinweis:** Die Höhenberechnung basiert auf dem Standard-Meeresspiegeldruck (1013,25 hPa) und kann je nach Wetterlage abweichen.

---

## Kanal 2 — Batterie (INA228 / BQ25798)

Hochpräzise Batteriedaten vom INA228-Coulomb-Counter und BQ25798-Ladecontroller.

| Feld | LPP-Typ | Einheit | Quelle | Beschreibung |
|------|---------|---------|--------|--------------|
| Spannung | Voltage | V | INA228 | Batteriespannung (24-Bit-ADC, ±0,1 % Genauigkeit) |
| SOC | Percentage | % | INA228 | Ladezustand per Coulomb-Counting — *optional, nur wenn kalibriert* |
| Strom | Current | A | INA228 | Batteriestrom. Negativ = Entladung, positiv = Ladung |
| Temperatur | Temperature | °C / °F | BQ25798 NTC | Batterietemperatur am NTC-Fühler |
| TTL | Distance | Tage | berechnet | Geschätzte Restlaufzeit — *optional, nur bei gültigem SOC* |

### SOC & TTL

SOC und TTL erscheinen nur, wenn der Coulomb-Counter einen gültigen Referenzpunkt hat. Dafür muss die Batteriekapazität konfiguriert sein (`set board.batcap`) und entweder ein manueller SOC gesetzt (`set board.soc`) oder ein „Charging Done"-Event eingetreten sein. Solange der SOC nicht gültig ist, werden diese Felder weggelassen.

### TTL-Kodierung

Der TTL-Wert (Time-To-Live) wird als **CayenneLPP-Distance-Wert** in Tagen übertragen, da CayenneLPP keinen nativen „Dauer"-Typ hat. Die Companion-App zeigt ihn als Entfernung an (z. B. „42 m"), aber der Wert repräsentiert **Tage Restlaufzeit**.

| Bedingung | Übertragener Wert | Bedeutung |
|-----------|-------------------|-----------|
| Endliche TTL | `ttlHours / 24.0` | Geschätzte verbleibende Tage im Akkubetrieb |
| Überschuss (Ladung > Verbrauch) | `990.0` (max. kodierbar) | Praktisch unendlich — Gerät gewinnt Ladung |
| Unbekannt (SOC noch nicht gültig) | *nicht gesendet* | TTL kann noch nicht berechnet werden |

### Sentinel-Werte Temperatur

Ungültige Temperaturwerte werden durch Sentinel-Werte angezeigt und nicht an die App gesendet:

| Wert | Bedeutung |
|------|-----------|
| −999 °C | I²C-Kommunikationsfehler |
| −888 °C | ADC noch nicht bereit |
| −99 °C | NTC offen (nicht angeschlossen) |
| +99 °C | NTC kurzgeschlossen |

---

## Kanal 3 — Solar (BQ25798)

Solareingangsdaten vom BQ25798-Ladecontroller.

| Feld | LPP-Typ | Einheit | Quelle | Beschreibung |
|------|---------|---------|--------|--------------|
| Spannung | Voltage | V | BQ25798 | Spannung am Solareingang (VBUS) |
| Strom | Current | A | BQ25798 | Solarstrom (IBUS) |
| MPPT 7-Tage | Percentage | % | Firmware | MPPT-Aktivierung der letzten 7 Tage. Zeigt, wie viel Prozent der Zeit der MPPT-Regler aktiv Solar-Energie eingespeist hat. |

> **Hinweis — Genauigkeit Solarstrom:** Der BQ25798 IBUS-ADC hat eine Auflösung von 1 mA (15-Bit-Modus), zeigt jedoch bei niedrigen Strömen einen erheblichen Messfehler (~±30 mA). Werte unter ca. 150 mA sollten nur als grobe Schätzungen betrachtet werden. Für präzise Strommessung nutzt die Batterie-Seite stattdessen den INA228.

> **Hinweis:** Der MPPT-Prozentwert ist ein gleitender 7-Tage-Durchschnitt. Ein niedriger Wert (z. B. 1 %) bedeutet, dass das Panel nur selten genug Leistung liefert, um den MPPT-Regler zu aktivieren — z. B. bei bedecktem Himmel oder ungünstigem Panelwinkel.

---

## Kanalzuordnung im Code

Die Kanäle werden dynamisch zugewiesen:

1. **Kanal 1** (`TELEM_CHANNEL_SELF`) ist fest definiert und enthält die MeshCore-Basisdaten plus BME280.
2. **Kanal 2** wird von `queryBoardTelemetry()` als nächster freier Kanal ermittelt (`findNextFreeChannel`).
3. **Kanal 3** = Kanal 2 + 1.

Da Kanal 1 der einzige vor `queryBoardTelemetry()` belegte Kanal ist, ergibt sich in der Praxis immer Kanal 2 (Batterie) und Kanal 3 (Solar).

```
Reihenfolge im CayenneLPP-Paket:
┌─────────────────────────────────────────────┐
│ Kanal 1: Spannung (INA228 / SOC-Fake)      │  ← MyMesh.cpp (getBattMilliVolts)
│ Kanal 1: Temp, Humidity, Pressure, Altitude │  ← BME280 (querySensors)
│ Kanal 2: VBAT, [SOC], IBAT, TBAT, [TTL]   │  ← queryBoardTelemetry()
│ Kanal 3: VSOL, ISOL, MPPT%                 │  ← queryBoardTelemetry()
└─────────────────────────────────────────────┘
```

> **Berechtigungen:** Kanal 2 und 3 werden nur gesendet, wenn der anfragende Client die Berechtigung `TELEM_PERM_ENVIRONMENT` besitzt. Gäste (Guest-Rolle) erhalten ausschließlich Kanal 1 mit Basisspannung.

## Siehe auch

- [README.md](README.md) — Übersicht, Feature-Matrix und Diagnose
- [DATASHEET.md](DATASHEET.md) — Hardware-Spezifikationen und Pinout
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle board-spezifischen CLI-Kommandos
- [QUICK_START.md](QUICK_START.md) — Schnelleinstieg und CLI-Konfiguration
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Vollständige technische Dokumentation
