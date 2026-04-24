# Inhero MR2 — FAQ

> 🇬🇧 [English Version](../FAQ.md)

## Inhalt


**⚡ Akku & Chemie**

1. [Welche Akkuchemie soll ich einsetzen?](#1-welche-akkuchemie-soll-ich-einsetzen)
2. [Kann ich auch Akkupacks ohne eingebauten NTC nutzen?](#2-kann-ich-auch-akkupacks-ohne-eingebauten-ntc-nutzen)
3. [Warum steigt die Stromaufnahme bei sinkender Batteriespannung?](#3-warum-steigt-die-stromaufnahme-bei-sinkender-batteriespannung)

**🔋 Laden & Solar**

4. [Welchen mAh-Wert gebe ich bei `set board.batcap` ein?](#4-welchen-mah-wert-gebe-ich-bei-set-boardbatcap-ein)
5. [Warum ist es wichtig, den maximalen Ladestrom `set board.imax` einzustellen?](#5-warum-ist-es-wichtig-den-maximalen-ladestrom-set-boardimax-einzustellen)
6. [Was wird durch `set board.fmax` beeinflusst?](#6-was-wird-durch-set-boardfmax-beeinflusst)
7. [Kann ich das Board über USB laden?](#7-kann-ich-das-board-über-usb-laden)
8. [Welche Solarpanels kann ich anschließen?](#8-welche-solarpanels-kann-ich-anschließen)
9. [Die rote LED (BQ-Status-LED) blinkt langsam und der Akku wird nicht geladen.](#9-die-rote-led-bq-status-led-blinkt-langsam-und-der-akku-wird-nicht-geladen)
10. [Warum lädt das Board nicht ohne geflashte Firmware?](#10-warum-lädt-das-board-nicht-ohne-geflashte-firmware)

**📊 SOC & Überwachung**

11. [Warum zeigt der SOC 0% oder N/A an?](#11-warum-zeigt-der-soc-0-oder-na-an)
12. [Wann sollte ich `set board.tccal` ausführen?](#12-wann-sollte-ich-set-boardtccal-ausführen)
13. [Wie funktioniert das Temperatur-Derating?](#13-wie-funktioniert-das-temperatur-derating)
14. [Was ist TTL (Time-To-Live)?](#14-was-ist-ttl-time-to-live)

**🔩 Hardware**

15. [Hat das Board einen Verpolschutz?](#15-hat-das-board-einen-verpolschutz)
16. [Was macht der Schalter „3.3V off“ und wann verwende ich ihn?](#16-was-macht-der-schalter-33v-off-und-wann-verwende-ich-ihn)
17. [Was bedeuten die LEDs?](#17-was-bedeuten-die-leds)
18. [Darf ich das Board ohne Antenne betreiben?](#18-darf-ich-das-board-ohne-antenne-betreiben)
19. [Warum hat die RTC keine Stützbatterie?](#19-warum-hat-die-rtc-keine-stützbatterie)
20. [Welche Maße haben die Befestigungsbohrungen?](#20-welche-maße-haben-die-befestigungsbohrungen)
21. [Sind Schnittstellen (UART/I2C) am Board herausgeführt?](#21-sind-schnittstellen-uarti2c-am-board-herausgeführt)

**⚙️ Firmware**

22. [Bleiben meine Einstellungen bei einem Firmware-Update erhalten?](#22-bleiben-meine-einstellungen-bei-einem-firmware-update-erhalten)
23. [Warum braucht das Repeater-Board eine korrekte Uhrzeit?](#23-warum-braucht-das-repeater-board-eine-korrekte-uhrzeit)
24. [Warum kann die Repeater-Uhr nicht zurückgestellt werden?](#24-warum-kann-die-repeater-uhr-nicht-zurückgestellt-werden)

---

**⚡ Akku & Chemie**

### 1. Welche Akkuchemie soll ich einsetzen?

Das Inhero MR2 unterstützt **Li-Ion**, **LiFePO4**, **LTO (2S)** und **Na-Ion**. Die richtige Wahl hängt von den Einsatzbedingungen ab — insbesondere Temperaturbereich, verfügbarer Bauraum und erwartete Lebensdauer.

Kurzempfehlung: **LiFePO4** für die meisten Indoor-/gemäßigten Setups, **LTO** für extreme Kälte oder maximale Zyklenlebensdauer, **Li-Ion** wenn der Bauraum knapp ist, **Na-Ion** für nachhaltige Kälteeinsätze.

→ **Ausführlicher Guide:** [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Detaillierter Vergleich, Vor- & Nachteile, Einsatzempfehlungen, Kapazitätsplanung, Solar-Dimensionierung, Sicherheitshinweise und Langzeitalterung.

→ **Setup:** [QUICK_START.md — Schritt 6](QUICK_START.md#6-akkuchemie-setzen) | [CLI_CHEAT_SHEET.md — Schnellstart-Rezepte](CLI_CHEAT_SHEET.md#schnellstart-rezepte)

---

### 2. Kann ich auch Akkupacks ohne eingebauten NTC nutzen?

**Ja, aber nur mit dem Onboard-NTC.** Schließe die Solder-Bridge auf der Rückseite des Boards – damit wird der Onboard-NTC (NCP15XH103F03RC, 10 kΩ @ 25 °C, Beta 3380) aktiviert. Der TS-Pin am Batteriestecker bleibt dann unbenutzt.

Falls dein Akkupack einen eingebauten NTC hat, muss dieser zwischen **TS (Pin 3)** und **GND (Pin 2)** verdrahtet sein (siehe [DATASHEET.md — Batterie-Stecker](DATASHEET.md#steckerbelegung--batterie-stecker-jst-ph20-3p-von-links-nach-rechts)). Ein kompatibler 10k-NTC (Beta ~3380) reicht für grundlegende Frostschutzfunktion – die Temperaturgenauigkeit sinkt jedoch etwas.

**Wichtig:** Ohne NTC (Solder-Bridge offen und kein externer NTC angeschlossen) interpretiert der BQ25798 den TS-Pin bei Li-Ion und LiFePO4 als Frostbedingung – das Laden wird blockiert und die BQ-Status-LED blinkt. Bei LTO und Na-Ion tritt dieses Problem nicht auf, da JEITA dort deaktiviert ist.

---

### 3. Warum steigt die Stromaufnahme bei sinkender Batteriespannung?

Das Inhero MR2 hat einen hocheffizienten **Buck-Converter**, der die Batteriespannung auf 3,3 V für MCU und Funk herunterregelt. Weil dieser Wandler effizient arbeitet, zieht das Board eine annähernd **konstante Leistung** (Watt), keinen konstanten Strom (Ampere).

Weil Leistung = Spannung × Strom:
- Bei 4,6 V (LTO voll): ~6,3 mA
- Bei 3,7 V (Li-Ion Nennspannung): ~7,8 mA
- Bei 3,2 V (LiFePO4 Nennspannung): ~9,1 mA

Alle drei Fälle verbrauchen exakt **29 mW**. Das ist normales Verhalten, kein Fehler.

**Praktische Konsequenz:** Akkus immer in **Wh** (Energie) dimensionieren, nicht in mAh — besonders beim Vergleich verschiedener Chemien. Eine naive „mA × Stunden“-Rechnung überschätzt den Kapazitätsbedarf für höherspannige Chemien wie LTO.

→ **Ausführliche Erklärung:** [BATTERY_GUIDE.md — Warum der Strom von der Batteriespannung abhängt](BATTERY_GUIDE.md#warum-der-strom-von-der-batteriespannung-abhängt)

---

**🔋 Laden & Solar**

### 4. Welchen mAh-Wert gebe ich bei `set board.batcap` ein?

Gib die **Nennkapazität abzüglich eines Abschlags** ein. Da die Ladeschlussspannung zugunsten der Akkuschonung reduziert ist, steht nicht die volle Nennkapazität zur Verfügung. Ein etwas pessimistischerer Wert ist sicherer: Wenn der SOC 10 % anzeigt, sind dann auch wirklich ≥ 10 % im Akku. So wirst du nicht von einem unerwarteten Low-Voltage-Sleep überrascht. Auch die TTL-Vorhersage (Time-To-Live) wird damit konservativer und zuverlässiger.

**Faustregel: 90 % der Nennkapazität.** Beispiel: 10.000 mAh Nennkapazität → `set board.batcap 9000`.

Bei parallelgeschalteten Zellen addieren sich die Kapazitäten vor dem Abschlag: Zwei 5.000-mAh-Zellen parallel = 10.000 mAh Nenn → `set board.batcap 9000`.

---

### 5. Warum ist es wichtig, den maximalen Ladestrom `set board.imax` einzustellen?

`imax` setzt den **maximalen Ladestrom** — den maximalen Strom, der in den Akku fließt. Die Firmware nutzt `imax` zusammen mit der Akkuspannung auch, um automatisch zu berechnen, wie viel Strom sie vom Solarpanel ziehen darf. Das verhindert, dass schwache Panels überlastet werden und der Laderegler abschaltet.

Warum `imax` korrekt einstellen?

1. **Grundlage für Frostschutz:** `imax` ist der Referenzwert für `fmax`. Beispiel: `imax 500` mit `fmax 20%` ergibt max. 100 mA Ladestrom im T-Cool-Bereich (+3 °C bis –2 °C).

2. **Akkuschonung:** Geringere Ladeströme sind immer schonender für den Akku. Setze `imax` nur so hoch wie nötig.

3. **Panel-Kompatibilität:** Ist `imax` zu hoch gesetzt, versucht das Board kurzzeitig mehr Strom vom Panel zu ziehen als es liefern kann — der Laderegler erkennt den Spannungseinbruch und stoppt das Laden.

**Berechnung:** Panelleistung ÷ Akkuspannung = imax.
Beispiel: 2-W-Panel, Li-Ion (3,7 V) → 2000 / 3,7 ≈ 540 mA → `set board.imax 540`.

---

### 6. Was wird durch `set board.fmax` beeinflusst?

`fmax` begrenzt den maximalen Ladestrom im **T-Cool-Bereich** (+3 °C bis –2 °C mit Inhero-Spannungsteiler) auf einen Prozentsatz von `imax`:

| Einstellung | Verhalten im T-Cool-Bereich |
|---|---|
| `0%` | Laden komplett blockiert |
| `20%` | Max. 20 % von imax (z. B. 500 mA → 100 mA) |
| `40%` | Max. 40 % von imax (z. B. 500 mA → 200 mA) |
| `100%` | Keine Reduzierung, voller Ladestrom |

**Unterhalb von ca. –2 °C (T-Cold)** wird das Laden für **Li-Ion und LiFePO4** per JEITA immer komplett blockiert – unabhängig von `fmax`.

**Wichtig:** Nur das Laden wird eingeschränkt. Bei ausreichend Solarleistung läuft das Board weiterhin auf Solarstrom – der Akku wird weder geladen noch entladen.

**LTO / Na-Ion:** `fmax` hat keine Wirkung, da JEITA für diese Chemien deaktiviert ist.

---

### 7. Kann ich das Board über USB laden?

**Ja.** USB-C VBUS (5 V) ist über eine **SS34-Schottky-Diode** mit dem BQ25798-VBUS-Eingang verbunden — dem **gleichen Eingang** wie das Solarpanel (siehe [DATASHEET.md — USB-Ladepfad](DATASHEET.md#usb-ladepfad)). Der BQ25798 hat nur einen VBUS-Eingang und unterscheidet nicht zwischen den beiden Quellen.

Bei erkanntem USB (nRF52840 VBUS-Sense) begrenzt die Firmware automatisch den Eingangsstrom auf **500 mA** (USB 2.0 Spec). Wird USB entfernt, wird das Eingangsstrom-Limit aus Akkuspannung und `board.imax` neu berechnet.

Es ist jeweils die Quelle aktiv, die die höhere Spannung am VBUS-Eingang liefert: Wenn die USB-Spannung (abzüglich Schottky-Drop) höher ist als die Solarspannung, lädt USB. Andernfalls lädt Solar. Beide Quellen können nicht gleichzeitig laden.

> **⚠ WARNUNG:** Die SS34-Diode verhindert Rückfluss vom Solarpanel zum USB-Bus, aber Strom **kann** von USB-VBUS durch den Solaranschluss abfließen. Ein **Kurzschluss am Solaranschluss verursacht auch einen Kurzschluss am USB-VBUS**. Niemals den Solareingang kurzschließen, solange USB angeschlossen ist.

---

### 8. Welche Solarpanels kann ich anschließen?

**Anforderungen:**
- **Eingangsspannung:** 3,6 V – 24 V (MPPT-Bereich des BQ25798)
- **Max. Leerlaufspannung (Voc):** 25 V – nicht überschreiten!
- **Stecker:** JST PH2.0-2P (Solar+, Solar–)

**Typische Panels:** 5 V oder 6 V monokristalline Solarpanels. Der Buck/Boost-Charger kann auch aus niedrigerer Panelspannung höhere Akkuspannungen laden (z. B. 5 V Panel → LTO 2S bei 5,4 V).

**Nicht geeignet:** 24-V-Panels oder Reihenschaltungen, deren Voc > 25 V überschreiten kann. Siehe [DATASHEET.md — Technische Daten](DATASHEET.md#technische-daten) für die vollständigen elektrischen Grenzwerte.

**Dimensionierung (Mitteleuropa):**
- **1 W monokristallin** ist die Minimalanforderung – nur bei Südausrichtung, vertikaler Montage, unverschattet und mit Akkukapazität ≥ 7 Ah.
- **Ab 2 W** ist sicherer Ganzjahresbetrieb möglich.

---

### 9. Die rote LED (BQ-Status-LED) blinkt langsam und der Akku wird nicht geladen.

Langsames Blinken der BQ-Status-LED signalisiert einen **Charger-Fehler**. Häufigste Ursachen:

1. **Kein NTC angeschlossen (häufigste Ursache):** Weder externer NTC am TS-Pin noch Solder-Bridge für den Onboard-NTC geschlossen. Der BQ25798 interpretiert den offenen TS-Pin als Frostbedingung und blockiert das Laden. → **Lösung:** Solder-Bridge schließen oder kompatiblen NTC (10 kΩ @ 25 °C, Beta ~3380) zwischen TS (Pin 3) und GND (Pin 2) anschließen.

2. **Tatsächlich zu kalt / zu warm:** Unterhalb von ca. –2 °C (T-Cold-Schwelle mit Inhero-Spannungsteiler) wird das Laden für Li-Ion und LiFePO4 per JEITA komplett blockiert. Oberhalb von ca. 58 °C (T-Hot-Schwelle) wird das Laden ebenfalls ausgesetzt. → Bei LTO und Na-Ion tritt dies nicht auf (JEITA deaktiviert).

3. **Anderer Charger-Fehler:** Der BQ25798 kann auch Fehler wie VBAT-Überspannung (VBAT_OVP), Eingangsüberspannung (VBUS_OVP) oder Watchdog-Timeout signalisieren. Diese treten im Normalbetrieb selten auf.

→ Prüfe mit [`get board.telem`](CLI_CHEAT_SHEET.md#getters) die aktuelle Temperatur und mit [`get board.cinfo`](CLI_CHEAT_SHEET.md#getters) den Charger-Status und Fehler-Flags.

---

### 10. Warum lädt das Board nicht ohne geflashte Firmware?

Das ist ein bewusstes Sicherheitsfeature. Der BQ25798-Charger wird über den **CE-Pin (Charge Enable)** gesteuert, der von der Firmware aktiv auf GPIO4 HIGH gesetzt werden muss.

**Ohne Firmware** (oder mit aktiviertem 3.3V-off-Schalter):
- Externer Pull-Down am DMN2004TK-7-FET-Gate → FET OFF → CE HIGH → **Laden deaktiviert**

Dadurch wird sichergestellt, dass der Akku nicht überladen werden kann, wenn die Firmware hängt oder nicht installiert ist. Firmware per USB flashen, um das Laden zu aktivieren. Siehe [IMPLEMENTATION_SUMMARY.md — CE Pin Safety](IMPLEMENTATION_SUMMARY.md#11-bq25798-ce-pin-safety-rev-11--fet-invertiert) für das Hardware-Design.

---

**📊 SOC & Überwachung**

### 11. Warum zeigt der SOC 0% oder N/A an?

**SOC zeigt N/A**, bis der Akku **zum ersten Mal am Board vollständig geladen** wurde. Der Coulomb-Counter braucht einen bekannten Referenzpunkt (100% = „Charge Done“-Ereignis), um den SOC zuverlässig zu berechnen. Den Akku nach der Inbetriebnahme einmal komplett über USB laden. Siehe [IMPLEMENTATION_SUMMARY.md — Coulomb Counter & SOC](IMPLEMENTATION_SUMMARY.md#2-coulomb-counter--soc-ladezustand) für den Tracking-Mechanismus.

**SOC zeigt 0%** nach dem Aufwachen aus dem **Low-Voltage-Sleep**. Das ist beabsichtigt: Der Coulomb-Counter lief während des Sleeps nicht, daher ist der Ladestand unbekannt. Der SOC startet bei 0% und beginnt wieder zu zählen. Beim nächsten „Charge Done“ synchronisiert sich der SOC sauber auf 100%.

**Hinweis:** Bei Kälte kann der SOC% auch durch Temperatur-Derating niedriger erscheinen als erwartet — das ist korrektes Verhalten, kein Sensorfehler. Siehe [FAQ #13](FAQ.md#13-wie-funktioniert-das-temperatur-derating).

---

### 12. Wann sollte ich `set board.tccal` ausführen?

**Idealerweise am frühen Morgen**, vor Sonnenaufgang. Zu diesem Zeitpunkt hat sich die Akkutemperatur über Nacht an die Umgebungstemperatur angeglichen, und Sonneneinstrahlung hat das Gehäuse noch nicht aufgewärmt. Das liefert dem BME280 und dem NTC die konsistenteste Basis für die Kalibrierung.

**Warum der Zeitpunkt wichtig ist:** Tagsüber erwärmt Sonneneinstrahlung das Gehäuse ungleichmäßig — der NTC (nahe am Akku) und der BME280 (auf der Platine) messen unterschiedliche Temperaturen, was zu einem unpräzisen Offset führt. Am frühen Morgen befinden sich beide Sensoren im thermischen Gleichgewicht.

**Warum überhaupt TCCal?** Ein NTC und die zugehörigen Spannungsteiler-Widerstände unterliegen Bauteiltoleranzen, die Messfehler erzeugen, die wesentlich größer sind als die des BME280. Da der BME280 on-board ist, kann er als Referenz dienen, um den NTC-Messwert zu kalibrieren.

**Wichtige Einschränkungen:**
- **Betrifft nur Telemetrie und CLI.** TCCal korrigiert die Akkutemperatur, die über `get board.telem` und Telemetrie angezeigt bzw. übertragen wird. Die JEITA-Schwellen des BQ25798 werden **nicht** beeinflusst — der Charger wertet den TS-Pin direkt in Hardware aus. Daher können die tatsächlichen JEITA-Schalttemperaturen geringfügig von den kalibrierten CLI-Messwerten abweichen.
- **1-Punkt-Kalibrierung.** Der Offset wird bei einer Temperatur ermittelt. Abseits der Kalibriertemperatur driftet die Korrektur, da NTC-Nichtlinearität und Teilerfehler temperaturabhängig sind.

**Befehl:** `set board.tccal` — kalibriert den NTC-Offset automatisch mit dem BME280 als Referenz. Mit `set board.tccal reset` den Offset auf 0.00 zurücksetzen. Aktuellen Offset prüfen: [`get board.tccal`](CLI_CHEAT_SHEET.md#getter-schnellreferenz).

---

### 13. Wie funktioniert das Temperatur-Derating?

SOC% ist **rein Coulomb-basiert** — er spiegelt die tatsächlich gespeicherte Ladung wider und ändert sich nicht mit der Temperatur. Nur reale Ladungsströme (gemessen vom INA228 Coulomb-Counter) verändern den SOC%.

Die **entnehmbare Kapazität** sinkt jedoch bei Kälte durch verlangsamte elektrochemische Kinetik und erhöhten Innenwiderstand bei TX-Peaks (~100 mA). Die Firmware berechnet einen chemiespezifischen Derating-Faktor `f(T)`, der verwendet wird für:
- **TTL-Berechnung** — Trapped-Charge-Modell: entnehmbar = max(0, verbleibend − Kapazität × (1−f(T)))
- **CLI-Anzeige** — `get board.telem` zeigt den derateten Wert in Klammern: `SOC:95.0% (78%)` = gespeichert (entnehmbar)

Der Derating-Faktor ist sichtbar in `get board.socdebug` (Feld `d=`).

→ **Vollständige Details:** [IMPLEMENTATION_SUMMARY.md — §5a Temperatur-Derating](IMPLEMENTATION_SUMMARY.md#5a-temperatur-derating)

---

### 14. Was ist TTL (Time-To-Live)?

TTL ist eine geschätzte **verbleibende Laufzeit** basierend auf der aktuellen Energiebilanz. Angezeigt in [`get board.stats`](CLI_CHEAT_SHEET.md#getters). Siehe [IMPLEMENTATION_SUMMARY.md — TTL-Prognose](IMPLEMENTATION_SUMMARY.md#5-time-to-live-ttl-prognose) für den Algorithmus.

**Funktionsweise:**
- Ein 168-Stunden-Ringpuffer (7 Tage) erfasst stündlich Lade-/Entladedaten des INA228-Coulomb-Counters.
- **Formel:** `TTL = (SOC% × Kapazität / 100) / |7-Tage-Durchschnitt täglicher Nettoverbrauch| × 24h`
- **Anzeigeformat:** `T:12d0h` (12 Tage, 0 Stunden) oder `T:72h` (< 24 Stunden)

**TTL zeigt N/A oder 0, wenn:**
- Weniger als 24 Stunden Daten gesammelt wurden
- Das Board auf Solarüberschuss läuft (kein Defizit)
- Akkukapazität unbekannt (`set board.batcap` nicht gesetzt)

**Kältebetrieb:** TTL verwendet das Trapped-Charge-Modell — Kälte sperrt den Boden der Entladekurve, dadurch fällt die entnehmbare Kapazität bei niedrigem SOC besonders steil. Im Winter bei 20% SOC kann die entnehmbare Kapazität bereits nahe null sein. Siehe [FAQ #13](FAQ.md#13-wie-funktioniert-das-temperatur-derating).

---

**🔩 Hardware**

### 15. Hat das Board einen Verpolschutz?

**Nein.** Das Board hat **keinen Hardware-Verpolschutz** – weder am Batterie- noch am Solareingang. Ein verpolt angeschlossener Akku oder ein verpolt angeschlossenes Solarpanel kann zu **sofortigem, irreversiblem Schaden** am Board führen.

**Immer die Polarität prüfen, bevor ein Kabel eingesteckt wird.** Siehe [DATASHEET.md — Sicherheits- und Schutzfunktionen](DATASHEET.md#sicherheits--und-schutzfunktionen).

---

### 16. Was macht der Schalter „3.3V off“ und wann verwende ich ihn?

Der Schiebeschalter **„3.3V off“** unten links auf der Platine steuert den EN-Pin des TPS62840 Buck-Converters (siehe [DATASHEET.md — 3.3V Power Switch](DATASHEET.md#33v-power-switch-33v-off)).

> **⚠ Achtung — Invertierte Logik:**
> - Schalterstellung **„ON“** = EN-Pin low = Board **ausgeschaltet**
> - Schalterstellung **„OFF“** = EN-Pin high = Board **läuft**

Mit deaktivierter 3,3-V-Schiene sind der nRF52840, das RF-Frontend und alle 3,3-V-versorgten Komponenten (INA228, RV-3028 RTC, BME280) stromlos. Nur der BQ25798-Charger-IC bleibt über VBAT versorgt (~15 µA Ruhestrom). **Das Laden ist in diesem Zustand deaktiviert** — die Firmware muss laufen, um den Charger zu überwachen. Hinweis: Die RTC verliert ihre Zeit, wenn die 3,3-V-Schiene abgeschaltet ist — siehe [FAQ #23](#23-warum-braucht-das-repeater-board-eine-korrekte-uhrzeit).

**Einsatzfälle:**
- **Antennentausch:** RF-Frontend sicher abschalten, ohne Akku oder Solar abzuklemmen.
- **Transport:** Board während Versand oder Standortwechsel abschalten.
- **Kurz-/mittelfristige Lagerung:** ~15 µA Gesamtverbrauch. Bei längerer Lagerung (Monate) den Akku komplett abklemmen.

---

### 17. Was bedeuten die LEDs?

Das Board hat drei LEDs:

| LED | Position | Farbe | Bedeutung |
|-----|----------|-------|----------|
| **LED1** | Rechts, oben | Blau | Heartbeat (periodisches Blinken im Normalbetrieb). Kurzer Blitz beim Boot für jede erfolgreich initialisierte Komponente (INA228, BQ25798, RTC). |
| **LED2** | Rechts, unten | Rot | Hardware-Fehleranzeige. Blinkt dauerhaft, wenn eine kritische Komponente (BQ25798, INA228 oder RTC) bei der Initialisierung nicht gefunden wurde. |
| **Charge-LED** | Unten rechts, neben Solar-Anschluss | Rot | BQ25798-Ladestatus-Ausgang (hardware-gesteuert). Dauerleuchten = Laden aktiv. Aus = kein Laden oder Laden abgeschlossen. Langsames Blinken = Charger-Fehler (siehe FAQ #9). |

Alle drei LEDs können mit [`set board.leds off`](CLI_CHEAT_SHEET.md#setters) deaktiviert werden.

**Hinweis:** Die Angaben zu LED1/LED2 gelten nur nach dem Boot der Firmware. Der Bootloader verwendet eigene LED-Muster (z. B. langsames blaues Pulsen bei OTA-/UF2-Updates).

---

### 18. Darf ich das Board ohne Antenne betreiben?

**Nein.** Betrieb ohne Antenne riskiert **irreversible Schäden** am RF-Frontend (SX1262-Radio). Immer beide Antennen (LoRa und BLE) anschließen, bevor das Board eingeschaltet wird.

Wenn Antennen an einem bereits installierten Board gewechselt werden müssen, den **3.3V-off-Schalter** verwenden (siehe FAQ #16), um das RF-Frontend sicher stromlos zu schalten.

---

### 19. Warum hat die RTC keine Stützbatterie?

Die RV-3028-C7 RTC hat zwei Hauptaufgaben:
1. **Stabile Zeitbasis** mit minimaler Drift für MeshCore.
2. **Wake-Up-Timer** für den Low-Voltage-Sleep (stündliches Aufwachen zur Spannungsprüfung).

Solange ein Akku angeschlossen ist, wird die RTC dauerhaft versorgt – auch im System-Sleep. Nach einem Low-Voltage-Sleep und Reboot bleibt die Uhrzeit erhalten.

Eine Stützbatterie (z. B. CR2032) wurde bewusst weggelassen. Ihr einziger Zusatznutzen wäre, die Uhrzeit bei abgeklemmtem Akku zu erhalten. Das rechtfertigt nicht den Platzverbrauch auf dem kompakten 45 × 40 mm Formfaktor. Warum eine korrekte Uhrzeit wichtig ist, erklärt [FAQ #23](#23-warum-braucht-das-repeater-board-eine-korrekte-uhrzeit).

---

### 20. Welche Maße haben die Befestigungsbohrungen?

Das Board hat **4× M2.5-Befestigungsbohrungen** mit einem Durchmesser von **2,5 mm** und einem Lochabstand von **35 × 40 mm**. Die Platine selbst misst 45 × 40 mm.

---

### 21. Sind Schnittstellen (UART/I2C) am Board herausgeführt?

**Ja.** Auf der Rückseite der Platine befinden sich zwei Reihen mit Castellated Pads:

**Reihe 1 — UART / I2C:**

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | GND | Masse |
| 2 | RX | UART Empfang |
| 3 | TX | UART Senden |
| 4 | SDA | I2C Daten |
| 5 | SCL | I2C Takt |
| 6 | 3.3V | 3,3 V Ausgang (max. 500 mA, geteilt mit Board-Verbrauch) |

**Reihe 2 — SWD (Debug):**

| Pin | Signal | Beschreibung |
|-----|--------|-------------|
| 1 | RESET | nRF52840 Reset |
| 2 | GND | Masse |
| 3 | SWCLK | SWD Takt |
| 4 | SWDIO | SWD Daten |
| 5 | 3.3V | 3,3 V Ausgang (max. 500 mA, geteilt mit Board-Verbrauch) |

Die Castellated Pads können direkt auf eine Trägerplatine gelötet werden. Siehe [DATASHEET.md — Header & Pads](DATASHEET.md#header--pads--rückseite) für das vollständige Pad-Layout.

---

**⚙️ Firmware**

### 22. Bleiben meine Einstellungen bei einem Firmware-Update erhalten?

**Ja.** Alle board-spezifischen Einstellungen werden im **LittleFS-Dateisystem** gespeichert, das bei Firmware-Updates erhalten bleibt. Dazu gehören:

- Akkuchemie (`set board.bat`)
- Akkukapazität (`set board.batcap`)
- Ladestrom (`set board.imax`)
- Frostschutz (`set board.fmax`)
- MPPT, LED-Einstellungen
- NTC-Kalibrierungsoffset
- Energiestatistiken (stündliche/tägliche Ringpuffer für TTL-Berechnung)

Einstellungen gehen nur bei einem vollständigen Flash-Erase oder Dateisystem-Korruption (selten) verloren. Siehe [IMPLEMENTATION_SUMMARY.md — Statistics Persistence](IMPLEMENTATION_SUMMARY.md#12-statistics-persistence) für technische Details.

### 23. Warum braucht das Repeater-Board eine korrekte Uhrzeit?

Die Firmware nutzt die RTC (Real-Time Clock) für mehrere Schutzmechanismen. Eine falsch gehende Uhr führt nicht zum Totalausfall — Pakete werden weiterhin weitergeleitet — aber es treten spürbare Probleme auf:

- **Advertisements werden verworfen:** Jedes Advertisement ist kryptographisch signiert (Ed25519 über Public Key + Zeitstempel + App-Daten). Empfänger vergleichen den enthaltenen Zeitstempel mit dem zuletzt gespeicherten Wert und ignorieren kleinere oder gleiche Timestamps als Replay-Attacke. Der bestehende Eintrag auf anderen Nodes bleibt erhalten, wird aber nicht mehr aktualisiert — Name, Position und „zuletzt gesehen" veralten zunehmend.
- **Debug-Logs mit falschen Zeitstempeln:** `getLogDateTime()` zeigt falsche absolute Uhrzeiten. Relative Berechnungen wie „zuletzt gehört vor X Sekunden" bleiben korrekt, da sie intern dieselbe (falsche) Clock-Quelle nutzen.

Login, Admin-Befehle und der Rate-Limiter sind **nicht betroffen** — Login/Befehle vergleichen ausschließlich Client-Timestamps untereinander, und der Rate-Limiter nutzt nur relative Zeitdifferenzen, die bei monoton laufender Uhr korrekt bleiben. Ein `clock sync` kann also jederzeit nach dem Einloggen ausgeführt werden.

**Wie wird die Uhr gesetzt?**
Das Inhero MR2 hat eine Hardware-RTC, die bei einem normalen Reboot die Zeit behält. Wird der Akku getrennt oder die 3.3V-Rail per Onboard-Switch abgeschaltet, verliert der RV-3028 seine Zeit und fällt auf den POR-Default (Januar 2000) zurück. Die Uhr kann über CLI (`clock sync` oder `time <epoch>`) von einem Admin-Client gesetzt werden. Der Repeater wird **nicht automatisch** von Clients synchronisiert.

> **Empfehlung:** Nach jedem Akkuwechsel oder Abschaltung der 3.3V-Rail des Boards per Onboard-Switch zeitnah `clock sync` über die CLI ausführen. Ein normaler Reboot ist unkritisch.

---

### 24. Warum kann die Repeater-Uhr nicht zurückgestellt werden?

`clock sync` und `time <epoch>` erlauben nur ein **Vorstellen** der Uhr — Zurückstellen wird mit `ERR: clock cannot go backwards` abgelehnt.

**Warum?** Die Firmware nutzt steigende Timestamps als Schutz gegen Replay-Attacken. Sowohl Advertisements als auch Admin-Befehle werden verworfen, wenn ihr Timestamp kleiner oder gleich dem zuletzt gespeicherten ist. Da andere Nodes im Mesh den letzten (hohen) Timestamp gespeichert haben, würde ein Uhrrücksprung dazu führen, dass neue Advertisements meshweit abgelehnt werden.

**Lösung: `clkreboot`** — setzt die Uhr auf einen niedrigen Wert zurück, startet das Board neu und räumt die Client-Tabelle auf. Danach `clock sync` ausführen, um die korrekte Zeit zu setzen.

> **Hinweis:** Nach `clkreboot` werden Advertisements vorübergehend von Nodes verworfen, die noch den alten Timestamp gespeichert haben. Die Sichtbarkeit normalisiert sich, sobald die Einträge dort ablaufen.

Siehe auch [FAQ #23](#23-warum-braucht-das-repeater-board-eine-korrekte-uhrzeit).

---

## Siehe auch

- [README.md](README.md) — Übersicht, Feature-Matrix und Diagnose
- [DATASHEET.md](DATASHEET.md) — Hardware-Datenblatt, Pinouts und Spezifikationen
- [QUICK_START.md](QUICK_START.md) — Inbetriebnahme und CLI-Setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle Board-spezifischen CLI-Kommandos
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Vollständige technische Dokumentation
- [BATTERY_GUIDE.md](BATTERY_GUIDE.md) — Akkuchemie-Vergleich und Einsatzempfehlungen
