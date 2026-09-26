# Inhero MR2 — Akkuchemie-Ratgeber

> 🇬🇧 [English Version](../BATTERY_GUIDE.md)

## Inhalt

- [Einleitung](#einleitung)
- [1. Chemie-Übersicht](#1-chemie-übersicht)
  - [Li-ion (NMC/NCA, 1S)](#li-ion-nmcnca-1s)
  - [LiFePO4 (LFP, 1S)](#lifepo4-lfp-1s)
  - [LTO (Lithium-Titanat, 2S)](#lto-lithium-titanat-2s)
  - [Na-ion (Natrium-Ionen, 1S)](#na-ion-natrium-ionen-1s)
- [2. Vergleichstabelle](#2-vergleichstabelle)
- [3. Temperaturverhalten](#3-temperaturverhalten)
  - [Kälteperformance-Ranking](#kälteperformance-ranking)
  - [Laden bei Kälte](#laden-bei-kälte)
  - [Akkutemperatur-Erfassung](#akkutemperatur-erfassung)
  - [Temperatur-Derating](#temperatur-derating)
- [4. Zellauswahl & Bauformen](#4-zellauswahl--bauformen)
  - [Li-ion-Zellen](#li-ion-zellen)
  - [LiFePO4-Zellen](#lifepo4-zellen)
  - [LTO-Zellen](#lto-zellen)
  - [Na-ion-Zellen](#na-ion-zellen)
- [5. Kapazitätsplanung](#5-kapazitätsplanung)
  - [Stromverbrauch](#stromverbrauch)
  - [Warum der Strom von der Akkuspannung abhängt](#warum-der-strom-von-der-akkuspannung-abhängt)
  - [Dimensionierung für Autonomie](#dimensionierung-für-autonomie)
  - [Die 90%-Regel](#die-90-regel)
- [6. Solar-Ladebetrachtungen](#6-solar-ladebetrachtungen)
- [7. Sicherheit & Schutz](#7-sicherheit--schutz)
- [8. Einsatzempfehlungen](#8-einsatzempfehlungen)
- [9. Langzeit-Alterung & Zyklenlebensdauer](#9-langzeit-alterung--zyklenlebensdauer)
- [10. Zukunftsausblick](#10-zukunftsausblick)
- [Siehe auch](#siehe-auch)

---

## Einleitung

Die Wahl der richtigen Akkuchemie ist eine der folgenreichsten Entscheidungen beim Deployment eines Inhero MR2 Repeaters. Sie beeinflusst die Akkulaufzeit, Kältezuverlässigkeit, Lebensdauer, Sicherheit und Gesamtbetriebskosten. Dieser Ratgeber liefert die Informationen für eine fundierte Entscheidung.

Das Inhero MR2 unterstützt **vier Akkuchemien**, jede mit eigenen Stärken. Es gibt keine „beste" Chemie — die richtige Wahl hängt von den Einsatzbedingungen ab.

---

## 1. Chemie-Übersicht

### Li-ion (NMC/NCA, 1S)

Die verbreitetste Akkuchemie. NMC (Nickel-Mangan-Kobalt) und NCA (Nickel-Kobalt-Aluminium) Zellen dominieren den Konsumentenmarkt.

**Stärken:**
- **Höchste Energiedichte** (~250 Wh/kg) — kleinstes und leichtestes Format pro Kapazität
- Weit verbreitet in vielen Bauformen (18650, 21700, Pouch)
- Günstig — Massenproduktion drückt die Preise
- Ausgereifte Technologie mit Jahrzehnten an Felddaten

**Schwächen:**
- Begrenzte Zyklenlebensdauer (500–1000 Zyklen bis 80% Kapazität)
- **Risiko des thermischen Durchgehens** — kann bei Missbrauch (Überladung, Kurzschluss, Beschädigung) in Brand geraten
- Kälteempfindlich: deutlicher Kapazitätsverlust unter 0 °C
- Darf unter 0 °C nicht geladen werden — Lithium-Plating-Risiko schädigt die Zelle permanent
- Hitzeempfindlich: beschleunigte Alterung über 40 °C
- **Kalendarische Alterung bei hohem SOC + Hitze** — der Alterungstreiber Nr. 1 für Solar-Repeater, wo der Akku im Sommer monatelang bei 95–100% SOC in Gehäusen steht, die 50 °C erreichen. Daher senkt das Inhero MR2 Vco auf 4,1 V ab
- Schutzschaltung (BMS) erforderlich gegen Über-/Tiefentladung

**Inhero MR2 Besonderheiten:**
- Ladeschlussspannung auf **4,1 V** gesetzt (konservativ, statt typisch 4,2 V) für bessere Zyklenlebensdauer
- **JEITA aktiv** — NTC erforderlich; Ladung gesperrt unter −2 °C (T-Cold)
- Frost-Ladestromreduzierung konfigurierbar über `set board.fmax`
- `set board.jeitaignore 1` schaltet die Temperaturüberwachung des Ladereglers komplett ab, begrenzt durch ein 0,05C-Gate — auf eigenes Risiko des Betreibers, siehe [Laden bei Kälte](#laden-bei-kälte)

### LiFePO4 (LFP, 1S)

Eisenphosphat-Kathoden-Chemie. Beliebt in Solar- und Off-Grid-Anwendungen wegen Sicherheit und Langlebigkeit.

**Stärken:**
- **Hervorragende Zyklenlebensdauer** (2000–5000 Zyklen bis 80% Kapazität)
- **Kein thermisches Durchgehen** — inhärent sichere Kathodenchemie
- Gute Energiedichte für die meisten Einsätze (~160 Wh/kg)
- Sehr flache Entladekurve — Spannung bleibt über weiten SOC-Bereich stabil
- Tolerant gegenüber moderater Über-/Tiefentladung

**Schwächen:**
- **Kälteempfindlichste** aller unterstützten Chemien
- Flache Entladekurve macht spannungsbasierte SOC-Schätzung unzuverlässig (das Inhero MR2 löst dies mit einem Coulomb-Counter)
- Etwas geringere Energiedichte als Li-ion
- Darf unter 0 °C nicht geladen werden — Lithium-Plating-Risiko schädigt die Zelle permanent

**Inhero MR2 Besonderheiten:**
- Ladeschlussspannung: **3,5 V**
- **JEITA aktiv** — NTC erforderlich; Ladung gesperrt unter −2 °C (T-Cold)
- Frost-Ladestromreduzierung konfigurierbar über `set board.fmax`
- `set board.jeitaignore 1` schaltet die Temperaturüberwachung des Ladereglers komplett ab, begrenzt durch ein 0,05C-Gate — auf eigenes Risiko des Betreibers, siehe [Laden bei Kälte](#laden-bei-kälte)
- JEITA-WARM-Zone in Firmware neutralisiert zur Vermeidung von VBAT_OVP (siehe [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md#jeita-warm-zone--vbat_ovp-vermeidung))

### LTO (Lithium-Titanat, 2S)

Lithium-Titanat-Anoden-Chemie. Eingesetzt in Industrie und ÖPNV wegen extremer Langlebigkeit.

**Stärken:**
- **Beste Kälteperformance** aller unterstützten Chemien — 82% entnehmbar bei −20 °C
- **Extreme Zyklenlebensdauer** (10 000+ Zyklen, einige Hersteller geben 20 000+ an)
- Sehr sicher — kein Lithium-Plating, kein thermisches Durchgehen
- Breiter Betriebstemperaturbereich (−30 °C bis +60 °C Laden)
- **Kann bei Frost geladen werden** — keine JEITA-Einschränkung nötig
- Schnellladefähig (bis 5C oder mehr)
- Sehr flache Entladekurve

**Schwächen:**
- **Niedrige Energiedichte** (~80 Wh/kg) — braucht 2–3× das Volumen von Li-ion für gleiche Kapazität
- **2S-Konfiguration erfordert externen Balancer** — ohne Balancer driften Zellen über die Zeit und riskieren Über-/Tiefentladung einzelner Zellen
- Exotische Bauformen — typisch zylindrisch mit Schraubterminals oder Aluminiumgehäuse
- Schwer Punkt-schweißbar (Aluminiumgehäuse)
- Teuer (2–3× pro Wh vs. Li-ion)
- Eingeschränkte Verfügbarkeit — nur über Speziallieferanten

**Inhero MR2 Besonderheiten:**
- Ladeschlussspannung: **5,4 V** (2× 2,7 V pro Zelle)
- **Keine JEITA-Überwachung nötig** (`needs_jeita = false`) — die Temperaturüberwachung des Ladereglers ist für diese Chemie abgeschaltet, ein NTC ist zum Laden nicht erforderlich
- Ein NTC ist optional. Ein bestückter NTC wird wie bei jeder anderen Chemie ausgelesen und gemeldet; ohne NTC kommt die Temperatur für das SOC-Derating vom **BME280** auf dem Board
- `set board.fmax` hat keine Wirkung (zeigt „N/A"), und `set board.jeitaignore` wird abgelehnt mit `Err: This chemistry runs without JEITA (always 1)`
- Zellzahl im BQ25798 auf 2S konfiguriert

### Na-ion (Natrium-Ionen, 1S)

Natrium-Ionen-Technologie — die nachhaltige Alternative mit abundanten, nicht-kritischen Rohstoffen.

**Stärken:**
- **Gute Kälteperformance** — 78% entnehmbar bei −20 °C
- **Kein Kobalt, kein Lithium** — nachhaltig und ethisch beschaffte Materialien (Natrium, Eisen, Mangan)
- **Lagerung und Versand sind zellabhängig** — einige Zellen dürfen laut Datenblatt bei 0 V gelagert und versandt werden, andere verlangen ausdrücklich 30–50 % SOC (z. B. HNXN NaCR26700-ME35); maßgeblich ist das Datenblatt der konkreten Zelle
- **Kann bei Frost geladen werden, wenn die Zelle dafür freigegeben ist** — das Board sperrt nicht (keine JEITA-Einschränkung); die Grenze setzt das Zell-Datenblatt, siehe [Na-ion-Zellen](#na-ion-zellen)
- Gutes Sicherheitsprofil — kein thermisches Durchgehen unter normalen Bedingungen
- Sich schnell verbessernde Technologie — Energiedichte und Zyklenlebensdauer steigen mit jeder Generation

**Schwächen:**
- **Neue Technologie** — eingeschränkte Zellverfügbarkeit Stand 2025/2026
- Geringere Energiedichte als Li-ion (~130 Wh/kg, steigend)
- Weniger validierte Zelloptionen und öffentliche Datenblätter
- **Ladetemperaturfenster je nach Hersteller verschieden** — von 0 °C (viele Consumer- und NFPP-Zellen) bis −20 °C (HiNa, AuroraCell, mit Raten- und SOC-Grenzen); maßgeblich ist das Zell-Datenblatt, das Board setzt es nicht durch
- Zyklenlebensdauer noch unter LiFePO4 bei den meisten aktuellen Zellen (1000–3000 Zyklen)
- Markt reift noch — Qualitätsvariation zwischen Herstellern

**Inhero MR2 Besonderheiten:**
- Ladeschlussspannung: **3,9 V**
- **Keine JEITA-Überwachung** (`needs_jeita = false`) — die Temperaturüberwachung des Ladereglers ist für diese Chemie abgeschaltet, ein NTC ist zum Laden nicht erforderlich. Das Board setzt für Na-ion also keine untere Ladetemperatur durch; die Zelle muss laut Datenblatt die kälteste Ladetemperatur am Standort abdecken (siehe [Na-ion-Zellen](#na-ion-zellen))
- Ein NTC ist optional. Ein bestückter NTC wird wie bei jeder anderen Chemie ausgelesen und gemeldet; bei 3,1 V nominal liegen weite Teile der Entladung unter der 3200-mV-Grenze der Erfassung, im reinen Akkubetrieb meldet die Messung dort also `N/A` (siehe [Akkutemperatur-Erfassung](#akkutemperatur-erfassung)). Ohne NTC kommt die Temperatur für das SOC-Derating vom **BME280** auf dem Board
- `set board.fmax` hat keine Wirkung (zeigt „N/A"), und `set board.jeitaignore` wird abgelehnt mit `Err: This chemistry runs without JEITA (always 1)`

---

## 2. Vergleichstabelle

| | Li-ion 1S | LiFePO4 1S | LTO 2S | Na-ion 1S |
|---|---|---|---|---|
| **Energiedichte** | ~250 Wh/kg | ~160 Wh/kg | ~80 Wh/kg | ~130 Wh/kg |
| **Zyklenlebensdauer (bis 80%)** | 500–1000 | 2000–5000 | 10 000+ | 1000–3000 |
| **Kälteperformance** | Mäßig | Schwächste | Beste | Gut |
| **Entnehmbar bei −20 °C** | 55% | 46% | 82% | 78% |
| **Entnehmbar bei −10 °C** | 65% | 58% | 86% | 83% |
| **Entnehmbar bei 0 °C** | 75% | 70% | 90% | 88% |
| **Therm. Durchgehen** | Ja (Risiko) | Nein | Nein | Nein |
| **NTC erforderlich?** | Ja¹ | Ja¹ | Nein (optional) | Nein (optional) |
| **Akkutemperatur gemeldet?** | Mit NTC | Mit NTC² | Mit NTC | Mit NTC² |
| **JEITA** | Aktiv | Aktiv | Nicht nötig (`needs_jeita = false`) | Aus (`needs_jeita = false`) |
| **Laden bei Frost?** | Nein (gesperrt <−2 °C)¹ | Nein (gesperrt <−2 °C)¹ | Ja | Zellabhängig³ |
| **Ladeschlussspannung** | 4,1 V | 3,5 V | 5,4 V (2S) | 3,9 V |
| **Low-V Sleep** | 3100 mV | 2700 mV | 3900 mV | 2500 mV |
| **Low-V Wake** | 3300 mV | 2900 mV | 4100 mV | 2700 mV |
| **Nennspannung** | 3,7 V | 3,2 V | 4,6 V | 3,1 V |
| **Zellformate** | 18650, 21700, Pouch | 18650, 26650, prismat. | Schraubterminal, Alu | 18650, prismatisch |
| **Verfügbarkeit** | Sehr gut | Gut | Eingeschränkt | Eingeschränkt |
| **Relativer Preis (pro Wh)** | Niedrig | Niedrig–Mittel | Hoch | Mittel |

¹ `set board.jeitaignore 1` hebt für Li-ion und LiFePO4 sowohl die Kältesperre als auch die NTC-Pflicht auf, solange das 0,05C-Gate hält. Damit entfällt auch die Ladeabschaltung auf der heißen Seite — siehe [Laden bei Kälte](#laden-bei-kälte).

² Ein bestückter NTC wird bei jeder Chemie ausgelesen. Im reinen Akkubetrieb ist der TS-Kanal unter 3200 mV abgeschaltet, daher melden LiFePO4 und Na-ion über weite Teile ihrer Entladung `N/A` — siehe [Akkutemperatur-Erfassung](#akkutemperatur-erfassung).

³ Das Board sperrt das Laden bei Na-ion nicht. Die zulässige Ladetemperatur steht im Zell-Datenblatt — je nach Hersteller 0 °C, −10 °C oder −20 °C. Siehe [Na-ion-Zellen](#na-ion-zellen).

> **Hinweis zu den Entnehmbar-Werten:** Das sind typische Datenblattwerte bei 0,2C–0,5C-Entladelast. Die Last des MR2 liegt weit unter 0,05C und ist damit deutlich sanfter; das Derating-Modell der Firmware (Abschnitt 3) zeigt daher in `get board.telem` höhere entnehmbare Werte.

---

## 3. Temperaturverhalten

### Kälteperformance-Ranking

Von bester zu schlechtester Kälteperformance:

1. **LTO** — 82% entnehmbar bei −20 °C, lädt bei Frost
2. **Na-ion** — 78% entnehmbar bei −20 °C; Laden bei Frost nur im Fenster des Zell-Datenblatts (je nach Zelle 0 °C bis −20 °C)
3. **Li-ion** — 55% entnehmbar bei −20 °C, Laden bei Frost gesperrt
4. **LiFePO4** — 46% entnehmbar bei −20 °C, Laden bei Frost gesperrt

*(Datenblattwerte bei 0,2C–0,5C-Last — siehe Hinweis in Abschnitt 2; unter der deutlich sanfteren Last des MR2 zeigt das Derating-Modell der Firmware weiter unten höhere Werte.)*

> Das Ranking überrascht möglicherweise Nutzer, die LiFePO4 als bewährtes „Arbeitstier" kennen. Während LiFePO4 bei Zyklenlebensdauer und Sicherheit glänzt, ist es tatsächlich die **schlechteste Chemie bei Kälte** unter den vier unterstützten. Das ist besonders relevant für alpine Einsätze und Winterbetrieb.

### Laden bei Kälte

| Chemie | Laden bei Frost? | Mechanismus |
|---|---|---|
| **Li-ion** | Nein — gesperrt unter −2 °C, außer `board.jeitaignore` ist scharf | JEITA T-Cold (Hardware, BQ25798) |
| **LiFePO4** | Nein — gesperrt unter −2 °C, außer `board.jeitaignore` ist scharf | JEITA T-Cold (Hardware, BQ25798) |
| **LTO** | Ja — lädt bei jeder Temperatur | Keine JEITA-Überwachung (`needs_jeita = false`) |
| **Na-ion** | Vom Board nicht gesperrt — die Grenze setzt das Zell-Datenblatt (je nach Hersteller 0 °C, −10 °C oder −20 °C) | Keine JEITA-Überwachung (`needs_jeita = false`); das Temperaturfenster ist die Zellwahl des Betreibers |

Für Li-ion und LiFePO4 ist die Ladung im **T-Cool-Bereich** (+3 °C bis −2 °C mit Inhero-Spannungsteiler) standardmäßig gesperrt und kann über `set board.fmax` (20%, 40% oder 100%) auf eine reduzierte Rate gesetzt werden. `fmax` wirkt nur in diesem Band — unterhalb von −2 °C (T-Cold) bleibt die Hardware-Sperre unabhängig von `fmax` bestehen, und nur `board.jeitaignore` hebt sie auf. Beachte: Die Auswahl von Li-ion oder LiFePO4 über `set board.bat` setzt `board.fmax` auf 0% zurück. [FAQ #6](FAQ.md#6-was-ist-frostladen-und-wie-wirken-fmax-und-jeitaignore-zusammen) stellt beide Einstellungen nebeneinander.

**Warum ist Frostladen bei Li-ion und LiFePO4 gefährlich?** Bei niedrigen Temperaturen können Lithium-Ionen nicht ordnungsgemäß in die Graphit-Anode interkalieren. Stattdessen lagern sie sich als metallisches Lithium auf der Anodenoberfläche ab („Lithium-Plating"). Das reduziert die Kapazität permanent und kann interne Kurzschlüsse erzeugen — ein Sicherheitsrisiko.

LTO verwendet eine Lithium-Titanat-Anode, die bei etwa 1,55 V arbeitet — weit oberhalb des Lithium-Plating-Potentials —, deshalb ist Frostladen dort sicher. Na-ion verwendet eine Hard-Carbon-Anode, deren Ladeplateau nur rund 0,1 V vom Natrium-Plating entfernt liegt; ob eine Zelle das Laden unter 0 °C verträgt, hängt von Elektrolyt und Zellauslegung ab, weshalb die Hersteller alles von 0 °C bis −20 °C angeben. Das Board überwacht das bei Na-ion nicht — maßgeblich ist das Zell-Datenblatt.

> **Felderfahrung vs. Theorie:** Viele Repeater-Betreiber laden Li-ion-Zellen bei Frost mit geringen Solarströmen und berichten über mehrere Winter hinweg von keiner messbaren Degradation. Die [YYCMesh-Community](https://yycmesh.com/blog/cold-weather-charging) dokumentierte zwei Jahre alpine Einsätze in den kanadischen Rockies (bis −40 °C) mit gewöhnlichen, ungeschützten 18650-Zellen (3000–3500 mAh) — die Innenwiderstände lagen weiterhin im Werksspezifikationsbereich. Ihr eigenes Arbeitsfenster beschreiben sie mit „most of our systems charge at < 0.1 C, often well below 0.05 C", gespeist aus 1-W- bis 6-W-Panels mit durchschnittlichen Ladeströmen typisch unter 200 mA und gelegentlichen Spitzen um 300 mA. Dieses Fenster **schließt** 0,05C ein — 200 mA in eine 3,5-Ah-Zelle sind 0,057C. Weitere Faktoren dort: passive Sonnenerwärmung der Gehäuse und Ladung während der wärmsten Tageszeit.
>
> Die Grenzen ihrer eigenen Belege benennen sie ebenfalls: Die theoretisch „sichere" Kälte-Laderate liegt bei etwa 0,02C, die gezeigte degradierte Zelle ist eine einzelne Vape-Zelle, gemessen gegen einen generischen Neuwert ohne Vormessung und ohne Kontrollzelle, und ein zuverlässiger Dendriten-Nachweis bräuchte NMR-Spektrometrie oder Bildgebung, „far outside the reach of the average hobbyist". Diese 0,02C sind die theoretische Zahl der Quelle selbst, während das Arbeitsfenster, das sie tatsächlich berichten, 0,05C einschließt. Einordnung: ehrliche Praxiserfahrung von Betreibern, die selbst sagen, dass sie den Mechanismus nicht messen können. Das ist weder ein Beweis, dass Frostladen harmlos ist, noch ein Grund, die Praxis abzutun. *Es kommt darauf an* — auf Laderate, Zellqualität, Panelgröße und Temperatur. Die Degradation durch Lithium-Plating ist **kumulativ und permanent** — sie zeigt sich als still verschwundene Kapazität über Jahre. Zwei zusätzliche Risiken werden häufig unterschätzt:
>
> 1. **PV-Panels liefern bei Kälte mehr Leistung** (Silizium-Temperaturkoeffizient ~−0,35%/°C). Ein 5-W-Panel bei −10 °C liefert deutlich mehr Strom als bei +25 °C. Schneereflexion kann die Leistung sogar über die Nennwerte hinaus steigern.
> 2. **Zellqualität variiert.** Ergebnisse mit Premium-Zellen (niedriger Innenwiderstand, konsistente Chemie) lassen sich nicht unbedingt auf Budget-Zellen übertragen.
>
> Das Inhero MR2 wird konservativ ausgeliefert: Der NTC-Akkutemperatursensor veranlasst den BQ25798, die Ladung zu blockieren, bis die Zelle sich über −2 °C erwärmt hat (JEITA T-Cold), und hält die Ladung standardmäßig auch in der T-Cool-Zone gesperrt (`board.fmax`-Standard 0%). Mit `set board.fmax` 20%, 40% oder 100% lädt das Board zwischen −2 °C und +3 °C stattdessen mit reduzierter Rate. An sonnigen Wintertagen läuft das Board über den Power-Path auf Solar, während der Akku geschützt bleibt. Sobald die direkte Sonneneinstrahlung das Gehäuse und die Akkutemperatur über den Schwellwert erwärmt — was bei durchdachtem Gehäusedesign erstaunlich schnell geht — wird die Ladung automatisch wieder freigegeben.

**Die Temperaturüberwachung übergehen: `set board.jeitaignore`**

Für Betreiber, die die oben beschriebene Praxis fahren wollen, bietet die Firmware sie als ausdrückliche, begrenzte Option an, die **standardmäßig aus** ist. `set board.jeitaignore 1` setzt das TS_IGNORE-Bit im BQ25798; der Laderegler behandelt den TS-Pin dann als immer in Ordnung und lädt durch den Frost hindurch. Das Kommando wird nur für Li-ion und LiFePO4 angenommen — LTO und Na-ion laufen ohnehin ohne JEITA und antworten `Err: This chemistry runs without JEITA (always 1)`.

**Das Gate.** Der Override wird nur wirksam, solange `board.batcap` ausdrücklich gesetzt wurde **und** `board.imax` bei höchstens **0,05C** dieser Kapazität liegt (9000 mAh → 450 mA). Außerhalb davon wird der Wunsch gespeichert, bleibt aber inaktiv, und die CLI benennt den Blocker: `jeitaignore set to 1, N/A, C>0.05` oder `jeitaignore set to 1, N/A, batcap not set`. Verworfen wird dabei nichts — `imax` wieder unter die Grenze zu senken oder `batcap` anzuheben schärft den Override von selbst wieder, und die Antwort sagt das mit `; jeitaignore 1`. Persistent ist der Wunsch; der wirksame Zustand wird bei jedem Boot und bei jedem Chemiewechsel neu abgeleitet, und vom Einschalten bis zum Anwenden der gespeicherten Konfiguration hat die Hardware-Überwachung das Sagen.

**Was man aufgibt.** TS_IGNORE nimmt den TS-Pin aus jeder Ladeentscheidung heraus, die heiße eingeschlossen. Solange der Override an ist, unterbricht der Laderegler die Ladung auch auf der heißen Seite nicht mehr (T-Hot, rund +58 °C am Spannungsteiler des MR2), und alle vier TS-Statusbits melden „kein Fehler". Für keine der beiden Grenzen gibt es einen Software-Ersatz: Das Board schläft im SYSTEMOFF mit aktivem Laderegler, dort läuft keine Regelschleife — die 0,05C-Schranke ist das gesamte Sicherheitsargument. Auf der heißen Seite hält genau diese Schranke das Restrisiko klein, weil 0,05C thermisch uninteressant ist. Auf der kalten Seite bleibt der Plating-Mechanismus unverändert; das Gate begrenzt die Rate, es beseitigt den Mechanismus nicht. Wer das Flag setzt, akzeptiert beschleunigte Zellalterung als Preis für Ladezeit im Winter.

**Wofür es ausgelegt ist.** Das Gate ist eine einzige Zahl für alle Temperaturen, während die vertretbare Laderate mit sinkender Zelltemperatur fällt — etwa um die Hälfte je 10 K. Knapp unter dem Gefrierpunkt ist der Abstand bei 0,05C komfortabel; er schrumpft mit jedem Grad, das der Standort tiefer liegt. Der Override ist auf mitteleuropäischen Frost ausgelegt, die Bedingungen, für die das [Datenblatt](DATASHEET.md) Modul- und Akkugröße bemisst. Für Standorte regelmäßig unter etwa −20 °C wählt man besser die passende Chemie: LTO ist von seinen Herstellern für das Laden bei −20 °C und darunter freigegeben; bei Na-ion sind es nur manche Zellen (HiNa und AuroraCell geben −20 °C mit reduzierter Rate, Spannung und SOC an, viele Consumer-Zellen enden bei −10 °C oder 0 °C) — Zell-Datenblatt prüfen. Das MR2 unterstützt beide Chemien.

**Solange er an ist.** `get board.fmax` antwortet `N/A`, `set board.fmax` wird mit `Err: Fmax N/A while jeitaignore is on` abgelehnt, und `get board.conf` hängt ` J:1` an. `set board.jeitaignore 0` schaltet den Override ab und stellt das gespeicherte `fmax`-Verhalten wieder her.

### Akkutemperatur-Erfassung

Die Akkutemperatur kommt von einem NTC am TS-Pin des BQ25798. Die Firmware liest diesen Kanal bei **jeder Chemie**; ob ein NTC bestückt ist, ist eine Frage der Verdrahtung und hängt nicht mehr an der eingestellten Chemie. Zwei Bedingungen entscheiden, ob überhaupt ein Wert verfügbar ist:

- **Akkuspannung.** Im reinen Akkubetrieb wird der TS-Kanal abgeschaltet, solange die Akkuspannung zwischen 0 und 3200 mV liegt, damit die Solarmessung weiter funktioniert. Liegt eine Eingangsquelle an — das Panel liefert Leistung —, bleibt der Kanal eingeschaltet, sodass eine kalte, fast leere Zelle beim Laden ihre Temperatur meldet. LTO 2S (4,6–5,4 V) liegt immer darüber und Li-ion 1S über den größten Teil seiner Kurve, während LiFePO4 (3,2 V nominal) und Na-ion (3,1 V nominal) weite Teile ihrer Entladung darunter verbringen — im reinen Akkubetrieb meldet die Akkutemperatur dort `N/A`, auch mit bestücktem NTC.
- **Plausibilität.** Ein dekodierter Wert wird gegen eine frische BME280-Messung geprüft; liegt er mehr als 15,0 °C daneben, wird er verworfen und als `N/A` gemeldet. Diese Prüfung gibt es, weil ein fehlender oder offener NTC keinen offensichtlich falschen Wert liefert — der Spannungsteiler dekodiert dann zu etwa −46 °C, was die reine Bereichsprüfung unbemerkt passiert.

Wurde 5 Minuten lang keine NTC-Messung akzeptiert, greift für das SOC-Derating die Temperatur des BME280 auf dem Board. Dieser Fallback gilt für jede Chemie — ob der NTC fehlt, die Akkuspannung zu niedrig ist oder die Messung verworfen wurde.

**Erforderlich zum Laden** ist ein NTC nur bei Li-ion und LiFePO4: Dort liest der BQ25798 einen offenen TS-Pin als Frost und sperrt die Ladung. Einen NTC zu bestücken (Lötbrücke oder externer Sensor) ist dafür die richtige Lösung. Für eine Installation ohne NTC bei diesen beiden Chemien ist `set board.jeitaignore 1` innerhalb des 0,05C-Gates die Alternative — damit fällt der TS-Pin ganz aus der Ladeentscheidung heraus, mit dem oben beschriebenen Preis.

### Temperatur-Derating

Die Firmware verwendet ein **Trapped-Charge-Modell**, um die **entnehmbare** Kapazität bei der aktuellen Akkutemperatur abzuschätzen: Kälte sperrt den Boden der Entladekurve — die Zelle erreicht ihre Abschaltspannung, während Ladung noch gespeichert ist. SOC% selbst ist rein Coulomb-basiert (gespeicherte Ladung) und temperaturunabhängig. Das Derating wird angewendet auf:
- **Batt-TTL-Berechnung** — Trapped Charge: entnehmbar = max(0, gespeichert − Kapazität × (1−f(T)))
- **CLI-Anzeige** — `get board.telem` zeigt den derateten Wert in Klammern: `SOC:95.0% (78%)`

Das Modell verwendet eine chemiespezifische lineare Funktion für den Derating-Faktor:

```
f(T) = max(f_min, 1,0 - k × (T_ref - T))    für T < T_ref
f(T) = 1,0                                    für T >= T_ref
```

T_ref = 25 °C für alle Chemien (kein Derating bei Raumtemperatur und darüber).

| Chemie | k (/°C) | f_min | Bei −20 °C | Bei −10 °C | Bei 0 °C | Bei 10 °C |
|--------|---------|-------|-----------|-----------|---------|---------|
| Li-ion | 0,005 | 0,75 | 0,78 | 0,83 | 0,88 | 0,93 |
| LiFePO4 | 0,006 | 0,70 | 0,73 | 0,79 | 0,85 | 0,91 |
| Na-ion | 0,003 | 0,85 | 0,87 | 0,90 | 0,93 | 0,96 |
| LTO | 0,002 | 0,88 | 0,91 | 0,93 | 0,95 | 0,97 |

**Praktische Effekte:**
- SOC% ändert sich nur durch reale Ladungsflüsse — nicht durch Temperatur
- Batt-TTL sinkt bei Kälte, besonders bei niedrigem SOC (Trapped-Charge-Modell sperrt den Boden)
- `get board.telem` zeigt beide Werte: `SOC:95.0% (78%)` = gespeichert (entnehmbar)

→ Siehe [FAQ #13 — Wie funktioniert das Temperatur-Derating?](FAQ.md#13-wie-funktioniert-das-temperatur-derating) für weitere technische Details.

---

## 4. Zellauswahl & Bauformen

### Li-ion-Zellen

| Bauform | Typische Kapazität | Hinweise |
|---|---|---|
| **18650** | 2500–3500 mAh | Am verbreitetsten; leicht zu beschaffen |
| **21700** | 4000–5000 mAh | Höhere Kapazität; wird zum neuen Standard |
| **Pouch** | Variiert | Flexible Formen; erfordert sorgfältige Montage |

**Tipps:**
- Zellen mit eingebauter Schutzschaltung (PCM) für Einzelzellbetrieb bevorzugen
- Bei Parallelpacks: Zellen matchen (gleicher Hersteller, gleiche Charge)
- Zellen mit integriertem NTC vereinfachen die Verdrahtung zum TS-Pin
- Empfohlen: Samsung, Sony/Murata, LG, Panasonic/Sanyo — No-Name-Zellen vermeiden

### LiFePO4-Zellen

| Bauform | Typische Kapazität | Hinweise |
|---|---|---|
| **18650** | 1400–1800 mAh | Geringere Kapazität als Li-ion 18650; weniger verbreitet |
| **26650** | 2500–3600 mAh | Größerer Durchmesser; beliebt für LFP |
| **32650** | 5000–6000 mAh | Große Rundzelle; gut für Hochkapazitätspacks |
| **Prismatisch** | 5000–50 000 mAh | Flachzellen; effiziente Raumnutzung |

**Tipps:**
- Die flache Entladekurve sagt wenig über den SOC aus — auf den Coulomb-Counter des Inhero MR2 verlassen
- Laden unter 0 °C vermeiden — der JEITA-Schutz des Boards erledigt das automatisch
- EVE, BYD, CATL sind etablierte Hersteller

### LTO-Zellen

| Bauform | Typische Kapazität | Hinweise |
|---|---|---|
| **Zylindrisch (Schraubterminal)** | 10 000–40 000 mAh | Gängigstes LTO-Format; M6/M8-Schraubterminals |
| **Prismatisch (Aluminium)** | 10 000–30 000 mAh | Aluminiumgehäuse; schwer punkt-schweißbar |

**⚠️ Wichtig: 2S erfordert einen Balancer**

Das Inhero MR2 konfiguriert den BQ25798 für 2S-Betrieb, bietet aber **kein eingebautes Zellbalancing**. Ein externes Balancer-Modul ist erforderlich, um Zellspannungsdrift über die Zeit zu verhindern. Ohne Balancer kann eine Zelle überladen werden, während die andere unterladen wird — das verschlechtert die Kapazität und kann Zellen beschädigen.

**Tipps:**
- Passiven oder aktiven Balancer verwenden, ausgelegt für den Zellspannungsbereich (2,0–2,7 V pro Zelle)
- Yinlong/Toshiba SCiB sind gängige LTO-Zellmarken
- 2–3× Volumen und Gewicht im Vergleich zu Li-ion für gleiche Energie einplanen
- Schraubterminals sind robust für Outdoor-Einsätze — kein Punktschweißen nötig

### Na-ion-Zellen

| Bauform | Typische Kapazität | Hinweise |
|---|---|---|
| **18650** | 1000–1500 mAh | Aufkommend; Zellen der ersten Generation |
| **26700** | 3000–3500 mAh | Schichtoxid-Consumer-Zellen (z. B. HAKADI) |
| **32140 / 33140** | 8000–10 000 mAh | Industrielle Rundzellen (HiNa, AuroraCell) |
| **Prismatisch** | 5000–20 000 mAh | Größere Formate erscheinen von HiNa, CATL, Faradion |

**Vor dem Kauf das Ladetemperaturfenster im Datenblatt prüfen.** Na-ion-Zellen unterscheiden sich: HiNa NaCR32140 lädt bis −20 °C (0,1C, 3,8 V, max. 75 % SOC unter −10 °C), AuroraCell 32140 bis −20 °C, HAKADI 26700 und viele 18650-Zellen bis −10 °C, NFPP-Zellen und etliche Budget-Zellen erst ab 0 °C. Das MR2 fährt Na-ion ohne Temperaturüberwachung, das Fenster der Zelle ist also die einzige wirksame Grenze — eine Zelle wählen, deren Fenster die kälteste Ladetemperatur am Standort abdeckt. Hintergrund: [CUK-SIB zu Kälteanwendungen](https://www.cuk-sib.com/de/blog/sodium-ion-batteries-for-cold-climate-applications).

**Tipps:**
- Technologie entwickelt sich schnell — vor dem Kauf aktuelle verfügbare Zellen prüfen
- Lager- und Versandzustand laut Zell-Datenblatt: einige Zellen erlauben 0 V, andere verlangen 30–50 % SOC (z. B. HNXN NaCR26700-ME35)
- HiNa, CATL, Faradion/Reliance sind zentrale Hersteller (Stand 2025/2026)

---

## 5. Kapazitätsplanung

### Stromverbrauch

Typischer Inhero MR2 Stromverbrauch (Repeater-Modus, LEDs aus):

| Zustand | Stromaufnahme | Hinweise |
|---|---|---|
| Idle (RX, kein TX) | ~7,6 mA @ 3,3 V | USB aus, SX1262 im RX |
| **Gemessen typisch** | **~12,3 mA @ 3,3 V** | 24h-Messung, Repeater mit typischem Traffic |
| **Worst Case (10% DC, EU868 g3)** | **~19,8 mA @ 3,3 V** | 10% Duty Cycle × ~130 mA TX + 90% × 7,6 mA Idle |
| TX-Burst (SX1262 +22 dBm) | ~130 mA @ 3,3 V | Nur kurze Bursts, reguliert durch Duty Cycle |
| Low-Voltage-Sleep | <0,5 mA | Solarladung läuft weiter |

> **Wie diese Werte ermittelt werden:** Der Worst Case setzt das EU868-**g3-Subband** (869,4–869,65 MHz) voraus, das bis zu +27 dBm ERP und einen **10% Duty Cycle** erlaubt — MeshCores derzeitiger Vorgabekanal 869,618 MHz liegt darin. Die Frequenz ist eine Firmware-Einstellung; in einem Band mit engerem Duty Cycle fällt der Worst Case entsprechend niedriger aus. Bei +22 dBm ziehen SX1262 + MCU ~130 mA während TX. Bei 10% TX-Anteil: `0,90 × 7,6 + 0,10 × 130 = 19,8 mA` → **~65 mW bzw. ~1,57 Wh/Tag** — das regulatorische Maximum.
>
> **Gemessen typisch (~12,3 mA):** Validierte 24h-Messung im Repeater-Betrieb mit typischem Traffic: **295 mAh/Tag @ 3,32 V** = 0,98 Wh/Tag → **~41 mW bzw. ~0,98 Wh/Tag**.

### Warum der Strom von der Akkuspannung abhängt

Das Inhero MR2 verwendet einen hocheffizienten **Buck-Converter**, um die 3,3-V-Schiene für MCU und Funk zu erzeugen. Dadurch zieht das Board eine annähernd **konstante Leistung** (Watt), keinen konstanten Strom (Ampere).

Weil Leistung = Spannung × Strom, bedeutet höhere Akkuspannung weniger Strom aus dem Akku — aber die Leistung bleibt gleich:

| Chemie | Nennspannung | Idle | Gemessen typisch | Worst Case (10% DC) |
|---|---|---|---|---|
| Na-ion | 3,1 V | ~8,1 mA (25 mW) | ~13,2 mA (41 mW) | ~21,0 mA (65 mW) |
| LiFePO4 | 3,2 V | ~7,8 mA (25 mW) | ~12,8 mA (41 mW) | ~20,3 mA (65 mW) |
| Li-ion | 3,7 V | ~6,8 mA (25 mW) | ~11,1 mA (41 mW) | ~17,6 mA (65 mW) |
| LTO (2S) | 4,6 V | ~5,4 mA (25 mW) | ~8,9 mA (41 mW) | ~14,1 mA (65 mW) |

> **Wichtig für die Kapazitätsplanung:** Nicht einfach mA × Stunden rechnen — das funktioniert nur innerhalb einer Chemie bei einer Spannung. Beim Vergleich verschiedener Chemien immer in **Wh** (Energie) rechnen: `Energie (Wh) = Wh/Tag × Tage`. Dann auf die eigene Chemie umrechnen: `mAh = Wh × 1000 ÷ V_nominal`. **0,98 Wh/Tag** (gemessen typisch) oder **1,57 Wh/Tag** (Worst Case 10% DC) je nach erwartetem Traffic und Sicherheitsreserve verwenden.
>
> **Beispiel:** 30 Tage Autonomie bei gemessen typischer Last = 29 Wh benötigt (Worst Case: 47 Wh).
> - LiFePO4 (3,2 V): 29 000 ÷ 3,2 = **9 063 mAh** (Worst Case: 14 688 mAh)
> - LTO 2S (4,6 V): 29 000 ÷ 4,6 = **6 304 mAh** (Worst Case: 10 217 mAh)

### Dimensionierung für Autonomie

**Energieansatz (empfohlen):** `Energie (Wh) = Wh/Tag × gewünschte Tage Autonomie` — **0,98 Wh/Tag** (gemessen typisch) oder **1,57 Wh/Tag** (Worst Case 10% DC) für konservative Auslegung

**Umrechnung in mAh für die eigene Chemie:** `mAh = Wh × 1000 ÷ V_nominal`

Die mAh-Spalte unten ist bei 3,3 V berechnet (≈ LiFePO4- / Na-ion-Nennspannung). Für Li-ion oder LTO die Energie-Spalte mit obiger Formel verwenden — siehe [Warum der Strom von der Akkuspannung abhängt](#warum-der-strom-von-der-akkuspannung-abhängt).

| Gewünschte Autonomie | Gemessen typisch (0,98 Wh/Tag) | Worst Case (1,57 Wh/Tag) | mAh @ 3,3 V (typisch) | Empfohlen |
|---|---|---|---|---|
| **3 Tage** (Indoor, Netz-Backup) | 2,9 Wh | 4,7 Wh | 891 mAh | 1500 mAh |
| **7 Tage** (Solar, Sommer) | 6,9 Wh | 11,0 Wh | 2079 mAh | 3500 mAh |
| **14 Tage** (Solar, Winter) | 13,7 Wh | 22,0 Wh | 4158 mAh | 6000 mAh |
| **30 Tage** (Alpin, minimale Solar) | 29,4 Wh | 47,1 Wh | 8909 mAh | 12 000+ mAh |

> **Kälte-Reserve:** Für Einsätze unter 0 °C die Kapazität um den Kehrwert des Derating-Faktors erhöhen. Beispiel: LiFePO4 bei −10 °C hat f(T) = 0,79, daher braucht man `Kapazität / 0,79 ≈ 1,27×` die Kapazität im Vergleich zu Raumtemperatur. Die Batt-TTL-Berechnung wendet dieses Derating automatisch an.

### Die 90%-Regel

`board.batcap` auf **90% der Nennkapazität** setzen. Das Inhero MR2 verwendet konservative Ladeschlussspannungen (z. B. 4,1 V statt 4,2 V für Li-ion), wodurch die oberen ~10% der Nennkapazität bewusst nicht genutzt werden — das verbessert die Zyklenlebensdauer erheblich.

**Beispiel:** 10 000 mAh Nenn → `set board.batcap 9000`

→ Siehe [FAQ #4](FAQ.md#4-welchen-mah-wert-gebe-ich-bei-set-boardbatcap-ein)

---

## 6. Solar-Ladebetrachtungen

**Formel für maximalen Ladestrom:** `I_Ladung (mA) = Panel-Leistung (W) / Nennspannung Akku (V)`

| Chemie | Panel | Ladestrom | `set board.imax` |
|---|---|---|---|
| Li-ion (3,7 V) | 2 W | 540 mA | `set board.imax 540` |
| LiFePO4 (3,2 V) | 1 W | 310 mA | `set board.imax 310` |
| LTO (4,6 V) | 5 W | 1090 mA | `set board.imax 1090` |
| Na-ion (3,1 V) | 3 W | 970 mA | `set board.imax 970` |

**Richtlinien zur Panel-Dimensionierung:**
- Das Inhero MR2 verbraucht ~7,6 mA @ 3,3 V im Idle, **gemessen ~12,3 mA typisch** (~0,98 Wh/Tag), Worst Case ~19,8 mA bei vollem EU868-g3-10%-Duty-Cycle (~1,57 Wh/Tag)
- Bei vertikaler Südausrichtung (siehe unten) sind **2 W Monokristallin sicher ausreichend** für Winterautarkie in Mitteleuropa mit 9 Ah LiFePO4-Akku. Praxistest: Selbst ein 1-W-Panel (vertikal, Süd, unverschattet, exponiert) mit 9 Ah LiFePO4 hat einen vollen mitteleuropäischen Winter überstanden
- Bei Standorten mit häufiger Bewölkung oder Teilverschattung mehr Reserve einplanen — 3–5 W empfohlen
- MPPT ist essenziell für maximale Leistungsausbeute — aktivieren mit `set board.mppt 1`

**Panel-Ausrichtung — vertikal ist besser für Autarkbetrieb:**

Konventionelle PV-Anlagen neigen Panels auf ~30–40°, um den Jahresertrag zu maximieren. Bei autarken Repeatern ist das Ziel ein anderes: **Winter-Performance maximieren**, besonders in den kritischen Monaten Dezember und Januar, wenn die Sonne am niedrigsten steht und die Tage am kürzesten sind. **Vertikale Montage (90°)** hat erhebliche Vorteile:

- **Niedrige Wintersonne** trifft ein vertikales Panel im nahezu optimalen Winkel, während ein 30°-geneigtes Panel das gleiche Licht nur streifend empfängt
- **Selbstreinigung:** Vertikale Panels lassen Schnee, Eis und Schmutz deutlich besser abrutschen — ein eingeschneites Panel liefert null Leistung, egal welche Nennleistung draufsteht
- **Praxis-Faustregel (Mitteleuropa):** Im Januar kann man mit ca. **1 Wh/Tag pro 1 Wp** Panelleistung rechnen bei vertikaler, südausgerichteter, unverschatteter, exponierter Montage. PVGIS-Daten bestätigen ~36 Wh/Monat (nach Systemverlusten) für ein 1-Wp-Panel in dieser Konfiguration. In der Praxis kann das MR2 an sehr trüben Tagen nicht ernten, wenn der Charger !PG (Power Not Good) meldet — konservativ gerechnet sind **~30 Wh/Monat bzw. ~1 Wh/Tag** nutzbar. Bei ~0,98 Wh/Tag gemessen typischem Verbrauch ergibt ein 1-Wp-Panel eine positive Energiebilanz im Januar. **2 Wp bieten komfortablen Spielraum** für Trübwetterperioden und Standorte mit mehr Traffic
- Im Sommer produzieren vertikale Panels weniger als optimal geneigte — aber der Sommerertrag ist nie der Engpass für Autarkie

**Chemiespezifische Aspekte:**
- **Li-ion / LiFePO4:** Solarladung ist bei Frost (<−2 °C) gesperrt, sofern nicht `set board.jeitaignore 1` scharf ist — auf eigenes Risiko des Betreibers. An kalten Wintertagen kann das Panel Strom liefern, aber der Akku nimmt keine Ladung an, bis er sich über −2 °C erwärmt. Das Board läuft derweil direkt auf Solar, wenn die Leistung ausreicht. Beachte: PV-Panels liefern **bei Kälte mehr Leistung** (Silizium-Temperaturkoeffizient ~−0,35%/°C) — tatsächliche Ladeströme können die Nennwerte überschreiten. Deshalb sitzt die Sperre in der Hardware, und deshalb hängt das Gate des `jeitaignore`-Overrides am eingestellten Ladestrom — siehe [Laden bei Kälte](#laden-bei-kälte).
- **LTO:** Solarladung funktioniert auch bei tiefem Frost — ein erheblicher Vorteil für alpine Einsätze, wo Frost tage- oder wochenlang anhalten kann.
- **Na-ion:** Das Board sperrt das Laden bei Frost nicht; ob die Zelle dort geladen werden darf, steht in ihrem Datenblatt (je nach Zelle 0 °C bis −20 °C). Mit einer für −20 °C freigegebenen Zelle gilt der alpine Vorteil, mit einer 0-°C-Zelle nicht — siehe [Na-ion-Zellen](#na-ion-zellen).

---

## 7. Sicherheit & Schutz

| Chemie | Therm. Durchgehen | BMS/Schutz erforderlich? | Inhero MR2 Schutz |
|---|---|---|---|
| **Li-ion** | ⚠️ Ja — Brand-/Explosionsgefahr bei Missbrauch | Ja — zwingend | JEITA, Low-V Sleep, OVP, Ladespannungslimit |
| **LiFePO4** | ✅ Nein — inhärent sicher | Empfohlen | JEITA, Low-V Sleep, OVP |
| **LTO** | ✅ Nein — inhärent sicher | Empfohlen (Balancer!) | Low-V Sleep, OVP, Zellzahl-Konfig |
| **Na-ion** | ✅ Nein unter Normalbedingungen | Empfohlen | Low-V Sleep, OVP |

**Eingebaute Sicherheitsfeatures des Inhero MR2 (alle Chemien):**
- **Low-Voltage-Sleep** — INA228 ALERT ISR löst System-Sleep aus, um Tiefentladung zu verhindern
- **Ladespannungslimit** — BQ25798 pro Chemie konfiguriert, gegen Überladung
- **VBAT_OVP** — Hardware-Überspannungsschutz im BQ25798
- **200 mV Hysterese** — Verhindert Motorboating (schnelles Ein/Aus-Toggeln) nahe leer
- **JEITA-Temperaturschutz** (nur Li-ion/LiFePO4) — Hardware-Ladekontrolle über NTC; `set board.jeitaignore 1` schaltet ihn innerhalb des 0,05C-Gates ab, auf eigenes Risiko des Betreibers ([Details](#laden-bei-kälte))

**Nutzerverantwortung:**
- Li-ion: Zellen mit Schutzschaltung (PCM) oder ordentlichem BMS verwenden
- LTO 2S: **Externer Balancer ist Pflicht** für Langzeitbetrieb
- Alle Chemien: Korrekte Chemie über `set board.bat` setzen — falsche Chemie = falsche Spannungen = Schadensrisiko

---

## 8. Einsatzempfehlungen

| Szenario | Empfohlen | Alternative | Hinweise |
|---|---|---|---|
| **Indoor, gemäßigtes Klima (0–40 °C)** | **LiFePO4** | Li-ion | LiFePO4: bestes Verhältnis Sicherheit + Zyklen |
| **Outdoor, gemäßigt (−5 bis +35 °C)** | **LiFePO4** | Li-ion | Frost selten; fmax behandelt gelegentliche Kälte |
| **Platzbeschränktes Gehäuse** | **Li-ion** | — | Höchste Energiedichte; nichts anderes passt |
| **Alpin, extreme Kälte (−20 °C und darunter)** | **LTO** | Na-ion (Zelle für −20 °C Ladung freigegeben) | LTO: lädt bei Frost, 82% Kapazität bei −20 °C |
| **Kaltes Klima, mäßiger Frost (−10 bis −15 °C)** | **LTO** oder **Na-ion** (Zelle für die Mindest-Ladetemperatur des Standorts freigegeben) | LiFePO4 (mit Reserve) | Na-ion balanciert Dichte und Kälte — Ladefenster im Zell-Datenblatt prüfen |
| **Maximale Lebensdauer (>10 Jahre)** | **LTO** | LiFePO4 | LTO: 10 000+ Zyklen; Solar-Repeater praktisch unbegrenzt |
| **Nachhaltigkeit / ethische Beschaffung** | **Na-ion** | LiFePO4 | Kein Kobalt, kein Lithium; verbessert sich schnell |
| **Maritim / Küste (Salz, Feuchtigkeit)** | **LiFePO4** | Li-ion | Versiegelte prismatische Zellen; inhärente Sicherheit |
| **Mobil / portabel** | **Li-ion** | LiFePO4 | Gewicht und Volumen zählen am meisten |
| **Budget-beschränkt** | **Li-ion** | LiFePO4 | Geringste Kosten pro Wh |

*(Die Prozentangaben zur entnehmbaren Kapazität sind Datenblattwerte bei 0,2C–0,5C-Last — siehe Hinweis in Abschnitt 2.)*

> **Checkliste für alpine Wintereinsätze:**
> 1. LTO wählen, oder eine Na-ion-Zelle, deren Datenblatt das Laden bei der Mindesttemperatur des Standorts erlaubt
> 2. Akkukapazität um 1,5–2× überdimensionieren für Kälte-Derating
> 3. Solarpanel um 3–5× überdimensionieren für kurze Wintertage
> 4. MPPT aktivieren (`set board.mppt 1`)
> 5. Bei Li-ion/LiFePO4: `set board.fmax` konfigurieren und `set board.tccal` ausführen
> 6. Überwachen über `get board.stats` und `get board.socdebug`

---

## 9. Langzeit-Alterung & Zyklenlebensdauer

| Chemie | Zyklen bis 80% | Kalendarische Alterung | Optimaler Lager-SOC |
|---|---|---|---|
| **Li-ion** | 500–1000 | Mäßig (schneller bei hoher Temp/SOC) | 40–60% bei 15–25 °C |
| **LiFePO4** | 2000–5000 | Gering | 50% bei Raumtemperatur |
| **LTO** | 10 000+ | Sehr gering | Beliebiger SOC; sehr tolerant |
| **Na-ion** | 1000–3000 | Gering | zellabhängig (0 V bis 30–50 % SOC) |

**Solar-Repeater-Kontext:** Ein gut dimensionierter Solarrepeater macht **keinen** Vollzyklus pro Tag. Das tatsächliche Profil ist flaches Mikro-Cycling mit starker saisonaler Variation:

- **Tägliche Entladung** beträgt nur **2–3%** der Akkukapazität (bei richtiger Dimensionierung: ≥ 7 Ah für eine <1-W-Last)
- **Tägliche Nachladung** bringt **10–20%** an sonnigen Tagen, abhängig von der Panelgröße
- **Winter (Dez–Jan):** Der Akku entlädt sich langsam über mehrtägige Trübwetterperioden — er „rettet“ den Repeater über die dunkle Zeit. SOC kann auf 30–50% fallen, bevor die nächste Sonnenphase kommt
- **Sommer:** Der Akku pendelt dauerhaft zwischen **95–100% SOC**, fällt selten unter 90%

Das bedeutet, der Akku erfährt vielleicht **10–30 äquivalente Vollzyklen pro Jahr** — nicht 365.

| Chemie | Zyklen bis 80% | Äquiv. Vollzyklen/Jahr | Erwartete Lebensdauer |
|---|---|---|---|
| **Li-ion** | 500–1000 | ~10–30 | **15–50+ Jahre** (zyklusbegrenzt) |
| **LiFePO4** | 2000–5000 | ~10–30 | **65+ Jahre** (zyklusbegrenzt) |
| **LTO** | 10 000+ | ~10–30 | **praktisch unbegrenzt** |
| **Na-ion** | 1000–3000 | ~10–30 | **30–100+ Jahre** (zyklusbegrenzt) |

> **Die eigentliche Alterungsgefahr ist nicht das Cycling — sondern kalendarische Alterung bei hohem SOC und hoher Temperatur.**
>
> Im Sommer steht der Akku monatelang bei 95–100% SOC in einem Gehäuse, das in praller Sonne 50 °C erreichen kann. Für Li-ion ist diese Kombination (hoher SOC + hohe Temperatur) der Alterungsbeschleuniger Nr. 1. Genau deshalb verwendet das Inhero MR2 **konservative Ladeschlussspannungen** (z. B. 4,1 V statt 4,2 V für Li-ion) — niedrigere Vco reduziert den Ruhe-SOC und verlangsamt die kalendarische Alterung drastisch.

**Wie stark wirkt sich das aus? Typischer NMC-Li-ion-Kapazitätsverlust pro Jahr (ohne Cycling, reine Lagerung):**

| Temperatur | 4,2 V (100% SOC) | 4,1 V (~85% SOC) | Reduktion |
|---|---|---|---|
| 25 °C (Indoor) | ~3–5%/Jahr | ~1–2%/Jahr | 2–3× langsamer |
| 40 °C (warmes Gehäuse) | ~8–15%/Jahr | ~3–5%/Jahr | 2–3× langsamer |
| **50 °C (sonnenexponiert)** | **~20–30%/Jahr** | **~6–10%/Jahr** | **3× langsamer** |

> Bei 4,2 V und 50 °C erreicht eine Li-ion-Zelle 80% Kapazität (End of Life) in ca. **3 Jahren**. Bei 4,1 V und 50 °C hält die gleiche Zelle **8–10 Jahre**. Die 100-mV-Vco-Absenkung allein bringt 2–3× mehr Lebensdauer — zum Preis von nur ~10% weniger nutzbarer Kapazität.
>
> **Fazit:** Für sonnenexponierte Outdoor-Installationen ist die Vco-Absenkung von 4,2→4,1 V die wirksamste Einzelmaßnahme zur Verlängerung der Li-ion-Lebensdauer. Wenn das Gehäuse regelmäßig über 40 °C kommt, LiFePO4 oder LTO in Betracht ziehen — diese Chemien sind praktisch immun gegen kalendarische Alterung bei hohem SOC.
>
> LiFePO4 und LTO sind wesentlich toleranter gegenüber dauerhaft hohem SOC. Na-ion altert mäßig. Für heiße/exponierte Standorte LiFePO4 oder LTO bevorzugen.

---

## 10. Zukunftsausblick

**Na-ion** ist die Chemie, die man beobachten sollte. Stand 2025/2026:
- Energiedichte verbessert sich mit jeder Generation (Ziel: 160+ Wh/kg)
- Große Hersteller (CATL, BYD, HiNa) fahren die Produktion hoch
- Zellkosten werden voraussichtlich innerhalb von 2–3 Jahren unter Li-ion fallen
- Ideal für stationäre Anwendungen, wo absolute Energiedichte weniger kritisch ist

**Festkörperbatterien** könnten ab 2028+ erscheinen, sind aber für Off-Grid-Repeater-Anwendungen kurzfristig kaum relevant.

**LTO** bleibt der Goldstandard für extreme Umgebungen und wird voraussichtlich für spezialisierte Einsätze relevant bleiben. Die hohen Kosten und geringe Dichte werden die Verbreitung weiterhin auf Fälle beschränken, wo Kälteperformance und Zyklenlebensdauer entscheidend sind.

**LiFePO4** wird auf absehbare Zeit die Mainstream-Wahl bleiben — bewährt, sicher, bezahlbar und in vielen Formaten verfügbar.

---

## Siehe auch

- [README.md](README.md) — Übersicht, Feature-Matrix und Diagnose
- [DATASHEET.md](DATASHEET.md) — Hardware-Datenblatt, Pinouts und Spezifikationen
- [QUICK_START.md](QUICK_START.md) — Schnellstart für Inbetriebnahme und CLI-Setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — Alle Board-spezifischen CLI-Kommandos
- [FAQ.md](FAQ.md) — Häufig gestellte Fragen
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Vollständige technische Dokumentation
