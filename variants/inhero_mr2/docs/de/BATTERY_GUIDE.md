# Inhero MR2 — Akkuchemie-Ratgeber

> 🇬🇧 [English Version](../BATTERY_GUIDE.md)

## Inhalt

- [Einleitung](#einleitung)
- [1. Chemie-Übersicht](#1-chemie-übersicht)
  - [Li-Ion (NMC/NCA, 1S)](#li-ion-nmcnca-1s)
  - [LiFePO4 (LFP, 1S)](#lifepo4-lfp-1s)
  - [LTO (Lithium-Titanat, 2S)](#lto-lithium-titanat-2s)
  - [Na-Ion (Natrium-Ionen, 1S)](#na-ion-natrium-ionen-1s)
- [2. Vergleichstabelle](#2-vergleichstabelle)
- [3. Temperaturverhalten](#3-temperaturverhalten)
  - [Kälteperformance-Ranking](#kälteperformance-ranking)
  - [Laden bei Kälte](#laden-bei-kälte)
  - [Temperatur-Derating (TTL & Anzeige)](#temperatur-derating-ttl--anzeige)
- [4. Zellauswahl & Bauformen](#4-zellauswahl--bauformen)
  - [Li-Ion-Zellen](#li-ion-zellen)
  - [LiFePO4-Zellen](#lifepo4-zellen)
  - [LTO-Zellen](#lto-zellen)
  - [Na-Ion-Zellen](#na-ion-zellen)
- [5. Kapazitätsplanung](#5-kapazitätsplanung)
  - [Stromverbrauch](#stromverbrauch)
  - [Warum der Strom von der Batteriespannung abhängt](#warum-der-strom-von-der-batteriespannung-abhängt)
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

Die Wahl der richtigen Akkuchemie ist eine der folgenreichsten Entscheidungen beim Deployment eines Inhero MR2 Repeaters. Sie beeinflusst Laufzeit, Kältezuverlässigkeit, Lebensdauer, Sicherheit und Gesamtbetriebskosten. Dieser Ratgeber liefert die Informationen für eine fundierte Entscheidung.

Das Inhero MR2 unterstützt **vier Akkuchemien**, jede mit eigenen Stärken. Es gibt keine „beste" Chemie — die richtige Wahl hängt von den Einsatzbedingungen ab.

---

## 1. Chemie-Übersicht

### Li-Ion (NMC/NCA, 1S)

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
- Etwas geringere Energiedichte als Li-Ion
- Darf unter 0 °C nicht geladen werden — Lithium-Plating-Risiko schädigt die Zelle permanent

**Inhero MR2 Besonderheiten:**
- Ladeschlussspannung: **3,5 V**
- **JEITA aktiv** — NTC erforderlich; Ladung gesperrt unter −2 °C (T-Cold)
- Frost-Ladestromreduzierung konfigurierbar über `set board.fmax`
- JEITA-WARM-Zone in Firmware neutralisiert zur Vermeidung von VBAT_OVP (siehe [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md#jeita-warm-zone--vbat_ovp-vermeidung))

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
- **Niedrige Energiedichte** (~80 Wh/kg) — braucht 2–3× das Volumen von Li-Ion für gleiche Kapazität
- **2S-Konfiguration erfordert externen Balancer** — ohne Balancer driften Zellen über die Zeit und riskieren Über-/Tiefentladung einzelner Zellen
- Exotische Bauformen — typisch zylindrisch mit Schraubterminals oder Aluminiumgehäuse
- Schwer Punkt-schweißbar (Aluminiumgehäuse)
- Teuer (2–3× pro Wh vs. Li-Ion)
- Eingeschränkte Verfügbarkeit — nur über Speziallieferanten

**Inhero MR2 Besonderheiten:**
- Ladeschlussspannung: **5,4 V** (2× 2,7 V pro Zelle)
- **JEITA deaktiviert** (`ts_ignore = true`) — kein NTC zum Laden erforderlich
- Temperatur für SOC-Derating kommt vom **BME280-Fallback** wenn kein NTC angeschlossen
- `set board.fmax` hat keine Wirkung (zeigt „N/A")
- Zellzahl im BQ25798 auf 2S konfiguriert

### Na-Ion (Natrium-Ionen, 1S)

Natrium-Ionen-Technologie — die nachhaltige Alternative mit abundanten, nicht-kritischen Rohstoffen.

**Stärken:**
- **Gute Kälteperformance** — 78% entnehmbar bei −20 °C
- **Kein Kobalt, kein Lithium** — nachhaltig und ethisch beschaffte Materialien (Natrium, Eisen, Mangan)
- **Kann bei 0 V gelagert und versandt werden** — kein Tiefentladeschaden (einzigartig unter allen Chemien)
- **Kann bei Frost geladen werden** — keine JEITA-Einschränkung nötig
- Gutes Sicherheitsprofil — kein thermisches Durchgehen unter normalen Bedingungen
- Sich schnell verbessernde Technologie — Energiedichte und Zyklenlebensdauer steigen mit jeder Generation

**Schwächen:**
- **Neue Technologie** — eingeschränkte Zellverfügbarkeit Stand 2025/2026
- Geringere Energiedichte als Li-Ion (~130 Wh/kg, steigend)
- Weniger validierte Zelloptionen und öffentliche Datenblätter
- Zyklenlebensdauer noch unter LiFePO4 bei den meisten aktuellen Zellen (1000–3000 Zyklen)
- Markt reift noch — Qualitätsvariation zwischen Herstellern

**Inhero MR2 Besonderheiten:**
- Ladeschlussspannung: **3,9 V**
- **JEITA deaktiviert** (`ts_ignore = true`) — kein NTC zum Laden erforderlich
- Temperatur für SOC-Derating kommt vom **BME280-Fallback** wenn kein NTC angeschlossen
- `set board.fmax` hat keine Wirkung (zeigt „N/A")

---

## 2. Vergleichstabelle

| | Li-Ion 1S | LiFePO4 1S | LTO 2S | Na-Ion 1S |
|---|---|---|---|---|
| **Energiedichte** | ~250 Wh/kg | ~160 Wh/kg | ~80 Wh/kg | ~130 Wh/kg |
| **Zyklenlebensdauer (bis 80%)** | 500–1000 | 2000–5000 | 10 000+ | 1000–3000 |
| **Kälteperformance** | Mäßig | Schwächste | Beste | Gut |
| **Entnehmbar bei −20 °C** | 55% | 46% | 82% | 78% |
| **Entnehmbar bei −10 °C** | 65% | 58% | 86% | 83% |
| **Entnehmbar bei 0 °C** | 75% | 70% | 90% | 88% |
| **Therm. Durchgehen** | Ja (Risiko) | Nein | Nein | Nein |
| **NTC erforderlich?** | Ja | Ja | Nein | Nein |
| **JEITA** | Aktiv | Aktiv | Deaktiviert | Deaktiviert |
| **Laden bei Frost?** | Nein (gesperrt <−2 °C) | Nein (gesperrt <−2 °C) | Ja | Ja |
| **Ladeschlussspannung** | 4,1 V | 3,5 V | 5,4 V (2S) | 3,9 V |
| **Low-V Sleep** | 3100 mV | 2700 mV | 3900 mV | 2500 mV |
| **Low-V Wake** | 3300 mV | 2900 mV | 4100 mV | 2700 mV |
| **Nennspannung** | 3,7 V | 3,2 V | 4,6 V | 3,1 V |
| **Zellformate** | 18650, 21700, Pouch | 18650, 26650, prismat. | Schraubterminal, Alu | 18650, prismatisch |
| **Verfügbarkeit** | Sehr gut | Gut | Eingeschränkt | Eingeschränkt |
| **Relativer Preis (pro Wh)** | Niedrig | Niedrig–Mittel | Hoch | Mittel |

---

## 3. Temperaturverhalten

### Kälteperformance-Ranking

Von bester zu schlechtester Kälteperformance:

1. **LTO** — 82% entnehmbar bei −20 °C, lädt bei Frost
2. **Na-Ion** — 78% entnehmbar bei −20 °C, lädt bei Frost
3. **Li-Ion** — 55% entnehmbar bei −20 °C, Laden bei Frost gesperrt
4. **LiFePO4** — 46% entnehmbar bei −20 °C, Laden bei Frost gesperrt

> Das Ranking überrascht möglicherweise Nutzer, die LiFePO4 als bewährtes „Arbeitstier" kennen. Während LiFePO4 bei Zyklenlebensdauer und Sicherheit glänzt, ist es tatsächlich die **schlechteste Chemie bei Kälte** unter den vier unterstützten. Das ist besonders relevant für alpine Einsätze und Winterbetrieb.

### Laden bei Kälte

| Chemie | Laden bei Frost? | Mechanismus |
|---|---|---|
| **Li-Ion** | Nein — gesperrt unter −2 °C | JEITA T-Cold (Hardware, BQ25798) |
| **LiFePO4** | Nein — gesperrt unter −2 °C | JEITA T-Cold (Hardware, BQ25798) |
| **LTO** | Ja — lädt bei jeder Temperatur | JEITA deaktiviert (`ts_ignore = true`) |
| **Na-Ion** | Ja — lädt bei jeder Temperatur | JEITA deaktiviert (`ts_ignore = true`) |

Für Li-Ion und LiFePO4 wird die Ladung auch im **T-Cool-Bereich** (+3 °C bis −2 °C mit Inhero-Spannungsteiler) reduziert, konfigurierbar über `set board.fmax`. Siehe [FAQ #6](FAQ.md#6-was-wird-durch-set-boardfmax-beeinflusst).

**Warum ist Frostladen bei Li-Ion und LiFePO4 gefährlich?** Bei niedrigen Temperaturen können Lithium-Ionen nicht ordnungsgemäß in die Graphit-Anode interkalieren. Stattdessen lagern sie sich als metallisches Lithium auf der Anodenoberfläche ab („Lithium-Plating"). Das reduziert die Kapazität permanent und kann interne Kurzschlüsse erzeugen — ein Sicherheitsrisiko.

LTO und Na-Ion verwenden andere Anodenmaterialien (Lithium-Titanat bzw. Hard Carbon), die nicht vom Lithium-Plating betroffen sind, weshalb Frostladen sicher ist.

> **Felderfahrung vs. Theorie:** Viele Repeater-Betreiber laden Li-Ion-Zellen bei Frost erfolgreich mit geringen Solarströmen (<0,1C) und berichten über mehrere Winter hinweg von keiner messbaren Degradation. Die [YYCMesh-Community](https://yycmesh.com/blog/cold-weather-charging) dokumentierte zwei Jahre alpine Einsätze in den kanadischen Rockies (bis −40 °C) mit Standard-18650-Zellen — die Innenwidersstände lagen weiterhin im Werksspezifikationsbereich. Ihre Schlüsselfaktoren: sehr geringe Laderaten (<0,05C), passive Sonnenerwärmung der Gehäuse und Ladung während der wärmsten Tageszeit.
>
> Das sind wertvolle Praxisdaten und der Ansatz funktioniert offensichtlich für viele Setups. Allerdings gelten diese Ergebnisse speziell für Konfigurationen mit **großen Akkukapazitäten und relativ geringer PV-Leistung** (Laderaten deutlich unter 0,05C). Sie sollten nicht als generelle Entwarnung beim Thema Lithium-Plating verstanden werden. *Es kommt darauf an* — auf Laderate, Zellqualität, Panelgröße und Temperatur. Die Degradation durch Lithium-Plating ist **kumulativ und subtil** — sie zeigt sich nicht als plötzlicher Ausfall, sondern als schleichender Kapazitätsverlust über Jahre. Zwei zusätzliche Risiken werden häufig unterschätzt:
>
> 1. **PV-Panels liefern bei Kälte mehr Leistung** (Silizium-Temperaturkoeffizient ~−0,35%/°C). Ein 5-W-Panel bei −10 °C liefert deutlich mehr Strom als bei +25 °C. Schneereflexion kann die Leistung sogar über die Nennwerte hinaus steigern.
> 2. **Zellqualität variiert.** Ergebnisse mit Premium-Zellen (niedriger Innenwiderstand, konsistente Chemie) lassen sich nicht unbedingt auf Budget-Zellen übertragen.
>
> Das Inhero MR2 verfolgt einen konservativen Ansatz: Der NTC-Batterietemperatursensor veranlasst den BQ25798, die Ladung zu blockieren, bis die Zelle sich über −2 °C erwärmt hat (JEITA T-Cold), und lädt danach bis zum Verlassen der T-Cool-Zone mit einer reduzierten Laderate (konfigurierbar über `set board.fmax`). An sonnigen Wintertagen läuft das Board über den Power-Path auf Solar, während der Akku geschützt bleibt. Sobald die direkte Sonneneinstrahlung das Gehäuse und die Akkutemperatur über den Schwellwert erwärmt — was bei durchdachtem Gehäusedesign erstaunlich schnell geht — wird die Ladung automatisch wieder freigegeben. Das bietet das Beste aus beiden Welten: kein Plating-Risiko bei gleichzeitig minimalem Verlust an Ladezeit.

### Temperatur-Derating (TTL & Anzeige)

Die Firmware verwendet ein **Trapped-Charge-Modell**: Kälte sperrt den Boden der Entladekurve — die Zelle erreicht ihre Abschaltspannung, während Ladung noch gespeichert ist. SOC% selbst ist rein Coulomb-basiert und temperaturunabhängig.

Das Modell verwendet eine chemiespezifische lineare Funktion für den Derating-Faktor:

```
f(T) = max(f_min, 1,0 - k × (T_ref - T))    für T < T_ref
f(T) = 1,0                                    für T >= T_ref
```

T_ref = 25 °C für alle Chemien (kein Derating bei Raumtemperatur und darüber).

| Chemie | k (/°C) | f_min | Bei −20 °C | Bei −10 °C | Bei 0 °C | Bei 10 °C |
|--------|---------|-------|-----------|-----------|---------|---------|
| Li-Ion | 0,005 | 0,75 | 0,78 | 0,83 | 0,88 | 0,93 |
| LiFePO4 | 0,006 | 0,70 | 0,73 | 0,79 | 0,85 | 0,91 |
| Na-Ion | 0,003 | 0,85 | 0,87 | 0,90 | 0,93 | 0,96 |
| LTO | 0,002 | 0,88 | 0,91 | 0,93 | 0,95 | 0,97 |

**Praktische Effekte:**
- SOC% ändert sich nur durch reale Ladungsflüsse — nicht durch Temperatur
- TTL sinkt bei Kälte, besonders bei niedrigem SOC (Trapped-Charge-Modell sperrt den Boden)
- `get board.telem` zeigt beide Werte: `SOC:95.0% (78%)` = gespeichert (entnehmbar)

→ Siehe [IMPLEMENTATION_SUMMARY.md — §5a Temperatur-Derating](IMPLEMENTATION_SUMMARY.md#5a-temperatur-derating) für die vollständige technische Implementierung.

---

## 4. Zellauswahl & Bauformen

### Li-Ion-Zellen

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
| **18650** | 1400–1800 mAh | Geringere Kapazität als Li-Ion 18650; weniger verbreitet |
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
- 2–3× Volumen und Gewicht im Vergleich zu Li-Ion für gleiche Energie einplanen
- Schraubterminals sind robust für Outdoor-Einsätze — kein Punktschweißen nötig

### Na-Ion-Zellen

| Bauform | Typische Kapazität | Hinweise |
|---|---|---|
| **18650** | 1000–1500 mAh | Aufkommend; Zellen der ersten Generation |
| **Prismatisch** | 5000–20 000 mAh | Größere Formate erscheinen von HiNa, CATL, Faradion |

**Tipps:**
- Technologie entwickelt sich schnell — vor dem Kauf aktuelle verfügbare Zellen prüfen
- Kann bei 0 V versandt werden (anders als alle Lithium-Chemien) — vereinfacht Logistik
- Keine besondere Lagerungsbehandlung nötig
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

> **Wie diese Werte ermittelt werden:** MeshCore sendet auf **869,618 MHz** im EU868-**g3-Subband** (869,4–869,65 MHz), das bis zu +27 dBm ERP und einen **10% Duty Cycle** erlaubt. Bei +22 dBm ziehen SX1262 + MCU ~130 mA während TX. Bei 10% TX-Anteil: `0,90 × 7,6 + 0,10 × 130 = 19,8 mA` → **~65 mW bzw. ~1,57 Wh/Tag** — das regulatorische Maximum.
>
> **Gemessen typisch (~12,3 mA):** Validierte 24h-Messung im Repeater-Betrieb mit typischem Traffic: **295 mAh/Tag @ 3,32 V** = 0,98 Wh/Tag → **~41 mW bzw. ~0,98 Wh/Tag**.

### Warum der Strom von der Batteriespannung abhängt

Das Inhero MR2 verwendet einen hocheffizienten **Buck-Converter**, um die 3,3-V-Schiene für MCU und Funk zu erzeugen. Dadurch zieht das Board eine annähernd **konstante Leistung** (Watt), keinen konstanten Strom (Ampere).

Weil Leistung = Spannung × Strom, bedeutet höhere Batteriespannung weniger Strom aus dem Akku — aber die Leistung bleibt gleich:

| Chemie | Nennspannung | Idle | Gemessen typisch | Worst Case (10% DC) |
|---|---|---|---|---|
| Na-Ion | 3,1 V | ~8,1 mA (25 mW) | ~13,2 mA (41 mW) | ~21,0 mA (65 mW) |
| LiFePO4 | 3,2 V | ~7,8 mA (25 mW) | ~12,8 mA (41 mW) | ~20,3 mA (65 mW) |
| Li-Ion | 3,7 V | ~6,8 mA (25 mW) | ~11,1 mA (41 mW) | ~17,6 mA (65 mW) |
| LTO (2S) | 4,6 V | ~5,4 mA (25 mW) | ~8,9 mA (41 mW) | ~14,1 mA (65 mW) |

> **Wichtig für die Kapazitätsplanung:** Nicht einfach mA × Stunden rechnen — das funktioniert nur innerhalb einer Chemie bei einer Spannung. Beim Vergleich verschiedener Chemien immer in **Wh** (Energie) rechnen: `Energie (Wh) = Wh/Tag × Tage`. Dann auf die eigene Chemie umrechnen: `mAh = Wh × 1000 ÷ V_nominal`. **0,98 Wh/Tag** (gemessen typisch) oder **1,57 Wh/Tag** (Worst Case 10% DC) je nach erwartetem Traffic und Sicherheitsreserve verwenden.
>
> **Beispiel:** 30 Tage Autonomie bei gemessen typischer Last = 29 Wh benötigt (Worst Case: 47 Wh).
> - LiFePO4 (3,2 V): 29 000 ÷ 3,2 = **9 063 mAh** (Worst Case: 14 688 mAh)
> - LTO 2S (4,6 V): 29 000 ÷ 4,6 = **6 304 mAh** (Worst Case: 10 217 mAh)

### Dimensionierung für Autonomie

**Energieansatz (empfohlen):** `Energie (Wh) = Wh/Tag × gewünschte Tage Autonomie` — **0,98 Wh/Tag** (gemessen typisch) oder **1,57 Wh/Tag** (Worst Case 10% DC) für konservative Auslegung

**Umrechnung in mAh für die eigene Chemie:** `mAh = Wh × 1000 ÷ V_nominal`

Die mAh-Spalte unten ist bei 3,3 V berechnet (≈ LiFePO4- / Na-Ion-Nennspannung). Für Li-Ion oder LTO die Energie-Spalte mit obiger Formel verwenden — siehe [Warum der Strom von der Batteriespannung abhängt](#warum-der-strom-von-der-batteriespannung-abhängt).

| Gewünschte Autonomie | Gemessen typisch (0,98 Wh/Tag) | Worst Case (1,57 Wh/Tag) | mAh @ 3,3 V (typisch) | Empfohlen |
|---|---|---|---|---|
| **3 Tage** (Indoor, Netz-Backup) | 2,9 Wh | 4,7 Wh | 891 mAh | 1500 mAh |
| **7 Tage** (Solar, Sommer) | 6,9 Wh | 11,0 Wh | 2079 mAh | 3500 mAh |
| **14 Tage** (Solar, Winter) | 13,7 Wh | 22,0 Wh | 4158 mAh | 6000 mAh |
| **30 Tage** (Alpin, minimale Solar) | 29,4 Wh | 47,1 Wh | 8909 mAh | 12 000+ mAh |

> **Kälte-Reserve:** Für Einsätze unter 0 °C die Kapazität um den Kehrwert des Derating-Faktors erhöhen. Beispiel: LiFePO4 bei −10 °C hat f(T) = 0,79, daher braucht man `Kapazität / 0,79 ≈ 1,27×` die Kapazität im Vergleich zu Raumtemperatur. Die TTL-Berechnung wendet dieses Derating automatisch an.

### Die 90%-Regel

`board.batcap` auf **90% der Nennkapazität** setzen. Das Inhero MR2 verwendet konservative Ladeschlussspannungen (z. B. 4,1 V statt 4,2 V für Li-Ion), wodurch die oberen ~10% der Nennkapazität bewusst nicht genutzt werden — das verbessert die Zyklenlebensdauer erheblich.

**Beispiel:** 10 000 mAh Nenn → `set board.batcap 9000`

→ Siehe [FAQ #4](FAQ.md#4-welchen-mah-wert-gebe-ich-bei-set-boardbatcap-ein)

---

## 6. Solar-Ladebetrachtungen

**Formel für maximalen Ladestrom:** `I_Ladung (mA) = Panel-Leistung (W) / Nennspannung Akku (V)`

| Chemie | Panel | Ladestrom | `set board.imax` |
|---|---|---|---|
| Li-Ion (3,7 V) | 2 W | 540 mA | `set board.imax 540` |
| LiFePO4 (3,2 V) | 1 W | 310 mA | `set board.imax 310` |
| LTO (4,6 V) | 5 W | 1090 mA | `set board.imax 1090` |
| Na-Ion (3,1 V) | 3 W | 970 mA | `set board.imax 970` |

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
- **Li-Ion / LiFePO4:** Solarladung ist bei Frost (<−2 °C) gesperrt. An kalten Wintertagen kann das Panel Strom liefern, aber der Akku nimmt keine Ladung an, bis er sich über −2 °C erwärmt. Das Board läuft derweil direkt auf Solar, wenn die Leistung ausreicht. Beachte: PV-Panels liefern **bei Kälte mehr Leistung** (Silizium-Temperaturkoeffizient ~−0,35%/°C) — tatsächliche Ladeströme können die Nennwerte überschreiten. Ein weiterer Grund, warum hardwareseitige Ladungsblockierung via JEITA essenziell ist, statt sich auf „geringe Ströme“ zu verlassen.
- **LTO / Na-Ion:** Solarladung funktioniert auch bei tiefem Frost — ein erheblicher Vorteil für alpine Einsätze, wo Frost tage- oder wochenlang anhalten kann.

---

## 7. Sicherheit & Schutz

| Chemie | Therm. Durchgehen | BMS/Schutz erforderlich? | Inhero MR2 Schutz |
|---|---|---|---|
| **Li-Ion** | ⚠️ Ja — Brand-/Explosionsgefahr bei Missbrauch | Ja — zwingend | JEITA, Low-V Sleep, OVP, Ladespannungslimit |
| **LiFePO4** | ✅ Nein — inhärent sicher | Empfohlen | JEITA, Low-V Sleep, OVP |
| **LTO** | ✅ Nein — inhärent sicher | Empfohlen (Balancer!) | Low-V Sleep, OVP, Zellzahl-Konfig |
| **Na-Ion** | ✅ Nein unter Normalbedingungen | Empfohlen | Low-V Sleep, OVP |

**Eingebaute Sicherheitsfeatures des Inhero MR2 (alle Chemien):**
- **Low-Voltage-Sleep** — INA228 ALERT ISR löst System-Sleep aus, um Tiefentladung zu verhindern
- **Ladespannungslimit** — BQ25798 pro Chemie konfiguriert, gegen Überladung
- **VBAT_OVP** — Hardware-Überspannungsschutz im BQ25798
- **200 mV Hysterese** — Verhindert Motorboating (schnelles Ein/Aus-Toggeln) nahe leer
- **JEITA-Temperaturschutz** (nur Li-Ion/LiFePO4) — Hardware-Ladekontrolle über NTC

**Nutzerverantwortung:**
- Li-Ion: Zellen mit Schutzschaltung (PCM) oder ordentlichem BMS verwenden
- LTO 2S: **Externer Balancer ist Pflicht** für Langzeitbetrieb
- Alle Chemien: Korrekte Chemie über `set board.bat` setzen — falsche Chemie = falsche Spannungen = Schadensrisiko

---

## 8. Einsatzempfehlungen

| Szenario | Empfohlen | Alternative | Hinweise |
|---|---|---|---|
| **Indoor, gemäßigtes Klima (0–40 °C)** | **LiFePO4** | Li-Ion | LiFePO4: bestes Verhältnis Sicherheit + Zyklen |
| **Outdoor, gemäßigt (−5 bis +35 °C)** | **LiFePO4** | Li-Ion | Frost selten; fmax behandelt gelegentliche Kälte |
| **Platzbeschränktes Gehäuse** | **Li-Ion** | — | Höchste Energiedichte; nichts anderes passt |
| **Alpin, extreme Kälte (−20 °C und darunter)** | **LTO** | Na-Ion | LTO: lädt bei Frost, 82% Kapazität bei −20 °C |
| **Kaltes Klima, mäßiger Frost (−10 bis −15 °C)** | **Na-Ion** oder **LTO** | LiFePO4 (mit Reserve) | Beide laden bei Frost; Na-Ion balanciert Dichte und Kälte |
| **Maximale Lebensdauer (>10 Jahre)** | **LTO** | LiFePO4 | LTO: 10 000+ Zyklen; Solar-Repeater praktisch unbegrenzt |
| **Nachhaltigkeit / ethische Beschaffung** | **Na-Ion** | LiFePO4 | Kein Kobalt, kein Lithium; verbessert sich schnell |
| **Maritim / Küste (Salz, Feuchtigkeit)** | **LiFePO4** | Li-Ion | Versiegelte prismatische Zellen; inhärente Sicherheit |
| **Mobil / portabel** | **Li-Ion** | LiFePO4 | Gewicht und Volumen zählen am meisten |
| **Budget-beschränkt** | **Li-Ion** | LiFePO4 | Geringste Kosten pro Wh |

> **Checkliste für alpine Wintereinsätze:**
> 1. LTO oder Na-Ion für Frostladen wählen
> 2. Akkukapazität um 1,5–2× überdimensionieren für Kälte-Derating
> 3. Solarpanel um 3–5× überdimensionieren für kurze Wintertage
> 4. MPPT aktivieren (`set board.mppt 1`)
> 5. Bei Li-Ion/LiFePO4: `set board.fmax` konfigurieren und `set board.tccal` ausführen
> 6. Überwachen über `get board.stats` und `get board.socdebug`

---

## 9. Langzeit-Alterung & Zyklenlebensdauer

| Chemie | Zyklen bis 80% | Kalendarische Alterung | Optimaler Lager-SOC |
|---|---|---|---|
| **Li-Ion** | 500–1000 | Mäßig (schneller bei hoher Temp/SOC) | 40–60% bei 15–25 °C |
| **LiFePO4** | 2000–5000 | Gering | 50% bei Raumtemperatur |
| **LTO** | 10 000+ | Sehr gering | Beliebiger SOC; sehr tolerant |
| **Na-Ion** | 1000–3000 | Gering | 0 V (einzigartig — kein Schaden) |

**Solar-Repeater-Kontext:** Ein gut dimensionierter Solarrepeater macht **keinen** Vollzyklus pro Tag. Das tatsächliche Profil ist flaches Mikro-Cycling mit starker saisonaler Variation:

- **Tägliche Entladung** beträgt nur **2–3%** der Akkukapazität (bei richtiger Dimensionierung: ≥ 7 Ah für eine <1-W-Last)
- **Tägliche Nachladung** bringt **10–20%** an sonnigen Tagen, abhängig von der Panelgröße
- **Winter (Dez–Jan):** Der Akku entlädt sich langsam über mehrtägige Trübwetterperioden — er „rettet“ den Repeater über die dunkle Zeit. SOC kann auf 30–50% fallen, bevor die nächste Sonnenphase kommt
- **Sommer:** Der Akku pendelt dauerhaft zwischen **95–100% SOC**, fällt selten unter 90%

Das bedeutet, der Akku erfährt vielleicht **10–30 äquivalente Vollzyklen pro Jahr** — nicht 365.

| Chemie | Zyklen bis 80% | Äquiv. Vollzyklen/Jahr | Erwartete Lebensdauer |
|---|---|---|---|
| **Li-Ion** | 500–1000 | ~10–30 | **15–50+ Jahre** (zyklusbegrenzt) |
| **LiFePO4** | 2000–5000 | ~10–30 | **65+ Jahre** (zyklusbegrenzt) |
| **LTO** | 10 000+ | ~10–30 | **praktisch unbegrenzt** |
| **Na-Ion** | 1000–3000 | ~10–30 | **30–100+ Jahre** (zyklusbegrenzt) |

> **Die eigentliche Alterungsgefahr ist nicht das Cycling — sondern kalendarische Alterung bei hohem SOC und hoher Temperatur.**
>
> Im Sommer steht der Akku monatelang bei 95–100% SOC in einem Gehäuse, das in praller Sonne 50 °C erreichen kann. Für Li-Ion ist diese Kombination (hoher SOC + hohe Temperatur) der Alterungsbeschleuniger Nr. 1. Genau deshalb verwendet das Inhero MR2 **konservative Ladeschlussspannungen** (z. B. 4,1 V statt 4,2 V für Li-Ion) — niedrigere Vco reduziert den Ruhe-SOC und verlangsamt die kalendarische Alterung drastisch.

**Wie stark wirkt sich das aus? Typischer NMC-Li-Ion-Kapazitätsverlust pro Jahr (ohne Cycling, reine Lagerung):**

| Temperatur | 4,2 V (100% SOC) | 4,1 V (~85% SOC) | Reduktion |
|---|---|---|---|
| 25 °C (Indoor) | ~3–5%/Jahr | ~1–2%/Jahr | 2–3× langsamer |
| 40 °C (warmes Gehäuse) | ~8–15%/Jahr | ~3–5%/Jahr | 2–3× langsamer |
| **50 °C (sonnenexponiert)** | **~20–30%/Jahr** | **~6–10%/Jahr** | **3× langsamer** |

> Bei 4,2 V und 50 °C erreicht eine Li-Ion-Zelle 80% Kapazität (End of Life) in ca. **3 Jahren**. Bei 4,1 V und 50 °C hält die gleiche Zelle **8–10 Jahre**. Die 100-mV-Vco-Absenkung allein bringt 2–3× mehr Lebensdauer — zum Preis von nur ~10% weniger nutzbarer Kapazität.
>
> **Fazit:** Für sonnenexponierte Outdoor-Installationen ist die Vco-Absenkung von 4,2→4,1 V die wirksamste Einzelmaßnahme zur Verlängerung der Li-Ion-Lebensdauer. Wenn das Gehäuse regelmäßig über 40 °C kommt, LiFePO4 oder LTO in Betracht ziehen — diese Chemien sind praktisch immun gegen kalendarische Alterung bei hohem SOC.
>
> LiFePO4 und LTO sind wesentlich toleranter gegenüber dauerhaft hohem SOC. Na-Ion altert mäßig. Für heiße/exponierte Standorte LiFePO4 oder LTO bevorzugen.

---

## 10. Zukunftsausblick

**Na-Ion** ist die Chemie, die man beobachten sollte. Stand 2025/2026:
- Energiedichte verbessert sich mit jeder Generation (Ziel: 160+ Wh/kg)
- Große Hersteller (CATL, BYD, HiNa) fahren die Produktion hoch
- Zellkosten werden voraussichtlich innerhalb von 2–3 Jahren unter Li-Ion fallen
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
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Vollständige technische Dokumentation
