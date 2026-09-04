# Inhero MR2 — Battery Chemistry Guide

> 🇩🇪 [Deutsche Version](de/BATTERY_GUIDE.md)

## Contents

- [Introduction](#introduction)
- [1. Chemistry Overview](#1-chemistry-overview)
  - [Li-ion (NMC/NCA, 1S)](#li-ion-nmcnca-1s)
  - [LiFePO4 (LFP, 1S)](#lifepo4-lfp-1s)
  - [LTO (Lithium Titanate, 2S)](#lto-lithium-titanate-2s)
  - [Na-ion (Sodium Ion, 1S)](#na-ion-sodium-ion-1s)
- [2. Comparison Table](#2-comparison-table)
- [3. Temperature Behavior](#3-temperature-behavior)
  - [Cold Performance Ranking](#cold-performance-ranking)
  - [Charging in Cold Conditions](#charging-in-cold-conditions)
  - [Battery Temperature Sensing](#battery-temperature-sensing)
  - [Temperature Derating](#temperature-derating)
- [4. Cell Selection & Form Factors](#4-cell-selection--form-factors)
  - [Li-ion Cells](#li-ion-cells)
  - [LiFePO4 Cells](#lifepo4-cells)
  - [LTO Cells](#lto-cells)
  - [Na-ion Cells](#na-ion-cells)
- [5. Capacity Planning](#5-capacity-planning)
  - [Power Consumption](#power-consumption)
  - [Why current depends on battery voltage](#why-current-depends-on-battery-voltage)
  - [Sizing for Autonomy](#sizing-for-autonomy)
  - [The 90% Rule](#the-90-rule)
- [6. Solar Charging Considerations](#6-solar-charging-considerations)
- [7. Safety & Protection](#7-safety--protection)
- [8. Deployment Recommendations](#8-deployment-recommendations)
- [9. Long-Term Aging & Cycle Life](#9-long-term-aging--cycle-life)
- [10. Future Outlook](#10-future-outlook)
- [See Also](#see-also)

---

## Introduction

Choosing the right battery chemistry is one of the most impactful decisions when deploying an Inhero MR2 repeater. It affects battery life, cold-weather reliability, service life, safety, and total cost of ownership. This guide provides the information needed to make an informed choice.

The Inhero MR2 supports **four battery chemistries**, each with distinct characteristics. There is no single "best" chemistry — the right choice depends on your deployment conditions.

---

## 1. Chemistry Overview

### Li-ion (NMC/NCA, 1S)

The most common rechargeable chemistry. NMC (Nickel-Manganese-Cobalt) and NCA (Nickel-Cobalt-Aluminum) cells dominate the consumer market.

**Strengths:**
- **Highest energy density** (~250 Wh/kg) — smallest and lightest for a given capacity
- Widely available in many form factors (18650, 21700, pouch)
- Inexpensive — large-scale production drives prices down
- Well-understood technology with decades of field data

**Weaknesses:**
- Limited cycle life (500–1000 cycles to 80% capacity)
- **Thermal runaway risk** — can ignite under abuse (overcharge, short circuit, puncture)
- Sensitive to cold: significant capacity loss below 0 °C
- Must not be charged below 0 °C — lithium plating risk damages the cell permanently
- Sensitive to heat: accelerated degradation above 40 °C
- **Calendar aging at high SOC + heat** — the #1 aging driver for solar repeaters, where the battery sits at 95–100% SOC for months in summer enclosures reaching 50 °C. This is why the Inhero MR2 lowers Vco to 4.1 V
- Requires protection circuit (BMS) to prevent overcharge/over-discharge

**Inhero MR2 specifics:**
- Charge voltage set to **4.1 V** (conservative, vs. typical 4.2 V) for improved cycle life
- **JEITA active** — NTC required; charging blocked below −2 °C (T-Cold)
- Frost charge reduction configurable via `set board.fmax`
- `set board.jeitaignore 1` switches the charger's temperature guard off entirely, within a 0.05C rate gate — at the operator's own risk, see [Charging in Cold Conditions](#charging-in-cold-conditions)

### LiFePO4 (LFP, 1S)

Iron-phosphate cathode chemistry. Popular in solar and off-grid applications for its safety and longevity.

**Strengths:**
- **Excellent cycle life** (2000–5000 cycles to 80% capacity)
- **No thermal runaway** — inherently safe cathode chemistry
- Good energy density for most deployments (~160 Wh/kg)
- Very flat discharge curve — voltage remains stable over a wide SOC range
- Tolerant to moderate overcharge/over-discharge

**Weaknesses:**
- **Most cold-sensitive** of all supported chemistries
- Flat discharge curve makes voltage-based SOC estimation unreliable (the Inhero MR2 solves this with a coulomb counter)
- Slightly lower energy density than Li-ion
- Must not be charged below 0 °C — lithium plating risk damages the cell permanently

**Inhero MR2 specifics:**
- Charge voltage: **3.5 V**
- **JEITA active** — NTC required; charging blocked below −2 °C (T-Cold)
- Frost charge reduction configurable via `set board.fmax`
- `set board.jeitaignore 1` switches the charger's temperature guard off entirely, within a 0.05C rate gate — at the operator's own risk, see [Charging in Cold Conditions](#charging-in-cold-conditions)
- JEITA WARM zone neutralized in firmware to prevent VBAT_OVP (see [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md#jeita-warm-zone--vbat_ovp-prevention))

### LTO (Lithium Titanate, 2S)

Lithium titanate anode chemistry. Used in industrial and transit applications for extreme durability.

**Strengths:**
- **Best cold performance** of all supported chemistries — 82% extractable at −20 °C
- **Extreme cycle life** (10 000+ cycles, some manufacturers claim 20 000+)
- Very safe — no lithium plating, no thermal runaway
- Wide operating temperature range (−30 °C to +60 °C charging)
- **Can be charged in frost** — no JEITA restriction needed
- Fast-charge capable (up to 5C or more)
- Very flat discharge curve

**Weaknesses:**
- **Low energy density** (~80 Wh/kg) — needs 2–3× the volume of Li-ion for the same capacity
- **2S configuration requires an external balancer** — without balancer, cells drift over time and risk over-charge/over-discharge of individual cells
- Exotic form factors — typically cylindrical with screw terminals or aluminum housing
- Difficult to spot-weld (aluminum housing)
- Expensive (2–3× per Wh vs. Li-ion)
- Limited retail availability — specialty suppliers only

**Inhero MR2 specifics:**
- Charge voltage: **5.4 V** (2× 2.7 V per cell)
- **No JEITA supervision needed** (`needs_jeita = false`) — the charger's temperature guard is switched off for this chemistry, so no NTC is required for charging
- An NTC is optional. A fitted one is read and reported like on any other chemistry; without one, the temperature for SOC derating comes from the **BME280** on the board
- `set board.fmax` has no effect (shown as "N/A"), and `set board.jeitaignore` is refused with `Err: This chemistry runs without JEITA (always 1)`
- Cell count set to 2S in BQ25798 configuration

### Na-ion (Sodium Ion, 1S)

Sodium-ion technology — the sustainable alternative using abundant, non-critical raw materials.

**Strengths:**
- **Good cold performance** — 78% extractable at −20 °C
- **No cobalt, no lithium** — ethically sourced, sustainable materials (sodium, iron, manganese)
- **Can be stored and shipped at 0 V** — no deep discharge damage (unique among all chemistries)
- **Can be charged in frost with cells rated for it** — the board does not block charging (no JEITA restriction); the cell datasheet sets the limit, see [Na-ion Cells](#na-ion-cells)
- Good safety profile — no thermal runaway under normal conditions
- Rapidly improving technology — energy density and cycle life increase with each generation

**Weaknesses:**
- **New technology** — limited cell availability as of 2025/2026
- Lower energy density than Li-ion (~130 Wh/kg, improving)
- Fewer validated cell options and public datasheets
- **Charge temperature window varies by manufacturer** — from 0 °C (many consumer and NFPP cells) to −20 °C (HiNa, AuroraCell, with rate and SOC limits); the cell datasheet is the authority, the board does not enforce it
- Cycle life still below LiFePO4 in most current cells (1000–3000 cycles)
- Market still maturing — quality variation between manufacturers

**Inhero MR2 specifics:**
- Charge voltage: **3.9 V**
- **No JEITA supervision** (`needs_jeita = false`) — the charger's temperature guard is off for this chemistry, so no NTC is required for charging. The board therefore enforces no lower charge temperature for Na-ion; choose a cell whose datasheet covers the coldest charging temperature at the site (see [Na-ion Cells](#na-ion-cells))
- An NTC is optional. A fitted one is read and reported like on any other chemistry; at 3.1 V nominal much of the discharge sits below the 3200 mV sensing bar, so on battery alone the reading is often `N/A` (see [Battery Temperature Sensing](#battery-temperature-sensing)). Without one, the temperature for SOC derating comes from the **BME280** on the board
- `set board.fmax` has no effect (shown as "N/A"), and `set board.jeitaignore` is refused with `Err: This chemistry runs without JEITA (always 1)`

---

## 2. Comparison Table

| | Li-ion 1S | LiFePO4 1S | LTO 2S | Na-ion 1S |
|---|---|---|---|---|
| **Energy density** | ~250 Wh/kg | ~160 Wh/kg | ~80 Wh/kg | ~130 Wh/kg |
| **Cycle life (to 80%)** | 500–1000 | 2000–5000 | 10 000+ | 1000–3000 |
| **Cold performance** | Moderate | Weakest | Best | Good |
| **Extractable at −20 °C** | 55% | 46% | 82% | 78% |
| **Extractable at −10 °C** | 65% | 58% | 86% | 83% |
| **Extractable at 0 °C** | 75% | 70% | 90% | 88% |
| **Thermal runaway risk** | Yes | No | No | No |
| **NTC required?** | Yes¹ | Yes¹ | No (optional) | No (optional) |
| **Battery temperature reported?** | With NTC | With NTC² | With NTC | With NTC² |
| **JEITA** | Active | Active | Not needed (`needs_jeita = false`) | Off (`needs_jeita = false`) |
| **Charge in frost?** | No (blocked <−2 °C)¹ | No (blocked <−2 °C)¹ | Yes | Cell-dependent³ |
| **Charge voltage** | 4.1 V | 3.5 V | 5.4 V (2S) | 3.9 V |
| **Low-V sleep** | 3100 mV | 2700 mV | 3900 mV | 2500 mV |
| **Low-V wake** | 3300 mV | 2900 mV | 4100 mV | 2700 mV |
| **Nominal voltage** | 3.7 V | 3.2 V | 4.6 V | 3.1 V |
| **Cell formats** | 18650, 21700, pouch | 18650, 26650, prismatic | Screw-terminal, aluminum | 18650, prismatic |
| **Availability** | Excellent | Good | Limited | Limited |
| **Relative cost (per Wh)** | Low | Low–Medium | High | Medium |

¹ `set board.jeitaignore 1` lifts both the cold block and the NTC requirement for Li-ion and LiFePO4, as long as the 0.05C gate passes. It also removes the hot-side charge suspend — see [Charging in Cold Conditions](#charging-in-cold-conditions).

² A fitted NTC is read on every chemistry. On battery alone the TS channel is off below 3200 mV, so LiFePO4 and Na-ion read `N/A` over much of their discharge — see [Battery Temperature Sensing](#battery-temperature-sensing).

³ The board does not block charging on Na-ion. The permissible charge temperature comes from the cell datasheet — 0 °C, −10 °C or −20 °C depending on the manufacturer. See [Na-ion Cells](#na-ion-cells).

> **Note on the extractable-capacity figures:** These are typical datasheet values at 0.2C–0.5C discharge loads. The MR2's sub-0.05C load is much gentler; the firmware's derating model (section 3) therefore shows higher extractable values in `get board.telem`.

---

## 3. Temperature Behavior

### Cold Performance Ranking

From best to worst cold-weather performance:

1. **LTO** — 82% extractable at −20 °C, charges in frost
2. **Na-ion** — 78% extractable at −20 °C; charging in frost only within the cell datasheet's window (0 °C to −20 °C depending on the cell)
3. **Li-ion** — 55% extractable at −20 °C, charging blocked in frost
4. **LiFePO4** — 46% extractable at −20 °C, charging blocked in frost

*(Datasheet values at 0.2C–0.5C loads — see the note in section 2; under the MR2's much gentler load the firmware's derating model below shows higher values.)*

> The ranking may surprise users familiar with LiFePO4's reputation as a "workhorse." While LiFePO4 excels in cycle life and safety, it is actually the **worst performer in cold** among the four supported chemistries. This matters significantly for alpine and winter deployments.

### Charging in Cold Conditions

| Chemistry | Charging in frost? | Mechanism |
|---|---|---|
| **Li-ion** | No — blocked below −2 °C, unless `board.jeitaignore` is armed | JEITA T-Cold (hardware, BQ25798) |
| **LiFePO4** | No — blocked below −2 °C, unless `board.jeitaignore` is armed | JEITA T-Cold (hardware, BQ25798) |
| **LTO** | Yes — charges at any temperature | No JEITA supervision (`needs_jeita = false`) |
| **Na-ion** | Not blocked by the board — the cell datasheet sets the limit (0 °C, −10 °C or −20 °C depending on the manufacturer) | No JEITA supervision (`needs_jeita = false`); the temperature window is the operator's cell choice |

For Li-ion and LiFePO4, charging in the **T-Cool range** (+3 °C to −2 °C with the Inhero voltage divider) is blocked by default and can be set to a reduced rate via `set board.fmax` (20%, 40% or 100%). `fmax` acts in this band only — below −2 °C (T-Cold) the hardware block stands regardless of `fmax`, and only `board.jeitaignore` lifts it. Note that selecting Li-ion or LiFePO4 via `set board.bat` resets `board.fmax` to 0%. [FAQ #6](FAQ.md#6-what-is-frost-charging-and-how-do-fmax-and-jeitaignore-work-together) shows the two settings side by side.

**Why is frost charging dangerous for Li-ion and LiFePO4?** At low temperatures, lithium ions cannot intercalate properly into the graphite anode. Instead, they deposit as metallic lithium on the anode surface ("lithium plating"). This permanently reduces capacity and can create internal short circuits — a safety hazard.

LTO uses a lithium titanate anode that works at about 1.55 V — far above the lithium plating potential — so frost charging is safe. Na-ion uses a hard carbon anode whose charge plateau sits within about 0.1 V of sodium plating; whether a given cell tolerates charging below 0 °C depends on its electrolyte and cell design, which is why manufacturers specify anything from 0 °C to −20 °C. The board does not supervise this for Na-ion — the cell datasheet is the authority.

> **Field experience vs. theory:** Many repeater operators charge Li-ion cells in frost with low solar currents and report no measurable degradation over multiple winters. The [YYCMesh community](https://yycmesh.com/blog/cold-weather-charging) documented two years of alpine deployments in the Canadian Rockies (down to −40 °C) with standard unprotected 18650 cells (3000–3500 mAh) and found internal resistance still within factory spec. They describe their own operating window as "most of our systems charge at < 0.1 C, often well below 0.05 C", fed by 1 W to 6 W panels with average charging currents typically below 200 mA and occasional peaks around 300 mA. That window **encloses** 0.05C — 200 mA into a 3.5 Ah cell is 0.057C. Their other factors: passive solar heating of enclosures, and charging coinciding with the warmest part of the day.
>
> They also name the limits of their own evidence: the theoretical "safe" cold charge rate is around 0.02C, the degraded cell they show is a single vape cell measured against a generic new value with no before-measurement and no control cell, and reliably detecting dendrites would need NMR spectrometry or imaging, "far outside the reach of the average hobbyist". That 0.02C is the source's own theoretical figure, while the operating window they actually report encloses 0.05C. Read it as what it is — honest field experience from operators who state themselves that they cannot measure the mechanism. It is neither proof that frost charging is harmless nor a reason to dismiss the practice. *It depends* — on charge rate, cell quality, panel size, and temperature. The degradation from lithium plating is **cumulative and permanent** — it shows up as capacity quietly gone over years. Two additional risks are often underestimated:
>
> 1. **PV panels produce more power in cold weather** (silicon temperature coefficient ~−0.35%/°C). A 5 W panel at −10 °C delivers significantly more current than at +25 °C. Snow reflection can push output even beyond rated wattage.
> 2. **Cell quality varies.** Results with premium cells (low internal resistance, consistent chemistry) may not transfer to budget cells.
>
> The Inhero MR2 ships conservatively: the NTC battery temperature sensor causes the BQ25798 to block charging until the cell warms above −2 °C (JEITA T-Cold) and, by default, keeps charging suspended through the T-Cool zone as well (`board.fmax` default 0%). Setting `set board.fmax` to 20%, 40% or 100% instead allows reduced-rate charging between −2 °C and +3 °C. On sunny winter days, the board runs from solar via the power path while the battery stays protected. Once direct sunlight heats up the enclosure and the battery temperature rises above the threshold — which happens surprisingly fast with proper enclosure design — charging resumes automatically.

**Overriding the temperature guard: `set board.jeitaignore`**

For operators who want the field practice described above, the firmware offers it as an explicit, bounded option that is **off by default**. `set board.jeitaignore 1` sets the BQ25798's TS_IGNORE bit; the charger then treats the TS pin as always good and keeps charging through frost. The command is accepted for Li-ion and LiFePO4 only — LTO and Na-ion already run without JEITA and answer `Err: This chemistry runs without JEITA (always 1)`.

**The gate.** The override only becomes effective while `board.batcap` has been set explicitly **and** `board.imax` is at or below **0.05C** of that capacity (9000 mAh → 450 mA). Outside that, the wish is stored but stays inactive and the CLI names the blocker: `jeitaignore set to 1, N/A, C>0.05`, or `jeitaignore set to 1, N/A, batcap not set`. Nothing is discarded — lowering `imax` back under the limit or raising `batcap` re-arms the override on its own, and the reply says so with `; jeitaignore 1`. The wish is what persists; the effective state is re-derived on every boot and on every chemistry change, and from power-on until the stored configuration is applied the hardware guard is in charge.

**What you give up.** TS_IGNORE removes the TS pin from every charge decision, the hot one included. While the override is on, the charger also stops suspending charge on the hot side (T-Hot, roughly +58 °C on the MR2 divider), and all four TS status bits report "no fault". The firmware offers no software replacement for either limit: the board sleeps in SYSTEMOFF with the charger enabled and no control loop runs there, so the 0.05C bound is the entire safety argument. On the hot side that bound is what keeps the residual risk small — 0.05C is thermally uninteresting. On the cold side the plating mechanism is unchanged; the gate bounds the rate, it does not remove the mechanism. Setting the flag is accepting accelerated cell ageing as the price for winter charge time.

**What it is scoped for.** The gate is a single number for every temperature, while the tolerable charge rate falls as the cell gets colder — roughly by half per 10 K. Just below the freezing point the margin at 0.05C is comfortable; it shrinks with every degree the site drops further. The override is dimensioned for central European frost, the conditions the [datasheet](DATASHEET.md) sizes panel and bank for. For sites regularly below about −20 °C, choose the chemistry accordingly: LTO is rated by its manufacturers for charging at −20 °C and below; among Na-ion cells only some are (HiNa and AuroraCell rate −20 °C with reduced rate, voltage and SOC, many consumer cells stop at −10 °C or 0 °C) — check the cell datasheet. The MR2 supports both chemistries.

**While it is on.** `get board.fmax` answers `N/A`, `set board.fmax` is refused with `Err: Fmax N/A while jeitaignore is on`, and `get board.conf` appends ` J:1`. `set board.jeitaignore 0` switches the override off and restores the stored `fmax` behaviour.

### Battery Temperature Sensing

The battery temperature comes from an NTC on the BQ25798 TS pin. The firmware reads that channel on **every chemistry**; whether an NTC is fitted is a wiring question and no longer depends on the configured chemistry. Two conditions decide whether a value is actually available:

- **Battery voltage.** Running on battery only, the TS channel is switched off while the battery voltage is between 0 and 3200 mV, so that solar readings keep working. With an input source qualified — the panel supplying power — the channel stays on, so a cold, nearly empty cell being charged does report its temperature. LTO 2S (4.6–5.4 V) is always above that bar and Li-ion 1S is above it for most of its curve, while LiFePO4 (3.2 V nominal) and Na-ion (3.1 V nominal) spend much of their discharge below it — on battery alone the temperature reads `N/A` there even with an NTC fitted.
- **Plausibility.** A decoded value is compared against a fresh BME280 reading; more than 15.0 °C apart, it is discarded and reported as `N/A`. This check exists because a missing or open NTC does not produce an obviously wrong number — the divider decodes to roughly −46 °C, which passes the plain range check unnoticed.

If no NTC reading has been accepted for 5 minutes, the temperature used for SOC derating falls back to the BME280 on the board. That fallback applies to every chemistry, whether the NTC is absent, the battery voltage is too low, or the reading was discarded.

An NTC is **required for charging** only on Li-ion and LiFePO4, where the BQ25798 reads an open TS pin as a frost condition and blocks charging. Fitting one (solder bridge or external sensor) is the correct fix for that. For an NTC-less installation of those two chemistries, `set board.jeitaignore 1` within the 0.05C gate is the alternative — it takes the TS pin out of the charge decision altogether, with the trade described above.

### Temperature Derating

The firmware uses a **Trapped Charge** model to estimate **extractable** capacity at the current battery temperature. Cold temperatures lock the bottom of the discharge curve — the cell reaches its cutoff voltage while charge is still stored. SOC% itself is purely Coulomb-based (stored charge) and temperature-independent. Derating is applied to:
- **Batt-TTL calculation** — Trapped Charge: extractable = max(0, remaining − capacity × (1−f(T)))
- **CLI display** — `get board.telem` shows the derated value in parentheses: `SOC:95.0% (78%)`

The derating model uses a per-chemistry linear function:

```
f(T) = max(f_min, 1.0 - k × (T_ref - T))    for T < T_ref
f(T) = 1.0                                    for T >= T_ref
```

T_ref = 25 °C for all chemistries (no derating at room temperature and above).

| Chemistry | k (/°C) | f_min | At −20 °C | At −10 °C | At 0 °C | At 10 °C |
|-----------|---------|-------|-----------|-----------|---------|---------|
| Li-ion | 0.005 | 0.75 | 0.78 | 0.83 | 0.88 | 0.93 |
| LiFePO4 | 0.006 | 0.70 | 0.73 | 0.79 | 0.85 | 0.91 |
| Na-ion | 0.003 | 0.85 | 0.87 | 0.90 | 0.93 | 0.96 |
| LTO | 0.002 | 0.88 | 0.91 | 0.93 | 0.95 | 0.97 |

**Practical effects:**
- SOC% is temperature-independent — only changes with actual charge flow
- The derated (extractable) value in `telem` decreases as the battery cools
- Batt-TTL estimates reflect extractable capacity at the current temperature (Trapped Charge model)

→ See [FAQ #13 — How does temperature derating work?](FAQ.md#13-how-does-temperature-derating-work) for further technical details.

---

## 4. Cell Selection & Form Factors

### Li-ion Cells

| Form Factor | Typical Capacity | Notes |
|---|---|---|
| **18650** | 2500–3500 mAh | Most common; widely available; easy to source |
| **21700** | 4000–5000 mAh | Higher capacity; becoming the new standard |
| **Pouch** | Varies | Custom shapes; requires careful mounting |

**Tips:**
- Prefer cells with built-in protection circuit (PCM) for standalone use
- For parallel packs, ensure cells are matched (same manufacturer, same batch)
- Cells with integrated NTC simplify wiring to the TS pin
- Recommended: Samsung, Sony/Murata, LG, Panasonic/Sanyo — avoid no-name cells

### LiFePO4 Cells

| Form Factor | Typical Capacity | Notes |
|---|---|---|
| **18650** | 1400–1800 mAh | Lower capacity than Li-ion 18650; less common |
| **26650** | 2500–3600 mAh | Larger diameter; popular for LFP |
| **32650** | 5000–6000 mAh | Large cylindrical; good for high-capacity packs |
| **Prismatic** | 5000–50 000 mAh | Flat cells; efficient use of space |

**Tips:**
- The flat discharge curve means voltage tells you little about SOC — rely on the Inhero MR2's coulomb counter
- Avoid charging below 0 °C — the board's JEITA protection handles this automatically
- EVE, BYD, CATL are reputable manufacturers

### LTO Cells

| Form Factor | Typical Capacity | Notes |
|---|---|---|
| **Cylindrical (screw terminal)** | 10 000–40 000 mAh | Most common LTO format; M6/M8 screw terminals |
| **Prismatic (aluminum)** | 10 000–30 000 mAh | Aluminum housing; cannot be spot-welded easily |

**⚠️ Important: 2S requires a balancer**

The Inhero MR2 configures the BQ25798 for 2S operation but provides **no built-in cell balancing**. An external balancer module is required to prevent cell voltage drift over time. Without a balancer, one cell may be overcharged while the other is undercharged — this degrades capacity and can damage cells.

**Tips:**
- Use a passive or active balancer board rated for your cell voltage range (2.0–2.7 V per cell)
- Yinlong/Toshiba SCiB are common LTO cell brands
- Expect 2–3× the volume and weight compared to Li-ion for the same energy
- Screw terminals are robust for outdoor deployments — no spot-welding needed

### Na-ion Cells

| Form Factor | Typical Capacity | Notes |
|---|---|---|
| **18650** | 1000–1500 mAh | Emerging; first-generation cells |
| **26700** | 3000–3500 mAh | Layered-oxide consumer cells (e.g. HAKADI) |
| **32140 / 33140** | 8000–10 000 mAh | Industrial cylindrical cells (HiNa, AuroraCell) |
| **Prismatic** | 5000–20 000 mAh | Larger formats appearing from HiNa, CATL, Faradion |

**Check the charge temperature window in the datasheet before buying.** Na-ion cells differ: HiNa NaCR32140 charges down to −20 °C (0.1C, 3.8 V, max 75 % SOC below −10 °C), AuroraCell 32140 to −20 °C, HAKADI 26700 and many 18650 cells to −10 °C, NFPP cells and several budget cells only from 0 °C upward. The MR2 runs Na-ion without a temperature guard, so the cell's window is the only limit in effect — pick a cell whose window covers the coldest charging temperature at the site. Background: [CUK-SIB on cold-climate use](https://www.cuk-sib.com/de/blog/sodium-ion-batteries-for-cold-climate-applications).

**Tips:**
- Technology is evolving rapidly — check latest available cells before purchasing
- Can be shipped at 0 V (unlike all lithium chemistries) — simplifies logistics
- No special handling required for storage
- HiNa, CATL, Faradion/Reliance are key manufacturers (as of 2025/2026)

---

## 5. Capacity Planning

### Power Consumption

Typical Inhero MR2 power consumption (repeater mode, LEDs off):

| Condition | Current Draw | Notes |
|---|---|---|
| Idle (RX, no TX) | ~7.6 mA @ 3.3 V | USB off, SX1262 in RX |
| **Measured typical** | **~12.3 mA @ 3.3 V** | 24h measurement, repeater with typical traffic |
| **Worst case (10% DC, EU868 g3)** | **~19.8 mA @ 3.3 V** | 10% duty cycle × ~130 mA TX + 90% × 7.6 mA idle |
| TX burst (SX1262 +22 dBm) | ~130 mA @ 3.3 V | Short bursts only, regulated by duty cycle |
| Low-voltage sleep | <0.5 mA | Solar charging continues |

> **How these values are determined:** The worst case assumes the EU868 **g3 sub-band** (869.4–869.65 MHz), which permits up to +27 dBm ERP and a **10% duty cycle** — MeshCore's current default channel of 869.618 MHz sits inside it. The frequency is a firmware setting; on a band with a tighter duty cycle the worst case drops accordingly. At +22 dBm the SX1262 + MCU draw ~130 mA during TX. With 10% TX time: `0.90 × 7.6 + 0.10 × 130 = 19.8 mA` → **~65 mW or ~1.57 Wh/day** — the regulatory maximum.
>
> **Measured typical (~12.3 mA):** Validated 24h measurement in repeater mode with typical traffic: **295 mAh/day @ 3.32 V** = 0.98 Wh/day → **~41 mW or ~0.98 Wh/day**.

### Why current depends on battery voltage

The Inhero MR2 uses a high-efficiency **buck converter** to produce the 3.3 V rail that powers the MCU and radio. This means the board draws roughly **constant power** (watts), not constant current (amps).

Since Power = Voltage × Current, a higher battery voltage means lower current from the battery — but the power stays the same:

| Chemistry | Nominal Voltage | Idle | Measured typical | Worst case (10% DC) |
|---|---|---|---|---|
| Na-ion | 3.1 V | ~8.1 mA (25 mW) | ~13.2 mA (41 mW) | ~21.0 mA (65 mW) |
| LiFePO4 | 3.2 V | ~7.8 mA (25 mW) | ~12.8 mA (41 mW) | ~20.3 mA (65 mW) |
| Li-ion | 3.7 V | ~6.8 mA (25 mW) | ~11.1 mA (41 mW) | ~17.6 mA (65 mW) |
| LTO (2S) | 4.6 V | ~5.4 mA (25 mW) | ~8.9 mA (41 mW) | ~14.1 mA (65 mW) |

> **Important for capacity planning:** Don't just multiply mA × hours to get mAh — that only works within one chemistry at one voltage. When comparing across chemistries, always calculate in **Wh** (energy): `Energy (Wh) = Wh/day × Days`. Then convert to your chemistry: `mAh = Wh × 1000 ÷ V_nominal`. Use **0.98 Wh/day** (measured typical) or **1.57 Wh/day** (worst case 10% DC) depending on expected traffic and safety margin.
>
> **Example:** 30 days autonomy at measured typical = 29 Wh needed (worst case: 47 Wh).
> - LiFePO4 (3.2 V): 29 000 ÷ 3.2 = **9 063 mAh** (worst case: 14 688 mAh)
> - LTO 2S (4.6 V): 29 000 ÷ 4.6 = **6 304 mAh** (worst case: 10 217 mAh)

### Sizing for Autonomy

**Energy approach (recommended):** `Energy (Wh) = Wh/day × Days of autonomy` — use **0.98 Wh/day** (measured typical) or **1.57 Wh/day** (worst case 10% DC) for conservative sizing

**Convert to mAh for your chemistry:** `mAh = Wh × 1000 ÷ V_nominal`

The mAh column below is calculated at 3.3 V (≈ LiFePO4 / Na-ion nominal). For Li-ion or LTO, use the energy column with the formula above — see [Why current depends on battery voltage](#why-current-depends-on-battery-voltage).

| Desired Autonomy | Measured typical (0.98 Wh/day) | Worst case (1.57 Wh/day) | mAh @ 3.3 V (typical) | Recommended |
|---|---|---|---|---|
| **3 days** (indoor, grid backup) | 2.9 Wh | 4.7 Wh | 891 mAh | 1500 mAh |
| **7 days** (solar, summer) | 6.9 Wh | 11.0 Wh | 2079 mAh | 3500 mAh |
| **14 days** (solar, winter) | 13.7 Wh | 22.0 Wh | 4158 mAh | 6000 mAh |
| **30 days** (alpine, minimal solar) | 29.4 Wh | 47.1 Wh | 8909 mAh | 12000+ mAh |

> **Cold-weather margin:** For deployments below 0 °C, increase capacity by the inverse of the derating factor to compensate for reduced extractable capacity. Example: LiFePO4 at −10 °C has f(T) = 0.79, so you need `capacity / 0.79 ≈ 1.27×` the capacity compared to room temperature. The Batt-TTL calculation applies this derating automatically.

### The 90% Rule

Set `board.batcap` to **90% of nominal capacity**. The Inhero MR2 uses conservative charge voltages (e.g. 4.1 V instead of 4.2 V for Li-ion), which means the top ~10% of nominal capacity is intentionally not used — this significantly improves cycle life.

**Example:** 10 000 mAh nominal → `set board.batcap 9000`

→ See [FAQ #4](FAQ.md#4-what-mah-value-should-i-enter-for-set-boardbatcap)

---

## 6. Solar Charging Considerations

**Maximum charge current formula:** `I_charge (mA) = Panel power (W) / Nominal battery voltage (V)`

| Chemistry | Panel | Charge Current | `set board.imax` |
|---|---|---|---|
| Li-ion (3.7 V) | 2 W | 540 mA | `set board.imax 540` |
| LiFePO4 (3.2 V) | 1 W | 310 mA | `set board.imax 310` |
| LTO (4.6 V) | 5 W | 1090 mA | `set board.imax 1090` |
| Na-ion (3.1 V) | 3 W | 970 mA | `set board.imax 970` |

**Panel sizing guidelines:**
- The Inhero MR2 consumes ~7.6 mA @ 3.3 V idle, **measured ~12.3 mA typical** (~0.98 Wh/day), worst case ~19.8 mA at full EU868 g3 10% duty cycle (~1.57 Wh/day)
- With vertical south-facing mounting (see below), **2 W monocrystalline is safely sufficient** for winter autonomy in central Europe with a 9 Ah LiFePO4 battery. Field-tested: even a 1 W panel (vertical, south, unshaded, exposed) with 9 Ah LiFePO4 survived a full central European winter
- For locations with frequent overcast periods or partial shading, add margin — 3–5 W recommended
- MPPT is essential for extracting maximum power — enable with `set board.mppt 1`

**Panel orientation — vertical is better for autonomy:**

Conventional PV installations tilt panels at ~30–40° to maximize annual yield. For off-grid repeaters, the goal is different: **maximize winter performance**, especially in the critical months of December and January when the sun is lowest and days are shortest. Mounting panels **vertically (90°)** has significant advantages:

- **Low winter sun** hits a vertical panel at near-optimal angle while a 30°-tilted panel receives the same light at a glancing angle
- **Self-cleaning:** vertical panels shed snow, ice, and dirt far more effectively — a snow-covered panel produces zero power regardless of rated wattage
- **Practical rule of thumb (central Europe):** expect approximately **1 Wh/day per 1 Wp** of panel rating in January with a vertical, south-facing, unshaded, exposed setup. PVGIS data confirms ~36 Wh/month (after system losses) for a 1 Wp panel in this configuration. In practice, the MR2 cannot harvest on very overcast days when the charger reports !PG (power not good), so a conservative estimate is **~30 Wh/month or ~1 Wh/day** usable. With ~0.98 Wh/day measured typical consumption, a 1 Wp panel provides a positive energy balance in January. **2 Wp provides comfortable headroom** for cloudy stretches and higher-traffic deployments
- In summer, vertical panels produce less than optimally tilted ones — but summer yield is never the bottleneck for autonomy

**Chemistry-specific considerations:**
- **Li-ion / LiFePO4:** Solar charging is blocked in frost (<−2 °C) unless `set board.jeitaignore 1` is armed, at the operator's own risk. On cold winter days, the panel may produce power but the battery won't accept charge until it warms above −2 °C. Meanwhile, the board runs directly on solar if power is sufficient. Note that PV panels produce **more power in cold weather** (silicon temperature coefficient ~−0.35%/°C), so actual charge currents can exceed nominal ratings. This is why the block sits in hardware, and why the `jeitaignore` override is gated on the configured charge current — see [Charging in Cold Conditions](#charging-in-cold-conditions).
- **LTO:** Solar charging works even in deep frost — a significant advantage for alpine deployments where frost can persist for days or weeks.
- **Na-ion:** The board does not block charging in frost; whether the cell may be charged there is in its datasheet (0 °C to −20 °C depending on the cell). With a −20 °C-rated cell the alpine advantage applies; with a 0 °C cell it does not — see [Na-ion Cells](#na-ion-cells).

---

## 7. Safety & Protection

| Chemistry | Thermal Runaway | Requires BMS/Protection? | Inhero MR2 Protection |
|---|---|---|---|
| **Li-ion** | ⚠️ Yes — fire/explosion risk under abuse | Yes — mandatory | JEITA, low-V sleep, OVP, charge voltage limit |
| **LiFePO4** | ✅ No — inherently safe | Recommended | JEITA, low-V sleep, OVP |
| **LTO** | ✅ No — inherently safe | Recommended (balancer!) | Low-V sleep, OVP, cell count config |
| **Na-ion** | ✅ No under normal conditions | Recommended | Low-V sleep, OVP |

**Inhero MR2 built-in safety features (all chemistries):**
- **Low-voltage sleep** — INA228 ALERT ISR triggers system sleep to prevent deep discharge
- **Charge voltage limit** — BQ25798 configured per chemistry to prevent overcharge
- **VBAT_OVP** — Hardware overvoltage protection in BQ25798
- **200 mV hysteresis** — Prevents motorboating (rapid on/off cycling) near empty
- **JEITA temperature protection** (Li-ion/LiFePO4 only) — Hardware charge control via NTC; `set board.jeitaignore 1` switches it off within the 0.05C gate, at the operator's own risk ([details](#charging-in-cold-conditions))

**User responsibilities:**
- Li-ion: Use cells with protection circuit (PCM) or a proper BMS
- LTO 2S: **External balancer is mandatory** for long-term operation
- All chemistries: Set correct chemistry via `set board.bat` — wrong chemistry = wrong voltages = damage risk

---

## 8. Deployment Recommendations

| Scenario | Recommended | Alternative | Notes |
|---|---|---|---|
| **Indoor, moderate climate (0–40 °C)** | **LiFePO4** | Li-ion | LiFePO4: best safety + cycle life balance |
| **Outdoor, temperate (−5 to +35 °C)** | **LiFePO4** | Li-ion | Frost is rare; fmax handles occasional cold |
| **Space-constrained enclosure** | **Li-ion** | — | Highest energy density; nothing else fits |
| **Alpine, extreme cold (−20 °C and below)** | **LTO** | Na-ion (cell rated for −20 °C charging) | LTO: charges in frost, 82% capacity at −20 °C |
| **Cold climate, moderate frost (−10 to −15 °C)** | **LTO** or **Na-ion** (cell rated for the site's minimum charging temperature) | LiFePO4 (with margin) | Na-ion balances density and cold — check the cell datasheet's charge window |
| **Maximum service life (>10 years)** | **LTO** | LiFePO4 | LTO: 10 000+ cycles; solar repeater essentially unlimited |
| **Sustainability / ethical sourcing** | **Na-ion** | LiFePO4 | No cobalt, no lithium; improving rapidly |
| **Maritime / coastal (salt, humidity)** | **LiFePO4** | Li-ion | Sealed prismatic cells; inherent safety in harsh environments |
| **Mobile / portable** | **Li-ion** | LiFePO4 | Weight and volume matter most |
| **Budget-constrained** | **Li-ion** | LiFePO4 | Lowest cost per Wh |

*(Extractable-capacity percentages are datasheet values at 0.2C–0.5C loads — see the note in section 2.)*

> **Winter alpine checklist:**
> 1. Choose LTO, or a Na-ion cell whose datasheet allows charging at the site's minimum temperature
> 2. Oversize battery capacity by 1.5–2× for cold derating
> 3. Oversize solar panel by 3–5× for short winter days
> 4. Enable MPPT (`set board.mppt 1`)
> 5. If using Li-ion/LiFePO4: configure `set board.fmax` and run `set board.tccal`
> 6. Monitor via `get board.stats` and `get board.socdebug`

---

## 9. Long-Term Aging & Cycle Life

| Chemistry | Cycles to 80% | Calendar Aging | Optimal Storage SOC |
|---|---|---|---|
| **Li-ion** | 500–1000 | Moderate (faster at high temp/SOC) | 40–60% at 15–25 °C |
| **LiFePO4** | 2000–5000 | Low | 50% at room temperature |
| **LTO** | 10 000+ | Very low | Any SOC; very tolerant |
| **Na-ion** | 1000–3000 | Low | 0 V (unique — no damage) |

**Solar repeater context:** A well-dimensioned solar repeater does **not** do 1 full cycle per day. The actual profile is shallow micro-cycling with strong seasonal variation:

- **Daily discharge** is only **2–3%** of battery capacity (with proper sizing: ≥ 7 Ah for a sub-1 W load)
- **Daily recharge** adds **10–20%** on sunny days, depending on panel size
- **Winter (Dec–Jan):** The battery slowly drains over multi-day overcast periods — it "carries" the repeater through the dark weeks. SOC may drop to 30–50% before the next sunny spell
- **Summer:** The battery permanently floats between **95–100% SOC**, rarely dropping below 90%

This means the battery experiences perhaps **10–30 equivalent full cycles per year** — not 365.

| Chemistry | Cycles to 80% | Equivalent Full Cycles/Year | Expected Service Life |
|---|---|---|---|
| **Li-ion** | 500–1000 | ~10–30 | **15–50+ years** (cycle-limited) |
| **LiFePO4** | 2000–5000 | ~10–30 | **65+ years** (cycle-limited) |
| **LTO** | 10 000+ | ~10–30 | **effectively unlimited** |
| **Na-ion** | 1000–3000 | ~10–30 | **30–100+ years** (cycle-limited) |

> **The real aging threat is not cycling — it's calendar aging at high SOC and temperature.**
>
> In summer, the battery sits at 95–100% SOC for months in an enclosure that can reach 50 °C in direct sun. For Li-ion, this combination (high SOC + high temperature) is the #1 aging accelerator. This is exactly why the Inhero MR2 uses **conservative charge cutoff voltages** (e.g. 4.1 V instead of 4.2 V for Li-ion) — lower Vco reduces the resting SOC and dramatically slows calendar aging.

**How bad is it? Typical NMC Li-ion calendar capacity loss per year (no cycling, storage only):**

| Temperature | 4.2 V (100% SOC) | 4.1 V (~85% SOC) | Reduction |
|---|---|---|---|
| 25 °C (indoor) | ~3–5%/year | ~1–2%/year | 2–3× slower |
| 40 °C (warm enclosure) | ~8–15%/year | ~3–5%/year | 2–3× slower |
| **50 °C (sun-exposed)** | **~20–30%/year** | **~6–10%/year** | **3× slower** |

> At 4.2 V and 50 °C, a Li-ion cell reaches 80% capacity (end of life) in roughly **3 years**. At 4.1 V and 50 °C, the same cell lasts **8–10 years**. The 100 mV Vco reduction alone buys 2–3× more service life — at the cost of only ~10% less usable capacity.
>
> **Bottom line:** For sun-exposed outdoor deployments, the Vco reduction from 4.2→4.1 V is the single most effective measure to extend Li-ion life. If the enclosure regularly exceeds 40 °C, consider LiFePO4 or LTO instead — these chemistries are largely immune to calendar aging at high SOC.
>
> LiFePO4 and LTO are far more tolerant of sustained high SOC. Na-ion ages moderately. For hot/exposed deployments, prefer LiFePO4 or LTO.

---

## 10. Future Outlook

**Na-ion** is the chemistry to watch. As of 2025/2026:
- Energy density is improving with each generation (target: 160+ Wh/kg)
- Major manufacturers (CATL, BYD, HiNa) are ramping production
- Cell costs are expected to drop below Li-ion within 2–3 years
- Ideal for stationary applications where absolute energy density is less critical

**Solid-state batteries** may appear in the 2028+ timeframe but are unlikely to be relevant for off-grid repeater applications in the near term.

**LTO** remains the gold standard for extreme environments and will likely stay relevant for specialized deployments. Its high cost and low density will continue to limit adoption to cases where cold performance and cycle life are critical.

**LiFePO4** will remain the mainstream choice for the foreseeable future — proven, safe, affordable, and available in many formats.

---

## See Also

- [README.md](README.md) — Overview, feature matrix and diagnostics
- [DATASHEET.md](DATASHEET.md) — Hardware datasheet, pinouts and specifications
- [QUICK_START.md](QUICK_START.md) — Quick start for commissioning and CLI setup
- [CLI_CHEAT_SHEET.md](CLI_CHEAT_SHEET.md) — All board-specific CLI commands at a glance
- [FAQ.md](FAQ.md) — Frequently asked questions
- [POWER_MANAGEMENT.md](POWER_MANAGEMENT.md) — Complete technical documentation
