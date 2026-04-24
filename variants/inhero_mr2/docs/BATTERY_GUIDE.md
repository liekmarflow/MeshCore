# Inhero MR2 — Battery Chemistry Guide

> 🇩🇪 [Deutsche Version](de/BATTERY_GUIDE.md)

## Contents

- [Introduction](#introduction)
- [1. Chemistry Overview](#1-chemistry-overview)
  - [Li-Ion (NMC/NCA, 1S)](#li-ion-nmcnca-1s)
  - [LiFePO4 (LFP, 1S)](#lifepo4-lfp-1s)
  - [LTO (Lithium Titanate, 2S)](#lto-lithium-titanate-2s)
  - [Na-Ion (Sodium Ion, 1S)](#na-ion-sodium-ion-1s)
- [2. Comparison Table](#2-comparison-table)
- [3. Temperature Behavior](#3-temperature-behavior)
  - [Cold Performance Ranking](#cold-performance-ranking)
  - [Charging in Cold Conditions](#charging-in-cold-conditions)
  - [Temperature Derating](#temperature-derating)
- [4. Cell Selection & Form Factors](#4-cell-selection--form-factors)
  - [Li-Ion Cells](#li-ion-cells)
  - [LiFePO4 Cells](#lifepo4-cells)
  - [LTO Cells](#lto-cells)
  - [Na-Ion Cells](#na-ion-cells)
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

Choosing the right battery chemistry is one of the most impactful decisions when deploying an Inhero MR2 repeater. It affects runtime, cold-weather reliability, service life, safety, and total cost of ownership. This guide provides the information needed to make an informed choice.

The Inhero MR2 supports **four battery chemistries**, each with distinct characteristics. There is no single "best" chemistry — the right choice depends on your deployment conditions.

---

## 1. Chemistry Overview

### Li-Ion (NMC/NCA, 1S)

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
- Slightly lower energy density than Li-Ion
- Must not be charged below 0 °C — lithium plating risk damages the cell permanently

**Inhero MR2 specifics:**
- Charge voltage: **3.5 V**
- **JEITA active** — NTC required; charging blocked below −2 °C (T-Cold)
- Frost charge reduction configurable via `set board.fmax`
- JEITA WARM zone neutralized in firmware to prevent VBAT_OVP (see [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md#jeita-warm-zone--vbat_ovp-prevention))

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
- **Low energy density** (~80 Wh/kg) — needs 2–3× the volume of Li-Ion for the same capacity
- **2S configuration requires an external balancer** — without balancer, cells drift over time and risk over-charge/over-discharge of individual cells
- Exotic form factors — typically cylindrical with screw terminals or aluminum housing
- Difficult to spot-weld (aluminum housing)
- Expensive (2–3× per Wh vs. Li-Ion)
- Limited retail availability — specialty suppliers only

**Inhero MR2 specifics:**
- Charge voltage: **5.4 V** (2× 2.7 V per cell)
- **JEITA disabled** (`ts_ignore = true`) — no NTC required for charging
- Temperature for SOC derating comes from **BME280 fallback** if no NTC connected
- `set board.fmax` has no effect (shown as "N/A")
- Cell count set to 2S in BQ25798 configuration

### Na-Ion (Sodium Ion, 1S)

Sodium-ion technology — the sustainable alternative using abundant, non-critical raw materials.

**Strengths:**
- **Good cold performance** — 78% extractable at −20 °C
- **No cobalt, no lithium** — ethically sourced, sustainable materials (sodium, iron, manganese)
- **Can be stored and shipped at 0 V** — no deep discharge damage (unique among all chemistries)
- **Can be charged in frost** — no JEITA restriction needed
- Good safety profile — no thermal runaway under normal conditions
- Rapidly improving technology — energy density and cycle life increase with each generation

**Weaknesses:**
- **New technology** — limited cell availability as of 2025/2026
- Lower energy density than Li-Ion (~130 Wh/kg, improving)
- Fewer validated cell options and public datasheets
- Cycle life still below LiFePO4 in most current cells (1000–3000 cycles)
- Market still maturing — quality variation between manufacturers

**Inhero MR2 specifics:**
- Charge voltage: **3.9 V**
- **JEITA disabled** (`ts_ignore = true`) — no NTC required for charging
- Temperature for SOC derating comes from **BME280 fallback** if no NTC connected
- `set board.fmax` has no effect (shown as "N/A")

---

## 2. Comparison Table

| | Li-Ion 1S | LiFePO4 1S | LTO 2S | Na-Ion 1S |
|---|---|---|---|---|
| **Energy density** | ~250 Wh/kg | ~160 Wh/kg | ~80 Wh/kg | ~130 Wh/kg |
| **Cycle life (to 80%)** | 500–1000 | 2000–5000 | 10 000+ | 1000–3000 |
| **Cold performance** | Moderate | Weakest | Best | Good |
| **Extractable at −20 °C** | 55% | 46% | 82% | 78% |
| **Extractable at −10 °C** | 65% | 58% | 86% | 83% |
| **Extractable at 0 °C** | 75% | 70% | 90% | 88% |
| **Thermal runaway risk** | Yes | No | No | No |
| **NTC required?** | Yes | Yes | No | No |
| **JEITA** | Active | Active | Disabled | Disabled |
| **Charge in frost?** | No (blocked <−2 °C) | No (blocked <−2 °C) | Yes | Yes |
| **Charge voltage** | 4.1 V | 3.5 V | 5.4 V (2S) | 3.9 V |
| **Low-V sleep** | 3100 mV | 2700 mV | 3900 mV | 2500 mV |
| **Low-V wake** | 3300 mV | 2900 mV | 4100 mV | 2700 mV |
| **Nominal voltage** | 3.7 V | 3.2 V | 4.6 V | 3.1 V |
| **Cell formats** | 18650, 21700, pouch | 18650, 26650, prismatic | Screw-terminal, aluminum | 18650, prismatic |
| **Availability** | Excellent | Good | Limited | Limited |
| **Relative cost (per Wh)** | Low | Low–Medium | High | Medium |

---

## 3. Temperature Behavior

### Cold Performance Ranking

From best to worst cold-weather performance:

1. **LTO** — 82% extractable at −20 °C, charges in frost
2. **Na-Ion** — 78% extractable at −20 °C, charges in frost
3. **Li-Ion** — 55% extractable at −20 °C, charging blocked in frost
4. **LiFePO4** — 46% extractable at −20 °C, charging blocked in frost

> The ranking may surprise users familiar with LiFePO4's reputation as a "workhorse." While LiFePO4 excels in cycle life and safety, it is actually the **worst performer in cold** among the four supported chemistries. This matters significantly for alpine and winter deployments.

### Charging in Cold Conditions

| Chemistry | Charging in frost? | Mechanism |
|---|---|---|
| **Li-Ion** | No — blocked below −2 °C | JEITA T-Cold (hardware, BQ25798) |
| **LiFePO4** | No — blocked below −2 °C | JEITA T-Cold (hardware, BQ25798) |
| **LTO** | Yes — charges at any temperature | JEITA disabled (`ts_ignore = true`) |
| **Na-Ion** | Yes — charges at any temperature | JEITA disabled (`ts_ignore = true`) |

For Li-Ion and LiFePO4, charging is also reduced in the **T-Cool range** (+3 °C to −2 °C with Inhero voltage divider), configurable via `set board.fmax`. See [FAQ #6](FAQ.md#6-what-does-set-boardfmax-control).

**Why is frost charging dangerous for Li-Ion and LiFePO4?** At low temperatures, lithium ions cannot intercalate properly into the graphite anode. Instead, they deposit as metallic lithium on the anode surface ("lithium plating"). This permanently reduces capacity and can create internal short circuits — a safety hazard.

LTO and Na-Ion use different anode materials (lithium titanate and hard carbon respectively) that do not suffer from lithium plating, making frost charging safe.

> **Field experience vs. theory:** Many repeater operators successfully charge Li-Ion cells in frost with low solar currents (<0.1C) and report no measurable degradation over multiple winters. The [YYCMesh community](https://yycmesh.com/blog/cold-weather-charging) documented two years of alpine deployments in the Canadian Rockies (down to −40 °C) with standard 18650 cells and found internal resistance still within factory spec. Their key factors: very low charge rates (<0.05C), passive solar heating of enclosures, and charging coinciding with the warmest part of the day.
>
> This is valuable real-world data and the practice clearly works for many setups. However, these results apply specifically to configurations with **large battery capacities and relatively low PV power** (keeping charge rates well below 0.05C). They should not be taken as a general dismissal of lithium plating risks. *It depends* — on charge rate, cell quality, panel size, and temperature. The degradation from lithium plating is **cumulative and subtle** — it may not manifest as sudden failure but as gradual capacity loss over years. Two additional risks are often underestimated:
>
> 1. **PV panels produce more power in cold weather** (silicon temperature coefficient ~−0.35%/°C). A 5 W panel at −10 °C delivers significantly more current than at +25 °C. Snow reflection can push output even beyond rated wattage.
> 2. **Cell quality varies.** Results with premium cells (low internal resistance, consistent chemistry) may not transfer to budget cells.
>
> The Inhero MR2 takes a conservative approach: the NTC battery temperature sensor causes the BQ25798 to block charging until the cell warms above −2 °C (JEITA T-Cold), and then charges at a reduced rate until the battery leaves the T-Cool zone (configurable via `set board.fmax`). On sunny winter days, the board runs from solar via the power path while the battery stays protected. Once direct sunlight heats up the enclosure and the battery temperature rises above the threshold — which happens surprisingly fast with proper enclosure design — charging resumes automatically. This gives the best of both worlds: no plating risk, yet minimal lost charge time.

### Temperature Derating

The firmware uses a **Trapped Charge** model to estimate **extractable** capacity at the current battery temperature. Cold temperatures lock the bottom of the discharge curve — the cell reaches its cutoff voltage while charge is still stored. SOC% itself is purely Coulomb-based (stored charge) and temperature-independent. Derating is applied to:
- **TTL calculation** — Trapped Charge: extractable = max(0, remaining − capacity × (1−f(T)))
- **CLI display** — `get board.telem` shows the derated value in parentheses: `SOC:95.0% (78%)`

The derating model uses a per-chemistry linear function:

```
f(T) = max(f_min, 1.0 - k × (T_ref - T))    for T < T_ref
f(T) = 1.0                                    for T >= T_ref
```

T_ref = 25 °C for all chemistries (no derating at room temperature and above).

| Chemistry | k (/°C) | f_min | At −20 °C | At −10 °C | At 0 °C | At 10 °C |
|-----------|---------|-------|-----------|-----------|---------|---------|
| Li-Ion | 0.005 | 0.75 | 0.78 | 0.83 | 0.88 | 0.93 |
| LiFePO4 | 0.006 | 0.70 | 0.73 | 0.79 | 0.85 | 0.91 |
| Na-Ion | 0.003 | 0.85 | 0.87 | 0.90 | 0.93 | 0.96 |
| LTO | 0.002 | 0.88 | 0.91 | 0.93 | 0.95 | 0.97 |

**Practical effects:**
- SOC% is temperature-independent — only changes with actual charge flow
- The derated (extractable) value in `telem` decreases as the battery cools
- TTL estimates reflect extractable capacity at the current temperature (Trapped Charge model)

→ See [IMPLEMENTATION_SUMMARY.md — §5a Temperature Derating](IMPLEMENTATION_SUMMARY.md#5a-temperature-derating) for the full technical implementation.

---

## 4. Cell Selection & Form Factors

### Li-Ion Cells

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
| **18650** | 1400–1800 mAh | Lower capacity than Li-Ion 18650; less common |
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
- Expect 2–3× the volume and weight compared to Li-Ion for the same energy
- Screw terminals are robust for outdoor deployments — no spot-welding needed

### Na-Ion Cells

| Form Factor | Typical Capacity | Notes |
|---|---|---|
| **18650** | 1000–1500 mAh | Emerging; first-generation cells |
| **Prismatic** | 5000–20 000 mAh | Larger formats appearing from HiNa, CATL, Faradion |

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

> **How these values are determined:** MeshCore operates on **869.618 MHz** in the EU868 **g3 sub-band** (869.4–869.65 MHz), which allows up to +27 dBm ERP and a **10% duty cycle**. At +22 dBm the SX1262 + MCU draw ~130 mA during TX. With 10% TX time: `0.90 × 7.6 + 0.10 × 130 = 19.8 mA` → **~65 mW or ~1.57 Wh/day** — the regulatory maximum.
>
> **Measured typical (~12.3 mA):** Validated 24h measurement in repeater mode with typical traffic: **295 mAh/day @ 3.32 V** = 0.98 Wh/day → **~41 mW or ~0.98 Wh/day**.

### Why current depends on battery voltage

The Inhero MR2 uses a high-efficiency **buck converter** to produce the 3.3 V rail that powers the MCU and radio. This means the board draws roughly **constant power** (watts), not constant current (amps).

Since Power = Voltage × Current, a higher battery voltage means lower current from the battery — but the power stays the same:

| Chemistry | Nominal Voltage | Idle | Measured typical | Worst case (10% DC) |
|---|---|---|---|---|
| Na-Ion | 3.1 V | ~8.1 mA (25 mW) | ~13.2 mA (41 mW) | ~21.0 mA (65 mW) |
| LiFePO4 | 3.2 V | ~7.8 mA (25 mW) | ~12.8 mA (41 mW) | ~20.3 mA (65 mW) |
| Li-Ion | 3.7 V | ~6.8 mA (25 mW) | ~11.1 mA (41 mW) | ~17.6 mA (65 mW) |
| LTO (2S) | 4.6 V | ~5.4 mA (25 mW) | ~8.9 mA (41 mW) | ~14.1 mA (65 mW) |

> **Important for capacity planning:** Don't just multiply mA × hours to get mAh — that only works within one chemistry at one voltage. When comparing across chemistries, always calculate in **Wh** (energy): `Energy (Wh) = Wh/day × Days`. Then convert to your chemistry: `mAh = Wh × 1000 ÷ V_nominal`. Use **0.98 Wh/day** (measured typical) or **1.57 Wh/day** (worst case 10% DC) depending on expected traffic and safety margin.
>
> **Example:** 30 days autonomy at measured typical = 29 Wh needed (worst case: 47 Wh).
> - LiFePO4 (3.2 V): 29 000 ÷ 3.2 = **9 063 mAh** (worst case: 14 688 mAh)
> - LTO 2S (4.6 V): 29 000 ÷ 4.6 = **6 304 mAh** (worst case: 10 217 mAh)

### Sizing for Autonomy

**Energy approach (recommended):** `Energy (Wh) = Wh/day × Days of autonomy` — use **0.98 Wh/day** (measured typical) or **1.57 Wh/day** (worst case 10% DC) for conservative sizing

**Convert to mAh for your chemistry:** `mAh = Wh × 1000 ÷ V_nominal`

The mAh column below is calculated at 3.3 V (≈ LiFePO4 / Na-Ion nominal). For Li-Ion or LTO, use the energy column with the formula above — see [Why current depends on battery voltage](#why-current-depends-on-battery-voltage).

| Desired Autonomy | Measured typical (0.98 Wh/day) | Worst case (1.57 Wh/day) | mAh @ 3.3 V (typical) | Recommended |
|---|---|---|---|---|
| **3 days** (indoor, grid backup) | 2.9 Wh | 4.7 Wh | 891 mAh | 1500 mAh |
| **7 days** (solar, summer) | 6.9 Wh | 11.0 Wh | 2079 mAh | 3500 mAh |
| **14 days** (solar, winter) | 13.7 Wh | 22.0 Wh | 4158 mAh | 6000 mAh |
| **30 days** (alpine, minimal solar) | 29.4 Wh | 47.1 Wh | 8909 mAh | 12000+ mAh |

> **Cold-weather margin:** For deployments below 0 °C, increase capacity by the inverse of the derating factor to compensate for reduced extractable capacity. Example: LiFePO4 at −10 °C has f(T) = 0.79, so you need `capacity / 0.79 ≈ 1.27×` the capacity compared to room temperature. The TTL calculation applies this derating automatically.

### The 90% Rule

Set `board.batcap` to **90% of nominal capacity**. The Inhero MR2 uses conservative charge voltages (e.g. 4.1 V instead of 4.2 V for Li-Ion), which means the top ~10% of nominal capacity is intentionally not used — this significantly improves cycle life.

**Example:** 10 000 mAh nominal → `set board.batcap 9000`

→ See [FAQ #4](FAQ.md#4-what-mah-value-should-i-enter-for-set-boardbatcap)

---

## 6. Solar Charging Considerations

**Maximum charge current formula:** `I_charge (mA) = Panel power (W) / Nominal battery voltage (V)`

| Chemistry | Panel | Charge Current | `set board.imax` |
|---|---|---|---|
| Li-Ion (3.7 V) | 2 W | 540 mA | `set board.imax 540` |
| LiFePO4 (3.2 V) | 1 W | 310 mA | `set board.imax 310` |
| LTO (4.6 V) | 5 W | 1090 mA | `set board.imax 1090` |
| Na-Ion (3.1 V) | 3 W | 970 mA | `set board.imax 970` |

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
- **Li-Ion / LiFePO4:** Solar charging is blocked in frost (<−2 °C). On cold winter days, the panel may produce power but the battery won't accept charge until it warms above −2 °C. Meanwhile, the board runs directly on solar if power is sufficient. Note that PV panels produce **more power in cold weather** (silicon temperature coefficient ~−0.35%/°C), so actual charge currents can exceed nominal ratings — another reason why hardware-level charge blocking via JEITA is essential rather than relying on "low current" assumptions.
- **LTO / Na-Ion:** Solar charging works even in deep frost — a significant advantage for alpine deployments where frost can persist for days or weeks.

---

## 7. Safety & Protection

| Chemistry | Thermal Runaway | Requires BMS/Protection? | Inhero MR2 Protection |
|---|---|---|---|
| **Li-Ion** | ⚠️ Yes — fire/explosion risk under abuse | Yes — mandatory | JEITA, low-V sleep, OVP, charge voltage limit |
| **LiFePO4** | ✅ No — inherently safe | Recommended | JEITA, low-V sleep, OVP |
| **LTO** | ✅ No — inherently safe | Recommended (balancer!) | Low-V sleep, OVP, cell count config |
| **Na-Ion** | ✅ No under normal conditions | Recommended | Low-V sleep, OVP |

**Inhero MR2 built-in safety features (all chemistries):**
- **Low-voltage sleep** — INA228 ALERT ISR triggers system sleep to prevent deep discharge
- **Charge voltage limit** — BQ25798 configured per chemistry to prevent overcharge
- **VBAT_OVP** — Hardware overvoltage protection in BQ25798
- **200 mV hysteresis** — Prevents motorboating (rapid on/off cycling) near empty
- **JEITA temperature protection** (Li-Ion/LiFePO4 only) — Hardware charge control via NTC

**User responsibilities:**
- Li-Ion: Use cells with protection circuit (PCM) or a proper BMS
- LTO 2S: **External balancer is mandatory** for long-term operation
- All chemistries: Set correct chemistry via `set board.bat` — wrong chemistry = wrong voltages = damage risk

---

## 8. Deployment Recommendations

| Scenario | Recommended | Alternative | Notes |
|---|---|---|---|
| **Indoor, moderate climate (0–40 °C)** | **LiFePO4** | Li-Ion | LiFePO4: best safety + cycle life balance |
| **Outdoor, temperate (−5 to +35 °C)** | **LiFePO4** | Li-Ion | Frost is rare; fmax handles occasional cold |
| **Space-constrained enclosure** | **Li-Ion** | — | Highest energy density; nothing else fits |
| **Alpine, extreme cold (−20 °C and below)** | **LTO** | Na-Ion | LTO: charges in frost, 82% capacity at −20 °C |
| **Cold climate, moderate frost (−10 to −15 °C)** | **Na-Ion** or **LTO** | LiFePO4 (with margin) | Both charge in frost; Na-Ion balances density and cold |
| **Maximum service life (>10 years)** | **LTO** | LiFePO4 | LTO: 10 000+ cycles; solar repeater essentially unlimited |
| **Sustainability / ethical sourcing** | **Na-Ion** | LiFePO4 | No cobalt, no lithium; improving rapidly |
| **Maritime / coastal (salt, humidity)** | **LiFePO4** | Li-Ion | Sealed prismatic cells; inherent safety in harsh environments |
| **Mobile / portable** | **Li-Ion** | LiFePO4 | Weight and volume matter most |
| **Budget-constrained** | **Li-Ion** | LiFePO4 | Lowest cost per Wh |

> **Winter alpine checklist:**
> 1. Choose LTO or Na-Ion for frost charging
> 2. Oversize battery capacity by 1.5–2× for cold derating
> 3. Oversize solar panel by 3–5× for short winter days
> 4. Enable MPPT (`set board.mppt 1`)
> 5. If using Li-Ion/LiFePO4: configure `set board.fmax` and run `set board.tccal`
> 6. Monitor via `get board.stats` and `get board.socdebug`

---

## 9. Long-Term Aging & Cycle Life

| Chemistry | Cycles to 80% | Calendar Aging | Optimal Storage SOC |
|---|---|---|---|
| **Li-Ion** | 500–1000 | Moderate (faster at high temp/SOC) | 40–60% at 15–25 °C |
| **LiFePO4** | 2000–5000 | Low | 50% at room temperature |
| **LTO** | 10 000+ | Very low | Any SOC; very tolerant |
| **Na-Ion** | 1000–3000 | Low | 0 V (unique — no damage) |

**Solar repeater context:** A well-dimensioned solar repeater does **not** do 1 full cycle per day. The actual profile is shallow micro-cycling with strong seasonal variation:

- **Daily discharge** is only **2–3%** of battery capacity (with proper sizing: ≥ 7 Ah for a sub-1 W load)
- **Daily recharge** adds **10–20%** on sunny days, depending on panel size
- **Winter (Dec–Jan):** The battery slowly drains over multi-day overcast periods — it "carries" the repeater through the dark weeks. SOC may drop to 30–50% before the next sunny spell
- **Summer:** The battery permanently floats between **95–100% SOC**, rarely dropping below 90%

This means the battery experiences perhaps **10–30 equivalent full cycles per year** — not 365.

| Chemistry | Cycles to 80% | Equivalent Full Cycles/Year | Expected Service Life |
|---|---|---|---|
| **Li-Ion** | 500–1000 | ~10–30 | **15–50+ years** (cycle-limited) |
| **LiFePO4** | 2000–5000 | ~10–30 | **65+ years** (cycle-limited) |
| **LTO** | 10 000+ | ~10–30 | **effectively unlimited** |
| **Na-Ion** | 1000–3000 | ~10–30 | **30–100+ years** (cycle-limited) |

> **The real aging threat is not cycling — it's calendar aging at high SOC and temperature.**
>
> In summer, the battery sits at 95–100% SOC for months in an enclosure that can reach 50 °C in direct sun. For Li-Ion, this combination (high SOC + high temperature) is the #1 aging accelerator. This is exactly why the Inhero MR2 uses **conservative charge cutoff voltages** (e.g. 4.1 V instead of 4.2 V for Li-Ion) — lower Vco reduces the resting SOC and dramatically slows calendar aging.

**How bad is it? Typical NMC Li-Ion calendar capacity loss per year (no cycling, storage only):**

| Temperature | 4.2 V (100% SOC) | 4.1 V (~85% SOC) | Reduction |
|---|---|---|---|
| 25 °C (indoor) | ~3–5%/year | ~1–2%/year | 2–3× slower |
| 40 °C (warm enclosure) | ~8–15%/year | ~3–5%/year | 2–3× slower |
| **50 °C (sun-exposed)** | **~20–30%/year** | **~6–10%/year** | **3× slower** |

> At 4.2 V and 50 °C, a Li-Ion cell reaches 80% capacity (end of life) in roughly **3 years**. At 4.1 V and 50 °C, the same cell lasts **8–10 years**. The 100 mV Vco reduction alone buys 2–3× more service life — at the cost of only ~10% less usable capacity.
>
> **Bottom line:** For sun-exposed outdoor deployments, the Vco reduction from 4.2→4.1 V is the single most effective measure to extend Li-Ion life. If the enclosure regularly exceeds 40 °C, consider LiFePO4 or LTO instead — these chemistries are largely immune to calendar aging at high SOC.
>
> LiFePO4 and LTO are far more tolerant of sustained high SOC. Na-Ion ages moderately. For hot/exposed deployments, prefer LiFePO4 or LTO.

---

## 10. Future Outlook

**Na-Ion** is the chemistry to watch. As of 2025/2026:
- Energy density is improving with each generation (target: 160+ Wh/kg)
- Major manufacturers (CATL, BYD, HiNa) are ramping production
- Cell costs are expected to drop below Li-Ion within 2–3 years
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
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) — Complete technical documentation
