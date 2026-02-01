# Varianten-Trennung: MR-1 (v0.1) und MR-2 (v0.2)

## Übersicht der Änderungen

Die Hardware-Varianten wurden sauber getrennt:

### Inhero MR-1 (v0.1 - Legacy)
**Pfad:** `variants/inhero_mr1/`

**Hardware:**
- MCP4652 Digital Potentiometer (I2C 0x2F)
- TP2120 UVLO Protection (analog)
- BQ25798 Battery Charger
- Keine RTC
- Kein Coulomb Counter

**Features:**
- Basis Power Management
- MPPT Solar Charging
- Voltage-based SOC Estimation
- Hardware-Detection entfernt (immer v0.1)

**Dateien:**
- `InheroMr1Board.h/cpp` - Vereinfacht, v0.2 Code entfernt
- `BoardConfigContainer.h/cpp` - Wie gehabt
- `platformio.ini` - Build für MR-1
- Board: `boards/inhero_mr1.json`

---

### Inhero MR-2 (v0.2 - Current)
**Pfad:** `variants/inhero_mr2/`

**Hardware:**
- INA228 Power Monitor (I2C 0x45)
- RV-3028-C7 RTC (I2C 0x52)
- TPS62840 Buck mit Hardware-UVLO
- BQ25798 Battery Charger
- 20mΩ Shunt für Coulomb Counter

**Features:**
- Advanced Power Management
  - 3-Layer Protection (Software + RTC + Hardware UVLO)
  - Coulomb Counter für Real-time SOC
  - 7-day Energy Balance Analysis
  - RTC Wake-up Management
  - Motorboating Prevention
- MPPT Solar Charging
- Adaptive Voltage Monitoring (10s/30s/60s)

**Dateien:**
- `InheroMr2Board.h/cpp` - Vollständiges v0.2 Power Management
- `BoardConfigContainer.h/cpp` - Mit INA228 + RTC Support
- `platformio.ini` - Build für MR-2
- Board: `boards/inhero_mr2.json`

---

## Wichtige Code-Änderungen

### MR-1 (v0.1)
```cpp
// InheroMr1Board.h
enum HardwareVersion {
  HW_V0_1 = 1   // Nur v0.1
};

// Keine v0.2 Methoden:
// - initiateShutdown() - entfernt
// - configureRTCWake() - entfernt  
// - getVoltageCriticalThreshold() - entfernt
// - rtcInterruptHandler() - entfernt
```

### MR-2 (v0.2)
```cpp
// InheroMr2Board.h
// Keine Hardware-Detection - immer v0.2

// Vollständiges v0.2 Power Management:
void initiateShutdown(uint8_t reason);
void configureRTCWake(uint32_t hours);
uint16_t getVoltageCriticalThreshold();
uint16_t getVoltageWakeThreshold();
uint16_t getVoltageHardwareCutoff();
static void rtcInterruptHandler();
```

---

## Board-Definitionen

### boards/inhero_mr1.json
```json
{
  "name": "Inhero MR-1",
  "usb_product": "Inhero MR1",
  "variant": "Inhero_MR1_Board"
}
```

### boards/inhero_mr2.json
```json
{
  "name": "Inhero MR-2",
  "usb_product": "Inhero MR2",
  "variant": "Inhero_MR2_Board"
}
```

---

## PlatformIO Builds

### MR-1
```bash
platformio run -e Inhero_MR1_repeater
```

### MR-2
```bash
platformio run -e Inhero_MR2_repeater
```

---

## Dokumentation

### MR-1 (Legacy)
- `variants/inhero_mr1/README.md` - Marked as legacy
- `variants/inhero_mr1/IMPLEMENTATION_SUMMARY.md` - Legacy v0.1 docs

### MR-2 (Current)
- `variants/inhero_mr2/README.md` - Current hardware
- `variants/inhero_mr2/IMPLEMENTATION_SUMMARY.md` - Complete v0.2 implementation

---

## Migration Path

Bestehende MR-1 Hardware (v0.1):
- Bleibt auf `variants/inhero_mr1`
- Code ist vereinfacht und wartungsfreundlich
- Keine v0.2 Features

Neue MR-2 Hardware (v0.2):
- Nutzt `variants/inhero_mr2`
- Vollständiges Power Management
- INA228 Coulomb Counter
- RTC Wake-up Support

---

Datum: 1. Februar 2026
