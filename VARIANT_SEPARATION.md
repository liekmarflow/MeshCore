# Varianten: MR-2 (v0.2)

## Status

Die aktuelle Referenzplattform ist MR-2 (v0.2).

## Inhero MR-2 (v0.2)
**Pfad:** `variants/inhero_mr2/`

**Hardware:**
- INA228 Power Monitor (I2C 0x45)
- RV-3028-C7 RTC (I2C 0x52)
- TPS62840 Buck mit Hardware-UVLO
- BQ25798 Battery Charger
- 20mΩ Shunt fuer Coulomb Counter

**Features:**
- Advanced Power Management
  - 3-Layer Protection (Software + RTC + Hardware UVLO)
  - Coulomb Counter fuer Real-time SOC
  - 7-day Energy Balance Analysis
  - RTC Wake-up Management
  - Motorboating Prevention
- MPPT Solar Charging
- Adaptive Voltage Monitoring (10s/30s/60s)

**Dateien:**
- `InheroMr2Board.h/cpp` - Vollstaendiges v0.2 Power Management
- `BoardConfigContainer.h/cpp` - INA228 + RTC Support
- `platformio.ini` - Build fuer MR-2
- Board: `boards/inhero_mr2.json`

## Board-Definition

### boards/inhero_mr2.json
```json
{
  "name": "Inhero MR-2",
  "usb_product": "Inhero MR2",
  "variant": "Inhero_MR2_Board"
}
```

## PlatformIO Build

```bash
platformio run -e Inhero_MR2_repeater
```

## Dokumentation

- `variants/inhero_mr2/README.md` - Current hardware
- `variants/inhero_mr2/IMPLEMENTATION_SUMMARY.md` - Complete v0.2 implementation

Datum: 14. Februar 2026
