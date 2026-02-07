# Battery Capacity Auto-Learning System (Inhero MR-2 v0.2)

## Übersicht

Das MR-2 v0.2 Board enthält ein intelligentes Auto-Learning System zur automatischen Kalibrierung der Batteriekapazität. Dies ist besonders wichtig für:

- **LiFePO4 Batterien**: Flache Entladekurve @ 3.2-3.3V macht Spannungsmessung unzuverlässig
- **Unbekannte Batterien**: Kapazität muss nicht manuell konfiguriert werden
- **Batterie-Alterung**: System erkennt Kapazitätsverlust automatisch

## Hardware-Voraussetzungen

- **INA228 Power Monitor** - 24-bit ADC mit ±0.1% Genauigkeit
- **20mΩ Shunt Resistor** - Präzise Strommessung (0.5W, ±1%)
- **BQ25798 Charger** - CHARGE_DONE Status für 100% SOC Detection
- **RV-3028-C7 RTC** - Wake-up Management während Shutdown
- **LittleFS Filesystem** - Persistente Speicherung von Kapazität und Learning-Status

## Lernmethoden

### Method 1: Full-Cycle Learning (100% → 0%)

#### Funktionsweise
1. **Start-Trigger:** BQ25798 meldet `CHARGE_DONE` (Batterie voll geladen)
2. **SOC-Reset:** System setzt SOC auf 100%, Coulomb Counter auf 0 mAh
3. **Akkumulation:** INA228 zählt alle entladenen mAh
4. **End-Trigger:** Spannung erreicht Danger Zone (z.B. 3.4V bei Li-Ion)
5. **Speicherung:** Gelernte Kapazität = Akkumulierte mAh, `capacity_learned=true`

#### Eigenschaften
- ✅ **Vorteil:** Sehr genau, funktioniert für alle Chemien
- ✅ **Vorteil:** Keine Benutzerinteraktion nötig
- ⚠️ **Nachteil:** Sehr langsam bei großen Batterien
  - Beispiel: 10Ah Batterie @ 13mA Verbrauch = **29 Tage**

#### Implementierung
```cpp
// Start-Bedingung (in voltageMonitorTask)
if (charging_status == CHARGE_DONE && 
    !learning_active && 
    !reverse_learning_active && 
    !capacity_learned) {
  
  socStats.learning_active = true;
  socStats.learning_start_mah = accumulated_mah;
  socStats.current_soc_percent = 100.0f;
  MESH_DEBUG_PRINTLN("Auto-learning Method 1 started");
}

// End-Bedingung (in voltageMonitorTask)
if (learning_active && vbat_mv <= danger_threshold) {
  float learned_capacity = accumulated_mah - socStats.learning_start_mah;
  setBatteryCapacity(learned_capacity);
  setCapacityLearned(true);
  socStats.learning_active = false;
  MESH_DEBUG_PRINTLN("Method 1 complete: %.0f mAh", learned_capacity);
}
```

### Method 2: Reverse Learning (0% → 100%)

#### Funktionsweise
1. **Start-Trigger:** Wake-up aus Danger Zone mit erhöhter Spannung
2. **SOC-Reset:** System setzt SOC auf 0%, Coulomb Counter auf 0 mAh
3. **USB-C Ladung:** Benutzer steckt USB-C ein, BQ25798 lädt Batterie
4. **Akkumulation:** INA228 zählt alle geladenen mAh
5. **End-Trigger:** BQ25798 meldet `CHARGE_DONE` (Batterie voll)
6. **Speicherung:** Gelernte Kapazität = Akkumulierte mAh, `capacity_learned=true`

#### Eigenschaften
- ✅ **Vorteil:** Schnell (Stunden statt Wochen)
- ✅ **Vorteil:** Nutzerfreundlich (USB-C Ladung)
- ⚠️ **Nachteil:** Benötigt Benutzerinteraktion (USB-C einstecken)
- ⚠️ **Nachteil:** Batterie muss erstmal leer werden

#### Implementierung
```cpp
// Start-Bedingung (in InheroMr2Board::begin)
if (shutdown_reason == SHUTDOWN_REASON_LOW_VOLTAGE && 
    vbat_mv >= critical_threshold) {
  
  MESH_DEBUG_PRINTLN("Voltage recovered to 0%% SOC, starting reverse learning");
  boardConfig.startReverseLearning();
}

// startReverseLearning() in BoardConfigContainer
void BoardConfigContainer::startReverseLearning() {
  if (!reverse_learning_active && !capacity_learned) {
    socStats.reverse_learning_active = true;
    socStats.reverse_learning_start_mah = accumulated_mah;
    socStats.current_soc_percent = 0.0f;
    MESH_DEBUG_PRINTLN("Reverse learning started (0% SOC)");
  }
}

// End-Bedingung (in voltageMonitorTask)
if (reverse_learning_active && charging_status == CHARGE_DONE) {
  float learned_capacity = accumulated_mah - socStats.reverse_learning_start_mah;
  setBatteryCapacity(learned_capacity);
  setCapacityLearned(true);
  socStats.reverse_learning_active = false;
  MESH_DEBUG_PRINTLN("Method 2 complete: %.0f mAh", learned_capacity);
}
```

## Learning Gate (Anti-Restart Mechanismus)

### Problem
Ohne Learning Gate würde das System bei jedem Boot ein neues Learning starten, auch wenn bereits eine gültige Kapazität gelernt wurde.

### Lösung: `capacity_learned` Flag

#### Filesystem-Persistenz
```cpp
// Speichern (setBatteryCapacity)
prefs.putInt(BATTERY_CAPACITY_KEY, (uint16_t)capacity_mah);
prefs.putString("cap_learned", "0");  // Manual entry

// Speichern (setCapacityLearned)
prefs.putString("cap_learned", learned ? "1" : "0");

// Laden (voltageMonitorTask)
SimplePreferences prefs;
if (prefs.begin("inheromr2")) {
  char buffer[8];
  prefs.getString("cap_learned", buffer, sizeof(buffer), "0");
  socStats.capacity_learned = (strcmp(buffer, "1") == 0);
  prefs.end();
}
```

#### Verhaltenstabelle

| Zustand | `capacity_learned` | Method 1 Start | Method 2 Start |
|---------|-------------------|----------------|----------------|
| Erste Inbetriebnahme | `false` | ✅ Bei CHARGE_DONE | ✅ Bei Wake-up |
| Nach Learning | `true` | ❌ Blockiert | ❌ Blockiert |
| Nach Reboot | `true` (persistent) | ❌ Blockiert | ❌ Blockiert |
| Nach `set board.batcap` | `false` | ✅ Bei CHARGE_DONE | ✅ Bei Wake-up |
| Nach `board.relearn` | `false` | ✅ Bei CHARGE_DONE | ✅ Bei Wake-up |

### Implementierung
```cpp
// Method 1 Start Check
if (charging_status == CHARGE_DONE && 
    !learning_active && 
    !reverse_learning_active && 
    !capacity_learned) {  // ⚠️ Gate hier!
  socStats.learning_active = true;
}

// Method 2 Start Check
void startReverseLearning() {
  if (!reverse_learning_active && !capacity_learned) {  // ⚠️ Gate hier!
    socStats.reverse_learning_active = true;
  }
}

// Reset via CLI
void resetLearning() {
  socStats.capacity_learned = false;
  setCapacityLearned(false);  // Persist to filesystem
  MESH_DEBUG_PRINTLN("Learning reset - auto-learning enabled");
}
```

## CLI Workflow & Best Practices

### Neue Installation (Method 1)
```bash
# 1. Status prüfen
board.learning
# Output: Learning IDLE Cap:2000mAh(manual/default)

# 2. Batterie voll laden (USB-C oder Solar)
# System wartet auf CHARGE_DONE...

# 3. Learning startet automatisch
board.learning
# Output: M1 ACTIVE (100%→0%, 120 mAh) Cap:2000mAh(manual)

# 4. Nach ~29 Tagen (10Ah @ 13mA)
board.learning
# Output: Learning IDLE Cap:9850mAh(learned)
```

### Schnelles Learning (Method 2)
```bash
# 1. Batterie komplett entladen (Danger Zone erreichen)
# Gerät schaltet sich ab → RTC Wake-up nach 1h

# 2. Wenn Spannung erholt → Reverse Learning startet
board.learning
# Output: M2 ACTIVE (0%→100%, 120 mAh) Cap:2000mAh(manual)

# 3. USB-C einstecken → Batterie wird geladen
# Nach ~8 Stunden (10Ah @ 1.2A Ladestrom)

# 4. Bei CHARGE_DONE
board.learning
# Output: Learning IDLE Cap:9820mAh(learned)
```

### Batterie-Austausch
```bash
# 1. Alte Kapazität löschen
board.relearn
# Output: Learning reset - auto-learning enabled

# 2. Status prüfen
board.learning
# Output: Learning IDLE Cap:9820mAh(manual/default)

# 3. Warten auf nächsten Learning-Trigger
# Entweder CHARGE_DONE (Method 1) oder Wake-up (Method 2)
```

### Manuelle Kapazität (Learning überspringen)
```bash
# Wenn Kapazität bekannt ist, kann Learning übersprungen werden
set board.batcap 10000
# Output: Battery capacity set to 10000 mAh
# ⚠️ Setzt capacity_learned=false → Auto-Learning aktiviert!

# ⚠️ Problem: Learning würde wieder starten
# Lösung: SimplePreferences müsste ein "manual override" Flag unterstützen
# Aktuell: Manuelle Kapazität wird als "ungelernt" behandelt
```

## Technische Spezifikationen

### Genauigkeit
- **INA228 ADC:** 24-bit, ±0.1% Genauigkeit
- **Shunt Resistor:** 20mΩ ±1% (0.5W)
- **Temperaturkompensation:** ±50ppm/°C
- **Typische Messabweichung:** <2% bei 20°C

### Stromverbrauch
- **Aktiver Betrieb:** ~13mA (SX1262 LoRa Repeater)
- **SYSTEMOFF:** 1-5µA (RTC Wake-up)
- **Learning-Overhead:** ~100µA (INA228 Coulomb Counter)

### Voltage Thresholds (Beispiel: Li-Ion 1S)
- **Hardware UVLO:** 3.2V (INA228 Alert → TPS62840 EN, absolute cutoff)
- **Critical Threshold:** 3.4V (0% SOC, danger zone boundary, software shutdown)
- **Hysteresis:** 200mV (prevents motorboating between UVLO and Critical)
- **Full Charge:** 4.2V (CHARGE_DONE, 100% SOC)

### Filesystem Layout
```
/inheromr2/
  ├── batCap.txt         # uint16_t capacity in mAh
  ├── cap_learned.txt    # "0" or "1" (learning status)
  ├── batType.txt        # "liion1s", "lifepo1s", "lto2s"
  ├── ina228Cal.txt      # float calibration factor
  └── ...
```

## Debugging & Troubleshooting

### Learning startet nicht

#### Symptom
```bash
board.learning
# Output: Learning IDLE Cap:2000mAh(learned)
```

**Ursache:** `capacity_learned=true` blockiert Auto-Start

**Lösung:**
```bash
board.relearn
board.learning
# Output: Learning IDLE Cap:2000mAh(manual/default)
# Warten auf CHARGE_DONE oder Wake-up
```

### Learning stoppt vorzeitig

#### Method 1 stoppt bei 50% SOC
**Ursache:** Voltage-Threshold zu hoch gesetzt

**Debug:**
```bash
board.diag
# Check: Vbat vs. danger_threshold
```

**Lösung:** Voltage-Thresholds in `BoardConfigContainer.h` anpassen

#### Method 2 startet nicht nach Wake-up
**Ursache:** GPREGRET2 nicht gesetzt oder RTC Wake-up fehlgeschlagen

**Debug:**
```cpp
uint8_t shutdown_reason = NRF_POWER->GPREGRET2;
MESH_DEBUG_PRINTLN("Boot reason: 0x%02X", shutdown_reason);
```

**Lösung:** RTC-Konfiguration prüfen (siehe `configureRTCWake`)

### Kapazität unrealistisch

#### Learning ergibt 150% der erwarteten Kapazität
**Ursache:** INA228 Kalibrationsfehler

**Lösung:**
```bash
# Messe echten Strom mit DMM während Entladung
# Erwarteter Strom: 100mA
# INA228 zeigt: 150mA
# Faktor: 100/150 = 0.667
set board.ibcal 0.667
board.relearn  # Re-learning mit korrigiertem Faktor
```

## Zukünftige Erweiterungen

### Adaptive Learning
- **Problem:** 10Ah @ 13mA = 29 Tage ist zu langsam
- **Idee:** Voltage-curve Sampling während Entladung
- **Challenge:** LiFePO4 hat flache Kurve → nur für Li-Ion verwendbar

### Multi-Point Calibration
- **Problem:** Kapazität ändert sich mit Temperatur und Alter
- **Idee:** Mehrere Learning-Zyklen über Zeit → Durchschnitt bilden
- **Challenge:** Filesystem-Speicherplatz für Historie

### Learning Quality Indicator
- **Problem:** Benutzer weiß nicht, wie zuverlässig gelernte Kapazität ist
- **Idee:** Konfidenz-Score basierend auf Anzahl Zyklen, Temperaturvariation
- **Implementation:** `board.learning` zeigt Quality-Score

## Siehe auch

- [README.md](README.md) - Allgemeine MR-2 v0.2 Dokumentation
- [IMPLEMENTATION_SUMMARY.md](IMPLEMENTATION_SUMMARY.md) - Technische Details
- [BoardConfigContainer.cpp](BoardConfigContainer.cpp) - Source Code
- [InheroMr2Board.cpp](InheroMr2Board.cpp) - Hardware Integration
