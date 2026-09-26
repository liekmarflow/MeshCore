// Host-only dependencies. Every configuration method under test is extracted
// from production by run.py; this model provides persistence/GPIO/BQ/INA I/O.
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using std::isfinite;
#define MESH_DEBUG_PRINTLN(...) do {} while (false)
#define BQ_CE_PIN 42
enum { OUTPUT, LOW, HIGH };
static int ce_level = LOW;
static unsigned pin_writes = 0;
static uint32_t now_ms = 12345;
static uint32_t lastPgStuckToggleTime = 0, lastPgStuckAttemptTime = 0;
static bool pgStuckRecoveryAttempted = false;
static void pinMode(int, int) {}
static void digitalWrite(int, int level) { ce_level = level; ++pin_writes; }
static uint32_t millis() { return now_ms; }

class SimplePreferences {
 public:
  static inline std::map<std::string, std::string> values;
  static inline unsigned mutations = 0;
  static inline int fail_at = -1;
  static inline bool fail_begin = false;
  static inline bool silently_ignore_write = false;
  bool begin(const char*) { return !fail_begin; }
  bool containsKey(const char* key) { return values.count(key) != 0; }
  bool mutationAllowed() { return static_cast<int>(mutations++) != fail_at; }
  size_t putString(const char* key, const char* value) {
    if (!mutationAllowed()) return 0;
    if (!silently_ignore_write) values[key] = value;
    return std::strlen(value);
  }
  size_t putInt(const char* key, uint16_t value) {
    return putString(key, std::to_string(value).c_str());
  }
  bool remove(const char* key) {
    if (!mutationAllowed()) return false;
    if (!silently_ignore_write) values.erase(key);
    return true;
  }
  size_t getString(const char* key, char* buffer, size_t size, const char* fallback = "") {
    const auto found = values.find(key);
    const std::string value = found == values.end() ? fallback : found->second;
    if (size) std::snprintf(buffer, size, "%s", value.c_str());
    return value.size();
  }
};

enum bq25798_cell_count_t { BQ25798_CELL_COUNT_1S, BQ25798_CELL_COUNT_2S };
enum {
  BQ25798_TREG_60C = 60, BQ25798_WDT_DISABLE, BQ25798_VOC_DLY_2S,
  BQ25798_VOC_RATE_2MIN, BQ25798_VOC_PCT_81_25, BQ25798_TS_COOL_5C,
  BQ25798_TS_WARM_55C, BQ25798_JEITA_VSET_UNCHANGED,
  BQ25798_JEITA_ISETC_SUSPEND = 0, BQ25798_JEITA_ISETC_UNCHANGED = 3,
  BQ25798_JEITA_ISETC_40_PERCENT = 2, BQ25798_JEITA_ISETC_20_PERCENT = 1,
  BQ25798_JEITA_ISETH_UNCHANGED = 3,
  BQ25798_REG_CHARGER_CONTROL_0 = 0x0F, BQ25798_REG_NTC_CONTROL_0 = 0x17,
  BQ25798_REG_NTC_CONTROL_1 = 0x18, BQ25798_REG_PRECHARGE_CONTROL = 0x08,
};

struct FakeBQ {
  unsigned writes = 0;
  int fail_at = -1;
  bool ignore_writes = false;
  bool readback_bad = false;
  unsigned reads = 0;
  int fail_read_at = -1;
  bool ts_ignore = false;
  bool charging = true;
  bool mppt = false;
  float ichg = 0.2f;
  float vreg = 4.1f;
  float sysmin = 2.5f;
  float iindpm = 2.0f;
  uint16_t precharge = 200;
  int cold = BQ25798_JEITA_ISETC_SUSPEND;
  int hot = BQ25798_JEITA_ISETH_UNCHANGED;
  bq25798_cell_count_t cells = BQ25798_CELL_COUNT_1S;
  uint8_t ntc0_unrelated = 0xA1;
  uint8_t ntc1_unrelated = 0xB0;
  bool readReg(uint8_t reg, uint8_t& value) {
    if (static_cast<int>(reads++) == fail_read_at) return false;
    switch (reg) {
      case 0x00: value = static_cast<uint8_t>((sysmin - 2.5f) / 0.25f); break;
      case 0x01: value = static_cast<uint16_t>(vreg / 0.01f) >> 8; break;
      case 0x02: value = static_cast<uint16_t>(vreg / 0.01f) & 0xFF; break;
      case 0x03: value = static_cast<uint16_t>(ichg / 0.01f) >> 8; break;
      case 0x04: value = static_cast<uint16_t>(ichg / 0.01f) & 0xFF; break;
      case 0x08: value = precharge / 40; break;
      case 0x0A: value = cells == BQ25798_CELL_COUNT_2S ? 0x40 : 0; break;
      case 0x0F: value = charging ? 0x20 : 0; break;
      case 0x15: value = mppt ? 1 : 0; break;
      case 0x17: value = ntc0_unrelated | (hot << 3) | (cold << 1); break;
      case 0x18: value = ntc1_unrelated | (ts_ignore ? 1 : 0); break;
      default: throw std::runtime_error("Unsupported charger register read");
    }
    if (readback_bad) value ^= 0xFF;
    return true;
  }
  bool writeReg(uint8_t reg, uint8_t value) {
    if (!permit()) return false;
    if (ignore_writes) return true;
    if (reg == 0x17) { cold = (value >> 1) & 3; hot = (value >> 3) & 3; ntc0_unrelated = value & ~0x1E; }
    else if (reg == 0x18) { ts_ignore = value & 1; ntc1_unrelated = value & ~1; }
    else throw std::runtime_error("Unsupported charger register write");
    return true;
  }
  bool permit() { return static_cast<int>(writes++) != fail_at; }
  template<class T> bool write(T& target, T value) {
    if (!permit()) return false;
    if (!ignore_writes) target = value;
    return true;
  }
  bool setCellCount(bq25798_cell_count_t value) {
    if (!write(cells, value)) return false;
    if (!ignore_writes) { ichg = 1.0f; sysmin = cells == BQ25798_CELL_COUNT_2S ? 7.0f : 3.5f;
      vreg = cells == BQ25798_CELL_COUNT_2S ? 8.4f : 4.2f; }
    return true;
  }
  bool setChargeLimitA(float value) { return write(ichg, value); }
  float getChargeLimitA() { return readback_bad ? -1.0f : ichg; }
  bool setChargeLimitV(float value) { return write(vreg, value); }
  float getChargeLimitV() { return readback_bad ? -1.0f : vreg; }
  bool setMinSystemV(float value) { return write(sysmin, value); }
  float getMinSystemV() { return readback_bad ? -1.0f : sysmin; }
  bool setPrechargeLimitmA(uint16_t value) { return write(precharge, static_cast<uint16_t>((value / 40) * 40)); }
  uint16_t getPrechargeLimitmA() { return readback_bad ? 0 : precharge; }
  bool setChargeEnable(bool value) { return write(charging, value); }
  bool getChargeEnable() { return readback_bad ? !charging : charging; }
  bool setTsIgnore(bool value) { return write(ts_ignore, value); }
  bool getTsIgnore() { return readback_bad ? !ts_ignore : ts_ignore; }
  bool setJeitaISetC(int value) { return write(cold, value); }
  int getJeitaISetC() { return readback_bad ? -1 : cold; }
  bool setJeitaISetH(int value) { return write(hot, value); }
  int getJeitaISetH() { return readback_bad ? -1 : hot; }
  bq25798_cell_count_t getCellCount() { return cells; }
  bool setMPPTenable(bool value) { return write(mppt, value); }
  bool getMPPTenable() { return mppt; }
  bool setInputLimitA(float value) { return write(iindpm, value); }
  bool setThermRegulationThresh(int) { return permit(); }
  bool setRechargeThreshOffsetV(float) { return permit(); }
  bool setPrechargeTimerEnable(bool) { return permit(); }
  bool setFastChargeTimerEnable(bool) { return permit(); }
  bool setWDT(int) { return permit(); }
  bool setExtILIMpin(bool) { return permit(); }
  bool setICOEnable(bool) { return permit(); }
  bool setVOCdelay(int) { return permit(); }
  bool setVOCrate(int) { return permit(); }
  bool setVOCpercent(int) { return permit(); }
  bool setAutoDPinsDetection(bool) { return permit(); }
  bool setStatPinEnable(bool) { return permit(); }
  bool setTsCool(int) { return permit(); }
  bool setTsWarm(int) { return permit(); }
  bool setJeitaVSet(int) { return permit(); }
  bool setAutoIBATDIS(bool) { return permit(); }
  void getTelemetryData(int) {}
};

struct FakeINA {
  unsigned resets = 0;
  bool resetCoulombCounter() { ++resets; return true; }
  float readCharge_mAh() { return 0.0f; }
};

#include "production_types.inc"
class BoardConfigContainer {
 public:
#include "production_declarations.inc"
  FakeBQ bq;
  bool bqInitialized = true;
  bool chargeConfigurationFault = false;
  static inline FakeBQ* bqDriverInstance = nullptr;
  static inline FakeINA* ina228DriverInstance = nullptr;
  static inline bool leds_enabled = true;
  static inline bool usbInputActive = false;
  static inline bool jeitaIgnoreActive = false;
  static inline bool lowVoltageAlertFired = false;
  static inline float tcCalOffset = 2.0f;
  static inline float lastValidBatteryTemp = 25.0f;
  static inline uint32_t lastTempUpdateMs = 100;
  static inline BatteryType cachedBatteryType = LIION_1S;
  static inline BatterySOCStats socStats = {};
  static inline MpptStatistics mpptStats = {};
  static inline unsigned iindpm_updates = 0;
  static inline unsigned alert_updates = 0;
  bool tickInitialized = true;
  bool lowVoltageRecovery = false;
  uint32_t lastMpptMs = 0, lastSocMs = 0, lastHourlyMs = 0;
  uint32_t lastLowVoltageMs = 0, lastLowVoltageSleepAttemptMs = 0;
  bool lowVoltageSleepAttempted = false;

  static BatteryType getBatteryTypeFromCommandString(const char*);
  static char* trim(char*);
  static const char* getBatteryTypeCommandString(BatteryType);
  static const char* getFrostChargeBehaviourCommandString(FrostChargeBehaviour);
  static FrostChargeBehaviour getFrostChargeBehaviourFromCommandString(const char*);
  static const BatteryProperties* getBatteryProperties(BatteryType);
  static float getNominalVoltage(BatteryType);
  BatteryType getBatteryType() const;
  FrostChargeBehaviour getFrostChargeBehaviour() const;
  uint16_t getMaxChargeCurrent_mA() const;
  float getBatteryCapacity() const;
  bool getMPPTEnabled() const;
  bool loadBatType(BatteryType&) const;
  bool loadFrost(FrostChargeBehaviour&) const;
  bool loadMaxChrgI(uint16_t&) const;
  static bool loadMpptEnabled(bool&);
  bool loadBatteryCapacity(float&) const;
  bool isBatteryCapacitySet() const;
  bool loadJeitaIgnoreEnabled(bool&) const;
  bool getJeitaIgnoreEnabled() const;
  static bool isJeitaIgnoreCurrentAllowed(uint16_t, float);
  bool jeitaIgnoreGateOk() const;
  bool setJeitaIgnore(bool);
  bool applyJeitaIgnore();
  bool applyJeitaIgnore(const BatteryProperties*);
  bool normalizeJeitaIgnore(const BatteryProperties*);
  bool disableCharging();
  bool recoverChargeConfiguration();
  bool hasChargeConfigurationFault() const;
  bool storeSetting(const char*, const char*) const;
  bool removeSetting(const char*) const;
  bool verifyChargeConfiguration(const BatteryProperties*);
  bool configureBaseBQ();
  bool configureChemistry(BatteryType);
  void resetBatteryStatistics(BatteryType);
  bool setBatteryType(BatteryType);
  bool setFrostChargeBehaviour(FrostChargeBehaviour);
  bool setMaxChargeCurrent_mA(uint16_t);
  bool setBatteryCapacity(float);
  bool isJeitaIgnoreActive() const { return jeitaIgnoreActive; }
  const char* getChargeCurrentAsStr();
  static const char* getAvailableBatOptions();
  static const char* getAvailableFrostChargeBehaviourOptions();
  static void updateSolarIINDPM() { ++iindpm_updates; }
  void armLowVoltageAlert(BatteryType) { ++alert_updates; }
  static void disarmLowVoltageAlert() { ++alert_updates; }
  static uint32_t getRTCTimestamp() { return 500000; }
};

#include "production_functions.inc"
#include "production_cli.inc"

static void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

static std::string cli_set(BoardConfigContainer& board, const char* command) {
  // Production trims the command buffer in place, so provide writable storage.
  std::vector<char> buffer(command, command + std::strlen(command) + 1);
  return handleSet(board, buffer.data());
}

static std::string cli_get(BoardConfigContainer& board, const char* command) {
  char reply[200] = {};
  require(handleGet(board, command, reply, sizeof(reply)), "getter not handled");
  return reply;
}

static void reset_environment() {
  SimplePreferences::values = {{"batType", "liion1s"}, {"frost", "20%"}, {"maxChrg", "200"},
    {"mpptEn", "1"}, {"batCap", "10000.0"}, {"jeitaIgn", "0"},
    {"leds_en", "0"}, {"tcCal", "2.0"}, {"altitude", "400.0"}};
  SimplePreferences::mutations = 0;
  SimplePreferences::fail_at = -1;
  SimplePreferences::fail_begin = false;
  SimplePreferences::silently_ignore_write = false;
  BoardConfigContainer::jeitaIgnoreActive = false;
  BoardConfigContainer::cachedBatteryType = BoardConfigContainer::LIION_1S;
  BoardConfigContainer::socStats = {};
  BoardConfigContainer::socStats.capacity_mah = 10000.0f;
  BoardConfigContainer::socStats.soc_valid = true;
  BoardConfigContainer::socStats.current_soc_percent = 72.0f;
  BoardConfigContainer::socStats.current_hour_charged_mah = 100.0f;
  BoardConfigContainer::socStats.charge_history_initialized = true;
  BoardConfigContainer::socStats.done_streak = 2;
  BoardConfigContainer::socStats.hours[4].charged_mah = 100.0f;
  BoardConfigContainer::mpptStats = {};
  BoardConfigContainer::mpptStats.hours[3].harvestedEnergy_mWh = 120;
  BoardConfigContainer::iindpm_updates = 0;
  BoardConfigContainer::alert_updates = 0;
  BoardConfigContainer::bqDriverInstance = nullptr;
  BoardConfigContainer::ina228DriverInstance = nullptr;
  BoardConfigContainer::usbInputActive = false;
  ce_level = HIGH;
  pin_writes = 0;
}

struct Snapshot {
  std::map<std::string, std::string> prefs = SimplePreferences::values;
  unsigned mutations = SimplePreferences::mutations;
  unsigned bq_writes;
  unsigned gpio_writes = pin_writes;
  BatterySOCStats stats = BoardConfigContainer::socStats;
  unsigned alerts = BoardConfigContainer::alert_updates;
  unsigned iindpm = BoardConfigContainer::iindpm_updates;
  explicit Snapshot(const BoardConfigContainer& board): bq_writes(board.bq.writes) {}
  void unchanged(const BoardConfigContainer& board) const {
    require(prefs == SimplePreferences::values, "Rejected setting changed persisted values");
    require(mutations == SimplePreferences::mutations, "Rejected setting attempted persistence");
    require(bq_writes == board.bq.writes, "Rejected setting touched charger");
    require(gpio_writes == pin_writes, "Rejected setting touched GPIO");
    require(std::memcmp(&stats, &BoardConfigContainer::socStats, sizeof(stats)) == 0,
            "Rejected setting changed battery statistics");
    require(alerts == BoardConfigContainer::alert_updates && iindpm == BoardConfigContainer::iindpm_updates,
            "Rejected setting changed charging/voltage monitoring");
  }
};

int main() {
  unsigned passed = 0;
  auto test = [&passed](const char* name, auto run) {
    reset_environment();
    run();
    ++passed;
    std::cout << "PASS " << name << '\n';
  };
  try {
    test("strict 0.05C boundary and finite/range validation", [] {
      using B = BoardConfigContainer;
      require(B::isJeitaIgnoreCurrentAllowed(499, 10000.0f), "below gate rejected");
      require(!B::isJeitaIgnoreCurrentAllowed(500, 10000.0f), "equality accepted");
      require(!B::isJeitaIgnoreCurrentAllowed(501, 10000.0f), "above gate accepted");
      require(B::isJeitaIgnoreCurrentAllowed(500, 10000.1f), "fractional capacity truncated");
      require(!B::isJeitaIgnoreCurrentAllowed(49, 10000.0f), "invalid current accepted");
      require(!B::isJeitaIgnoreCurrentAllowed(1501, 100000.0f), "high current accepted");
      require(!B::isJeitaIgnoreCurrentAllowed(200, std::numeric_limits<float>::infinity()), "infinite capacity accepted");
      require(!B::isJeitaIgnoreCurrentAllowed(200, std::numeric_limits<float>::quiet_NaN()), "NaN capacity accepted");
    });
    test("rejected override changes no settings or hardware", [] {
      BoardConfigContainer board;
      SimplePreferences::values.erase("batCap");
      Snapshot missing(board);
      require(!board.setJeitaIgnore(true), "unset explicit capacity accepted");
      missing.unchanged(board);
      SimplePreferences::values["batCap"] = "4000.0";
      Snapshot equal(board);
      require(!board.setJeitaIgnore(true), "equal gate accepted");
      equal.unchanged(board);
      SimplePreferences::values["batCap"] = "3999.0";
      Snapshot above(board);
      require(!board.setJeitaIgnore(true), "above gate accepted");
      above.unchanged(board);
    });
    test("enabled override rejects incompatible imax and capacity atomically", [] {
      BoardConfigContainer board;
      require(board.setJeitaIgnore(true), "valid override rejected");
      Snapshot before(board);
      require(!board.setMaxChargeCurrent_mA(500), "equal current gate accepted");
      before.unchanged(board);
      require(!board.setMaxChargeCurrent_mA(501), "above current gate accepted");
      before.unchanged(board);
      require(!board.setBatteryCapacity(4000.0f), "equal capacity gate accepted");
      before.unchanged(board);
      require(!board.setBatteryCapacity(3999.0f), "below capacity gate accepted");
      before.unchanged(board);
      require(board.getJeitaIgnoreEnabled() && board.isJeitaIgnoreActive(), "rejection disabled override");
      require(board.setMaxChargeCurrent_mA(490), "valid changed current rejected");
      require(board.setBatteryCapacity(11000.0f), "valid changed capacity rejected");
    });
    test("fmax discarded on enable, default on disable, repeated off preserves new fmax", [] {
      BoardConfigContainer board;
      require(board.getFrostChargeBehaviour() == BoardConfigContainer::I_REDUCE_TO_20, "fixture frost");
      require(board.setJeitaIgnore(true), "enable failed");
      require(board.getFrostChargeBehaviour() == BoardConfigContainer::NO_CHARGE, "old frost remained hidden");
      require(board.bq.ts_ignore, "hardware ignore missing");
      require(board.bq.ntc0_unrelated == 0xA1 && board.bq.ntc1_unrelated == 0xB0,
              "JEITA update changed unrelated NTC register bits");
      require(board.setJeitaIgnore(false), "disable failed");
      require(board.getFrostChargeBehaviour() == BoardConfigContainer::NO_CHARGE, "old frost restored");
      require(!board.bq.ts_ignore, "hardware ignore still on");
      require(board.setFrostChargeBehaviour(BoardConfigContainer::I_REDUCE_TO_40), "new frost rejected");
      require(board.setJeitaIgnore(false), "repeated off failed");
      require(board.getFrostChargeBehaviour() == BoardConfigContainer::I_REDUCE_TO_40, "repeated off erased new frost");
    });
    test("valid override survives reboot, invalid legacy wishes never reappear", [] {
      BoardConfigContainer board;
      require(board.setJeitaIgnore(true), "enable failed");
      BoardConfigContainer rebooted;
      BoardConfigContainer::jeitaIgnoreActive = false;
      BoardConfigContainer::socStats.capacity_mah = 0.0f; // begin fills the RAM cache later
      const auto* props = BoardConfigContainer::getBatteryProperties(BoardConfigContainer::LIION_1S);
      require(rebooted.configureBaseBQ() && rebooted.configureChemistry(BoardConfigContainer::LIION_1S),
              "valid reboot charger configuration failed");
      require(rebooted.getJeitaIgnoreEnabled() && rebooted.isJeitaIgnoreActive(), "valid override lost across reboot");
      SimplePreferences::values["maxChrg"] = "500";
      require(rebooted.normalizeJeitaIgnore(props), "legacy normalization failed");
      require(!rebooted.getJeitaIgnoreEnabled(), "legacy invalid wish retained");
      require(rebooted.getFrostChargeBehaviour() == BoardConfigContainer::NO_CHARGE, "legacy hidden frost retained");
      require(rebooted.setMaxChargeCurrent_mA(200), "safe current rejected after cleanup");
      require(rebooted.applyJeitaIgnore(), "apply after cleanup failed");
      require(!rebooted.isJeitaIgnoreActive() && !rebooted.getJeitaIgnoreEnabled(), "legacy wish reactivated");
    });
    test("same chemistry is a complete no-op", [] {
      BoardConfigContainer board;
      Snapshot before(board);
      require(board.setBatteryType(BoardConfigContainer::LIION_1S), "same chemistry failed");
      before.unchanged(board);
    });
    test("every actual chemistry change resets battery state, preserves installation settings", [] {
      using B = BoardConfigContainer;
      for (B::BatteryType from : {B::BAT_UNKNOWN, B::LTO_2S, B::LIFEPO4_1S, B::LIION_1S, B::NAION_1S}) {
        for (B::BatteryType to : {B::BAT_UNKNOWN, B::LTO_2S, B::LIFEPO4_1S, B::LIION_1S, B::NAION_1S}) {
          if (from == to) continue;
          reset_environment();
          SimplePreferences::values["batType"] = B::getBatteryTypeCommandString(from);
          SimplePreferences::values["jeitaIgn"] = "1";
          SimplePreferences::values["maxChrg"] = "900";
          B board;
          FakeINA ina;
          B::ina228DriverInstance = &ina;
          B::bqDriverInstance = &board.bq;
          require(board.setBatteryType(to), "chemistry change failed");
          require(board.getBatteryType() == to, "wrong chemistry stored");
          require(board.getMaxChargeCurrent_mA() == 200, "old imax survived chemistry change");
          require(!board.getMPPTEnabled(), "old MPPT survived chemistry change");
          require(!board.isBatteryCapacitySet(), "old explicit capacity survived chemistry change");
          require(board.getBatteryCapacity() == (to == B::LIFEPO4_1S ? 1500.0f : 2000.0f), "wrong default capacity");
          require(!board.getJeitaIgnoreEnabled(), "old JEITA user override survived chemistry change");
          require(board.getFrostChargeBehaviour() == B::NO_CHARGE, "old frost survived chemistry change");
          require(!B::socStats.soc_valid && B::socStats.current_soc_percent == 0.0f, "old SOC survived chemistry change");
          require(B::socStats.current_hour_charged_mah == 0.0f && B::socStats.hours[4].charged_mah == 0.0f,
                  "old battery history survived chemistry change");
          require(B::mpptStats.hours[3].harvestedEnergy_mWh == 0, "old charge energy survived chemistry change");
          require(!B::socStats.charge_history_initialized && B::socStats.done_streak == 0,
                  "old Coulomb history or charging-DONE streak survived chemistry change");
          require(SimplePreferences::values.at("leds_en") == "0" && SimplePreferences::values.at("tcCal") == "2.0" &&
                  SimplePreferences::values.at("altitude") == "400.0", "installation settings erased");
          require((ce_level == HIGH) == B::getBatteryProperties(to)->charge_enable, "wrong final CE state");
          B::ina228DriverInstance = nullptr;
          B::bqDriverInstance = nullptr;
        }
      }
    });
    test("chemistry-forced JEITA ignores user gate", [] {
      for (auto type : {BoardConfigContainer::LTO_2S, BoardConfigContainer::NAION_1S}) {
        reset_environment();
        BoardConfigContainer board;
        require(board.setBatteryType(type), "forced chemistry failed");
        require(board.isJeitaIgnoreActive() && !board.getJeitaIgnoreEnabled(), "forced chemistry has user wish");
        require(board.setMaxChargeCurrent_mA(1500), "forced chemistry incorrectly constrained current");
        require(board.setBatteryCapacity(100.0f), "forced chemistry incorrectly constrained capacity");
      }
    });
    test("CLI gate errors are exact and changes are atomically rejected", [] {
      BoardConfigContainer board;
      SimplePreferences::values.erase("batCap");
      Snapshot missing(board);
      require(cli_set(board, "jeitaignore 1") == "N/A, batcap not set", "missing capacity response");
      missing.unchanged(board);
      SimplePreferences::values["batCap"] = "4000.0";
      Snapshot equal(board);
      require(cli_set(board, "jeitaignore 1") == "N/A, imax >=0,05C", "equality response");
      equal.unchanged(board);
      SimplePreferences::values["batCap"] = "10000.0";
      require(cli_set(board, "jeitaignore 1") == "jeitaignore set to 1", "enable response");
      Snapshot active(board);
      require(cli_set(board, "imax 500") == "N/A, jeitaignore=1", "current rejection response");
      active.unchanged(board);
      require(cli_set(board, "batcap 4000") == "N/A, jeitaignore=1", "capacity rejection response");
      active.unchanged(board);
      require(cli_get(board, "jeitaignore") == "jeitaignore 1", "active getter response");
      require(cli_get(board, "fmax") == "N/A", "active frost getter response");
      require(cli_set(board, "jeitaignore 0") == "jeitaignore set to 0", "disable response");
      require(cli_get(board, "fmax") == "0%", "frost default getter response");
      require(cli_get(board, "jeitaignore") == "jeitaignore 0", "disabled getter response");
      require(cli_get(board, "saved").find("Err:") == 0, "removed getter still recognized");
      require(cli_get(board, "saved").find("saved") == std::string::npos, "removed getter still advertised");
    });
    test("capacity is normalized to the visible whole-mAh value before the gate", [] {
      BoardConfigContainer board;
      SimplePreferences::values["maxChrg"] = "500";
      SimplePreferences::values["batCap"] = "11000";
      BoardConfigContainer::socStats.capacity_mah = 11000.0f;
      require(board.setJeitaIgnore(true), "valid override rejected");
      Snapshot before(board);
      require(!board.setBatteryCapacity(10000.1f), "rounded capacity equality accepted by core");
      before.unchanged(board);
      require(cli_set(board, "batcap 10000.1") == "N/A, jeitaignore=1", "rounded equality response");
      before.unchanged(board);
      require(board.setBatteryCapacity(10000.6f), "valid rounded capacity rejected");
      require(board.getBatteryCapacity() == 10001.0f, "hidden fractional capacity in cache");
      require(SimplePreferences::values.at("batCap") == "10001", "hidden fractional capacity persisted");
      require(cli_get(board, "batcap") == "10001 mAh (set)", "capacity getter differs from gate");
    });
    test("override persistence failures report failure without a deferred request", [] {
      BoardConfigContainer baseline;
      require(baseline.setJeitaIgnore(true), "baseline override failed");
      const unsigned mutations = SimplePreferences::mutations;
      for (unsigned fail = 0; fail < mutations; ++fail) {
        reset_environment();
        BoardConfigContainer board;
        SimplePreferences::fail_at = fail;
        require(!board.setJeitaIgnore(true), "failed preference write reported success");
        require(!board.getJeitaIgnoreEnabled(), "failed enable left a latent override");
        require(ce_level == LOW, "failed enable did not disable charging");
      }
      reset_environment();
      BoardConfigContainer board;
      SimplePreferences::silently_ignore_write = true;
      require(!board.setJeitaIgnore(true), "ignored preference write reported success");
      require(!board.getJeitaIgnoreEnabled() && ce_level == LOW, "ignored write left unsafe state");
    });
    test("every JEITA I2C read/write failure aborts and leaves no enable request", [] {
      BoardConfigContainer baseline;
      require(baseline.setJeitaIgnore(true), "baseline override failed");
      const unsigned reads = baseline.bq.reads;
      const unsigned writes = baseline.bq.writes;
      for (unsigned fail = 0; fail < reads; ++fail) {
        reset_environment();
        BoardConfigContainer board;
        board.bq.fail_read_at = fail;
        require(!board.setJeitaIgnore(true), "JEITA register read failure reported success");
        require(!board.getJeitaIgnoreEnabled() && ce_level == LOW, "failed JEITA read left unsafe state");
      }
      for (unsigned fail = 0; fail < writes; ++fail) {
        reset_environment();
        BoardConfigContainer board;
        board.bq.fail_at = fail;
        require(!board.setJeitaIgnore(true), "JEITA register write failure reported success");
        require(!board.getJeitaIgnoreEnabled() && ce_level == LOW, "failed JEITA write left unsafe state");
      }
      reset_environment();
      BoardConfigContainer board;
      board.bq.ignore_writes = true;
      require(!board.setJeitaIgnore(true), "ignored JEITA register write reported success");
      require(!board.getJeitaIgnoreEnabled() && ce_level == LOW, "ignored JEITA write left unsafe state");
    });
    test("legacy invalid request is removed even when rewriting it fails", [] {
      BoardConfigContainer board;
      SimplePreferences::values["jeitaIgn"] = "1";
      SimplePreferences::values["maxChrg"] = "500";
      SimplePreferences::fail_at = 0;
      require(!board.normalizeJeitaIgnore(BoardConfigContainer::getBatteryProperties(BoardConfigContainer::LIION_1S)),
              "failed legacy rewrite reported success");
      require(!board.getJeitaIgnoreEnabled(), "legacy request survived failed rewrite");
      require(ce_level == LOW, "failed legacy cleanup left charging enabled");
      SimplePreferences::fail_at = -1;
      require(board.setMaxChargeCurrent_mA(200), "valid current after migration rejected");
      require(board.applyJeitaIgnore(), "apply after migration failed");
      require(!board.isJeitaIgnoreActive(), "legacy request returned after valid current");
    });
    test("chemistry reset aborts safely at each persistence operation", [] {
      BoardConfigContainer baseline;
      require(baseline.setBatteryType(BoardConfigContainer::LTO_2S), "baseline chemistry failed");
      const unsigned operations = SimplePreferences::mutations;
      require(operations > 5, "chemistry persistence coverage unexpectedly empty");
      for (unsigned fail = 0; fail < operations; ++fail) {
        reset_environment();
        BoardConfigContainer board;
        SimplePreferences::fail_at = static_cast<int>(fail);
        require(!board.setBatteryType(BoardConfigContainer::LTO_2S), "failed chemistry preference write reported success");
        require(ce_level == LOW, "failed chemistry persistence left charging enabled");
        require(board.getBatteryType() != BoardConfigContainer::LTO_2S, "partially initialized new chemistry accepted");
        require(SimplePreferences::values.at("altitude") == "400.0", "failed reset changed installation setting");
      }
    });
    test("chemistry configuration aborts safely at every checked register read", [] {
      BoardConfigContainer baseline;
      require(baseline.setBatteryType(BoardConfigContainer::LTO_2S), "baseline chemistry failed");
      const unsigned reads = baseline.bq.reads;
      require(reads > 10, "chemistry readback coverage unexpectedly empty");
      for (unsigned fail = 0; fail < reads; ++fail) {
        reset_environment();
        BoardConfigContainer board;
        board.bq.fail_read_at = static_cast<int>(fail);
        require(!board.setBatteryType(BoardConfigContainer::LTO_2S), "failed chemistry read reported success");
        require(ce_level == LOW, "failed chemistry read left charging enabled");
        require(board.getBatteryType() != BoardConfigContainer::LTO_2S, "failed chemistry hardware accepted");
      }
    });
    test("chemistry configuration aborts safely at every charger write", [] {
      BoardConfigContainer baseline;
      require(baseline.setBatteryType(BoardConfigContainer::LTO_2S), "baseline chemistry failed");
      const unsigned writes = baseline.bq.writes;
      require(writes > 20, "charger write coverage unexpectedly empty");
      for (unsigned fail = 0; fail < writes; ++fail) {
        reset_environment();
        BoardConfigContainer board;
        board.bq.fail_at = static_cast<int>(fail);
        require(!board.setBatteryType(BoardConfigContainer::LTO_2S), "failed chemistry write reported success");
        require(ce_level == LOW, "failed chemistry write left charging enabled");
        require(board.getBatteryType() != BoardConfigContainer::LTO_2S, "failed chemistry write accepted");
      }
    });
    test("imax/capacity storage and ICHG readback failures report errors", [] {
      BoardConfigContainer board;
      SimplePreferences::fail_at = 0;
      require(!board.setBatteryCapacity(12000.0f), "failed capacity persistence reported success");
      require(board.getBatteryCapacity() == 10000.0f, "failed capacity save changed cache");
      require(ce_level == LOW, "failed capacity write left charging enabled");
      reset_environment();
      BoardConfigContainer other;
      other.bq.fail_read_at = 0;
      require(!other.setMaxChargeCurrent_mA(300), "failed ICHG readback reported success");
      require(ce_level == LOW, "failed ICHG readback left charging enabled");
    });
    test("accepted override remains visible after hardware error and retry recovers full configuration", [] {
      BoardConfigContainer board;
      require(board.setJeitaIgnore(true), "enable failed");
      board.bq.fail_read_at = static_cast<int>(board.bq.reads);
      require(!board.setMaxChargeCurrent_mA(300), "injected hardware error ignored");
      require(ce_level == LOW && board.chargeConfigurationFault, "hardware error failed to latch charge fault");
      require(board.getJeitaIgnoreEnabled() && !board.isJeitaIgnoreActive(), "fault state lost accepted setting");
      require(cli_get(board, "jeitaignore") == "jeitaignore 1", "accepted override hidden after hardware fault");
      require(cli_get(board, "fmax") == "N/A", "accepted override exposed irrelevant frost after hardware fault");
      Snapshot before(board);
      require(cli_set(board, "imax 500") == "N/A, jeitaignore=1", "fault allowed incompatible current");
      before.unchanged(board);
      require(cli_set(board, "fmax 40%").find("Err: Fmax N/A") == 0, "fault allowed hidden frost setting");
      before.unchanged(board);
      board.bq.fail_read_at = -1;
      require(board.setMaxChargeCurrent_mA(300), "retry did not recover charger");
      require(!board.chargeConfigurationFault && ce_level == HIGH && board.bq.charging,
              "successful retry left charging disabled");
      require(board.bq.ts_ignore && board.isJeitaIgnoreActive(), "successful retry lost active override");
      require(std::abs(board.bq.ichg - 0.3f) < 0.001f && std::abs(board.bq.vreg - 4.1f) < 0.001f &&
              std::abs(board.bq.sysmin - 2.5f) < 0.001f, "retry did not restore full checked configuration");
    });
    test("same-chemistry retry recovers hardware without resetting saved settings", [] {
      BoardConfigContainer board;
      require(board.disableCharging(), "could not inject latched hardware failure");
      auto settings = SimplePreferences::values;
      settings.erase("chgFault");
      require(board.setBatteryType(BoardConfigContainer::LIION_1S), "same chemistry did not recover fault");
      require(settings == SimplePreferences::values, "same-chemistry recovery reset stored configuration");
      require(!board.chargeConfigurationFault && ce_level == HIGH && board.bq.charging,
              "same-chemistry recovery left charging disabled");
    });
    test("charger fault survives MCU reset and clears only after full successful recovery", [] {
      BoardConfigContainer board;
      require(board.setJeitaIgnore(true), "enable failed");
      board.bq.fail_read_at = static_cast<int>(board.bq.reads);
      require(!board.setMaxChargeCurrent_mA(300), "injected hardware error ignored");
      require(SimplePreferences::values.at("chgFault") == "1", "charger fault not persisted");
      BoardConfigContainer rebooted;
      require(!rebooted.chargeConfigurationFault, "fresh MCU unexpectedly retains RAM fault");
      require(rebooted.hasChargeConfigurationFault(), "MCU reset lost retained fault interlock");
      rebooted.bq = board.bq; // The separately powered charger retains its registers.
      rebooted.bq.fail_read_at = static_cast<int>(rebooted.bq.reads);
      require(!rebooted.recoverChargeConfiguration(), "failed recovery unexpectedly succeeded");
      require(rebooted.hasChargeConfigurationFault() && SimplePreferences::values.count("chgFault") == 1,
              "failed recovery cleared persistent interlock");
      require(ce_level == LOW, "failed recovery released physical charge permission");
      rebooted.bq.fail_read_at = -1;
      require(rebooted.recoverChargeConfiguration(), "successful recovery rejected");
      require(!rebooted.hasChargeConfigurationFault() && SimplePreferences::values.count("chgFault") == 0,
              "successful recovery retained stale interlock");
      require(ce_level == HIGH && rebooted.bq.charging && rebooted.bq.ts_ignore,
              "successful recovery did not apply accepted safe settings");
      BoardConfigContainer next_boot;
      require(!next_boot.hasChargeConfigurationFault(), "cleared interlock returned on next MCU reset");
    });
    test("unreadable storage and any retained marker prevent early sleep permission", [] {
      BoardConfigContainer board;
      SimplePreferences::fail_begin = true;
      require(board.hasChargeConfigurationFault(), "unreadable fault storage treated as healthy");
      SimplePreferences::fail_begin = false;
      SimplePreferences::values["chgFault"] = "corrupt";
      require(board.hasChargeConfigurationFault(), "malformed retained marker ignored");
    });
    test("MCU reset after partial JEITA programming cannot retain an unaccepted override", [] {
      BoardConfigContainer board;
      // Disable read + two NTC reads + first readback pass; TS_IGNORE is written
      // before this second readback fails, leaving hardware temporarily changed.
      board.bq.fail_read_at = 4;
      require(!board.setJeitaIgnore(true), "failed final NTC readback ignored");
      require(board.bq.ts_ignore, "fault injection did not leave partial hardware state");
      require(!board.getJeitaIgnoreEnabled(), "failed enable persisted as accepted");
      BoardConfigContainer rebooted;
      rebooted.bq = board.bq;
      rebooted.bq.fail_read_at = -1;
      require(rebooted.hasChargeConfigurationFault(), "partial hardware state allowed early sleep");
      require(rebooted.recoverChargeConfiguration(), "partial hardware recovery failed");
      require(!rebooted.bq.ts_ignore && !rebooted.getJeitaIgnoreEnabled(), "unaccepted override survived recovery");
      require(!rebooted.hasChargeConfigurationFault() && ce_level == HIGH,
              "partial hardware recovery did not clear interlock safely");
    });
    test("recovery through another setter resynchronizes persisted capacity and visible cache", [] {
      BoardConfigContainer board;
      board.bq.fail_read_at = 1; // Fail full recovery after the capacity was saved.
      require(!board.setBatteryCapacity(12000.0f), "injected post-save charger failure ignored");
      require(SimplePreferences::values.at("batCap") == "12000", "fault injection missed saved capacity");
      require(board.getBatteryCapacity() == 10000.0f, "failed setter unexpectedly updated cache");
      board.bq.fail_read_at = -1;
      require(board.setMaxChargeCurrent_mA(250), "recovery through another setter failed");
      require(board.getBatteryCapacity() == 12000.0f && !BoardConfigContainer::socStats.soc_valid,
              "recovered capacity cache/SOC differs from persisted gate input");
      require(cli_get(board, "batcap") == "12000 mAh (set)", "recovered getter reports stale capacity");
    });
    std::cout << "PASS " << passed << " MR2 JEITA regression groups\n";
  } catch (const std::exception& error) {
    std::cerr << "FAIL after " << passed << " groups: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
