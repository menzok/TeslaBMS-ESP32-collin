#pragma once

#include <Arduino.h>
#include <EEPROM.h>

// ====================== EEPROM Version ======================
#define EEPROM_VERSION      0x13

// ====================== Default Settings ======================
// These defaults prioritize safety, longevity, and real-world solar/off-grid usage with Tesla modules

inline constexpr float DEFAULT_OVERVOLTAGE = 4.25f;   // 50mV safety headroom above the cell's absolute maximum of 4.20V
inline constexpr float DEFAULT_UNDERVOLTAGE = 2.90f;   // Conservative lower limit for daily cycling; protects long-term battery health
inline constexpr float DEFAULT_OVERTEMP = 60.0f;   // Maximum safe temperature for Tesla modules in enclosures/solar sheds
inline constexpr float DEFAULT_UNDERTEMP = -10.0f;  // Standard safe lower limit for discharge on 18650/2170 cells
inline constexpr float DEFAULT_BALANCE_VOLTAGE = 3.95f;   // Balancing starts near full charge so passive bleed resistors can work effectively
inline constexpr float DEFAULT_BALANCE_HYST = 0.025f;  // 25mV hysteresis prevents rapid chattering and unnecessary heat
inline constexpr bool   DEFAULT_PRECHARGE_ENABLED = true; // Pre-charge is generally recommended to protect contactors and reduce arcing, especially in high-voltage setups
inline constexpr uint32_t DEFAULT_PRECHARGE_TIMEOUT_MS = 8000;     // 8 seconds max for pre-charge
inline constexpr bool DEFAULT_CURRENT_SENSOR_PRESENT = false; // Hall effect / current detector installed?
inline constexpr float DEFAULT_CURRENT_SENSOR_VBIAS = 2.5f;     // For a current sensor like the QN-C15S
inline constexpr float DEFAULT_CURRENT_SENSOR_VRANGE = 0.625f;   // For a current sensor like the QN-C15S with ą rated current range around the bias point
inline constexpr int   DEFAULT_CURRENT_SENSOR_RATED_AMPS = 500;   //Current sensor AMP rating QN-C15S default
inline constexpr float DEFAULT_SOC_PERCENT = 50.0f;
inline constexpr float DEFAULT_COULOMB_COUNT_AH = 0.0f;
inline constexpr uint8_t DEFAULT_PARALLEL_STRINGS = 2;
inline constexpr float DEFAULT_OVERCURRENT_THRESHOLD_A = 350.0f;
inline constexpr uint32_t DEFAULT_STORAGE_WAKE_INTERVAL_MS = 24 * 60 * 60 * 1000UL;
inline constexpr uint32_t DEFAULT_STORAGE_BALANCE_DURATION_MS = 120000UL;
inline constexpr uint8_t DEFAULT_CELL_FAULT_DEBOUNCE = 3;

// ====================== EEPROM Settings Struct ======================
struct FaultEntry {
    enum class Type : uint8_t {
        None,
        OverVoltage,
        UnderVoltage,
        OverTemperature,
        UnderTemperature,
        OverCurrent,
        CommsError
    };
    Type type = Type::None;
    uint8_t module = 0;
    uint8_t cell = 0;
    float value = 0.0f;
    uint32_t timestamp = 0;
    uint32_t clearedTimestamp = 0;
};

typedef struct {
    uint8_t version;
    uint8_t checksum;
    uint8_t logLevel;
    // Saftey THresholds
    float OverVSetpoint;
    float UnderVSetpoint;
    float OverTSetpoint;
    float UnderTSetpoint;
    float balanceVoltage;
    float balanceHyst;
    // Contactor settings
    bool    prechargeEnabled;
    uint32_t prechargeTimeoutMs;      // fallback timer when no current sensor
    // Current sensor settings
    bool    currentSensorPresent;     // Hall effect sensor installed?
    float currentSensorVbias;       // e.g. 2.5f for QN-C15S
    float currentSensorVrange;      // e.g. 0.625f for QN-C15S
    int     currentSensorRatedAmps;   // e.g. 500 for QN-C15S
    // State of Charge persistent state 
    float   socPercent;               // current SOC 0.0-100.0
    float   coulombCountAh;           // net Ah since last reset (+/-)
    //Battery Configuration
    uint8_t parallelStrings;      // number of parallel strings (e.g. 1, 2, 3...)
    //overlord settings
    FaultEntry faultLog[5];
    float OVERCURRENT_THRESHOLD_A; //  Short protection Use fuses DUMMY but just incase.
    uint32_t STORAGE_WAKE_INTERVAL_MS;  // How long to wait between storage mode wake cycles to balance and refresh the cells (e.g. 24 hours)
    uint32_t STORAGE_BALANCE_DURATION_MS;    // How long to keep the BMS awake and balancing during each storage mode wake cycle (e.g. 2 minutes)
    uint8_t CELL_FAULT_DEBOUNCE;  // number of consecutive fault readings before faulting (to prevent noise/chatter)
} EEPROMData;



extern EEPROMData eepromdata;

class EEPROMSettings {
public:
    static void load();
    static void save();
    static void loadDefaults();           // full factory reset

    // === Per-menu reset functions (exactly matching your grouping) ===
    static void resetSafetyThresholds();      // OverV/UnderV/OverT/UnderT + balance + OVERCURRENT + CELL_FAULT_DEBOUNCE
    static void resetAdditionalHardware();    // Precharge + current sensor fields
    static void resetBatteryConfig();         // parallelStrings + STORAGE_WAKE + STORAGE_BALANCE
    static void resetFaultLog();              // (not exposed in Fault menu per your request)
};