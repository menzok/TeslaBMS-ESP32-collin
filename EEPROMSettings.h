#pragma once

#include <Arduino.h>
#include <EEPROM.h>

// ====================== EEPROM Version ======================
#define EEPROM_VERSION      0x12

// ====================== Default Settings ======================
// These defaults prioritize safety, longevity, and real-world solar/off-grid usage with Tesla modules

constexpr float DEFAULT_OVERVOLTAGE = 4.25f;   // 50mV safety headroom above the cell's absolute maximum of 4.20V
constexpr float DEFAULT_UNDERVOLTAGE = 2.90f;   // Conservative lower limit for daily cycling; protects long-term battery health
constexpr float DEFAULT_OVERTEMP = 60.0f;   // Maximum safe temperature for Tesla modules in enclosures/solar sheds
constexpr float DEFAULT_UNDERTEMP = -10.0f;  // Standard safe lower limit for discharge on 18650/2170 cells
constexpr float DEFAULT_BALANCE_VOLTAGE = 3.95f;   // Balancing starts near full charge so passive bleed resistors can work effectively
constexpr float DEFAULT_BALANCE_HYST = 0.025f;  // 25mV hysteresis prevents rapid chattering and unnecessary heat
constexpr bool   DEFAULT_PRECHARGE_ENABLED = true; // Pre-charge is generally recommended to protect contactors and reduce arcing, especially in high-voltage setups
constexpr uint32_t DEFAULT_PRECHARGE_TIMEOUT_MS = 8000;     // 8 seconds max for pre-charge
constexpr bool DEFAULT_CURRENT_SENSOR_PRESENT = false; // Hall effect / current detector installed?
constexpr float DEFAULT_CURRENT_SENSOR_VBIAS = 2.5f;     // For a current sensor like the QN-C15S
constexpr float DEFAULT_CURRENT_SENSOR_VRANGE = 0.625f;   // For a current sensor like the QN-C15S with ± rated current range around the bias point
constexpr int   DEFAULT_CURRENT_SENSOR_RATED_AMPS = 500;   // QN-C15S default
constexpr float DEFAULT_SOC_PERCENT = 50.0f; 
constexpr float DEFAULT_COULOMB_COUNT_AH = 0.0f;



// ====================== EEPROM Settings Struct ======================
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
} EEPROMData;

extern EEPROMData eepromdata;

class EEPROMSettings {
public:
    static void load();
    static void save();
    static void loadDefaults();
};