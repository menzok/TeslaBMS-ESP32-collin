/*
 * EEPROMSettings.cpp  handler for all things relating to EEPROM, including loading/saving settings and factory reset
 */

#include "EEPROMSettings.h"
#include "Logger.h"

EEPROMData eepromdata;

// CRC-8 (poly 0x07, init 0x00) over every byte of eepromdata except the checksum field itself.
// The checksum field sits at byte offset 1; we cover [0] and [2..sizeof-1].
static uint8_t calcEEPROMChecksum()
{
    const uint8_t* p = (const uint8_t*)&eepromdata;
    uint8_t crc = 0;
    for (size_t i = 0; i < sizeof(EEPROMData); i++) {
        if (i == 1) continue;   // skip the checksum byte itself
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

void EEPROMSettings::loadDefaults()
{
    Logger::console("Loading factory defaults...");

    eepromdata.version = EEPROM_VERSION;
    eepromdata.logLevel = 2;

    resetSafetyThresholds();
    resetAdditionalHardware();
    resetBatteryConfig();
    // faultLog is deliberately NOT touched here

    Logger::console("Factory defaults loaded");
    save();
}

void EEPROMSettings::load()
{
    EEPROM.begin(sizeof(EEPROMData));
    EEPROM.get(0, eepromdata);

    if (eepromdata.version != EEPROM_VERSION ||
        eepromdata.checksum != calcEEPROMChecksum())
    {
        Logger::console("EEPROM invalid or empty - Resetting to factory defaults");
        memset(&eepromdata, 0, sizeof(EEPROMData));
        loadDefaults();
    }
    else
    {
        Logger::console("Settings loaded from EEPROM successfully");
    }
    Logger::setLoglevel((Logger::LogLevel)eepromdata.logLevel);
}

void EEPROMSettings::save()
{
    eepromdata.checksum = calcEEPROMChecksum();
    EEPROM.put(0, eepromdata);
    EEPROM.commit();

    Logger::console("Settings saved to EEPROM");
}

// ====================== Per-menu resets ======================
void EEPROMSettings::resetSafetyThresholds()
{
    eepromdata.OverVSetpoint = DEFAULT_OVERVOLTAGE;
    eepromdata.UnderVSetpoint = DEFAULT_UNDERVOLTAGE;
    eepromdata.OverTSetpoint = DEFAULT_OVERTEMP;
    eepromdata.UnderTSetpoint = DEFAULT_UNDERTEMP;
    eepromdata.balanceVoltage = DEFAULT_BALANCE_VOLTAGE;
    eepromdata.balanceHyst = DEFAULT_BALANCE_HYST;
    eepromdata.OVERCURRENT_THRESHOLD_A = DEFAULT_OVERCURRENT_THRESHOLD_A;
    eepromdata.CELL_FAULT_DEBOUNCE = DEFAULT_CELL_FAULT_DEBOUNCE;
    save();
}

void EEPROMSettings::resetAdditionalHardware()
{
    eepromdata.prechargeEnabled = DEFAULT_PRECHARGE_ENABLED;
    eepromdata.prechargeTimeoutMs = DEFAULT_PRECHARGE_TIMEOUT_MS;
    eepromdata.currentSensorPresent = DEFAULT_CURRENT_SENSOR_PRESENT;
    eepromdata.currentSensorVbias = DEFAULT_CURRENT_SENSOR_VBIAS;
    eepromdata.currentSensorVrange = DEFAULT_CURRENT_SENSOR_VRANGE;
    eepromdata.currentSensorRatedAmps = DEFAULT_CURRENT_SENSOR_RATED_AMPS;
    save();
}

void EEPROMSettings::resetBatteryConfig()
{
    eepromdata.parallelStrings = DEFAULT_PARALLEL_STRINGS;
    eepromdata.STORAGE_WAKE_INTERVAL_MS = DEFAULT_STORAGE_WAKE_INTERVAL_MS;
    eepromdata.STORAGE_BALANCE_DURATION_MS = DEFAULT_STORAGE_BALANCE_DURATION_MS;
    eepromdata.socPercent = DEFAULT_SOC_PERCENT;
    eepromdata.coulombCountAh = DEFAULT_COULOMB_COUNT_AH;
    save();
}

void EEPROMSettings::resetFaultLog()
{
    memset(eepromdata.faultLog, 0, sizeof(eepromdata.faultLog));
    save();
}