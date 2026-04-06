/*
 * EEPROMSettings.cpp  handler for all things relating to EEPROM, including loading/saving settings and factory reset
 */

#include "EEPROMSettings.h"
#include "Logger.h"

EEPROMData eepromdata;

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
    EEPROM.get(0, eepromdata);

    if (eepromdata.version != EEPROM_VERSION ||
        eepromdata.checksum != (eepromdata.version + 42))
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
    eepromdata.checksum = eepromdata.version + 42;
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
    save();
}

void EEPROMSettings::resetFaultLog()
{
    memset(eepromdata.faultLog, 0, sizeof(eepromdata.faultLog));
    save();
}