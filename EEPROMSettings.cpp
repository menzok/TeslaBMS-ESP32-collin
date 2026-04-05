
/*
 * EEPROMSettings.cpp  handler for all things relating to EEPROM, including loading/saving settings and factory reset
 */

#include "EEPROMSettings.h"
#include "Logger.h"

EEPROMData eepromdata;

void EEPROMSettings::loadDefaults()
{
    Logger::console("Loading factory defaults...");

    eepromdata.version  = EEPROM_VERSION;
    eepromdata.logLevel = 2;

    eepromdata.OverVSetpoint = DEFAULT_OVERVOLTAGE;
    eepromdata.UnderVSetpoint = DEFAULT_UNDERVOLTAGE;
    eepromdata.OverTSetpoint = DEFAULT_OVERTEMP;
    eepromdata.UnderTSetpoint = DEFAULT_UNDERTEMP;
    eepromdata.balanceVoltage = DEFAULT_BALANCE_VOLTAGE;
    eepromdata.balanceHyst = DEFAULT_BALANCE_HYST;

    eepromdata.prechargeEnabled = DEFAULT_PRECHARGE_ENABLED;
    eepromdata.currentSensorPresent = DEFAULT_CURRENT_SENSOR_PRESENT;
    eepromdata.prechargeTimeoutMs = DEFAULT_PRECHARGE_TIMEOUT_MS;
    eepromdata.currentSensorVbias = DEFAULT_CURRENT_SENSOR_VBIAS;
    eepromdata.currentSensorVrange = DEFAULT_CURRENT_SENSOR_VRANGE;
    eepromdata.currentSensorRatedAmps = DEFAULT_CURRENT_SENSOR_RATED_AMPS;
    eepromdata.socPercent = DEFAULT_SOC_PERCENT;
    eepromdata.coulombCountAh = DEFAULT_COULOMB_COUNT_AH;
    eepromdata.parallelStrings = DEFAULT_PARALLEL_STRINGS;

    Logger::console("Factory defaults loaded");
    save();
}

void EEPROMSettings::load()
{
    EEPROM.get(0, eepromdata);

    if (eepromdata.version != EEPROM_VERSION)
    {
        Logger::console("EEPROM invalid or empty - Resetting to factory defaults");
        loadDefaults();
    }
    else
    {
        Logger::console("Settings loaded from EEPROM successfully");
    }
    Logger::setLoglevel((Logger::LogLevel)eepromdata.logLevel); //set log level.... derp.
}

void EEPROMSettings::save()
{
    eepromdata.checksum = eepromdata.version + 42;

    EEPROM.put(0, eepromdata);
    EEPROM.commit();

    Logger::console("Settings saved to EEPROM");
}