#include "Logger.h"
#include "SerialConsole.h"
#include "BMSModuleManager.h"
// Removed: SystemIO.h and any CAN include

#define BMS_BAUD  612500 //612500

BMSModuleManager bms;
EEPROMSettings settings;
SerialConsole console;
uint32_t lastUpdate;

void loadSettings()
{
    Logger::console("Resetting to factory defaults");
    settings.version = EEPROM_VERSION;
    settings.checksum = 0;
    settings.logLevel = 2;          // removed canSpeed + batteryID

    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
}

// REMOVED: entire initializeCAN() function

void setup()
{
    delay(2000);
    SERIALCONSOLE.begin(115200);
    SERIALCONSOLE.println("Tesla BMS ESP32-WROOM - CAN stripped");

    // BMS UART on ESP32 (pins 16=RX, 17=TX - matches original wiring)
    SERIAL.begin(BMS_BAUD, SERIAL_8N1, 16, 17);

    SERIALCONSOLE.println("Started serial interface to BMS.");

    pinMode(13, INPUT);

    loadSettings();

    // REMOVED: "Initialize CAN" + initializeCAN()

    SERIALCONSOLE.println("Init BMS board numbers");
    bms.renumberBoardIDs();

    lastUpdate = 0;

    SERIALCONSOLE.println("BMS clear faults");
    bms.clearFaults();
    SERIALCONSOLE.println("End of setup");
    SERIALCONSOLE.println("Send ? for help, d for detailed updates, p for summary.");
    delay(1000);
}

void loop()
{
    console.loop();

    if (millis() > (lastUpdate + 1000))
    {
        lastUpdate = millis();
        bms.balanceCells();
        bms.getAllVoltTemp();
    }

    // REMOVED: entire CAN receive block (CAN0.available / read / processCANMsg)
}