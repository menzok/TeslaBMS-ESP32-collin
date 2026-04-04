#include "Logger.h"
#include "SerialConsoleMenu.h"
#include "BMSModuleManager.h"
#include "EEPROMSettings.h"


BMSModuleManager bms;
Menu menu;
uint32_t lastUpdate = 0;



void setup() {
    delay(2000);
	initPins(); // Set pin modes for any hardware features (fault chain, etc.)
	SERIALCONSOLE.begin(115200);  // open USB serial for console output
	SERIAL.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN); // Open the hardware serial port for talking to the BMS modules
    EEPROMSettings::load(); // Load any saved settings from EEPROM (e.g. cell count, balancing thresholds, etc.)

	SERIALCONSOLE.println("Scanning for connected BMS boards..."); //Scan for modules and print results
	bms.findBoards();
    if (bms.getNumberOfModules() == 0) {
        SERIALCONSOLE.println("No modules detected. Setup required.");
    }
    else {
        SERIALCONSOLE.printf("Found %d connected BMS modules\n", bms.getNumberOfModules());
    }
    SERIALCONSOLE.println("Type m, menu, help, or ? to open the menu"); 
    
}

void loop() {
   
    if (millis() > (lastUpdate + 1000)) {
        lastUpdate = millis();
        bms.getAllVoltTemp();
        bms.balanceCells();
        
    }
	menu.loop();
    
}