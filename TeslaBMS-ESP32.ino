#include "Logger.h"
#include "SerialConsoleMenu.h"
#include "BMSOverlord.h"
#include "EEPROMSettings.h"
#include "ExternalCommsLayer.h"


// === GLOBAL OBJECT INSTANCES ===
BMSModuleManager bms;               // the module manager
ContactorController contactor;     // contactor logic
SOCCalculator socCalculator;        // SOC & current calculation
BMSOverlord Overlord;               // the main BMS controller
Menu menu;            // the interactive serial console menu
// ===================================================
uint32_t lastUpdate = 0;



void setup() {
    delay(2000);
	initPins(); // Set pin modes for any hardware features (fault chain, etc.)
	SERIALCONSOLE.begin(115200);  // open USB serial for console output
	SERIAL.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN); // Open the hardware serial port for talking to the BMS modules
    EEPROMSettings::load(); // Load any saved settings from EEPROM (e.g. cell count, balancing thresholds, etc.)
    Overlord.init();
    ExternalComms.init();
    SERIALCONSOLE.println("Type m, menu, help, or ? to open the menu"); 
    
}

void loop() {
   
    if (millis() - lastUpdate >= 1000UL) {
        lastUpdate = millis();
		Overlord.update();
        ExternalComms.update();
        
    }
	menu.loop();
    
}