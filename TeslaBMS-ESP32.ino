#include "Logger.h"
#include "SerialConsole.h"  // TODO: replace with Menu.h when Menu is ready
#include "BMSModuleManager.h"
#include "EEPROMSettings.h"






BMSModuleManager bms;
SerialConsole console;  // TODO: replace with:  Menu menu;
uint32_t lastUpdate = 0;



void setup() {
    delay(2000);
    pinMode(FAULT_CHAIN_PIN, INPUT); //fault chain monitoring pin
	SERIALCONSOLE.begin(115200);  // open USB serial for console output
	SERIAL.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN); // Open the hardware serial port for talking to the BMS modules
	
	EEPROMSettings::load(); 

    SERIALCONSOLE.println("Finding connected boards, RUN Board renumbering on pack change");
	bms.findBoards();

    
}

void loop() {
    console.loop();

    if (millis() > (lastUpdate + 1000)) {
        lastUpdate = millis();
        bms.balanceCells();
        bms.getAllVoltTemp();
    }

}