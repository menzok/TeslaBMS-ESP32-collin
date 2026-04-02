#include "Logger.h"
#include "SerialConsole.h"  // TODO: replace with Menu.h when Menu is ready
#include "BMSModuleManager.h"
#include "EEPROMSettings.h"
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient mqtt(espClient);

// ==================== CHANGE THESE ====================
const char* ssid = "Zoom";
const char* password = "gdr543l7";
const char* mqtt_server = "192.168.1.213";   // ← your Pi IP
// =======================================================



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

    // ==================== YOUR PROVEN WORKING WIFI BLOCK ====================
    SERIALCONSOLE.print("SSID: ");
    SERIALCONSOLE.println(ssid);
    SERIALCONSOLE.print("Password length: ");
    SERIALCONSOLE.println(strlen(password));
    SERIALCONSOLE.print("Password bytes (hex): ");
    for (int i = 0; i < strlen(password); i++) {
        SERIALCONSOLE.printf("%02X ", password[i]);
    }
    SERIALCONSOLE.println("\n");

    SERIALCONSOLE.println("Scanning for networks...");
    int n = WiFi.scanNetworks();
    SERIALCONSOLE.printf("%d networks found\n", n);
    for (int i = 0; i < n; i++) {
        if (String(WiFi.SSID(i)) == ssid) SERIALCONSOLE.print(">>> ");
        SERIALCONSOLE.printf("%s  RSSI:%d\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }

    SERIALCONSOLE.printf("\nConnecting to %s...\n", ssid);
    WiFi.begin(ssid, password);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 30000) {
        delay(500);
        SERIALCONSOLE.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        SERIALCONSOLE.println("\n\nWiFi CONNECTED!");
        SERIALCONSOLE.print("IP address: ");
        SERIALCONSOLE.println(WiFi.localIP());
    } else {
        SERIALCONSOLE.println("\n\nWiFi FAILED!");
        SERIALCONSOLE.printf("Status code: %d\n", WiFi.status());
        while (1) delay(1000);
    }

    mqtt.setServer(mqtt_server, 1883);
    mqtt.connect("TeslaBMS");

    // Publish device status once (required by dbus-mqtt-devices)
    mqtt.publish("device/teslabms/Status", "{\"clientId\":\"teslabms\",\"connected\":1,\"version\":\"1.0\",\"services\":{\"b1\":\"battery\"}}");
}

void loop() {
    console.loop();

    if (millis() > (lastUpdate + 1000)) {
        lastUpdate = millis();
        bms.balanceCells();
        bms.getAllVoltTemp();
    }

    if (!mqtt.connected()) {
        mqtt.connect("TeslaBMS");
    }
    mqtt.loop();

      if (millis() - lastUpdate > 1000) {
        lastUpdate = millis();

        float avgCell = bms.getAvgCellVolt();
        uint8_t soc = (avgCell >= 4.2f) ? 100 : (avgCell <= 3.0f) ? 0 : (uint8_t)((avgCell - 3.0f) / 1.2f * 100.0f);
        float packV = bms.getPackVoltage();
        float current = 0.0f;

        char json[128];

        // Exact JSON format the driver expects
        sprintf(json, "{\"Dc\":{\"Voltage\":%.2f,\"Current\":%.2f,\"Power\":%.2f},\"Soc\":%d}", 
                packV, current, packV * current, soc);

        mqtt.publish("teslabms/battery", json);   // matches the topic in config.ini

       // SERIALCONSOLE.printf("Published JSON → SOC:%d%% V:%.2fV\n", soc, packV);
    }
}