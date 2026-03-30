#include "Logger.h"
#include "SerialConsole.h"
#include "BMSModuleManager.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <Preferences.h>

WiFiClient espClient;
PubSubClient mqtt(espClient);
Preferences prefs;

// ==================== CHANGE THESE ====================
const char* ssid = "Zoom";
const char* password = "gdr543l7";
const char* mqtt_server = "192.168.1.213";   // ← your Pi IP
// =======================================================

#define BMS_BAUD  612500
#define PREFS_NAMESPACE "teslabms"

BMSModuleManager bms;
EEPROMSettings settings;
SerialConsole console;
uint32_t lastUpdate = 0;

// Safety alarm state tracked across loops to avoid spamming MQTT
bool alarmOverV = false;
bool alarmUnderV = false;
bool alarmOverT = false;
bool alarmUnderT = false;
bool alarmCellGap = false;

void saveSettings() {
    prefs.begin(PREFS_NAMESPACE, false);
    prefs.putUChar("version",       settings.version);
    prefs.putUInt ("canSpeed",      settings.canSpeed);
    prefs.putUChar("batteryID",     settings.batteryID);
    prefs.putUChar("logLevel",      settings.logLevel);
    prefs.putFloat("OverVSP",       settings.OverVSetpoint);
    prefs.putFloat("UnderVSP",      settings.UnderVSetpoint);
    prefs.putFloat("ChargeVSP",     settings.ChargeVsetpoint);
    prefs.putFloat("DischVSP",      settings.DischVsetpoint);
    prefs.putFloat("ChargeHys",     settings.ChargeHys);
    prefs.putFloat("DischHys",      settings.DischHys);
    prefs.putFloat("WarnOff",       settings.WarnOff);
    prefs.putFloat("CellGap",       settings.CellGap);
    prefs.putFloat("IgnoreVolt",    settings.IgnoreVolt);
    prefs.putUChar("IgnoreTemp",    settings.IgnoreTemp);
    prefs.putFloat("OverTSP",       settings.OverTSetpoint);
    prefs.putFloat("UnderTSP",      settings.UnderTSetpoint);
    prefs.putFloat("balanceV",      settings.balanceVoltage);
    prefs.putFloat("balanceHyst",   settings.balanceHyst);
    prefs.putInt  ("Scells",        settings.Scells);
    prefs.putInt  ("Pstrings",      settings.Pstrings);
    prefs.putUShort("triptime",     settings.triptime);
    prefs.end();
    Logger::console("Settings saved to NVS");
}

void loadSettings() {
    prefs.begin(PREFS_NAMESPACE, true); // read-only
    uint8_t storedVersion = prefs.getUChar("version", 0);
    prefs.end();

    if (storedVersion != EEPROM_VERSION) {
        // No valid settings stored – apply factory defaults
        Logger::console("No saved settings found (or version mismatch). Loading factory defaults.");
        settings.version          = EEPROM_VERSION;
        settings.checksum         = 0;
        settings.canSpeed         = 500000;
        settings.batteryID        = 1;
        settings.logLevel         = 2;
        settings.OverVSetpoint    = 4.20f;
        settings.UnderVSetpoint   = 3.00f;
        settings.ChargeVsetpoint  = 4.15f;
        settings.DischVsetpoint   = 3.10f;
        settings.ChargeHys        = 0.05f;
        settings.DischHys         = 0.05f;
        settings.WarnOff          = 0.10f;
        settings.CellGap          = 0.20f;
        settings.IgnoreVolt       = 0.50f;
        settings.IgnoreTemp       = 0;     // 0 = use both sensors
        settings.OverTSetpoint    = 50.0f;
        settings.UnderTSetpoint   = -20.0f;
        settings.balanceVoltage   = 4.10f;
        settings.balanceHyst      = 0.04f;
        settings.Scells           = 6;
        settings.Pstrings         = 1;
        settings.triptime         = 1000;  // 1 second fault persistence before trip
        saveSettings();
    } else {
        Logger::console("Loading settings from NVS.");
        prefs.begin(PREFS_NAMESPACE, true);
        settings.version          = storedVersion;
        settings.checksum         = 0;
        settings.canSpeed         = prefs.getUInt ("canSpeed",   500000);
        settings.batteryID        = prefs.getUChar("batteryID",  1);
        settings.logLevel         = prefs.getUChar("logLevel",   2);
        settings.OverVSetpoint    = prefs.getFloat("OverVSP",    4.20f);
        settings.UnderVSetpoint   = prefs.getFloat("UnderVSP",   3.00f);
        settings.ChargeVsetpoint  = prefs.getFloat("ChargeVSP",  4.15f);
        settings.DischVsetpoint   = prefs.getFloat("DischVSP",   3.10f);
        settings.ChargeHys        = prefs.getFloat("ChargeHys",  0.05f);
        settings.DischHys         = prefs.getFloat("DischHys",   0.05f);
        settings.WarnOff          = prefs.getFloat("WarnOff",    0.10f);
        settings.CellGap          = prefs.getFloat("CellGap",    0.20f);
        settings.IgnoreVolt       = prefs.getFloat("IgnoreVolt", 0.50f);
        settings.IgnoreTemp       = prefs.getUChar("IgnoreTemp", 0);
        settings.OverTSetpoint    = prefs.getFloat("OverTSP",    50.0f);
        settings.UnderTSetpoint   = prefs.getFloat("UnderTSP",   -20.0f);
        settings.balanceVoltage   = prefs.getFloat("balanceV",   4.10f);
        settings.balanceHyst      = prefs.getFloat("balanceHyst",0.04f);
        settings.Scells           = prefs.getInt  ("Scells",     6);
        settings.Pstrings         = prefs.getInt  ("Pstrings",   1);
        settings.triptime         = prefs.getUShort("triptime",  1000);
        prefs.end();
    }

    Logger::setLoglevel((Logger::LogLevel)settings.logLevel);
}

void checkSafetyAlarms() {
    float highCell = bms.getHighCellVolt();
    float lowCell  = bms.getLowCellVolt();
    float highTemp = bms.getHighTemperature();
    float lowTemp  = bms.getLowTemperature();
    float cellGap  = highCell - lowCell;

    bool newAlarmOV  = (highCell > settings.OverVSetpoint);
    bool newAlarmUV  = (lowCell  > 0.5f && lowCell < settings.UnderVSetpoint);  // ignore unpopulated cells
    bool newAlarmOT  = (highTemp > TEMP_SENSOR_DISCONNECTED && highTemp > settings.OverTSetpoint);
    bool newAlarmUT  = (highTemp > TEMP_SENSOR_DISCONNECTED && lowTemp  < settings.UnderTSetpoint);
    bool newAlarmCG  = (cellGap  > settings.CellGap);

    if (newAlarmOV  != alarmOverV)  { if (newAlarmOV)  Logger::warn("ALARM SET: Cell overvoltage: %.3fV (limit %.3fV)",  highCell, settings.OverVSetpoint);  else Logger::info("ALARM CLEAR: Cell overvoltage resolved"); alarmOverV  = newAlarmOV; }
    if (newAlarmUV  != alarmUnderV) { if (newAlarmUV)  Logger::warn("ALARM SET: Cell undervoltage: %.3fV (limit %.3fV)", lowCell,  settings.UnderVSetpoint); else Logger::info("ALARM CLEAR: Cell undervoltage resolved"); alarmUnderV = newAlarmUV; }
    if (newAlarmOT  != alarmOverT)  { if (newAlarmOT)  Logger::warn("ALARM SET: Over temperature: %.1fC (limit %.1fC)",  highTemp, settings.OverTSetpoint);  else Logger::info("ALARM CLEAR: Over temperature resolved"); alarmOverT  = newAlarmOT; }
    if (newAlarmUT  != alarmUnderT) { if (newAlarmUT)  Logger::warn("ALARM SET: Under temperature: %.1fC (limit %.1fC)", lowTemp,  settings.UnderTSetpoint); else Logger::info("ALARM CLEAR: Under temperature resolved"); alarmUnderT = newAlarmUT; }
    if (newAlarmCG  != alarmCellGap){ if (newAlarmCG)  Logger::warn("ALARM SET: Cell gap too large: %.0fmV (limit %.0fmV)", cellGap*1000.0f, settings.CellGap*1000.0f); else Logger::info("ALARM CLEAR: Cell gap within limits"); alarmCellGap= newAlarmCG; }
}

void setup() {
    delay(2000);
    SERIALCONSOLE.begin(115200);
    SERIALCONSOLE.println("\n=== TeslaBMS MQTT for dbus-mqtt-devices ===");

    SERIAL.begin(BMS_BAUD, SERIAL_8N1, 16, 17);

    pinMode(13, INPUT);
    loadSettings();

    // Apply sensor/cell ignore settings to manager
    bms.setPstrings(settings.Pstrings);
    bms.setSensors(settings.IgnoreTemp, settings.IgnoreVolt);

    SERIALCONSOLE.println("Init BMS board numbers");
    bms.renumberBoardIDs();
    bms.clearFaults();

    // ==================== YOUR PROVEN WORKING WIFI BLOCK ====================
    SERIALCONSOLE.print("SSID: ");
    SERIALCONSOLE.println(ssid);
    SERIALCONSOLE.print("Password length: ");
    SERIALCONSOLE.println(strlen(password));
    SERIALCONSOLE.print("Password bytes (hex): ");
    for (int i = 0; i < (int)strlen(password); i++) {
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
        checkSafetyAlarms();
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

        // Build alarm flags bitmask for MQTT
        uint8_t alarms = 0;
        if (alarmOverV)   alarms |= 0x01;
        if (alarmUnderV)  alarms |= 0x02;
        if (alarmOverT)   alarms |= 0x04;
        if (alarmUnderT)  alarms |= 0x08;
        if (alarmCellGap) alarms |= 0x10;

        char json[192];

        // Exact JSON format the driver expects, with added alarm and cell extremes
        snprintf(json, sizeof(json),
            "{\"Dc\":{\"Voltage\":%.2f,\"Current\":%.2f,\"Power\":%.2f},\"Soc\":%d,"
            "\"Alarms\":%d,\"LowCell\":%.3f,\"HighCell\":%.3f,\"CellGap\":%.0f}",
            packV, current, packV * current, soc,
            alarms, bms.getLowCellVolt(), bms.getHighCellVolt(),
            (bms.getHighCellVolt() - bms.getLowCellVolt()) * 1000.0f);

        mqtt.publish("teslabms/battery", json);

        SERIALCONSOLE.printf("Published JSON → SOC:%d%% V:%.2fV Alarms:0x%02X\n", soc, packV, alarms);
    }
}