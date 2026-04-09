#include "ExternalCommsLayer.h"
#include "BMSModuleManager.h"
#include "BMSOverlord.h"
#include "ContactorController.h"
#include "SOCCalculator.h"

extern BMSModuleManager bms;
extern BMSOverlord Overlord;
extern ContactorController contactor;
extern SOCCalculator socCalculator;

void ExternalCommsLayer::init() {
    EXTERNAL_COMM_SERIAL.begin(EXTERNAL_COMM_BAUD, SERIAL_8N1,
        EXTERNAL_COMM_RX_PIN, EXTERNAL_COMM_TX_PIN);
}

uint16_t ExternalCommsLayer::calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;                     // Modbus initial value
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            }
            else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void ExternalCommsLayer::buildPayload(uint8_t* payload) {
    BatterySummary summary = bms.getBatterySummary();
    float packVoltageV = summary.voltage;
    float packCurrentA = socCalculator.getPackCurrentAmps();
    float avgTempC = bms.getAvgTemperature();
    float avgCellVoltV = bms.getAvgCellVolt();          // approved getter
    float powerW = packVoltageV * packCurrentA;

    // PackVoltage_mV - 10 mV resolution
    uint16_t packV = (uint16_t)round(packVoltageV * 100.0f);

    // PackCurrent_mA - 100 mA resolution (signed)
    int16_t packI = (int16_t)round(packCurrentA * 10.0f);

    // SOC (0-100)
    uint8_t soc = summary.soc;

    // AVG_Temperature_C - whole °C, no offset, -30..+50 range
    int8_t temp = (int8_t)round(avgTempC);

    // Power_Watts - 1 W resolution
    int16_t power = (int16_t)round(powerW);

    // AvgCellVolt - 10 mV resolution
    uint16_t avgCell = (uint16_t)round(avgCellVoltV * 100.0f);

    // Reserved + AlarmFlags stub + states
    uint16_t reserved16 = 0;
    uint16_t alarmFlags = ALARM_NONE;

    // OverlordState (0=Normal, 1=Fault, 2=StorageMode)
    uint8_t overlordState = 0;
    auto state = Overlord.getState();
    if (state == BMSOverlord::BMSState::Fault || state == BMSOverlord::BMSState::Shutdown) {
        overlordState = 1;
    }
    else if (state == BMSOverlord::BMSState::StorageMode) {
        overlordState = 2;
    }

    // ContactorState - simple index (same as enum)
    uint8_t contactorState = (uint8_t)contactor.getState();

    // Build exact 18-byte payload
    size_t i = 0;
    payload[i++] = (packV >> 8) & 0xFF;      payload[i++] = packV & 0xFF;
    payload[i++] = (packI >> 8) & 0xFF;      payload[i++] = packI & 0xFF;
    payload[i++] = soc;
    payload[i++] = (uint8_t)temp;
    payload[i++] = (power >> 8) & 0xFF;      payload[i++] = power & 0xFF;
    payload[i++] = (avgCell >> 8) & 0xFF;    payload[i++] = avgCell & 0xFF;
    payload[i++] = (reserved16 >> 8) & 0xFF; payload[i++] = reserved16 & 0xFF;
    payload[i++] = (alarmFlags >> 8) & 0xFF; payload[i++] = alarmFlags & 0xFF;
    payload[i++] = overlordState;
    payload[i++] = contactorState;
    payload[i++] = 0;  // reserve byte 2
    payload[i++] = 0;  // reserve byte 3
}

void ExternalCommsLayer::sendPacket() {
    txBuffer[0] = 0xAA;
    uint8_t payload[18];
    buildPayload(payload);
    memcpy(&txBuffer[1], payload, 18);

    uint16_t crc = calculateCRC16(&txBuffer[1], 18);   // CRC on payload only
    txBuffer[19] = crc & 0xFF;                         // low byte first
    txBuffer[20] = (crc >> 8) & 0xFF;

    EXTERNAL_COMM_SERIAL.write(txBuffer, 21);
}

bool ExternalCommsLayer::processIncomingCommand() {
    if (EXTERNAL_COMM_SERIAL.available() < 4) return false;

    uint8_t buf[4];
    EXTERNAL_COMM_SERIAL.readBytes(buf, 4);

    if (buf[0] != 0xAA) return false;

    // Validate CRC on command byte only (start byte excluded)
    uint16_t calcCRC = calculateCRC16(&buf[1], 1);
    uint16_t rxCRC = buf[2] | (buf[3] << 8);
    if (calcCRC != rxCRC) return false;   // ignore invalid frame

    uint8_t cmd = buf[1];

    if (cmd == EXT_CMD_SHUTDOWN) {
        Overlord.requestShutdown();
    }
    else if (cmd == EXT_CMD_STARTUP) {
        Overlord.requestStartup();
    }
    // CMD_SEND_DATA_NOW does nothing extra

    sendPacket();   // reply with full data packet
    return true;
}

void ExternalCommsLayer::update() {
    bool hadCommand = processIncomingCommand();

    if (!hadCommand) {
        sendPacket();   // unsolicited data push (heartbeat)
    }
}

// Global instance
ExternalCommsLayer ExternalComms;