#include "ExternalCommsLayer.h"
#include "BMSModuleManager.h"
#include "BMSOverlord.h"
#include "ContactorController.h"
#include "SOCCalculator.h"
#include "EEPROMSettings.h"

extern BMSModuleManager    bms;
extern BMSOverlord         Overlord;
extern ContactorController contactor;
extern SOCCalculator       socCalculator;
extern EEPROMData		 eepromdata;

// ─────────────────────────────────────────────────────────────────────────────

void ExternalCommsLayer::init() {
    EXTERNAL_COMM_SERIAL.begin(EXTERNAL_COMM_BAUD, SERIAL_8N1,
        EXTERNAL_COMM_RX_PIN, EXTERNAL_COMM_TX_PIN);
}

// ─── CRC-16/MODBUS ────────────────────────────────────────────────────────────

uint16_t ExternalCommsLayer::calculateCRC16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
        }
    }
    return crc;
}

// ─── Payload builder (26 bytes) ───────────────────────────────────────────────

void ExternalCommsLayer::buildPayload(uint8_t* payload) {

    // ── Live telemetry ─────────────────────────────────────────────────────
    BatterySummary summary  = bms.getBatterySummary();
    float packVoltageV      = summary.voltage;
    float packCurrentA      = socCalculator.getPackCurrentAmps();
    float avgTempC          = bms.getAvgTemperature();
    float avgCellVoltV      = bms.getAvgCellVolt();
    float powerW            = packVoltageV * packCurrentA;

    uint16_t packV   = (uint16_t)round(packVoltageV * 100.0f);  // 10 mV steps
    int16_t  packI   = (int16_t) round(packCurrentA * 10.0f);   // 100 mA steps
    uint8_t  soc     = summary.soc;
    int8_t   temp    = (int8_t)  round(avgTempC);
    int16_t  power   = (int16_t) round(powerW);
    uint16_t avgCell = (uint16_t)round(avgCellVoltV * 100.0f);  // 10 mV steps

    // ── Alarm flags ────────────────────────────────────────────────────────
    // Bits 5-15 are reserved and default to 0 until Overlord exposes them
    uint16_t alarmFlags = ALARM_NONE;

    // ── Overlord state ─────────────────────────────────────────────────────
    uint8_t overlordState = 0;
    auto state = Overlord.getState();
    if (state == BMSOverlord::BMSState::Fault ||
        state == BMSOverlord::BMSState::Shutdown) {
        overlordState = 1;
    } else if (state == BMSOverlord::BMSState::StorageMode) {
        overlordState = 2;
    }

    uint8_t contactorState = (uint8_t)contactor.getState();

    // ── EEPROM config — read fresh every frame ─────────────────────────────
    uint16_t overV   = (uint16_t)round(eepromdata.OverVSetpoint  * 1000.0f); // 1 mV steps
    uint16_t underV  = (uint16_t)round(eepromdata.UnderVSetpoint * 1000.0f);
    int8_t   overT   = (int8_t)  round(eepromdata.OverTSetpoint);
    int8_t   underT  = (int8_t)  round(eepromdata.UnderTSetpoint);
    uint8_t  modules = (uint8_t) bms.getNumberOfModules();
    uint8_t  strings = (uint8_t) eepromdata.parallelStrings;

    // Overcurrent threshold — 0.1A resolution, uint16
    // eepromdata.OVERCURRENT_THRESHOLD_A is a float in Amps
    uint16_t overCurrent = (uint16_t)round(eepromdata.OVERCURRENT_THRESHOLD_A * 10.0f);

    // ── Pack into 26 bytes (big-endian for all multi-byte fields) ──────────
    size_t i = 0;

    // Live telemetry
    payload[i++] = (packV >> 8)      & 0xFF;  payload[i++] = packV      & 0xFF;  // [0-1]
    payload[i++] = (packI >> 8)      & 0xFF;  payload[i++] = packI      & 0xFF;  // [2-3]
    payload[i++] = soc;                                                            // [4]
    payload[i++] = (uint8_t)temp;                                                  // [5]
    payload[i++] = (power >> 8)      & 0xFF;  payload[i++] = power      & 0xFF;  // [6-7]
    payload[i++] = (avgCell >> 8)    & 0xFF;  payload[i++] = avgCell    & 0xFF;  // [8-9]

    // Status
    payload[i++] = (alarmFlags >> 8) & 0xFF;  payload[i++] = alarmFlags & 0xFF;  // [10-11]
    payload[i++] = overlordState;                                                  // [12]
    payload[i++] = contactorState;                                                 // [13]

    // EEPROM config
    payload[i++] = (overV >> 8)      & 0xFF;  payload[i++] = overV      & 0xFF;  // [14-15]
    payload[i++] = (underV >> 8)     & 0xFF;  payload[i++] = underV     & 0xFF;  // [16-17]
    payload[i++] = (uint8_t)overT;                                                 // [18]
    payload[i++] = (uint8_t)underT;                                                // [19]
    payload[i++] = modules;                                                        // [20]
    payload[i++] = strings;                                                        // [21]

    // Overcurrent threshold
    payload[i++] = (overCurrent >> 8) & 0xFF; payload[i++] = overCurrent & 0xFF; // [22-23]

    // Reserved
    payload[i++] = 0x00;                                                           // [24]
    payload[i++] = 0x00;                                                           // [25]

    // i == EXT_PAYLOAD_LEN (26)
}

// ─── Frame transmitter ────────────────────────────────────────────────────────

void ExternalCommsLayer::sendPacket() {
    // Frame: [0xAA][26-byte payload][CRC_lo][CRC_hi]
    txBuffer[0] = 0xAA;
    buildPayload(&txBuffer[1]);
    uint16_t crc = calculateCRC16(&txBuffer[1], EXT_PAYLOAD_LEN);
    txBuffer[27] = crc & 0xFF;          // CRC low byte
    txBuffer[28] = (crc >> 8) & 0xFF;   // CRC high byte
    EXTERNAL_COMM_SERIAL.write(txBuffer, EXT_FRAME_LEN);
}

// ─── Command processor ────────────────────────────────────────────────────────

void ExternalCommsLayer::processIncomingCommand() {
    if (EXTERNAL_COMM_SERIAL.available() < 4) return;

    uint8_t buf[4];
    EXTERNAL_COMM_SERIAL.readBytes(buf, 4);

    if (buf[0] != 0xAA) return;

    // Validate CRC over the single command byte only
    uint16_t calcCRC = calculateCRC16(&buf[1], 1);
    uint16_t rxCRC   = buf[2] | (buf[3] << 8);
    if (calcCRC != rxCRC) return;

    uint8_t cmd = buf[1];

    if (cmd == EXT_CMD_SHUTDOWN) {
        Overlord.requestShutdown();
    } else if (cmd == EXT_CMD_STARTUP) {
        Overlord.requestStartup();
    }
    // EXT_CMD_SEND_DATA: no special action, fall through to sendPacket()

    sendPacket();
}

// ─── Main update ──────────────────────────────────────────────────────────────

void ExternalCommsLayer::update() {
    processIncomingCommand();   // only reply when asked — no unsolicited sends
}

// Global instance
ExternalCommsLayer ExternalComms;