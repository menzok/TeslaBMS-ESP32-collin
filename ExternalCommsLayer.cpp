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

// ─── Payload builder (34 bytes) ───────────────────────────────────────────────

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
    uint16_t alarmFlags = ALARM_NONE;
    for (int m = 1; m <= MAX_MODULE_ADDR; m++) {
        if (!bms.moduleExists(m)) continue;
        for (int c = 0; c < 6; c++) {
            CellDetails cell = bms.getCellDetails(m, c);
            float v    = cell.cellVoltage;
            float tC   = (float)cell.highTemp - 40.0f;   // decode +40 offset
            if (v > eepromdata.OverVSetpoint)  alarmFlags |= ALARM_OVER_VOLTAGE;
            if (v < eepromdata.UnderVSetpoint) alarmFlags |= ALARM_UNDER_VOLTAGE;
            if (tC > eepromdata.OverTSetpoint)  alarmFlags |= ALARM_OVER_TEMP;
            if (tC < eepromdata.UnderTSetpoint) alarmFlags |= ALARM_UNDER_TEMP;
        }
    }
    if (fabsf(packCurrentA) > eepromdata.OVERCURRENT_THRESHOLD_A)
        alarmFlags |= ALARM_OVER_CURRENT;

    // ── Overlord state ─────────────────────────────────────────────────────
    uint8_t overlordState = OVERLORD_STATE_NORMAL;
    auto state = Overlord.getState();
    if      (state == BMSOverlord::BMSState::Fault)       overlordState = OVERLORD_STATE_FAULT;
    else if (state == BMSOverlord::BMSState::StorageMode) overlordState = OVERLORD_STATE_STORAGE;
    else if (state == BMSOverlord::BMSState::Shutdown)    overlordState = OVERLORD_STATE_SHUTDOWN;

    uint8_t contactorState = (uint8_t)contactor.getState();

    // ── EEPROM config — read fresh every frame ─────────────────────────────
    uint16_t overV   = (uint16_t)round(eepromdata.OverVSetpoint  * 1000.0f); // 1 mV steps
    uint16_t underV  = (uint16_t)round(eepromdata.UnderVSetpoint * 1000.0f);
    int8_t   overT   = (int8_t)  round(eepromdata.OverTSetpoint);
    int8_t   underT  = (int8_t)  round(eepromdata.UnderTSetpoint);
    uint8_t  modules = (uint8_t) bms.getNumberOfModules();
    uint8_t  strings = (uint8_t) eepromdata.parallelStrings;

    // Overcurrent threshold — 0.1A resolution, uint16
    uint16_t overCurrent = (uint16_t)round(eepromdata.OVERCURRENT_THRESHOLD_A * 10.0f);

    // ── Status flags ───────────────────────────────────────────────────────
    uint8_t statusFlags = 0x00;
    if (eepromdata.currentSensorPresent) statusFlags |= STATUS_FLAG_CURRENT_SENSOR;
    if (bms.isAnyBalancing())            statusFlags |= STATUS_FLAG_BALANCING;

    // ── Active fault mask — 1<<type for each fault entry that hasn't been cleared
    // FaultEntry::Type: None=0, OverVoltage=1, UnderVoltage=2, OverTemperature=3,
    //                   UnderTemperature=4, OverCurrent=5, CommsError=6
    uint8_t activeFaultMask = 0x00;
    for (int fi = 0; fi < 5; fi++) {
        const FaultEntry& fe = eepromdata.faultLog[fi];
        if (fe.type != FaultEntry::Type::None && fe.clearedTimestamp == 0) {
            activeFaultMask |= (uint8_t)(1 << (uint8_t)fe.type);
        }
    }

    // ── Min / max cell voltage — 1 mV resolution ──────────────────────────
    uint16_t lowCell  = (uint16_t)round(bms.getLowestCellVoltage()  * 1000.0f);
    uint16_t highCell = (uint16_t)round(bms.getHighestCellVoltage() * 1000.0f);

    // ── Min / max module temperature ──────────────────────────────────────
    int8_t minT = (int8_t)round(bms.getMinTemperature());
    int8_t maxT = (int8_t)round(bms.getMaxTemperature());

    // ── Pack into 34 bytes (big-endian for all multi-byte fields) ──────────
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

    // Status flags and fault mask
    payload[i++] = statusFlags;                                                    // [24]
    payload[i++] = activeFaultMask;                                                // [25]

    // Min / max cell voltages
    payload[i++] = (lowCell >> 8)   & 0xFF;  payload[i++] = lowCell   & 0xFF;   // [26-27]
    payload[i++] = (highCell >> 8)  & 0xFF;  payload[i++] = highCell  & 0xFF;   // [28-29]

    // Min / max temperatures
    payload[i++] = (uint8_t)minT;                                                  // [30]
    payload[i++] = (uint8_t)maxT;                                                  // [31]

    // Reserved
    payload[i++] = 0x00;                                                           // [32]
    payload[i++] = 0x00;                                                           // [33]

    // i == EXT_PAYLOAD_LEN (34)
}

// ─── Frame transmitter ────────────────────────────────────────────────────────

void ExternalCommsLayer::sendPacket() {
    // Frame: [0xAA][34-byte payload][CRC_lo][CRC_hi]  = 37 bytes total
    txBuffer[0] = 0xAA;
    buildPayload(&txBuffer[1]);
    uint16_t crc = calculateCRC16(&txBuffer[1], EXT_PAYLOAD_LEN);
    txBuffer[1 + EXT_PAYLOAD_LEN] = crc & 0xFF;          // txBuffer[35] — CRC low byte
    txBuffer[1 + EXT_PAYLOAD_LEN + 1] = (crc >> 8) & 0xFF;   // txBuffer[36] — CRC high byte
    EXTERNAL_COMM_SERIAL.write(txBuffer, EXT_FRAME_LEN);
}

// ─── Command processor ──────────────────────────────────────────────────────
//
// Two command frame formats are supported:
//
//   Control (4 bytes): [0xAA][cmd][CRC_lo][CRC_hi]
//     CRC computed over [cmd] (1 byte)
//     Commands: EXT_CMD_SHUTDOWN, EXT_CMD_STARTUP
//
//   Data request (7 bytes) — ping-pong exchange:
//     [0xAA][0x03][curr_hi][curr_lo][staleness][CRC_lo][CRC_hi]
//     CRC computed over [cmd][curr_hi][curr_lo][staleness] (4 bytes)
//     Venus embeds its shunt/SCS current here; ESP32 replies with telemetry.
//     staleness=0 → shunt data is fresh; staleness>0 → stale/unavailable.
//
// Non-blocking by design: if there are not enough bytes in the hardware FIFO
// we return immediately and revisit on the next loop() tick.
//
// If the first byte is not 0xAA the buffer is flushed so stale or garbage bytes
// don't block the next valid frame.
//

void ExternalCommsLayer::processIncomingCommand() {
    // Need at least 4 bytes for any command frame
    if (EXTERNAL_COMM_SERIAL.available() < 4) return;

    uint8_t start = EXTERNAL_COMM_SERIAL.read();
    if (start != 0xAA) {
        // Flush remaining garbage so the next frame starts clean
        while (EXTERNAL_COMM_SERIAL.available()) {
            EXTERNAL_COMM_SERIAL.read();
        }
        return;
    }

    uint8_t cmd = EXTERNAL_COMM_SERIAL.read();

    if (cmd == EXT_CMD_SEND_DATA) {
        // Data-request frame: 3 payload bytes + 2 CRC bytes still to come (5 total).
        // readBytes() honours the serial timeout so we never spin indefinitely.
        uint8_t data[5];
        if (EXTERNAL_COMM_SERIAL.readBytes(data, 5) < 5) return;  // timeout — discard

        // data[0..1] = shunt current × 10 (int16 BE), data[2] = staleness
        // data[3..4] = CRC little-endian over [cmd, data[0], data[1], data[2]]
        uint8_t crcInput[4] = { cmd, data[0], data[1], data[2] };
        uint16_t calcCRC = calculateCRC16(crcInput, 4);
        uint16_t rxCRC   = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
        if (calcCRC != rxCRC) return;

        int16_t rawI    = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
        _shuntCurrentA  = rawI / 10.0f;
        _shuntStaleness = data[2];
        _shuntUpdateMs  = millis();

        sendPacket();
        return;
    }

    // Control command (SHUTDOWN / STARTUP): 2 CRC bytes follow
    uint8_t crcBytes[2];
    if (EXTERNAL_COMM_SERIAL.readBytes(crcBytes, 2) < 2) return;  // timeout — discard
    uint16_t calcCRC = calculateCRC16(&cmd, 1);
    uint16_t rxCRC   = (uint16_t)crcBytes[0] | ((uint16_t)crcBytes[1] << 8);
    if (calcCRC != rxCRC) return;

    if (cmd == EXT_CMD_SHUTDOWN) {
        Overlord.requestShutdown();
    } else if (cmd == EXT_CMD_STARTUP) {
        Overlord.requestStartup();
    }
    // Control commands do not generate a telemetry reply
}

// ─── Main update ──────────────────────────────────────────────────────────────

void ExternalCommsLayer::update() {
    processIncomingCommand();   // only reply when asked — no unsolicited sends
}

// Global instance
ExternalCommsLayer ExternalComms;