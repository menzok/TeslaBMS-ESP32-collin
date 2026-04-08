/*
 * ContactorController.cpp
 */

#include "ContactorController.h"
#include "config.h"          
#include "EEPROMSettings.h"  
#include "SOCCalculator.h"

extern SOCCalculator socCalculator;


void ContactorController::init() {
    currentState = OPEN;
}

void ContactorController::open() {
    digitalWrite(PRECHARGE_RELAY_PIN, LOW);
    digitalWrite(CONTACTOR_RELAY_PIN, LOW);
    currentState = OPEN;
}

void ContactorController::close() {
    if (currentState == CONNECTED) {
        return;                    // Already closed - do nothing
    }

	prechargeStartTime = millis(); //note time and start pre-charge sequence (doesnt finish here, just starts it)

    if (eepromdata.prechargeEnabled) {
        digitalWrite(PRECHARGE_RELAY_PIN, HIGH);
        currentState = PRECHARGING;
    }
    else {
        digitalWrite(CONTACTOR_RELAY_PIN, HIGH);
        currentState = CONNECTED;
    }
}

void ContactorController::update() {
    unsigned long now = millis();

    switch (currentState) {
    case PRECHARGING: {
        bool prechargeDone = false;

        if (!eepromdata.prechargeEnabled) {
            prechargeDone = true;
        }
        else if (eepromdata.currentSensorPresent) {
            float current = socCalculator.getPackCurrentAmps();
            if (fabsf(current) < 0.5f || (now - prechargeStartTime >= eepromdata.prechargeTimeoutMs)) {
                prechargeDone = true;
            }
        }
        else {
            // timed fallback when no current sensor
            if (now - prechargeStartTime > eepromdata.prechargeTimeoutMs) {
                prechargeDone = true;
            }
        }

        // safety timeout
        if (now - prechargeStartTime > eepromdata.prechargeTimeoutMs) {
            open();
            currentState = FAULT;
            return;
        }

        if (prechargeDone) {
            digitalWrite(CONTACTOR_RELAY_PIN, HIGH);
            postCloseDelayStart = now;
            currentState = CONNECTED;
        }
        break;
    }

    case CONNECTED:
        // Turn off small pre-charge relay after 500 ms
        if (now - postCloseDelayStart > 500) {
            digitalWrite(PRECHARGE_RELAY_PIN, LOW);
        }
        break;

    case OPEN:
    case FAULT:
        break;
    }
}

ContactorState ContactorController::getState() const {
    return currentState;
}

