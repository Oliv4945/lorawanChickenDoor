#include <Arduino.h>
#include <avr/wdt.h>
#include "Wire.h"
#include "Max44009.h"
#include "CayenneLPP.h"
#include "LowPower.h"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>


// Enum
enum doorStates_t  {
    doorClosed,
    doorOpened,
    doorUndefined
};

// Constants
#define PIN_VCC A3
const uint8_t luxSensorAddress   = 0xCA;
const uint8_t lppChannelLuxValue = 0;
const uint8_t lppChannelLuxError = 1;

// Globals
uint16_t txInterval;
uint8_t doorState;
uint16_t doorMovementTimeMs;

// Pins
const uint8_t pinMotorSpeed = 8;
const uint8_t pinMotorDir   = 7;
const uint8_t pinMosfet     = 5;

// Structures
struct lux {
    uint16_t value;
    int error;
};


// Functions
void updateLuxSensor(lux *sensor);
void onEvent (ev_t ev);
void do_send(osjob_t* j);
float readVoltage_f(int pin);
void doorClose();
void doorOpen();


// Objects
Max44009 luxSensor(luxSensorAddress);
static osjob_t sendjob;
CayenneLPP lpp(51);


// LoRaWAN configuration
// Little-endian format, LSB first
// For TTN issued APPEUI, the last bytes should be 0xD5, 0xB3,0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0xD5, 0xB3, 0x70 };
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Big endian format
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LMIC_UNUSED_PIN,
    .dio = {2, 3, 4}
};

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); }



// Arduino setup procedure
// Called once at startup
void setup() {
    // Watchdog
    wdt_enable(WDTO_8S);
    wdt_reset();
    // Serial init
    Serial.begin(57600);
    Serial.print("Starting ");

    pinMode(pinMotorSpeed, OUTPUT); // A
    pinMode(pinMotorDir, OUTPUT);   // B
    digitalWrite(pinMotorSpeed, LOW);
    digitalWrite(pinMotorDir, LOW);
    pinMode(pinMosfet, OUTPUT);
    digitalWrite(pinMosfet, LOW);

    // Always assume opened door at start
    doorState = doorOpened;
    txInterval = 5*60;
    doorMovementTimeMs = 600;

    // LMIC init
    wdt_reset();
    os_init();
    LMIC_reset();
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // LMIC_setDrTxpow(DR_SF12, 14);
    LMIC_setAdrMode(1);
    // Start job
    do_send(&sendjob);
}

// Called by Arduino after setup()
void loop() {
    // local variables

    // Actual loop
    while (1) {

        // Clear watchdog
        wdt_reset();
        // Update mac
        os_runloop_once();
    }
}


void updateLuxSensor(lux *sensor) {
    wdt_reset();
    sensor->value = (uint16_t) luxSensor.getLux();
    sensor->error = luxSensor.getError();
    Serial.print(F("Lux: "));
    Serial.print(sensor->value);
    Serial.print(F(" lux - error: "));
    Serial.println(sensor->error);
}



void onEvent (ev_t ev) {
    wdt_reset();
    uint16_t sleepTime = 0;
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            /*{
              
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("artKey: ");
              for (uint8_t i=0; i<sizeof(artKey); ++i) {
                Serial.print(artKey[i], HEX);
              }
              Serial.println("");
              Serial.print("nwkKey: ");
              for (uint8_t i=0; i<sizeof(nwkKey); ++i) {
                Serial.print(nwkKey[i], HEX);
              }
              Serial.println("");
            }*/
            // Disable link check validation.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen > 0) {
                Serial.print(F("Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
                if (LMIC.frame[LMIC.dataBeg-1] == 1) {
                    txInterval = (LMIC.frame[LMIC.dataBeg] << 8) + LMIC.frame[LMIC.dataBeg + 1];
                    Serial.print("New sleep time:");
                    Serial.println(txInterval);
                } else if (LMIC.frame[LMIC.dataBeg-1] == 2) {
                    Serial.println("Closing door");
                    doorClose();
                } else if (LMIC.frame[LMIC.dataBeg-1] == 3) {
                    Serial.println("Opening door");
                    doorClose();
                } else if (LMIC.frame[LMIC.dataBeg-1] == 4) {
                    doorMovementTimeMs = (LMIC.frame[LMIC.dataBeg] << 8) + LMIC.frame[LMIC.dataBeg + 1];
                    Serial.print("New travel time:");
                    Serial.println(doorMovementTimeMs);
                }
            }
            // Enter in sleep mode
            wdt_reset();
            wdt_disable();
            while ( sleepTime < txInterval ) {
                switch (sleepTime) {
                    case 1:
                        LowPower.powerDown( SLEEP_1S, ADC_OFF, BOD_OFF );
                        sleepTime += 1;
                    case 2:
                        LowPower.powerDown( SLEEP_2S, ADC_OFF, BOD_OFF );
                        sleepTime += 2;
                    case 3:
                    case 4:
                        LowPower.powerDown( SLEEP_4S, ADC_OFF, BOD_OFF );
                        sleepTime += 4;
                    default:
                        LowPower.powerDown( SLEEP_8S, ADC_OFF, BOD_OFF );
                        sleepTime += 8;
                }
            }
            wdt_enable(WDTO_4S);
            wdt_reset();
            // Schedule next transmission
            // os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            // Or send now
            do_send(&sendjob);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        /*case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;*/
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}


void do_send(osjob_t* j) {
    lux luxSensor;
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        updateLuxSensor(&luxSensor);
        // Change door state
        if (luxSensor.error == 0) {
            if ((doorState != doorClosed) && (luxSensor.value < 1)) {
                doorClose();
            }
            if ((doorState != doorOpened) && (luxSensor.value > 1)) {
                doorOpen();
            }
        }

        lpp.reset();
        lpp.addLuminosity(lppChannelLuxValue, luxSensor.value);
        lpp.addDigitalOutput(lppChannelLuxError, luxSensor.error);
        lpp.addAnalogOutput(3, readVoltage_f(PIN_VCC));
        lpp.addDigitalInput(4, doorState);
        // LMIC_setTxData2 (u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed)
        uint8_t ackUp = false;
        LMIC_setTxData2(2, lpp.getBuffer(), lpp.getSize(), ackUp);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

float readVoltage_f(int pin) {
    float voltage;
    // Vin = analogRead(pin)*3.3/1024*2*417/406; // Read value*3.3/1024, Times 2 for resistors, *417/406 for calibration
    voltage = ((float) analogRead(pin))*0.0064453125; // TBC - Add calibration value in configuration.h
    delay(1); // Time for ADC to settle
    voltage = ((float) analogRead(pin))*0.0064453125;
    return voltage;
}

void doorOpen() {
    wdt_reset();
    doorState = doorOpened;
    digitalWrite(pinMosfet, HIGH);

    uint16_t remainingTime = doorMovementTimeMs;
    digitalWrite(pinMotorSpeed, HIGH);

    while ( remainingTime > 2000 ) {
        delay(2000);
        remainingTime -= 2000;
        wdt_reset();
    };
    delay(remainingTime);
    wdt_reset();

    digitalWrite(pinMotorSpeed, LOW);
    digitalWrite(pinMosfet, LOW);
}

void doorClose() {
    wdt_reset();
    doorState = doorClosed;
    digitalWrite(pinMosfet, HIGH);

    uint16_t remainingTime = doorMovementTimeMs;
    digitalWrite(pinMotorDir, HIGH);

    while ( remainingTime > 2000 ) {
        delay(2000);
        remainingTime -= 2000;
        wdt_reset();
    };
    delay(remainingTime);
    wdt_reset();

    digitalWrite(pinMotorDir, LOW);
    digitalWrite(pinMosfet, LOW);
}
