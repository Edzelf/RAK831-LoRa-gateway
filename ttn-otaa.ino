/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

//#define HELTEC
#define WEMOS
//#define KPN
#define TTN


#ifdef HELTEC
#define SS    18
#define RESET 14
#define DIO0  26
#define DIO1  33
#define DIO2  32
#define ID    "Heltec"
#endif

#ifdef WEMOS
#define SS    15
#define RESET 0xFF
#define DIO0  4 
#define DIO1  5
#define DIO2  0xFF
#define ID    "Wemos"
#endif



//// LoRaWAN end-device address (DevAddr)
//#ifdef KPN
//static const u4_t DEVADDR = 0x14203841 ;
//#endif
//#ifdef TTN
//static const u4_t DEVADDR = 0x26011D00 ;
//#endif

osjob_t initjob;

static const u1_t APPEUI[8]={ 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x00, 0x9A, 0xC6 } ;
void os_getArtEui (u1_t* buf)
{
  Serial.println ( "os_getArtEui called" ) ;
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = APPEUI[7-i] ;
  }
}

static const u1_t DEVEUI[8]= { 0x0C, 0x80, 0xAD, 0x05, 0x50, 0xA4, 0x0A, 0xE0 } ;
void os_getDevEui (u1_t* buf) 
{
  Serial.println ( "os_getDevEui called" ) ;
  for ( int i = 0 ; i < 8 ; i++ )
  {
    buf[i] = DEVEUI[7-i] ;
  }
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t APPKEY[16] = { 0xC1, 0x14, 0xC8, 0x82, 0xF4, 0x1E, 0x35, 0x10, 0xFF, 0x29, 0x09, 0x4C, 0x23, 0x0F, 0xA1, 0xC6 } ;
void os_getDevKey (u1_t* buf)
{
  Serial.println ( "Unexpected call to os_getDevKey" ) ;
  memcpy_P(buf, APPKEY, 16);
}

static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = SS,
    .rxtx = 0xFF,
    .rst = RESET,
    .dio = { DIO0, DIO1, DIO2 },
} ;


static void initfunc (osjob_t* j) {
    // reset MAC state
    Serial.println ( "Reset MAC" ) ;
    LMIC_reset();
    // start joining
    Serial.println ( "Start joining" ) ;
    LMIC_startJoining();
    Serial.println ( "Initfunc finished" ) ;
    // init done - onEvent() callback will be invoked...
}



void onEvent (ev_t ev)
{
    Serial.println() ;
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin ( 115200 ) ;
    Serial.println(F("\nStarting"));

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif
    // LMIC init
    os_init();
    os_setCallback ( &initjob, initfunc ) ;
    // Reset the MAC state. Session and pending data transfers will be discarded.
    //LMIC_reset();

    // Start job (sending automatically starts OTAA too)
    //do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
