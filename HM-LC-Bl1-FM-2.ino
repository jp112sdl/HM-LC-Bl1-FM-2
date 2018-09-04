//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2017-12-14 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include "Blind2.h"

// we use a Pro Mini
// Arduino pin for the LED
#define LED_PIN 5
// Arduino pin for the config button
// B0 == PIN 8 on Pro Mini
#define CONFIG_BUTTON_PIN 8

#define ON_RELAY_PIN 14
#define DIR_RELAY_PIN 15

#define ON_RELAY2_PIN 16
#define DIR_RELAY2_PIN 17

#define UP_BUTTON_PIN 7
#define DOWN_BUTTON_PIN 4

#define UP_BUTTON2_PIN 6
#define DOWN_BUTTON2_PIN 3

// number of available peers per channel
#define PEERS_PER_CHANNEL 6

// all library classes are placed in the namespace 'as'
using namespace as;

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0x00, 0x05, 0xaf},            // Device ID
  "JPBL200001",                  // Device Serial
  {0x00, 0x05},                  // Device Model
  0x24,                          // Firmware Version
  as::DeviceType::BlindActuator, // Device Type
  {0x01, 0x00}                   // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AvrSPI<10, 11, 12, 13> RadioSPI;
typedef AskSin<StatusLed<LED_PIN>, NoBattery, Radio<RadioSPI, 2> > Hal;

DEFREGISTER(BlindReg0, MASTERID_REGS, DREG_INTKEY, DREG_CONFBUTTONTIME, DREG_LOCALRESETDISABLE)

class BlindList0 : public RegList0<BlindReg0> {
  public:
    BlindList0 (uint16_t addr) : RegList0<BlindReg0>(addr) {}
    void defaults () {
      clear();
      // intKeyVisible(false);
      confButtonTime(0xff);
      // localResetDisable(false);
    }
};

class BlChannel : public BlindChannel<Hal, PEERS_PER_CHANNEL, BlindList0> {
  public:
    typedef BlindChannel<Hal, PEERS_PER_CHANNEL, BlindList0> BaseChannel;
    BlChannel () {}
    virtual ~BlChannel () {}

    virtual void switchState(uint8_t oldstate, uint8_t newstate, uint32_t stateDelay) {
      BaseChannel::switchState(oldstate, newstate, stateDelay);
      if ( newstate == AS_CM_JT_RAMPON && stateDelay > 0 ) {
        motorUp();
      }
      else if ( newstate == AS_CM_JT_RAMPOFF && stateDelay > 0 ) {
        motorDown();
      }
      else {
        motorStop();
      }
    }

    void motorUp () {
      digitalWrite(dir_relay_pin, HIGH);
      digitalWrite(on_relay_pin, HIGH);
    }

    void motorDown () {
      digitalWrite(dir_relay_pin, LOW);
      digitalWrite(on_relay_pin, HIGH);
    }

    void motorStop () {
      digitalWrite(dir_relay_pin, LOW);
      digitalWrite(on_relay_pin, LOW);
    }

    void init (uint8_t op, uint8_t dp) {
      pinMode(op, OUTPUT);
      pinMode(dp, OUTPUT);
      motorStop();
      BaseChannel::init(op, dp);
    }
};

// setup the device with channel type and number of channels
typedef MultiChannelDevice<Hal, BlChannel, 2, BlindList0> BlindType;

Hal hal;
BlindType sdev(devinfo, 0x20);
ConfigButton<BlindType> cfgBtn(sdev);
InternalButton<BlindType> btnup(sdev, 1);
InternalButton<BlindType> btndown(sdev, 2);
InternalButton<BlindType> btnup2(sdev, 3);
InternalButton<BlindType> btndown2(sdev, 4);

void initPeerings (bool first) {
  // create internal peerings - CCU2 needs this
  if ( first == true ) {
    HMID devid;
    sdev.getDeviceID(devid);
    Peer p1(devid, 1);
    Peer p2(devid, 2);
    Peer p3(devid, 3);
    Peer p4(devid, 4);
    sdev.channel(1).peer(p1, p2);
    sdev.channel(2).peer(p3, p4);
  }
}

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  //storage().setByte(0,0);
  bool first = sdev.init(hal);
  sdev.channel(1).init(ON_RELAY_PIN, DIR_RELAY_PIN);
  sdev.channel(2).init(ON_RELAY2_PIN, DIR_RELAY2_PIN);

  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  buttonISR(btnup, UP_BUTTON_PIN);
  buttonISR(btndown, DOWN_BUTTON_PIN);
  buttonISR(btnup2, UP_BUTTON2_PIN);
  buttonISR(btndown2, DOWN_BUTTON2_PIN);

  initPeerings(first);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    hal.activity.savePower<Idle<> >(hal);
  }
}
