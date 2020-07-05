//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2020-07-04 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#define SENSOR_ONLY

#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>

#include "Sensors/Ads1x15.h"
#define CT_SENSOR_AMPS    30
#define ADS_SENSOR_GAIN   adsGain_t::GAIN_TWO


#define CONFIG_BUTTON_PIN  8
#define LED_PIN            4

#define NUM_CHANNELS       3

#define PEERS_PER_CHANNEL  4

using namespace as;

//Korrekturfaktor der Clock-Ungenauigkeit, wenn keine RTC verwendet wird
#define SYSCLOCK_FACTOR    0.88

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xF3, 0x4E, 0x01},          // Device ID
  "JPCUR00001",                // Device Serial
  {0xF3, 0x4E},                // Device Model
  0x10,                        // Firmware Version
  0x53,                        // Device Type
  {0x01, 0x01}                 // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AskSin<StatusLed<LED_PIN>, BatterySensor, Radio<AvrSPI<10, 11, 12, 13>, 2>> BaseHal;
class Hal : public BaseHal {
  public:
    void init (const HMID& id) {
      BaseHal::init(id);
      // measure battery every 1h
      battery.init(seconds2ticks(60UL * 60), sysclock);
      battery.low(22);
      battery.critical(19);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;

DEFREGISTER(DReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x21, 0x22, DREG_POWERSUPPLY)
class DevList0 : public RegList0<DReg0> {
  public:
    DevList0 (uint16_t addr) : RegList0<DReg0>(addr) {}

    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x21, (value >> 8) & 0xff) && this->writeRegister(0x22, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x21, 0) << 8) + this->readRegister(0x22, 0);
    }

    void defaults () {
      clear();
      lowBatLimit(22);
      Sendeintervall(180);
      powerSupply(true); //true = battery mode
    }
};

DEFREGISTER(CReg1)
class DevList1 : public RegList1<CReg1> {
  public:
    DevList1 (uint16_t addr) : RegList1<CReg1>(addr) {}
    void defaults () {
      clear();
    }
};

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint16_t *current, bool batlow) {
      Message::init(0xa + (NUM_CHANNELS * 3), msgcnt, 0x53, BCAST, batlow ? 0x80 : 0x00, 0x01);
      DPRINT(F("+Current (#0) : ")); DDECLN(current[0]);
      pload[0] =  (current[0] >> 8) & 0xff;
      pload[1] = (current[0])   & 0xff;
      for (uint8_t s = 0; s < NUM_CHANNELS - 1; s++) {
        DPRINT(F("+Current (#")); DDEC(s + 1); DPRINT(F(") : ")); DDECLN(current[s+1]);
        pload[2+(s * 3)] = 0x02 + s;
        pload[3+(s * 3)] = (current[s+1] >> 8) & 0xff;
        pload[4+(s * 3)] =  current[s+1]       & 0xff;
      }
    }
};

class MeasureChannel : public Channel<Hal, DevList1, EmptyList, List4, PEERS_PER_CHANNEL, DevList0> {
  public:
    MeasureChannel () : Channel() {}
    virtual ~MeasureChannel () {}
    uint8_t status () const { return 0; }
    uint8_t flags  () const { return this->device().battery().low() ? 0x80 : 0x00; }
};

class DevType : public MultiChannelDevice<Hal, MeasureChannel, NUM_CHANNELS, DevList0> {
private:
  bool powerModeBattery;
public:
  class CurrentSensors : public Alarm {
    DevType&   dev;
    uint16_t current[NUM_CHANNELS];
    Sens_Ads1x15<0x48, CT_SENSOR_AMPS> ads1;
    Sens_Ads1x15<0x49, CT_SENSOR_AMPS> ads2;
    public:
       CurrentSensors (DevType& d) : Alarm(0), dev(d) {}
       virtual ~CurrentSensors () {}
       void measure() {

         current[0] = ads1.getCurrent_0_1(500);
         current[1] = ads1.getCurrent_2_3(500);
         current[2] = ads2.getCurrent_0_1(500);

         // HIER DIE MESSUNG DURCHFÜHREN
         // Stromstärke muss mit Faktor 100 übertragen werden
         // 5.21A -> current = 521;
         for (uint8_t s = 0; s < NUM_CHANNELS; s++) {
           current[s] = random(23719);
         }
       }

       void init() {
         ads1.init(ADS_SENSOR_GAIN);
         ads2.init(ADS_SENSOR_GAIN);
       }

       virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
         measure();
         tick = seconds2ticks((max(10, dev.getList0().Sendeintervall()) * SYSCLOCK_FACTOR));

         measure();

         MeasureEventMsg& msg = (MeasureEventMsg&)dev.message();
         bool batlow = dev.onBattery() ? dev.battery().low() : false;
         msg.init(dev.nextcount(), current, batlow);
         dev.broadcastEvent(msg);
         sysclock.add(*this);
       }
    } currentsensors;

    typedef MultiChannelDevice<Hal, MeasureChannel, NUM_CHANNELS, DevList0> TSDevice;
    DevType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr), powerModeBattery(true), currentsensors(*this) {}
    virtual ~DevType () {}

    void init (Hal& hal) {
      TSDevice::init(hal);
      currentsensors.set(seconds2ticks(3));
      sysclock.add(currentsensors);
    }

    virtual void configChanged () {
      TSDevice::configChanged();
      uint8_t batlow = max(this->getList0().lowBatLimit(),19);
      DPRINT(F("*LOWBAT Limit: "));
      DDECLN(batlow);
      this->battery().low(batlow);

      DPRINT(F("*TX Intervall: ")); DDECLN(this->getList0().Sendeintervall());

      onBattery(this->getList0().powerSupply());
      DPRINT(F("*Batterymode : ")); DDECLN(onBattery());
    }

    void onBattery(bool bat) {
      powerModeBattery = bat;
    }

    bool onBattery() {
      return powerModeBattery;
    }
};

DevType sdev(devinfo, 0x20);
ConfigButton<DevType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  sdev.init(hal);
  buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
  sdev.initDone();
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    if (sdev.onBattery() == true)
      hal.activity.savePower<Sleep<>>(hal);
    else
      hal.activity.savePower<Idle<>>(hal);
  }
}
