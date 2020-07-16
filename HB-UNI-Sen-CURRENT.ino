//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2020-07-04 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define LCD_ADDRESS 0x27

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#define SENSOR_ONLY

#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>

#ifdef LCD_ADDRESS
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
#endif

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

typedef struct {
  uint16_t current;
  bool aboveThreshold;
  bool belowThreshold;
} _currentSensor;

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

DEFREGISTER(CReg1, CREG_AES_ACTIVE, CREG_COND_TX_THRESHOLD_HI, CREG_COND_TX_THRESHOLD_LO, CREG_COND_TX_DECISION_ABOVE, CREG_COND_TX_DECISION_BELOW, 0x01, 0x02, 0x03)
class DevList1 : public RegList1<CReg1> {
  public:
    DevList1 (uint16_t addr) : RegList1<CReg1>(addr) {}

    bool sampleTime (uint16_t value) const {
      return this->writeRegister(0x01, (value >> 8) & 0xff) && this->writeRegister(0x02, value & 0xff);
    }
    uint16_t sampleTime () const {
      return (this->readRegister(0x01, 0) << 8) + this->readRegister(0x02, 0);
    }

    bool sensorType (uint16_t value) const {
      return this->writeRegister(0x03, value & 0xff);
    }
    uint16_t sensorType () const {
      return this->readRegister(0x03, 0);
    }

    void defaults () {
      clear();
      condTxDecisionAbove(200);
      condTxDecisionBelow(0);
      condTxThresholdHi(1000);
      condTxThresholdLo(0);
      sensorType(0);
      sampleTime(500);
    }
};

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, _currentSensor *sensor, bool batlow) {
      Message::init(0xa + (NUM_CHANNELS * 3), msgcnt, 0x53, BCAST, batlow ? 0x80 : 0x00, 0x01);
      DPRINT(F("+Current (#0) : ")); DDECLN(sensor[0].current);
      pload[0] =  (sensor[0].current >> 8) & 0xff;
      pload[1] = (sensor[0].current)   & 0xff;
      for (uint8_t s = 0; s < NUM_CHANNELS - 1; s++) {
        DPRINT(F("+Current (#")); DDEC(s + 1); DPRINT(F(") : ")); DDECLN(sensor[s+1].current);
        pload[2+(s * 3)] = 0x02 + s;
        pload[3+(s * 3)] = (sensor[s+1].current >> 8) & 0xff;
        pload[4+(s * 3)] =  sensor[s+1].current       & 0xff;
      }
    }
};

class MeasureChannel : public Channel<Hal, DevList1, EmptyList, List4, PEERS_PER_CHANNEL, DevList0> {
  private:
    enum sensorTypes { SCT013015, SCT013020, SCT013030, SCT013050, SCT0130100, INA219, ACS712_or_other };
  public:
    MeasureChannel () : Channel() {}
    virtual ~MeasureChannel () {}

    virtual void configChanged() {
      DPRINT("configChanged on Ch");DDECLN(number());
      DPRINT("condTxThresholdHi  : ");DDECLN(this->getList1().condTxThresholdHi());
      DPRINT("condTxDecisionAbove: ");DDECLN(this->getList1().condTxDecisionAbove());
      DPRINT("condTxThresholdLo  : ");DDECLN(this->getList1().condTxThresholdLo());
      DPRINT("condTxDecisionBelow: ");DDECLN(this->getList1().condTxDecisionBelow());
      DPRINT("sensorType         : ");DDECLN(this->getList1().sensorType());
      DPRINT("sampleTime         : ");DDECLN(this->getList1().sampleTime());
    }

    void checkConditions(_currentSensor *sensor) {
      uint8_t sIdx = number()-1;

      static uint8_t evcnt  = 0;
      uint8_t decisionValue = 0;
      bool sendConditionalSwitchCommand = false;

      uint16_t current     = sensor[sIdx].current;
      uint16_t thresholdHi = this->getList1().condTxThresholdHi();
      uint16_t thresholdLo = this->getList1().condTxThresholdLo();

      if (thresholdHi > 0) {
        SensorEventMsg& rmsg = (SensorEventMsg&)device().message();

        if (current > thresholdHi && sensor[sIdx].aboveThreshold == false) {
          sensor[sIdx].aboveThreshold = true;
          sensor[sIdx].belowThreshold = false;
          decisionValue = this->getList1().condTxDecisionAbove();
          sendConditionalSwitchCommand = true;
        }

        if (current < thresholdLo && sensor[sIdx].belowThreshold == false) {
          sensor[sIdx].belowThreshold = true;
          sensor[sIdx].aboveThreshold = false;
          decisionValue = this->getList1().condTxDecisionBelow();
          sendConditionalSwitchCommand = true;
        }

        if (sendConditionalSwitchCommand == true) {
          DPRINTLN("sendConditionalSwitchCommand");
          rmsg.init(device().nextcount(), number(), evcnt++, decisionValue, false , false);
          device().sendPeerEvent(rmsg, *this);
        }
      }
    }

    uint8_t sctFactor() {
      uint8_t sensorType = this->getList1().sensorType();
      switch (sensorType) {
        case SCT013015:
          return 15;
        case SCT013020:
          return 20;
        case SCT013030:
          return 30;
        case SCT013050:
          return 50;
        case SCT0130100:
          return 100;
        default:
          return 0;
      }
    }

    uint16_t sampleTime() {
      return this->getList1().sampleTime();
    }

    uint8_t status () const { return 0; }
    uint8_t flags  () const { return this->device().battery().low() ? 0x80 : 0x00; }
};

class DevType : public MultiChannelDevice<Hal, MeasureChannel, NUM_CHANNELS, DevList0> {
private:
  bool powerModeBattery;
  uint16_t txInterval;
  bool boot;
public:
  class CurrentSensors : public Alarm {
    DevType&   dev;
    Sens_Ads1x15<0x48> ads1;
    Sens_Ads1x15<0x49> ads2;
    _currentSensor cs[NUM_CHANNELS];
    public:
       CurrentSensors (DevType& d) : Alarm(0), dev(d) {}
       virtual ~CurrentSensors () {}
       void measure() {

         // HIER DIE MESSUNG DURCHFÜHREN
         // Stromstärke muss mit Faktor 100 übertragen werden
         // 5.21A -> current = 521;

         cs[0].current = ads1.getCurrent_0_1(dev.channel(1).sampleTime(), dev.channel(1).sctFactor());
         cs[1].current = ads1.getCurrent_2_3(dev.channel(2).sampleTime(), dev.channel(2).sctFactor());
         cs[2].current = ads2.getCurrent_0_1(dev.channel(3).sampleTime(), dev.channel(3).sctFactor());

         for (uint8_t s = 0; s < NUM_CHANNELS; s++) {
           uint16_t c = random(5000);
           DPRINT(F("+Current: ")); DDECLN(c);
           cs[s].current = c;
         }

#ifdef LCD_ADDRESS
         lcd.setCursor(0, 0);
         lcd.print(" L1    L2    L3 ");

         lcd.setCursor(0, 1);
         lcd.print("                ");

         lcd.setCursor(cs[0].current > 999 ? 0 : 1, 1);
         lcd.print(cs[0].current / 100.0, 1);

         lcd.setCursor(cs[1].current > 999 ? 6 : 7, 1);
         lcd.print(cs[1].current / 100.0, 1);

         lcd.setCursor(cs[2].current > 999 ? 12 : 13, 1);
         lcd.print(cs[2].current / 100.0, 1);
#endif
       }

       void init() {
         ads1.init(ADS_SENSOR_GAIN);
         ads2.init(ADS_SENSOR_GAIN);
       }

       virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
         measure();

         for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
           dev.channel(ch+1).checkConditions(cs);

         set(seconds2ticks(max(10, dev.txInterval * SYSCLOCK_FACTOR)));

         MeasureEventMsg& msg = (MeasureEventMsg&)dev.message();
         bool batlow = dev.onBattery() ? dev.battery().low() : false;
         msg.init(dev.nextcount(), cs, batlow);
         dev.broadcastEvent(msg);
         sysclock.add(*this);
       }
    } currentsensors;

  typedef MultiChannelDevice<Hal, MeasureChannel, NUM_CHANNELS, DevList0> TSDevice;
  DevType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr), powerModeBattery(true), txInterval(0), boot(true), currentsensors(*this) {}
  virtual ~DevType () {}

  void init (Hal& hal) {
    TSDevice::init(hal);

#ifdef LCD_ADDRESS
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print(ASKSIN_PLUS_PLUS_IDENTIFIER);
    lcd.setCursor(3, 1);
    lcd.setContrast(200);
    uint8_t serial[11];
    getDeviceSerial(serial);
    serial[10]=0;
    lcd.print((char*)serial);
#endif
    }

  virtual void configChanged () {
    TSDevice::configChanged();
    uint8_t batlow = max(this->getList0().lowBatLimit(),19);
    DPRINT(F("*LOWBAT Limit: "));
    DDECLN(batlow);
    this->battery().low(batlow);

    if (this->getList0().Sendeintervall() != txInterval) {
      txInterval = this->getList0().Sendeintervall();
      DPRINT(F("*TX Intervall: ")); DDECLN(txInterval);
      if (boot == false) {
        sysclock.cancel(currentsensors);
        currentsensors.set(seconds2ticks(max(10, txInterval) * SYSCLOCK_FACTOR));
      } else {
        currentsensors.set(seconds2ticks(3));
        boot = false;
      }
      sysclock.add(currentsensors);
    }

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
