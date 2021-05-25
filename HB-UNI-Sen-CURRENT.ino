//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2020-07-04 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// ci-test=yes board=328p aes=no

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
#define BACKLIGHT_BTN_PIN  6
#endif

#define SENS_ADS1x15
// #define SENS_INA219

#ifdef SENS_ADS1x15
#include "Sensors/Ads1x15.h"
#define ADS1115_ADDR_1 0x4B
#define ADS1115_ADDR_2 0x4A
#define ADS_SENSOR_GAIN   adsGain_t::GAIN_TWO
#define NUM_CHANNELS       3
#endif

#ifdef SENS_INA219
#include "Sensors/INA219_WE.h"
#define NUM_CHANNELS       1
#endif

#define CONFIG_BUTTON_PIN  8
#define LED_PIN            4
#define BUSY_LED_PIN       5

#define PEERS_PER_CHANNEL  4

using namespace as;

//Korrekturfaktor der Clock-Ungenauigkeit, wenn keine RTC verwendet wird
#define SYSCLOCK_FACTOR    0.88

enum conditionTypes { ct_none, ct_above, ct_below, ct_disabled };
typedef struct {
  uint32_t current;
  uint8_t conditionType = ct_none;
  bool ok;
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
typedef AskSin<StatusLed<LED_PIN>, BatterySensor, Radio<AvrSPI<10, 11, 12, 13>, 2>> Hal;
typedef StatusLed<BUSY_LED_PIN> BusyLed;
Hal hal;

DEFREGISTER(DReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x07, 0x1f, 0x20, 0x21, DREG_BACKONTIME, DREG_POWERSUPPLY)
class DevList0 : public RegList0<DReg0> {
  public:
    DevList0 (uint16_t addr) : RegList0<DReg0>(addr) {}

    bool Sendeintervall (uint8_t value) const {
      return this->writeRegister(0x21, value & 0xff);
    }
    uint8_t Sendeintervall () const {
      return this->readRegister(0x21, 0);
    }

    bool Messintervall (uint16_t value) const {
      return this->writeRegister(0x1f, (value >> 8) & 0xff) && this->writeRegister(0x20, value & 0xff);
    }
    uint16_t Messintervall () const {
      return (this->readRegister(0x1f, 0) << 8) + this->readRegister(0x20, 0);
    }

    bool conditionCheckAverage () const {
      return this->readRegister(0x07,0);
    }
    bool conditionCheckAverage (uint8_t value) const {
      return this->writeRegister(0x07,value);
    }

    void defaults () {
      clear();
      lowBatLimit(22);
      Sendeintervall(12);
      Messintervall(10);
      powerSupply(true); //true = battery mode
      backOnTime(10);
      conditionCheckAverage(false);
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
      condTxThresholdHi(1000);
      condTxDecisionAbove(200);
      condTxThresholdLo(100);
      condTxDecisionBelow(0);
      sensorType(0);
      sampleTime(500);
    }
};

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint32_t *sensorValues, _currentSensor *cs, bool batlow) {
      Message::init(0xa + (NUM_CHANNELS * 3), msgcnt, 0x53, BCAST, batlow ? 0x80 : 0x00, cs[0].ok ? 0x41 : 0xC1);
      //DPRINT(F("+Current (#0) : ")); DDECLN(sensorValues[0]);
      pload[0] =  (sensorValues[0] >> 8) & 0xff;
      pload[1] = (sensorValues[0])   & 0xff;
      for (uint8_t s = 0; s < NUM_CHANNELS - 1; s++) {
        //DPRINT(F("+Current (#")); DDEC(s + 1); DPRINT(F(") : ")); DDECLN(sensorValues[s+1]);
        pload[2+(s * 3)] = (cs[s+1].ok ? 0x42 : 0xC2) + s;
        pload[3+(s * 3)] = (sensorValues[s+1] >> 8) & 0xff;
        pload[4+(s * 3)] =  sensorValues[s+1]       & 0xff;
      }
    }
};

#ifdef LCD_ADDRESS
class LcdType :  public Button {
public:
  class BacklightAlarm : public Alarm {
    LcdType& lcdDev;
  public:
    BacklightAlarm (LcdType& l) :  Alarm(0), lcdDev(l) {}
    virtual ~BacklightAlarm () {}
    void restartTimer(uint8_t sec) {
      sysclock.cancel(*this);
      set(seconds2ticks(sec));
      lcdDev.lcd.backlight();
      sysclock.add(*this);
    }

    virtual void trigger (__attribute__((unused)) AlarmClock& clock) {
      lcdDev.lcd.noBacklight();
    }
  }backlightalarm;
private:
  bool _present;
  bool first;
  uint8_t backlightOnTime;

  byte arrowUp[8] = { 0b00000, 0b00100, 0b01110, 0b10101, 0b00100, 0b00100, 0b00100, 0b00000 };
  byte arrowDown[8] = { 0b00000, 0b00100, 0b00100, 0b00100, 0b10101, 0b01110, 0b00100, 0b00000 };

public:
  LiquidCrystal_I2C lcd;
  LcdType () :  backlightalarm(*this), _present(false), first(true), backlightOnTime(10), lcd(LCD_ADDRESS, 16, 2){}
  virtual ~LcdType () {}

  void displayValues(_currentSensor *cs) {
    if (_present == true) {
      if (first) {
        first = false;
       lcd.setCursor(0, 0);
       lcd.print(" L1    L2    L3 ");
      }
     lcd.setCursor(0, 1);
     lcd.print("                ");

     lcd.setCursor(cs[0].current > 999 ? 0 : 1, 1);
     lcd.print(cs[0].current / 100.0, 1);

     lcd.setCursor(cs[1].current > 999 ? 6 : 7, 1);
     lcd.print(cs[1].current / 100.0, 1);

     lcd.setCursor(cs[2].current > 999 ? 12 : 13, 1);
     lcd.print(cs[2].current / 100.0, 1);
    }
  }

  void showCondition(uint8_t conditionType, uint8_t channel) {
    lcd.setCursor(channel * 6 + 3, 0);
    if (conditionType == ct_above) lcd.write(byte(0));
    if (conditionType == ct_below) lcd.write(byte(1));
    if (conditionType == ct_none) lcd.print("*");
    if (conditionType == ct_disabled) lcd.print(" ");
  }

  void initLCD(uint8_t *serial) {
    Wire.begin();
    Wire.beginTransmission(LCD_ADDRESS);
    if (Wire.endTransmission() == 0) {
      _present = true;
      lcd.init();
      lcd.createChar(0, arrowUp);
      // create a new character
      lcd.createChar(1, arrowDown);
      lcd.backlight();
      lcd.setCursor(0, 0);
      lcd.print(ASKSIN_PLUS_PLUS_IDENTIFIER);
      lcd.setCursor(3, 1);
      lcd.setContrast(200);
      lcd.print((char*)serial);

      if (backlightOnTime > 0) backlightalarm.restartTimer(backlightOnTime);

    } else {
      DPRINT("LCD Display not found at 0x");DHEXLN((uint8_t)LCD_ADDRESS);
    }
  }

  void setBackLightOnTime(uint8_t t) {
    backlightOnTime = t;
    if (backlightOnTime == 0)
      lcd.backlight();
    else
      lcd.noBacklight();
  }

  virtual void state(uint8_t s) {
    Button::state(s);
    if (s==released ) {
      if (backlightOnTime > 0) backlightalarm.restartTimer(backlightOnTime);
    }
  }
};
LcdType lcd;
#endif

class MeasureChannel : public Channel<Hal, DevList1, EmptyList, List4, PEERS_PER_CHANNEL, DevList0> {
  private:
    enum sensorTypes { SCT013015, SCT013020, SCT013030, SCT013050, SCT0130100, INA219, ACS712_or_other };
  public:
    MeasureChannel () : Channel() {}
    virtual ~MeasureChannel () {}

    virtual void configChanged() {
    /*  DPRINT("configChanged on Ch");DDECLN(number());
      DPRINT("condTxThresholdHi  : ");DDECLN(this->getList1().condTxThresholdHi());
      DPRINT("condTxDecisionAbove: ");DDECLN(this->getList1().condTxDecisionAbove());
      DPRINT("condTxThresholdLo  : ");DDECLN(this->getList1().condTxThresholdLo());
      DPRINT("condTxDecisionBelow: ");DDECLN(this->getList1().condTxDecisionBelow());
      DPRINT("sensorType         : ");DDECLN(this->getList1().sensorType());
      DPRINT("sampleTime         : ");DDECLN(this->getList1().sampleTime());*/
    }

    void checkConditions(_currentSensor *sensor) {
      uint8_t sIdx = number()-1;

      static uint8_t evcnt  = 0;
      uint8_t decisionValue = 0;
      bool sendConditionalSwitchCommand = false;

      uint32_t current     = sensor[sIdx].current;
      uint16_t thresholdHi = this->getList1().condTxThresholdHi();
      uint16_t thresholdLo = this->getList1().condTxThresholdLo();

      if (thresholdHi > 0) {
        SensorEventMsg& rmsg = (SensorEventMsg&)device().message();

        if (current > thresholdHi && sensor[sIdx].conditionType != ct_above) {
          sensor[sIdx].conditionType = ct_above;
          decisionValue = this->getList1().condTxDecisionAbove();
          sendConditionalSwitchCommand = true;
        }

        if (current < thresholdLo && sensor[sIdx].conditionType != ct_below) {
          sensor[sIdx].conditionType = ct_below;
          decisionValue = this->getList1().condTxDecisionBelow();
          sendConditionalSwitchCommand = true;
        }

        if (sendConditionalSwitchCommand == true) {
          DPRINTLN("sendConditionalSwitchCommand");
          rmsg.init(device().nextcount(), number(), evcnt++, decisionValue, false , false);
          device().sendPeerEvent(rmsg, *this);
        }
#ifdef LCD_ADDRESS
        lcd.showCondition(sensor[sIdx].conditionType, sIdx);
#endif        
      } else {
#ifdef LCD_ADDRESS
        lcd.showCondition(ct_disabled, sIdx);
#endif        
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
    uint8_t flags  () const { return device().battery().low() ? 0x80 : 00; }
};

class DevType : public MultiChannelDevice<Hal, MeasureChannel, NUM_CHANNELS, DevList0> {
private:
  bool powerModeBattery;
  uint16_t txInterval;
  uint16_t measureInterval;
  bool boot;
  bool lcd_present;
  BusyLed busyLed;
public:
  class CurrentSensors : public Alarm {
    DevType&   dev;
    
#ifdef SENS_ADS1x15
    Sens_Ads1x15<ADS1115_ADDR_1> ads1;
    Sens_Ads1x15<ADS1115_ADDR_2> ads2;
#endif

#ifdef SENS_INA219
    INA219_WE ina219;
#endif
    
    _currentSensor cs[NUM_CHANNELS];
    uint32_t cumulatedCurrentValues[NUM_CHANNELS];
    uint8_t measureCount;
    public:
       CurrentSensors (DevType& d) : Alarm(0), dev(d), measureCount(0) {}
       virtual ~CurrentSensors () {}
       void measure() {
         //measurement here:
         if (dev.channel(1).getList1().sensorType() == 5) {
#ifdef SENS_INA219
           cs[0].current = ina219.getCurrent_mA(); 
           cs[0].ok = true;
#endif
         } else {
#ifdef SENS_ADS1x15
           cs[0].current = ads1.getCurrent_0_1(dev.channel(1).sampleTime(), dev.channel(1).sctFactor());cs[0].ok = ads1.checkSensor();
           cs[1].current = ads1.getCurrent_2_3(dev.channel(2).sampleTime(), dev.channel(2).sctFactor());cs[1].ok = ads1.checkSensor();
           cs[2].current = ads2.getCurrent_0_1(dev.channel(3).sampleTime(), dev.channel(3).sctFactor());cs[2].ok = ads2.checkSensor();
#endif
         }  

#ifdef LCD_ADDRESS
         lcd.displayValues(cs);
#endif
       }

       void init() {
         if (dev.channel(1).getList1().sensorType() == 5) {
#ifdef SENS_INA219
             Wire.begin();
             ina219.init();
             ina219.setBusRange(BRNG_16);
             ina219.setADCMode(SAMPLE_MODE_64);
             ina219.setPGain(PG_320); // PG_320 = 3.2A / PG_160 = 1.6A / PG_80 = 0,8A / PG_40 = 0,4A
#endif
         } else {
#ifdef SENS_ADS1x16
             ads1.init(ADS_SENSOR_GAIN);
             ads2.init(ADS_SENSOR_GAIN);
#endif
         }
       }

       virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
         dev.busyLed.ledOn();
         measure();
         dev.busyLed.ledOff();

         // add measured current to the cumulated values
#ifdef SENS_INA219
         cumulatedCurrentValues[0] += cs[0].current;
#endif
#ifdef SENS_ADS1x16
         cumulatedCurrentValues[0] += cs[0].current;
         cumulatedCurrentValues[1] += cs[1].current;
         cumulatedCurrentValues[2] += cs[2].current;
#endif
         measureCount++;
         DPRINT("measure() #");DDEC(measureCount);DPRINT(" of ");DDECLN(dev.txInterval);

         // check if any of the sensors is above/below a threshold
         for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++) {
           // check conditions on each measure() or only on txInterval
           if (dev.getList0().conditionCheckAverage() == false) {
             dev.channel(ch+1).checkConditions(cs);
           } else {
             if (measureCount >= dev.txInterval) {
               cs[ch].current = cumulatedCurrentValues[ch] / measureCount;
               dev.channel(ch+1).checkConditions(cs);
             }
           }
         }

         set(seconds2ticks(max(10, dev.measureInterval * SYSCLOCK_FACTOR)));

         // send the cyclic message 
         if (measureCount >= dev.txInterval) {
           bool batlow = dev.onBattery() ? dev.battery().low() : false;

           // divide cumulated values by measure count
           for (uint8_t ch = 0; ch < NUM_CHANNELS; ch++)
             cumulatedCurrentValues[ch] /= measureCount;
             
           MeasureEventMsg& msg = (MeasureEventMsg&)dev.message();             
           msg.init(dev.nextcount(), cumulatedCurrentValues, cs, batlow);
           dev.broadcastEvent(msg);

           // reset values
           measureCount = 0;
           for (uint8_t s = 0; s < NUM_CHANNELS; s++)
             cumulatedCurrentValues[s] = 0;
         }
         sysclock.add(*this);
       }
    } currentSensors;

  typedef MultiChannelDevice<Hal, MeasureChannel, NUM_CHANNELS, DevList0> TSDevice;
  DevType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr), powerModeBattery(true), txInterval(0), measureInterval(0), boot(true), lcd_present(false), currentSensors(*this) {}
  virtual ~DevType () {}

  void init (Hal& hal) {
    TSDevice::init(hal);
    currentSensors.init();
    busyLed.init();
    busyLed.ledOn(seconds2ticks(2));
  }

  virtual void configChanged () {
    TSDevice::configChanged();
    uint8_t batlow = max(this->getList0().lowBatLimit(),19);
    // DPRINT(F("*LOWBAT Limit: "));DDECLN(batlow);
    this->battery().low(batlow);

    if (this->getList0().Messintervall() != measureInterval) {
      measureInterval = this->getList0().Messintervall();
      //DPRINT(F("*ME Intervall: ")); DDECLN(measureInterval);
      if (boot == false) {
        sysclock.cancel(currentSensors);
        currentSensors.set(seconds2ticks(max(10, measureInterval) * SYSCLOCK_FACTOR));
      } else {
        currentSensors.set(seconds2ticks(3));
      }
      sysclock.add(currentSensors);
    }

    txInterval = max(1, this->getList0().Sendeintervall());
    //DPRINT(F("*TX Intervall: ")); DDECLN(txInterval);

    onBattery(this->getList0().powerSupply());
    //DPRINT(F("*Batterymode : ")); DDECLN(onBattery());

    uint8_t bOn = this->getList0().backOnTime();
    //DPRINT(F("*LCD Backlight Ontime : ")); DDECLN(bOn);
#ifdef LCD_ADDRESS    
    lcd.setBackLightOnTime(bOn);
#endif    

    //bool cc = this->getList0().conditionCheckAverage();
    //DPRINT(F("*Condition Check on Average : ")); DDECLN(cc);

    boot = false;
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
  hal.battery.init(seconds2ticks(60UL * 60), sysclock);
  hal.battery.critical(19);
  sdev.initDone();
#ifdef LCD_ADDRESS
  uint8_t serial[11];
  sdev.getDeviceSerial(serial);
  serial[10]=0;
  lcd.initLCD(serial);
  buttonISR(lcd, BACKLIGHT_BTN_PIN);
#endif
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
