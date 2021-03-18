/*
 * Ads1x15.h
 *
 *  Created on: 5 Jul 2020
 *      Author: jp112sdl
 *
 *      https://www.luisllamas.es/arduino-sensor-corriente-sct-013/
 */

#ifndef SENSORS_ADS1X15_H_
#define SENSORS_ADS1X15_H_

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

  // ads.setGain(GAIN_TWOTHIRDS);  +/- 6.144V  1 bit = 0.1875mV (default)
  // ads.setGain(GAIN_ONE);        +/- 4.096V  1 bit = 0.125mV
  // ads.setGain(GAIN_TWO);        +/- 2.048V  1 bit = 0.0625mV
  // ads.setGain(GAIN_FOUR);       +/- 1.024V  1 bit = 0.03125mV
  // ads.setGain(GAIN_EIGHT);      +/- 0.512V  1 bit = 0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    +/- 0.256V  1 bit = 0.0078125mV
namespace as {

template <uint8_t ADDRESS>
class Sens_Ads1x15 {
private:
  Adafruit_ADS1115 ads;
  int16_t val_0_1;
  int16_t val_2_3;
  bool present;
public:

  Sens_Ads1x15 () : val_0_1(0), val_2_3(0), present(false) {}
  ~Sens_Ads1x15() {}

  void init (adsGain_t gain) {
    ads.begin(ADDRESS);
    Wire.beginTransmission(ADDRESS);
    DPRINT(F("ADS1115 init at address "));DHEX(ADDRESS);
    if (Wire.endTransmission() == 0) {
      ads.setGain(gain);
      DPRINT(F(" OK, gain "));DDECLN(gain);;
      present = true;
    } else {
      DPRINTLN(F(" FAILED!"));
      present = false;
    }
  }

  bool checkSensor() {
    Wire.beginTransmission(ADDRESS);
    bool isOk = (Wire.endTransmission() == 0);
    //DPRINT("checkSensor() ");DHEX(ADDRESS);DPRINT(" is ");DPRINTLN(isOk ? "OK":"NOT OK");
    present = isOk;
    return isOk;
  }

  int16_t rawToVolt(int16_t in) {
    switch (ads.getGain()) {
      case (GAIN_TWOTHIRDS):
        return in / 5.3F;
      case (GAIN_ONE):
        return in / 8;
      case (GAIN_TWO):
        return in / 16;
      case (GAIN_FOUR):
        return in / 32;
      case (GAIN_EIGHT):
        return in / 64;
      case (GAIN_SIXTEEN):
        return in / 128;
     default:
      return in;
    }
  }

  uint32_t getCurrent(bool adc_0_1, uint16_t sampleTimeMS, uint8_t factor) {
    if (present) {
      long ms = millis();
      int32_t voltage = 0;
      int32_t current = 0;
      int32_t sum = 0;
      int16_t counter = 0;
      while (millis() - ms < sampleTimeMS)
      {
        voltage = rawToVolt(adc_0_1 ? ads.readADC_Differential_0_1():ads.readADC_Differential_2_3());
        current = voltage * factor;
        current /= 10.0;

        sum += sq(current);
        counter++;
       }
      current = sqrt(sum / counter);
      return (current);
    } else {
      return 0;
    }
  }

  uint32_t getCurrent_0_1(uint16_t sampleTimeMS, uint8_t factor) {
     return getCurrent(true,sampleTimeMS, factor);
  }

  uint32_t getCurrent_2_3(uint16_t sampleTimeMS, uint8_t factor) {
     return getCurrent(false,sampleTimeMS, factor);
  }

  int16_t read_0_1 () {
    if (present == true) {
    val_0_1 = ads.readADC_Differential_0_1();
    DPRINT(F("Differential: ")); DDEC(val_0_1); DPRINT(F(" (")); DDEC(rawToVolt(val_0_1)); DPRINTLN(F("mV)"));
    return val_0_1;
    } else {
      return 0;
    }
  }

  int16_t read_2_3 () {
    if (present == true) {
      val_2_3 = ads.readADC_Differential_2_3();
      DPRINT(F("Differential: ")); DDEC(val_2_3); DPRINT(F(" (")); DDEC(rawToVolt(val_2_3)); DPRINTLN(F("mV)"));
      return val_2_3;
    } else {
      return 0;
    }
}

};

}


#endif /* SENSORS_ADS1X15_H_ */
