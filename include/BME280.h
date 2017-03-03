#ifndef __BME280_H__
#define __BME280_H__
#include <stdint.h>
#include "I2CSETUP.h"

class BME280 {
  private:
    union { // can't be static as different chips have different tables
      unsigned char raw[64];
      struct {
        uint16_t    dig_T1; // +00 (@0x88); first half
        int16_t     dig_T2;
        int16_t     dig_T3;

        uint16_t    dig_P1;
        int16_t     dig_P2;
        int16_t     dig_P3;
        int16_t     dig_P4;
        int16_t     dig_P5;
        int16_t     dig_P6;
        int16_t     dig_P7;
        int16_t     dig_P8;
        int16_t     dig_P9;
        uint8_t     unused;
        uint8_t     dig_H1; // +25

        int16_t     dig_H2; // +26 (@0xE1); second half
        uint8_t     dig_H3; // +28
        int16_t     dig_H4; // +30
        int16_t     dig_H5; // +32
        int8_t      dig_H6;
      };
    } calTbl;
    char text[256]; // toString & toJSON buffer

		int fd = -1;

    time_t tmstamp = 0;
    int32_t adc_T, adc_H, adc_P; // ADC data; updated by acquireData

    void waitForStatus(); // wait while samples are updated to registers
    void acquireOnce(); // actually read data from the chip
    void readCalTbl(); // load calibration table from the chip

    double t_fine(); // 'internal' temperature; used in H & P calculations
    double compensate_T(); // internal naming convension from Bosch
    double compensate_H();
    double compensate_P();
  public:
    BME280(I2CSETUP&);
    ~BME280() {}

    I2CSETUP i2c;

    void acquireData(); // use acquireOnce and validate

    uint32_t getRCID();

    double getTemperature();
    double getHumidity();
    double getPressure();

    virtual const char *toString();
    virtual const char *toJSON();
};
#endif // __BME280_H__
