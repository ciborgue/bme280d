#include <cstring>
#include <stdexcept>

#include <unistd.h>
#include <stdint.h>
#include <syslog.h>

#include <wiringPiI2C.h>

#include "config.h"
#include "I2CSETUP.h"
#include "BME280.h"

// #define BME280_DEBUG_LOGGING

using namespace std;

BME280::BME280(I2CSETUP setup) : i2c(setup) {}
void BME280::waitForStatus() {
  for (int i = 0; i < DEFAULT_RETRY_COUNT; i++) {
    if ((wiringPiI2CReadReg8(fd, 0xf3) & 0x01) == 0) {
      return;
    }
    usleep(DEFAULT_USEC_DELAY);
  }
  throw runtime_error("Timeout waiting for BME280 status");
}
void BME280::readCalTbl() {
  calTbl.raw[0] = 0x88; // 1st part
  write(fd, calTbl.raw, 1);
  read(fd, calTbl.raw, 26);

  calTbl.raw[26] = 0xe1; // 2nd part
  write(fd, calTbl.raw + 26, 1);
  read(fd, calTbl.raw + 26, 8);

  calTbl.dig_H4 = (calTbl.raw[29] << 4) | (calTbl.raw[30] & 0x0f);
  calTbl.dig_H5 = (calTbl.raw[32] << 4) | ((calTbl.raw[31] & 0xf0) >> 4);
}
void BME280::acquireOnce() {
  uint8_t buffer[8];

  waitForStatus();
  buffer[0] = 0xf7; // read from 0xF7 towards FE
  write(fd, buffer, 1);
  read(fd, buffer, 8);

  adc_P = (buffer[2] >> 4) | (buffer[1] << 4) | (buffer[0] << 12);
  adc_T = (buffer[5] >> 4) | (buffer[4] << 4) | (buffer[3] << 12);
  adc_H = buffer[7] | (buffer[6] << 8);
#ifdef BME280_DEBUG_LOGGING
	syslog(LOG_MAKEPRI(LOG_USER, LOG_DEBUG), "acquireOnce: T:%5x H:%5x P:%5x",
			adc_T, adc_H, adc_P);
#endif
}
void BME280::acquireData() {
  if ((fd = wiringPiI2CSetup(i2c.address)) == -1) {
    throw runtime_error("can't open I2C bus");
  }
#ifdef BME280_DEBUG_LOGGING
	syslog(LOG_MAKEPRI(LOG_USER, LOG_DEBUG), "acquireData: resetting chip");
#endif
  wiringPiI2CWriteReg8(fd, 0xe0, 0xb6); // reset
  if (wiringPiI2CReadReg8(fd, 0xd0) != 0x60) {
    throw runtime_error("BME280 reg# 0xd0 != 0x60; is chip installed?");
  }
#ifdef BME280_DEBUG_LOGGING
	syslog(LOG_MAKEPRI(LOG_USER, LOG_DEBUG), "acquireData: chip ID is correct");
#endif

  // This is chip setup; refer to the original documentation for the values
  wiringPiI2CWriteReg8(fd, 0xf2, 0x03); // humidity oversampling x4
  wiringPiI2CWriteReg8(fd, 0xf4, 0x6f); // 011 011 11 (T=x4, P=x4, Mode=Normal)
  wiringPiI2CWriteReg8(fd, 0xf5, 0x50); // 010 100 0 0 (Tsb Filter 0 SPI)
#ifdef BME280_DEBUG_LOGGING
	syslog(LOG_MAKEPRI(LOG_USER, LOG_DEBUG), "acquireData: chip configured");
#endif

  readCalTbl();
#ifdef BME280_DEBUG_LOGGING
	syslog(LOG_MAKEPRI(LOG_USER, LOG_DEBUG), "acquireData: retrieved calTbl");
#endif

  for (int i = 0; i < DEFAULT_RETRY_COUNT; i++) {
    usleep(DEFAULT_USEC_DELAY);
    acquireOnce();
    if (adc_P != 0x80000 || adc_T != 0x80000 || adc_H != 0x8000) {
      goto readOK;
    }
  }
	throw runtime_error("BME280 output stuck; is sensor damaged?");
readOK:
	close(fd);
  tmstamp = time(NULL); // last time reading acquired
}
double BME280::t_fine() {
  double var1 = (adc_T / 16384.0 - calTbl.dig_T1 / 1024.0) * calTbl.dig_T2;
  double var2 = ((adc_T / 131072.0 - calTbl.dig_T1 / 8192.0) *
    (adc_T / 131072.0 - calTbl.dig_T1 / 8192.0)) * calTbl.dig_T3;
  return var1 + var2;
}
double BME280::compensate_T() {
  return t_fine() / 5120.0;
}
double BME280::compensate_H() {
  double var_H = (t_fine() - 76800.0);
  var_H = (adc_H - (calTbl.dig_H4 * 64.0 + calTbl.dig_H5 / 16384.0 * var_H)) *
    (calTbl.dig_H2 / 65536.0 * (1.0 + calTbl.dig_H6 / 67108864.0 * var_H *
    (1.0 + calTbl.dig_H3 / 67108864.0 * var_H)));
  var_H = var_H * (1.0 - calTbl.dig_H1 * var_H / 524288.0);
  if (var_H > 100.0) {
    var_H = 100.0;
  } else if (var_H < 0.0) {
    var_H = 0.0;
  }
  return var_H;
}
double BME280::compensate_P() {
  double var1 = (t_fine() / 2.0) - 64000.0;
  double var2 = var1 * var1 * calTbl.dig_P6 / 32768.0;
  var2 = var2 + var1 * calTbl.dig_P5 * 2.0;
  var2 = (var2 / 4.0) + (calTbl.dig_P4 * 65536.0);
  var1 = (calTbl.dig_P3 * var1 * var1 / 524288.0 + calTbl.dig_P2 * var1)
    / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * calTbl.dig_P1;
  if (var1 == 0.0) {
    return 0; // avoid exception caused by division by zero
  }
  double p = 1048576.0 - adc_P;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = calTbl.dig_P9 * p * p / 2147483648.0;
  var2 = p * calTbl.dig_P8 / 32768.0;
  p = p + (var1 + var2 + calTbl.dig_P7) / 16.0;
  return p;
}
uint32_t BME280::getRCID() {
  return (((uint32_t) i2c.channel) << 8) | i2c.address;
}
double BME280::getTemperature() {
  return compensate_T();
}
double BME280::getHumidity() {
  return compensate_H();
}
double BME280::getPressure() {
  return compensate_P();
}
const char *BME280::toString() {
  strncpy(text, "tm: ", sizeof text);

  int out = strlen(text);
  strftime(text + out, sizeof text - out, "%FT%TZ", localtime(&tmstamp));

  out = strlen(text);
  snprintf(text + out, sizeof text - out,
      "; ch:%02d; ad:%02X;"
      " %+.2fC; %.2f%%; %.1fmb",
      i2c.channel, i2c.address,
      getTemperature(), getHumidity(), getPressure() / 100);
  return text;
}
const char *BME280::toJSON() {
  snprintf(text, sizeof text,
      "\"BMA0280%02X%02X\": {\"timestamp\": ",
      i2c.channel, i2c.address);

  int start = strlen(text);
  strftime(text + start, sizeof text - start,
      "\"%FT%TZ\"", localtime(&tmstamp));

  start = strlen(text);
  snprintf(text + start, sizeof text - start,
      ", \"temperature\": %.5f, \"humidity\": %.5f, \"pressure\": %.5f}",
      getTemperature(), getHumidity(), getPressure() / 100);
  return text;
}
