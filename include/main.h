#ifndef MAIN_H
#define MAIN_H

#include "rtos.h"
#include "mbed.h"
#include "pinDef.h"
#include "INA229/INA229.h"

#define CONFIG_SET (0x01 << 4)
#define CONFIG_ADC_SET 0xFB6B
#define SHUNT_CALIBRATION 0xBB8
#define CURRENT_LSB_CALIBRATION 0.000029
#define OVERVOLTAGE_BUS_THRESHOLD 0x1A40
#define UNDERVOLTAGE_BUS_THRESHOLD 0xD80
#define OVERTEMPERATURE_THRESHOLD 0x2800

#define NEUTRAL_PWM 1500
#define MAX_PWM 1900
#define MIN_PWM 1100

//Add mbed pins definitions
SPI spi(SPI_MOSI, SPI_MISO, SPI_SCLK);
DigitalOut chip_select(SPI_CS);
DigitalOut led2(LED2);

PwmOut pwm(PWM_OUT);
DigitalOut in(IN);

#endif