/**
 * @file power_management.h
 * @author Francis Alonzo (francisalonzo29@gmail.com)
 * @brief Power Management
 * @version 0.1
 * @date 2022-01-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "rtos.h"
#include "mbed.h"
#include "pinDef.h"
#include "address_I2C.h"
#include "Utility/utility.h"
#include "PCA9531/PCA9531.h"
#include "RS485/RS485.h"
#include "RS485/RS485_definition.h"
#include "SD_Card/sd_card.h"
// #include "INA229/INA229.h"

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

#define nb_motor 8
#define nb_12v 2
#define nb_fan 2
#define nb_cs_adress 4

typedef uint8_t flags_t;

//###################################################
//             PINOUT FONCTION DEFINITION
//###################################################

PwmOut pwm[nb_motor] = {PwmOut(PWM1), PwmOut(PWM2), PwmOut(PWM3), PwmOut(PWM4), 
    PwmOut(PWM5), PwmOut(PWM6), PwmOut(PWM7), PwmOut(PWM8)};

DigitalOut enable_motor[nb_motor] = {DigitalOut(MTR1), DigitalOut(MTR2), DigitalOut(MTR3), DigitalOut(MTR4), 
    DigitalOut(MTR5), DigitalOut(MTR6), DigitalOut(MTR7), DigitalOut(MTR8)};

DigitalOut cs_address[nb_cs_adress] = {DigitalOut(A0), DigitalOut(A1), DigitalOut(A2), DigitalOut(A3)};

DigitalOut fan[nb_fan] = {DigitalOut(FAN1), DigitalOut(FAN2)};

DigitalOut reset_led(RESET_LED);

DigitalOut kill_enable(KILL_ENABLE);

DigitalOut latch_demux(LATCH_DEMUX);
DigitalOut enable_demux(E_DEMUX);
DigitalOut sd_cs(SD_CS);

DigitalOut red_tristate(RED_TRISTATE);
DigitalOut yellow_tristate(YELLOW_TRISTATE);
DigitalOut green_tristate(GREEN_TRISTATE);

DigitalOut sd_led(SD_LED);
DigitalOut kill_led(KILL_LED);

DigitalIn status_motor[nb_motor] = {DigitalIn(STATUS1), DigitalIn(STATUS2), DigitalIn(STATUS3), DigitalIn(STATUS4),
    DigitalIn(STATUS5), DigitalIn(STATUS6), DigitalIn(STATUS7), DigitalIn(STATUS8)};

DigitalIn alert[nb_motor + nb_12v] = {DigitalIn(ALERT1), DigitalIn(ALERT2), DigitalIn(ALERT3), DigitalIn(ALERT4),
    DigitalIn(ALERT5), DigitalIn(ALERT6), DigitalIn(ALERT7), DigitalIn(ALERT8), DigitalIn(ALERT9),
    DigitalIn(ALERT10)};

DigitalIn send_sd_rs(SEND_SD_RS);
DigitalIn send_to_sd(SEND_TO_SD);
InterruptIn kill_input(KILL_3V3);
DigitalIn pwm_stop(PWM_STOP);

//###################################################
//             OBJECTS DEFINITION
//###################################################

SPI spi(MOSI, MISO, SCLK);
SPI spi_sd(MOSI_SD, MISO_SD, SCLK_SD);
RS485 rs485(SLAVE_PSU0);
I2C i2c_bus(I2C_SDA, I2C_SCL);
PCA9531 ledDriver1(&i2c_bus, LED_DRIVER1);
PCA9531 ledDriver2(&i2c_bus, LED_DRIVER2);

//###################################################
//             THREAD DEFINITION
//###################################################

Thread readSensor;
Thread activateMotor;
Thread readMotorStatus;
Thread emergencyStop;
Thread pwmController;
Thread fanController;

//###################################################
//             VARIABLES DEFINITION
//###################################################

int8_t fault_detection[nb_motor] = {0};
uint8_t status_data[nb_motor] = {0};
uint8_t enable_motor_data[nb_motor] = {0};

flags_t killswitch = 0;

//###################################################
//             FUNCTIONS DEFINITION
//###################################################

void led_feedbackFunction(double_t batt1, double_t batt2);


#endif