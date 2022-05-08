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
#include "address_i2c.h"
#include "Utility/utility.h"
#include "PCA9531/PCA9531.h"
#include "RS485/RS485.h"
#include "RS485/RS485_definition.h"
#include "SD_Card/sd_card.h"
#include "INA228/INA228.h"

#define CONFIG_SET (0x01 << 4)
#define CONFIG_ADC_SET (0xFB6B)
#define SHUNT_CALIBRATION (0xBB8)
#define CURRENT_LSB_CALIBRATION (0.000029)
#define OVERVOLTAGE_BUS_THRESHOLD (0x1A40)
#define UNDERVOLTAGE_BUS_THRESHOLD (0xD80)
#define OVERTEMPERATURE_THRESHOLD (0x2800)

#define NEUTRAL_PWM (1500)
#define MAX_PWM (1900)
#define MIN_PWM (1100)

#define NB_MOTORS (8)
#define NB_12V (2)
#define NB_FAN (2)
#define NB_CS_ADRESS (4)

#if defined(USE_KILL_SIGNAL_HIGH)
    #define KILL_ACTIVATION_STATUS (1)
#elif defined(KILL_SWITCH_ACTIVE_LOW)
    #define USE_KILL_SIGNAL_LOW (0)
#else
    #error "Error: kill activation state not defined, plese define USE_KILL_SIGNAL_HIGH or USE_KILL_SIGNAL_LOW before including power_management.h"
#endif


typedef uint8_t flags_t;

//###################################################
//             PINOUT FONCTION DEFINITION
//###################################################

PwmOut pwm[NB_MOTORS] = {PwmOut(PWM1), PwmOut(PWM2), PwmOut(PWM3), PwmOut(PWM4), 
    PwmOut(PWM5), PwmOut(PWM6), PwmOut(PWM7), PwmOut(PWM8)};

DigitalOut enable_motor[NB_MOTORS] = {DigitalOut(MTR1), DigitalOut(MTR2), DigitalOut(MTR3), DigitalOut(MTR4), 
    DigitalOut(MTR5), DigitalOut(MTR6), DigitalOut(MTR7), DigitalOut(MTR8)};

DigitalOut cs_address[NB_CS_ADRESS] = {DigitalOut(A0), DigitalOut(A1), DigitalOut(A2), DigitalOut(A3)};

DigitalOut fan[NB_FAN] = {DigitalOut(FAN1), DigitalOut(FAN2)};

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

DigitalIn status_motor[NB_MOTORS] = {DigitalIn(STATUS1), DigitalIn(STATUS2), DigitalIn(STATUS3), DigitalIn(STATUS4),
    DigitalIn(STATUS5), DigitalIn(STATUS6), DigitalIn(STATUS7), DigitalIn(STATUS8)}; // 0 = fault, 1 = nominal

DigitalIn alert[NB_MOTORS + NB_12V] = {DigitalIn(ALERT1), DigitalIn(ALERT2), DigitalIn(ALERT3), DigitalIn(ALERT4),
    DigitalIn(ALERT5), DigitalIn(ALERT6), DigitalIn(ALERT7), DigitalIn(ALERT8), DigitalIn(ALERT9),
    DigitalIn(ALERT10)};// 0 = fault, 1 = nominal

DigitalIn send_sd_rs(SEND_SD_RS);
DigitalIn send_to_sd(SEND_TO_SD);
DigitalIn kill_input(KILL_3V3);
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

INA228 sensor[NB_MOTORS + NB_12V] = {INA228(&i2c_bus, I2C_M1), INA228(&i2c_bus, I2C_M2), INA228(&i2c_bus, I2C_M3), 
    INA228(&i2c_bus, I2C_M4), INA228(&i2c_bus, I2C_M5), INA228(&i2c_bus, I2C_M6), INA228(&i2c_bus, I2C_M7), 
    INA228(&i2c_bus, I2C_M8), INA228(&i2c_bus, I2C_12V1), INA228(&i2c_bus, I2C_12V2)};

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

int8_t fault_detection[NB_MOTORS] = {0};
uint8_t status_data[NB_MOTORS] = {0};

//contains the motor activation status requested by the control, the actual satus
//may be different if the kill switch is activated
uint8_t enable_motor_request[NB_MOTORS] = {0};

uint16_t data_pwm_request[NB_MOTORS] = {0};

Mutex mutexPWM;
Mutex mutexStatusMotor;

//###################################################
//             FUNCTIONS DEFINITION
//###################################################

void led_feedbackFunction(double_t batt1, double_t batt2);


#endif