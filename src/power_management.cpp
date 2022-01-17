/**
 * @file power_management.cpp
 * @author Francis Alonzo (francisalonzo29@gmail.com)
 * @brief Power management
 * @version 0.1
 * @date 2022-01-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "power_management.h"

//Insert all function thread here
void threadFunction()
{
  //Declaration, only put one command per thread
  while(1)
  {
    //rs.read, action & rs.write
  }
}

void led_feedbackFunction(double_t batt1, double_t batt2)
{
    uint16_t stateBattery1 = 0;
    uint16_t stateBattery2 = 0;
    uint16_t stateMotor = 0;

    // Batteries
    if(batt1 > 16.4) stateBattery1 = 0b01010100;
    else if (batt1 <= 16.4 && batt1 > 15.8) stateBattery1 = 0b01010000;
    else if (batt1 <= 15.8 && batt1 > 15.4) stateBattery1 = 0b01000000;
    else if (batt1 <= 15.4 && batt1 > 14.8) stateBattery1 = 0b0000001;
    else stateBattery1 = 0b00000000;    // 14,8V is the lowest voltage we are confortable with

    if(batt2 > 16.4) stateBattery2 = 0b00010101;    // Connection inverted on the led driver for battery 2
    else if (batt2 <= 16.4 && batt2 > 15.8) stateBattery2 = 0b00000101;
    else if (batt2 <= 15.8 && batt2 > 15.4) stateBattery2 = 0b00000001;
    else if (batt2 <= 15.4 && batt2 > 14.8) stateBattery2 = 0b01000000;
    else stateBattery2 = 0b00000000;    // 14,8V is the lowest voltage we are confortable with

    // Motors
    stateMotor = 0;
    for(uint8_t i=0; i<nb_motor/2; ++i)
    {
        stateMotor += fault_detection[i];
        stateMotor = (stateMotor<<0x2);
    }

    for(uint8_t i=nb_motor-1; i>(nb_motor/2)-1; --i) // Not sure this works but the board is changing soon
    {                                                // so I won't work on it
        stateMotor += fault_detection[i];

        if(i != 4) stateMotor = (stateMotor<<0x2);
    }

    // Kill
    if(kill_input.read() == 0) kill_led = 1;
    else kill_led = 0;

    ledDriver1.setLEDs((stateBattery1 << 8) | stateBattery2);
    ledDriver2.setLEDs(stateMotor);
}

int main()
{
    reset_led = 0;
    red_tristate = 1;
    yellow_tristate = 0;
    green_tristate = 0;

    for(uint8_t i = 0; i < nb_motor; ++i)
    {
        pwm[i].period_us(2000);
        pwm[i].pulsewidth_us(1500);
        enable_motor_data[i] = 0;
    }

    for(uint8_t i = 0; i < nb_fan; ++i)
    {
        fan[i] = 0;
    }

    uint8_t i = 0;

    while(i < nb_12v + nb_motor)
    {
        sensor[i].setConfig(CONFIG_SET);
        sensor[i].setConfigADC(CONFIG_ADC_SET);
        sensor[i].setShuntCal(SHUNT_CALIBRATION);
        sensor[i].setCurrentLSB(CURRENT_LSB_CALIBRATION);
        if(sensor[i].getConfig() == CONFIG_SET || sensor[i].getConfigADC() == CONFIG_ADC_SET ||
            sensor[i].getShuntCal() == SHUNT_CALIBRATION || sensor[i].getCurrentLSB() == CURRENT_LSB_CALIBRATION) ++i;
    }

    reset_led = 1;

    ledDriver2.setPrescaler(151, 0);
    ledDriver2.setDutyCycle(64,0);

    red_tristate = 0;
    yellow_tristate = 1;
    green_tristate = 0;
}