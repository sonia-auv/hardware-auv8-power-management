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
#define USE_KILL_SIGNAL_HIGH
#include "power_management.h"

bool isKillSwitchActivated()
{
 return kill_input == KILL_ACTIVATION_STATUS;
}

void receiveMotorEnableRequestCallback()
{
    uint8_t cmd_array[1] = {CMD_ACT_MOTOR};
    uint8_t receive[255];
    uint8_t nb_cmd = 1;
    uint8_t size_cmd = NB_MOTORS;

    while(true)
    {
        if(rs485.read(cmd_array, nb_cmd, receive) == size_cmd)
        {
            for(uint8_t i = 0; i < NB_MOTORS; ++i)
            {
                enable_motor_request[i] = receive[i] & 0x01;
            }
        }
    }
}

void readMotorStatusCallback()
{
    uint8_t cmd_array[1] = {CMD_READ_MOTOR};
    uint8_t send[255] = {0};
    uint8_t nb_bytes = NB_MOTORS;

    while(true)
    {
        for(uint8_t i = 0; i < NB_MOTORS; ++i)
        {
            status_data[i] = ~(status_motor[i]) & 0x01;
            send[i] = (status_data[i] == 1) ? 0x02 : 0x00; //if error detected, send 0x02, 0 otherwise
        }
        rs485.write(SLAVE_PSU0, cmd_array[0], nb_bytes, send);
        ThisThread::sleep_for(1000);
    }
}

void emergencyStopCallback()
{

    uint8_t error_status_motor[NB_MOTORS]= {0};

    while(true)
    {
    
        //set pwm to neutral if kill switch activated
        if(!isKillSwitchActivated()){
            mutexPWM.lock();
            for(uint8_t i = 0; i< NB_MOTORS; i++){
                pwm[i].pulsewidth_us(NEUTRAL_PWM);
            }
            mutexPWM.unlock();
        }

        //get status of all motor
        mutexStatusMotor.lock();
        for(uint8_t i = 0; i < NB_MOTORS; i++){
            error_status_motor[i] = status_motor[i];
        }
        mutexStatusMotor.unlock();

        //set enable status depending on the state of the kill stwitch and on the motor error code
        for(uint8_t i = 0; i < NB_MOTORS; ++i)
        {
            if(isKillSwitchActivated() && error_status_motor[i]){  //set motor to requested state
                enable_motor[i] = enable_motor_request[i];
            }else{ //disable motor regardless of requested state
                enable_motor[i] = 0x00;
            }
        }



        ThisThread::sleep_for(200);
    }
}

void pwmControllerCallback()
{
    uint8_t cmd_array[1] = {CMD_PWM};
    uint8_t pwm_received[255];
    uint8_t nb_command = 1;
    uint8_t size_command = 16;
    uint16_t data_pwm = 0;

    while(true)
    {
        if(rs485.read(cmd_array, nb_command, pwm_received) == size_command)
        {
            mutexPWM.lock();
            for(uint8_t i=0; i<NB_MOTORS; ++i)
            {
                if(kill_input == 0)
                {
                    data_pwm = pwm_received[(2*i)+1]+pwm_received[2*i]*256;
                    if(data_pwm >= MIN_PWM && data_pwm <= MAX_PWM)
                    {
                        pwm[i].pulsewidth_us(data_pwm);
                    }
                }
                else
                {
                    pwm[i].pulsewidth_us(NEUTRAL_PWM);
                }
            }
            mutexPWM.unlock();
        }
    }
}

void check_mask(INA228 sensor)
{
  uint8_t data_ready = 0;

  while(data_ready == 0)
  {
    data_ready = ((sensor.getAlertFlags()>>1) & 0x01);
    ThisThread::sleep_for(1);
  }
}

void readSensorCallback()
{
  uint8_t cmd_array[2]={CMD_VOLTAGE, CMD_CURRENT};
  uint8_t voltage_send[255]={0};
  uint8_t current_send[255]={0};
  uint8_t nb_sensor = NB_MOTORS+NB_12V;
  uint8_t nb_byte_send = 4*(nb_sensor);
  double_t voltage, current, batt1 = 0, batt2 = 0;

  while(true)
  {
    for(uint8_t i=0; i<nb_sensor; ++i)
    {
      check_mask(sensor[i]);

      voltage = sensor[i].getBusVolt();
      current = sensor[i].getCurrent();

      putFloatInArray(voltage_send, voltage, i*4);
      putFloatInArray(current_send, current, i*4);

      if(i == 8) batt1 = voltage;
      if(i == 9) batt2 = voltage;
    }
    led_feedbackFunction(batt1, batt2);
    rs485.write(SLAVE_PSU0, cmd_array[0], nb_byte_send, voltage_send);
    rs485.write(SLAVE_PSU0, cmd_array[1], nb_byte_send, current_send);
    ThisThread::sleep_for(500);
  }
}

uint16_t ledStatus(uint8_t led_nb, bool led_on)
{
    uint8_t led_state = (led_on) ? 0b01 : 0b00;
    return led_state << (2*led_nb);
}

uint16_t led_getStateBattery(double_t batt, uint8_t battNumber)
{
    uint16_t stateBattery;

    //bat number should be 0 or 1 ONLY (not 1 or 2)
    if(battNumber > 1){
        return 0xFFFF;
    }

    //This array gives the position of 4 led for the battries indicator for both batteries
    //For example when battery 1 is at 1/4, the led 7 is on,
    // when battery 0 is at 3/4, the led 1 and 2 are on
    //note: the 1/4 led is only on when the battery is at 1/4. the other led are on as long as the threshold is not reached   
    uint8_t LED_POSITION[2][4] ={
    /*  1/4  2/4  3/4 Full*/
         {0,   1,   2,   3},
         {7,   6,   5,   4}
    };

    uint8_t* pos = LED_POSITION[battNumber];

    // Batteries
    if(batt > 16.4)                       {stateBattery = ledStatus(pos[0],false) | ledStatus(pos[1],true)  | ledStatus(pos[2],true)  | ledStatus(pos[3],true);} // | XXX:
    else if (batt <= 16.4 && batt > 15.8) {stateBattery = ledStatus(pos[0],false) | ledStatus(pos[1],true)  | ledStatus(pos[2],true)  | ledStatus(pos[3],false);}// | XX :
    else if (batt <= 15.8 && batt > 15.4) {stateBattery = ledStatus(pos[0],false) | ledStatus(pos[1],true)  | ledStatus(pos[2],false) | ledStatus(pos[3],false);}// | X  :
    else if (batt <= 15.4 && batt > 14.8) {stateBattery = ledStatus(pos[0],true)  | ledStatus(pos[1],false) | ledStatus(pos[2],false) | ledStatus(pos[3],false);}// |X   :
    else                                  {stateBattery = ledStatus(pos[0],false) | ledStatus(pos[1],false) | ledStatus(pos[2],false) | ledStatus(pos[3],false);}// |    :  
    // 14,8V is the lowest voltage we are confortable with

    return stateBattery;
}

void led_feedbackFunction(double_t batt1, double_t batt2)
{
    uint16_t stateBattery = 0;
    uint16_t stateMotor = 0;

    stateBattery = led_getStateBattery(batt1, 0) | led_getStateBattery(batt2, 2);
    // Batteries
    // if(batt1 > 16.4) stateBattery1 = 0b01010100;
    // else if (batt1 <= 16.4 && batt1 > 15.8) stateBattery1 = 0b01010000;
    // else if (batt1 <= 15.8 && batt1 > 15.4) stateBattery1 = 0b01000000;
    // else if (batt1 <= 15.4 && batt1 > 14.8) stateBattery1 = 0b0000001;
    // else stateBattery1 = 0b00000000;    // 14,8V is the lowest voltage we are confortable with

    // if(batt2 > 16.4) stateBattery2 = 0b00010101;    // Connection inverted on the led driver for battery 2
    // else if (batt2 <= 16.4 && batt2 > 15.8) stateBattery2 = 0b00000101;
    // else if (batt2 <= 15.8 && batt2 > 15.4) stateBattery2 = 0b00000001;
    // else if (batt2 <= 15.4 && batt2 > 14.8) stateBattery2 = 0b01000000;
    // else stateBattery2 = 0b00000000;    // 14,8V is the lowest voltage we are confortable with

    //ojt: TODO: check 
    // Motors
    stateMotor = 0;
    for(uint8_t i=0; i<NB_MOTORS/2; ++i)
    {
        stateMotor += fault_detection[i];
        stateMotor = (stateMotor<<0x2);
    }

    for(uint8_t i=NB_MOTORS-1; i>(NB_MOTORS/2)-1; --i) // Not sure this works but the board is changing soon
    {                                                // so I won't work on it
        stateMotor += fault_detection[i];

        if(i != 4) stateMotor = (stateMotor<<0x2);
    }

    // Kill
    if(isKillSwitchActivated == 0) kill_led = 1;
    else kill_led = 0;

    ledDriver1.setLEDs(stateBattery);
    ledDriver2.setLEDs(stateMotor);
}

int main()
{
    reset_led = 0;
    red_tristate = 1;
    yellow_tristate = 0;
    green_tristate = 0;

    for(uint8_t i = 0; i < NB_MOTORS; ++i)
    {
        pwm[i].period_us(2000);
        pwm[i].pulsewidth_us(1500);
        enable_motor_request[i] = 0;
    }

    for(uint8_t i = 0; i < NB_FAN; ++i)
    {
        fan[i] = 0;
    }

    spi.format(8, 1);
    spi.frequency(1000000);
    spi_sd.format(8, 1);
    spi_sd.frequency(1000000);

    uint8_t i = 0;
    while(i < NB_12V + NB_MOTORS)
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

    readMotorStatus.start(readMotorStatusCallback);
    readMotorStatus.set_priority(osPriorityHigh);

    emergencyStop.start(emergencyStopCallback);
    emergencyStop.set_priority(osPriorityHigh1);

    pwmController.start(pwmControllerCallback);
    pwmController.set_priority(osPriorityHigh);

    red_tristate = 0;
    yellow_tristate = 0;
    green_tristate = 1;
}