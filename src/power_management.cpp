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

uint16_t applyCalibration(uint16_t pwm)
{
    return pwm + CALIB_VAL_PWM;
}

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
            enable_motor_request.mutex.lock();
            for(uint8_t i = 0; i < NB_MOTORS; ++i)
            {
                enable_motor_request.request[i] = receive[i] & 0x01;
            }
            enable_motor_request.mutex.unlock();

        }
    }
}

void readMotorStatusCallback()
{
    uint8_t cmd_array[1] = {CMD_READ_MOTOR};
    uint8_t send[255] = {0};
    uint8_t nb_bytes = NB_MOTORS;
    uint8_t motor_failure_state_cpy[NB_MOTORS];
    uint8_t enable_motor_resquest_cpy[NB_MOTORS];

    while(true)
    {
        enable_motor_request.mutex.lock();
        for(uint8_t i = 0; i < NB_MOTORS; i++)
        {
           enable_motor_resquest_cpy[i] = enable_motor_request.request[i];
        }
        enable_motor_request.mutex.unlock();


        //get motor faillure state and set message
        motor_failure_state.mutex.lock();
        for(uint8_t i = 0; i < NB_MOTORS; ++i)
        {
            motor_failure_state.state[i] = ~(status_motor[i]) & 0x01;
            motor_failure_state_cpy[i] = motor_failure_state.state[i]; 
            //if error detected, send 0x02, otherwise we send to enable request
            send[i] = (motor_failure_state.state[i] == 1) ? 0x02 : enable_motor_resquest_cpy[i]; 
        }
        motor_failure_state.mutex.unlock();


        //if a motor is in a failure state, we deactivate it. The user will
        //have to renable it explicitly 
        enable_motor_request.mutex.lock();
        for(uint8_t i = 0; i < NB_MOTORS; i++)
        {
           if(motor_failure_state_cpy[i] == 1){
               enable_motor_request.request[i] = 0;
           }
        }
        enable_motor_request.mutex.unlock();


        rs485.write(SLAVE_PWR_MANAGEMENT, cmd_array[0], nb_bytes, send);
        ThisThread::sleep_for(1000);
    }
}


void motorControllerCallback()
{

    uint8_t error_status_motor[NB_MOTORS]= {0};
    uint8_t enable_motor_request_copy[NB_MOTORS]= {0};
    motor_state_t motor_state_copy[NB_MOTORS];

    while(true)
    {
        //set pwm to neutral if kill switch activated
        if(isKillSwitchActivated()){
            //note pwmControllerCallback all ready check if the kill is
            //activated, but it waits for a new message to arrive
            mutexPWM.lock();
            for(uint8_t i = 0; i< NB_MOTORS; i++){
                pwm[i].pulsewidth_us(applyCalibration(NEUTRAL_PWM) );
            }
            mutexPWM.unlock();
        }

        //get status of all motor
        motor_failure_state.mutex.lock();
        for(uint8_t i = 0; i < NB_MOTORS; i++){ error_status_motor[i] = motor_failure_state.state[i];}
        motor_failure_state.mutex.unlock();

        enable_motor_request.mutex.lock();
        for(uint8_t i = 0; i < NB_MOTORS; i++){ enable_motor_request_copy[i] = enable_motor_request.request[i];}
        enable_motor_request.mutex.unlock();
        


        //set enable status depending on the state of the kill stwitch and on the motor error code
        for(uint8_t i = 0; i < NB_MOTORS; ++i)
        {
            if(isKillSwitchActivated()){
                enable_motor[i] = 0;
                motor_state_copy[i] = (enable_motor_request_copy[i]) ? MOTOR_ON : MOTOR_OFF;
            }else if(error_status_motor[i]){
                motor_state_copy[i] = MOTOR_FAILURE;
                //enable_motor[i] = 0;
                enable_motor[i] = (enable_motor_request_copy[i]) ? MOTOR_ON : MOTOR_OFF;

            }else{
                enable_motor[i] = enable_motor_request_copy[i];
                motor_state_copy[i] = (enable_motor_request_copy[i]) ? MOTOR_ON : MOTOR_OFF;
            }
        }

        motor_state.mutex.lock();
        //for(int i=0; i<NB_MOTORS; i++)  {motor_state.state[i] = motor_state_copy[i];}
         //motor_state_t motor_state_tmp[NB_MOTORS]={MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_OFF, MOTOR_ON, MOTOR_FAILURE, MOTOR_OFF};
        for(int i=0; i<NB_MOTORS; i++)  {motor_state.state[i] = motor_state_copy[i];}
        motor_state.mutex.unlock();

        ThisThread::sleep_for(500);
    }
}

void pwmControllerCallback()
{
    uint8_t cmd_array[1] = {CMD_PWM};
    uint8_t pwm_received[255];
    uint8_t nb_command = 1;
    uint8_t size_command = 16;
    uint16_t data_pwm = 0;
    uint8_t enable_motor_request_copy[NB_MOTORS]= {0};

    while(true)
    {
        if(rs485.read(cmd_array, nb_command, pwm_received) == size_command)
        {
            enable_motor_request.mutex.lock();
            for(uint8_t i = 0; i < NB_MOTORS; i++){ enable_motor_request_copy[i] = enable_motor_request.request[i];}
            enable_motor_request.mutex.unlock();

            mutexPWM.lock();
            for(uint8_t i=0; i<NB_MOTORS; ++i)
            {
                if( ( !isKillSwitchActivated() ) && (enable_motor_request_copy[i] == 1) )
                {
                    data_pwm = pwm_received[(2*i)+1]+pwm_received[2*i]*256;
                    if(data_pwm >= MIN_PWM && data_pwm <= MAX_PWM)
                    {
                        pwm[i].pulsewidth_us(applyCalibration(data_pwm));
                    }
                }
                else
                {
                    pwm[i].pulsewidth_us(applyCalibration(NEUTRAL_PWM));
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
  uint8_t cmd_array[3]={CMD_VOLTAGE, CMD_CURRENT, CMD_TEMPERATURE};
  uint8_t voltage_send[255]={0};
  uint8_t current_send[255]={0};
  uint8_t temperature_send[255]={0};
  uint8_t nb_sensor = NB_MOTORS+NB_12V;
  uint8_t nb_byte_send = 4*(nb_sensor);
  double_t voltage, current, temperature, batt0 = 0, batt1 = 0;

  while(true)
  {
    for(uint8_t i=0; i<nb_sensor; ++i)
    {
      check_mask(sensor[i]);
      voltage = sensor[i].getBusVolt();
      current = sensor[i].getCurrent();
      temperature = sensor[i].getDieTemp();
      putFloatInArray(voltage_send, voltage, i*4);
      putFloatInArray(current_send, current, i*4);
      putFloatInArray(temperature_send, temperature, i*4);
      if(i == 8)    {batt0 = voltage;} 
      if(i == 9)    {batt1 = voltage;}
    }

    batt_state.mutex.lock();
    batt_state.batt[0]=batt0;
    batt_state.batt[1]=batt1;
    batt_state.mutex.unlock();

    rs485.write(SLAVE_PWR_MANAGEMENT, cmd_array[0], nb_byte_send, voltage_send);
    rs485.write(SLAVE_PWR_MANAGEMENT, cmd_array[1], nb_byte_send, current_send);
    rs485.write(SLAVE_PWR_MANAGEMENT, cmd_array[2], nb_byte_send, temperature_send);
    ThisThread::sleep_for(500);
  }
}

uint16_t ledStatus(uint8_t led_nb, led_state_t led_ctrl)
{
    uint8_t led_state;
    switch (led_ctrl)
    {
    case LED_ON:
        led_state = 0b01;
        break;
    case LED_PWM1:
        led_state = 0b10;
        break;
    case LED_PWM2:
        led_state = 0b11;
        break;
    default: //LED_OFF
        led_state = 0b00;
    }

    return led_state << (2*led_nb);
}

uint16_t led_getStateBattery(double_t batt, uint8_t battNumber)
{
    uint16_t ledCmd_batteries;

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
    if(batt > 16.4)                       {ledCmd_batteries = ledStatus(pos[0],LED_OFF) | ledStatus(pos[1],LED_ON)  | ledStatus(pos[2],LED_ON)  | ledStatus(pos[3],LED_ON);} // | XXX:
    else if (batt <= 16.4 && batt > 15.8) {ledCmd_batteries = ledStatus(pos[0],LED_OFF) | ledStatus(pos[1],LED_ON)  | ledStatus(pos[2],LED_ON)  | ledStatus(pos[3],LED_OFF);}// | XX :
    else if (batt <= 15.8 && batt > 15.4) {ledCmd_batteries = ledStatus(pos[0],LED_OFF) | ledStatus(pos[1],LED_ON)  | ledStatus(pos[2],LED_OFF) | ledStatus(pos[3],LED_OFF);}// | X  :
    else if (batt <= 15.4 && batt > 14.8) {ledCmd_batteries = ledStatus(pos[0],LED_ON)  | ledStatus(pos[1],LED_OFF) | ledStatus(pos[2],LED_OFF) | ledStatus(pos[3],LED_OFF);}// |X   :
    else                                  {ledCmd_batteries = ledStatus(pos[0],LED_OFF) | ledStatus(pos[1],LED_OFF) | ledStatus(pos[2],LED_OFF) | ledStatus(pos[3],LED_OFF);}// |    :  
    // 14,8V is the lowest voltage we are confortable with

    return ledCmd_batteries;
}

uint16_t led_getStateMotor(motor_state_t* motorState, uint8_t nbMotors)
{
    uint16_t ledCmd_motor=0;
    led_state_t currentLEDstate;

    if(nbMotors != NB_MOTORS){
        return 0xFFFF; //something went very bad...
    }

    //This array gives the position of led for the corresponding motro
    //(index 0 = M1, index 7 =M8) 
    uint8_t LED_POSITION[NB_MOTORS] ={4,5,6,7,3,2,1,0};

    for(int i=0; i<nbMotors; i++)
    {
        if(motorState[i] == MOTOR_ON){
            currentLEDstate = LED_ON;
        }else if (motorState[i] == MOTOR_FAILURE)
        {
            currentLEDstate = LED_PWM1;
        }else{
            currentLEDstate = LED_OFF;
        }
        
        ledCmd_motor |=ledStatus(LED_POSITION[i],currentLEDstate);
    }

    return ledCmd_motor;
}

void led_feedbackCallback(void)
{
    uint16_t ledCmd_batteries = 0;
    uint16_t ledCmd_motor = 0;
    double_t batt0, batt1;
    motor_state_t motor_state_cpy[NB_MOTORS];

    while (1)
    {
        //copy localy the value of the batteries and the state of the
        //motors so we dont have to worry about the mutx afterward
        batt_state.mutex.lock();
        batt0 = batt_state.batt[0];
        batt1 = batt_state.batt[1];
        batt_state.mutex.unlock();

        motor_state.mutex.lock();
        for(int i=0; i<NB_MOTORS; i++) {motor_state_cpy[i] = motor_state.state[i];}
        motor_state.mutex.unlock();



        ledCmd_batteries = led_getStateBattery(batt0, 0) | led_getStateBattery(batt1, 1);
        ledCmd_motor = led_getStateMotor(motor_state_cpy, NB_MOTORS);

        kill_led = (isKillSwitchActivated())? 0: 1;

        ledDriver1.setLEDs(ledCmd_batteries);
        //ledDriver2.setLEDs(0b1010101010101010);
        
        ledDriver2.setLEDs(ledCmd_motor);
        ThisThread::sleep_for(1000);
    }
    

}

int main()
{
    reset_led = 0;
    sd_led = 1;
    red_tristate = 1;
    yellow_tristate = 0;
    green_tristate = 0;

    for(uint8_t i = 0; i < NB_MOTORS; ++i)
    {
        pwm[i].period_us(2000);
        pwm[i].pulsewidth_us(applyCalibration(NEUTRAL_PWM));
        enable_motor_request.request[i] = 0;
        motor_failure_state.state[i] = 0;
    }

    for(uint8_t i = 0; i < NB_FAN; ++i)
    {
        fan[i] = 1;
    }

    uint8_t i = 0;
    while(i < (NB_12V + NB_MOTORS))
    {
        uint16_t man = sensor[i].getManufacturer();

        //if(man !=0){
        //    i++;
        //}
        sensor[i].setConfig(CONFIG_SET);
        sensor[i].setConfigADC(CONFIG_ADC_SET);
        sensor[i].setShuntCal(SHUNT_CALIBRATION);
        sensor[i].setCurrentLSB(CURRENT_LSB_CALIBRATION);
        float currentLSB = sensor[i].getCurrentLSB();
        printf("Current LSB config: %0.3f\n", currentLSB);

        if(sensor[i].getConfig() == CONFIG_SET && sensor[i].getConfigADC() == CONFIG_ADC_SET && sensor[i].getShuntCal() == SHUNT_CALIBRATION){
            i++;
        }

        //if(sensor[i].getConfig() == CONFIG_SET && sensor[i].getConfigADC() == CONFIG_ADC_SET &&
        //    sensor[i].getShuntCal() == SHUNT_CALIBRATION && sensor[i].getCurrentLSB() == CURRENT_LSB_CALIBRATION) ++i;
    }

    reset_led = 1;

    ledDriver2.setPrescaler(151, 0);
    ledDriver2.setDutyCycle(64,0);

    red_tristate = 0;
    yellow_tristate = 1;
    green_tristate = 0;

    readSensor.start(readSensorCallback);
    readSensor.set_priority(osPriorityHigh2);

    readMotorStatus.start(readMotorStatusCallback);
    readMotorStatus.set_priority(osPriorityHigh);

    motorController.start(motorControllerCallback);
    motorController.set_priority(osPriorityHigh1);

    ledController.start(led_feedbackCallback);
    ledController.set_priority(osPriorityHigh1);

    motorEnableRqst.start(receiveMotorEnableRequestCallback);
    motorEnableRqst.set_priority(osPriorityHigh);

    pwmController.start(pwmControllerCallback);
    pwmController.set_priority(osPriorityHigh3);

    threadIsAlive.start(callback(isAliveThread, &rs485));
    threadIsAlive.set_priority(osPriorityHigh3);

    red_tristate = 0;
    yellow_tristate = 0;
    green_tristate = 1;
}
