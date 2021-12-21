/***
 * 
 * Example code: This code is a simple program that turn on/off a LED with a button while another LED flash.
 * 
 ***/

#include "main.h"

//Declaration of threads
Thread thread;

//Insert all function thread here
void threadFunction()
{
  //Declaration, only put one command per thread
  while(1)
  {
    //rs.read, action & rs.write
  }
}
 
int main()
{
  INA229 device(&spi, &chip_select);
  spi.format(8, 1);
  spi.frequency(100000);

  device.setConfig(CONFIG_SET);
  if(device.getConfig() == CONFIG_SET) (led2.read()) ? led2 = 0 : led2 = 1;
  
  ThisThread::sleep_for(1000);  

  device.setConfigADC(CONFIG_ADC_SET);
  if(device.getConfigADC() == CONFIG_ADC_SET) (led2.read()) ? led2 = 0 : led2 = 1;

  ThisThread::sleep_for(1000);

  device.setShuntCal(SHUNT_CALIBRATION);
  if(device.getShuntCal() == SHUNT_CALIBRATION) (led2.read()) ? led2 = 0 : led2 = 1;

  ThisThread::sleep_for(1000);

  device.setBOVL(OVERVOLTAGE_BUS_THRESHOLD);
  if(device.getBOVL() == OVERVOLTAGE_BUS_THRESHOLD) (led2.read()) ? led2 = 0 : led2 = 1;

  ThisThread::sleep_for(1000);

  device.setBUVL(UNDERVOLTAGE_BUS_THRESHOLD);
  if(device.getBUVL() == UNDERVOLTAGE_BUS_THRESHOLD) (led2.read()) ? led2 = 0 : led2 = 1;

  ThisThread::sleep_for(1000);

  device.setOverTempLimit(OVERTEMPERATURE_THRESHOLD);
  if(device.getOverTempLimit() == OVERTEMPERATURE_THRESHOLD) (led2.read()) ? led2 = 0 : led2 = 1;

  ThisThread::sleep_for(1000);

  thread.start(threadFunction);
  thread.set_priority(osPriorityHigh);
}