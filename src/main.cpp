/***
 * 
 * Example code: This code is a simple program that turn on/off a LED with a button while another LED flash.
 * 
 ***/

#include "main.h"

int main()
{
  // Init l'ESC
  pwm.period_us(2000);
  pwm.pulsewidth_us(NEUTRAL_PWM);
  in = 0;
  led2 = 0;
  led3 = 0;
  ThisThread::sleep_for(1000);

  // Allumage du moteur
  in = 1;
  ThisThread::sleep_for(5000);

  // Test #1 : Puissance maximale compé durant 10 mins
  pwm.pulsewidth_us(1800);
  for (uint8_t i = 0; i < 10; ++i)
  {
    ThisThread::sleep_for(60000); // Sleep 60 secondes
    led3 = !(led3);
  }

  // // Test #2 : Puissance maximale moteur 30 sec ON / 30 sec OFF
  // for (uint8_t i = 0; i < 10; ++i)
  // {
  //   pwm.pulsewidth_us(1900);
  //   ThisThread::sleep_for(30000); // Sleep 30 sec
  //   led3 = !(led3);
  //   pwm.pulsewidth_us(1500);
  //   ThisThread::sleep_for(30000); // Sleep 30 sec
  //   led3 = !(led3);
  // }

  // // Test #3 : Puissance maximale moteur - Test destructif
  // pwm.pulsewidth_us(1900);
  // for (uint8_t i = 0; i < 10; ++i) // Ne devrait pas être en mesure de finir avant la fin
  // {
  //   ThisThread::sleep_for(60000); // Sleep 1 minute  
  // }

  // Test terminé
  pwm.pulsewidth_us(NEUTRAL_PWM);
  in = 0;
  led2 = 1;
  led3 = 0;
}