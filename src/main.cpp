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

  // // Test #1 : Puissance maximale compé durant 10 mins
  // pwm.pulsewidth_us(1800);
  // for (uint8_t i = 0; i < 10; ++i)
  // {
  //   ThisThread::sleep_for(60000); // Sleep 60 secondes
  //   led3 = !(led3);
  // }

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

  // // Test #3 : Inversement de puissance sur l'alimentation
  // for (uint8_t i = 0; i < 10; ++i)
  // {
  //   pwm.pulsewidth_us(1700);
  //   ThisThread::sleep_for(5000); // Sleep 5 sec
  //   led3 = !(led3);
  //   pwm.pulsewidth_us(1300);
  //   ThisThread::sleep_for(5000); // Sleep 5 sec
  //   led3 = !(led3);
  //   pwm.pulsewidth_us(1700);
  //   ThisThread::sleep_for(5000); // Sleep 5 sec
  //   led3 = !(led3);
  //   pwm.pulsewidth_us(NEUTRAL_PWM);
  //   ThisThread::sleep_for(10000); // Sleep 10 sec
  // }

  // // Test #4 : Test de température à 70°C
  // uint32_t pwm_send = NEUTRAL_PWM + 100;

  // while(pwm_send < MAX_PWM)
  // {
  //   led3 = !(led3);
  //   pwm.pulsewidth_us(pwm_send);
  //   ThisThread::sleep_for(120000); // Sleep 120 secondes
  //   led3 = !(led3);
  //   pwm.pulsewidth_us(NEUTRAL_PWM); // Repos + Cooldown
  //   ThisThread::sleep_for(30000); // Sleep 30 sec
  //   pwm_send += 50;
  // }

  // Test #5 : Valeur température finale pour la puissance observée
  pwm.pulsewidth_us(1725);
  for (uint8_t i = 0; i < 10; ++i) // Ne devrait pas être en mesure de finir avant la fin
  {
    ThisThread::sleep_for(60000); // Sleep 1 minute  
  }

  // // Test #5 : Puissance maximale moteur - Test destructif
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