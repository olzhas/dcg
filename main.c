#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <bcm2835.h>

#include "declare.h"
#include "encoder_thread.h"
#include "magnet_thread.h"
#include "energy_thread.h"
#include "utilities.h"

bool comparse(int argc, char* argv[]) {
  if (argc < 3) {  // must have two arguments: kp and kd
    fprintf(stderr, "\nInsufficient command line arguments, try again\n");
    fprintf(stderr, "\n%s kp kd\n\n", argv[0]);
    return false;
  }

  kp = atof(argv[1]);
  kd = atof(argv[2]);

  return true;
}

int main(int argc, char* argv[]) {
  if (comparse(argc, argv) == false) {
    return EXIT_FAILURE;
  }

  if (bcm2835_init() == false) {
    return EXIT_FAILURE;
  }

  // setting PWM_PIN as pwm from channel 0 in markspace mode with range = RANGE
  bcm2835_gpio_fsel(PWM_PIN, BCM2835_GPIO_FSEL_ALT5);  // ALT5 is pwm mode
  bcm2835_pwm_set_clock(
      BCM2835_PWM_CLOCK_DIVIDER_16);        // pwm freq = 19.2 / 16 MHz
  bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1);  // markspace mode
  bcm2835_pwm_set_range(PWM_CHANNEL, RANGE);

  bcm2835_gpio_fsel(OE_SHIFTER, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_set_pud(
      OE_SHIFTER,
      BCM2835_GPIO_PUD_DOWN);  // pull-down for output enable of logic shifters

  bcm2835_gpio_fsel(MOTOR_D3, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_set_pud(MOTOR_D3,
                       BCM2835_GPIO_PUD_DOWN);  // pull-down for motor enable

  bcm2835_gpio_fsel(PA0, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_set_pud(PA0, BCM2835_GPIO_PUD_UP);
  bcm2835_gpio_write(PA0, HIGH);

  bcm2835_gpio_write(OE_SHIFTER, HIGH);
  bcm2835_gpio_write(MOTOR_D3, LOW);

  // creating and running threads
  pthread_t th1, th2, th3, th4;

  // FIXME: test the function (SPI work well after a while)
  pthread_create(&th1, NULL, (void*)magnet_thread, NULL);
  // FIXME: test the function
  pthread_create(&th2, NULL, (void*)encoder_thread, NULL);
  // FIXME: test the function
  pthread_create(&th3, NULL, (void*)calculate_energy, NULL);
  // FIXME: test the function
  pthread_create(&th4, NULL, (void*)calculate_I_ref, NULL);
  //*/

  bcm2835_delay(300);
  // delay to make sure that all threads are initialised
  // and iC-MU is conofigured
  printf("\nPress enter to start the motor.");
  getchar();
  bcm2835_gpio_write(MOTOR_D3, HIGH);

  start = 1;
  printf("Started.\n");

  printf("\nPress enter to stop the motor.\n");
  getchar();
  bcm2835_gpio_write(MOTOR_D3, LOW);

  bcm2835_spi_end();
  bcm2835_close();

  return EXIT_SUCCESS;
}
