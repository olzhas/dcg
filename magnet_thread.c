#include "declare.h"
#include "magnet_thread.h"
#include "utilities.h"
#include <time.h>

#define SPI_DELAY 1000  // in nanoseconds
#define NSEC_DELAY(DURATION) \
  nanosleep((struct timespec[]){{.tv_sec = 0, .tv_nsec = DURATION}}, NULL);

void magnet_thread() {
  // init timer structure
  // TODO init timer function ??
  const struct timespec timer = {.tv_sec = (int)dT_XD,
                                 .tv_nsec = 1.0E9 * (dT_XD - (int)dT_XD)};

  printf("Magnet thread started.\n");

  bcm2835_spi_begin();
  bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);     // The default
  bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);                  // The default
  bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128);  // The default
  bcm2835_spi_chipSelect(BCM2835_SPI_CS0);                     // The default
  bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);     // the default

  while (!start) {
  }

  while (1) {
    // code for obtaining value from iC-MU
    char mag_buf[] = {0xF5};  //  Data to send: first byte is op code,
    //  rest depends on the opcode
    char mag_in[2] = {0};

    bcm2835_gpio_write(PA0, LOW);
    bcm2835_spi_transfernb(mag_buf, mag_in, sizeof(mag_in));
    NSEC_DELAY(SPI_DELAY);
    bcm2835_gpio_write(PA0, HIGH);

    if (mag_in[1] == 0x80) {
      char status[] = {0xA6};
      char status_in[3] = {0};

      bcm2835_gpio_write(PA0, LOW);
      bcm2835_spi_transfernb(status, status_in, sizeof(status_in));
      NSEC_DELAY(SPI_DELAY);
      bcm2835_gpio_write(PA0, HIGH);

      float mag_reading = (256.0 * status_in[1]) + status_in[2];

      // code for calculating xd

      float mag_position = (360.0 * mag_reading) / 65536.0;
      //#ifdef DEBUG
      printf("mag: Reading: %f,	angle: %f,	read: %X, %X\n", mag_reading,
             mag_position, status_in[1], status_in[2]);
      fflush(stdout);

      if ((status_in[1] == status_in[2]) && status_in[1] == 0) {
        printf("%x %x %x\n", status_in[0], status_in[1], status_in[2]);
        printf(
            "##############################################################\n"
            "##############################################################\n"
            "##############################################################\n");
        return;
      }
      //#endif
      //			xd = pow(sin(mag_reading),-0.5);

      xd = mag_position / 2.0;
    } else {
      printf("DATA NOT VALID or too early\n");
      printf("%x %x\n\n", mag_in[0], mag_in[1]);
    }
    //			xd = 100.0;
    // bcm2835_gpio_write(MAG_PIN, LOW);

    // reset sampling flag
    // sample_magnet = 0;

    // set magnet flag high

    nanosleep(&timer, NULL);
  }
}
