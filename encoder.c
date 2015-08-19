#include "encoder.h"

#include <bcm2835.h>
#include "declare.h"

int ENCODER_init()
{
    bcm2835_gpio_fsel(ENC_PIN, BCM2835_GPIO_FSEL_OUTP);

    bcm2835_gpio_fsel(RST_COUNT, BCM2835_GPIO_FSEL_OUTP); // reset count
    bcm2835_gpio_write(RST_COUNT, LOW);

    // setting modes of counter pins

    bcm2835_gpio_fsel(OE_COUNT, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_set_pud(OE_COUNT,
        BCM2835_GPIO_PUD_UP); // pull-up for eoutput enable
    bcm2835_gpio_fsel(SEL1, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(SEL2, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_fsel(D0, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(D1, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(D2, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(D3, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(D4, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(D5, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(D6, BCM2835_GPIO_FSEL_INPT);
    bcm2835_gpio_fsel(D7, BCM2835_GPIO_FSEL_INPT);

    bcm2835_gpio_set_pud(D0, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(D1, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(D2, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(D3, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(D4, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(D5, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(D6, BCM2835_GPIO_PUD_DOWN);
    bcm2835_gpio_set_pud(D7, BCM2835_GPIO_PUD_DOWN);

    return 0; // FIXME added error messages
}
