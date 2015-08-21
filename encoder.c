#include "encoder.h"

#include <stdio.h>
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

//==============================================================================
float ENCODER_read()
{
    uint32_t encoder_array = 0;

    // setting OE for counter
    bcm2835_gpio_write(OE_COUNT, LOW);

    // reading MSB (24-31)

    bcm2835_gpio_write(SEL1, LOW);
    bcm2835_gpio_write(SEL2, HIGH);

    encoder_array |= bcm2835_gpio_lev(D0) << 24;
    encoder_array |= bcm2835_gpio_lev(D1) << 25;
    encoder_array |= bcm2835_gpio_lev(D2) << 26;
    encoder_array |= bcm2835_gpio_lev(D3) << 27;
    encoder_array |= bcm2835_gpio_lev(D4) << 28;
    encoder_array |= bcm2835_gpio_lev(D5) << 29;
    encoder_array |= bcm2835_gpio_lev(D6) << 30;
    encoder_array |= bcm2835_gpio_lev(D7) << 31;

    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

    // reading 3rd Byte (16-23)

    bcm2835_gpio_write(SEL1, HIGH);
    bcm2835_gpio_write(SEL2, HIGH);

    encoder_array |= bcm2835_gpio_lev(D0) << 16;
    encoder_array |= bcm2835_gpio_lev(D1) << 17;
    encoder_array |= bcm2835_gpio_lev(D2) << 18;
    encoder_array |= bcm2835_gpio_lev(D3) << 19;
    encoder_array |= bcm2835_gpio_lev(D4) << 20;
    encoder_array |= bcm2835_gpio_lev(D5) << 21;
    encoder_array |= bcm2835_gpio_lev(D6) << 22;
    encoder_array |= bcm2835_gpio_lev(D7) << 23;

    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

    // reading 2nd byte (8-15)

    bcm2835_gpio_write(SEL1, LOW);
    bcm2835_gpio_write(SEL2, LOW);

    encoder_array |= bcm2835_gpio_lev(D0) << 8;
    encoder_array |= bcm2835_gpio_lev(D1) << 9;
    encoder_array |= bcm2835_gpio_lev(D2) << 10;
    encoder_array |= bcm2835_gpio_lev(D3) << 11;
    encoder_array |= bcm2835_gpio_lev(D4) << 12;
    encoder_array |= bcm2835_gpio_lev(D5) << 13;
    encoder_array |= bcm2835_gpio_lev(D6) << 14;
    encoder_array |= bcm2835_gpio_lev(D7) << 15;

    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

    // reading LSB (0-7)

    bcm2835_gpio_write(SEL1, HIGH);
    bcm2835_gpio_write(SEL2, LOW);

    encoder_array |= bcm2835_gpio_lev(D0) << 0;
    encoder_array |= bcm2835_gpio_lev(D1) << 1;
    encoder_array |= bcm2835_gpio_lev(D2) << 2;
    encoder_array |= bcm2835_gpio_lev(D3) << 3;
    encoder_array |= bcm2835_gpio_lev(D4) << 4;
    encoder_array |= bcm2835_gpio_lev(D5) << 5;
    encoder_array |= bcm2835_gpio_lev(D6) << 6;
    encoder_array |= bcm2835_gpio_lev(D7) << 7;

    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));
    //
    // printf("\n");

    // reset OE value
    bcm2835_gpio_write(OE_COUNT, HIGH);

    // convert data to decimal

    int count = encoder_array;
    // printf("UNION: dec %d - hex 0x%x, sizeof %d\n", count, count,
    // sizeof(encoder_array));

    float rotation = (float)count / 4096.0;
    //	printf("Decimal value of x = %d \n",count);
    //	printf("Rotations = %f \n",rotation);

    return rotation;
}
