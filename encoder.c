#include "encoder.h"

#include <stdio.h>
#include <bcm2835.h>
#include <math.h>

#include "declare.h"

extern pthread_mutex_t mtx_read;

int ENCODER_init()
{

    pthread_mutex_lock(&mtx_read);
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
    pthread_mutex_unlock(&mtx_read);

    return 0; // FIXME added error messages
}

//==============================================================================
double ENCODER_read()
{
    static int run = 0;
    static uint32_t zero_shift = 0;
    // mutex lock
    pthread_mutex_lock(&mtx_read);
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

    volatile uint32_t encoder_array = 0;
    uint64_t readbit = 0;

    // setting OE for counter
    bcm2835_gpio_write(OE_COUNT, LOW);

    // reading MSB (24-31)

    bcm2835_gpio_write(SEL1, LOW);
    bcm2835_gpio_write(SEL2, HIGH);
    int pins[8] = { D0, D1, D2, D3, D4, D5, D6, D7 };
    for (int i = 24; i < 32; i++) {
        readbit = bcm2835_gpio_lev(pins[i - 24]);
        //readbit = readbit << i;
        encoder_array += pow(2.0, i) * readbit;
    }
    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

    // reading 3rd Byte (16-23)

    bcm2835_gpio_write(SEL1, HIGH);
    bcm2835_gpio_write(SEL2, HIGH);

    for (int i = 16; i < 24; i++) {
        readbit = bcm2835_gpio_lev(pins[i - 16]);
        encoder_array += pow(2.0, i) * readbit;
    }

    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

    // reading 2nd byte (8-15)

    bcm2835_gpio_write(SEL1, LOW);
    bcm2835_gpio_write(SEL2, LOW);

    for (int i = 8; i < 16; i++) {
        readbit = bcm2835_gpio_lev(pins[i - 8]);
        encoder_array += pow(2.0, i) * readbit;
    }

    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

    // reading LSB (0-7)

    bcm2835_gpio_write(SEL1, HIGH);
    bcm2835_gpio_write(SEL2, LOW);

    for (int i = 0; i < 8; i++) {
        readbit = bcm2835_gpio_lev(pins[i]);
        encoder_array += pow(2.0, i) * readbit;
    }

    // printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
    //     bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
    //     bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));
    //
    // printf("\n");

    // reset OE value
    bcm2835_gpio_write(OE_COUNT, HIGH);

    // mutex lock
    pthread_mutex_unlock(&mtx_read);

    // convert data to decimal
    //    if(encoder_array - )
    //encoder_array = encoder_array & 0x00FFFFFF;
    if (run == 0 && encoder_array != 0) {

        bcm2835_gpio_fsel(RST_COUNT, BCM2835_GPIO_FSEL_OUTP); // reset count
        bcm2835_gpio_write(RST_COUNT, HIGH); // now start counting
        bcm2835_delayMicroseconds(1);
        bcm2835_gpio_write(RST_COUNT, LOW); // now start counting
        bcm2835_delayMicroseconds(1);
        bcm2835_gpio_write(RST_COUNT, HIGH); // now start counting
        run = 1;

        return ENCODER_read();
    }

    //printf("%x\n", encoder_array);

    //encoder_array = encoder_array - zero_shift;
    double count = encoder_array;

    double length = (double)count * 2.0 / (4096.0 * 4.8);
    // NOTE the shaft encoder is probably not 225780, 465802

    // printf("UNION: dec %ld - hex 0x%x, sizeof %d\n", encoder_array, encoder_array,
    //     sizeof(encoder_array));
    // printf("Decimal value of shift = %ld \n", zero_shift);
    // printf("length (mm) = %lf \n", length);

    return length;
}
