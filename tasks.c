#include "tasks.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <bcm2835.h>

#include "declare.h"
#include "utils.h"

//==============================================================================
void encoder_thread()
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

    printf("Encoder Thread started.\n");

    bcm2835_gpio_write(RST_COUNT, HIGH); // now start counting

    for (;;) {

        // code for obtaining value from encoder IC
        x = calculate_encoder(); // no. of rotations

        // x = (x / 4.81) * 0.002;	// distance
        // traveled by slider in metres

        // code for calculating xf and dx

        //			printf("enc_th: freq are : %f,  %f\n",
        // freq_diff, freq_filt);
        discrete_diff(); // updating value of dx by calling practical
        // differentiator
        low_pass_filter(); // update xf, the filtered value of x

        // calculate_I_ref();
    }
}

//==============================================================================
float calculate_encoder()
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
    printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
        bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
        bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

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

    printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
        bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
        bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

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

    printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
        bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
        bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

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

    printf("%d%d%d%d%d%d%d%d ", bcm2835_gpio_lev(D7), bcm2835_gpio_lev(D6),
        bcm2835_gpio_lev(D5), bcm2835_gpio_lev(D4), bcm2835_gpio_lev(D3),
        bcm2835_gpio_lev(D2), bcm2835_gpio_lev(D1), bcm2835_gpio_lev(D0));

    printf("\n");
    // reset OE value
    bcm2835_gpio_write(OE_COUNT, HIGH);

    // convert data to decimal

    int count = encoder_array;
    printf("UNION: dec %d - hex 0x%x, sizeof %d\n", count, count,
        sizeof(encoder_array));

    float rotation = (float)count / 4096.0;
    //	printf("Decimal value of x = %d \n",count);
    //	printf("Rotations = %f \n",rotation);

    return rotation;
}

//==============================================================================
void calculate_energy()
{
    // init timer structure
    // TODO init timer function ??
    const struct timespec timer = {.tv_sec = (int)dT_PO,
        .tv_nsec = 1.0E9 * (dT_PO - (int)dT_PO) };

    uint8_t send;
    uint8_t data;

    char volt[2] = { 0x00, 0x00 };
    char curr[2] = { 0x00, 0x01 };
    char pwr[2] = { 0x00, 0x00 };
    char bus_volt[2] = { 0x00, 0x00 };

    uint8_t write_address = 0x40; // find using i2cdetect -y 1

    char config_write[3] = { 0x00, 0x39,
        0x9F }; // first byte is address, next two are data
    char calib_write[3] = { 0x05, 0x10, 0x00 }; // for value 0x1000 the current LSB
    // = 1 e-3 A. (see datasheet on how
    // to determine calibration
    // register value

    char voltage_addr[1] = { 0x01 }; // voltage register address
    char current_addr[1] = { 0x04 };
    char power_addr[1] = { 0x03 };
    char bus_addr[1] = { 0x02 };

    int volt_read = 0;
    float voltage = 0;
    float current = 1.0;
    float power_read = 0;
    float bus_voltage = 0;

    printf("Energy thread started.\n");

    FILE* fp = fopen("current_file.txt", "w"); // Open file for writing
    if (fp == NULL) {
        fprintf(stderr, "Cannot open current_file.txt for writing\n");
        exit(EXIT_FAILURE);
    }

    while (start == 0) { // TODO introduce mutexes
    }

    float su2 = 0.0;

    send = bcm2835_i2c_write(config_write, 3);
    send = bcm2835_i2c_write(calib_write, 3);

    for (;;) {
        float t = 0.0;
        bcm2835_i2c_begin(); // I2C begin
        bcm2835_i2c_set_baudrate(100000);

        bcm2835_i2c_setSlaveAddress(write_address); // write
        bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);

        //			send =
        // bcm2835_i2c_read_register_rs(voltage_addr, volt, 2);
        send = bcm2835_i2c_read_register_rs(current_addr, curr, 2);
        send = bcm2835_i2c_read_register_rs(bus_addr, bus_volt, 2);
        send = bcm2835_i2c_read_register_rs(power_addr, pwr, 2);

        // bus voltage, resolution is always 4 mV
        int bus_16 = (bus_volt[0] << 8) | (bus_volt[1]);
        bus_16 = bus_16 >> 3;
        bus_voltage = (float)bus_16 * 0.004; // in Volts

        int curr_16 = (curr[0] << 8) | (curr[1]);

        if (curr[0] > 127) // if sign bit is 1
            current = (float)(curr_16 - 0x10000) / 1000.0; // in A (because LSB is 0.5 mA)
        else
            current = (float)curr_16 / 1000.0; // in A

        // power LSB = 20 * current LSB = 20 mW. Therefore, power = power read x
        // 20 (mW)
        int pow_16 = (pwr[0] << 8) | (pwr[1]);
        power = ((float)pow_16 * 20.0) / 1000.0; // (in W)
        if (current < 0)
            power = power * -1.0;

        //			discrete_intg();  // update value of energy

        // calculate power
        float power_cal = bus_voltage * current;

        //			printf("pwr: V, I, P_cal, P_meas = %f, %f, %f,
        //%f \n", bus_voltage, current, power_cal, power);

        fprintf(fp, "%f\t%f\n", current, t);
        su2++;

        bcm2835_i2c_end(); // I2C end
    }
    fclose(fp);
}
#define SPI_DELAY 1000 // in nanoseconds

#define NSEC_DELAY(DURATION)                                        \
    clock_nanosleep(CLOCK_MONOTONIC, 0,                             \
        (struct timespec[]){ {.tv_sec = 0, .tv_nsec = DURATION } }, \
        NULL);

//==============================================================================
void magnet_thread()
{
    printf("Magnet thread started.\n");

    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST); // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0); // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_128); // The default
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0); // The default
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); // the default

    for (;;) {
        // code for obtaining value from iC-MU
        char mag_buf[] = { 0xF5 }; //  Data to send: first byte is op code,
        //  rest depends on the opcode
        char mag_in[2] = { 0 };

        bcm2835_gpio_write(PA0, LOW);
        bcm2835_spi_transfernb(mag_buf, mag_in, sizeof(mag_in));
        NSEC_DELAY(SPI_DELAY); //
        bcm2835_gpio_write(PA0, HIGH);

        if (mag_in[1] == 0x80) {
            char status[] = { 0xA6 };
            char status_in[3] = { 0 };

            bcm2835_gpio_write(PA0, LOW);
            bcm2835_spi_transfernb(status, status_in, sizeof(status_in));
            NSEC_DELAY(SPI_DELAY);
            bcm2835_gpio_write(PA0, HIGH);

            float mag_reading = (256.0 * status_in[1]) + status_in[2];

            // code for calculating xd

            float mag_position = (360.0 * mag_reading) / 65536.0;
            //#ifdef DEBUG
            //   printf("mag: Reading: %f,	angle: %f,	read: %X, %X\n",
            //   mag_reading,
            //          mag_position, status_in[1], status_in[2]);
            //   fflush(stdout);

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
        }
        else {
            printf("DATA NOT VALID or too early\n");
            printf("%x %x\n\n", mag_in[0], mag_in[1]);
        }
        //			xd = 100.0;
        // bcm2835_gpio_write(MAG_PIN, LOW);
    }
}
