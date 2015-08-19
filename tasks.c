#include "tasks.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <bcm2835.h>

#include "declare.h"
#include "encoder.h"
#include "utils.h"

//==============================================================================
void encoder_thread()
{
    ENCODER_init();

    bcm2835_gpio_write(RST_COUNT, HIGH); // now start counting
    printf("Encoder Thread started.\n");

    for (;;) {
        // code for obtaining value from encoder IC
        state->x = ENCODER_read(); // no. of rotations

        // x = (x / 4.81) * 0.002;	// distance traveled by slider in metres

        state->xd = discrete_diff(state); // updating value of dx by calling practical
        // differentiator
        state->x_filtered = low_pass_filter(state); // update xf, the filtered value of x

        state->current_ref = calculate_current_ref(state);
    }
}

//==============================================================================
void calculate_energy()
{
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
