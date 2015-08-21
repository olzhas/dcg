#include "tasks.h"

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>
#include <bcm2835.h>
#include <strings.h>
#include <syslog.h>

#include "declare.h"
#include "encoder.h"
#include "utils.h"

//==============================================================================
void control_thread(void* data)
{
    /* TODO encapsulate this */
    struct sigaction disp;

    bzero(&disp, sizeof(disp));
    disp.sa_handler = SIG_IGN;

    if (sigaction(SIGRTMIN + 1, &disp, NULL) < 0) {
        syslog(LOG_CRIT, "sigaction_main: %m");
        _exit(1);
    }

    /* signal handling routines */
    struct thread_info_* thread_info = (struct thread_info_*)data;
    sigset_t* set = thread_info->pset;
    int s;
    siginfo_t sig;

    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 1000;

    struct state_* pstate = thread_info->pstate;
    /*==========================*/

    // TODO move somewhere else
    ENCODER_init();

    bcm2835_gpio_write(RST_COUNT, HIGH); // now start counting
    //

    printf("control thread started\n");

    for (;;) {
        s = sigtimedwait(set, &sig, &timeout); // locks execution
        if (s >= 0) {
            if (s != SIGRTMIN) {
                write(STDERR_FILENO, "wrong signal\n", sizeof("wrong signal\n"));
                continue;
            }

            // code for obtaining value from encoder IC
            pstate->x = ENCODER_read(); // no. of rotations

            // x = (x / 4.81) * 0.002;	// distance traveled by slider in metres

            pstate->dx = discrete_diff(pstate);
            pstate->x_filtered = low_pass_filter(pstate);
            pstate->current_ref = calculate_current_ref(pstate);

            // maxon controller requires PWM value to be 10% and 90%
            // so, current_ref should be scaled beetween 103 and 921 (10% and 90% of 1024)

            uint32_t pwm_value = 204 * pstate->current_ref + 512.0;
            //printf("PWM command: %d\n", pwm_value);
            bcm2835_pwm_set_data(PWM_CHANNEL, pwm_value);
        }
        else {
            switch (errno) {
            case EAGAIN:
            case EINTR:
                break;
            default:
                handle_error_en(s, "sigwait");
                break;
            }
        }
    }
}

//==============================================================================
void calculate_energy(void* data)
{
    struct state_* pstate = (struct state_*)data;

    uint8_t send;

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

    float su2 = 0.0;

    send = bcm2835_i2c_write(config_write, 3);
    send = bcm2835_i2c_write(calib_write, 3);

    for (;;) {
        // TODO sigwait
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
        pstate->power = ((float)pow_16 * 20.0) / 1000.0; // (in W)
        if (current < 0)
            pstate->power *= -1.0;

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
void magnet_thread(void* data)
{

    /* TODO encapsulate this */
    struct sigaction disp;

    bzero(&disp, sizeof(disp));
    disp.sa_handler = SIG_IGN;

    if (sigaction(SIGRTMIN, &disp, NULL) < 0) {
        syslog(LOG_CRIT, "sigaction_main: %m");
        _exit(1);
    }

    /* signal handling routines */
    struct thread_info_* thread_info = (struct thread_info_*)data;
    sigset_t* set = thread_info->pset;
    int s;
    siginfo_t sig;

    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 1000;

    struct state_* pstate = thread_info->pstate;
    /*==========================*/

    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST); // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0); // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); // The default
    //bcm2835_spi_chipSelect(BCM2835_SPI_CS0); // The default
    //bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); // the default

    bcm2835_gpio_fsel(D6, BCM2835_GPIO_FSEL_INPT); // ATTENTION: If there's a problem in reading encoder data (the D6 and/or D5 Pins), this is the issue. Can be solved by changing the library.
    bcm2835_gpio_fsel(D5, BCM2835_GPIO_FSEL_INPT);

    printf("Magnet thread started.\n");

    for (;;) {

        s = sigtimedwait(set, &sig, &timeout); // locks execution
        if (s >= 0) {
            if (s != SIGRTMIN + 1) { // TODO make it automatic
                write(STDERR_FILENO, "wrong signal\n", sizeof("wrong signal\n"));
                continue;
            }

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
                printf("mag: Reading: %f,	angle: %f,	read: %X, %X\n",
                    mag_reading,
                    mag_position, status_in[1], status_in[2]);
                fflush(stdout);
                /*
                if ((status_in[1] == status_in[2]) && status_in[1] == 0) {
                    printf("%x %x %x\n", status_in[0], status_in[1], status_in[2]);
                    printf(
                        "##############################################################\n"
                        "##############################################################\n"
                        "##############################################################\n");
                    return;
                }
                */
                //#endif

                //pstate->x_desired = mag_position / 2.0;
            }
            else {
                printf("DATA NOT VALID or too early\n");
                printf("%x %x\n\n", mag_in[0], mag_in[1]);
            }
        }
        else {
            switch (errno) {
            case EAGAIN:
            case EINTR:
                break;
            default:
                handle_error_en(s, "sigwait");
                break;
            }
        }

        //			xd = 100.0;
        // bcm2835_gpio_write(MAG_PIN, LOW);
    }
}
