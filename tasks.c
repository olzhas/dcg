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

#define DURATION 300000
#define NUM_THREAD 3

extern pthread_mutex_t mtx_read;
extern struct position_output pos_out;
extern struct current_output curr_out;
extern struct magnet_output magn_out;

//==============================================================================
void control_thread(void* data)
{
    int sig_ign_len = 2;
    int sig_ignore[] = { SIGRTMIN + 1, SIGRTMIN + 2 };
    /* TODO encapsulate this */
    struct sigaction disp;

    bzero(&disp, sizeof(disp));
    disp.sa_handler = SIG_IGN;
    for (int i = 0; i < sig_ign_len; i++) {
        if (sigaction(sig_ignore[i], &disp, NULL) < 0) {
            syslog(LOG_CRIT, "sigaction_main: %m");
            _exit(1);
        }
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

    pos_out.filename = get_filename("shaftencoder");
    //measure time

    if (clock_gettime(CLOCK_REALTIME, &pos_out.now) != 0) { // is it a good practice ?
        fprintf(stderr, "clock_gettime, energy");
        exit(EXIT_FAILURE);
    }

    printf("control thread started\n");

    struct timespec toc;

    pos_out.log_iter = 0;
    bcm2835_gpio_fsel(RST_COUNT, BCM2835_GPIO_FSEL_OUTP); // reset count
    bcm2835_gpio_write(RST_COUNT, HIGH); // now start counting
    bcm2835_delayMicroseconds(1);
    bcm2835_gpio_write(RST_COUNT, LOW); // now start counting
    //bcm2835_delayMicroseconds(1);
    //bcm2835_gpio_write(RST_COUNT, HIGH); // now start counting
    for (;;) {

        s = sigtimedwait(set, &sig, &timeout); // locks execution
        if (s >= 0) {
            if (s != SIGRTMIN) {
                write(STDERR_FILENO, "wrong signal\n", sizeof("wrong signal\n"));
                continue;
            }

            // code for obtaining value from encoder IC
            bcm2835_gpio_write(RST_COUNT, HIGH);
            pstate->x = ENCODER_read(); // mm distance

            // x = (x / 4.81) * 0.002;	// distance traveled by slider in metres

            pstate->dx = discrete_diff(pstate);
            pstate->x_filtered = low_pass_filter(pstate);
            pstate->current_ref = calculate_current_ref(pstate);

            // printf("desired = %f, filtered = %f, current = %f\n",
            //     pstate->x_desired, pstate->x_filtered, pstate->current_ref);

            // maxon controller requires PWM value to be 10% and 90%
            // so, current_ref should be scaled beetween 103 and 921 (10% and 90% of 1024)

            uint32_t pwm_value
                = 204 * pstate->current_ref + 512.0;

            //printf("PWM command: %d\n", pwm_value);

            bcm2835_pwm_set_data(PWM_CHANNEL, pwm_value);

            //=================================================================

            clock_gettime(CLOCK_REALTIME, &toc);

            toc.tv_sec = toc.tv_sec - pos_out.now.tv_sec;
            toc.tv_nsec = toc.tv_nsec - pos_out.now.tv_nsec;
            if (toc.tv_nsec < 0) {
                toc.tv_nsec += 1000000000L;
                toc.tv_sec--;
            }
            pos_out.tv_sec[pos_out.log_iter] = toc.tv_sec;
            pos_out.tv_nsec[pos_out.log_iter] = toc.tv_nsec;
            pos_out.x[pos_out.log_iter] = pstate->x;
            pos_out.xf[pos_out.log_iter] = pstate->x_filtered;
            pos_out.dx[pos_out.log_iter] = pstate->dx;
            ++pos_out.log_iter;
            //fprintf(fp, "%d.%09ld\t%lf\t%lf\t%lf\n", (int)toc.tv_sec, toc.tv_nsec, pstate->x, pstate->x_filtered, pstate->dx);
            //fflush(fp);
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

void energy_thread(void* data)
{

    int sig_ign_len = 2;
    int sig_ignore[] = { SIGRTMIN, SIGRTMIN + 1 };
    /* TODO encapsulate this */
    struct sigaction disp;

    bzero(&disp, sizeof(disp));
    disp.sa_handler = SIG_IGN;
    for (int i = 0; i < sig_ign_len; i++) {
        if (sigaction(sig_ignore[i], &disp, NULL) < 0) {
            syslog(LOG_CRIT, "sigaction_main: %m");
            _exit(1);
        }
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

    uint8_t send;

    char volt[2] = { 0x00, 0x00 };
    char curr[2] = { 0x00, 0x01 };
    char pwr[2] = { 0x00, 0x00 };
    char bus_volt[2] = { 0x00, 0x00 };

    uint8_t write_address = 0x40; // find using i2cdetect -y 1

    char config_write[3] = { 0x00, 0x39,
        0x9F }; // first byte is address, next two are data
    char calib_write[3] = { 0x05, 0x10, 0x00 }; // for value 0x1000 the current LSB
    // = 1 e-3 A. (see datasheet on how to determine calibration register value)
    // page 17/41

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

    pthread_mutex_lock(&mtx_read);

    bcm2835_i2c_begin(); // I2C begin
    bcm2835_i2c_set_baudrate(100000);

    bcm2835_i2c_setSlaveAddress(write_address); // write
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_626);

    send = bcm2835_i2c_write(config_write, 3);
    if (send != BCM2835_I2C_REASON_OK) {
        fprintf(stderr, "bcm2835_i2c_write error: %d\n", send);
        exit(EXIT_FAILURE); // TODO send sigint to main()
    }

    send = bcm2835_i2c_write(calib_write, 3);
    if (send != BCM2835_I2C_REASON_OK) {
        fprintf(stderr, "bcm2835_i2c_write error: %d\n", send);
        exit(EXIT_FAILURE); // TODO send sigint to main()
    }

    pthread_mutex_unlock(&mtx_read);

    curr_out.filename = get_filename("power");

    //measure time

    if (clock_gettime(CLOCK_REALTIME, &curr_out.now) != 0) { // is it a good practice ?
        fprintf(stderr, "clock_gettime, energy");
        exit(EXIT_FAILURE);
    }

    curr_out.log_iter = 0;

    for (;;) {
        s = sigtimedwait(set, &sig, &timeout); // locks execution
        if (s >= 0) {
            if (s != SIGRTMIN + 2) {
                write(STDERR_FILENO, "wrong signal\n", sizeof("wrong signal\n"));
                continue;
            }

            pthread_mutex_lock(&mtx_read);

            //send = bcm2835_i2c_read_register_rs(voltage_addr, volt, 2);
            send = bcm2835_i2c_read_register_rs(current_addr, curr, 2);
            send = bcm2835_i2c_read_register_rs(bus_addr, bus_volt, 2);
            send = bcm2835_i2c_read_register_rs(power_addr, pwr, 2);
            pthread_mutex_unlock(&mtx_read);

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

            pstate->energy = discrete_integ(pstate, 0.01); // update value of energy
            // NOTE think about better interface for integration routines

            // calculate power
            float power_cal = bus_voltage * current;

            //			printf("pwr: V, I, P_cal, P_meas = %f, %f, %f,
            //%f \n", bus_voltage, current, power_cal, power);
            struct timespec toc;
            clock_gettime(CLOCK_REALTIME, &toc);

            toc.tv_sec = toc.tv_sec - curr_out.now.tv_sec;
            toc.tv_nsec = toc.tv_nsec - curr_out.now.tv_nsec;
            if (toc.tv_nsec < 0) {
                toc.tv_nsec += 1000000000L;
                toc.tv_sec--;
            }

            curr_out.tv_sec[curr_out.log_iter] = toc.tv_sec;
            curr_out.tv_nsec[curr_out.log_iter] = toc.tv_nsec;
            curr_out.current[curr_out.log_iter] = (double)current;
            curr_out.log_iter++;
            // fprintf(fp, "%d.%09ld\t%f\n", (int)toc.tv_sec, toc.tv_nsec, current);
            // fflush(fp); // if not called there is a possibility to lose some data if program exited abnormally
            //fprintf(stdout, "%f\t%f\n", current, t);
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

// TODO place somewhere else, looks random and ugly
#define SPI_DELAY 1000 // in nanoseconds

#define NSEC_DELAY(DURATION)                                        \
    clock_nanosleep(CLOCK_MONOTONIC, 0,                             \
        (struct timespec[]){ {.tv_sec = 0, .tv_nsec = DURATION } }, \
        NULL);

//==============================================================================
void magnet_thread(void* data)
{
    int sig_ign_len = 2;
    int sig_ignore[] = { SIGRTMIN, SIGRTMIN + 2 };
    /* TODO encapsulate this */
    struct sigaction disp;

    bzero(&disp, sizeof(disp));
    disp.sa_handler = SIG_IGN;
    for (int i = 0; i < sig_ign_len; i++) {
        if (sigaction(sig_ignore[i], &disp, NULL) < 0) {
            syslog(LOG_CRIT, "sigaction_main: %m");
            _exit(1);
        }
    }

    /* signal handling routines */
    struct thread_info_* thread_info = (struct thread_info_*)data;
    sigset_t* set = thread_info->pset;
    int s;
    siginfo_t sig;

    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 1000;

    /*==========================*/
    pthread_mutex_lock(&mtx_read);
    bcm2835_spi_begin();
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST); // The default
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0); // The default
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_64); // The default
    pthread_mutex_unlock(&mtx_read);
    //bcm2835_spi_chipSelect(BCM2835_SPI_CS0); // The default
    //bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW); // the default

    printf("Magnet thread started.\n");

    NSEC_DELAY(1000);

    printf("Magnet sensor activated.\n");

    magn_out.filename = get_filename("magnet");

    if (clock_gettime(CLOCK_REALTIME, &magn_out.now) != 0) { // is it a good practice ?
        fprintf(stderr, "clock_gettime, energy");
        exit(EXIT_FAILURE);
    }

    magn_out.log_iter = 0;

    // NOTE calibration of magnet, zero
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
            char mag_in[3] = { 0 };

            // mutex lock

            int t = 0; // try 10 times
            pthread_mutex_lock(&mtx_read);
            while (mag_in[1] != 0x80 && t < 10) {
                bcm2835_gpio_write(PA0, LOW);
                bcm2835_spi_transfernb(mag_buf, mag_in, sizeof(mag_in));
                NSEC_DELAY(500);
                bcm2835_gpio_write(PA0, HIGH);
                t++;
            }
            pthread_mutex_unlock(&mtx_read);

            if (t > 1)
                printf("t %d\n ", t);

            if (t < 10) {

                char status[] = { 0xA6 };
                char status_in[3] = { 0 };
                pthread_mutex_lock(&mtx_read);
                bcm2835_gpio_write(PA0, LOW);
                bcm2835_spi_transfernb(status, status_in, sizeof(status_in));
                NSEC_DELAY(SPI_DELAY);
                bcm2835_gpio_write(PA0, HIGH);
                pthread_mutex_unlock(&mtx_read);

                double mag_reading = (256.0 * status_in[1]) + status_in[2];

                // code for calculating xd

                double mag_position = (360.0 * mag_reading) / 65536.0;

                //=================================================================
                struct timespec toc;
                clock_gettime(CLOCK_REALTIME, &toc);

                toc.tv_sec = toc.tv_sec - magn_out.now.tv_sec;
                toc.tv_nsec = toc.tv_nsec - magn_out.now.tv_nsec;
                if (toc.tv_nsec < 0) {
                    toc.tv_nsec += 1000000000L;
                    toc.tv_sec--;
                }

                magn_out.tv_sec[magn_out.log_iter] = toc.tv_sec;
                magn_out.tv_nsec[magn_out.log_iter] = toc.tv_nsec;
                magn_out.magn[magn_out.log_iter] = mag_position;
                magn_out.log_iter++;
//==============================================================================

#define DEBUG
#ifdef DEBUG

                // printf("mag: Reading: %.3f,	angle: %.3f,	read: %x, %X, %X\n",
                //     mag_reading,
                //     mag_position, status_in[0], status_in[1], status_in[2]);
                // fflush(stdout);

                if ((status_in[1] == status_in[2]) && status_in[1] == 0) {
                    printf("%x %x %x\n", status_in[0], status_in[1], status_in[2]);

                    printf(
                        "##############################################################\n"
                        "##############################################################\n"
                        "##############################################################\n");

                    //exit(EXIT_FAILURE);
                }
#endif
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
    }
}
