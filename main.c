#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <bcm2835.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <strings.h>
#include <sys/syslog.h>
#include <stdint.h>

#include "declare.h"
#include "tasks.h"
#include "utils.h"

//==============================================================================
int main(int argc, char* argv[])
{
    struct state_ state;
    struct config_ config;

    if (argc < 3) {
        fprintf(stderr, "\nInsufficient command line arguments, try again\n");
        fprintf(stderr, "\n%s kp kd\n\n", argv[0]);
        return EXIT_FAILURE;
    }
    else {
        config.kp = strtod(argv[1], NULL);
        config.kd = strtod(argv[2], NULL);
    }

    if (bcm2835_init() == false) {
        exit(EXIT_FAILURE);
    }

    // setting PWM_PIN as pwm from channel 0 in markspace mode with range = RANGE
    bcm2835_gpio_fsel(PWM_PIN, BCM2835_GPIO_FSEL_ALT5); // ALT5 is pwm mode
    bcm2835_pwm_set_clock(
        BCM2835_PWM_CLOCK_DIVIDER_16); // pwm freq = 19.2 / 16 MHz
    bcm2835_pwm_set_mode(PWM_CHANNEL, 1, 1); // markspace mode
    bcm2835_pwm_set_range(PWM_CHANNEL, RANGE);

    bcm2835_gpio_fsel(OE_SHIFTER, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_set_pud(
        OE_SHIFTER,
        BCM2835_GPIO_PUD_DOWN); // pull-down for output enable of logic shifters

    bcm2835_gpio_fsel(MOTOR_D3, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_set_pud(MOTOR_D3,
        BCM2835_GPIO_PUD_DOWN); // pull-down for motor enable

    bcm2835_gpio_fsel(PA0, BCM2835_GPIO_FSEL_OUTP);
    bcm2835_gpio_set_pud(PA0, BCM2835_GPIO_PUD_UP);
    bcm2835_gpio_write(PA0, HIGH);

    bcm2835_gpio_write(OE_SHIFTER, HIGH);
    bcm2835_gpio_write(MOTOR_D3, LOW);

    /* setting up timers */

    // main thread should ignore SIGRTMIN to SIGRTMAX
    int i;
    int n;
    struct sigaction disp;

    bzero(&disp, sizeof(disp));
    disp.sa_handler = SIG_IGN;
    for (i = 0; i < 2; i++) {
        if (sigaction(SIGRTMIN + i, &disp, NULL) < 0) {
            syslog(LOG_CRIT, "sigaction_main: %m");
            _exit(1);
        }
    }

    pthread_t thread[2];
    sigset_t set[2];
    int s;

    /* Block SIGQUIT and SIGUSR1; other threads created by main()
       will inherit a copy of the signal mask. */

    struct sched_param sch_param[2];
    int policy[2] = { 0, 0 };
    void* task_func[] = {
        magnet_thread, encoder_thread, calculate_energy, calculate_I_ref
    };

    sigset_t mask_set;
    sigemptyset(&mask_set);
    for (i = 0; i < 2; i++) {
        sigaddset(&mask_set, SIGRTMIN + i);
    }
    s = pthread_sigmask(SIG_BLOCK, &mask_set, NULL);
    if (s != 0)
        handle_error_en(s, "pthread_sigmask");

    for (i = 0; i < 2; i++) {
        sigemptyset(&set[i]);
        sigaddset(&set[i], SIGRTMIN + i);

        s = pthread_create(&thread[i], NULL, task_func[i], (void*)(&set[i]));

        if (s != 0)
            handle_error_en(s, "pthread_create");

        s = pthread_getschedparam(thread[i], &policy[i], &sch_param[i]);
        if (s != 0)
            handle_error_en(s, "pthread_getschedparam");

        policy[i] = SCHED_RR;
        sch_param[i].sched_priority = sched_get_priority_max(SCHED_RR);

        s = pthread_setschedparam(thread[i], policy[i], &sch_param[i]);
        if (s != 0)
            handle_error_en(s, "pthread_setschedparam");
    }

    getchar();

    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    struct sigevent sev[2] = { 0 };
    struct itimerspec its[2];
    timer_t timerid[2];

    uint64_t freq_nanosec[2] = { 100e3, 100e3 };
    uint64_t start_delay = 200e6;

    for (i = 0; i < 2; i++) {
        sev[i].sigev_notify = SIGEV_SIGNAL;
        sev[i].sigev_signo = SIGRTMIN + i;

        if (timer_create(CLOCK_REALTIME, &sev[i], &timerid[i]) == -1)
            handle_error_en(-1, "timer_create");

        its[i].it_value.tv_sec = now.tv_sec + (now.tv_nsec + start_delay) / 1000000000;
        its[i].it_value.tv_nsec = (now.tv_nsec + start_delay) % 1000000000; // start in 100ms
        its[i].it_interval.tv_sec = 0;
        its[i].it_interval.tv_nsec = freq_nanosec[i];

        printf("timer ID is 0x%lx\n", (long)timerid[i]);

        if (timer_settime(timerid[i], TIMER_ABSTIME, &its[i], NULL) == -1)
            handle_error_en(-1, "timer_settime");
    }
    /* Main thread carries on to create other threads and/or do
       other work */

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
