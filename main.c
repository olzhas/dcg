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

int main(int argc, char* argv[])
{
    if (argc < 3) {
        fprintf(stderr, "\nInsufficient command line arguments, try again\n");
        fprintf(stderr, "\n%s kp kd\n\n", argv[0]);
        return EXIT_FAILURE;
    }

    /* configuration stage */
    struct config_ config;

    config.controller_freq = 1e6;
    config.freq_diff = 200.0;
    config.freq_filt = 200.0;
    config.current_range = 2.0; // for range -x to +x put I_range = x
    config.kp = strtod(argv[1], NULL);
    config.kd = strtod(argv[2], NULL);

    uint64_t freq_nanosec[3] = {
        config.controller_freq, // FIXME: quadrature encoder or pd controller freq?
        1e6, // magnetic encoder
        10e6 // power-energy sensor
        // TODO: check if can query the sensor faster (more frequent)
    };

    /* store information about the state */
    struct state_ state;
    state.config = &config;

    /* init the library */
    if (bcm2835_init() == 0)
        exit(EXIT_FAILURE);

    PWM_init();

    /* setting up timers */

    // main thread should ignore SIGRTMIN to SIGRTMAX
    int i;
    struct sigaction disp;

    bzero(&disp, sizeof(disp));
    disp.sa_handler = SIG_IGN;
    for (i = 0; i < 1; i++) {
        if (sigaction(SIGRTMIN + i, &disp, NULL) < 0) {
            syslog(LOG_CRIT, "sigaction_main: %m"); // TODO another methor for logging info
            _exit(EXIT_FAILURE);
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
        control_thread, magnet_thread, calculate_energy
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

    for (i = 0; i < 2; i++) {
        sev[i].sigev_notify = SIGEV_SIGNAL;
        sev[i].sigev_signo = SIGRTMIN + i;

        if (timer_create(CLOCK_REALTIME, &sev[i], &timerid[i]) == -1)
            handle_error_en(-1, "timer_create");

        its[i].it_value.tv_sec = now.tv_sec + (now.tv_nsec + START_TIMER_DELAY) / 1000000000;
        its[i].it_value.tv_nsec = (now.tv_nsec + START_TIMER_DELAY) % 1000000000; // start in 100ms
        its[i].it_interval.tv_sec = 0;
        its[i].it_interval.tv_nsec = freq_nanosec[i];

        printf("timer ID is 0x%lx\n", (long)timerid[i]);

        if (timer_settime(timerid[i], TIMER_ABSTIME, &its[i], NULL) == -1)
            handle_error_en(-1, "timer_settime");
    }
    /* Main thread carries on to create other threads and/or do
       other work */
    /*
    //FIXME integrate old part
    bcm2835_delay(300);
    // delay to make sure that all threads are initialised
    // and iC-MU is conofigured
    printf("\nPress enter to start the motor.");
    getchar();
    bcm2835_gpio_write(MOTOR_D3, HIGH);

    printf("Started.\n");

    printf("\nPress enter to stop the motor.\n");
    getchar();
    bcm2835_gpio_write(MOTOR_D3, LOW);

    bcm2835_spi_end();
    bcm2835_close();
    */
    return EXIT_SUCCESS;
}
