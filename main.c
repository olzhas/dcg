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

#define NUM_THREAD 3

pthread_mutex_t mtx_read = PTHREAD_MUTEX_INITIALIZER;

struct position_output pos_out = { 0 };
struct current_output curr_out = { 0 };
struct magnet_output magn_out = { 0 };

void intHandler(int dummy)
{
    puts("Stopping...\n");
    bcm2835_gpio_write(MOTOR_D3, LOW);

    bcm2835_i2c_end();
    bcm2835_spi_end();
    //==========================================================================
    /* ENCODER OUTPUT */
    printf("encoder data: ");
    FILE* fp = fopen(pos_out.filename, "w");
    // Open file for writing, no need to fclose, OS will do it
    if (fp == NULL) {
        printf("Cannot open %s for writing\n", pos_out.filename);
        exit(EXIT_FAILURE); // TODO send sigint to main()
    }

    char buf[] = "2015-12-31 12:59:59.123456789";
    timespec2str(buf, &pos_out.now);
    fprintf(fp, "samples: %lld\n", pos_out.log_iter);
    fprintf(fp, "now: %s\n", buf);
    fprintf(fp, "timestamp\tencoder\n");

    for (size_t i = 0; i < pos_out.log_iter; i++) {
        fprintf(fp, "%d.%09ld\t%lf\t%lf\t%lf\n",
            (int)pos_out.tv_sec[i], pos_out.tv_nsec[i],
            pos_out.x[i], pos_out.xf[i], pos_out.dx[i]);
        fflush(fp);
    }
    fclose(fp);
    puts("done");
    /* ENCODER OUTPUT */
    //==========================================================================

    //==========================================================================
    /* CURRENT OUTPUT */
    printf("current data: ");
    fp = fopen(curr_out.filename, "w");
    // Open file for writing, no need to fclose, OS will do it
    if (fp == NULL) {
        printf("Cannot open %s for writing\n", curr_out.filename);
        exit(EXIT_FAILURE); // TODO send sigint to main()
    }
    timespec2str(buf, &curr_out.now);
    fprintf(fp, "samples: %lld\n", curr_out.log_iter);
    fprintf(fp, "now: %s\n", buf);
    fprintf(fp, "timestamp\tcurrent\n");

    for (size_t i = 0; i < curr_out.log_iter; i++) {
        fprintf(fp, "%d.%09ld\t%f\n",
            (int)curr_out.tv_sec[i], curr_out.tv_nsec[i], curr_out.current[i]);
        fflush(fp);
    }
    fclose(fp);
    puts("done");
    /* CURRENT OUTPUT */
    //==========================================================================

    //==========================================================================
    /* MAGNET OUTPUT */
    printf("magnet encoder data: ");
    fp = fopen(magn_out.filename, "w");
    // Open file for writing, no need to fclose, OS will do it
    if (fp == NULL) {
        printf("Cannot open %s for writing\n", magn_out.filename);
        exit(EXIT_FAILURE); // TODO send sigint to main()
    }
    timespec2str(buf, &magn_out.now);
    fprintf(fp, "samples: %lld\n", magn_out.log_iter);
    fprintf(fp, "now: %s\n", buf);
    fprintf(fp, "timestamp\tmagnet\n");

    for (size_t i = 0; i < magn_out.log_iter; i++) {
        fprintf(fp, "%d.%09ld\t%lf\n",
            (int)magn_out.tv_sec[i], magn_out.tv_nsec[i], magn_out.magn[i]);
        fflush(fp);
    }
    fclose(fp);
    puts("done");
    /* MAGNET OUTPUT */
    //==========================================================================

    bcm2835_close();
}

int main(int argc, char* argv[])
{
    if (argc < 4) {
        fprintf(stderr, "\nInsufficient command line arguments, try again\n");
        fprintf(stderr, "\n%s kp kd x_desired\n\n", argv[0]);
        return EXIT_FAILURE;
    }

    /* configuration stage */
    struct config_ config;

    config.controller_freq = 1e6;
    config.freq_diff = 400.0;
    config.freq_filt = 100.0;
    config.current_range = 2.0; // for range -x to +x put I_range = x
    config.kp = strtod(argv[1], NULL);
    config.kd = strtod(argv[2], NULL);
    config.zero_config = 0;

    uint64_t freq_nanosec[3] = {
        config.controller_freq, // NOTE: quadrature encoder or pd controller freq?
        1e6, // magnetic encoder
        5e6 // power-energy sensor
        // TODO: check if can query the sensor faster (more frequent)
    };

    /* store information about the state */
    struct state_ state;
    state.config = &config;
    state.x_desired = strtod(argv[3], NULL);

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
    for (i = 0; i < NUM_THREAD; i++) {
        if (sigaction(SIGRTMIN + i, &disp, NULL) < 0) {
            syslog(LOG_CRIT, "sigaction_main: %m"); // TODO another methor for logging info
            _exit(EXIT_FAILURE); // exit(EXIT_FAILURE) // ?
        }
    }

    signal(SIGINT, intHandler);

    pthread_t thread[NUM_THREAD];
    sigset_t set[NUM_THREAD];
    int s;

    /* Block SIGQUIT and SIGUSR1; other threads created by main()
       will inherit a copy of the signal mask. */

    struct thread_info_ thread_info[NUM_THREAD];
    struct sched_param sch_param[NUM_THREAD];
    int policy[NUM_THREAD] = { 0 };
    void* task_func[NUM_THREAD] = {
        control_thread,
        magnet_thread,
        energy_thread
    };

    /* telling what signals must be sent to threads */
    sigset_t mask_set;
    sigemptyset(&mask_set);
    for (i = 0; i < NUM_THREAD; i++) {
        sigaddset(&mask_set, SIGRTMIN + i);
    }
    s = pthread_sigmask(SIG_BLOCK, &mask_set, NULL);
    if (s != 0)
        handle_error_en(s, "pthread_sigmask");

    /* starting threads */
    for (i = 0; i < NUM_THREAD; i++) {

        sigemptyset(&set[i]);
        sigaddset(&set[i], SIGRTMIN + i);

        thread_info[i].pstate = &state;
        thread_info[i].pset = &set[i];

        s = pthread_create(&thread[i], NULL, task_func[i], (void*)(&thread_info[i]));

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

    printf("Please press Enter to proceed\n");
    getchar();

    /* Setting up timers */
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);

    struct sigevent sev[NUM_THREAD] = { { 0 } };
    struct itimerspec its[NUM_THREAD];
    timer_t timerid[NUM_THREAD];

    for (i = 0; i < NUM_THREAD; i++) {
        sev[i].sigev_notify = SIGEV_SIGNAL;
        sev[i].sigev_signo = SIGRTMIN + i;

        if (timer_create(CLOCK_REALTIME, &sev[i], &timerid[i]) == -1)
            handle_error_en(-1, "timer_create");

        /* each of timers starts __at the same moment__ */

        its[i].it_value.tv_sec = now.tv_sec + (now.tv_nsec + START_TIMER_DELAY) / 1000000000;
        its[i].it_value.tv_nsec = (now.tv_nsec + START_TIMER_DELAY) % 1000000000; // start in 100ms
        its[i].it_interval.tv_sec = 0;
        its[i].it_interval.tv_nsec = freq_nanosec[i];

        printf("timer ID is 0x%lx\n", (long)timerid[i]);

        if (timer_settime(timerid[i], TIMER_ABSTIME, &its[i], NULL) == -1)
            handle_error_en(-1, "timer_settime");
    }

    bcm2835_gpio_write(MOTOR_D3, HIGH);
    printf("Started.\n");

    printf("\n Press CTRL-C to stop the motor.\n");

    /* PROCEDURES START */
    bcm2835_delay(1000);

    int experiment = 0;

    if (experiment == 0) {

        const int DELAY_KMIN = 60000;
        const int DELAY_STEP = 800;
        const int DELAY_KMAX = 20000;

        state.x_desired = 5;
        bcm2835_delay(DELAY_KMIN);
        for (int i = 10; i <= 86.7; i += 5) {
            state.x_desired = i;
            bcm2835_delay(DELAY_STEP);
        }
        state.x_desired = 86.7;
        bcm2835_delay(DELAY_KMAX);
    }

    if (experiment == 1) {
        printf("Please enter the desired position:\n");
        double x_desired = 10.0;
        while (x_desired < 87.0) {
            scanf("%lf", &x_desired);
            printf("x_desired = %lf\n", x_desired);
            state.x_desired = x_desired;
        }
    }

    if (experiment == 2) {

        const int begin_delay = 1000;
        const int DELAY_KMIN = 20000;
        const int DELAY_STEP = 500;
        const int DELAY_KMAX = 20000;

        state.x_desired = 10;
        bcm2835_delay(begin_delay);

        state.x_desired = 86;
        bcm2835_delay(DELAY_KMIN);
        for (int i = 86; i >= 10; i -= 5) {
            state.x_desired = i;
            bcm2835_delay(DELAY_STEP);
        }
        state.x_desired = 5;
        bcm2835_delay(DELAY_KMAX);
    }
    /* PROCEDURES END */

    intHandler(0);
    /* return is virtually unreachable,
     intHandler handles correct deinitialization of the board*/
    return EXIT_SUCCESS;
}
