#ifndef DECLARE_H_
#define DECLARE_H_

#include <stdint.h>
#include <signal.h>
#include <pthread.h>
#include <stdio.h>

// define various pins for testing

#define PWM_PIN RPI_BPLUS_GPIO_J8_12 // pwm
#define ENC_PIN RPI_BPLUS_GPIO_J8_38
#define MAG_PIN RPI_BPLUS_GPIO_J8_40

#define PWM_CHANNEL 0
#define RANGE 1024 // pwm range = (0-1024)
#define BILLION 1E9
#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

// defining pins for quadrature decoder (encoder counter)

#define OE_COUNT RPI_BPLUS_GPIO_J8_13 // output enable for encoder counter
#define SEL1 RPI_BPLUS_GPIO_J8_16 // sel 1 for encoder counter
#define SEL2 RPI_BPLUS_GPIO_J8_18 // sel 2 for encoder counter
#define D0 RPI_BPLUS_GPIO_J8_22 // 8 data pins
#define D1 RPI_BPLUS_GPIO_J8_33
#define D2 RPI_BPLUS_GPIO_J8_32
#define D3 RPI_BPLUS_GPIO_J8_31
#define D4 RPI_BPLUS_GPIO_J8_29
#define D5 RPI_BPLUS_GPIO_J8_26
#define D6 RPI_BPLUS_GPIO_J8_24
#define D7 RPI_BPLUS_GPIO_J8_35

#define PA0 RPI_BPLUS_GPIO_J8_15

#define OE_SHIFTER RPI_BPLUS_GPIO_J8_07
#define MOTOR_D3 RPI_BPLUS_GPIO_J8_08
#define RST_COUNT RPI_BPLUS_GPIO_J8_11

#define START_TIMER_DELAY 10000000 // 10e6 = 10ms

struct config_ {
    double freq_diff; // differentiator cut-off frequency
    double freq_filt;
    double kp; // kp for PD loop
    double kd; // kd for PD loop
    double current_range; // for calculating PWM value
    uint32_t controller_freq;
    uint64_t zero_shift;
    uint8_t zero_config;
};

struct state_ {
    double x; // position
    double x_filtered; // filtered value of position
    double x_desired; // desired value of position
    double dx; // motor speed
    double current_ref; // reference current (control action)
    double power; // instantaneous power measured by IC
    double energy;
    struct config_* config;
};

struct thread_info_ {
    struct state_* pstate;
    sigset_t* pset;
};

#define A_LOT 1024000

struct position_output {
    struct timespec now;
    char* filename;
    time_t tv_sec[A_LOT];
    long tv_nsec[A_LOT];
    double x[A_LOT];
    double xf[A_LOT];
    double dx[A_LOT];
    uint64_t log_iter;
};

struct current_output {
    struct timespec now;
    char* filename;
    time_t tv_sec[A_LOT];
    long tv_nsec[A_LOT];
    float current[A_LOT];
    uint64_t log_iter;
};

struct magnet_output {
    struct timespec now;
    char* filename;
    time_t tv_sec[A_LOT];
    long tv_nsec[A_LOT];
    double magn[A_LOT];
    uint64_t log_iter;
};

#endif // DECLARE_H_
