#include "utils.h"

#include <stdint.h>
#include <stdio.h>
#include <time.h>
#include <bcm2835.h>
#include <math.h>

#include "declare.h"

//==============================================================================
double calculate_current_ref(const struct state_* pstate)
{
#define DEBUG
#ifdef DEBUG //TODO come up with better logging technique
    printf("x_desired = %f, x = %f, x_filtered = %f, dx = %f\n",
        pstate->x_desired, pstate->x, pstate->x_filtered, pstate->dx);
#endif

    double current_ref;
    double kp = pstate->config->kp;
    double kd = pstate->config->kd;
    double xd = pstate->x_desired;
    double xf = pstate->x_filtered;
    double dx = pstate->dx;

    // calculate I_ref and convert to pwm value
    current_ref = (double)((kp) * (xd - xf)) - (kd) * (dx);

    // #ifdef DEBUG
    //     printf("%lf\n", current_ref);
    // #endif
    // limitting current
    double current_range = pstate->config->current_range;
    if (current_ref > current_range) {
        current_ref = current_range;
    }
    else if (current_ref < -current_range) {
        current_ref = -current_range;
    }

    // #ifdef DEBUG
    //     printf("%lf\n", current_ref);
    // #endif
    return current_ref;
}

//==============================================================================
double discrete_diff(const struct state_* pstate)
{
    static double x_old[2] = { 0.0, 0.0 };
    static double dx_old[2] = { 0.0, 0.0 };

    double freq_diff = pstate->config->freq_diff;
    double dT_PD = (double)pstate->config->controller_freq / 1e9;

    long double tau = 1.0 / (2.0 * M_PI * freq_diff);

    double temp1 = (2.0 * tau + dT_PD);
    double temp2 = (2.0 * tau - dT_PD);

    // printf("filt: temps = %.10f,  %.10f\n",temp1, temp2);

    double A = 2.0 * (dT_PD / (temp1 * temp1));
    double B = -1.0 * A;
    double C = 2.0 * (temp2 / temp1);
    double D = -1.0 * (temp2 / temp1) * (temp2 / temp1);
    static int itr = 1;

    double dx;
    double x = pstate->x;
    dx = A * x + B * x_old[1] + C * dx_old[0] + D * dx_old[1];

    //printf("A B C D are: %f,  %f,  %f,  %f\n", A, B, C, D);
    //printf("dT, tau are: %f,  %.10f\n", dT, tau);

    if (itr < 2) {
        dx = (x - x_old[0]) / dT_PD;
        ++itr;
    }

    dx_old[1] = dx_old[0];
    dx_old[0] = pstate->dx;

    x_old[1] = x_old[0];
    x_old[0] = pstate->x;

    //	printf("Diff. Done. and dx = %f\n", derivative);
}

//==============================================================================
double low_pass_filter(const struct state_* pstate)
{
    static float x_old[2] = { 0.0, 0.0 };
    static float xf_old[2] = { 0.0, 0.0 };

    double dT_PD = pstate->config->controller_freq / 1e9;

    double a = 2.0 * M_PI * pstate->config->freq_filt;
    double p = 2.0 / dT_PD;

    double A = pow(a / (a + p), 2.0);
    double B = 2.0 * A;
    double C = A;
    double D = -2.0 * (a - p) / (a + p);
    double E = -1.0 * pow(((a - p) / (a + p)), 2.0);

    static int itr = 1;

    double xf;
    double x = pstate->x;

    xf = (A * x) + B * x_old[0] + C * x_old[1] + D * xf_old[0] + E * xf_old[1];

    //	printf("filt: A B C D are: %f, %f,  %f, %f\n", A, B, C, D);

    if (itr < 2) {
        xf = x;
        ++itr;
    }

    xf_old[1] = xf_old[0];
    xf_old[0] = xf;

    x_old[1] = x_old[0];
    x_old[0] = x;

    //	printf("filt: Filtering done and xf = %f\n", xf);
    return xf;
}

//==============================================================================
void discrete_intg(const struct state_* pstate)
{
    /*
    //TODO verify the implementation
    static float pow_old;
    static float E_old;

    //float A = dT_PO / 2.0; // FIXME n
    static float itr = 1.0;

    //pstate->energy = E_old + A * (pstate->power + pow_old); // z transform used to find equation // FIXME
    //	printf("intg: A and power, energy = %f, %f, %f\n", A, power, energy);

    if (itr < 1.5) {
        //pstate->energy = (pstate->power * dT_PO) / 2.0; //FIXME
        ++itr;
    }

    E_old = pstate->energy;

    pow_old = pstate->power;
    */
}

//==============================================================================
// TODO add error messages
void PWM_init()
{
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
}
