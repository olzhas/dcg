#include "utilities.h"

#include "declare.h"

//==============================================================================
void calculate_I_ref() {
  while (start == 0) {
  }

  while (1) {
    if (encoder_flag)  // encoder flag = 1 means xf and dx are calculated
    {
      // calculate I_ref and convert to pwm value
      I_ref = (float)((kp) * (xd - xf)) - (kd) * (dx);
      // I_range = 2;     // for range -2 to +2, set during motor
      // controller setup

      if (I_ref > I_range)
        I_ref = I_range;
      if (I_ref < (-1.0 * I_range))
        I_ref = (-1.0 * I_range);

      // maxon controller requires PWM value to be 10% and 90%
      // so, I_ref should be scaцled beetween 103 and 921 (10% and 90% of 1024)

      float pwm_value = (204.0 * I_ref) + 512.0;

      bcm2835_pwm_set_data(PWM_CHANNEL, pwm_value);

      // printf("i_ref: X, Xf and Xd = %f,  %f,  %f\n", x, xf, xd);
      // reset flag
      encoder_flag = 0;
    }
  }
}

//==============================================================================
void discrete_diff() {
  static double x_old[2] = {0.0, 0.0};
  static double dx_old[2] = {0.0, 0.0};
  long double tau = 1.0 / (2.0 * 3.14159 * freq_diff);

  double temp1 = (2.0 * tau + dT_PD);
  double temp2 = (2.0 * tau - dT_PD);

  // printf("filt: temps = %.10f,  %.10f\n",temp1, temp2);

  float A = 2.0 * (dT_PD / (temp1 * temp1));
  float B = -1.0 * A;
  float C = 2.0 * (temp2 / temp1);
  float D = -1.0 * (temp2 / temp1) * (temp2 / temp1);
  static float itr = 1;

  dx = A * x + B * x_old[1] + C * dx_old[0] + D * dx_old[1];

  //	printf("A B C D are: %f,  %f,  %f,  %f\n", A, B, C, D);
  // printf("dT, tau are: %f,  %.10f\n", dT, tau);

  if (itr < 2.5) {
    dx = (x - x_old[0]) / dT_PD;
    ++itr;
  }

  dx_old[1] = dx_old[0];
  dx_old[0] = dx;

  x_old[1] = x_old[0];
  x_old[0] = x;

  //	printf("Diff. Done. and dx = %f\n", derivative);
}

//==============================================================================
void low_pass_filter() {
  static float x_old[2] = {0.0, 0.0};
  static float xf_old[2] = {0.0, 0.0};

  float a = 2.0 * 3.14159 * freq_filt;
  float p = 2.0 / dT_PD;

  float A = pow(a / (a + p), 2.0);
  float B = 2.0 * A;
  float C = A;
  float D = -2.0 * (a - p) / (a + p);
  float E = -1.0 * pow(((a - p) / (a + p)), 2.0);
  static float itr = 1;

  xf = (A * x) + B * x_old[0] + C * x_old[1] + D * xf_old[0] + E * xf_old[1];

  //	printf("filt: A B C D are: %f, %f,  %f, %f\n", A, B, C, D);

  if (itr < 2.5) {
    xf = x;
    ++itr;
  }

  xf_old[1] = xf_old[0];
  xf_old[0] = xf;

  x_old[1] = x_old[0];
  x_old[0] = x;

  //	printf("filt: Filtering done and xf = %f\n", xf);
}

//==============================================================================
void discrete_intg() {
  static float pow_old;
  static float E_old;

  float A = dT_PO / 2.0;
  static float itr = 1.0;

  energy = E_old + A * (power + pow_old);  // z transform used to find equation
  //	printf("intg: A and power, energy = %f, %f, %f\n", A, power, energy);

  if (itr < 1.5) {
    energy = (power * dT_PO) / 2.0;
    ++itr;
  }

  E_old = energy;

  pow_old = power;
}
