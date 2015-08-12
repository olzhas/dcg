#ifndef ENCODER_THREADS_H_
#define ENCODER_THREADS_H_

#include <stdio.h>
#include <math.h>
#include <bcm2835.h>

void encoder_thread(void);
float calculate_encoder(void);

#endif  // ENCODER_THREADS_H_
