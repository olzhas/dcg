#ifndef TASKS_H_
#define TASKS_H_

#include <stdlib.h>
#include <stdio.h>
#include <bcm2835.h>

//==============================================================================

void control_thread();

//==============================================================================

void energy_thread();

//==============================================================================

void magnet_thread();

#endif // TASKS_H_
