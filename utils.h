#ifndef UTILS_H_
#define UTILS_H_
#include "declare.h"

// calculation routines
//==============================================================================
double calculate_current_ref(const struct state_* pstate);
//==============================================================================
double discrete_diff(const struct state_* pstate);
//==============================================================================
double low_pass_filter(const struct state_* pstate);
//==============================================================================
double discrete_integ(const struct state_* pstate, const double freq);
//==============================================================================

// hardware routines
//==============================================================================
void PWM_init();
//==============================================================================

#define handle_error_en(en, msg) \
    do {                         \
        /*errno = en;*/          \
        perror(msg);             \
        exit(EXIT_FAILURE);      \
    } while (0)

#endif // UTILS_H_
