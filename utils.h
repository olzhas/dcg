#ifndef UTILS_H_
#define UTILS_H_

void calculate_I_ref();
void discrete_diff();
void low_pass_filter();
void discrete_intg();

#define handle_error_en(en, msg) \
    do {                         \
        /*errno = en;*/          \
        perror(msg);             \
        exit(EXIT_FAILURE);      \
    } while (0)

#endif // UTILS_H_
