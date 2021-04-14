#ifndef UTIL_H_
#define UTIL_H_

#include "math.h"
#include "Modules/mRS232.h"

#define TO_RADIANS(x) x * M_PI / 180

typedef struct {
    double x;
    double y;
} point;

void print_string(const char *string) {
    mRs232_Uart4WriteString((Int8 *)string);
};

#endif
