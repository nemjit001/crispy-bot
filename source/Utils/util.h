#ifndef UTIL_H_
#define UTIL_H_

#include "Modules/mRS232.h"
#include "math.h"

#define TO_RADIANS(x) x * M_PI / 180

typedef struct {
    double x;
    double y;
} point;

void print_string(const char *string)
{
    mRs232_Uart4WriteString((Int8 *)string);
}

double vector_to_angle(double x0, double y0, double x1, double y1){
    double x = x1 - x0;
    double y = y1 - y0;

    return (atan2(x, y) / M_PI) * 180.0;
}

#endif
