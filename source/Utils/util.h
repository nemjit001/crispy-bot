#ifndef UTIL_H_
#define UTIL_H_

#include "Modules/mRS232.h"
#include "math.h"

#define CAM_HEIGHT 39.5     // In cm, lens tot grond
#define LINE_DIST 30.0      // In cm, wiel tot lijn
#define LENS_WHEELS_DIST 9.0  // In cm, lens tot wiel, horizontaal
#define CAM_ANGLE atan2(CAM_HEIGHT, LINE_DIST + LENS_WHEELS_DIST)
#define FOV_X (60 * M_PI / 180.0)
#define FOV_Y (40 * M_PI / 180.0)
#define RES_X 79
#define RES_Y 52

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
