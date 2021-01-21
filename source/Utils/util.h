#ifndef UTIL_H_
#define UTIL_H_

#include "Modules/mRS232.h"
#include "math.h"

#define CAM_HEIGHT 32.8
#define CAM_ANGLE 65 * M_PI / 180.0
#define FOV_X 60
#define FOV_Y 40
#define RES_X 636
#define RES_Y 104

typedef struct {
    double x;
    double y;
} point;

void print_string(const char *string)
{
    mRs232_Uart4WriteString((Int8 *)string);
}

point convert_point(int x, int y)
{
    double angleX = FOV_X * (x / (double)RES_X) - (FOV_X / 2);
    double angleY = CAM_ANGLE + (FOV_Y * (y / (double)RES_Y)) - (FOV_Y / 2);
    
    double pointY = CAM_HEIGHT * tan(angleY);
    
    double beamLength = sqrt(CAM_HEIGHT * CAM_HEIGHT + pointY * pointY);
    
    double pointX = beamLength * tan(angleX);

    point p;
    p.x = pointX;
    p.y = pointY;
    
    return p;
}

double vector_to_angle(double x0, double y0, double x1, double y1){
    double x = x1 - x0;
    double y = y0 - y1;

    return (atan2(y, x) / M_PI) * 180.0;
}

#endif
