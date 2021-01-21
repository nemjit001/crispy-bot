#ifndef UTIL_H_
#define UTIL_H_

#include "Modules/mRS232.h"
#include "math.h"

#define CAM_HEIGHT 34
#define CAM_ANGLE 54 * M_PI / 180.0
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

#endif
