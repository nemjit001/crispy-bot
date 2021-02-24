#ifndef UTIL_H_
#define UTIL_H_

#include "Modules/mRS232.h"
#include "math.h"

#define CAM_HEIGHT 30.5
#define CAM_ANGLE (62 * M_PI / 180.0)
#define FOV_X (60 * M_PI / 180.0)
#define FOV_Y (40 * M_PI / 180.0)
#define RES_X 316
#define RES_Y 208

typedef struct {
    double x;
    double y;
} point;

void print_string(const char *string)
{
    mRs232_Uart4WriteString((Int8 *)string);
}

point convert_point(int x, int y) {
    double angleX = FOV_X * (x / (double)RES_X) - (FOV_X / 2.0);
    double angleY = CAM_ANGLE + (FOV_Y * ((RES_Y - y) / (double)RES_Y)) - (FOV_Y / 2.0);
    
    double pointY = CAM_HEIGHT * tan(angleY);
    
    double beamLength = sqrt(CAM_HEIGHT * CAM_HEIGHT + pointY * pointY);
    
    double pointX = beamLength * tan(angleX);

    point p;
    p.x = pointX;
    p.y = pointY;

    //char data2[64];
	//sprintf(data2, "x: %d, y: %d -> x: %d, y: %d", x, y, (int)pointX, (int)pointY);

	//print_string(data2);
    
    return p;
}

double vector_to_angle(double x0, double y0, double x1, double y1){
    double x = x1 - x0;
    double y = y0 - y1;

    return (atan2(x, y) / M_PI) * 180.0;
}

#endif
