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
    float x;
    float y;
} point;

void print_string(char *string){
    mRs232_Uart4WriteString((Int8 *)string);
}

point convert_point(int x, int y) {
    float angleX = FOV_X * (x / (float)RES_X) - (FOV_X / 2);
    float angleY = CAM_ANGLE + (FOV_Y * (y / (float)RES_Y)) - (FOV_Y / 2);
    
    float pointY = CAM_HEIGHT * tan(angleY);
    
    float beamLength = sqrt(CAM_HEIGHT * CAM_HEIGHT + pointY * pointY);
    
    float pointX = beamLength * tan(angleX);

    point p;
    p.x = pointX;
    p.y = pointY;
    
    return p;
}

#endif