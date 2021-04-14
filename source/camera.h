#ifndef CAMERA_H_
#define CAMERA_H_

#include "Pixy/Pixy2SPI_SS.h"

#define CAM_HEIGHT 39.5         // In cm, lens tot grond
#define LINE_DIST 80.0          // In cm, wiel tot lijn
#define LENS_WHEELS_DIST 9.0    // In cm, lens tot wiel, horizontaal
#define THRESHOLD 7
#define CAM_ANGLE atan2(LINE_DIST + LENS_WHEELS_DIST, CAM_HEIGHT)

#define FOV_X TO_RADIANS(68)
#define FOV_Y TO_RADIANS(45)

class Camera {
private:
    int findEdgeHor(int y, int start, int stop);
    int findEdgeVer(int x, int start, int stop);

    Pixy2SPI_SS pixy;
public:
    int res_x, res_y;
    

    point getMid(point prev, int y);
    point getMid(point prev, int y, int &firstEdge, int &secEdge);
    float getDepth(int x);
    point pixel_to_point(int x, int y);
    point point_to_pixel(float x, float y);
    int getGrayScale(int x, int y);
    void setLamp(bool on);

    Camera() {
        pixy.init();
        pixy.setLED(0, 255, 0);

        res_x = pixy.frameWidth;
        res_y = pixy.frameHeight;
    };
};

#endif
