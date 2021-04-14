#include "camera.h"

point Camera::pixel_to_point(int x, int y) {
    double angleX = FOV_X * (x / (double)res_x) - (FOV_X / 2.0);
    double angleY = CAM_ANGLE + (FOV_Y * ((res_y - y) / (double)res_y)) - (FOV_Y / 2.0);

    double pointY = CAM_HEIGHT * tan(angleY);
    
    double beamLength = sqrt(CAM_HEIGHT * CAM_HEIGHT + pointY * pointY);
    
    double pointX = beamLength * tan(angleX);

    return {pointX, pointY};
}

point Camera::point_to_pixel(float x, float y) {
	float angleY = atan2(y, CAM_HEIGHT);
	float beamLength = sqrt(CAM_HEIGHT * CAM_HEIGHT + y * y);

	int pixY = res_y - round(((angleY - CAM_ANGLE + FOV_Y / 2.0) / (float)FOV_Y) * res_y);

	float angleX = atan2(x, beamLength);
	
	int pixX = round(((angleX + FOV_X / 2.0) / (float)FOV_X) * res_x);

	return {pixX, pixY};
}

uint8_t Camera::getGrayScale(int x, int y){
	uint8_t grayscale;

	if(x < 0) x = 0;
	else if(y < 0) y = 0;
	else if(x > res_x - 1) x = res_x -1;
	else if (y > res_y - 1) y = res_y - 1;

	grayscale = getGrayScale(x, y);

	return grayscale;
}

void Camera::setLamp(bool on){
	if(on)pixy.setLamp(1, 1);
	else pixy.setLamp(0,0);
}

point Camera::pixel_to_point(point p) {
	return pixel_to_point(p.x, p.y);
}

point Camera::point_to_pixel(point p) {
	return point_to_pixel(p.x, p.y);
}

int Camera::findEdgeHor(int y, int start, int stop) {
    int diff = 0, i = start;
    int sign = (start < stop) ? 1 : -1;
	uint8_t c1, c2;

	c1 = getGrayScale(i, y);

    while (i * sign < stop) {
		c2 = getGrayScale(i + sign, y);

        diff = c1 - c2;
        if (diff >= THRESHOLD) {
            return i;
        }

		c1 = c2;
        i += sign;
    }

    return -1;
}

int Camera::findEdgeVer(int x, int start, int stop) {
	int diff = 0, i = start;
	uint8_t c1, c2;
    int sign = (start < stop) ? 1 : -1;

	c1 = getGrayScale(res_x / 2 + x, i);

    while (i * sign < stop) {
		c2 = getGrayScale(res_x / 2 + x, i + sign);

        diff = c1 - c2;
        if (diff >= THRESHOLD) {
            return i;
        }
		
		c1 = c2;
        i += sign;
    }

    return -1;
}

point Camera::getMid(point prev, int y, int &firstEdge, int &secEdge) {
	point p;
	float mid;

	mid = point_to_pixel(prev.x, prev.y).x;

	if (mid < 10) mid = 10;
	if (mid > res_x - 10) mid = res_x - 10;

	firstEdge = findEdgeHor(y, mid, 0);
	secEdge = findEdgeHor(y, mid, res_x - 1);

	// printf("%d, %d\n", firstEdge, secEdge);

	if (firstEdge == -1 && secEdge != -1) {
		p = pixel_to_point(secEdge, y);
		p.x -= 25;
	}
	else if (secEdge == -1 && firstEdge != -1) {
		p = pixel_to_point(firstEdge, y);
		p.x += 25;
	}
	else if (firstEdge == -1 && secEdge == -1) {
		p = pixel_to_point(res_x / 2, y);
	}
	else {
		p = pixel_to_point((firstEdge + secEdge) / 2.0, y);
	}

    return p;
}

point Camera::getMid(point prev, int y) {
	int firstEdge, secEdge;

    return getMid(prev, y, firstEdge, secEdge);
}

float Camera::getDepth(int offset) {
    int y = findEdgeVer(offset, res_y, 0);

	return pixel_to_point(res_x / 2 + offset, y).y;
}

void Camera::getRow(int y, uint8_t camData[]) {
	uint8_t c;

	for (int x = 0; x < res_x; x++) {
		c = getGrayScale(x, y);
		camData[x] = c;
	}
}

void Camera::getCol(int x, uint8_t camData[]) {
	uint8_t c;

	for (int y = 0; y < res_y; y++) {
		c = getGrayScale(x, y);
		camData[y] = c;
	}
}