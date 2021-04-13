void rover::printLineDist() {
	getCamData(res_y / 2, camData1);

	char buf[32];

	sprintf(buf, "%d,", res_x);
    print_string(buf);

    for (int i = 0; i < res_x; i++) {
		sprintf(buf, "%d,", camData1[i]);
        print_string(buf);
    }

	print_string("\r\n");
}

void rover::printLineDist2() {
	getCamData(line1, camData1);

	char buf[32];

	sprintf(buf, "%d,", res_x);
    print_string(buf);

    for (int i = 0; i < res_x; i++) {
		sprintf(buf, "%d,", camData1[i]);
        print_string(buf);
    }

	int firstEdge, secEdge;
	point mid = getMid(midLower, line1, firstEdge, secEdge);
	int x = (int)point_to_pixel(mid.x, mid.y).x;

	sprintf(buf, "%d,%d,%d,\r\n", x, firstEdge, secEdge);
	print_string(buf);
}

void rover::printLineDist3() {
	getCamData(depth_p + 10, camData1);

	char buf[32];

	sprintf(buf, "%d,", res_x);
    print_string(buf);

    for (int i = 0; i < res_x; i++) {
		sprintf(buf, "%d,", camData1[i]);
        print_string(buf);
    }

	int firstEdge, secEdge;
	point mid = getMid(midUpper, depth_p + 10, firstEdge, secEdge);
	point p = point_to_pixel(mid.x, mid.y);
	int x = (int)p.x;
	
	p = pixel_to_point(res_x / 2, depth_p);
	int dist = (int)p.y;

	sprintf(buf, "%d,%d,%d,%d,\r\n", x, firstEdge, secEdge, dist);
	print_string(buf);
}

void rover::printLineDist4() {
	char buf[32];
	uint8_t c;

    for (int i = 0; i < res_y; i--) {
		pixy.video.getRGB(res_x / 2, i, &c, false);
		sprintf(buf, "%d,", c);
        print_string(buf);
    }
	
	point p = pixel_to_point(res_x / 2, depth_p);
	int dist = (int)p.y;

	sprintf(buf, "%d,%d,\r\n", depth_p, dist);
	print_string(buf);
}

void rover::printLineDist5() {
	int firstEdge, secEdge, mid;
	point p;

	getCamData(line1, camData1);
	getCamData(depth_p, camData2);

	char buf[32];

	sprintf(buf, "%d,", res_x);
    print_string(buf);

    for (int i = 0; i < res_x; i++) {
		sprintf(buf, "%d,", camData1[i]);
        print_string(buf);
    }

	p = getMid(midLower, line1, firstEdge, secEdge);
	mid = (int)point_to_pixel(p.x, p.y).x;
	sprintf(buf, "%d,%d,%d,", mid, firstEdge, secEdge);
	print_string(buf);

	for (int i = 0; i < res_x; i++) {
		sprintf(buf, "%d,", camData2[i]);
        print_string(buf);
    }

	p = getMid(midUpper, depth_p, firstEdge, secEdge);
	mid = (int)point_to_pixel(p.x, p.y).x;
	sprintf(buf, "%d,%d,%d,", mid, firstEdge, secEdge);
	print_string(buf);

	sprintf(buf, "%d,%d,\r\n", (int)dist, spee);
	print_string(buf);
}


void rover::printCamData() {
	// Print mid, firstEdge en secEdge van beide lijnen, en het verste punt (dist)
	// mid1, firstEdge1, secEdge1, dist1, mid2, firstEdge2, secEdg2, dist2, maxdistX, maxdistY
	int firstEdge, secEdge;
	point mid, p1, p2;
	char buf[128];

	// Lower line
	mid = getMid(midLower, line1, firstEdge, secEdge);
	p1 = pixel_to_point(firstEdge, line1);
	p2 = pixel_to_point(secEdge, line1);

	sprintf(buf, "%d,%d,%d,%d,", (int)mid.x, (int)p1.x, (int)p2.x, (int)mid.y);
	print_string(buf);

	// Dist
	int depth = getDepth(res_x / 2, line1);

	if (depth == -1) {
		print_string("-1,-1,-1,-1,-1,-1,\r\n");
		return;
	}

	p1 = pixel_to_point(res_x / 2, depth);
	dist = p1.y;
	dist -= 60;
	depth = point_to_pixel(res_x / 2, dist).y;

	// Upper line
	mid = getMid({0, 100}, depth, firstEdge, secEdge);
	p1 = pixel_to_point(firstEdge, depth);
	p2 = pixel_to_point(secEdge, depth);

	sprintf(buf, "%d,%d,%d,%d,", (int)mid.x, (int)p1.x, (int)p2.x, (int)mid.y);
	print_string(buf);

	p1 = pixel_to_point(res_x / 2, depth);

	sprintf(buf, "%d,%d,\r\n", (int)p1.x, (int)p1.y);
	print_string(buf);
}