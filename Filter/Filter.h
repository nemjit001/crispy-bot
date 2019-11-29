#pragma once

#include <cmath>
#include "mbed.h"

#define _USE_MATH_DEFINES
#define KERNEL_SIZE 3

class Filter {
private:
    int kernelSize = KERNEL_SIZE;
    double kernel[KERNEL_SIZE];

    void calculateKernel(double);
    void applyKernel(uint16_t*, int);
public:
    void applyGaussianBlur(uint16_t*, int);
};