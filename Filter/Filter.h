#pragma once

#include <cmath>
#include "mbed.h"

// #define _USE_MATH_DEFINES
#define KERNEL_SIZE 3

class Filter {
private:
    int kernelSize = KERNEL_SIZE;

    void calculateKernel(double*, double);
    void applyKernel(uint16_t*, int, double*);
public:
    void applyGaussianBlur(uint16_t*, int);
};