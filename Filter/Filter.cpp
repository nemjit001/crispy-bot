#include "Filter.h"

void Filter::applyGaussianBlur(uint16_t* array, int arraySize) {
    double sigma = 1;
    double kernel[KERNEL_SIZE];
    calculateKernel(kernel, sigma);

    applyKernel(array, arraySize, kernel);
}

void Filter::calculateKernel(double* kernel, double sigma) {
    double  r = 0,
            sum = 0,
            s = 2.0 * pow(sigma, 2);
    int halfSize = kernelSize / 2;

    for (int i = 0; i < kernelSize; ++i) {
        r = pow(i - halfSize, 2);
        kernel[i] = (exp(-(r / s)) / (M_PI * s));
        sum += kernel[i];
    }

    for (int i = 0; i < kernelSize; ++i) {
        kernel[i] /= sum;
    }
}

void Filter::applyKernel(uint16_t* array, int arraySize, double* kernel) {
    int width = kernelSize / 2, index;
    uint16_t pixelValue;

    for (int i = 0; i < arraySize; i++) {
        pixelValue = 0;

        for (int j = 0; j <= kernelSize; j++) {
            index = i + j + width;

            if (index < 0) index = 0;
            else if (index > 127) index = 127;

            pixelValue += array[index] * kernel[j];
        }

        array[i] = pixelValue;
    }
}