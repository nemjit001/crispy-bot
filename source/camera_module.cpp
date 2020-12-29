#include "camera_module.h"

int cameraModule::getVectors(Vector **vectors)
{
    this->pixy.line.getAllFeatures(LINE_VECTOR, 1);

    *vectors = this->pixy.line.vectors;

    return this->pixy.line.numVectors;
}
