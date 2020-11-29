#ifndef __CUBETRANSFORMATIONS_H__
#define __CUBETRANSFORMATIONS_H__

#include "transformations.h"
#include <stdint.h>

static const int FRONT_RIGHT_UP_IDX    = 0;
static const int FRONT_RIGHT_DOWN_IDX  = 1;
static const int FRONT_LEFT_DOWN_IDX   = 2;
static const int FRONT_LEFT_UP_IDX     = 3;
static const int BACK_LEFT_UP_IDX      = 4;
static const int BACK_LEFT_DOWN_IDX    = 5;
static const int BACK_RIGHT_DOWN_IDX   = 6;
static const int BACK_RIGHT_UP_IDX     = 7;

static const float origin[4] = {0, 0, 0, 1};

static const float cubeVertices[8][4] = {
    { 1,  1,  1,  1},
    { 1,  1, -1,  1},
    { 1, -1, -1,  1},
    { 1, -1,  1,  1},
    {-1, -1,  1,  1},
    {-1, -1, -1,  1},
    {-1,  1, -1,  1},
    {-1,  1,  1,  1}
};

volatile static float cubeEdges[19][4];

volatile static float cubeVectorDataFloats[19][2];

void translateCube(float tx, float ty, float tz);
void scaleCube(float sx, float sy, float sz);
void rotateXCube(int deg);
void rotateYCube(int deg);
void rotateZCube(int deg);
void calculateCubeVectorData(uint16_t vectorData[19][2]);

#endif