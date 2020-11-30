#ifndef __CUBETRANSFORMATIONS_H__
#define __CUBETRANSFORMATIONS_H__

#include "transformations.h"
#include <stdint.h>

#define LEFT_CUBE   0
#define RIGHT_CUBE  1

static const int FRONT_RIGHT_UP_IDX    = 0;
static const int FRONT_RIGHT_DOWN_IDX  = 1;
static const int FRONT_LEFT_DOWN_IDX   = 2;
static const int FRONT_LEFT_UP_IDX     = 3;
static const int BACK_LEFT_UP_IDX      = 4;
static const int BACK_LEFT_DOWN_IDX    = 5;
static const int BACK_RIGHT_DOWN_IDX   = 6;
static const int BACK_RIGHT_UP_IDX     = 7;

static const float origin[4] = {0, 0, 0, 1};

static float leftCubeVertices[8][4] = {
    { 1, -1,  1,  1},
    { 1, -1, -1,  1},
    { 1, -3, -1,  1},
    { 1, -3,  1,  1},
    {-1, -3,  1,  1},
    {-1, -3, -1,  1},
    {-1, -1, -1,  1},
    {-1, -1,  1,  1}
};

static float rightCubeVertices[8][4] = {
    { 1,  3,  1,  1},
    { 1,  3, -1,  1},
    { 1,  1, -1,  1},
    { 1,  1,  1,  1},
    {-1,  1,  1,  1},
    {-1,  1, -1,  1},
    {-1,  3, -1,  1},
    {-1,  3,  1,  1}
};

static float cubeEdges[38][4];

static float cubeVectorDataFloats[38][2];

void translateCube(int cube, float tx, float ty, float tz);
void scaleCube(int cube, float sx, float sy, float sz);
void rotateXCube(int cube, int deg);
void rotateYCube(int cube, int deg);
void rotateZCube(int cube, int deg);
void calculateCubeVectorData(uint16_t vectorData[2][38]);

#endif