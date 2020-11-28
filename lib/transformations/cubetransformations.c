#include "cubetransformations.h"

void translateCube(float tx, float ty, float tz) {
    for (int i = 0; i < 8; ++i) {
        translate(cubeVertices[i], tx, ty, tz);
    }
}

void scaleCube(float sx, float sy, float sz) {
    for (int i = 0; i < 8; ++i) {
        scale(cubeVertices[i], sx, sy, sz);
    }
}

void rotateXCube(int deg) {
    for (int i = 0; i < 8; ++i) {
        rotateX(cubeVertices[i], deg);
    }
}

void rotateYCube(int deg) {
    for (int i = 0; i < 8; ++i) {
        rotateY(cubeVertices[i], deg);
    }
}

void rotateZCube(int deg) {
    for (int i = 0; i < 8; ++i) {
        rotateZ(cubeVertices[i], deg);
    }
}

void subtractCubeVertices(int idx1, int idx2, float result[4]) {
    for (int i = 0; i < 4; ++i) {
        float a = idx1 < 0 ? origin[i] : cubeVertices[idx1][i];
        float b = idx1 < 0 ? origin[i] : cubeVertices[idx2][i];
        result[i] = a - b;
    }
}

void calculateCubeEdges() {
    subtractCubeVertices(       -1,                     -1,             cubeEdges[0]);
    subtractCubeVertices(FRONT_RIGHT_UP_IDX,            -1,             cubeEdges[1]);
    subtractCubeVertices(BACK_RIGHT_UP_IDX,     FRONT_RIGHT_UP_IDX,     cubeEdges[2]);
    subtractCubeVertices(BACK_RIGHT_DOWN_IDX,   BACK_RIGHT_UP_IDX,      cubeEdges[3]);
    subtractCubeVertices(BACK_LEFT_DOWN_IDX,    BACK_RIGHT_DOWN_IDX,    cubeEdges[4]);
    subtractCubeVertices(BACK_LEFT_UP_IDX,      BACK_LEFT_DOWN_IDX,     cubeEdges[5]);
    subtractCubeVertices(FRONT_LEFT_UP_IDX,     BACK_LEFT_UP_IDX,       cubeEdges[6]);
    subtractCubeVertices(FRONT_LEFT_DOWN_IDX,   FRONT_LEFT_UP_IDX,      cubeEdges[7]);
    subtractCubeVertices(FRONT_RIGHT_DOWN_IDX,  FRONT_LEFT_DOWN_IDX,    cubeEdges[8]);
    subtractCubeVertices(FRONT_RIGHT_UP_IDX,    FRONT_RIGHT_DOWN_IDX,   cubeEdges[9]);
    subtractCubeVertices(FRONT_LEFT_UP_IDX,     FRONT_RIGHT_UP_IDX,     cubeEdges[10]);
    subtractCubeVertices(FRONT_LEFT_DOWN_IDX,   FRONT_LEFT_UP_IDX,      cubeEdges[11]);
    subtractCubeVertices(BACK_LEFT_DOWN_IDX,    FRONT_LEFT_DOWN_IDX,    cubeEdges[12]);
    subtractCubeVertices(BACK_LEFT_UP_IDX,      BACK_LEFT_DOWN_IDX,     cubeEdges[13]);
    subtractCubeVertices(BACK_RIGHT_UP_IDX,     BACK_LEFT_UP_IDX,       cubeEdges[14]);
    subtractCubeVertices(BACK_RIGHT_DOWN_IDX,   BACK_RIGHT_UP_IDX,      cubeEdges[15]);
    subtractCubeVertices(FRONT_RIGHT_DOWN_IDX,  BACK_RIGHT_DOWN_IDX,    cubeEdges[16]);
    subtractCubeVertices(FRONT_RIGHT_UP_IDX,    FRONT_RIGHT_DOWN_IDX,   cubeEdges[17]);
    subtractCubeVertices(        -1,            FRONT_RIGHT_UP_IDX,     cubeEdges[18]);
}

void calculateCubeVectorData(float vectorData[19][2]) {
    calculateCubeEdges();
    for (int i = 0; i < 19; ++i) {
        projectOrthogonally(cubeEdges[i], vectorData[i]);
    }
}