#include "cubetransformations.h"

void translateCube(int cube, float tx, float ty, float tz) {
    if (cube == LEFT_CUBE) {
        for (int i = 0; i < 8; ++i) {
            translate(leftCubeVertices[i], tx, ty, tz);
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            translate(rightCubeVertices[i], tx, ty, tz);
        }
    }
}

void scaleCube(int cube, float sx, float sy, float sz) {
    if (cube == LEFT_CUBE) {
        for (int i = 0; i < 8; ++i) {
            scale(leftCubeVertices[i], sx, sy, sz);
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            scale(rightCubeVertices[i], sx, sy, sz);
        }
    }
}

void rotateXCube(int cube, int deg) {
    if (cube == LEFT_CUBE) {
        for (int i = 0; i < 8; ++i) {
            rotateX(leftCubeVertices[i], deg);
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            rotateX(rightCubeVertices[i], deg);
        }
    }
}

void rotateYCube(int cube, int deg) {
    if (cube == LEFT_CUBE) {
        for (int i = 0; i < 8; ++i) {
            rotateY(leftCubeVertices[i], deg);
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            rotateY(rightCubeVertices[i], deg);
        }
    }
}

void rotateZCube(int cube, int deg) {
    if (cube == LEFT_CUBE) {
        for (int i = 0; i < 8; ++i) {
            rotateZ(leftCubeVertices[i], deg);
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            rotateZ(rightCubeVertices[i], deg);
        }
    }
}

void subtractCubeVertices(int cube, int idx1, int idx2, float result[4]) {
    if (cube == LEFT_CUBE) {
        float weight1 = idx1 < 0 ? origin[3] : leftCubeVertices[idx1][3];
        float weight2 = idx2 < 0 ? origin[3] : leftCubeVertices[idx2][3];
        for (int i = 0; i < 3; ++i) {
            result[i] = weight2 * leftCubeVertices[idx1][i] - weight1 * leftCubeVertices[idx2][i];
        }
        result[3] = weight1 * weight2;
    } else {
        float weight1 = idx1 < 0 ? origin[3] : rightCubeVertices[idx1][3];
        float weight2 = idx2 < 0 ? origin[3] : rightCubeVertices[idx2][3];
        for (int i = 0; i < 3; ++i) {
            result[i] = weight2 * rightCubeVertices[idx1][i] - weight1 * rightCubeVertices[idx2][i];
        }
        result[3] = weight1 * weight2;
    }
}

void calculateCubeEdges() {
    subtractCubeVertices(LEFT_CUBE,          -1,                    -1,             cubeEdges[0]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_RIGHT_UP_IDX,            -1,             cubeEdges[1]);
    subtractCubeVertices(LEFT_CUBE,  BACK_RIGHT_UP_IDX,     FRONT_RIGHT_UP_IDX,     cubeEdges[2]);
    subtractCubeVertices(LEFT_CUBE,  BACK_RIGHT_DOWN_IDX,   BACK_RIGHT_UP_IDX,      cubeEdges[3]);
    subtractCubeVertices(LEFT_CUBE,  BACK_LEFT_DOWN_IDX,    BACK_RIGHT_DOWN_IDX,    cubeEdges[4]);
    subtractCubeVertices(LEFT_CUBE,  BACK_LEFT_UP_IDX,      BACK_LEFT_DOWN_IDX,     cubeEdges[5]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_LEFT_UP_IDX,     BACK_LEFT_UP_IDX,       cubeEdges[6]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_LEFT_DOWN_IDX,   FRONT_LEFT_UP_IDX,      cubeEdges[7]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_RIGHT_DOWN_IDX,  FRONT_LEFT_DOWN_IDX,    cubeEdges[8]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_RIGHT_UP_IDX,    FRONT_RIGHT_DOWN_IDX,   cubeEdges[9]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_LEFT_UP_IDX,     FRONT_RIGHT_UP_IDX,     cubeEdges[10]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_LEFT_DOWN_IDX,   FRONT_LEFT_UP_IDX,      cubeEdges[11]);
    subtractCubeVertices(LEFT_CUBE,  BACK_LEFT_DOWN_IDX,    FRONT_LEFT_DOWN_IDX,    cubeEdges[12]);
    subtractCubeVertices(LEFT_CUBE,  BACK_LEFT_UP_IDX,      BACK_LEFT_DOWN_IDX,     cubeEdges[13]);
    subtractCubeVertices(LEFT_CUBE,  BACK_RIGHT_UP_IDX,     BACK_LEFT_UP_IDX,       cubeEdges[14]);
    subtractCubeVertices(LEFT_CUBE,  BACK_RIGHT_DOWN_IDX,   BACK_RIGHT_UP_IDX,      cubeEdges[15]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_RIGHT_DOWN_IDX,  BACK_RIGHT_DOWN_IDX,    cubeEdges[16]);
    subtractCubeVertices(LEFT_CUBE,  FRONT_RIGHT_UP_IDX,    FRONT_RIGHT_DOWN_IDX,   cubeEdges[17]);
    subtractCubeVertices(LEFT_CUBE,          -1,            FRONT_RIGHT_UP_IDX,     cubeEdges[18]);

    subtractCubeVertices(RIGHT_CUBE,         -1,                    -1,             cubeEdges[19]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_RIGHT_UP_IDX,            -1,             cubeEdges[20]);
    subtractCubeVertices(RIGHT_CUBE, BACK_RIGHT_UP_IDX,     FRONT_RIGHT_UP_IDX,     cubeEdges[21]);
    subtractCubeVertices(RIGHT_CUBE, BACK_RIGHT_DOWN_IDX,   BACK_RIGHT_UP_IDX,      cubeEdges[22]);
    subtractCubeVertices(RIGHT_CUBE, BACK_LEFT_DOWN_IDX,    BACK_RIGHT_DOWN_IDX,    cubeEdges[23]);
    subtractCubeVertices(RIGHT_CUBE, BACK_LEFT_UP_IDX,      BACK_LEFT_DOWN_IDX,     cubeEdges[24]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_LEFT_UP_IDX,     BACK_LEFT_UP_IDX,       cubeEdges[25]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_LEFT_DOWN_IDX,   FRONT_LEFT_UP_IDX,      cubeEdges[26]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_RIGHT_DOWN_IDX,  FRONT_LEFT_DOWN_IDX,    cubeEdges[27]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_RIGHT_UP_IDX,    FRONT_RIGHT_DOWN_IDX,   cubeEdges[28]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_LEFT_UP_IDX,     FRONT_RIGHT_UP_IDX,     cubeEdges[29]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_LEFT_DOWN_IDX,   FRONT_LEFT_UP_IDX,      cubeEdges[30]);
    subtractCubeVertices(RIGHT_CUBE, BACK_LEFT_DOWN_IDX,    FRONT_LEFT_DOWN_IDX,    cubeEdges[31]);
    subtractCubeVertices(RIGHT_CUBE, BACK_LEFT_UP_IDX,      BACK_LEFT_DOWN_IDX,     cubeEdges[32]);
    subtractCubeVertices(RIGHT_CUBE, BACK_RIGHT_UP_IDX,     BACK_LEFT_UP_IDX,       cubeEdges[33]);
    subtractCubeVertices(RIGHT_CUBE, BACK_RIGHT_DOWN_IDX,   BACK_RIGHT_UP_IDX,      cubeEdges[34]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_RIGHT_DOWN_IDX,  BACK_RIGHT_DOWN_IDX,    cubeEdges[35]);
    subtractCubeVertices(RIGHT_CUBE, FRONT_RIGHT_UP_IDX,    FRONT_RIGHT_DOWN_IDX,   cubeEdges[36]);
    subtractCubeVertices(RIGHT_CUBE,         -1,            FRONT_RIGHT_UP_IDX,     cubeEdges[37]);
}

void calculateCubeVectorDataFloats() {
    calculateCubeEdges();
    for (int i = 0; i < 38; ++i) {
        projectOrthogonally(cubeEdges[i], cubeVectorDataFloats[i]);
    }
}

#define NEG (1<<10)
void calculateCubeVectorData(uint16_t vectorData[2][38]) {
    calculateCubeVectorDataFloats();
    for (int i = 0; i < 38; ++i) {
        for (int j = 0; j < 2; ++j) {
            float val = 100 * cubeVectorDataFloats[i][j];
            if (val >= 0) {
                vectorData[j][i] = (uint16_t)(val);
            } else {
                vectorData[j][i] = NEG|((uint16_t)(-val));
            }
        }
    }
}