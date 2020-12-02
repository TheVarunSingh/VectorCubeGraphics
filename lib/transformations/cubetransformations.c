#include "cubetransformations.h"

/**
 * Transformations to the cube are performed by transforming each vertex of the cube.
 */

void translateCube(int cube, float tx, float ty, float tz) {
    // Translate a cube by tx in the x direction, ty in the y direction, and tz in the z direction
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
    // Scales a cube by tx in the x direction, ty in the y direction, and tz in the z direction
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
    // Rotates a cube about the x axis
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
    // Rotates a cube about the y axis
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
    // Rotates the left cube about the x=0,y=-2 axis and the right cube about the x=0,y=2 axis
    translateCube(cube, 0, 2-(4*cube), 0);
    if (cube == LEFT_CUBE) {
        for (int i = 0; i < 8; ++i) {
            rotateZ(leftCubeVertices[i], deg);
        }
    } else {
        for (int i = 0; i < 8; ++i) {
            rotateZ(rightCubeVertices[i], deg);
        }
    }
    translateCube(cube, 0, (4*cube)-2, 0);
}

void subtractCubeVertices(int cube, int idx1, int idx2, float result[4]) {
    // Homogenous coordinate subtraction in order to get the vector between two poitns
    if (cube == LEFT_CUBE) {
        float weight1 = idx1 < 0 ? origin[3] : leftCubeVertices[idx1][3];
        float weight2 = idx2 < 0 ? origin[3] : leftCubeVertices[idx2][3];
        for (int i = 0; i < 3; ++i) {
            float a = idx1 < 0 ? origin[i] : leftCubeVertices[idx1][i];
            float b = idx2 < 0 ? origin[i] : leftCubeVertices[idx2][i];
            result[i] = weight2 * a - weight1 * b;
        }
        result[3] = weight1 * weight2;
    } else {
        float weight1 = idx1 < 0 ? origin[3] : rightCubeVertices[idx1][3];
        float weight2 = idx2 < 0 ? origin[3] : rightCubeVertices[idx2][3];
        for (int i = 0; i < 3; ++i) {
            float a = idx1 < 0 ? origin[i] : rightCubeVertices[idx1][i];
            float b = idx2 < 0 ? origin[i] : rightCubeVertices[idx2][i];
            result[i] = weight2 * a - weight1 * b;
        }
        result[3] = weight1 * weight2;
    }
}

void calculateCubeEdges() {
    // Calculate the edges of the cube by taking the difference between consecutive vertices
    // Start and end at the origin

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
    // Calculate the edge values while still using floats
    calculateCubeEdges();
    for (int i = 0; i < 38; ++i) {
        projectOrthogonally(cubeEdges[i], cubeVectorDataFloats[i]);
    }
}

#define NEG (1<<10)
void calculateCubeVectorData(uint16_t vectorData[2][38]) {
    // Calculate the screen coordinate values (-512 to 512) for the edgees
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