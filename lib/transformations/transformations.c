#include "transformations.h"

void multiply4x4byVector(float A[4][4], float b[4]) {
    float result[4];

    for (int i = 0; i < 4; ++i) {
        float dotProduct = 0.0f;
        for (int k = 0; k < 4; ++k) {
            dotProduct += A[i][k] * b[k];
        }
        result[i] = dotProduct;
    }

    for (int i = 0; i < 4; ++i) {
        b[i] = result[i];
    }
}

void translate(float vector[4], float tx, float ty, float tz) {
    float translationMatrix[4][4] = {
        { 1,  0,  0,  0},
        { 0,  1,  0,  0},
        { 0,  0,  1,  0},
        {tx, ty, tz,  1}
    };
    multiply4x4byVector(translationMatrix, vector);
}

void scale(float vector[4], float sx, float sy, float sz) {
    float scalingMatrix[4][4] = {
        {sx,  0,  0,  0},
        { 0, sy,  0,  0},
        { 0,  0, sz,  0},
        { 0,  0,  0,  1}
    };
    multiply4x4byVector(scalingMatrix, vector);
}

void rotatePositiveX(float vector[4]) {
    float rotationMatrix[4][4] = {
        {1,    0   ,     0   ,  0},
        {0, cos1deg, -sin1deg,  0},
        {0, sin1deg,  cos1deg,  0},
        {0,    0   ,     0   ,  1}
    };
    multiply4x4byVector(rotationMatrix, vector);
}

void rotateNegativeX(float vector[4]) {
    float rotationMatrix[4][4] = {
        {1,     0   ,    0   ,  0},
        {0,  cos1deg, sin1deg,  0},
        {0, -sin1deg, cos1deg,  0},
        {0,     0   ,    0   ,  1}
    };
    multiply4x4byVector(rotationMatrix, vector);
}

void rotateX(float vector[4], int deg) {
    if (deg == 0) return;
    else if (deg > 0) {
        for (int i = 0; i < deg; ++ i) {
            rotatePositiveX(vector);
        }
    } else {
        for (int i = 0; i < deg; ++ i) {
            rotateNegativeX(vector);
        }
    }
}

void rotatePositiveY(float vector[4]) {
    float rotationMatrix[4][4] = {
        { cos1deg, 0, sin1deg,  0},
        {    0   , 1,    0   ,  0},
        {-sin1deg, 0, cos1deg,  0},
        {    0   , 0,    0   ,  1}
    };
    multiply4x4byVector(rotationMatrix, vector);
}

void rotateNegativeY(float vector[4]) {
    float rotationMatrix[4][4] = {
        {cos1deg, 0, -sin1deg,  0},
        {   0   , 1,     0   ,  0},
        {sin1deg, 0,  cos1deg,  0},
        {   0   , 0,     0   ,  1}
    };
    multiply4x4byVector(rotationMatrix, vector);
}

void rotateY(float vector[4], int deg) {
    if (deg == 0) return;
    else if (deg > 0) {
        for (int i = 0; i < deg; ++ i) {
            rotatePositiveY(vector);
        }
    } else {
        for (int i = 0; i < deg; ++ i) {
            rotateNegativeY(vector);
        }
    }
}

void rotatePositiveZ(float vector[4]) {
    float rotationMatrix[4][4] = {
        {cos1deg, -sin1deg,  0,  0},
        {sin1deg,  cos1deg,  0,  0},
        {   0   ,     0   ,  1,  0},
        {   0   ,     0   ,  0,  1}
    };
    multiply4x4byVector(rotationMatrix, vector);
}

void rotateNegativeZ(float vector[4]) {
    float rotationMatrix[4][4] = {
        { cos1deg, sin1deg,  0,  0},
        {-sin1deg, cos1deg,  0,  0},
        {    0   ,    0   ,  1,  0},
        {    0   ,    0   ,  0,  1}
    };
    multiply4x4byVector(rotationMatrix, vector);
}

void rotateZ(float vector[4], int deg) {
    if (deg == 0) return;
    else if (deg > 0) {
        for (int i = 0; i < deg; ++ i) {
            rotatePositiveZ(vector);
        }
    } else {
        for (int i = 0; i < deg; ++ i) {
            rotateNegativeZ(vector);
        }
    }
}

void normalizeHomogenousCoordinates(float coordinates[4]) {
    for (int i = 0; i < 3; ++ i) {
        coordinates[i] /= coordinates[3];
    }
    coordinates[3] = 1.0f;
}

// https://en.wikipedia.org/wiki/Orthographic_projection
void projectOrthogonally(float homogenousCoordinates[4], float xyCoordinates[2]) {
    normalizeHomogenousCoordinates(homogenousCoordinates);
    xyCoordinates[0] = homogenousCoordinates[0];
    xyCoordinates[1] = homogenousCoordinates[1];
}