#ifndef __TRANSFORMATIONS_H__
#define __TRANSFORMATIONS_H__

static const float sin1deg = 0.0174524064372835128194189785163161924722527203071396426836124276f;
static const float cos1deg = 0.9998476951563912391570115588139148516927403105831859396583207145f;

void multiply4x4byVector(float A[4][4], float b[4]);
void translate(float vector[4], float tx, float ty, float tz);
void scale(float vector[4], float sx, float sy, float sz);
void rotateX(float vector[4], int deg);
void rotateY(float vector[4], int deg);
void rotateZ(float vector[4], int deg);
void normalizeHomogenousCoordinates(float coordinates[4]);
void projectOrthogonally(float homogenousCoordinates[4], float xyCoordinates[2]);

#endif