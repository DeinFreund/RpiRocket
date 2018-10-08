#ifndef __MTVECTOR_H__
#define __MTVECTOR_H__
/**
 * Interface for basic vector calculations.
 * All operations are prefixed with 'mt' to avoid name clashes and get an
 * attempt for a unique prefix.
 *
 * @author Maurice Tollmien
 */

#include <stdarg.h>

typedef struct {
    float x;
    float y;
    float z;
} MTVec3D;

#define MT_PI 3.14159265

MTVec3D mtToVector3D(float x, float y, float z);
float mtVectorLength3D(MTVec3D vector);
MTVec3D mtNormVector3D(MTVec3D vector);
MTVec3D mtCrossProduct3D(MTVec3D a, MTVec3D b);
MTVec3D mtMultiplyVectorScalar (MTVec3D a, float s);
MTVec3D mtSubtractVectorVector (MTVec3D a, MTVec3D b);
MTVec3D mtDivideVectorScalar (MTVec3D a, float s);
MTVec3D mtAddVectorVector (MTVec3D a, MTVec3D b);
void  mtPrintVector (MTVec3D a);
float mtAngleVectorVector (MTVec3D a, MTVec3D b);
float mtRadToDeg (float rad);
float mtDegToRad (float deg);
float mtScalarProduct (MTVec3D a, MTVec3D b);

#endif
