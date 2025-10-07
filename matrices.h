#ifndef _H_MATH_MATRICES
#define _H_MATH_MATRICES

#include "vectors.h"

typedef struct mat2 {
    union {
        struct {
            float _11, _12,
                    _21, _22;
        };
        float asArray[4];
    };

    inline float* operator[] (int i) {
        return &(asArray[i * 2]);
    }
} mat2;

typedef struct mat3 {
    union {
        struct {
            float _11, _12, _13,
                    _21, _22, _23,
                    _31, _32, _33;
        };
        float asArray[9];
    };

    inline float* operator[] (int i) {
        return &(asArray[i * 3]);
    }
} mat3;


typedef struct mat4 {
    union {
        struct {
            float _11, _12, _13, _14,
                    _21, _22, _23, _24,
                    _31, _32, _33, _34,
                    _41, _42, _43, _44;
        };
        float asArray[16];
    };

    inline float* operator[] (int i) {
        return &(asArray[i * 4]);
    };
} mat4;

void Transpose(const float *srcMat, float *dstMat, int srcRows, int srcCols);
mat2 Transpose(const mat2& matrix);
mat3 Transpose(const mat3& matrix);
mat4 Transpose(const mat4& matrix);

mat2 operator*(const mat2& matrix, float scalar);
mat3 operator*(const mat3& matrix, float scalar);
mat4 operator*(const mat4& matrix, float scalar);

bool Multiply(float *out, const float *matrixA, int aRows, int aCols, float* matB, int bRows, int bCols);
mat2 operator*(const mat2& matA, const mat2& matB);
mat3 operator*(const mat3& matA, const mat3& matB);
mat4 operator*(const mat4& matA, const mat4& matB);

#endif 