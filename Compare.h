#ifndef _COMPARE_H
#define _COMPARE_H

#include <cmath>
#include <cfloat>


#define CMP(x, y) \
    (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

#endif