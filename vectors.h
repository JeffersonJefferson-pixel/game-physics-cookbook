#ifndef _H_MATH_VECTORS_
#define _H_MATH_VECTORS_

typedef struct vec2
{
  union
  {
    struct
    {
      float x;
      float y;
    };
    float asArray[2];
  };

  inline float &operator[](int i)
  {
    return asArray[i];
  }

  inline vec2() : x(0.0f), y(0.0f) {}
  inline vec2(float _x, float _y) : x(_x), y(_y) {}
} vec2;

typedef struct vec3
{
  union
  {
    struct
    {
      float x;
      float y;
      float z;
    };
    float asArray[3];
  };

  float &operator[](int i)
  {
    return asArray[i];
  }

  inline vec3() : x(0.0f), y(0.0f), z(0.0f) {}
  inline vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
} vec3;

vec2 operator+(const vec2& l, const vec2& r);
vec3 operator+(const vec3& l, const vec3& r);
vec2 operator-(const vec2& l, const vec2& r);
vec3 operator+(const vec3& l, const vec3& r);
vec2 operator*(const vec2& l, const vec2& r);
vec3 operator*(const vec3& l, const vec3& r);
vec2 operator*(const vec2& l, float r);
vec3 operator*(const vec3& l, float r);
bool operator==(const vec2& l, const vec2& r);
bool operator==(const vec2& l, const vec2& r);
bool operator!=(const vec2& l, const vec2& r);
bool operator!=(const vec3& l, const vec3& r);

#endif