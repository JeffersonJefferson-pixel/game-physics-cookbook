#ifndef _H_GEOMETRY_3D_
#define _H_GEOMETRY_3D_

#include "vectors.h"
#include "matrices.h"

typedef vec3 Point;

// line segment

typedef struct Line
{
    Point start;
    Point end;

    inline Line() {}
    inline Line(const Point &s, const Point &e) : start(s), end(e) {}
} Line;

float Length(const Line &line);
float LengthSq(const Line &line);

// ray

typedef struct Ray
{
    Point origin;
    vec3 direction;

    inline Ray() : direction(0.0f, 0.0f, 1.0f) {}
    inline Ray(const Point &o, const vec3 &d) : origin(o), direction(d)
    {
        NormalizeDirection();
    }
    inline void NormalizeDirection()
    {
        Normalize(direction);
    }
} Ray;

Ray FromPoints(const Point &from, const Point &to);

// sphere

typedef struct Sphere
{
    Point position;
    float radius;

    inline Sphere() : radius(1.0f) {}
    inline Sphere(const Point &p, float r) : position(p), radius(r) {}
} Sphere;

// aabb

typedef struct AABB
{
    Point position;
    vec3 size;

    inline AABB() : size(1, 1, 1) {}
    inline AABB(const Point &p, const vec3 &s) : position(p), size(s) {}
} AABB;

vec3 GetMin(const AABB &aabb);
vec3 GetMax(const AABB &aabb);
AABB FromMinMax(const vec3 &min, const vec3 &max);

// oriented bounding box

typedef struct OBB
{
    Point position;
    vec3 size;
    mat3 orientation;

    inline OBB() : size(1, 1, 1) {}
    inline OBB(const Point &p, const vec3 &s, const mat3 &o) : position(p), size(s), orientation(o) {}
} OBB;

typedef struct Plane
{
    vec3 normal;
    // distance from origin.
    float distance;

    inline Plane() : normal(1, 0, 0) {}
    inline Plane(const vec3 &n, float d) : normal(n), distance(d) {}
} Plane;

float PlaneEquation(const Point &py, const Plane &plane);

typedef struct Triangle
{
    union
    {
        // named points
        struct
        {
            Point a;
            Point b;
            Point c;
        };
        // array of points
        Point points[3];
        // array of floats
        float values[9];
    };

    inline Triangle() {}
    inline Triangle(const Point &p1, const Point &p2, const Point &p3) : a(p1), b(p2), c(p3) {}
} Triangle;

// 3d point tests

// point and sphere

bool PointInSphere(const Point &point, const Sphere &sphere);
Point ClosestPoint(const Sphere &sphere, const Point &point);

// point and AABB

bool PointInAABB(const Point &point, const AABB &aabb);
Point ClosestPoint(const AABB &aabb, const Point &point);

// point and OBB
bool PointInOBB(const Point &point, const OBB &obb);
Point ClosestPoint(const OBB &obb, const Point &point);

// point and plane
bool PointOnPlane(const Point &point, const Plane &plane);
Point ClosestPoint(const Plane &plane, const Point &point);

// point and line
bool PointOnLine(const Point &point, const Line &line);
Point ClosestPoint(const Line &line, const Point &point);

// point and ray
bool PointOnRay(const Point &point, const Ray &ray);
Point ClosestPoint(const Ray &ray, const Point &point);

// 3d shape intersection

// sphere and sphere
bool SphereSphere(const Sphere &s1, const Sphere &s2);

// sphere and aabb
bool SphereAABB(const Sphere &sphere, const AABB &aabb);

// sphere and obb
bool SphereOBB(const Sphere &sphere, const OBB &obb);

// sphere and plane
bool SpherePlane(const Sphere &sphere, const Plane &plane);

// aabb and aabb
bool AABBAABB(const AABB &aabb1, const AABB &aabb2);

typedef struct Interval
{
    float min;
    float max;
} Interval;

Interval GetInterval(const AABB &rect, const vec3 &axis);
Interval GetInterval(const OBB &rect, const vec3 &axis);
bool OverlapOnAxis(const AABB &aabb, const OBB &obb, const vec3 &axis);
bool AABBOBB(const AABB &aabb, const OBB &obb);

// aabb and plane
bool AABBPlane(const AABB &aabb, const Plane &plane);

// obb and obb

bool OverlapOnAxis(const OBB &obb1, const OBB &obb2, const vec3 &axis);
bool OBBOBB(const OBB &obb1, const OBB &obb2);

// obb and plane
bool OBBPlane(const OBB &obb, const Plane &plane);

// plane and plane
bool PlanePlane(const Plane &plane1, const Plane &plane2);

// 3D line intersection

// ray and sphere
float Raycast(const Sphere &sphere, const Ray &ray);

// ray and aabb
float Raycast(const AABB &aabb, const Ray &ray);

// ray and oobb
float Raycast(const OBB &obb, const Ray &ray);

// ray and plane
float Raycast(const Plane &plane, const Ray &ray);

// line and sphere
bool Linetest(const Sphere &sphere, const Line &line);

// line and aabb
bool Linetest(const AABB &aabb, const Line &line);

// line and obb
bool Linetest(const OBB &obb, const Line &line);

// line and plane
bool Linetest(const Plane &plane, const Line &line);

// triangle and meshes

bool PointInTriangle(const Point &p, const Triangle &t);

Plane FromTriangle(const Triangle &t);

Point ClosestPoint(const Triangle &t, const Point &p);

// test if a triangle and sphere intersect.
bool TriangleSphere(const Triangle &t, const Sphere &s);

// triangle and aabb

Interval GetInterval(const Triangle &triangle, vec3 &axis);
bool OverlapOnAxis(const AABB &aabb, const Triangle &triangle, const vec3 &axis);
// test if a triangle and aabb intersect
bool TriangleAABB(const Triangle &t, const AABB &a);

// triangle and obb

bool OverlapOnAxis(const OBB &obb, const Triangle &triangle, const vec3 &axis);
bool TriangleOBB(const Triangle &t, const OBB &o);
#endif