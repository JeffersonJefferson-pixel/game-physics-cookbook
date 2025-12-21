#include "Geometry3D.h"
#include <cmath>
#include <cfloat>
#include <list>

#define CMP(x, y) \
    (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

float Length(const Line &line)
{
    return Magnitude(line.start - line.end);
}

float LengthSq(const Line &line)
{
    return MagnitudeSq(line.start - line.end);
}

Ray FromPoints(const Point &from, const Point &to)
{
    return Ray(from, Normalized(to - from));
}

vec3 GetMin(const AABB &aabb)
{
    vec3 p1 = aabb.position + aabb.size;
    vec3 p2 = aabb.position - aabb.size;

    return vec3(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
}

vec3 GetMax(const AABB &aabb)
{
    vec3 p1 = aabb.position + aabb.size;
    vec3 p2 = aabb.position - aabb.size;

    return vec3(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
}

AABB FromMinMax(const vec3 &min, const vec3 &max)
{
    return AABB((min + max) * 0.5, (max - min) * 0.5f);
}

float PlaneEquation(const Point &point, const Plane &plane)
{
    return Dot(point, plane.normal) - plane.distance;
}

bool PointInSphere(const Point &point, const Sphere &sphere)
{
    // distance between point and sphere center
    float magSq = MagnitudeSq(point - sphere.position);
    float radSq = sphere.radius * sphere.radius;
    return magSq < radSq;
}

Point ClosestPoint(const Sphere &sphere, const Point &point)
{
    vec3 sphereToPoint = point - sphere.position;
    Normalize(sphereToPoint);
    sphereToPoint = sphereToPoint * sphere.radius;
    return sphere.position + sphereToPoint;
}

bool PointInAABB(const Point &point, const AABB &aabb)
{
    // compare with minimum and maximum point of aabb component-wise.
    Point min = GetMin(aabb);
    Point max = GetMax(aabb);
    if (point.x < min.x || point.y < min.y || point.z < min.z)
    {
        return false;
    }
    if (point.x > max.x || point.y > max.y || point.z > max.z)
    {
        return false;
    }
    return true;
}

Point ClosestPoint(const AABB &aabb, const Point &point)
{
    // clamp closest point to minimum and maximum point of aabb.
    Point result = point;
    Point min = GetMin(aabb);
    Point max = GetMax(aabb);

    result.x = (result.x < min.x) ? min.x : result.x;
    result.y = (result.y < min.y) ? min.y : result.y;
    result.z = (result.z < min.z) ? min.z : result.z;
    result.x = (result.x > max.x) ? max.x : result.x;
    result.y = (result.y > max.y) ? max.y : result.y;
    result.z = (result.z > max.z) ? max.z : result.z;

    return result;
}

bool PointInOBB(const Point &point, const OBB &obb)
{
    vec3 dir = point - obb.position;
    // project point to local axes of the box.
    for (int i = 0; i < 3; ++i)
    {
        const float *orientation = &obb.orientation.asArray[i * 3];
        vec3 axis(orientation[0], orientation[1], orientation[2]);
        float distance = Dot(dir, axis);
        if (distance > obb.size.asArray[i])
        {
            return false;
        }
        if (distance < -obb.size.asArray[i])
        {
            return false;
        }
    }

    return true;
}

Point ClosestPoint(const OBB &obb, const Point &point)
{
    Point result = obb.position;
    vec3 dir = point - obb.position;
    // project
    for (int i = 0; i < 3; ++i)
    {
        const float *orientation = &obb.orientation.asArray[i * 3];
        vec3 axis(orientation[0], orientation[1], orientation[2]);
        float distance = Dot(dir, axis);
        // clamp
        if (distance > obb.size.asArray[i])
        {
            distance = obb.size.asArray[i];
        }
        if (distance < -obb.size.asArray[i])
        {
            distance = -obb.size.asArray[i];
        }
        // move along axis by clamped distance
        result = result + (axis * distance);
    }

    return result;
}

bool PointOnPlane(const Point &point, const Plane &plane)
{
    return CMP(Dot(point, plane.normal) - plane.distance, 0.0f);
}

Point ClosestPoint(const Plane &plane, const Point &point)
{
    float dot = Dot(plane.normal, point);
    // distance from point to plane
    float distance = dot - plane.distance;
    // move point on plane.
    return point - plane.normal * distance;
}

Point ClosestPoint(const Line &line, const Point &point)
{
    // project point on to line
    vec3 lVec = line.end - line.start;
    float t = Dot(point - line.start, lVec) / Dot(lVec, lVec);
    // clamp
    t = fmaxf(t, 0.0f);
    t = fminf(t, 1.0f);
    // evaluate parametric function.
    return line.start + lVec * t;
}

bool PointOnLine(const Point &point, const Line &line)
{
    // find closest point on line and check if it is actually the point by comparing distance.
    Point closest = ClosestPoint(line, point);
    float distanceSq = MagnitudeSq(closest - point);
    return CMP(distanceSq, 0.0f);
}

bool PointOnRay(const Point &point, const Ray &ray)
{
    if (point == ray.origin)
    {
        return true;
    }
    vec3 norm = point - ray.origin;
    Normalize(norm);
    float diff = Dot(norm, ray.direction);
    return CMP(diff, 1.0f);
}

Point ClosestPoint(const Ray &ray, const Point &point)
{
    float t = Dot(point - ray.origin, ray.direction);
    t = fmaxf(t, 0.0f);
    return ray.origin + ray.direction * t;
}

bool SphereSphere(const Sphere &s1, const Sphere &s2)
{
    // compare distance between sphere center and sum of its radius.
    float radiiSum = s1.radius + s2.radius;
    float sqDistance = Magnitude(s1.position - s2.position);

    return sqDistance < radiiSum * radiiSum;
}

bool SphereAABB(const Sphere &sphere, const AABB &aabb)
{
    // find closest point on AABB to the sphere
    Point closestPoint = ClosestPoint(aabb, sphere.position);
    // compare distance between sphere to the point and the sphere radius
    float distSq = MagnitudeSq(sphere.position - closestPoint);
    float radiusSq = sphere.radius * sphere.radius;
    return distSq < radiusSq;
}

bool SphereOBB(const Sphere &sphere, const OBB &obb)
{
    Point closestPoint = ClosestPoint(obb, sphere.position);
    float distSq = MagnitudeSq(sphere.position - closestPoint);
    float radiusSq = sphere.radius * sphere.radius;
    return distSq < radiusSq;
}

bool SpherePlane(const Sphere &s, const Plane &p)
{
    Point closestPoint = ClosestPoint(p, s.position);
    float distSq = MagnitudeSq(s.position - closestPoint);
    float radiusSq = s.radius * s.radius;
    return distSq < radiusSq;
}

bool AABBAABB(const AABB &aabb1, const AABB &aabb2)
{
    // interval test on each of the axes
    Point aMin = GetMin(aabb1);
    Point aMax = GetMax(aabb2);
    Point bMin = GetMin(aabb2);
    Point bMax = GetMax(aabb2);

    return (aMin.x < bMax.x && aMax.x > bMin.x) &&
           (aMin.y < bMax.y && aMax.y > bMin.y) &&
           (aMin.z < bMax.z && aMax.z > bMin.z);
}

Interval GetInterval(const AABB &aabb, const vec3 &axis)
{
    vec3 i = GetMin(aabb);
    vec3 a = GetMax(aabb);
    // eight vertices of aabb
    vec3 vertex[8] = {
        vec3(i.x, a.y, a.z),
        vec3(i.x, a.y, i.z),
        vec3(i.x, i.y, a.z),
        vec3(i.x, i.y, i.z),
        vec3(a.x, a.y, a.z),
        vec3(a.x, a.y, i.z),
        vec3(a.x, i.y, a.z),
        vec3(a.x, i.y, i.z)};
    // project vertex onto axis and find min/max.
    Interval result;
    result.min = result.max = Dot(axis, vertex[0]);
    for (int i = 1; i < 8; ++i)
    {
        float projection = Dot(axis, vertex[i]);
        result.min = projection < result.min ? projection : result.min;
        result.max = projection > result.max ? projection : result.max;
    }
    return result;
}

Interval GetInterval(const OBB &obb, const vec3 &axis)
{
    // center
    vec3 c = obb.position;
    // extents
    vec3 e = obb.size;
    // axis
    const float *o = obb.orientation.asArray;
    vec3 a[] = {
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8])};
    // vertex
    vec3 vertex[8];
    vertex[0] = c + a[0] * e[0] + a[1] * e[1] + a[2] * e[2];
    vertex[1] = c - a[0] * e[0] + a[1] * e[1] + a[2] * e[2];
    vertex[2] = c + a[0] * e[0] - a[1] * e[1] + a[2] * e[2];
    vertex[3] = c + a[0] * e[0] + a[1] * e[1] - a[2] * e[2];
    vertex[4] = c - a[0] * e[0] - a[1] * e[1] - a[2] * e[2];
    vertex[5] = c + a[0] * e[0] - a[1] * e[1] - a[2] * e[2];
    vertex[6] = c - a[0] * e[0] + a[1] * e[1] - a[2] * e[2];
    vertex[7] = c - a[0] * e[0] - a[1] * e[1] + a[2] * e[2];
    // project onto axis and get min/max.
    Interval result;
    result.min = result.max = Dot(axis, vertex[0]);
    for (int i = 1; i < 8; ++i)
    {
        float projection = Dot(axis, vertex[i]);
        result.min = projection < result.min ? projection : result.min;
        result.max = projection > result.max ? projection : result.max;
    }

    return result;
}

bool OverlapOnAxis(const AABB &aabb, const OBB &obb, const vec3 &axis)
{
    Interval a = GetInterval(aabb, axis);
    Interval b = GetInterval(obb, axis);
    return b.min <= a.max && a.min <= b.max;
}

bool AABBOBB(const AABB &aabb, const OBB &obb)
{
    const float *o = obb.orientation.asArray;

    // axis of seperation.
    vec3 test[15] = {
        // aabb axis
        vec3(1, 0, 0),
        vec3(0, 1, 0),
        vec3(0, 0, 1),
        // obb axis
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8])};
    // axis of separation by taking cross product between aabb and obb axis.
    for (int i = 0; i < 3; ++i)
    {
        test[6 + i * 3 + 0] = Cross(test[i], test[0]);
        test[6 + i * 3 + 1] = Cross(test[i], test[1]);
        test[6 + i * 3 + 2] = Cross(test[i], test[2]);
    }
    // check all axis of seperation for overlap.
    for (int i = 0; i < 15; ++i)
    {
        if (!OverlapOnAxis(aabb, obb, test[i]))
        {
            return false;
        }
    }

    return true;
}

bool AABBPlane(const AABB &aabb, const Plane &plane)
{
    // projet half extents of AABB onto the plane normal.
    float pLen = aabb.size.x * fabsf(plane.normal.x) +
                 aabb.size.y * fabsf(plane.normal.y) +
                 aabb.size.z * fabsf(plane.normal.z);
    // distance from aabb to plane.
    float dot = Dot(plane.normal, aabb.position);
    float dist = dot - plane.distance;

    return fabsf(dist) < pLen;
}

bool OverlapOnAxis(const OBB &obb1, const OBB &obb2, const vec3 &axis)
{
    Interval a = GetInterval(obb1, axis);
    Interval b = GetInterval(obb2, axis);
    return b.min <= a.max && a.min <= b.max;
}

bool OBBOBB(const OBB &obb1, const OBB &obb2)
{
    const float *o1 = obb1.orientation.asArray;
    const float *o2 = obb2.orientation.asArray;
    // axis of potential separation
    vec3 test[15] = {
        vec3(o1[0], o1[1], o1[2]),
        vec3(o1[3], o1[4], o1[5]),
        vec3(o1[6], o1[7], o1[8]),
        vec3(o2[0], o2[1], o2[2]),
        vec3(o2[3], o2[4], o2[5]),
        vec3(o2[6], o2[7], o2[8]),
    };
    // cross product
    for (int i = 0; i < 3; ++i)
    {
        test[6 + i * 3 + 0] = Cross(test[i], test[0]);
        test[6 + i * 3 + 1] = Cross(test[i], test[1]);
        test[6 + i * 3 + 2] = Cross(test[i], test[2]);
    }
    // check overlap on each axis

    for (int i = 0; i < 15; ++i)
    {
        if (!OverlapOnAxis(obb1, obb2, test[i]))
        {
            return false;
        }
    }

    return true;
}

bool OBBPlane(const OBB &obb, const Plane &plane)
{
    // project half extents of obb onto the plane normal.
    const float *o = obb.orientation.asArray;
    vec3 rot[] = {
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8]),
    };
    vec3 normal = plane.normal;
    float pLen = obb.size.x * fabsf(Dot(normal, rot[0])) +
                 obb.size.y * fabsf(Dot(normal, rot[1])) +
                 obb.size.z * fabsf(Dot(normal, rot[2]));
    // distance between obb and plane
    float dot = Dot(plane.normal, obb.position);
    float dist = dot - plane.distance;
    return fabsf(dist) <= pLen;
}

bool PlanePlane(const Plane &plane1, const Plane &plane2)
{
    // check if plane normals are parallel.
    vec3 d = Cross(plane1.normal, plane2.normal);

    return !CMP(Dot(d, d), 0);
}

void ResetRaycastResult(RaycastResult* outResult) {
    if (outResult != 0) {
        outResult->t = -1;
        outResult->hit = false;
        outResult->normal = vec3(0, 0, 1);
        outResult->point = vec3(0, 0, 0);
    }
}

bool Raycast(const Sphere &sphere, const Ray &ray, RaycastResult* outResult)
{
    ResetRaycastResult(outResult);
    // vector from spehre to ray origin.
    vec3 e = sphere.position - ray.origin;
    float rSq = sphere.radius * sphere.radius;
    float eSq = MagnitudeSq(e);
    // project vector to ray direction.
    float a = Dot(e, ray.direction);
    // side of a triangle
    float bSq = eSq - (a * a);
    float f = sqrt(rSq - bSq);
    float t = a - f;
    // compare radius against the side
    // no collision
    if (rSq - (eSq - (a * a)) < 0.0f)
    {
        return false;
    }
    else if (eSq < rSq)
    {
        // ray inside sphere
        t = a + f;
    }
    if (outResult != 0) {
        outResult->t = t;
        outResult->hit = true;
        outResult->point = ray.origin + ray.direction * t;
        // vector from center of sphere to point of impact.
        outResult->normal = Normalized(outResult->point - sphere.position);
    }
    return true;
}

bool Raycast(const AABB &aabb, const Ray &ray, RaycastResult* outResult)
{
    ResetRaycastResult(outResult);
    vec3 min = GetMin(aabb);
    vec3 max = GetMax(aabb);

    // find intersections against each of the tree slabs.
    float t[] = {0 , 0, 0, 0, 0, 0};
    t[0] = (min.x - ray.origin.x) / ray.direction.x;
    t[1] = (max.x - ray.origin.x) / ray.direction.x;
    t[2] = (min.y - ray.origin.y) / ray.direction.y;
    t[3] = (max.y - ray.origin.y) / ray.direction.y;
    t[4] = (min.z - ray.origin.z) / ray.direction.z;
    t[5] = (max.z - ray.origin.z) / ray.direction.z;

    // largest minimum
    float tmin = fmaxf(fmaxf(fminf(t[0], t[1]), fminf(t[2], t[3])), fminf(t[4], t[5]));

    // smallest maximum
    float tmax = fminf(fminf(fmaxf(t[0], t[1]), fmaxf(t[2], t[3])), fmaxf(t[4], t[5]));

    if (tmax < 0)
    {
        // aabb behind ray.
        return false;
    }
    if (tmin > tmax)
    {
        // ray does not intersect.
        return false;
    }
    float t_result = tmin;
    if (tmin < 0.0f)
    {
        // ray inside aabb
        t_result = tmax;
    }
    if (outResult != 0) {
        outResult->t = t_result;
        outResult->hit = true;
        outResult->point = ray.origin + ray.direction * t_result;
        vec3 normals[] = {
            vec3(-1, 0, 0), vec3(1, 0, 0),
            vec3(0, -1, 0), vec3(0, 1, 0),
            vec3(0, 0, -1), vec3(0, 0, 1)
        };
        for (int i = 0; i < 6; ++i) {
            if (CMP(t_result, t[i])) {
                outResult->normal = normals[i];
            }
        }
    }

    return true;
}

bool Raycast(const OBB &obb, const Ray &ray, RaycastResult* outResult)
{
    ResetRaycastResult(outResult);
    const float *o = obb.orientation.asArray;
    const float *size = obb.size.asArray;
    // obb axis
    vec3 X(o[0], o[1], o[2]);
    vec3 Y(o[3], o[4], o[5]);
    vec3 Z(o[6], o[7], o[8]);
    // vector from ray to obb
    vec3 p = obb.position - ray.origin;
    // project direction of ray onto axis of obb.
    vec3 f(
        Dot(X, ray.direction),
        Dot(Y, ray.direction),
        Dot(Z, ray.direction));
    // project p onto obb axis.
    vec3 e(
        Dot(X, p),
        Dot(Y, p),
        Dot(Z, p));
    // find tmin, tmax for each slab
    float t[6] = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 3; ++i)
    {
        if (CMP(f[i], 0))
        {
            // ray parallel
            if (-e[i] - size[i] > 0 || -e[i] + size[i] < 0)
            {
                // ray outside slab
                return -1;
            }
            // avoid division by 0
            f[i] = 0.00001;
        }
        // min
        t[i * 2 + 0] = (e[i] + size[i]) / f[i];
        // max
        t[i * 2 + 1] = (e[i] - size[i]) / f[i];
    }
    // largest min
    float tmin = fmaxf(
        fmaxf(
            fminf(t[0], t[1]),
            fminf(t[2], t[3])),
        fminf(t[4], t[5]));
    // smallest max
    float tmax = fminf(
        fminf(
            fmaxf(t[0], t[1]),
            fmaxf(t[2], t[3])),
        fmaxf(t[4], t[5]));
    if (tmax < 0)
    {
        return false;
    }
    if (tmin > tmax)
    {
        return false;
    }
    float t_result = tmin;
    if (tmin < 0.0f)
    {
        t_result = tmax;
    }
    if (outResult != 0) {
        outResult->hit = true;
        outResult->t = t_result;
        outResult->point = ray.origin + ray.direction * t_result;
        vec3 normals[] = {
            X, X * -1.0f,
            Y, Y * -1.0f,
            Z, Z * -1.0f
        };
        for (int i = 0; i < 6; ++i) {
            if (CMP(t_result, t[i])) {
                outResult->normal = Normalized(normals[i]);
            }
        }
    }

    return true;
}

bool Raycast(const Plane &plane, const Ray &ray, RaycastResult* outResult)
{
    ResetRaycastResult(outResult);
    float nd = Dot(ray.direction, plane.normal);
    float pn = Dot(ray.origin, plane.normal);
    // ray and plane point normal point in the same direction
    if (nd >= 0.0f)
    {
        return false;
    }
    // plane equation
    float t = (plane.distance - pn) / nd;
    if (t >= 0.0f)
    {
        if (outResult != 0) {
            outResult->t = t;
            outResult->hit = true;
            outResult->point = ray.origin + ray.direction*t;
            outResult->normal = Normalized(plane.normal);
        }
        return true;
    }
    // negative t is not valid
    return false;
}

bool Linetest(const Sphere &sphere, const Line &line)
{
    // find closest point between line and sphere
    Point closest = ClosestPoint(line, sphere.position);
    // distance between closest poinst ans sphere center
    float distSq = MagnitudeSq(sphere.position - closest);
    // compare against sphere radius
    return distSq <= sphere.radius * sphere.radius;
}

bool Linetest(const AABB &aabb, const Line &line)
{
    // create ray out of the line.
    Ray ray;
    ray.origin = line.start;
    ray.direction = Normalized(line.end - line.start);
    // use ray cast of aabb and ray
    RaycastResult result;
    if (!Raycast(aabb, ray, &result)) {
        return false;
    }
    float t = result.t;

    return t >= 0 && t * t <= LengthSq(line);
}

bool Linetest(const OBB &obb, const Line &line)
{
    // create ray from line.
    Ray ray;
    ray.origin = line.start;
    ray.direction = Normalized(line.end - line.start);
    // use raycast of obb and ray
    RaycastResult result;
    if (!Raycast(obb, ray, &result)) {
        return false;
    }
    float t = result.t;
    // check t is within line segment.
    return t >= 0 && t * t <= LengthSq(line);
}

bool Linetest(const Plane &plane, const Line &line)
{
    vec3 ab = line.end - line.start;

    float nA = Dot(plane.normal, line.start);
    float nAB = Dot(plane.normal, ab);

    // line and plane are parallel
    if (CMP(nAB, 0))
    {
        return false;
    }

    // line segment intersect when it satisfies the plane equation.
    float t = (plane.distance - nA) / nAB;
    return t >= 0.0f && t <= 1.0f;
}

bool PointInTriangle(const Point &p, const Triangle &t)
{
}

bool PointInTraingle(const Point &p, const Triangle &t)
{
    // create triangle in the local coordinate system of the point.
    vec3 a = t.a - p;
    vec3 b = t.b - p;
    vec3 c = t.c - p;
    // normal of each side of pyramid
    vec3 normPBC = Cross(b, c);
    vec3 normPCA = Cross(c, a);
    vec3 normPAB = Cross(a, b);
    // check if normals are pointing in the same direction.
    if (Dot(normPBC, normPCA) < 0.0f)
    {
        return false;
    }
    else if (Dot(normPBC, normPAB) < 0.0f)
    {
        return false;
    }
    return true;
}

Plane FromTriangle(const Triangle &t)
{
    Plane result;
    result.normal = Normalized(
        Cross(t.b - t.a, t.c - t.a));
    result.distance = Dot(result.normal, t.a);
    return result;
}

Point ClosestPoint(const Triangle &t, const Point &p)
{
    // create plane from triangle.
    Plane plane = FromTriangle(t);
    // find closest point on plane to the point.
    Point closest = ClosestPoint(plane, p);
    // closest point in triangle
    if (PointInTriangle(closest, t))
    {
        return closest;
    }
    // closest point is on the edge of the triangle
    Point c1 = ClosestPoint(Line(t.a, t.b), p);
    Point c2 = ClosestPoint(Line(t.b, t.c), p);
    Point c3 = ClosestPoint(Line(t.c, t.a), p);
    float magSq1 = MagnitudeSq(p - c1);
    float magSq2 = MagnitudeSq(p - c2);
    float magSq3 = MagnitudeSq(p - c3);
    if (magSq1 < magSq2 && magSq1 < magSq3)
    {
        return c1;
    }
    else if (magSq2 < magSq1 && magSq2 < magSq3)
    {
        return c2;
    }
    return c3;
}

bool TriangleSphere(const Triangle &t, const Sphere &s)
{
    // find closest point on the triangle to the sphere.
    Point closest = ClosestPoint(t, s.position);
    // compare distance to sphere radius.
    float magSq = MagnitudeSq(closest - s.position);
    return magSq <= s.radius * s.radius;
}

Interval GetInterval(const Triangle &triangle, const vec3 &axis)
{
    // iterate through triangle points to get minimum and maximum.
    Interval result;
    result.min = Dot(axis, triangle.points[0]);
    result.max = result.min;

    for (int i = 1; i < 3; ++i)
    {
        float value = Dot(axis, triangle.points[i]);
        result.min = fminf(result.min, value);
        result.max = fmaxf(result.max, value);
    }

    return result;
}

bool OverlapOnAxis(const AABB &aabb, const Triangle &triangle, const vec3 &axis)
{
    Interval a = GetInterval(aabb, axis);
    Interval b = GetInterval(triangle, axis);
    return b.min <= a.max && a.min <= b.max;
}

bool TriangleAABB(const Triangle &t, const AABB &a)
{
    // SAT
    // edge vectors of triangle
    vec3 f0 = t.b - t.a;
    vec3 f1 = t.c - t.b;
    vec3 f2 = t.a - t.c;
    // face normals of aabb
    vec3 u0(1.0f, 0.0f, 0.0f);
    vec3 u1(0.0f, 1.0f, 0.0f);
    vec3 u2(0.0f, 0.0f, 1.0f);
    // potential separating axes.
    vec3 test[13] = {
        // normals of the aabb
        u0,
        u1,
        u2,
        // normal of triangle
        Cross(f0, f1),
        // cross product of aabb normals and triangle edges
        Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
        Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
        Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)};
    // test for overlap on each axes.
    for (int i = 0; i < 13; ++i)
    {
        if (!OverlapOnAxis(a, t, test[i]))
        {
            return false;
        }
    }

    // no axis of separation found.
    return true;
}

bool OverlapOnAxis(const OBB &obb, const Triangle &triangle, const vec3 &axis)
{
    Interval a = GetInterval(obb, axis);
    Interval b = GetInterval(triangle, axis);
    return b.min <= a.max && a.min <= b.max;
}

bool TriangleOBB(const Triangle &t, const OBB &o)
{
    // edge vectors of triangle
    vec3 f0 = t.b - t.a;
    vec3 f1 = t.c - t.b;
    vec3 f2 = t.a - t.c;
    // face normals of obb
    const float *orientation = o.orientation.asArray;
    vec3 u0(orientation[0], orientation[1], orientation[2]);
    vec3 u1(orientation[3], orientation[4], orientation[5]);
    vec3 u2(orientation[6], orientation[7], orientation[8]);
    // axes
    vec3 test[13] = {
        // obb normals
        u0,
        u1,
        u2,
        // triangle normal
        Cross(f0, f1),
        // cross product of aabb normals and triangle edges
        Cross(u0, f0), Cross(u0, f1), Cross(u0, f2),
        Cross(u1, f0), Cross(u1, f1), Cross(u1, f2),
        Cross(u2, f0), Cross(u2, f1), Cross(u2, f2)};
    // check for overlap on axes
    for (int i = 0; i < 13; ++i)
    {
        if (!OverlapOnAxis(o, t, test[i]))
        {
            return false;
        }
    }
    // no axis of separation.
    return true;
}

bool TrianglePlane(const Triangle &t, const Plane &p)
{
    // check which side of the plane that the point of triangle is on
    float side1 = PlaneEquation(t.a, p);
    float side2 = PlaneEquation(t.b, p);
    float side3 = PlaneEquation(t.c, p);

    // on the plane.
    if (CMP(side1, 0) && CMP(side2, 0) && CMP(side3, 0))
    {
        return true;
    }

    // in front of the plane.
    if (side1 > 0 && side2 > 0 && side3 > 0)
    {
        return false;
    }

    // behind the plane.
    if (side1 < 0 && side2 < 0 && side3 < 0)
    {
        return false;
    }

    // point on opposite side.
    return true;
}

bool OverlapOnAxis(const Triangle &t1, const Triangle &t2, const vec3 &axis)
{
    Interval a = GetInterval(t1, axis);
    Interval b = GetInterval(t2, axis);
    return b.min <= a.max && a.min <= b.max;
}

bool TriangleTriangle(const Triangle &t1, const Triangle &t2)
{
    // triangle edges
    vec3 t1_f0 = t1.b - t1.a;
    vec3 t1_f1 = t1.c - t1.b;
    vec3 t1_f2 = t1.a - t1.c;

    vec3 t2_f0 = t2.b - t2.a;
    vec3 t2_f1 = t2.c - t2.b;
    vec3 t2_f2 = t2.a - t2.c;

    // potential axis of separation
    vec3 axisToTest[] = {
        // normal of triangles
        Cross(t1_f0, t1_f1),
        Cross(t2_f0, t2_f1),
        // corss of triangle edges
        Cross(t2_f0, t1_f0),
        Cross(t2_f0, t1_f1),
        Cross(t2_f0, t1_f2),
        Cross(t2_f1, t1_f0),
        Cross(t2_f1, t1_f1),
        Cross(t2_f1, t1_f2),
        Cross(t2_f2, t1_f0),
        Cross(t2_f2, t1_f1),
        Cross(t2_f2, t1_f2),
    };

    for (int i = 0; i < 11; ++i)
    {
        if (!OverlapOnAxis(t1, t2, axisToTest[i]))
        {
            return false;
        }
    }
    // no separating axis.
    return true;
}

vec3 SatCrossEdge(const vec3 &a, const vec3 &b, const vec3 &c, const vec3 &d)
{
    vec3 ab = a - b;
    vec3 cd = c - d;
    vec3 result = Cross(ab, cd);
    // handle parallel sides.
    if (!CMP(MagnitudeSq(result), 0))
    {
        return result;
    }
    else
    {
        vec3 axis = Cross(ab, c - a);
        result = Cross(ab, axis);
        if (!CMP(MagnitudeSq(result), 0))
        {
            return result;
        }
    }
    return vec3();
}

bool TriangleTriangleRobust(const Triangle &t1, const Triangle &t2)
{
    // triangle edges
    vec3 t1_f0 = t1.b - t1.a;
    vec3 t1_f1 = t1.c - t1.b;
    vec3 t1_f2 = t1.a - t1.c;

    vec3 t2_f0 = t2.b - t2.a;
    vec3 t2_f1 = t2.c - t2.b;
    vec3 t2_f2 = t2.a - t2.c;

    // potential axis of separation
    vec3 axisToTest[] = {
        // normal of triangles
        SatCrossEdge(t1.a, t1.b, t1.b, t1.c),
        SatCrossEdge(t2.a, t2.b, t2.c, t2.c),
        // corss of triangle edges
        SatCrossEdge(t2.a, t2.b, t1.a, t1.b),
        SatCrossEdge(t2.a, t2.b, t1.b, t1.c),
        SatCrossEdge(t2.a, t2.b, t1.c, t1.a),
        SatCrossEdge(t2.b, t2.c, t1.a, t1.b),
        SatCrossEdge(t2.b, t2.c, t1.b, t1.c),
        SatCrossEdge(t2.b, t2.c, t1.c, t1.a),
        SatCrossEdge(t2.c, t2.a, t1.a, t1.b),
        SatCrossEdge(t2.c, t2.a, t1.b, t1.c),
        SatCrossEdge(t2.c, t2.a, t1.c, t1.a),
    };

    for (int i = 0; i < 11; ++i)
    {
        if (!OverlapOnAxis(t1, t2, axisToTest[i]))
        {
            // handle edges on straight line.
            if (!CMP(MagnitudeSq(axisToTest[i]), 0))
            {
                return false;
            }
        }
    }
    // no separating axis.
    return true;
}

vec3 Barycentric(const Point& p, const Triangle& t) {
    // vectors from test point to triangle points.
    vec3 ap = p - t.a;
    vec3 bp = p - t.b;
    vec3 cp = p - t.c;
    // triangle edges.s
    vec3 ab = t.b - t.a;
    vec3 ac = t.c - t.a;
    vec3 bc = t.c - t.b;
    vec3 cb = t.b - t.c;
    vec3 ca = t.a - t.c;
    // vector perpendicular to edge ab.
    vec3 v = ab - Project(ab, cb);
    float a = 1.0f - (Dot(v, ap) / Dot(v, ab));
    // vector perpendicular to edge bc.
    v = bc - Project(bc, ac);
    float b = 1.0f - (Dot(v, bp) / Dot(v, bc));
    // vector perpendicular to edge ca.
    v = ca - Project(ca, ab);
    float c = 1.0f - (Dot(v, cp) / Dot(v, ca));

    return vec3(a, b, c);
}

bool Raycast(const Triangle& triangle, const Ray& ray, RaycastResult* outResult) {
    ResetRaycastResult(outResult);
    // create plane from triangle.
    Plane plane = FromTriangle(triangle);
    // raycast against plane.
    RaycastResult planeResult;
    if (!Raycast(plane, ray, &planeResult)) {
        return false;
    }
    float t = planeResult.t;
    // point on plane
    Point result = ray.origin + ray.direction * t;
    vec3 barycentric = Barycentric(result, triangle);
    if (barycentric.x >= 0.0f && barycentric.x <= 1.0f && 
        barycentric.y >= 0.0f && barycentric.y <= 1.0f &&
        barycentric.z >= 0.0f && barycentric.z <= 1.0f) {
        // point within triangle.
        if (outResult != 0) {
            outResult->t = t;
            outResult->hit = true;
            outResult->point = ray.origin + ray.direction * t;
            outResult->normal = plane.normal;
        }
        return true;
    }
    // point outside triangle.
    return false;
}

bool Linetest(const Triangle& triangle, const Line& line) {
    // create ray from line.
    Ray ray;
    ray.origin = line.start;
    ray.direction = Normalized(line.end - line.start);
    // raycast against triangle
    RaycastResult raycast;
    if (!Raycast(triangle, ray, &raycast)) {
        return false;
    }
    float t = raycast.t;
    // check if within line segment.
    return t >= 0 && t * t <= LengthSq(line);
}

void AccelerateMesh(Mesh& mesh) {
    if (mesh.accelerator != 0) {
        return;
    }
    // find minimum and maximum points of mesh.
    vec3 min = mesh.vertices[0];
    vec3 max = mesh.vertices[0];
    for (int i = 1; i < mesh.numTriangles * 3; ++i) {
        min.x = fminf(mesh.vertices[i].x, min.x);
        min.y = fminf(mesh.vertices[i].y, min.y);
        min.z = fminf(mesh.vertices[i].z, min.z);
        max.x = fmaxf(mesh.vertices[i].x, max.x);
        max.y = fmaxf(mesh.vertices[i].y, max.y);
        max.z = fmaxf(mesh.vertices[i].z, max.z);
    }
    // create bvh for mesh.
    mesh.accelerator = new BVHNode();
    mesh.accelerator->bounds = FromMinMax(min, max);
    mesh.accelerator->numTriangles = mesh.numTriangles;
    // allocate memory for triangle indices.
    mesh.accelerator->triangles = new int[mesh.numTriangles];
    for (int i = 0 ; i < mesh.numTriangles; ++i) {
        mesh.accelerator-> triangles[i] = i;
    }
    // split bvh tree.
    SplitBVHNode(mesh.accelerator, mesh, 3);   
}

void SplitBVHNode(BVHNode* node, const Mesh& model, int depth) {
    if (depth-- == 0) {
        return;
    }
    if (node->children == 0) {
        // split leaf node
        if (node->numTriangles > 0) {
            // eight children
            node->children = new BVHNode[8];
            vec3 c = node->bounds.position;
            vec3 e = node->bounds.size * 0.5f;

            node->children[0].bounds = AABB(c + vec3(-e.x, +e.y, -e.z), e);
            node->children[1].bounds = AABB(c + vec3(+e.x, +e.y, -e.z), e);
            node->children[2].bounds = AABB(c + vec3(-e.x, +e.y, +e.z), e);
            node->children[3].bounds = AABB(c + vec3(+e.x, +e.y, +e.z), e);
            node->children[4].bounds = AABB(c + vec3(-e.x, -e.y, -e.z), e);
            node->children[5].bounds = AABB(c + vec3(+e.x, -e.y, -e.z), e);
            node->children[6].bounds = AABB(c + vec3(-e.x, -e.y, +e.z), e);
            node->children[7].bounds = AABB(c + vec3(+e.x, -e.y, +e.z), e);

        }

    }

    if (node->children != 0 && node->numTriangles > 0) {
        // assign triangles to child node.
        for (int i = 0; i < 8; ++i) {
            node->children[i].numTriangles = 0;
            for (int j = 0; j < node->numTriangles; ++j) {
                Triangle t = model.triangles[node->triangles[j]];
                // check if intersect with child bound
                if (TriangleAABB(t, node->children[i].bounds)) {
                    node->children[i].numTriangles += 1;
                }
            }
            if (node->children[i].numTriangles == 0) {
                continue;
            }
            node->children[i].triangles = new int[node->children[i].numTriangles];
            int index = 0;
            for (int j = 0; j < node->numTriangles; ++j) {
                Triangle t = model.triangles[node->triangles[j]];
                if (TriangleAABB(t, node->children[i].bounds)) {
                    node->children[i].triangles[index++] = node->triangles[j]; 
                }
            }
        }

        // clean up
        node->numTriangles = 0;
        delete[] node->triangles;
        node->triangles = 0;

        // recurse
        for (int i = 0; i < 8; ++i) {
            SplitBVHNode(&node->children[i], model, depth);
        }
    }
}

void FreeBVHNode(BVHNode* node) {
    // free children
    if (node->children != 0) {
        // recursive free bvh node children
        for (int i = 0; i < 8; ++i) {
            FreeBVHNode(&node->children[i]);
        }
        delete[] node->children;
        node->children = 0;
    }
    // free triangles
    if (node->numTriangles != 0 || node->triangles != 0) {
        delete[] node->triangles;
        node->triangles = 0;
        node->numTriangles = 0;
    }
}

float MeshRay(const Mesh& mesh, const Ray& ray) {
    if (mesh.accelerator == 0) {
        // iterate through triangles in mesh and raycast against each
        for (int i = 0; i < mesh.numTriangles; ++i) {
            RaycastResult raycast;
            Raycast(mesh.triangles[i], ray, &raycast);
            float result = raycast.t;
            if (result >= 0) {
                return result;
            }
        }
    } else {
        // dfs bvh tree
        std::list<BVHNode*> toProcess;
        toProcess.push_front(mesh.accelerator);
        while (!toProcess.empty()) {
            // current node
            BVHNode* iterator = *(toProcess.begin());
            toProcess.erase(toProcess.begin());
            // has triangles
            if (iterator->numTriangles >= 0) {
                for (int i = 0; i < iterator->numTriangles; ++i) {
                    RaycastResult raycast;
                    Raycast(mesh.triangles[iterator->triangles[i]], ray, &raycast);
                    float r = raycast.t;
                    if (r >= 0) {
                        return r;
                    }
                }
            }
            // has children
            if (iterator->children != 0) {
                for (int i = 8 - 1; i >= 0; --i) {
                    // raycast against bound
                    RaycastResult raycast;
                    Raycast(iterator->children[i].bounds, ray, &raycast);
                    if (raycast.t >= 0) {
                        toProcess.push_front(&iterator->children[i]);
                    }
                }
            }
        }
    }

    return -1;
}

bool MeshAABB(const Mesh& mesh, const AABB& aabb) {
    if (mesh.accelerator == 0) {
        // iterate through triangles in mesh and test against each
        for (int i = 0; i < mesh.numTriangles; ++i) {
            if (TriangleAABB(mesh.triangles[i], aabb)) {
                return true;
            }
        }
    } else {
        // dfs bvh tree
        std::list<BVHNode*> toProcess;
        toProcess.push_front(mesh.accelerator);
        while (!toProcess.empty()) {
            // current node
            BVHNode* iterator = *(toProcess.begin());
            toProcess.erase(toProcess.begin());
            // has triangles
            if (iterator->numTriangles >= 0) {
                for (int i = 0; i < iterator->numTriangles; ++i) {
                    if (TriangleAABB(mesh.triangles[iterator->triangles[i]], aabb)) {
                        return true;
                    }
                }
            }
            // has children
            if (iterator->children != 0) {
                for (int i = 8 - 1; i >= 0; --i) {
                    // test against bound
                    if (AABBAABB(iterator->children[i].bounds, aabb) >= 0) {
                        toProcess.push_front(&iterator->children[i]);
                    }
                }
            }
        }
    }

    return -1;
}

bool Linetest(const Mesh& mesh, const Line& line) {
    if (mesh.accelerator == 0) {
        // iterate through triangles in mesh and test against each
        for (int i = 0; i < mesh.numTriangles; ++i) {
            if (Linetest(mesh.triangles[i], line)) {
                return true;
            }
        }
    } else {
        // dfs bvh tree
        std::list<BVHNode*> toProcess;
        toProcess.push_front(mesh.accelerator);
        while (!toProcess.empty()) {
            // current node
            BVHNode* iterator = *(toProcess.begin());
            toProcess.erase(toProcess.begin());
            // has triangles
            if (iterator->numTriangles >= 0) {
                for (int i = 0; i < iterator->numTriangles; ++i) {
                    if (Linetest(mesh.triangles[iterator->triangles[i]], line)) {
                        return true;
                    }
                }
            }
            // has children
            if (iterator->children != 0) {
                for (int i = 8 - 1; i >= 0; --i) {
                    // test against bound
                    if (Linetest(iterator->children[i].bounds, line) >= 0) {
                        toProcess.push_front(&iterator->children[i]);
                    }
                }
            }
        }
    }

    return -1;
}

bool MeshSphere(const Mesh& mesh, const Sphere& sphere) {
    if (mesh.accelerator == 0) {
        // iterate through triangles in mesh and test against each
        for (int i = 0; i < mesh.numTriangles; ++i) {
            if (TriangleSphere(mesh.triangles[i], sphere)) {
                return true;
            }
        }
    } else {
        // dfs bvh tree
        std::list<BVHNode*> toProcess;
        toProcess.push_front(mesh.accelerator);
        while (!toProcess.empty()) {
            // current node
            BVHNode* iterator = *(toProcess.begin());
            toProcess.erase(toProcess.begin());
            // has triangles
            if (iterator->numTriangles >= 0) {
                for (int i = 0; i < iterator->numTriangles; ++i) {
                    if (TriangleSphere(mesh.triangles[iterator->triangles[i]], sphere)) {
                        return true;
                    }
                }
            }
            // has children
            if (iterator->children != 0) {
                for (int i = 8 - 1; i >= 0; --i) {
                    // test against bound
                    if (SphereAABB(sphere, iterator->children[i].bounds) >= 0) {
                        toProcess.push_front(&iterator->children[i]);
                    }
                }
            }
        }
    }

    return -1;
}

bool MeshOBB(const Mesh& mesh, const OBB& obb) {
    if (mesh.accelerator == 0) {
        // iterate through triangles in mesh and test against each
        for (int i = 0; i < mesh.numTriangles; ++i) {
            if (TriangleOBB(mesh.triangles[i], obb)) {
                return true;
            }
        }
    } else {
        // dfs bvh tree
        std::list<BVHNode*> toProcess;
        toProcess.push_front(mesh.accelerator);
        while (!toProcess.empty()) {
            // current node
            BVHNode* iterator = *(toProcess.begin());
            toProcess.erase(toProcess.begin());
            // has triangles
            if (iterator->numTriangles >= 0) {
                for (int i = 0; i < iterator->numTriangles; ++i) {
                    if (TriangleOBB(mesh.triangles[iterator->triangles[i]], obb)) {
                        return true;
                    }
                }
            }
            // has children
            if (iterator->children != 0) {
                for (int i = 8 - 1; i >= 0; --i) {
                    // test against bound
                    if (AABBOBB(iterator->children[i].bounds, obb) >= 0) {
                        toProcess.push_front(&iterator->children[i]);
                    }
                }
            }
        }
    }

    return -1;
}

bool MeshPlane(const Mesh& mesh, const Plane& plane) {
    if (mesh.accelerator == 0) {
        // iterate through triangles in mesh and test against each
        for (int i = 0; i < mesh.numTriangles; ++i) {
            if (TrianglePlane(mesh.triangles[i], plane)) {
                return true;
            }
        }
    } else {
        // dfs bvh tree
        std::list<BVHNode*> toProcess;
        toProcess.push_front(mesh.accelerator);
        while (!toProcess.empty()) {
            // current node
            BVHNode* iterator = *(toProcess.begin());
            toProcess.erase(toProcess.begin());
            // has triangles
            if (iterator->numTriangles >= 0) {
                for (int i = 0; i < iterator->numTriangles; ++i) {
                    if (TrianglePlane(mesh.triangles[iterator->triangles[i]], plane)) {
                        return true;
                    }
                }
            }
            // has children
            if (iterator->children != 0) {
                for (int i = 8 - 1; i >= 0; --i) {
                    // test against bound
                    if (AABBPlane(iterator->children[i].bounds, plane) >= 0) {
                        toProcess.push_front(&iterator->children[i]);
                    }
                }
            }
        }
    }

    return -1;
}


bool MeshTriangle(const Mesh& mesh, const Triangle& triangle) {
    if (mesh.accelerator == 0) {
        // iterate through triangles in mesh and test against each
        for (int i = 0; i < mesh.numTriangles; ++i) {
            if (TriangleTriangle(mesh.triangles[i], triangle)) {
                return true;
            }
        }
    } else {
        // dfs bvh tree
        std::list<BVHNode*> toProcess;
        toProcess.push_front(mesh.accelerator);
        while (!toProcess.empty()) {
            // current node
            BVHNode* iterator = *(toProcess.begin());
            toProcess.erase(toProcess.begin());
            // has triangles
            if (iterator->numTriangles >= 0) {
                for (int i = 0; i < iterator->numTriangles; ++i) {
                    if (TriangleTriangle(mesh.triangles[iterator->triangles[i]], triangle)) {
                        return true;
                    }
                }
            }
            // has children
            if (iterator->children != 0) {
                for (int i = 8 - 1; i >= 0; --i) {
                    // test against bound
                    if (TriangleAABB(triangle, iterator->children[i].bounds) >= 0) {
                        toProcess.push_front(&iterator->children[i]);
                    }
                }
            }
        }
    }

    return -1;
}

void Model::setContent(Mesh* mesh) {
    content = mesh;
    // calculate bound for mesh
    if (content != 0) {
        vec3 min = mesh->vertices[0];
        vec3 max = mesh->vertices[0];

        for (int i = 1; i < mesh->numTriangles * 3; ++i) {
            min.x = fminf(mesh->vertices[i].x, min.x);
            min.y = fminf(mesh->vertices[i].y, min.y);
            min.z = fminf(mesh->vertices[i].z, min.z);
            max.x = fmaxf(mesh->vertices[i].x, max.x);
            max.y = fmaxf(mesh->vertices[i].y, max.y);
            max.z = fmaxf(mesh->vertices[i].z, max.z);
        }
        bounds = FromMinMax(min, max);
    }
}

mat4 GetWorldMatrix(const Model& model) {
    mat4 translation = Translation(model.position);
    mat4 rotation = Rotation(
        model.rotation.x,
        model.rotation.y,
        model.rotation.z
    );
    mat4 localMat = rotation * translation;
    // world matrix of parent
    mat4 parentMat;
    if (model.parent != 0) {
        parentMat = GetWorldMatrix(*model.parent);
    }
    // combine local and parent world matrix
    return localMat * parentMat;
}

OBB GetOBB(const Model& model) {
    mat4 world = GetWorldMatrix(model);
    AABB aabb = model.GetBounds();
    OBB obb;
    obb.size = aabb.size;
    obb.position = MultiplyPoint(aabb.position, world);
    obb.orientation = Cut(world, 3, 3);

    return obb;
}

float ModelRay(const Model& model, const Ray& ray) {
    // get inverse world matrix
    mat4 world = GetWorldMatrix(model);
    mat4 inv = Inverse(world);
    Ray local;
    // transform ray to local space of model
    local.origin = MultiplyPoint(ray.origin, inv);
    local.direction = MultiplyVector(ray.direction, inv);
    local.NormalizeDirection();
    if (model.GetMesh() != 0) {
        return MeshRay(*(model.GetMesh()), local);
    }
    return -1;
}

bool Linetest(const Model& model, const Line& line) {
    mat4 world = GetWorldMatrix(model);
    mat4 inv = Inverse(world);
    Line local;
    local.start = MultiplyPoint(line.start, inv);
    local.end = MultiplyPoint(line.end, inv);
    if (model.GetMesh() != 0) {
        return Linetest(*(model.GetMesh()), local);
    }
    return false;
}

bool ModelSphere(const Model& model, const Sphere& sphere) {
    mat4 world = GetWorldMatrix(model);
    mat4 inv = Inverse(world);
    Sphere local;
    local.position = MultiplyPoint(sphere.position, inv);
    if (model.GetMesh() != 0) {
        return MeshSphere(*(model.GetMesh()), local);
    }
    return false;
}

bool ModelAABB(const Model& model, const AABB& aabb) {
    mat4 world = GetWorldMatrix(model);
    mat4 inv = Inverse(world);
    OBB local;
    local.size = aabb.size;
    local.position = MultiplyPoint(aabb.position, inv);
    local.orientation = Cut(inv, 3, 3);
    if (model.GetMesh() != 0) {
        return MeshOBB(*(model.GetMesh()), local);
    }
    return false;
} 

bool ModelOBB(const Model& model, const OBB& obb) {
    mat4 world = GetWorldMatrix(model);
    mat4 inv = Inverse(world);
    OBB local;
    local.size = obb.size;
    local.position = MultiplyPoint(obb.position, inv);
    local.orientation = obb.orientation * Cut(inv, 3, 3);
    if (model.GetMesh() != 0) {
        return MeshOBB(*(model.GetMesh()), local);
    }
    return false;
}

bool ModelPlane(const Model& model, const Plane& plane) {
    mat4 world = GetWorldMatrix(model);
    mat4 inv = Inverse(world);
    Plane local;
    local.normal = MultiplyVector(plane.normal, inv);
    local.distance = plane.distance;
    if (model.GetMesh() != 0) {
        return MeshPlane(*(model.GetMesh()), local);
    }
    return false;
}

bool ModelTriangle(const Model& model, const Triangle& triangle) {
    mat4 world = GetWorldMatrix(model);
    mat4 inv = Inverse(world);
    Triangle local;
    local.a = MultiplyPoint(triangle.a, inv);
    local.b = MultiplyPoint(triangle.b, inv);
    local.c = MultiplyPoint(triangle.c, inv);
    if (model.GetMesh() != 0) {
        return MeshTriangle(*(model.GetMesh()), local);
    }
    return false;
}

Point Intersection(Plane p1, Plane p2, Plane p3) {
    // solve using  cramer's rule
    // coefficient matrix.
    mat3 D(
        p1.normal.x, p2.normal.x, p3.normal.x,
        p1.normal.y, p2.normal.y, p3.normal.y,
        p1.normal.z, p2.normal.z, p3.normal.z
    );
    // answer row
    vec3 A(-p1.distance, -p2.distance, -p3.distance);
    // row replaced by answer row
    mat3 Dx = D;
    mat3 Dy = D;
    mat3 Dz = D;
    Dx._11 = A.x; Dx._12 = A.y; Dx._13 = A.z;
    Dy._21 = A.x; Dy._22 = A.y; Dy._23 = A.z;
    Dz._31 = A.x; Dz._32 = A.y; Dz._33 = A.z;
    // determinant
    float detD = Determinant(D);
    if (CMP(detD, 0)) {
        return Point();
    }
    float detDx = Determinant(Dx);
    float detDy = Determinant(Dy);
    float detDz = Determinant(Dz);
    return Point(detDx / detD, detDy / detD, detDz / detD);
}

void GetCorners(const Frustum& f, vec3* outCorners) {
    // call intersection on the frustum planes.
    outCorners[0] = Intersection(f._near, f.top, f.left);
    outCorners[1] = Intersection(f._near, f.top, f.right);
    outCorners[2] = Intersection(f._near, f.bottom, f.left);
    outCorners[3] = Intersection(f._near, f.bottom, f.right);
    outCorners[2] = Intersection(f._far, f.bottom, f.left);
    outCorners[3] = Intersection(f._far, f.bottom, f.right);
    outCorners[2] = Intersection(f._far, f.bottom, f.left);
    outCorners[3] = Intersection(f._far, f.bottom, f.right);
}

bool Intersects(const Frustum& f, const Point& p) {
    // loop planes of frustum
    for (int i = 0; i < 6; ++i) {
        vec3 normal = f.planes[i].normal;
        float dist = f.planes[i].distance;
        float side = Dot(p, normal) + dist;
        if (side < 0.0f) {
            // behind plane
            return false;
        }
    }
    return true;
}

bool Intersects(const Frustum& f, const Sphere& s) {
    for (int i = 0; i < 6; ++i) {
        vec3 normal = f.planes[i].normal;
        float dist = f.planes[i].distance;
        float side = Dot(s.position, normal) + dist;
        if (side < -s.radius) {
            // behind plane
            return false;
        }
    }
    return true;
}

float Classify(const AABB& aabb, const Plane& plane) {
    // projection of positive extents of aabb onto the plane.
    float r = fabsf(aabb.size.x * plane.normal.x) + fabsf(aabb.size.y * plane.normal.y) + fabsf(aabb.size.z * plane.normal.z);
    // signed distance betweeen center of aabb and plane.
    float d = Dot(plane.normal, aabb.position) + plane.distance;
    if (fabsf(d) < r) {
        return 0.0f;
    } else if (d < 0.0f) {
        // box behind plane
        return d + r;
    }
    // box in front of plane
    return d - r;
}

float Classify(const OBB& obb, const Plane& plane) {
    // transform plane normal to local space of the obb
    vec3 normal = MultiplyVector(plane.normal, obb.orientation);
    float r = fabsf(obb.size.x * normal.x) + fabsf(obb.size.y * normal.y) + fabsf(obb.size.z * normal.z);
    // signed distance
    float d = Dot(plane.normal, obb.position) + plane.distance;

    if (fabsf(d) < r) {
        return 0.0f;
    } else if (d < 0.0f) {
        return d + r;
    }
    return d - r;
}

bool Intersects(const Frustum& f, const AABB& aabb) {
    for (int i = 0; i < 6; ++i) {
        if (Classify(aabb, f.planes[i]) < 0) {
            return false;
        }
    }
    return true;
}

bool Intersects(const Frustum& f, const OBB& obb) {
    for (int i = 0; i < 6; ++i) {
        if (Classify(obb, f.planes[i]) < 0) {
            return false;
        }
    }
    return true;
}

vec3 Unproject(const vec3& viewportPoint, const vec2& viewportOrigin, const vec2& viewportSize, const mat4& view, const mat4& projection) {
    // normalize input vector to the view port.
    float normalized[4] = {
        (viewportPoint.x - viewportOrigin.x) / viewportSize.x,
        (viewportPoint.y - viewportOrigin.y) / viewportSize.y,
        viewportPoint.z,
        1.0f
    };
    // translate to ndc space
    float ndcSpace[4] = {
        normalized[0], normalized[1],
        normalized[2], normalized[3]
    };
    ndcSpace[0] = ndcSpace[0] * 2.0f - 1.0f;
    ndcSpace[1] = 1.0f - ndcSpace[1] * 2.0f;
    if (ndcSpace[2] < 0.0f) {
        ndcSpace[2] = 0.0f;
    }
    if (ndcSpace[2] > 1.0f) {
        ndcSpace[2] = 1.0f;
    }
    // transform to eye space
    mat4 invProjection = Inverse(projection);
    float eyeSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    Multiply(eyeSpace, ndcSpace, 1, 4, invProjection.asArray, 4, 4);
    // traslate into world space
    mat4 invView = Inverse(view);
    float worldSpace[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    Multiply(worldSpace, eyeSpace, 1, 4, invView.asArray, 4, 4);
    // undo perspective divide
    if (!CMP(worldSpace[3], 0.0f)) {
        worldSpace[0] /= worldSpace[3];
        worldSpace[1] /= worldSpace[3];
        worldSpace[2] /= worldSpace[3];
    }

    return vec3(worldSpace[0], worldSpace[1], worldSpace[2]);
}

Ray GetPickRay(const vec2& viewportPoint, const vec2& viewportOrigin, const vec2& viewportSize, const mat4& view, const mat4& projection) {
    // near and far point.
    vec3 nearPoint(viewportPoint.x, viewportPoint.y, 0.0f);
    vec3 farPoint(viewportPoint.x, viewportPoint.y, 1.0f);
    // unproject
    vec3 pNear = Unproject(nearPoint, viewportOrigin, viewportSize, view, projection);
    vec3 pFar = Unproject(farPoint, viewportOrigin, viewportSize, view, projection);
    // construct ray
    vec3 normal = Normalized(pFar - pNear);
    vec3 origin = pNear;
    return Ray(origin, normal);
}

void ResetCollisionManifold(CollisionManifold* result) {
    if (result != 0) {
        result->colliding = false;
        result->normal = vec3(0, 0, 1);
        result->depth = FLT_MAX;
        result->contacts.clear();
    }
}

CollisionManifold FindCollisionFeatures(const Sphere& A, const Sphere& B) {
    CollisionManifold result;
    ResetCollisionManifold(&result);

    // check for intersection
    float r = A.radius + B.radius;
    vec3 d = B.position - A.position;
    if (Magnitude(d) - r *r > 0 || MagnitudeSq(d) == 0.0f) {
        return result;
    }

    Normalize(d);
    // set manifold
    result.colliding = true;
    result.normal = d;
    // half the distance between the sphere minus the combined radius
    result.depth = fabsf(Magnitude(d) - r) * 0.5f;
    // distance to intersectino point
    float dtp = A.radius - result.depth;
    Point contact = A.position + d * dtp;
    result.contacts.push_back(contact);

    return result;
}

CollisionManifold FindCollisionFeatures(const OBB& A, const Sphere& B) {
    CollisionManifold result;
    ResetCollisionManifold(&result);
    // check intersection
    // closest poitn on surface of obb
    Point closestPoint = ClosestPoint(A, B.position);
    float distanceSq = MagnitudeSq(closestPoint - B.position);
    if (distanceSq > B.radius * B.radius) {
        return result;
    }
    // normal is vector from closest point on the obb to center of sphere
    vec3 normal;
    if (CMP(distanceSq, 0.0f)) {
        // closest point at center of sphere
        float mSq = Magnitude(closestPoint - A.position);
        if (CMP(mSq, 0.0f)) {
            // can't find normal vector
            return result;
        } 
        normal = Normalized(closestPoint - A.position);
    } else {
        normal = Normalized(B.position - closestPoint);
    }
    // closest point on surface of sphere
    Point outsidePoint = B.position - normal * B.radius;
    float distance = Magnitude(closestPoint - outsidePoint);
    result.colliding = true;
    // halfway between the objects along the collision normal
    result.contacts.push_back(closestPoint + (outsidePoint - closestPoint) * 0.5f);
    result.normal = normal;
    result.depth = distance * 0.5f;

    return result;
}

std::vector<Point> GetVertices(const OBB& obb) {
    std::vector<vec3> v;
    v.resize(8);
    vec3 C = obb.position;
    vec3 E = obb.size;
    const float* o = obb.orientation.asArray;
    vec3 A[] = {
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8]),
    };
    v[0] = C + A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    v[1] = C - A[0] * E[0] + A[1] * E[1] + A[2] * E[2];
    v[2] = C + A[0] * E[0] - A[1] * E[2] + A[2] * E[2];
    v[3] = C + A[0] * E[0] + A[1] * E[2] - A[2] * E[2];
    v[4] = C - A[0] * E[0] - A[1] * E[2] - A[2] * E[2];
    v[5] = C + A[0] * E[0] - A[1] * E[2] - A[2] * E[2];
    v[6] = C - A[0] * E[0] + A[1] * E[2] - A[2] * E[2];
    v[7] = C - A[0] * E[0] - A[1] * E[2] + A[2] * E[2];

    return v;
}

std::vector<Line> GetEdges(const OBB& obb) {
    std::vector<Line> result;
    result.reserve(12);
    std::vector<Point> v = GetVertices(obb);
    int index[][2] = {
        {6, 1}, {6, 3}, {6, 4}, {2, 7} ,{2, 5}, {2, 0},
        {0, 1}, {0, 3}, {7, 1}, {7, 4}, {4, 5}, {5, 3}
    };
    for (int j = 0; j < 12; ++j) {
        result.push_back(Line(
            v[index[j][0]], v[index[j][1]]
        ));
    }
    return result;
}

std::vector<Plane> GetPlanes(const OBB& obb) {
    vec3 c = obb.position;
    vec3 e = obb.size;
    const float* o = obb.orientation.asArray;
    vec3 a[] = {
        vec3(o[0], o[1], o[2]),
        vec3(o[3], o[4], o[5]),
        vec3(o[6], o[7], o[8])
    };
    std::vector<Plane> result;
    result.resize(6);
    result[0] = Plane(a[0], Dot(a[0], (c + a[0] * e.x)));
    result[1] = Plane(a[0] * -1.0f, -Dot(a[0], (c - a[0] * e.x)));
    result[2] = Plane(a[1], Dot(a[1], (c + a[1] * e.y)));
    result[3] = Plane(a[1] * -1.0f, -Dot(a[1], (c - a[1] * e.y)));
    result[4] = Plane(a[2], Dot(a[2], (c + a[2] * e.z)));
    result[5] = Plane(a[2] * -1.0f, -Dot(a[2], (c - a[2] * e.z)));
    
    return result;
}