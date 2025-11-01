#include "Geometry3D.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) \
    (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

float Length(const Line& line) {
    return Magnitude(line.start - line.end);
}

float LengthSq(const Line& line) {
    return MagnitudeSq(line.start - line.end);
}

Ray FromPoints(const Point& from, const Point& to) {
    return Ray(from, Normalized(to - from));
}

vec3 GetMin(const AABB& aabb) {
    vec3 p1 = aabb.position + aabb.size;
    vec3 p2 = aabb.position - aabb.size;

    return vec3(fminf(p1.x, p2.x), fminf(p1.y, p2.y), fminf(p1.z, p2.z));
}

vec3 GetMax(const AABB& aabb) {
    vec3 p1 = aabb.position + aabb.size;
    vec3 p2 = aabb.position - aabb.size;

    return  vec3(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y), fmaxf(p1.z, p2.z));
}

AABB FromMinMax(const vec3& min, const vec3& max) {
    return AABB((min + max) * 0.5, (max - min) * 0.5f);
}

float PlaneEquation(const Point& point, const Plane& plane) {
    return Dot(point, plane.normal) - plane.distance;
}

bool PointInSphere(const Point& point, const Sphere& sphere) {
    // distance between point and sphere center
    float magSq = MagnitudeSq(point - sphere.position);
    float radSq = sphere.radius * sphere.radius;
    return magSq < radSq;
}

Point ClosestPoint(const Sphere& sphere, const Point& point) {
    vec3 sphereToPoint = point - sphere.position;
    Normalize(sphereToPoint);
    sphereToPoint = sphereToPoint * sphere.radius;
    return sphere.position + sphereToPoint;
} 

bool PointInAABB(const Point& point, const AABB& aabb) {
    // compare with minimum and maximum point of aabb component-wise.
    Point min = GetMin(aabb);
    Point max = GetMax(aabb);
    if (point.x < min.x || point.y < min.y || point.z < min.z) {
        return false;
    }
    if (point.x > max.x || point.y > max.y || point.z > max.z) {
        return false;
    }
    return true;
}

Point ClosestPoint(const AABB& aabb, const Point& point) {
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

bool PointInOBB(const Point& point, const OBB& obb) {
    vec3 dir = point - obb.position;
    // project point to local axes of the box.
    for (int i = 0; i < 3; ++i) {
        const float* orientation = &obb.orientation.asArray[i * 3];
        vec3 axis(orientation[0], orientation[1], orientation[2]);
        float distance = Dot(dir, axis);
        if (distance > obb.size.asArray[i]) {
            return false;
        }
        if (distance < -obb.size.asArray[i]) {
            return false;
        }
    }

    return true;
}

Point ClosestPoint(const OBB& obb, const Point& point) {
    Point result = obb.position;
    vec3 dir = point - obb.position;
    // project
    for (int i = 0; i < 3; ++i) {
        const float* orientation = &obb.orientation.asArray[i * 3];
        vec3 axis(orientation[0], orientation[1], orientation[2]);
        float distance = Dot(dir, axis);
        // clamp
        if (distance > obb.size.asArray[i]) {
            distance = obb.size.asArray[i];
        }
        if (distance < -obb.size.asArray[i]) {
            distance = -obb.size.asArray[i];
        }
        // move along axis by clamped distance
        result = result + (axis * distance);
    }

    return result;
}

bool PointOnPlane(const Point& point, const Plane& plane) {
    return CMP(Dot(point, plane.normal) - plane.distance, 0.0f);
}

Point ClosestPoint(const Plane& plane, const Point& point) {
    float dot = Dot(plane.normal, point);
    // distance from point to plane
    float distance = dot - plane.distance;
    // move point on plane.
    return point - plane.normal * distance;
}

Point ClosestPoint(const Line& line, const Point& point) {
    // project point on to line
    vec3 lVec = line.end - line.start;
    float t = Dot(point - line.start, lVec) / Dot(lVec, lVec);
    // clamp
    t = fmaxf(t, 0.0f);
    t = fminf(t, 1.0f);
    // evaluate parametric function.
    return line.start + lVec * t;
}

bool PointOnLine(const Point& point, const Line& line) {
    // find closest point on line and check if it is actually the point by comparing distance.
    Point closest = ClosestPoint(line, point);
    float distanceSq = MagnitudeSq(closest - point);
    return CMP(distanceSq, 0.0f);
}

bool PointOnRay(const Point& point, const Ray& ray) {
    if (point == ray.origin) {
        return true;
    }
    vec3 norm = point - ray.origin;
    Normalize(norm);
    float diff = Dot(norm, ray.direction);
    return CMP(diff, 1.0f);
}

Point ClosestPoint(const Ray& ray, const Point& point) {
    float t = Dot(point - ray.origin, ray.direction);
    t = fmaxf(t, 0.0f);
    return ray.origin + ray.direction * t;
}