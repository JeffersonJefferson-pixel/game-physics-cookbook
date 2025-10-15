#include "Geometry2D.h"
#include "matrices.h"
#include <cmath>
#include <cfloat>

#define CMP(x, y) \
    (fabsf((x) - (y)) <= FLT_EPSILON * fmaxf(1.0f, fmaxf(fabsf(x), fabsf(y))))

float Length(const Line2D& line) {
    return Magnitude(line.end - line.start);
}

float LengthSq(const Line2D& line) {
    return MagnitudeSq(line.end - line.start);
}

vec2 GetMin(const Rectangle2D& rect) {
    vec2 p1 = rect.origin;
    vec2 p2 = rect.origin + rect.size;

    return vec2(fminf(p1.x, p2.x), fminf(p1.y, p2.y));
}

vec2 GetMax(const Rectangle2D& rect) {
    vec2 p1 = rect.origin;
    vec2 p2 = rect.origin + rect.size;

    return vec2(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y));
}

Rectangle2D FromMinMax(const vec2& min, const vec2& max) {
    return Rectangle2D(min, max - min);
}

bool PointOnLine(const Point2D& p, const Line2D& line) {
    // get slope-intercept form (y = mx + b).
    float dy = line.end.y - line.start.y;
    float dx = line.end.x - line.start.x;
    float M = dy / dx;
    float B = line.start.y - M * line.start.x;
    return CMP(p.y, M * p.x + B);
}

bool PointInCircle(const Point2D& point, const Circle& c) {
    // line between the circle center and the point.
    Line2D line(point, c.position);
    // compare distance of the line and the circle radius.
    if (LengthSq(line) < c.radius * c.radius) {
        return true;
    }
    return false;
}

bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle) {
    // clip point by rectangle minimum and maximum point.
    vec2 min = GetMin(rectangle);
    vec2 max = GetMax(rectangle);

    return min.x <= point.x && min.y <= point.y && point.x <= max.x && point.y <= max.y;
}

bool PointInOrientedRectangle(const Point2D& point, const OrientedRectangle& rectangle) {
    // translate point into local space of rectangle
    vec2 rotVector = point - rectangle.position;
    // rotate backward
    float theta = -DEG2RAD(rectangle.rotation);
    float zRotation2x2[] = {
        cosf(theta), sinf(theta),
        -sinf(theta), cosf(theta)
    };
    Multiply(rotVector.asArray, vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
    Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
    // shift
    vec2 localPoint = rotVector + rectangle.halfExtents;
    return PointInRectangle(localPoint, localRectangle);
}

bool LineCircle(const Line2D& l, const Circle& c) {
    vec2 ab = l.end - l.start;
    float t = Dot(c.position - l.start, ab) / Dot(ab, ab);
    if (t < 0.0f || t > 1.0f) {
        return false;
    }
    Point2D closestPoint = l.start + ab * t;

    Line2D circleToClosest(c.position, closestPoint);
    return LengthSq(circleToClosest) < c.radius * c.radius;
}

bool LineRectangle(const Line2D& l, const Rectangle2D& r) {
    // endpoints inside rectangle.
    if (PointInRectangle(l.start, r) || PointInRectangle(l.end, r)) {
        return true;
    }

    // parametric distance where line crosses rectangle.
    vec2 norm = Normalized(l.end - l.start);
    norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
    norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
    vec2 min = (GetMin(r) - l.start) * norm;
    vec2 max = (GetMax(r) - l.start) * norm;

    // overlap of t interval
    float tmin = fmaxf(fminf(min.x, max.x), fminf(min.y, max.y));
    float tmax = fminf(fmaxf(min.x, max.x), fmaxf(min.y, max.y));
    if (tmax < 0 || tmin > tmax) {
        return false;
    }
    float t = (tmin < 0.0f) ? tmax : tmin;
    return t > 0.0f && t * t < LengthSq(l);
}

bool LineOrientedRectangle(const Line2D& line, const OrientedRectangle& rectangle) {
    // transofmr line to local space of rectangle
    float theta = -DEG2RAD(rectangle.rotation);
    float zRotation2x2[] = {
        cosf(theta), sinf(theta),
        -sinf(theta), cosf(theta)
    };
    Line2D localLine;

    vec2 rotVector = line.start - rectangle.position;
    Multiply(rotVector.asArray, vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
    localLine.end = rotVector + rectangle.halfExtents;

    Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
    return LineRectangle(localLine, localRectangle);
}