#ifndef _H_2D_GEOMETRY
#define _H_2D_GEOMETRY

#include "vectors.h"

typedef vec2 Point2D;

// line segment.
typedef struct Line2D
{
    Point2D start;
    Point2D end;

    inline Line2D() {}
    inline Line2D(const Point2D &s, const Point2D &e) : start(s), end(e) {}
} Line2D;

// length of line segment.
float Length(const Line2D &line);
float LengthSq(const Line2D &line);

// circle
typedef struct Circle
{
    Point2D position;
    float radius;

    inline Circle() : radius(1.0f) {}
    inline Circle(const Point2D &p, float r) : position(p), radius(r) {}
} Circle;

// rectangle
typedef struct Rectangle2D
{
    Point2D origin;
    vec2 size;

    inline Rectangle2D() : size(1, 1) {}
    inline Rectangle2D(const Point2D &o, const vec2 &s) : origin(o), size(s) {}
} Rectangle2D;

// get minimum and maximum point of a rectangle.
vec2 GetMin(const Rectangle2D &rect);
vec2 GetMax(const Rectangle2D &rect);

Rectangle2D FromMinMax(const vec2 &min, const vec2 &max);

// oriented rectangle
typedef struct OrientedRectangle
{
    Point2D position;
    vec2 halfExtents;
    float rotation;

    // create unit rectangle at origin with no rotation.
    inline OrientedRectangle() : halfExtents(1.0f, 1.0f), rotation(0.0f) {}
    // create rectangle given center point and half extents.
    inline OrientedRectangle(const Point2D& p, const vec2& e) : position(p), halfExtents(e), rotation(0.0f) {}
    // create oriented rectangle given center point, half extents and rotation in degree.
    inline OrientedRectangle(const Point2D& pos, const vec2& ext, float rot) : position(pos), halfExtents(ext), rotation(rot) {}
} OrientedRectangle;

// check if point is on a line.
bool PointOnLine(const Point2D& point, const Line2D& line);
// check if point is in a circle.
bool PointInCircle(const Point2D& point, const Circle& c);
// check if point is in a rectangle.
bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle);
bool PointInOrientedRectangle(const Point2D& point, const OrientedRectangle& reectangle);

// line intersection

bool LineCircle(const Line2D& line, const Circle & circle);

// 2D collision

// circle-circle collision
bool CircleCircle(const Circle& c1, const Circle& c2);
// circle-rectange collision
bool CircleRectangle(const Circle& cirlce, const Rectangle2D& rect);
// circle-oriented-rectangle collision
bool CircleOrientedRectangle(const Circle& circle, const OrientedRectangle& rect);
// rectangle-rectangle collision
bool RectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2);
#endif