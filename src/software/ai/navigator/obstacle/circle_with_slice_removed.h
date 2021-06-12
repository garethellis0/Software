#pragma once

#include <optional>

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/point.h"
#include "software/geom/triangle.h"

/**
 * This is sort of like a pizza with a single slice of pizza removed, but
 * the person cutting out the slice did a really bad job, so the "tip" of the pizza slice
 * can be anywhere from the center of the pizza to the outside edge.
 *
 * The slice can be thought of as being made of up a triangle and a circular segment
 * (https://en.wikipedia.org/wiki/Circular_segment). This is important to understand,
 * as the code and tests use this terminology.
 */
 // TODO: should this class name have the `Obstacle` suffix
class CircleWithSliceRemoved : public Obstacle
{
   public:
    CircleWithSliceRemoved() = delete;

    /**
     * Construct a CircleWithSliceRemoved
     *
     * @param center The center of the obstacle
     * @param radius The radius of the obstacle
     * @param slice_tip The point at the tip of the slice to remove. This must be within
     *                  the circle, and cannot be at the center of the circle.
     * @param slice_angle The angle of the slice to remove. Must be less then 180 deg.
     *
     * @throw invalid_argument if the slice tip is outside the circle OR at the center of
     *                         the circle
     * @throw invalid_argument if the angle of the slice is > 180 deg
     */
    CircleWithSliceRemoved(Point center, double radius, Point slice_tip,
                           Angle slice_angle);

    // TODO: jdoc
    // TODO: test this
    CircleWithSliceRemoved(Circle circle, Point slice_tip,
                           Angle slice_angle);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    std::string toString(void) const override;
    void accept(ObstacleVisitor& visitor) const override;

    friend bool operator==(const CircleWithSliceRemoved& lhs, const CircleWithSliceRemoved& rhs);

    // TODO: jdoc
    // TODO: test
    Point origin() const;

    // TODO: jdoc
    // TODO: test
    double radius() const;

    // TODO: jdoc
    // TODO: test
    Angle orientation() const;

    // TODO: jdoc
    // TODO: test
    Point sliceTip() const;

    // TODO: jdoc
    // TODO: test
    // TODO: need to provide some guarantees about ordering for these points
    //       (probably better to split into two funcions instead)
    std::pair<Point,Point> sliceCircleIntersectionPoints() const;

   private:
    /**
     * Get the smallest possible triangle that fully contains the slice
     * @return The smallest possible triangle that fully contains the slice
     */
    std::optional<Triangle> triangleContainingSlice() const;

    // TODO: jdoc
    Angle sliceDirection() const;

    // TODO: jdoc
    double sliceMinRadius() const;

    // TODO: jdoc
    double sliceMaxRadius() const;

    // TODO: jdoc
    bool pointInSliceFov(const Point& p) const;

    // TODO: comments for these members
    Circle circle;
    Point slice_tip;
    Angle slice_angle;
};

// TODO: test thi

bool operator==(const CircleWithSliceRemoved& lhs, const CircleWithSliceRemoved& rhs)
;
