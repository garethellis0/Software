#include "software/ai/navigator/obstacle/circle_with_slice_removed.h"

#include <math.h>

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"

CircleWithSliceRemoved::CircleWithSliceRemoved(Point center, double radius,
                                               Point slice_tip, Angle slice_angle)
    : CircleWithSliceRemoved(Circle(center, radius), slice_tip, slice_angle)
{
}

CircleWithSliceRemoved::CircleWithSliceRemoved(Circle circle, Point slice_tip,
        Angle slice_angle)
        : circle(circle), slice_tip(slice_tip), slice_angle(slice_angle)
{
    if (!::contains(circle, slice_tip))
    {
        std::stringstream ss;
        ss << "Slice tip" << slice_tip << " is not contained within circle " << circle;
        throw std::invalid_argument(ss.str());
    }
    if (circle.origin() == slice_tip)
    {
        std::stringstream ss;
        ss << "Slice tip" << slice_tip << " is at circle origin " << circle.origin();
        throw std::invalid_argument(ss.str());
    }
    const auto deg_180 = Angle::fromDegrees(180);
    if (slice_angle > deg_180)
    {
        std::stringstream ss;
        ss << "Slice angle " << slice_angle << " > " << deg_180;
        throw std::invalid_argument(ss.str());
    }
}

bool CircleWithSliceRemoved::contains(const Point& p) const
{
    bool within_circle         = ::contains(circle, p);
    bool within_slice_triangle = false;
    auto optional_triangle_containing_slice = triangleContainingSlice();
    if(optional_triangle_containing_slice) {
        within_slice_triangle = ::contains(optional_triangle_containing_slice.value(), p);
    }

    return within_circle && !within_slice_triangle;
}
double CircleWithSliceRemoved::distance(const Point& p) const
{
    if (contains(p))
    {
        return 0.0;
    }

    if (pointInSliceFov(p))
    {
        // Construct segments for the "sides" of the slice
        const double origin_to_slice_tip = (slice_tip - circle.origin()).length();
        const double theta = (Angle::half() - slice_angle / 2.0).toRadians();
        double segment_length =
            std::sqrt(
                std::pow(origin_to_slice_tip, 2.0) * std::pow(std::cos(theta), 2.0) -
                std::pow(origin_to_slice_tip, 2.0) + std::pow(circle.radius(), 2.0)) +
            origin_to_slice_tip * std::cos(theta);
        Segment cw_slice_segment(
            slice_tip,
            slice_tip + Vector::createFromAngle(sliceDirection() - slice_angle / 2.0)
                            .normalize(segment_length));
        Segment ccw_slice_segment(
            slice_tip,
            slice_tip + Vector::createFromAngle(sliceDirection() + slice_angle / 2.0)
                            .normalize(segment_length));
        return std::min(::distance(p, cw_slice_segment),
                        ::distance(p, ccw_slice_segment));
    }
    else
    {
        // The closest point is going to be on the circle
        return ::distance(p, circle);
    }
}

bool CircleWithSliceRemoved::intersects(const Segment& segment) const
{
    // If the segment intersects the circle and does not lie entirely in the
    // slice, it's intersecting
    const bool intersects_with_circle        = ::intersects(circle, segment);
    const bool both_points_in_slice_fov =
        pointInSliceFov(segment.getStart()) && pointInSliceFov(segment.getEnd());

    return intersects_with_circle && !both_points_in_slice_fov;
}
std::string CircleWithSliceRemoved::toString(void) const
{  // TODO
    return "";
}
void CircleWithSliceRemoved::accept(ObstacleVisitor& visitor) const
{
    visitor.visit(*this);
}

Point CircleWithSliceRemoved::origin() const {
    return circle.origin();
}

double CircleWithSliceRemoved::radius() const {
    return circle.radius();
}

Angle CircleWithSliceRemoved::orientation() const {
    return (slice_tip - origin()).orientation();
}

Point CircleWithSliceRemoved::sliceTip() const {
    return slice_tip;
}

std::pair<Point,Point> CircleWithSliceRemoved::sliceCircleIntersectionPoints() const {
    // TODO: we can probably just re-use this in `distance`, instead of nasty copy-paste

    const double origin_to_slice_tip = (slice_tip - circle.origin()).length();
    const double theta = (Angle::half() - slice_angle / 2.0).toRadians();
    double segment_length =
            std::sqrt(
                    std::pow(origin_to_slice_tip, 2.0) * std::pow(std::cos(theta), 2.0) -
                    std::pow(origin_to_slice_tip, 2.0) + std::pow(circle.radius(), 2.0)) +
            origin_to_slice_tip * std::cos(theta);
    return
    {
            slice_tip + Vector::createFromAngle(sliceDirection() - slice_angle / 2.0)
                    .normalize(segment_length),
            slice_tip + Vector::createFromAngle(sliceDirection() + slice_angle / 2.0)
                    .normalize(segment_length),
    };
}

std::optional<Triangle> CircleWithSliceRemoved::triangleContainingSlice() const
{
    // TODO: ascii diagram

    // Vector from the slice tip to the base of the triangle
    Vector triangle_center_vector =
        Vector::createFromAngle(sliceDirection()).normalize(sliceMinRadius());
    Point triangle_base_center = slice_tip + triangle_center_vector;

    // Rays along the edge of the triangle in the cw/ccw direction from the center
    Ray triangle_cw_ray  = Ray(slice_tip, sliceDirection() - slice_angle / 2.0);
    Ray triangle_ccw_ray = Ray(slice_tip, sliceDirection() + slice_angle / 2.0);

    // Find the other two points of the triangle (besides the slice tip) by
    // shooting a ray from the center outwards along the base of the triangle
    Ray base_ray_cw  = Ray(triangle_base_center, sliceDirection() - Angle::quarter());
    Ray base_ray_ccw = Ray(triangle_base_center, sliceDirection() + Angle::quarter());

    std::optional<Point> cw_point_optional = ::intersection(triangle_cw_ray, base_ray_cw);
    std::optional<Point> ccw_point_optional =
        ::intersection(triangle_ccw_ray, base_ray_ccw);

    if (!cw_point_optional || !ccw_point_optional)
    {
        return std::nullopt;
    }

    return Triangle(slice_tip, cw_point_optional.value(), ccw_point_optional.value());
}

Angle CircleWithSliceRemoved::sliceDirection() const
{
    return (slice_tip - circle.origin()).orientation();
}

double CircleWithSliceRemoved::sliceMinRadius() const
{
    return circle.radius() - ::distance(slice_tip, circle.origin());
}

double CircleWithSliceRemoved::sliceMaxRadius() const
{
    return circle.radius() - ::distance(slice_tip, circle.origin());
}

bool CircleWithSliceRemoved::pointInSliceFov(const Point& p) const
{
    const Angle slice_cw_angle     = sliceDirection() - slice_angle / 2.0;
    const Angle slice_ccw_angle    = sliceDirection() + slice_angle / 2.0;
    const Angle slice_tip_to_point = (p - slice_tip).orientation();

    return (slice_tip_to_point.minDiff(slice_cw_angle) < slice_angle) &&
           (slice_tip_to_point.minDiff(slice_ccw_angle) < slice_angle);
}

bool operator==(const CircleWithSliceRemoved& lhs, const CircleWithSliceRemoved& rhs) {
    return (lhs.circle == rhs.circle) && (lhs.slice_tip == rhs.slice_tip) && (lhs.slice_angle == rhs.slice_angle);
}
