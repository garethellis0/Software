#include "software/ai/navigator/obstacle/circle_with_slice_removed.h"

#include <gtest/gtest.h>
#include <math.h>

#include "software/geom/algorithms/distance.h"

// NOTE: PLEASE READ THE CircleWithSliceRemoved CLASS JAVADOC. It explains terms used in
// the
//       comments in these tests.

TEST(CircleWithSliceRemovedTest, slice_tip_not_in_circle_throws_exception)
{
    // Test that a slice tip not within the circle throws an exception
    EXPECT_THROW(
        CircleWithSliceRemoved(Point(1, 1), 1, Point(5, 5), Angle::fromDegrees(45));
        , std::invalid_argument);
}

TEST(CircleWithSliceRemovedTest, slice_tip_at_circle_center_throws_exception)
{
    // Test that a slice tip not within the circle throws an exception
    EXPECT_THROW(
        CircleWithSliceRemoved(Point(1, 1), 1, Point(1, 1), Angle::fromDegrees(45));
        , std::invalid_argument);
}

TEST(CircleWithSliceRemovedTest, angle_greater_then_180_deg_throws_exception)
{
    // Test the a slice angle > 180 deg throws an exception
    EXPECT_THROW(
        CircleWithSliceRemoved(Point(1, 1), 1, Point(1.2, 1.2), Angle::fromDegrees(181));
        , std::invalid_argument);
}

TEST(CircleWithSliceRemovedTest, angle_eq_to_180_deg_does_not_throw_exception)
{
    // Test the a slice angle of 180 deg does not throw an exception
    EXPECT_NO_THROW(CircleWithSliceRemoved(Point(1, 1), 1, Point(1.2, 1.2),
                                           Angle::fromDegrees(180)););
}

TEST(CircleWithSliceRemovedTest, contains_point_in_circle_but_not_in_slice)
{
    // Test where the point is within the circle, but not within the slice,
    // and so *is* contained by the obstacle
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(30));

    EXPECT_TRUE(obstacle.contains(Point(1.2, 0.8)));
    EXPECT_TRUE(obstacle.contains(Point(0.8, 1.2)));
    EXPECT_TRUE(obstacle.contains(Point(0.8, 0.8)));
    EXPECT_TRUE(obstacle.contains(Point(1.0, 1.0)));
    EXPECT_TRUE(obstacle.contains(Point(1.2, 1.2)));
    EXPECT_TRUE(obstacle.contains(Point(1.5, 1.4)));
    EXPECT_TRUE(obstacle.contains(Point(1.4, 1.5)));
}

TEST(CircleWithSliceRemovedTest, contains_point_outside_circle)
{
    // Test where the point is outside the circle, and so is not contained in the obstacle
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(45));

    EXPECT_FALSE(obstacle.contains(Point(2.1, 1)));
    EXPECT_FALSE(obstacle.contains(Point(1, 2.1)));
    EXPECT_FALSE(obstacle.contains(Point(2.1, 2.1)));
    EXPECT_FALSE(obstacle.contains(Point(0, 0)));
    EXPECT_FALSE(obstacle.contains(Point(0, 0.9)));
    EXPECT_FALSE(obstacle.contains(Point(0.9, 0)));
}

TEST(CircleWithSliceRemovedTest, contains_point_inside_triangle_of_slice)
{
    // Test where the point is within the circle, and within the triangle of the slice,
    // and so is not inside the obstacle
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_FALSE(obstacle.contains(Point(1.51, 1.51)));
    EXPECT_FALSE(obstacle.contains(Point(1.8, 1.6)));
    EXPECT_FALSE(obstacle.contains(Point(1.6, 1.8)));
}

TEST(CircleWithSliceRemovedTest, contains_point_inside_circular_segment_of_slice)
{
    // Test where the point is within the circle, and within the circular segment of the
    // slice, and so is not inside the obstacle
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1),
                                    Angle::fromDegrees(45));

    EXPECT_FALSE(obstacle.contains(Point(2.0, 1.999)));
    EXPECT_FALSE(obstacle.contains(Point(1.9, 1)));
    EXPECT_FALSE(obstacle.contains(Point(2.0, 1.1)));
    EXPECT_FALSE(obstacle.contains(Point(2.0, 0.9)));
}

TEST(CircleWithSliceRemovedTest, distance_to_point_in_circle_but_not_in_slice)
{
    // Test where the point is within the circle, but not within the slice,
    // and so *is* contained by the obstacle

    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(30));

    EXPECT_EQ(0, obstacle.distance(Point(1.2, 0.8)));
    EXPECT_EQ(0, obstacle.distance(Point(0.8, 1.2)));
    EXPECT_EQ(0, obstacle.distance(Point(0.8, 0.8)));
    EXPECT_EQ(0, obstacle.distance(Point(1.0, 1.0)));
    EXPECT_EQ(0, obstacle.distance(Point(1.2, 1.2)));
    EXPECT_EQ(0, obstacle.distance(Point(1.5, 1.4)));
    EXPECT_EQ(0, obstacle.distance(Point(1.4, 1.5)));
}

TEST(CircleWithSliceRemovedTest, distance_to_point_outside_circle_and_not_in_slice_fov)
{
    // Test where the point is outside the circle, and not within the FOV of the slice
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1),
                                    Angle::fromDegrees(45));

    // On boundary
    EXPECT_NEAR(0.0, obstacle.distance(Point(2.0, 1.0)), 1e-7);
    EXPECT_NEAR(0.0, obstacle.distance(Point(1.0, 2.0)), 1e-7);
    EXPECT_NEAR(0.0, obstacle.distance(Point(1.0, 0.0)), 1e-7);
    EXPECT_NEAR(0.0, obstacle.distance(Point(0.0, 1.0)), 1e-7);

    // Off boundary
    EXPECT_NEAR(1.0, obstacle.distance(Point(1.0, 3.0)), 1e-7);
    EXPECT_NEAR(1.0, obstacle.distance(Point(1.0, -1.0)), 1e-7);
    EXPECT_NEAR(1.0, obstacle.distance(Point(-1.0, 1.0)), 1e-7);
    EXPECT_NEAR(std::sqrt(2.0) - 1, obstacle.distance(Point(0, 0)), 1e-7);
}

// TODO: actually enable this test, it's a possible source of errors, but it's late
//       and this is a particularly tricky case to setup cases for
// TEST(CircleWithSliceRemovedTest, distance_to_point_outside_circle_and_within_slice_fov)
// {
//    // Test where the point is outside the circle, and within the FOV of the slice
//
//    CircleWithSliceRemoved obstacle(Point(1,1), 1, Point(1.5,1.5),
//    Angle::fromDegrees(90));
//    // TODO
//}

TEST(CircleWithSliceRemovedTest, distance_to_point_inside_slice)
{
    // Test where the point is within within the triangle of the slice

    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_NEAR(0.2, obstacle.distance(Point(1.7, 1.8)), 1e-7);
    EXPECT_NEAR(0.2, obstacle.distance(Point(1.8, 1.7)), 1e-7);
    EXPECT_NEAR(0.3, obstacle.distance(Point(1.8, 1.8)), 1e-7);
}

TEST(CircleWithSliceRemovedTest, intersects_segment_fully_outside_circle)
{
    // Segment fully outside the circle should not intersect
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_FALSE(obstacle.intersects(Segment(Point(2.1, 1.0), Point(2.1, 1.0))));
    EXPECT_FALSE(obstacle.intersects(Segment(Point(1.0, 2.1), Point(1.0, 2.1))));
    EXPECT_FALSE(obstacle.intersects(Segment(Point(1.0, 2.1), Point(1.0, 2.1))));
    EXPECT_FALSE(obstacle.intersects(Segment(Point(1.0, 3.1), Point(3.1, 1.0))));
}

TEST(CircleWithSliceRemovedTest, intersects_segment_fully_within_triangle)
{
    // Segment fully within the triangle should not intercept
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_FALSE(obstacle.intersects(Segment(Point(1.51, 1.57), Point(1.6, 1.54))));
}

TEST(CircleWithSliceRemovedTest, intersects_segment_fully_within_circular_segment)
{
    // Segment fully within the segment should not intercept
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_FALSE(obstacle.intersects(Segment(Point(1.991, 1.992), Point(1.993, 1.991))));
}

TEST(CircleWithSliceRemovedTest, intersects_segment_overlaps_slice_but_not_obstacle)
{
    // Segment overlapping triangle and circular segment, but not obstacle
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_FALSE(obstacle.intersects(Segment(Point(1.6, 1.7), Point(5.0, 5.0))));
}

TEST(CircleWithSliceRemovedTest, intersects_segment_fully_within_circle_and_not_slice)
{
    // Segment fully within the circle, and does not overlap at all with the slice
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_TRUE(obstacle.intersects(Segment(Point(0.5, 0.6), Point(1.0, 0.3))));
}

TEST(CircleWithSliceRemovedTest,
     intersects_segment_overlaps_obstacle_with_one_end_in_triangle)
{
    // Segment with one end in the triangle, and the other end in the obstacle
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_TRUE(obstacle.intersects(Segment(Point(1.6, 1.6), Point(1.0, 0.3))));
    EXPECT_TRUE(obstacle.intersects(Segment(Point(1.6, 1.6), Point(0.3, 1.0))));
}

TEST(CircleWithSliceRemovedTest,
     intersects_segment_overlaps_triangle_but_both_ends_in_obstacle)
{
    // Segment overlapping the triangle, but with both of it's ends in the obstacle
    CircleWithSliceRemoved obstacle(Point(1, 1), 1, Point(1.5, 1.5),
                                    Angle::fromDegrees(90));

    EXPECT_TRUE(obstacle.intersects(Segment(Point(1.7, 1.4), Point(1.7, 1.4))));
}

// TODO: test toString
// TODO: test accept
