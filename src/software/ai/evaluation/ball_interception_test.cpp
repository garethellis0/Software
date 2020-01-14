#include "software/ai/evaluation/ball_interception.h"

#include <gtest/gtest.h>
#include <tuple>

#include "shared/constants.h"
#include "software/test_util/test_util.h"

/**
 * (Test Name, Ball Speed, Robot Position, Robot Position Offset At Best Intercept,
 * Tolerance)
 *
 * TODO: ASCII art here?
 *
 * Test Name: The name of the test case, to make figuring out what test failed easier
 * Ball Speed: Initial speed of the ball
 * Robot Position: Initial position of the robot (relative to the ball)
 * Robot Position Offset At Best Intercept: The offset from the ball start position along
 *                                          the ball velocity vector that the robot should
 *                                          be at to intercept the ball. If std::nullopt,
 *                                          no intercept expected.
 * Tolerance: the tolerance, in meters, for the robot position at the intercept. Note that
 *            this is just along the direction of travel for the ball, it's expected that
 *            the robot will lie on the ball trajectory, not off to the side.
 */
using BallInterceptionTestParams =
    std::tuple<std::string, double, Point, std::optional<double>, double>;

class BallInterceptionTest : public ::testing::TestWithParam<BallInterceptionTestParams>
{
};

// TODO: should have a few parameterized tests where we vary the ball position/direction
//       but maintain the same relative robot positions to ensure the intercept is
//       only affected by the relative positions of the robot/ball, not absolute

// TODO: better name for this parameterized test
// TODO: jdoc for this parameterized test, explain that ball is always at (0,0), robot
TEST_P(BallInterceptionTest, test_intercept_positions)
{
    double ball_speed                                        = std::get<1>(GetParam());
    Point robot_position                                     = std::get<2>(GetParam());
    std::optional<double> robot_offset_at_expected_intercept = std::get<3>(GetParam());
    double intercept_position_tolerance                      = std::get<4>(GetParam());

    Ball ball({0, 0}, {ball_speed, 0}, Timestamp::fromSeconds(0));
    Robot robot(0, robot_position, {0, 0}, Angle::fromDegrees(0),
                AngularVelocity::fromDegrees(0), Timestamp::fromSeconds(0));
    Field field = ::Test::TestUtil::createSSLDivBField();

    std::optional<RobotState> robot_state_at_best_intercept_found =
        findBestBallInterception(robot, ball, field);

    if (robot_offset_at_expected_intercept.has_value())
    {
        // TODO: ASCII diagram for what we're expecting here?

        Point expected_robot_position_at_intercept = {
            robot_offset_at_expected_intercept.value(), 0};


        ASSERT_TRUE(robot_state_at_best_intercept_found.has_value());
        RobotState robot_state_at_best_intercept =
            robot_state_at_best_intercept_found.value();

        // Robot state should be in the future, relative to the initial robot
        EXPECT_GE(robot_state_at_best_intercept.timestamp(), robot.lastUpdateTimestamp());

        // Robot should be near the expected position, within tolerance
        double robot_dist_from_expected_position =
            (robot_state_at_best_intercept.position() -
             expected_robot_position_at_intercept)
                .length();
        EXPECT_LE(robot_dist_from_expected_position, intercept_position_tolerance);

        // Robot should be in front of the the ball and facing it along the x axis
        EXPECT_EQ(Angle::half(), robot_state_at_best_intercept.orientation());

        // Robot should not be moving
        // NOTE: this could change with future implementations
        EXPECT_EQ(Vector(0, 0), robot_state_at_best_intercept.velocity());
        EXPECT_EQ(AngularVelocity::zero(),
                  robot_state_at_best_intercept.angularVelocity());

        // Ball should not have passed the front of the robot at the timestamp on
        // the returned robot state
        Point predicted_ball_position = ball.estimatePositionAtFutureTime(
            robot_state_at_best_intercept.timestamp() - robot.lastUpdateTimestamp());
        double robot_front_x_position =
            robot_state_at_best_intercept.position().x() - DIST_TO_FRONT_OF_ROBOT_METERS;
        EXPECT_LE(predicted_ball_position.x(), robot_front_x_position);
    }
    else
    {
        // No intercept expected
        EXPECT_EQ(std::nullopt, robot_state_at_best_intercept_found);
    }
}

INSTANTIATE_TEST_CASE_P(
    All, BallInterceptionTest,
    ::testing::Values(
        std::make_tuple("robot_directly_in_ball_path_and_close_to_ball", 1.0,
                        Point(DIST_TO_FRONT_OF_ROBOT_METERS / 2 + 0.01, 0),
                        DIST_TO_FRONT_OF_ROBOT_METERS / 2 + 0.01, 0.01),
        std::make_tuple("robot_directly_in_ball_path_and_far_from_ball", 1.0,
                        Point(1.0, 0), 0.7, 0.3),
        std::make_tuple("robot_just_off_ball_path_and_close_to_ball", 1.0,
                        Point(DIST_TO_FRONT_OF_ROBOT_METERS / 2 + 0.1, 0.1), 0.05, 0.05),
        std::make_tuple("robot_just_off_ball_path_and_far_from_ball", 1.0, Point(1.0, 0.1),
                        0.7, 0.3),
        std::make_tuple("robot_to_one_side_of_ball", 1.0, Point(0.0, 0.2),
                        0.8, 0.5),
        std::make_tuple("ball_moving_too_fast_to_intercept", 8.0, Point(0.0, 0.2),
                        std::nullopt, 0.0)
                        ),
    [](const ::testing::TestParamInfo<BallInterceptionTest::ParamType>& test_params) {
        return std::get<0>(test_params.param);
    });

