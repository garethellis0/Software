#include "software/ai/evaluation/ball_interception.h"

#include <algorithm>

#include "shared/constants.h"
#include "software/ai/evaluation/pass.h"
#include "software/util/optimization/gradient_descent_optimizer.h"

std::optional<RobotState> findBestBallInterception(const Robot& robot, const Ball& ball,
                                                   const Field& field)
{
    // TODO: need to checks for ball v. robot timestamp

    // TODO: is this comment actually descriptive to anyone reading it without context?
    // We sample times in the future to test, more densely sampling in the near future
    // as it's easy for gradient descent to miss local optima there.
    std::vector<Duration> initial_time_offsets_to_test;
    static const long num_positions_to_sample_in_first_second = 30;
    for (long i = 0; i < num_positions_to_sample_in_first_second; i++)
    {
        Duration t_offset = Duration::fromSeconds(
            static_cast<double>(i) /
            static_cast<double>(num_positions_to_sample_in_first_second));
        initial_time_offsets_to_test.emplace_back(t_offset);
    }
    static const long num_positions_to_sample_after_first_second_until_10th_second = 30;
    for (long i = 0; i < num_positions_to_sample_after_first_second_until_10th_second;
         i++)
    {
        Duration t_offset = Duration::fromSeconds(
            1 + 9 * static_cast<double>(i) /
                    static_cast<double>(
                        num_positions_to_sample_after_first_second_until_10th_second));
        initial_time_offsets_to_test.emplace_back(t_offset);
    }

    // The cost function, where a lower cost indicates better timestamp for interception
    auto cost_function = [&robot, &ball, &field](const Duration& ball_time_to_position) {
        // TODO: expose this parameter?
        static const Duration safety_margin = Duration::fromSeconds(0.1);

        // TODO: we're not accounting for the fact that we want the _front_ of the robot
        //       right in front of the ball, *NOT* the center

        const Point ball_position =
            ball.estimatePositionAtFutureTime(ball_time_to_position);
        const Point estimated_ball_position =
            ball.estimatePositionAtFutureTime(ball_time_to_position);
        const Vector estimated_ball_velocity =
            ball.estimateVelocityAtFutureTime(ball_time_to_position);

        // TODO: should use `getTimeToOrientationToRobot` to check that the robot can
        //       turn in time as well, take the max of those

        // The robot needs to be offset from the ball so that the ball is in front of it
        const Point required_robot_position =
            estimated_ball_position +
            estimated_ball_velocity.normalize(DIST_TO_FRONT_OF_ROBOT_METERS);
        const Duration robot_time_to_position =
            AI::Evaluation::getTimeToPositionForRobot(
                robot, required_robot_position, ROBOT_MAX_SPEED_METERS_PER_SECOND,
                ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED) +
            safety_margin;

        const Duration time_diff_at_interception_point =
            ball_time_to_position - robot_time_to_position;


        // If the time diff is positive then the ball got to the position before the
        // robot, if it's negative then the robot got to the position before the ball, and
        // wasted time waiting for the ball when it could have moved closer to the ball.
        // We want to aim to get to the position just before the ball does.
        // TODO: use log-sum-exp approximation to absolute value here?
        return abs(time_diff_at_interception_point.getSeconds());
    };

    // Find the best time offset of all those sampled
    auto best_initial_t_offset_and_cost = std::make_pair<const Duration, double>(
        Duration::fromSeconds(100), std::numeric_limits<double>::max());
    for (Duration t_offset : initial_time_offsets_to_test)
    {
        double cost = cost_function(t_offset);
        if (cost < best_initial_t_offset_and_cost.second)
        {
            best_initial_t_offset_and_cost.first = t_offset;
            best_initial_t_offset_and_cost.second = cost;
        }
    }

    // Assuming the best time offset is in the same valley containing the global minima,
    // use gradient descent to refine the offset
    Util::GradientDescentOptimizer<1> optimizer;
    optimizer.minimize(
        [&cost_function](std::array<double, 1> params) {
            return cost_function(Duration::fromSeconds(std::get<0>(params)));
        },
        {best_initial_t_offset_and_cost.first.getSeconds()}, 100);

    const Duration best_time_offset = best_initial_t_offset_and_cost.first;

    const Point estimated_ball_position =
        ball.estimatePositionAtFutureTime(best_time_offset);
    const Vector estimated_ball_velocity =
        ball.estimateVelocityAtFutureTime(best_time_offset);

    // The center of the robot lie on the path of the ball, and should be offset by
    // distance from the center to the front of the robot
    const Point robot_position =
        estimated_ball_position +
        estimated_ball_velocity.normalize(DIST_TO_FRONT_OF_ROBOT_METERS);
    const Angle robot_orientation =
        (robot_position - estimated_ball_position).orientation();
    const Timestamp robot_timestamp = robot.lastUpdateTimestamp() + best_time_offset;
    return RobotState(robot_position, Vector(0, 0), robot_orientation,
                      AngularVelocity::zero(), robot_timestamp);
}
