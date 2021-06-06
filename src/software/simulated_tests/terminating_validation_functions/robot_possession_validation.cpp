#include "software/simulated_tests/terminating_validation_functions/robot_possession_validation.h"

#include <gtest/gtest.h>

#include "software/logger/logger.h"

void robotReceivedBall(RobotId robot_id, std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield)
{
    auto ball_near_dribbler = [robot_id](std::shared_ptr<World> world_ptr) {
        std::optional<Robot> robot_optional =
            world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot_optional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " + std::to_string(robot_id);
        }

        Robot robot         = robot_optional.value();
        Point ball_position = world_ptr->ball().position();
        return robot.isNearDribbler(ball_position, 0.05);
    };

    while (!ball_near_dribbler(world_ptr))
    {
        yield("Robot with ID " + std::to_string(robot_id) + " has not received the ball");
    }
}

void robotMovesBallToPositionWithoutLosingPossession(RobotId robot_id,Point target_ball_position, std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
    static constexpr double BALL_AT_POSITION_TOLERANCE = 0.05;
    bool got_possession_previously = false;
    Timestamp last_possession_time;
    const auto& ball           = world_ptr->ball();
    while ((ball.position() - target_ball_position).length() > BALL_AT_POSITION_TOLERANCE)
    {
        const auto& robot_optional = world_ptr->friendlyTeam().getRobotById(robot_id);
        if (!robot_optional.has_value())
        {
            LOG(FATAL) << "There is no robot with ID: " << robot_id;
        }
        bool has_possession = robot_optional->isNearDribbler(ball.position(), 0.05);
        const auto& current_time = world_ptr->getMostRecentTimestamp();

        if (has_possession)
        {
            last_possession_time = current_time;
        }
        if (!got_possession_previously)
        {
            got_possession_previously = has_possession;
        }
        else
        {
            // We got possession, make sure we retained it.
            ASSERT_LE(current_time - last_possession_time, Duration::fromSeconds(0.5))
                    << "We lost possession at time " << current_time;
        }
        std::stringstream ss;
        ss << "Ball at " << ball.position() << ", but expected position is  " << target_ball_position;
        yield(ss.str());
    }

    // Make sure that we got possession at least once
    ASSERT_TRUE(got_possession_previously) << "Ball got to target position, but robot with ID " << robot_id << " never gained possession";
}
