#pragma once

#include "software/simulated_tests/validation/validation_function.h"
#include "software/world/world.h"

/**
 * Checks if the robot represented by robot_id has received the ball at
 * any point in the test
 * @param robot_id the ID of the robot in question, there must exist a robot
 * for the given robot_id
 * @param world_ptr the world pointer given by the simulator. Gets updated every tick
 * @param yield A coroutine push-type to call to yield control to the next routine
 *              (coroutines) with a message explaining why we returned.
 */
void robotReceivedBall(RobotId robot_id, std::shared_ptr<World> world_ptr,
                       ValidationCoroutine::push_type& yield);

/**
 * Check that the robot gets possession of the ball and moves it to the given position
 * without losing the ball.
 *
 * @param robot_id The id of the robot we expect to move the ball
 * @param target_ball_position The position we expect the robot to move the ball to
 * @param world_ptr The world in which we're performing the validation
 * @param yield A coroutine push-type to call to yield control to the next routine
 *              (coroutines) with a message explaining why we returned.
 */
void robotMovesBallToPositionWithoutLosingPossession(RobotId robot_id, Point target_ball_position, std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield);