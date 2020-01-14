// TODO: should all this be in the `BallInterception` tactic instead? Should limit scope?

#include "software/new_geom/point.h"
#include "software/world/ball.h"
#include "software/world/robot.h"
#include "software/world/field.h"

/**
 * Find the best position for the given robot to intercept the given ball
 *
 * In the case where the timestamp on the Robot and Ball are different, this function
 * will choose the larger timestamp and use the estimated state for the object with
 * the smaller timestamp.
 *
 * @param robot The robot that should intercept the ball
 * @param ball The ball the robot should intercept
 * @param field The field the intercept is to be performed on
 *
 * @return The state of the robot at the best interception
 */
std::optional<RobotState> findBestBallInterception(const Robot& robot, const Ball& ball,
                                                   const Field& field);