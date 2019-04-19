/**
 * Implementation of the ReceiverTactic
 */
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"
#include "ai/hl/stp/action/move_action.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"

using namespace AI::Passing;
using namespace Evaluation;

ReceiverTactic::ReceiverTactic(Pass pass, const Ball& ball, bool loop_forever)
    : pass(std::move(pass)), ball(ball), Tactic(loop_forever)
{
}

std::string ReceiverTactic::getName() const
{
    return "Receiver Tactic";
}

void ReceiverTactic::updateParams(const Pass& updated_pass, const Ball& updated_ball)
{
    this->ball = updated_ball;
    this->pass = updated_pass;
}

double ReceiverTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the pass receive position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - pass.receiverPoint()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

std::unique_ptr<Intent> ReceiverTactic::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    // First get to the receiver position
    MoveAction move_action = MoveAction();
    do
    {
        // We want the robot to move to the receiving position for the shot and also
        // rotate to the correct orientation to face where the pass is coming from
        yield(move_action.updateStateAndGetNextIntent(
                *robot, pass.receiverPoint(), pass.receiverOrientation(), 0));
        // TODO: MoveAction latches on done, which is gonna break some stuff here
    } while (!move_action.done());

    // Wait until the pass starts
    while(ball.lastUpdateTimestamp() < pass.startTime()){
        yield(move_action.updateStateAndGetNextIntent(
                *robot, pass.receiverPoint(), pass.receiverOrientation(), 0));
    }

    // Check if we can shoot on the enemy goal from the receiver position
    std::optional<std::pair<Point, Angle>> best_shot_opt =
            calcBestShotOnEnemyGoal(field, friendly_team, enemy_team, robot);

    // If we have a shot with a sufficiently large open angle, then we should take it
    if (best_shot_opt && best_shot_opt->second > MIN_SHOT_OPEN_ANGLE_DEGREES){
        auto [best_shot_target, _] = *best_shot_opt;

        // TODO: When to end this?
        do {
            // Figure out the closest point on the balls trajectory to the robot
            Point closest_ball_pos = closestPointOnLine(
                    robot->position(), ball.position(),
                    ball.estimatePositionAtFutureTime(Duration::fromSeconds(0.1)));
            Ray shot(closest_ball_pos, best_shot_opt - closest_ball_pos);

            // Determine the best position to be in
            Angle ideal_orientation = getOneTimeShotDirection(shot, ball);
            Vector ideal_orientation_vec = Vector(ideal_orientation.cos(), ideal_orientation.sin());

            // The best position is determined such that the robot stays in the ideal
            // orientation, but moves forwards/backwards so that the ball hits the kicker,
            // rather then the center of the robot
            Point ideal_position = closest_ball_pos -
                    ideal_orientation_vec.norm(DIST_TO_FRONT_OF_ROBOT_METERS);

            yield(move_action.updateStateAndGetNextIntent(
                    *robot, pass.receiverPoint(), pass.receiverOrientation(), 0, false, true));
        } while(true);
    }
    // If we can't shoot on the enemy goal, just try to receive the pass as cleanly as
    // possible
    else {
        // TODO: When to end this?
        do {
            Point ball_receive_pos = closestPointOnLine(
                    robot->position(), ball.position(),
                    ball.estimatePositionAtFutureTime(Duration::fromSeconds(0.1)));
            Angle ball_receive_orientation = (robot->position() - ball.position()).orientation();

            yield(move_action.updateStateAndGetNextIntent(
                    *robot, ball_receive_pos, ball_receive_orientation, 0, true, false));
        } while(true);
    }
}

Angle ReceiverTactic::getOneTimeShotDirection(const Ray &shot, const Ball &ball) {
    Point shot_vector = shot.getDirection();
    Angle shot_dir = shot.toVector().orientation();
    Point bot_vector = shot_vector.norm();

    // TODO: magic numbers here??
    Point ball_vel = ball.velocity();
    Point lateral_vel = ball_vel - (ball_vel.dot(-bot_vector))*(-bot_vector);
    double lateral_speed = 0.3*lateral_vel.len();
    double kick_speed = 5.5;
    Angle shot_offset = Angle::asin(lateral_speed/kick_speed);

    //check which direction the ball is going in so we can decide which direction to apply the offset in
    if(lateral_vel.dot(shot_vector.rotate(Angle::quarter())) > 0){
        // need to go clockwise
        shot_offset = - shot_offset;
    }
    return shot_dir + shot_offset;
}
