/**
 * Implementation of the PasserTactic
 */
#include "ai/hl/stp/tactic/passer_tactic.h"

#include "ai/hl/stp/action/kick_action.h"
#include "ai/hl/stp/action/move_action.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"

using namespace AI::Passing;

PasserTactic::PasserTactic(Pass pass, const Ball &ball, bool loop_forever)
    : pass(std::move(pass)), ball(ball), Tactic(loop_forever)
{
}

std::string PasserTactic::getName() const
{
    return "Passer Tactic";
}

void PasserTactic::updateParams(const Pass &updated_pass,
                                const Ball &ball)
{
    this->pass = updated_pass;
    this->ball = ball;
}

double PasserTactic::calculateRobotCost(const Robot& robot, const World& world)
{
    // Prefer robots closer to the pass start position
    // We normalize with the total field length so that robots that are within the field
    // have a cost less than 1
    double cost =
        (robot.position() - pass.passerPoint()).len() / world.field().totalLength();
    return std::clamp<double>(cost, 0, 1);
}

std::unique_ptr<Intent> PasserTactic::calculateNextIntent(
    intent_coroutine::push_type& yield)
{
    MoveAction move_action = MoveAction(MoveAction::ROBOT_CLOSE_TO_DEST_THRESHOLD, true);
    // Move to a position just behind the ball (in the direction of the pass)
    // until it's time to perform the pass
    while (ball.lastUpdateTimestamp() < pass.startTime())
    {
        // We want to wait just behind where the pass is supposed to start, so that the
        // ball is *almost* touching the kicker
        Vector ball_offset =
            Vector::createFromAngle(pass.passerOrientation())
                .norm(DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS * 2);
        Point wait_position = pass.passerPoint() - ball_offset;

        yield(move_action.updateStateAndGetNextIntent(*robot, wait_position,
                                                      pass.passerOrientation(), 0));
    }

    KickAction kick_action = KickAction();
    do
    {
        // TODO: We should be aligning to the ball position here, NOT the passerPoint. We can use the ball
        // timestamp instead of giving this a time as well

        // We want the robot to move to the starting position for the shot and also
        // rotate to the correct orientation to face the shot
        yield(kick_action.updateStateAndGetNextIntent(
            *robot, ball.position(), pass.receiverPoint(), pass.speed()));
    } while (!kick_action.done());
}
