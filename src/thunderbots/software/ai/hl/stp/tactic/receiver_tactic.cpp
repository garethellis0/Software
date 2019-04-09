/**
 * Implementation of the ReceiverTactic
 */
#include "ai/hl/stp/tactic/receiver_tactic.h"

#include "ai/hl/stp/action/move_action.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/logger/init.h"

using namespace AI::Passing;

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
        // rotate to the correct orientation to face the shot
        yield(move_action.updateStateAndGetNextIntent(
                *robot, pass.receiverPoint(), pass.receiverOrientation(), 0));
    } while (!move_action.done());

    // Once we're at the receiver position, automatically align to the ball
    do {
        // TODO:
        // If within angle to shoot at net and have shot, shoot at net, otherwise just catch
        // and call the tactic finished
        // YOU ARE HERE: Need to expose "autokick" and "enable dribbler" flag in the "MoveAction"
    } while(true);

}
