/**
 * Interface of the ReceiverTactic
 */
#pragma once

#include "ai/hl/stp/tactic/tactic.h"
#include "ai/passing/pass.h"
#include "geom/ray.h"

/**
 * This tactic is for a robot receiving a pass. It should be used in conjunction with
 * the `PasserTactic` in order to complete the pass.
 *
 * TODO: Is the below statement true?
 * Note that this tactic does not take into account the time the pass should occur at,
 * it simply tries to move to the best position to receive the pass as possible
 */
class ReceiverTactic : public Tactic
{
   public:
    /**
     * Creates a new ReceiverTactic
     *
     * @param pass The pass this tactic should try to receive
     * @param ball The ball being passed
     * @param loop_forever Whether or not this Tactic should never complete. If true, the
     * tactic will be restarted every time it completes
     */
    explicit ReceiverTactic(AI::Passing::Pass pass, const Ball& ball, bool loop_forever);

    std::string getName() const override;

    /**
     * Updates the parameters for this ReceiverTactic.
     *
     * @param pass The pass this tactic should try to receive
     * @param ball The ball being passed
     */
    void updateParams(const AI::Passing::Pass& updated_pass, const Ball& ball);

    /**
     * Calculates the cost of assigning the given robot to this Tactic. Prefers robots
     * closer to the block destination
     *
     * @param robot The robot to evaluate the cost for
     * @param world The state of the world with which to perform the evaluation
     * @return A cost in the range [0,1] indicating the cost of assigning the given robot
     * to this tactic. Lower cost values indicate a more preferred robot.
     */
    double calculateRobotCost(const Robot& robot, const World& world) override;

   private:
    // The minimum open angle we must have for a shot before we will attempt it
    static constexpr double MIN_SHOT_OPEN_ANGLE_DEGREES = 10;

    std::unique_ptr<Intent> calculateNextIntent(
        intent_coroutine::push_type& yield) override;

    /**
     * Calculate the angle the robot should be at in order to perform the given shot
     *
     * @param shot A Ray representing the shot we want to take
     * @param ball The ball we want to shoot
     *
     * @return The angle to position the robot at so that it can redirect the ball to
     *         the shot vector at the position where the shot vector and ball velocity
     *         vectors intersect
     */
    static Angle getOneTimeShotDirection(const Ray& shot, const Ball& ball);

    // Tactic parameters
    // The pass this tactic is executing
    AI::Passing::Pass pass;

    // The ball being passed
    Ball ball;
};
