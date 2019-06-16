#pragma once

#include "ai/hl/stp/play/play.h"
#include "ai/hl/stp/tactic/cherry_pick_tactic.h"
#include "ai/hl/stp/tactic/move_tactic.h"

/**
 * The play for corner kicks
 */
class CornerKickPlay : public Play
{
   public:
    static const std::string name;

    CornerKickPlay();

    std::string getName() const override;

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield) override;
private:

    // TODO: Unused, delete this
    // The minimum score a pass must have before we will try to take it
    static constexpr double MIN_PASS_SCORE = 0.3;

    // The maximum time that we will wait before committing to a pass
    const Duration MAX_TIME_TO_COMMIT_TO_PASS;

    /**
     * Updates the given cherry-pick tactics
     * @param tactics
     */
    void updateCherryPickTactics(std::vector<std::shared_ptr<CherryPickTactic>> tactics);

    /**
     * Update the tactic that aligns the robot to the ball in preperation to pass
     *
     * @param align_to_ball_tactic
     */
    void updateAlignToBallTactic(std::shared_ptr<MoveTactic> align_to_ball_tactic);

    /**
     * Updates the pass generator
     *
     * @param pass_generator
     */
    void updatePassGenerator(Passing::PassGenerator& pass_generator);
};