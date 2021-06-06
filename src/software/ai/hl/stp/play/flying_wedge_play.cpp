#include "software/ai/hl/stp/play/flying_wedge_play.h"

#include <memory>
#include <string>

#include "software/ai/hl/stp/tactic/dribble/dribble_tactic.h"
#include "software/ai/hl/stp/tactic/move/move_tactic.h"

FlyingWedgePlay::FlyingWedgePlay(std::shared_ptr<const PlayConfig> config)
: Play(config, true) {
    // TODO
}

bool FlyingWedgePlay::isApplicable(const World &world) const {
    // TODO
    return true;
}

bool FlyingWedgePlay::invariantHolds(const World &world) const {
    // TODO
    return true;
}

void FlyingWedgePlay::getNextTactics(TacticCoroutine::push_type &yield, const World &world) {
    // This play:
    // 1. Sets up the robots in a flying wedge formation, with two robots
    //    ahead of a third robot, which holds the ball.
    // 2. Drives the wedge right down the field, trying to get as deep into the enemy
    //    end as possible.

    auto dribbler_tactic = std::make_shared<DribbleTactic>();
    auto wedge_defender_1 = std::make_shared<MoveTactic>(true);
    auto wedge_defender_2 = std::make_shared<MoveTactic>(true);


}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, FlyingWedgePlay, PlayConfig> factory;
