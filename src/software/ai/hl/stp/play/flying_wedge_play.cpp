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

    dribbler_tactic->updateControlParams(Point(2,0), std::nullopt);

    while(true) {
        Point ball_position = world.ball().position();
        if (auto dribbler_robot = dribbler_tactic->getAssignedRobot()) {
            // Position the defenders relative to the dribbler robot
            Point dribbler_robot_position = dribbler_robot->position();
            Vector dribbler_to_ball = ball_position - dribbler_robot_position;
            Vector longitudinal_vector = dribbler_to_ball.normalize(WEDGE_DEFENDER_LONGITUDINAL_OFFSET);
            Vector lateral_vector = dribbler_to_ball.perpendicular().normalize(WEDGE_DEFENDER_LATERAL_OFFSET);
            wedge_defender_1->updateControlParams(ball_position + longitudinal_vector + lateral_vector, Angle::zero(), 0.0, MaxAllowedSpeedMode::PHYSICAL_LIMIT);
            wedge_defender_2->updateControlParams(ball_position + longitudinal_vector - lateral_vector, Angle::zero(), 0.0, MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        } else {
            // Position the defenders to offsets in front of the ball, facing towards
            // the enemy end
            wedge_defender_1->updateControlParams(ball_position + Vector(WEDGE_DEFENDER_LONGITUDINAL_OFFSET, WEDGE_DEFENDER_LATERAL_OFFSET), Angle::zero(), 0.0, MaxAllowedSpeedMode::PHYSICAL_LIMIT);
            wedge_defender_2->updateControlParams(ball_position + Vector(WEDGE_DEFENDER_LONGITUDINAL_OFFSET, -WEDGE_DEFENDER_LATERAL_OFFSET), Angle::zero(), 0.0, MaxAllowedSpeedMode::PHYSICAL_LIMIT);
        }

        yield({{dribbler_tactic, wedge_defender_1, wedge_defender_2}});
    }
}

// Register this play in the genericFactory
static TGenericFactory<std::string, Play, FlyingWedgePlay, PlayConfig> factory;
