#include "ai/hl/stp/play/shoot_or_pass_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"

const std::string ShootOrPassPlay::name = "Example Play";

std::string ShootOrPassPlay::getName() const
{
    return ShootOrPassPlay::name;
}

bool ShootOrPassPlay::isApplicable(const World &world) const
{
    return true;
}

bool ShootOrPassPlay::invariantHolds(const World &world) const
{
    return true;
}

void ShootOrPassPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // TODO: general description of play here like you have for corner kick

    // Create two cherry-pickers in the front two halves
    // Create patrollers off to the sides in loops
    // Start a PassGenerator from the current ball position
    // Move robot into position to pass/shoot (facing the net)
    // Start a 2 second timer over which we linearly decrease score tolerance
    // do
    //      shoot on the enemy net if we have an open shot
    //      if good enough pass found, take it
    //      if enemy within X meters of us, chip the ball to the enemy corners in such a way that it goes out the side of the field (to set positions)
    // while(...)



    // Create MoveTactics that will loop forever
    auto move_tactic_1 = std::make_shared<MoveTactic>(true);
    auto move_tactic_2 = std::make_shared<MoveTactic>(true);
    auto move_tactic_3 = std::make_shared<MoveTactic>(true);
    auto move_tactic_4 = std::make_shared<MoveTactic>(true);
    auto move_tactic_5 = std::make_shared<MoveTactic>(true);
    auto move_tactic_6 = std::make_shared<MoveTactic>(true);

    do
    {
        // The angle between each robot spaced out in a circle around the ball
        Angle angle_between_robots = Angle::full() / world.friendlyTeam().numRobots();

        // Move the robots in a circle around the ball, facing the ball
        move_tactic_1->updateParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 1),
            (angle_between_robots * 1) + Angle::half(), 0);
        move_tactic_2->updateParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 2),
            (angle_between_robots * 2) + Angle::half(), 0);
        move_tactic_3->updateParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 3),
            (angle_between_robots * 3) + Angle::half(), 0);
        move_tactic_4->updateParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 4),
            (angle_between_robots * 4) + Angle::half(), 0);
        move_tactic_5->updateParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 5),
            (angle_between_robots * 5) + Angle::half(), 0);
        move_tactic_6->updateParams(
            world.ball().position() + Point::createFromAngle(angle_between_robots * 6),
            (angle_between_robots * 6) + Angle::half(), 0);

        // yield the Tactics this Play wants to run, in order of priority
        yield({move_tactic_1, move_tactic_2, move_tactic_3, move_tactic_4, move_tactic_5,
               move_tactic_6});
    } while (true);
}

// Register this play in the PlayFactory
static TPlayFactory<ShootOrPassPlay> factory;
