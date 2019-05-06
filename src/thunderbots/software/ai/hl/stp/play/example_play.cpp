#include "ai/hl/stp/play/example_play.h"

#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/passing/pass_generator.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"

const std::string ExamplePlay::name = "Example Play";

std::string ExamplePlay::getName() const
{
    return ExamplePlay::name;
}

bool ExamplePlay::isApplicable(const World &world) const
{
    return true;
}

bool ExamplePlay::invariantHolds(const World &world) const
{
    return true;
}

std::vector<std::shared_ptr<Tactic>> ExamplePlay::getNextTactics(
    TacticCoroutine::push_type &yield, const World &world)
{
    AI::Passing::PassGenerator pass_generator(world, world.ball().position());
    pass_generator.setPasserRobotId(0);

    auto [pass, score] = pass_generator.getBestPassSoFar();
    do {
        pass_generator.setWorld(world);
        pass_generator.setPasserPoint(world.ball().position());
        auto best_pass_and_score = pass_generator.getBestPassSoFar();
        pass = best_pass_and_score.first;
                score = best_pass_and_score.second;
        std::cout << "Score: " << score << std::endl;
        std::cout << "Pass: " << pass << std::endl;
        std::cout << "Offset: " << (pass.startTime() - world.ball().lastUpdateTimestamp()).getSeconds() << std::endl;
        std::cout << std::endl;
        yield({});
    } while (score < 0.1 || isnan(score));

    auto receiver_tactic = std::make_shared<ReceiverTactic>(world.field(), world.friendlyTeam(), world.enemyTeam(), pass, world.ball(), false);
    auto passer_tactic = std::make_shared<PasserTactic>(pass, world.ball(), false);

    do {
        receiver_tactic->updateParams(world.friendlyTeam(), world.enemyTeam(), pass, world.ball());
        passer_tactic->updateParams(pass, world.ball());
        yield({receiver_tactic, passer_tactic});
    } while (true);

    do {
        pass_generator.setWorld(world);
        pass_generator.setPasserPoint(world.ball().position());
        auto [pass, score] = pass_generator.getBestPassSoFar();
        std::cout << "Score: " << score << std::endl;
        std::cout << "Pass: " << pass << std::endl;
        std::cout << "Offset: " << (pass.startTime() - world.ball().lastUpdateTimestamp()).getSeconds() << std::endl;
        std::cout << std::endl;

        if (score > 0.2){
            receiver_tactic->updateParams(world.friendlyTeam(), world.enemyTeam(), pass, world.ball());
            passer_tactic->updateParams(pass, world.ball());
            yield({receiver_tactic, passer_tactic});
        } else {
            yield({});
        }
    } while (true);

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
static TPlayFactory<ExamplePlay> factory;
