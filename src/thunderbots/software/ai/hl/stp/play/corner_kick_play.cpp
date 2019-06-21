#include "ai/hl/stp/play/corner_kick_play.h"

#include <ai/hl/stp/evaluation/ball.h>

#include <g3log/g3log.hpp>

#include "ai/hl/stp/evaluation/ball.h"
#include "ai/hl/stp/evaluation/possession.h"
#include "ai/hl/stp/play/play_factory.h"
#include "ai/hl/stp/tactic/move_tactic.h"
#include "ai/hl/stp/tactic/passer_tactic.h"
#include "ai/hl/stp/tactic/receiver_tactic.h"
#include "ai/passing/pass_generator.h"
#include "shared/constants.h"
#include "util/logger/custom_logging_levels.h"

using namespace Passing;

const std::string CornerKickPlay::name = "Corner Kick Play";

CornerKickPlay::CornerKickPlay() : MAX_TIME_TO_COMMIT_TO_PASS(Duration::fromSeconds(5)) {}

std::string CornerKickPlay::getName() const
{
    return CornerKickPlay::name;
}

bool CornerKickPlay::isApplicable(const World &world) const
{
    return world.gameState().isOurDirectFree() &&
           Evaluation::ballInFriendlyCorner(world.field(), world.ball(),
                                            BALL_IN_CORNER_RADIUS);
}

bool CornerKickPlay::invariantHolds(const World &world) const
{
    return !Evaluation::teamHasPossession(world.enemyTeam(), world.ball());
}

void CornerKickPlay::getNextTactics(TacticCoroutine::push_type &yield)
{
    // TODO: Rough description of what this function does (stages, tactics, etc.) here

    // Figure out if we're taking the kick from the +y or -y corner
    bool kick_from_pos_corner = world.ball().position().y() > 0;

    // We want the two cherry pickers to be in rectangles on the +y and -y sides of the
    // field in the +x half. We also further offset the rectangle from the goal line
    // for the cherry-picker closer to where we're taking the corner kick from
    Vector enemy_defense_area_pos_y_x_offset(world.field().enemyDefenseArea().width(), 0);
    Vector enemy_defense_area_neg_y_x_offset(world.field().enemyDefenseArea().width(), 0);
    if (kick_from_pos_corner)
    {
        enemy_defense_area_pos_y_x_offset +=
            {world.field().enemyDefenseArea().width(), 0};
    }
    else
    {
        // kick from neg corner
        enemy_defense_area_neg_y_x_offset +=
            {world.field().enemyDefenseArea().width(), 0};
    }
    Vector center_line_x_offset(1, 0);
    Rectangle neg_y_cherry_pick_rectangle(
        world.field().centerPoint() + center_line_x_offset,
        world.field().enemyCornerNeg() - enemy_defense_area_neg_y_x_offset);
    Rectangle pos_y_cherry_pick_rectangle(
        world.field().centerPoint() + center_line_x_offset,
        world.field().enemyCornerPos() - enemy_defense_area_pos_y_x_offset);

    // This tactic will move a robot into position to initially take the free-kick
    auto align_to_ball_tactic = std::make_shared<MoveTactic>();

    // These two tactics will set robots to roam around the field, trying to put
    // themselves into a good position to receive a pass
    auto cherry_pick_tactic_pos_y =
        std::make_shared<CherryPickTactic>(world, pos_y_cherry_pick_rectangle);
    auto cherry_pick_tactic_neg_y =
        std::make_shared<CherryPickTactic>(world, neg_y_cherry_pick_rectangle);

    // Setup two bait robots on the opposite side of the field to where the corner kick
    // is taking place to pull enemies away from the goal
    Point opposite_corner_to_kick = kick_from_pos_corner ? world.field().enemyCornerNeg()
                                                         : world.field().enemyCornerPos();
    Point bait_move_tactic_1_pos =
        opposite_corner_to_kick - Vector(world.field().enemyDefenseArea().width() * 0.5,
                                         copysign(0.5, opposite_corner_to_kick.y()));
    Point bait_move_tactic_2_pos =
        opposite_corner_to_kick - Vector(world.field().enemyDefenseArea().width() * 1.5,
                                         copysign(0.5, opposite_corner_to_kick.y()));
    auto bait_move_tactic_1 = std::make_shared<MoveTactic>(true);
    auto bait_move_tactic_2 = std::make_shared<MoveTactic>(true);
    bait_move_tactic_1->updateParams(
        bait_move_tactic_1_pos,
        (world.field().enemyGoal() - bait_move_tactic_1_pos).orientation(), 0.0);
    bait_move_tactic_2->updateParams(
        bait_move_tactic_2_pos,
        (world.field().enemyGoal() - bait_move_tactic_2_pos).orientation(), 0.0);

    PassGenerator pass_generator(world, world.ball().position());

    std::pair<Pass, double> best_pass_and_score_so_far =
        pass_generator.getBestPassSoFar();

    // Wait for a robot to be assigned to align to take the corner
    while (!align_to_ball_tactic->getAssignedRobot())
    {
        LOG(DEBUG) << "Nothing assigned to align to ball yet";
        updateAlignToBallTactic(align_to_ball_tactic);
        updateCherryPickTactics({cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y});
        updatePassGenerator(pass_generator);

        yield({align_to_ball_tactic, cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y,
               bait_move_tactic_1, bait_move_tactic_2});
    }


    // Set the passer on the pass generator
    pass_generator.setPasserRobotId(align_to_ball_tactic->getAssignedRobot()->id());
    LOG(DEBUG) << "Assigned robot " << align_to_ball_tactic->getAssignedRobot()->id()
               << " to be the passer in the PassGenerator";

    // Align the kicker to take the corner kick and wait for a good pass
    // To get the best pass possible we start by aiming for a perfect one and then
    // decrease the minimum score over time
    double min_score = 1;
    // TODO: change this to use the world timestamp (Issue #423)
    Timestamp commit_stage_start_time = world.ball().lastUpdateTimestamp();
    do
    {
        // TODO: Bit of hack this.......
        // Continue trying to align to deal with the case where we've finished
        // aligning but there's not passing to
        if (align_to_ball_tactic->done())
        {
            align_to_ball_tactic = std::make_shared<MoveTactic>();
        }
        updateAlignToBallTactic(align_to_ball_tactic);
        updateCherryPickTactics({cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y});
        updatePassGenerator(pass_generator);

        yield({align_to_ball_tactic, cherry_pick_tactic_pos_y, cherry_pick_tactic_neg_y,
               bait_move_tactic_1, bait_move_tactic_2});

        best_pass_and_score_so_far = pass_generator.getBestPassSoFar();
        LOG(DEBUG) << "Best pass found so far is: " << best_pass_and_score_so_far.first;
        LOG(DEBUG) << "    with score: " << best_pass_and_score_so_far.second;

        // TODO: change this to use the world timestamp (Issue #423)
        Duration time_since_commit_stage_start =
            world.ball().lastUpdateTimestamp() - commit_stage_start_time;
        min_score = 1 - std::min(time_since_commit_stage_start.getSeconds() /
                                     MAX_TIME_TO_COMMIT_TO_PASS.getSeconds(),
                                 1.0);
    } while (!align_to_ball_tactic->done() ||
             best_pass_and_score_so_far.second < min_score);

    // Commit to a pass
    Pass pass = best_pass_and_score_so_far.first;

    LOG(DEBUG) << "Committing to pass: " << best_pass_and_score_so_far.first;
    LOG(DEBUG) << "Score of pass we committed to: " << best_pass_and_score_so_far.second;

    // Destruct the PassGenerator and CherryPick tactics (which contain a PassGenerator
    // each) to save a significant number of CPU cycles
    // TODO: stop the PassGenerators here instead of destructing them (Issue #636)
    pass_generator.~PassGenerator();
    cherry_pick_tactic_pos_y->~CherryPickTactic();
    cherry_pick_tactic_neg_y->~CherryPickTactic();

    // TODO: send the remaining robots forward in the hope we'll make opportunities

    // Perform the pass and wait until the receiver is finished
    auto passer = std::make_shared<PasserTactic>(pass, world.ball(), false);
    auto receiver =
        std::make_shared<ReceiverTactic>(world.field(), world.friendlyTeam(),
                                         world.enemyTeam(), pass, world.ball(), false);
    do
    {
        passer->updateParams(pass, world.ball());
        receiver->updateParams(world.friendlyTeam(), world.enemyTeam(), pass,
                               world.ball());
        yield({passer, receiver, bait_move_tactic_1, bait_move_tactic_2});
    } while (!receiver->done());

    LOG(DEBUG) << "Finished";
}

void CornerKickPlay::updateCherryPickTactics(
    std::vector<std::shared_ptr<CherryPickTactic>> tactics)
{
    for (auto &tactic : tactics)
    {
        tactic->updateParams(world);
    }
}

void CornerKickPlay::updateAlignToBallTactic(
    std::shared_ptr<MoveTactic> align_to_ball_tactic)
{
    Vector ball_to_center_vec = Vector(0, 0) - world.ball().position();
    // We want the kicker to get into position behind the ball facing the center
    // of the field
    align_to_ball_tactic->updateParams(
        world.ball().position() - ball_to_center_vec.norm(ROBOT_MAX_RADIUS_METERS * 2),
        ball_to_center_vec.orientation(), 0);
}

void CornerKickPlay::updatePassGenerator(PassGenerator &pass_generator)
{
    pass_generator.setWorld(world);
    pass_generator.setPasserPoint(world.ball().position());
}

// Register this play in the PlayFactory
static TPlayFactory<CornerKickPlay> factory;
