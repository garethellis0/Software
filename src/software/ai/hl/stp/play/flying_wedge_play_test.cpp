#include "software/ai/hl/stp/play/flying_wedge_play.h"

#include <gtest/gtest.h>

#include "shared/constants.h"
#include "software/simulated_tests/simulated_play_test_fixture.h"
#include "software/test_util/test_util.h"
#include "software/simulated_tests/terminating_validation_functions/robot_possession_validation.h"

class FlyingWedgePlayTest: public SimulatedPlayTestFixture {
protected:
    // FUTURE_TODO: What if the simulator field changes? this doesn't seem like a great
    //              solution.
    Field field = Field::createSSLDivisionBField();
};

TEST_F(FlyingWedgePlayTest, ball_in_center_no_enemies_blocking) {
    enableVisualizer();
    BallState ball_state( Point(-2.5, 0), Vector(0,0));

    // TODO: put these robots in better positions
    auto friendly_robots = TestUtil::createStationaryRobotStatesWithId(
            {Point(-3.5, 0),
             Point(-3, 0), Point(-2, 1.5), Point(-2, -1.5),
             });

    setFriendlyGoalie(0);

    auto enemy_robots = TestUtil::createStationaryRobotStatesWithId(
            {Point(1, 2.5), Point(1, -2.5), field.enemyGoalCenter(),
             field.enemyDefenseArea().negXNegYCorner(),
             field.enemyDefenseArea().negXPosYCorner()});
    setEnemyGoalie(0);
    setAIPlay(TYPENAME(FlyingWedgePlay));

    static constexpr double WEDGE_HEAD_ROBOTS_OFFSET = 2.0 / 3.0 * ROBOT_RADIUS;
    static constexpr double WEDGE_HEAD_TO_TAIL_OFFSET = 1.5 * ROBOT_RADIUS;

    setRefereeCommand(RefereeCommand::NORMAL_START, RefereeCommand::INDIRECT_FREE_US);

    auto robots_in_formation = [](std::shared_ptr<World> world_ptr) -> ::testing::AssertionResult{
        auto robot_1_optional = world_ptr->friendlyTeam().getRobotById(1);
        LOG_IF(FATAL, !robot_1_optional);
        auto robot_2_optional = world_ptr->friendlyTeam().getRobotById(2);
        LOG_IF(FATAL, !robot_2_optional);
        auto robot_3_optional = world_ptr->friendlyTeam().getRobotById(3);
        LOG_IF(FATAL, !robot_3_optional);

        Vector robot_1_to_2_offset = robot_2_optional->position() - robot_1_optional->position();
        auto robot_2_in_position = TestUtil::equalWithinTolerance(Vector(WEDGE_HEAD_ROBOTS_OFFSET, WEDGE_HEAD_TO_TAIL_OFFSET), robot_1_to_2_offset, 0.1);
        Vector robot_1_to_3_offset = robot_3_optional->position() - robot_1_optional->position();
        auto robot_3_in_position = TestUtil::equalWithinTolerance(Vector(WEDGE_HEAD_ROBOTS_OFFSET, WEDGE_HEAD_TO_TAIL_OFFSET), robot_1_to_3_offset, 0.1);

        if (robot_2_in_position && robot_3_in_position) {
            return ::testing::AssertionSuccess();
        } else {
            return ::testing::AssertionFailure() << robot_2_in_position << ", " << robot_3_in_position;
        }
    };

    std::vector<ValidationFunction> terminating_validation_functions = {
            // We expect the robot at the "tail" of the wedge to move the ball to the target position
            [](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                robotMovesBallToPositionWithoutLosingPossession(3, Point(2, 0), world_ptr, yield);
            },
    };


    std::vector<ValidationFunction> non_terminating_validation_functions = {
            // We expect the robots that make up the "head" of the wedge to stay at
            // fixed offsets in front of the "tail" robot
            [&](std::shared_ptr<World> world_ptr, ValidationCoroutine::push_type& yield) {
                // The robots must get in formation
                const Timestamp MIN_TIME_TO_SETUP_FORMATION = Timestamp::fromSeconds(3);
                while(world_ptr->getMostRecentTimestamp() < MIN_TIME_TO_SETUP_FORMATION
                && !robots_in_formation(world_ptr)) {
                    yield("Waiting for robots to get into formation");
                }
                if (!robots_in_formation(world_ptr)) {
                    FAIL() << "Robots didn't make it into formation within " << MIN_TIME_TO_SETUP_FORMATION;
                }
                // Once they get in formation, they must remain in formation
                while (true) {
                    ASSERT_TRUE(robots_in_formation(world_ptr));
                    yield("Ensuring robots remain in formation");
                }
            },
    };

    runTest(field, ball_state, friendly_robots, enemy_robots,
            terminating_validation_functions, non_terminating_validation_functions,
            Duration::fromSeconds(10));
}