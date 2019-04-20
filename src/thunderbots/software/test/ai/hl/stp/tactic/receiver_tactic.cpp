/**
 * Tests for the Reciever Tactic
 */
#include "ai/hl/stp/tactic/receiver_tactic.h"

#include <gtest/gtest.h>

#include "ai/intent/kick_intent.h"
#include "ai/intent/move_intent.h"
#include "test/test_util/test_util.h"

#include "shared/constants.h"

using namespace AI::Passing;

TEST(ReceiverTacticTest, robot_not_at_receive_position_pass_not_started){
    Robot receiver = Robot(13, Point(1, -3), Vector(), Angle::zero(), AngularVelocity::zero(),
                        Timestamp::fromSeconds(0));

    Pass pass({1, 1}, {0.5, 0}, 2.29, Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({1,1}, {0,0}, Timestamp::fromSeconds(0));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // We should be moving towards the position to receive the ball, and turning to
    // face the direction the pass is coming from
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_DOUBLE_EQ(0.5, move_intent.getDestination().x());
    EXPECT_DOUBLE_EQ(0.0, move_intent.getDestination().y());
    EXPECT_EQ(pass.receiverOrientation(), move_intent.getFinalAngle());
    EXPECT_FALSE(move_intent.getDribblerEnabled());
    EXPECT_FALSE(move_intent.getAutoKickEnabled());
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_not_started){
    Pass pass({1, 1}, {0.5, 0}, 2.29, Timestamp::fromSeconds(5));

    Robot receiver = Robot(13, Point(0.5, 0), Vector(), pass.receiverOrientation(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(0));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({1,1}, {0,0}, Timestamp::fromSeconds(0));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since we're already setup to receive the pass, we should just be trying to move
    // to our current position. We should continue to yield new Move Intents even though
    // we're at the target position
    for (int i = 0; i < 5; i++){
        tactic.updateParams(friendly_team, enemy_team, pass, ball);

        MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
        EXPECT_EQ(13, move_intent.getRobotId());
        EXPECT_DOUBLE_EQ(0.5, move_intent.getDestination().x());
        EXPECT_DOUBLE_EQ(0.0, move_intent.getDestination().y());
        EXPECT_EQ(pass.receiverOrientation(), move_intent.getFinalAngle());
        EXPECT_FALSE(move_intent.getDribblerEnabled());
        EXPECT_FALSE(move_intent.getAutoKickEnabled());
    }
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_started_goal_open_angle_feasible){
    // Test case where we can feasibly one-touch the ball into the net

    Pass pass({1, 1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({1,-3}, {-1, 3}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // We should be trying to move into a position to properly deflect the ball into
    // the net with a kick
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_LT(move_intent.getDestination().x(), -0.001);
    EXPECT_GT(move_intent.getDestination().x(), -0.1);

    EXPECT_GT(move_intent.getDestination().y(), 0.001);
    EXPECT_LT(move_intent.getDestination().y(), 0.1);

    EXPECT_LT(move_intent.getFinalAngle().toDegrees(), -1);
    EXPECT_GT(move_intent.getFinalAngle().toDegrees(), -90);

    EXPECT_FALSE(move_intent.getDribblerEnabled());
    EXPECT_TRUE(move_intent.getAutoKickEnabled());
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_started_goal_open_angle_not_feasible){
    // Test case where we can't feasibly one-touch the ball into the net because the
    // deflection angle between the pass and shot would be way too large

    Pass pass({-1, -1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    Ball ball({-1,-1}, {1,1}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since there's no reasonable way we could one-touch kick the pass into the net,
    // we should be lining up to receive it
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_NEAR(0.0, move_intent.getDestination().x(), 0.0001);
    EXPECT_NEAR(0.0, move_intent.getDestination().y(), 0.0001);
    EXPECT_EQ(pass.receiverOrientation(), move_intent.getFinalAngle());

    EXPECT_TRUE(move_intent.getDribblerEnabled());
    EXPECT_FALSE(move_intent.getAutoKickEnabled());
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_started_goal_blocked){
    // Test case where we are facing right towards the goal, but it is blocked

    Pass pass({0.5, 0.5}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({
        Robot(0, {1, -0.2}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(1, {1, -0.1}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(2, {1, 0.0}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(3, {1, 0.1}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0)),
        Robot(4, {1, 0.2}, {0,0}, Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0))
    });

    Ball ball({0.5, 0.5}, {-0.5,-0.5}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since there's no reasonable way we could one-touch kick the pass into the net,
    // we should be lining up to receive it
    MoveIntent move_intent = dynamic_cast<MoveIntent &>(*tactic.getNextIntent());
    EXPECT_EQ(13, move_intent.getRobotId());
    EXPECT_NEAR(0.0, move_intent.getDestination().x(), 0.0001);
    EXPECT_NEAR(0.0, move_intent.getDestination().y(), 0.0001);
    EXPECT_EQ(pass.receiverOrientation(), move_intent.getFinalAngle());

    EXPECT_TRUE(move_intent.getDribblerEnabled());
    EXPECT_FALSE(move_intent.getAutoKickEnabled());
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_received){
    // Test where we have received the pass and the ball is now positioned in our dribbler

    Pass pass({-1, 1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    // Position the ball just in front of the robot dribbler
    Point ball_pos = receiver.position() + Vector(receiver.orientation().cos(), receiver.orientation().sin()).norm(DIST_TO_FRONT_OF_ROBOT_METERS + 0.02);
    Ball ball(ball_pos, {0,0}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since we've received the ball, we shouldn't yield anything
    EXPECT_EQ(std::unique_ptr<Intent>({}), tactic.getNextIntent());
}

TEST(ReceiverTacticTest, robot_at_receive_position_pass_one_touch_kicked){
    // Test where we have just one-touch kicked the pass towards the goal

    Pass pass({-1, 1}, {0, 0}, 2.29, Timestamp::fromSeconds(0.49));

    Robot receiver = Robot(13, Point(0, 0), Vector(), pass.receiverOrientation(), AngularVelocity::zero(),
                           Timestamp::fromSeconds(5));

    Team friendly_team(Duration::fromSeconds(10));
    friendly_team.updateRobots({receiver});

    Team enemy_team(Duration::fromSeconds(10));
    enemy_team.updateRobots({});

    // The ball is travelling away from the origin towards the enemy net
    Ball ball({1,0}, {4,0}, Timestamp::fromSeconds(5));

    ReceiverTactic tactic(::Test::TestUtil::createSSLDivBField(), friendly_team, enemy_team, pass, ball, false);

    tactic.updateRobot(receiver);

    // Since we've kicked the ball, we shouldn't yield anything
    EXPECT_EQ(std::unique_ptr<Intent>({}), tactic.getNextIntent());
}

