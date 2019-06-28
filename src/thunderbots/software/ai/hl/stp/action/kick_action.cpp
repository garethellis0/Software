#include "ai/hl/stp/action/kick_action.h"

#include <ai/world/ball.h>

#include "ai/intent/kick_intent.h"
#include "ai/intent/move_intent.h"
#include "geom/polygon.h"
#include "geom/util.h"
#include "shared/constants.h"
#include "util/parameter/dynamic_parameters.h"

#include "util/canvas_messenger/canvas_messenger.h"

KickAction::KickAction() : Action(), ball({0, 0}, {0, 0}, Timestamp::fromSeconds(0)) {}

std::unique_ptr<Intent>
KickAction::updateStateAndGetNextIntent(const Robot &robot, const Ball &ball,
                                        Point kick_target,
                                        double kick_speed_meters_per_second)
{
    return updateStateAndGetNextIntent(robot, ball,
                                       (kick_target - ball.position()).orientation(),
                                       kick_speed_meters_per_second);
}

std::unique_ptr<Intent>
KickAction::updateStateAndGetNextIntent(const Robot &robot, const Ball &ball,
                                        Angle kick_direction,
                                        double kick_speed_meters_per_second)
{
    // Update the parameters stored by this Action
    this->robot                        = robot;
    this->ball                         = ball;
    this->kick_direction               = kick_direction;
    this->kick_speed_meters_per_second = kick_speed_meters_per_second;

    return getNextIntent();
}


void KickAction::calculateNextIntentBallNotMoving(IntentCoroutine::push_type &yield){
    // How large the triangle is that defines the region where the robot is
    // behind the ball and ready to kick.
    // We want to keep the region small enough that we won't use the KickIntent from too
    // far away (since the KickIntent doesn't avoid obstacles and we risk colliding
    // with something), but large enough we can reasonably get in the region and kick the
    // ball successfully.
    // This value is 'X' in the ASCII art below
    double size_of_region_behind_ball = 4 * ROBOT_MAX_RADIUS_METERS;

    // ASCII art showing the region behind the ball
    // Diagram not to scale
    //
    //                             X
    //                      v-------------v
    //
    //                   >  B-------------C
    //                   |   \           /
    //                   |    \         /
    //                   |     \       /       <- Region considered "behind ball"
    //                 X |      \     /
    //                   |       \   /
    //                   |        \ /
    //    The ball is    >         A
    //    at A
    //                             |
    //                             V
    //                     direction of kick

    // A vector in the direction opposite the kick (behind the ball)
    Vector behind_ball =
            Vector::createFromAngle(this->kick_direction + Angle::half());


    // The points below make up the triangle that defines the region we treat as
    // "behind the ball". They correspond to the vertices labeled 'A', 'B', and 'C'
    // in the ASCII diagram

    // We make the region close enough to the ball so that the robot will still be
    // inside it when taking the kick.
    Point behind_ball_vertex_A = ball.position();
    Point behind_ball_vertex_B =
            behind_ball_vertex_A + behind_ball.norm(size_of_region_behind_ball) +
            behind_ball.perp().norm(size_of_region_behind_ball / 2);
    Point behind_ball_vertex_C =
            behind_ball_vertex_A + behind_ball.norm(size_of_region_behind_ball) -
            behind_ball.perp().norm(size_of_region_behind_ball / 2);

    Polygon behind_ball_region =
            Polygon({behind_ball_vertex_A, behind_ball_vertex_B, behind_ball_vertex_C});

    bool robot_behind_ball = behind_ball_region.containsPoint(robot->position());
    // The point in the middle of the region behind the ball
    Point point_behind_ball =
            ball.position() + behind_ball.norm(size_of_region_behind_ball * 3 / 4);

    // If we're not in position to kick, move into position
    if (!robot_behind_ball)
    {
        yield(std::make_unique<MoveIntent>(robot->id(), point_behind_ball,
                                           kick_direction, 0.0, 0));
    }
    else
    {
        yield(std::make_unique<KickIntent>(robot->id(), ball.position(), kick_direction,
                                           kick_speed_meters_per_second, 0));
    }
}

void KickAction::calculateNextIntentBallMoving(IntentCoroutine::push_type &yield){
    // Find the dist from the point right in front of the robots kicker to the ball
    double dist_to_ball_in_dribbler =
            DIST_TO_FRONT_OF_ROBOT_METERS + BALL_MAX_RADIUS_METERS;
    Point ball_contact_point =
            robot->position() +
            Vector::createFromAngle(robot->orientation()).norm(dist_to_ball_in_dribbler);
    double robot_to_ball_dist = (ball_contact_point - ball.position()).len();

    // Calculate the offset from the ball we want to target in order to
    // intercept it
    double velocity_to_offset_ratio =
            Util::DynamicParameters::KickAction::velocity_to_offset_ratio.value();
    double intercept_offset_dist = velocity_to_offset_ratio * ball.velocity().len() * robot_to_ball_dist;

    // Calculate what position the ball will be at
    Point ball_intercept_position = ball.position() + ball.velocity().norm(intercept_offset_dist);

    Util::CanvasMessenger::getInstance()->drawPoint(Util::CanvasMessenger::Layer::ROBOTS, ball_intercept_position, BALL_MAX_RADIUS_METERS, {255, 0, 0, 255});

    yield(std::make_unique<MoveIntent>(robot->id(), ball_intercept_position - Vector::createFromAngle(robot->orientation()).norm(dist_to_ball_in_dribbler),
                                       kick_direction, ball.velocity().len(), 0.0, false, AutokickType::AUTOKICK));
}


void KickAction::calculateNextIntent(IntentCoroutine::push_type &yield)
{
    while (true){
        if (ball.velocity().len() < MIN_MOVING_BALL_SPEED){
            calculateNextIntentBallNotMoving(yield);
        } else {
            calculateNextIntentBallMoving(yield);
        }
    }
}
