#include "software/ai/navigator/obstacle/robot_navigation_obstacle_factory.h"
#include "software/ai/navigator/obstacle/circle_with_slice_removed.h"

RobotNavigationObstacleFactory::RobotNavigationObstacleFactory(
    std::shared_ptr<const RobotNavigationObstacleConfig> config)
    : config(config),
      robot_radius_expansion_amount(config->getRobotObstacleInflationFactor()->value() *
                                    ROBOT_MAX_RADIUS_METERS)
{
    config->getRobotObstacleInflationFactor()->registerCallbackFunction(
        [&](double new_value) {
            robot_radius_expansion_amount = new_value * ROBOT_MAX_RADIUS_METERS;
        });
}

std::vector<ObstaclePtr> RobotNavigationObstacleFactory::createFromMotionConstraint(
    const MotionConstraint &motion_constraint, const World &world) const
{
    std::vector<ObstaclePtr> obstacles;

    switch (motion_constraint)
    {
        case MotionConstraint::ENEMY_ROBOTS_COLLISION:
        {
            std::vector<ObstaclePtr> enemy_robot_obstacles =
                createFromTeam(world.enemyTeam(), world.ball());
            obstacles.insert(obstacles.end(), enemy_robot_obstacles.begin(),
                             enemy_robot_obstacles.end());
        }
        break;
        case MotionConstraint::CENTER_CIRCLE:
            obstacles.push_back(createFromShape(
                Circle(world.field().centerPoint(), world.field().centerCircleRadius())));
            break;
        case MotionConstraint::HALF_METER_AROUND_BALL:
            // 0.5 represents half a metre radius
            obstacles.push_back(createFromShape(Circle(world.ball().position(), 0.5)));
            break;
        case MotionConstraint::INFLATED_ENEMY_DEFENSE_AREA:
        {
            obstacles.push_back(createFromFieldRectangle(
                world.field().enemyDefenseArea(), world.field().fieldLines(),
                world.field().fieldBoundary(), 0.3));
        }
        break;
        case MotionConstraint::FRIENDLY_DEFENSE_AREA:
            obstacles.push_back(createFromFieldRectangle(
                world.field().friendlyDefenseArea(), world.field().fieldLines(),
                world.field().fieldBoundary()));
            break;
        case MotionConstraint::ENEMY_DEFENSE_AREA:
            obstacles.push_back(createFromFieldRectangle(world.field().enemyDefenseArea(),
                                                         world.field().fieldLines(),
                                                         world.field().fieldBoundary()));
            break;
        case MotionConstraint::FRIENDLY_HALF:
            obstacles.push_back(createFromFieldRectangle(world.field().friendlyHalf(),
                                                         world.field().fieldLines(),
                                                         world.field().fieldBoundary()));
            break;
        case MotionConstraint::ENEMY_HALF:
            obstacles.push_back(createFromFieldRectangle(world.field().enemyHalf(),
                                                         world.field().fieldLines(),
                                                         world.field().fieldBoundary()));
            break;
    }

    return obstacles;
}

std::vector<ObstaclePtr> RobotNavigationObstacleFactory::createFromMotionConstraints(
    const std::set<MotionConstraint> &motion_constraints, const World &world) const
{
    std::vector<ObstaclePtr> obstacles;
    for (auto motion_constraint : motion_constraints)
    {
        auto new_obstacles = createFromMotionConstraint(motion_constraint, world);
        obstacles.insert(obstacles.end(), new_obstacles.begin(), new_obstacles.end());
    }

    return obstacles;
}

ObstaclePtr RobotNavigationObstacleFactory::createFromRobot(const Robot &robot, const std::optional<Ball>& ball) const
{
    // radius of a hexagonal approximation of a robot
    double robot_hexagon_radius =
        (ROBOT_MAX_RADIUS_METERS + robot_radius_expansion_amount) * 2.0 / std::sqrt(3);

    // vector in the direction of the velocity and proportional to the magnitude of the
    // velocity
    Vector expanded_velocity_vector = robot.velocity().normalize(
        robot.velocity().length() * config->getSpeedScalingFactor()->value() +
        robot_radius_expansion_amount);

    /* If the robot is travelling slower than a threshold, then a stationary robot
     * obstacle will be returned. If the robot is travelling faster than a threshold, then
     * the robot will be represented by a velocity obstacle, which is an irregular hexagon
     * like so:
     *
     *                        _____
     *                       /     \
     *                      /       \
     *       The robot >   +    R    +       <
     *       is at R       |         |       |
     *                     |         |       | The length of the velocity
     *                     |         |       | obstacle extension is
     *                     |         |       | proportional to the robot velocity
     *                     |         |       |
     *                     +---------+       <
     *                          |
     *                          |
     *                          V
     *                velocity of the robot
     */

    if (expanded_velocity_vector.length() > robot_hexagon_radius)
    {
        Vector velocity_norm_radius =
            expanded_velocity_vector.normalize(robot_hexagon_radius);
        return std::make_shared<GeomObstacle<Polygon>>(Polygon(
            {// left side of robot
             robot.position() + velocity_norm_radius.rotate(Angle::quarter()),
             // back left of robot
             robot.position() + velocity_norm_radius.rotate(Angle::fromDegrees(150)),
             // back right of robot
             robot.position() + velocity_norm_radius.rotate(Angle::fromDegrees(210)),
             // right side of robot
             robot.position() + velocity_norm_radius.rotate(Angle::threeQuarter()),
             // right side of velocity obstacle extension
             robot.position() + velocity_norm_radius.rotate(Angle::threeQuarter()) +
                 expanded_velocity_vector,
             // left side of velocity obstacle extension
             robot.position() + velocity_norm_radius.rotate(Angle::quarter()) +
                 expanded_velocity_vector}));
    }
    else
    {
        if(ball) {
            return createFromRobotPosition(robot.position(), ball->position());
        } else {
            return createFromRobotPosition(robot.position());
        }
    }
}

std::vector<ObstaclePtr> RobotNavigationObstacleFactory::createFromTeam(
    const Team &team, const Ball& ball) const
{
    std::vector<ObstaclePtr> obstacles;
    for (const auto &robot : team.getAllRobots())
    {
        obstacles.push_back(createFromRobot(robot, ball));
    }
    return obstacles;
}

ObstaclePtr RobotNavigationObstacleFactory::createFromBallPosition(
    const Point &ball_position) const
{
    return createFromShape(Circle(ball_position, BALL_MAX_RADIUS_METERS));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromRobotPosition(
    const Point &robot_position, const Point& ball_position) const
{
    auto circle_around_robot = createFromShape(Circle(robot_position, ROBOT_MAX_RADIUS_METERS));
    const Vector ball_to_robot = robot_position - ball_position;
    const Point slice_tip = ball_position + ball_to_robot.normalize(BALL_MAX_RADIUS_METERS);
    if (circle_around_robot->contains(slice_tip)) {
        // TODO: we should leverage existing functions that inflate here instead of
        //       directly using the inflation factor
        return std::make_shared<CircleWithSliceRemoved>(robot_position, ROBOT_MAX_RADIUS_METERS + robot_radius_expansion_amount, slice_tip, Angle::half());
    }
    return circle_around_robot;
}

ObstaclePtr RobotNavigationObstacleFactory::createFromRobotPosition(
        const Point &robot_position) const
{
    // TODO: call this override from the other one?
    return createFromShape(Circle(robot_position, ROBOT_MAX_RADIUS_METERS));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromShape(const Circle &circle) const
{
    return std::make_shared<GeomObstacle<Circle>>(
        Circle(circle.origin(), circle.radius() + robot_radius_expansion_amount));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromShape(const Polygon &polygon) const
{
    return std::make_shared<GeomObstacle<Polygon>>(
        polygon.expand(Vector(-1, 0).normalize(robot_radius_expansion_amount))
            .expand(Vector(1, 0).normalize(robot_radius_expansion_amount))
            .expand(Vector(0, -1).normalize(robot_radius_expansion_amount))
            .expand(Vector(0, 1).normalize(robot_radius_expansion_amount)));
}

ObstaclePtr RobotNavigationObstacleFactory::createFromFieldRectangle(
    const Rectangle &field_rectangle, const Rectangle &field_lines,
    const Rectangle &field_boundary, double additional_expansion_amount) const
{
    double xMin             = field_rectangle.xMin();
    double xMax             = field_rectangle.xMax();
    double yMin             = field_rectangle.yMin();
    double yMax             = field_rectangle.yMax();
    double expansion_amount = robot_radius_expansion_amount + additional_expansion_amount;

    xMin =
        (xMin == field_lines.xMin()) ? field_boundary.xMin() : (xMin - expansion_amount);
    xMax =
        (xMax == field_lines.xMax()) ? field_boundary.xMax() : (xMax + expansion_amount);
    yMin =
        (yMin == field_lines.yMin()) ? field_boundary.yMin() : (yMin - expansion_amount);
    yMax =
        (yMax == field_lines.yMax()) ? field_boundary.yMax() : (yMax + expansion_amount);

    return std::make_shared<GeomObstacle<Polygon>>(
        Rectangle(Point(xMin, yMin), Point(xMax, yMax)));
}
