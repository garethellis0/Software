#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/motion_constraint/motion_constraint.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/point.h"
#include "software/logger/logger.h"
#include "software/world/world.h"

/**
 * The RobotNavigationObstacleFactory creates obstacles for navigation with a robot
 * NOTE: All obstacles created include at least an additional robot radius margin on all
 * sides of the obstacle
 */
class RobotNavigationObstacleFactory
{
   public:
    RobotNavigationObstacleFactory() = delete;

    /**
     * Create an RobotNavigationObstacleFactory with the given configuration
     *
     * @param config The configuration used to determine how obstacles should be generated
     */
    RobotNavigationObstacleFactory(
        std::shared_ptr<const RobotNavigationObstacleConfig> config);

    /**
     * Create obstacles for the given motion constraints
     *
     * @param motion_constraints The motion constraints to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraints
     */
    std::vector<ObstaclePtr> createFromMotionConstraints(
        const std::set<MotionConstraint> &motion_constraints, const World &world) const;

    /**
     * Create obstacles for the given motion constraint
     *
     * @param motion_constraint The motion constraint to create obstacles for
     * @param world World we're enforcing motion constraints in
     *
     * @return Obstacles representing the given motion constraint
     */
    std::vector<ObstaclePtr> createFromMotionConstraint(
        const MotionConstraint &motion_constraint, const World &world) const;

    /**
     * Create an obstacle representing the given robot
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     * If the robot is moving slowly, the obstacle is also shaped around the ball so that
     * we aren't prevented from getting the ball just because it's near an enemy.
     *
     * @param robot The robot to get a representative obstacle for
     * @param ball A ball that could be near the robot.
     *
     * @return An obstacle representing the given robot, shaped to still allow access
     *         to the ball.
     */
    ObstaclePtr createFromRobot(const Robot &robot, const std::optional<Ball>& ball) const;

    /**
     * Create a list of obstacles representing the given team
     *
     * These obstacles take into account the velocity of the robot to extend the
     * created obstacle in the robot's direction of travel.
     * The obstacles are also shaped to allow access to the ball.
     *
     * @param team The team to get representative obstacles for
     * @param ball A ball that robot obstacles should be shaped to allow access to (if
     *             they're moving slow)
     *
     * @return A list of obstacles representing the given team
     */
    std::vector<ObstaclePtr> createFromTeam(const Team &team, const Ball& ball) const;

    /**
     * Create circle obstacle around robot with additional radius scaling
     *
     * @param robot_position robot position
     * @param ball_position If this is within the robot obstacle, we remove a slice so
     *                      that we aren't prevented from navigating to the ball.
     *
     * @return obstacle around the robot
     */
     // TODO: just combine the overrides and have the ball be optional
    ObstaclePtr createFromRobotPosition(const Point &robot_position, const Point& ball_position) const;

    /**
     * Create circle obstacle around robot with additional radius scaling
     *
     * @param robot_position robot position
     *
     * @return obstacle around the robot
     */
    ObstaclePtr createFromRobotPosition(const Point &robot_position) const;

    /**
     * Create circle obstacle around ball
     *
     * @param ball_position ball position to make obstacle around
     *
     * @return obstacle around the ball
     */
    ObstaclePtr createFromBallPosition(const Point &ball_position) const;

    /**
     * Returns an obstacle for the shape
     * NOTE: as with all other obstacles created by RobotNavigationObstacleFactory, the
     * shapes are expanded on all sides to account for the radius of the robot
     *
     * @param The shape to expand
     *
     * @return ObstaclePtr
     */
    ObstaclePtr createFromShape(const Circle &circle) const;
    ObstaclePtr createFromShape(const Polygon &polygon) const;

   private:
    std::shared_ptr<const RobotNavigationObstacleConfig> config;
    double robot_radius_expansion_amount;

    /**
     * Returns an obstacle for the field_rectangle expanded on all sides to account for
     * the size of the robot. If a side of the field_rectangle lies along a field line,
     * then it is expanded out to the field boundary
     *
     * @param field_rectangle The rectangle to make obstacle
     * @param field_lines The rectangle representing field lines
     * @param field_boundary The rectangle representing field boundary
     * @param additional_expansion_amount (optional) The amount to expand all sides of the
     * rectangle in addition to the robot radius
     *
     * @return ObstaclePtr
     */
    ObstaclePtr createFromFieldRectangle(const Rectangle &field_rectangle,
                                         const Rectangle &field_lines,
                                         const Rectangle &field_boundary,
                                         double additional_expansion_amount = 0.0) const;
};
