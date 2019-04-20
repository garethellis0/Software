#ifndef PROJECT_CALC_BEST_SHOT_H
#define PROJECT_CALC_BEST_SHOT_H

#include "ai/world/field.h"
#include "ai/world/robot.h"
#include "ai/world/world.h"
#include "geom/point.h"

namespace Evaluation
{
    /**
     * Finds the best point in the enemy goal to shoot at
     *
     * @param field The field on which the shot is taking place
     * @param obstacles A list of all the obstacles which could impede the ball
     * @param shot_start The point that the shot starts from
     * @param radius The radius of all obstacles
     *
     * @return The point in the enemy goal to shoot at, and the size of the open angular
     *         interval through which the shot will go. A larger open angle generally
     *         indicates a safer shot. Returns std::nullopt if no shot found.
     */
    std::optional<std::pair<Point, Angle>> calcBestShotOnEnemyGoal(const Field &field,
                                                    const std::vector<Point> &obstacles,
                                                    const Point &shot_start, double radius);

    /**
     * Finds the length of the all continuous interval (angle-wise) of the
     * goal that can be seen from a point.
     *
     * @param field The field on which the shot is taking place
     * @param obstacles A list of all the obstacles which could impede the ball
     * @param shot_start The point that the shot starts from
     * @param radius The radius of all obstacles
     *
     * @return A set of points in the enemy goal to shoot at, and the size of the open
     *         angular interval for each shot. A larger open angle generally indicates a
     *         safer shot.
     */
    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(
        const Field &field, const std::vector<Point> &obstacles, const Point &shot_start,
        double radius);

    // TODO: This should take a radius again.........
    /**
     * Finds the length of the largest continuous interval (angle-wise) of the
     * goal that can be seen from a point.
     *
     * @param field The field the shot is being performed on
     * @param friendly_team The friendly robots
     * @param enemy_team The enemy robots
     * @param shooting_robot The robot performing the shot (this is assumed to be on the
     *                       friendly team)
     *
     * @return The point in the enemy goal to shoot at, and the size of the open angular
     *         interval through which the shot will go. A larger open angle generally
     *         indicates a safer shot. Returns std::nullopt if no shot found.
     */
    std::optional<std::pair<Point, Angle>> calcBestShotOnEnemyGoal(const Field &field, const Team &friendly_team, const Team &enemy_team, const Robot &shooting_robot);


    // TODO: This should take a radius again.........
    /**
     * Finds a list of points that we can shoot at in the enemy goal
     *
     * @param field The field the shot is being performed on
     * @param friendly_team The friendly robots
     * @param enemy_team The enemy robots
     * @param shooting_robot The robot performing the shot (this is assumed to be on the
     *                       friendly team)
     *
     * @return A set of points in the enemy goal to shoot at, and the size of the open
     *         angular interval for each shot. A larger open angle generally indicates a
     *         safer shot.
     */
    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(const Field &field, const Team &friendly_team, const Team &enemy_team, const Robot &shooting_robot);

}  // namespace Evaluation
#endif
