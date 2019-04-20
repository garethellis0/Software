#include "shared/constants.h"
#include "ai/hl/stp/evaluation/calc_best_shot.h"

#include "geom/util.h"

namespace Evaluation
{
    std::optional<std::pair<Point, Angle>> calcBestShotOnEnemyGoal(const Field &field,
                                                    const std::vector<Point> &obstacles,
                                                    const Point &shot_start, double radius)
    {
        const Point goal_post_neg = field.enemyGoalpostNeg();
        const Point goal_post_pos = field.enemyGoalpostPos();
        return angleSweepCircles(shot_start, goal_post_neg, goal_post_pos, obstacles, radius);
    }

    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(
        const Field &field, const std::vector<Point> &obstacles, const Point &p,
        double radius)
    {
        const Point goal_post_neg = field.enemyGoalpostNeg();
        const Point goal_post_pos = field.enemyGoalpostPos();
        return angleSweepCirclesAll(p, goal_post_neg, goal_post_pos, obstacles, radius);
    }

    std::optional<std::pair<Point, Angle>> calcBestShotOnEnemyGoal(const Field &field, const Team &friendly_team, const Team &enemy_team, const Robot &shooting_robot)
    {
        std::vector<Point> obstacles;
        // create a vector of points for all the robots except the shooting one
        for (const Robot &i : enemy_team.getAllRobots())
        {
            obstacles.emplace_back(i.position());
        }
        for (const Robot &fpl : friendly_team.getAllRobots())
        {
            if (fpl.id() == shooting_robot.id()) {
                // Skip over the robot performing the shot
                continue;
            }
            obstacles.emplace_back(fpl.position());
        }

        return calcBestShotOnEnemyGoal(field, obstacles, shooting_robot.position(), ROBOT_MAX_RADIUS_METERS);
    }

    std::vector<std::pair<Point, Angle>> calcBestShotOnEnemyGoalAll(const Field &field, const Team &friendly_team, const Team &enemy_team, const Robot &shooting_robot)
    {
        std::vector<Point> obstacles;
        for (const Robot &fpl : friendly_team.getAllRobots())
        {
            if (fpl.id() == shooting_robot.id())
            {
                // Skip over the robot performing the shot
                continue;
            }
            obstacles.push_back(fpl.position());
        }
        for (const Robot &i : enemy_team.getAllRobots())
        {
            obstacles.push_back(i.position());
        }
        return calcBestShotOnEnemyGoalAll(field, obstacles, shooting_robot.position(), ROBOT_MAX_RADIUS_METERS);
    }

}  // namespace Evaluation
