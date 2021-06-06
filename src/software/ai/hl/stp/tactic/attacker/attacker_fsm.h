#pragma once

#include "shared/parameter/cpp_dynamic_parameters.h"
#include "software/ai/evaluation/shot.h"
#include "software/ai/hl/stp/tactic/chip/chip_fsm.h"
#include "software/ai/hl/stp/tactic/pivot_kick/pivot_kick_fsm.h"
#include "software/ai/hl/stp/tactic/tactic.h"
#include "software/ai/intent/move_intent.h"
#include "software/ai/passing/pass.h"
#include "software/ai/evaluation/calc_best_shot.h"

struct AttackerFSM
{
    struct ControlParams
    {
        // The pass to execute
        std::optional<Pass> pass = std::nullopt;
        // The shot to take
        std::optional<Shot> shot = std::nullopt;
        // The point the robot will chip towards if it is unable to shoot and is in danger
        // of losing the ball to an enemy
        std::optional<Point> chip_target;
        // shoot goal config
        std::shared_ptr<const AttackerTacticConfig> attacker_tactic_config;
    };

    DEFINE_UPDATE_STRUCT_WITH_CONTROL_AND_COMMON_PARAMS

    auto operator()()
    {
        using namespace boost::sml;

        const auto pivot_kick_s = state<PivotKickFSM>;
        const auto keep_away_s  = state<DribbleFSM>;
        const auto update_e     = event<Update>;

        /**
         * Action that updates the PivotKickFSM to shoot or pass
         *
         * @param event AttackerFSM::Update event
         * @param processEvent processes the PivotKickFSM::Update
         */
        const auto pivot_kick = [](auto event,
                                   back::process<PivotKickFSM::Update> processEvent) {
            auto ball_position = event.common.world.ball().position();
            Point chip_target  = event.common.world.field().enemyGoalCenter();
            if (event.control_params.chip_target)
            {
                chip_target = event.control_params.chip_target.value();
            }
            // default to chipping the ball away
            PivotKickFSM::ControlParams control_params{
                .kick_origin    = ball_position,
                .kick_direction = (chip_target - ball_position).orientation(),
                .auto_chip_or_kick =
                    AutoChipOrKick{AutoChipOrKickMode::AUTOCHIP,
                                   (chip_target - ball_position).length()}};

            if (event.control_params.shot)
            {
                // shoot on net
                control_params = PivotKickFSM::ControlParams{
                    .kick_origin = ball_position,
                    .kick_direction =
                        (event.control_params.shot->getPointToShootAt() - ball_position)
                            .orientation(),
                    .auto_chip_or_kick =
                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                       BALL_MAX_SPEED_METERS_PER_SECOND - 0.5}};
            }
            else if (event.control_params.pass)
            {
                control_params = PivotKickFSM::ControlParams{
                    .kick_origin    = event.control_params.pass->passerPoint(),
                    .kick_direction = event.control_params.pass->passerOrientation(),
                    .auto_chip_or_kick =
                        AutoChipOrKick{AutoChipOrKickMode::AUTOKICK,
                                       event.control_params.pass->speed()}};
            }
            processEvent(PivotKickFSM::Update(control_params, event.common));
        };

        /**
         * Action that updates the DribbleFSM to keep the ball away
         *
         * @param event AttackerFSM::Update event
         * @param processEvent processes the DribbleFSM::Update
         */
        const auto keep_away = [](auto event,
                                  back::process<DribbleFSM::Update> processEvent) {
            // Sample shots in a rectangle around the robot and move to the best one
            // We save the shot origin and shot
            std::vector<std::pair<Point, Shot>> shots;
            static constexpr double X_RESOLUTION = 20;
            static constexpr double Y_RESOLUTION = 20;
            for (int i = 0; i < X_RESOLUTION; i ++) {
                for (int j = 0; j < Y_RESOLUTION; j ++) {
                    static constexpr double MAX_X_OFFSET = 2;
                    static constexpr double MAX_Y_OFFSET = 2.5;
                    const double x = i * (2.0 * MAX_X_OFFSET) / X_RESOLUTION;
                    const double y = j * (2.0 * MAX_Y_OFFSET) / Y_RESOLUTION;
                    const Point shot_point(x,y);
                    if(auto shot = calcBestShotOnGoal(
                            event.common.world.field(),
                            event.common.world.friendlyTeam(),
                            event.common.world.enemyTeam(),
                            shot_point,
                            TeamType::ENEMY
                            )) {
                        shots.emplace_back(std::make_pair(shot_point, shot.value()));
                    }
                }
            }
            // Take the best shot
            std::optional<std::pair<Point, Shot>> best_shot;
            for(auto& shot : shots) {
                if(!best_shot ||(shot.second.getOpenAngle() > best_shot->second.getOpenAngle())) {
                    best_shot = shot;
                }
            }
            // TODO (#2073): Implement a more effective keep away tactic
            DribbleFSM::ControlParams control_params{
                .dribble_destination       = std::nullopt,
                .final_dribble_orientation = std::nullopt,
                .allow_excessive_dribbling = false};
            if(best_shot) {
                control_params = {
                        .dribble_destination       = best_shot->first,
                        .final_dribble_orientation = (best_shot->second.getPointToShootAt() - best_shot->first).orientation(),
                        .allow_excessive_dribbling = false};
            }
            processEvent(DribbleFSM::Update(control_params, event.common));
        };

        /**
         * Guard that checks if the ball should be kicked, which is when there's a nearby
         * enemy or a good pass/shot
         *
         * @param event AttackerFSM::Update event
         *
         * @return if the ball should be kicked
         */
        const auto should_kick = [](auto event) {
            // check for enemy threat
            Circle about_to_steal_danger_zone(event.common.robot.position(),
                                              event.control_params.attacker_tactic_config
                                                  ->getEnemyAboutToStealBallRadius()
                                                  ->value());
            for (const auto &enemy : event.common.world.enemyTeam().getAllRobots())
            {
                if (contains(about_to_steal_danger_zone, enemy.position()))
                {
                    return true;
                }
            }
            // otherwise check for shot or pass
            return event.control_params.pass || event.control_params.shot;
        };

        return make_transition_table(
            // src_state + event [guard] / action = dest_state
            *keep_away_s + update_e[should_kick] / pivot_kick = pivot_kick_s,
            keep_away_s + update_e[!should_kick] / keep_away,
            pivot_kick_s + update_e / pivot_kick, pivot_kick_s = X);
    }
};
