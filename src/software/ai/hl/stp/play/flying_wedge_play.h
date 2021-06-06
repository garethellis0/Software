#pragma once

#include "software/ai/hl/stp/play/play.h"

/**
 * A play where several robots drive forwards to "clear the way" for a single
 * robot behind them that's dribbling the ball
 */
class FlyingWedgePlay : public Play {
public:
    FlyingWedgePlay(std::shared_ptr<const PlayConfig> config);

    bool isApplicable(const World &world) const override;

    bool invariantHolds(const World &world) const override;

    void getNextTactics(TacticCoroutine::push_type &yield, const World &world) override;

    // The offset of each defender from the dribbling robot, perpendicular to
    // the dribbler's direction of motion
    const double WEDGE_DEFENDER_LATERAL_OFFSET = ROBOT_MAX_RADIUS_METERS * 1.5;
    const double WEDGE_DEFENDER_LONGITUDINAL_OFFSET = ROBOT_MAX_RADIUS_METERS * 4;
};