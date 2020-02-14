#include "firmware/main/app/primitives/spin_primitive.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "firmware/main/app/control/bangbang.h"
#include "firmware/main/app/control/control.h"
#include "firmware/main/app/control/physbot.h"
#include "firmware/main/math/polynomial_2d.h"
#include "firmware/main/math/tbots_math.h"
#include "firmware/main/math/vector_2d.h"
#include "firmware/main/shared/physics.h"

// The maximum number of points we might want to track
#define MAX_NUM_TRACKING_POINTS 100

typedef struct SpinPrimitiveState
{
    // TODO: create a timestamp struct
    Vector2d_t points_to_track[MAX_NUM_TRACKING_POINTS];
    float timestamps_for_points_to_track[MAX_NUM_TRACKING_POINTS];

    size_t num_points_to_track;
} SpinPrimitiveState_t;
DEFINE_PRIMITIVE_STATE_CREATE_AND_DESTROY_FUNCTIONS(SpinPrimitiveState_t)

static void spin_start(const primitive_params_t *p, void *void_state_ptr,
                       FirmwareWorld_t *world)
{
    // Ignore all parameters, we're hijacking this primitive for trajectory
    // planner testing
    SpinPrimitiveState_t *state = (SpinPrimitiveState_t *)void_state_ptr;

    Polynomial2dOrder2_t poly = {.x = {.coefficients = {0, 1, 0}},
                                 .y = {.coefficients = {1, 0, 0}}};

    state->num_points_to_track = app_control_generateTrackingPointsForPoly(
        world, poly, state->points_to_track, state->timestamps_for_points_to_track,
        MAX_NUM_TRACKING_POINTS);

    assert(state->num_points_to_track <= MAX_NUM_TRACKING_POINTS);
}

static void spin_end(void *void_state_ptr, FirmwareWorld_t *world) {}

static void spin_tick(void *void_state_ptr, FirmwareWorld_t *world)
{
    const FirmwareRobot_t *robot      = app_firmware_world_getRobot(world);
    const SpinPrimitiveState_t *state = (SpinPrimitiveState_t *)void_state_ptr;
    const float current_time_seconds  = app_firmware_world_getCurrentTimeInSeconds(world);

    // Check if this trajectory is in the past. If it is then we have nothing reasonable
    // to do
    const float last_tracking_point_timestamp_seconds =
        state->timestamps_for_points_to_track[state->num_points_to_track - 1];
    if (current_time_seconds >= last_tracking_point_timestamp_seconds)
    {
        app_wheel_coast(app_firmware_robot_getFrontLeftWheel(robot));
        app_wheel_coast(app_firmware_robot_getFrontRightWheel(robot));
        app_wheel_coast(app_firmware_robot_getBackLeftWheel(robot));
        app_wheel_coast(app_firmware_robot_getBackRightWheel(robot));
        return;
    }

    // Figure out what point we should be tracking. We should really do this via
    // a binary search, or better yet, save the last point we tracked, but this
    // is a hacky prototype, so we don't care for now.
    size_t curr_tracking_point_index = 1;
    while (curr_tracking_point_index < state->num_points_to_track &&
           state->timestamps_for_points_to_track[curr_tracking_point_index - 1] >=
               current_time_seconds)
    {
        curr_tracking_point_index++;
    }

    // TODO: we should never have to do these vector operations manually
    Vector2d_t point_to_track = state->points_to_track[curr_tracking_point_index];
    const float dx            = point_to_track.x - app_firmware_robot_getPositionX(robot);
    const float dy            = point_to_track.y - app_firmware_robot_getPositionY(robot);
    const float total_disp    = sqrtf(dx * dx + dy * dy);
    float major_vec[2];
    float minor_vec[2];
    major_vec[0] = dx / total_disp;
    major_vec[1] = dy / total_disp;
    minor_vec[0] = major_vec[0];
    minor_vec[1] = major_vec[1];
    rotate(minor_vec, P_PI / 2);

    float point_to_track_array[2];
    point_to_track_array[0] = point_to_track.x;
    point_to_track_array[1] = point_to_track.y;

    PhysBot pb = app_physbot_create(robot, point_to_track_array, major_vec, minor_vec);

    float max_major_a     = 3.5;
    float max_major_v     = 3.0;
    float major_params[3] = {0, max_major_a, max_major_v};
    app_physbot_planMove(&pb.maj, major_params);

    // plan minor axis movement
    float max_minor_a     = 1.5;
    float max_minor_v     = 1.5;
    float minor_params[3] = {0, max_minor_a, max_minor_v};
    app_physbot_planMove(&pb.min, minor_params);

    // plan rotation movement
    plan_move_rotation(&pb, app_firmware_robot_getVelocityAngular(robot));

    float accel[3] = {0, 0, pb.rot.accel};

    // rotate the accel and apply it
    app_physbot_computeAccelInLocalCoordinates(
        accel, pb, app_firmware_robot_getOrientation(robot), major_vec, minor_vec);

    app_control_applyAccel(robot, accel[0], accel[1], accel[2]);
}

/**
 * \brief The spin movement primitive.
 */
const primitive_t SPIN_PRIMITIVE = {.direct        = false,
                                    .start         = &spin_start,
                                    .end           = &spin_end,
                                    .tick          = &spin_tick,
                                    .create_state  = &createSpinPrimitiveState_t,
                                    .destroy_state = &destroySpinPrimitiveState_t};
