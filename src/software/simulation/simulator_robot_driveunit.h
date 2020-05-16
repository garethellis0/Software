#include "software/new_geom/angular_velocity.h"
#include "software/new_geom/vector.h"

/**
 * A simulated Drive Unit for a robot. The drive unit encompasses everything from voltage
 * being applied to the motor to the force applied to the robot via the wheels
 */
class SimulatorRobotDriveUnit
{
   public:
    struct MotorConstants
    {
        double volts_per_rad_per_second;
        double newton_meters_per_amp;
        double resistance_ohms;
        double inductance_henries;
    };

    explicit SimulatorRobotDriveUnit() = delete;

    /**
     * Create a drive unit for a Simulator Robot
     *
     * @param motor_constants Constants defining the motor in this drive unit
     * @param motor_rotations_per_wheel_rotation The gear ratio between the motor and
     *                                           the wheel
     * @param wheel_radius_meters The radius of the wheel, in meters
     */
    SimulatorRobotDriveUnit(Vector wheel_center_position, MotorConstants motor_constants,
                            double motor_rotations_per_wheel_rotation,
                            double wheel_radius_meters);

    void setLinearVelocity(Vector linear_velocity);

    void setAngularVelocity(AngularVelocity angular_velocity);

    void setVoltageAppliedToMotor(double voltage);

        double getForceAppliedByWheelFromVoltage(Vector robot_linear_velocity,
                                                 AngularVelocity robot_angular_velocity,
                                                 double voltage_applied_to_motor);

   private:
    // TODO: jdocs for these?
    Vector wheel_center_position;
    MotorConstants motor_constants;
    double motor_rotations_per_wheel_rotation;
    double wheel_radius_meters;
    // TODO: better name for this? It's accurate but confusing........
    double current_current_amps;
};