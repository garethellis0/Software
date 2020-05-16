#include "software/simulation/simulator_robot_driveunit.h"

#include <gtest/gtest.h>

class SimulatorRobotDriveUnitTest : public testing::Test
{
   protected:
    SimulatorRobotDriveUnit simulateDriving(
        std::vector<std::tuple<Vector, AngularVelocity, double>>
            linear_vel_and_ang_vel_and_voltage,
        double timestep = 1e-3);
};

// Test that the there is no voltage drop from back-emf if the wheel is travelling
// perpendicular to the direction it's facing
TEST_F(SimulatorRobotDriveUnitTest, wheel_pointing_perpendicular_to_direction_of_travel){
}

// Test where the wheel is already moving in the direction we're applying force in
TEST_F(SimulatorRobotDriveUnitTest, wheel_pointing_parallel_to_direction_of_travel){
}

// Test where the wheel is moving against the direction we're applying force in
