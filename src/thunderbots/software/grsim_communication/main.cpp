#include <ros/ros.h>
#include <thunderbots_msgs/Primitive.h>
#include <thunderbots_msgs/PrimitiveArray.h>
#include <thunderbots_msgs/World.h>
#include <shared/constants.h>

#include <boost/circular_buffer.hpp>

#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/primitive.h"
#include "ai/primitive/primitive_factory.h"
#include "grsim_communication/grsim_backend.h"
#include "util/constants.h"
#include "geom/util.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameter_utils.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/ros_messages.h"

#include <iostream>
#include <fstream>


/**
 * A sigmoid function with a given offset from 0 and rate of change
 *
 * By default this increases from -v to positive v, ie. y = 1 / (1+e^(-x))
 * To flip the sigmoid around (ie. increasing from +v to -v), subtract it from 1
 *
 * When using this function, it is strongly encouraged that you look at it's
 * implementation and go plot it, play around with the numbers a bit to really
 * understand what they're doing.
 *
 * @param v The value to evaluate over the sigmoid
 * @param offset The offset of the center of the  sigmoid from 0
 * @param sig_width The length (in either x or y) required to cause the value of the
 *                 sigmoid to go from 0.018 to 0.982
 *
 * @return A value in [0,1] that is the value of the sigmoid at the value v
 */
double sigmoid(const double& v, const double& offset,
               const double& sig_width)
{
    // This is factor that changes how quickly the sigmoid goes from 0 to 1 it. We divide
    // 8 by it because that is the distance a sigmoid function centered about 0 takes to
    // go from 0.018 to 0.982 (and that is what the `sig_width` is, as per the javadoc
    // comment for this function)
    double sig_change_factor = 8 / sig_width;

    return 1 / (1 + std::exp(sig_change_factor * (offset - v)));
}

class Plant {
public:
    Plant(Vector initial_vel, Timestamp initial_time)  :
            curr_velocity(initial_vel),
            prev_update_time(initial_time)
    {}

    /**
     * Gets the current robot velocity
     * @param desired_acceleration
     * @return The new velocity of the robot
     */
    Vector getUpdatedVelocity(Vector desired_acceleration, Timestamp curr_time){

        // Re-scale the desired acceleration to account for wheel-slip
        double wheel_slip_multiplier = sigmoid(
                desired_acceleration.len(),
                MAX_ACCELERATION + MAX_ACCELERATION_SIGMOID_WIDTH/2,
                MAX_ACCELERATION_SIGMOID_WIDTH
        );
        desired_acceleration.norm(wheel_slip_multiplier*desired_acceleration.len());

        // Slow down due to friction, friction force is in newtons and acts against
        // the direction of robot motion
        double friction_force = FIELD_FRICTION_MU * ROBOT_MASS_KG * GRAVITY_N;
//        double friction_acceleration_mag = friction_force / ROBOT_MASS_KG;
        double friction_acceleration_mag = FIELD_FRICTION_MU * curr_velocity.len();
        Vector friction_acceleration_vec = -curr_velocity.norm(friction_acceleration_mag);

        // Add both the desired acceleration and the frictional acceleration to the robot
        Duration dt = curr_time - prev_update_time;
        desired_acceleration.norm(
                desired_acceleration.len() * dt.getSeconds()
        );
        friction_acceleration_vec.norm(
                friction_acceleration_vec.len() * dt.getSeconds()
        );
        curr_velocity = curr_velocity + desired_acceleration + friction_acceleration_vec;
//        curr_velocity = curr_velocity + desired_acceleration;

        // Cap velocity
        curr_velocity.norm(
                std::clamp(
                        curr_velocity.len(),
                        0.0,
                        ROBOT_MAX_SPEED_METERS_PER_SECOND
                )
        );

        prev_update_time = curr_time;

        return curr_velocity;
    }

private:

    double ROBOT_MASS_KG = 2;
    double FIELD_FRICTION_MU = 0.05;
    double GRAVITY_N = 9.8;
    double MAX_ACCELERATION_SIGMOID_WIDTH = 0.5;
    double MAX_ACCELERATION = ROBOT_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED;

    Vector curr_velocity;
    Timestamp prev_update_time;
};

class Controller {
public:
    Controller(): previous_errors(100000000) {}
    /**
     * Calculate the acceleration magnitude for a given robot displacement from a target
     *
     * @param error_meters The error in meters
     * @return  The magnitude of the desired acceleration in m/s^2
     */
    double update(double error_meters, Timestamp curr_time){
        P = Util::DynamicParameters::PID::P.value();
        I = Util::DynamicParameters::PID::I.value();
        D = Util::DynamicParameters::PID::D.value();

        if(Util::DynamicParameters::PID::reset_robot.value()) {
            previous_errors.clear();
        }

        // Update the previous saved states
        previous_errors.push_front(std::make_pair(error_meters, curr_time.getSeconds()));

        double integral = 0;
        for(int i = 0; i < previous_errors.size()-1; i++){
            integral +=
                    (previous_errors[i+1].first + previous_errors[i].first)/2 *
                    (previous_errors[i].second - previous_errors[i+1].second);
        }
//        if (previous_errors.size() > 1){
//            integral /= (previous_errors.front().second - previous_errors.back().second);
//        }

        double derivative = 0;
        if (previous_errors.size() > 2){
            derivative = (previous_errors[0].first - previous_errors[1].first) /
                    (previous_errors[0].second - previous_errors[1].second);
        }

//        std::cout << integral << std::endl;

        return P*error_meters + I*integral + D*derivative;
    }

private:
    double P = 0; // 0.15
    double I = 0; // 0.12
    double D = 0; // 0,08

    boost::circular_buffer<std::pair<double, double>> previous_errors;
};


// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace
{
    // The GrSimBackend responsible for handling communication with grSim
    GrSimBackend grsim_backend(Util::Constants::GRSIM_COMMAND_NETWORK_ADDRESS,
                               Util::Constants::GRSIM_COMMAND_NETWORK_PORT);
    // The current state of the world
    World world;

    // The plant for robot control
    Plant robot_plant({0,0}, Timestamp::fromSeconds(0));

    // The controller for robot control
    Controller robot_controller;

    // The goal destination for the robot
    Point desired_position({0,0});
    Point starting_position({0, 0});

    // The last timestamp from a robot update that we care about
    Timestamp last_robot_update_time = Timestamp::fromSeconds(0);
    std::ofstream outfile;
    Timestamp starting_time;

}  // namespace

void primitiveUpdateCallback(const thunderbots_msgs::PrimitiveArray::ConstPtr& msg)
{
    std::vector<std::unique_ptr<Primitive>> primitives;
    thunderbots_msgs::PrimitiveArray prim_array_msg = *msg;
    for (const thunderbots_msgs::Primitive& prim_msg : prim_array_msg.primitives)
    {
        primitives.emplace_back(AI::Primitive::createPrimitiveFromROSMessage(prim_msg));
    }

//    grsim_backend.sendPrimitives(primitives, world.friendlyTeam(), world.ball());
}

void worldUpdateCallback(const thunderbots_msgs::World::ConstPtr& msg)
{
    thunderbots_msgs::World world_msg = *msg;
    world = Util::ROSMessages::createWorldFromROSMessage(world_msg);

    if(Util::DynamicParameters::PID::logenable.value() && Util::DynamicParameters::PID::reset_robot.value()) {
        if(outfile.is_open()){
            outfile.close();
            std::cout << "closing file" << std::endl;
        }
        outfile.open(Util::DynamicParameters::PID::logfile.value());
        std::cout << "opening file" << std::endl;
    }
    if(!Util::DynamicParameters::PID::logenable.value()) {
        if(outfile.is_open()){
            outfile.close();
            std::cout << "closing file" << std::endl;
        }
    }

    auto robot = world.friendlyTeam().getRobotById(0);
    if (robot && robot->lastUpdateTimestamp() > last_robot_update_time){
        if(Util::DynamicParameters::PID::reset_robot.value()) {
            grsim_backend.setRobot(0, true, Point(-5, 0), Angle::zero());
            starting_position = robot->position();
            starting_time = robot->lastUpdateTimestamp();
        }

        last_robot_update_time = robot->lastUpdateTimestamp();
        Timestamp curr_time = robot->lastUpdateTimestamp();
        double error_meters = (starting_position - desired_position).len() - (starting_position - robot->position()).len();
        Vector acc_dir = desired_position - starting_position;
        double acc_mag = robot_controller.update(error_meters, curr_time);
        Vector new_acc_dir = acc_dir.norm(std::fabs(acc_mag));
        if(acc_mag < 0.0) {
            new_acc_dir = new_acc_dir.rotate(Angle::half());
        }
        Vector robot_velocity = robot_plant.getUpdatedVelocity(new_acc_dir, curr_time);

        robot_velocity = robot_velocity.rotate(-robot->orientation());

        auto packet = grsim_backend.createGrSimPacketWithRobotVelocity(
                0, TeamColour::YELLOW, robot_velocity, AngularVelocity::zero(), 0, false, false);
        grsim_backend.sendGrSimPacket(packet);

        outfile << (robot->lastUpdateTimestamp() - starting_time).getSeconds() << ", "
                << robot->position().x() << ", "
                << robot->position().y() << ", "
                << 0.0 << ", "
                << error_meters << std::endl;
    }
}

int main(int argc, char** argv)
{
    // Init ROS node
    ros::init(argc, argv, "grsim_communication");
    ros::NodeHandle node_handle;



    // Create subscribers to topics we care about
    //ros::Subscriber primitive_subscriber = node_handle.subscribe(
    //    Util::Constants::AI_PRIMITIVES_TOPIC, 1, primitiveUpdateCallback);
    ros::Subscriber world_subscriber = node_handle.subscribe(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1, worldUpdateCallback);

    // Initialize the logger
    Util::Logger::LoggerSingleton::initializeLogger(node_handle);

    // Initialize Dynamic Parameters
    auto update_subscribers =
        Util::DynamicParameters::initUpdateSubscriptions(node_handle);

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin();

    return 0;
}
