#include <iostream>

#include "ros-test/mpc_messages/RobotState.h"
#include "ros-test/mpc_messages/WheelSpeeds.h"
#include <ros/ros.h>
#include "software/backend/backend.h"

// TODO: put this in another file (.c and .h)
// TODO: better name for this class
class RobotInterface : public ThreadedObserver<World>
{
   public:
    RobotInterface() = delete;

    /**
     * Create a RobotInterface
     *
     * @param node_handle A node handle that can be used to interact with ROS
     * @param robot_state_topic_name The name of the topic that we should
     *                               publish robot state updates to
     * @param wheel_commands_topic_name The name of the topic that we should
     *                                  read wheel commands from
     */
    RobotInterface(ros::NodeHandle nodehandle, std::string robot_state_topic_name,
                  std::string wheel_commands_topic_name)
    {
        robot_state_publisher =
            nodehandle.advertise<mpc_messages::RobotState>(robot_state_topic_name, 2);
        wheel_commands_subscriber = nodehandle.subscribe(
            wheel_commands_topic_name, 2, &RobotInterface::wheelCommandsCallback, this);
    }

   private:
    /**
     * This function will be called with new worlds when they are received
     * @param world
     */
    void onValueReceived(World world)
    {
        // TODO: implement me
    }

    /**
     * This function handles new wheel commands when they are received
     * @param wheel_commands The wheel commands to execute on the robot
     */
    void wheelCommandsCallback(const mpc_messages::WheelSpeedsConstPtr wheel_commands)
    {
        // TODO: implement me
    }

    ros::Publisher robot_state_publisher;
    ros::Subscriber wheel_commands_subscriber;
};

int main(int argc, char** argv)
{
    // TODO: delete this
    //    mpc_messages::WheelSpeeds wheel_speeds;
    //
    //    wheel_speeds.wheel_1_speed = 1;
    //    wheel_speeds.wheel_2_speed = 3;
    //    wheel_speeds.wheel_3_speed = 6;
    //    wheel_speeds.wheel_4_speed = 9;
    //    std::cout << wheel_speeds << std::endl;

    ros::init(argc, argv, "robot_interface");
    ros::NodeHandle node_handle;

    auto world_observer = std::make_shared<RobotInterface>(node_handle, "robot_state",
                                                          "robot_wheel_commands");

    // TODO: actually need to set this to something
    std::unique_ptr<Backend> backend;

    backend->Subject<World>::registerObserver(world_observer);

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin();

    return 0;
}
