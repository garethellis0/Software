#include <iostream>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <g3log/g3log.hpp>

#include "software/backend/backend.h"
#include "software/backend/radio_backend.h"

struct RobotStateQueueStruct {
    double x_pos;
    double y_pos;
    double yaw;
};

struct RobotWheelCommands {
  double front_right_rpm;
  double front_left_rpm;
  double front_right_rpm;
  double front_left_rpm;
}

// TODO: put this in another file (.c and .h)
// TODO: better name for this class
class RobotInterface : public ThreadedObserver<World>
{
   public:
    RobotInterface() = delete;

    /**
     * Create a RobotInterface
     *
     * @param robot_state_ipc_queue_name The name of boost interprocess queue
     *                                   that we should push robot state 
     *                                   updates to
     * @param robot_wheel_commands_ipc_queue_name The name of the boost 
     *                                            interprocess queue that we
     *                                            should get wheel commands to
     *                                            send to the robot from
     *
     */
    RobotInterface(std::string robot_state_ipc_queue_name, 
        std::string robot_wheel_commands_ipc_queue_name) :
      robot_state_message_queue_name(robot_state_ipc_queue_name),
      robot_wheel_commands_message_queue_name(robot_wheel_commands_ipc_queue_name),
      robot_state_message_queue(boost::interprocess::open_or_create, 
          robot_state_ipc_queue_name.c_str(), 
          MAX_QUEUE_SIZE, MAX_MSG_SIZE_BYTES),
      robot_wheel_commands_message_queue(boost::interprocess::open_or_create, 
          robot_wheel_commands_ipc_queue_name.c_str(), 
          MAX_QUEUE_SIZE, MAX_MSG_SIZE_BYTES)
    {
      receive_wheel_speeds_thread = std::thread(
          [this]() { return receiveWheelSpeedsLoop(); });
    }

    ~RobotInterface(){

      // Stop the thread that's constantly pulling robot wheel_commands
      _in_destructor = true;
      receive_wheel_speeds_thread.join();


      // Close message queues
      boost::interprocess::message_queue::remove(robot_state_message_queue_name.c_str());
      boost::interprocess::message_queue::remove(robot_wheel_commands_message_queue_name.c_str());
    }

   private:

    static const int MAX_QUEUE_SIZE = 100;
    static const int MAX_MSG_SIZE_BYTES = 100;

    static const double RECEIVE_WHEEL_SPEEDS_TIMEOUT_MS = 100;

    /**
     * This function will be called with new worlds when they are received
     * @param world
     */
    void onValueReceived(World world)
    {
      auto friendly_team = world.friendlyTeam();
      auto robots = friendly_team.getAllRobots();
      if (robots.size() < 1){
        LOG(WARNING) << "No robots visible!";
        return;
      }
      
      auto robot = robots[0];
      RobotStateQueueStruct queue_elem {
        .x_pos = robot.position().x(),
        .y_pos = robot.position().y(),
        .yaw = robot.orientation().toRadians(),
      }; 

      robot_state_message_queue.send(&queue_elem, sizeof(queue_elem), 0);
    }

    /**
     * An infinite loop that receives wheel speeds and pushes them to the robot
     */
    void receiveWheelSpeedsLoop(){
      while(!_in_destructor){

        // We use the version of receive with a timeout so that we can
        // regularly check if this class was destructed, and stop the loop
        // if so
        ptime t_timeout = boost::microsec_clock::universal_time() + 
          boost::milliseconds(RECEIVE_WHEEL_SPEEDS_TIMEOUT_MS);
        RobotWheelCommands wheel_commands;
        size_t received_size = 0;

        if (robot_wheel_commands_message_queue.timed_received(
              &wheel_commands, sizeof(wheel_commands), 
              received_size, 0, t_timeout)){
          std::cout << wheel_commands << std::endl;
          // TODO: actually send these to the robot
        }
      }
    }

    ///**
    // * This function handles new wheel commands when they are received
    // * @param wheel_commands The wheel commands to execute on the robot
    // */
    //void wheelCommandsCallback(const mpc_messages::WheelSpeedsConstPtr wheel_commands)
    //{
    //    // TODO: implement me
    //}

    // The message queue to push robot state updates to, and it's name
    boost::interprocess::message_queue robot_state_message_queue;
    std::string robot_state_message_queue_name;

    // The message queue to read wheel commands from, and it's name
    boost::interprocess::message_queue robot_wheel_commands_message_queue;
    std::string robot_wheel_commands_message_queue_name;

    std::atomic<bool> _in_destructor;

    std::thread receive_wheel_speeds_thread; 
};

int main(int argc, char** argv)
{

    auto world_observer = std::make_shared<RobotInterface>(
        "robot_state", "robot_wheel_commands");

    std::unique_ptr<Backend> backend = std::make_unique<RadioBackend>();

    backend->Subject<World>::registerObserver(world_observer);

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();

    return 0;
}
