#include <boost/interprocess/ipc/message_queue.hpp>
#include <g3log/g3log.hpp>
#include <iostream>
#include <chrono>
#include <ostream>

#include "software/backend/backend.h"
#include "software/backend/grsim_backend.h"
#include "software/backend/radio_backend.h"
#include "software/logger/init.h"

// These structs MUST be kept exactly in sync with the ones
// in the autorally code, or extremely nasty things will happen.
// TODO: put this in a shared header somewhere
struct RobotStateMsgQueueEntry
{
    double x_pos_m;
    double y_pos_m;
    double yaw_rad;

    double x_vel_m_per_s;
    double y_vel_m_per_s;
    double angular_vel_rad_per_s;

    double timestamp_secs_since_epoch;
};
std::ostream& operator<<(std::ostream& o, const RobotStateMsgQueueEntry& state)
{
    // clang-format off
    o << "x_pos_m: " << state.x_pos_m 
      << ", y_pos_m: " << state.y_pos_m 
      << ", yaw_rad: " << state.yaw_rad 
      << ", x_vel_m_per_s: " << state.x_vel_m_per_s 
      << ", y_vel_m_per_s: " << state.y_vel_m_per_s
      << ", angular_vel_rad_per_s: " << state.angular_vel_rad_per_s
      << ", timestamp_secs_since_epoch: " << state.timestamp_secs_since_epoch; 
    // clang-format on
    return o;
}
struct RobotWheelCommandsMsgQueueEntry
{
    double front_right_rad_per_s;
    double front_left_rad_per_s;
    double back_right_rad_per_s;
    double back_left_rad_per_s;

    double timestamp_secs_since_epoch;
};
std::ostream& operator<<(std::ostream& o, const RobotWheelCommandsMsgQueueEntry& cmds)
{
    // clang-format off
    o << "Front Right: "  << cmds.front_right_rad_per_s 
      << ", Front Left: " << cmds.front_left_rad_per_s
      << ", Back Right: " << cmds.back_right_rad_per_s 
      << ", Back Left: "  << cmds.back_left_rad_per_s
      << ", timestamp_secs_since_epoch: " << cmds.timestamp_secs_since_epoch; 
    // clang-format on
    return o;
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
                   std::string robot_wheel_commands_ipc_queue_name)
        : robot_state_message_queue_name(robot_state_ipc_queue_name),
          robot_wheel_commands_message_queue_name(robot_wheel_commands_ipc_queue_name),
          robot_state_message_queue(boost::interprocess::open_or_create,
                                    robot_state_ipc_queue_name.c_str(), MAX_QUEUE_SIZE,
                                    sizeof(RobotStateMsgQueueEntry)),
          robot_wheel_commands_message_queue(boost::interprocess::open_or_create,
                                             robot_wheel_commands_ipc_queue_name.c_str(),
                                             MAX_QUEUE_SIZE, 
                                             sizeof(RobotWheelCommandsMsgQueueEntry)),
          _in_destructor(false)
    {
        receive_wheel_speeds_thread =
            std::thread([this]() { return receiveWheelSpeedsLoop(); });
    }

    ~RobotInterface()
    {
        // Stop the thread that's constantly pulling robot wheel_commands
        _in_destructor = true;
        receive_wheel_speeds_thread.join();


        // Close message queues
        boost::interprocess::message_queue::remove(
            robot_state_message_queue_name.c_str());
        boost::interprocess::message_queue::remove(
            robot_wheel_commands_message_queue_name.c_str());
    }

   private:
    static const int MAX_QUEUE_SIZE     = 50;

    const int RECEIVE_WHEEL_SPEEDS_TIMEOUT_MS = 100;

    /**
     * This function will be called with new worlds when they are received
     * @param world
     */
    void onValueReceived(World world)
    {
        auto friendly_team = world.friendlyTeam();
        auto robots        = friendly_team.getAllRobots();
        if (robots.size() < 1)
        {
            LOG(WARNING) << "No robots visible!";
            return;
        }

        unsigned long current_time_since_epoch_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count();
        double current_time_since_epoch_secs = 
          static_cast<double>(current_time_since_epoch_ns) * (10e-9);

        auto robot = robots[0];
        RobotStateMsgQueueEntry queue_elem{
            .x_pos_m               = robot.position().x(),
            .y_pos_m               = robot.position().y(),
            .yaw_rad               = robot.orientation().toRadians(),
            .x_vel_m_per_s         = robot.velocity().x(),
            .y_vel_m_per_s         = robot.velocity().y(),
            .angular_vel_rad_per_s = robot.angularVelocity().toRadians(),
            .timestamp_secs_since_epoch = current_time_since_epoch_secs
        };

        robot_state_message_queue.send(&queue_elem, sizeof(queue_elem), 0);
    }

    /**
     * An infinite loop that receives wheel speeds and pushes them to the robot
     */
    void receiveWheelSpeedsLoop()
    {
        while (!_in_destructor)
        {
            // We use the version of receive with a timeout so that we can
            // regularly check if this class was destructed, and stop the loop
            // if so
            const boost::posix_time::ptime t_timeout =
                boost::posix_time::microsec_clock::universal_time() +
                boost::posix_time::milliseconds(RECEIVE_WHEEL_SPEEDS_TIMEOUT_MS);
            RobotWheelCommandsMsgQueueEntry wheel_commands;
            size_t received_size  = 0;
            unsigned int priority = 0;

            // TODO
            // For some reason boost will only allow a minimum max message
            // size of 100, even though the queue is configured to have a 
            // max message size equal to the struct size. 
            const bool data_available = robot_wheel_commands_message_queue.timed_receive(
                &wheel_commands, 100, received_size, priority,
                t_timeout);

            if (data_available && received_size == sizeof(wheel_commands))
            {
                std::cout << "Received Commands: (" << wheel_commands << ")" << std::endl;
                // TODO: actually send these to the robot
            }
            else if (data_available && received_size != sizeof(wheel_commands))
            {
                LOG(WARNING) << "Received message of size " << received_size
                             << " which is different from the expected size "
                             << sizeof(wheel_commands);
            }
        }
    }

    // The message queue to push robot state updates to, and it's name
    boost::interprocess::message_queue robot_state_message_queue;
    const std::string robot_state_message_queue_name;

    // The message queue to read wheel commands from, and it's name
    boost::interprocess::message_queue robot_wheel_commands_message_queue;
    const std::string robot_wheel_commands_message_queue_name;

    std::atomic<bool> _in_destructor;

    std::thread receive_wheel_speeds_thread;
};

int main(int argc, char** argv)
{

    Util::Logger::LoggerSingleton::initializeLogger();

    auto world_observer =
        std::make_shared<RobotInterface>("robot_state", "robot_wheel_commands");

    std::unique_ptr<Backend> backend = std::make_unique<GrSimBackend>();

    backend->Subject<World>::registerObserver(world_observer);

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();

    return 0;
}
