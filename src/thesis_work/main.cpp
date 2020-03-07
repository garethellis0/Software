#include <boost/interprocess/ipc/message_queue.hpp>
#include <chrono>
#include <g3log/g3log.hpp>
#include <iostream>
#include <ostream>

#include "software/ai/primitive/direct_wheels_primitive.h"
#include "software/backend/backend.h"
#include "software/backend/grsim_backend.h"
#include "software/backend/radio_backend.h"
#include "software/logger/init.h"

// These structs MUST be kept exactly in sync with the ones
// in the autorally code, or extremely nasty things will happen.
// Do *NOT* using floating point values in these, as
// boost::interprocess:message_queue seems to serialize/deserilize these
// differently between different versions
// TODO: put this in a shared header somewhere
struct RobotStateMsgQueueEntry
{
    int32_t x_pos_mm;
    int32_t y_pos_mm;
    int32_t yaw_milli_rad;

    int32_t x_vel_mm_per_s;
    int32_t y_vel_mm_per_s;
    int32_t angular_vel_milli_rad_per_s;

    // We represent the timestamp in two parts, with the final timestamp
    // being the sum of the two
    int32_t timestamp_secs;
    int32_t timestamp_nano_secs_correction;

    /**
     * Set the timestamp variables from a time given in seconds
     */
    void setTimestampFromSecs(double t_secs)
    {
        timestamp_secs                 = std::floor(t_secs);
        timestamp_nano_secs_correction = std::floor((t_secs - timestamp_secs) * 1e9);
    }
};
inline std::ostream& operator<<(std::ostream& o, const RobotStateMsgQueueEntry& state)
{
    // clang-format off
    o << "x_pos_mm: " << state.x_pos_mm 
      << ", y_pos_mm: " << state.y_pos_mm 
      << ", yaw_milli_rad: " << state.yaw_milli_rad 
      << ", x_vel_mm_per_s: " << state.x_vel_mm_per_s 
      << ", y_vel_mm_per_s: " << state.y_vel_mm_per_s
      << ", angular_vel_milli_rad_per_s: " << state.angular_vel_milli_rad_per_s
      << ", timestamp_secs: " << state.timestamp_secs
      << ", timestamp_nano_secs_correction: " << state.timestamp_nano_secs_correction;
    // clang-format on
    return o;
}
struct RobotWheelCommandsMsgQueueEntry
{
    int32_t front_right_milli_newton;
    int32_t front_left_milli_newton;
    int32_t back_right_milli_newton;
    int32_t back_left_milli_newton;

    // We represent the timestamp in two parts, with the final timestamp
    // being the sum of the two
    int32_t timestamp_secs;
    int32_t timestamp_nano_secs_correction;
};
inline std::ostream& operator<<(std::ostream& o,
                                const RobotWheelCommandsMsgQueueEntry& cmds)
{
    // clang-format off
    o << "Front Right: "                      << cmds.front_right_milli_newton 
      << ", Front Left: "                     << cmds.front_left_milli_newton
      << ", Back Right: "                     << cmds.back_right_milli_newton 
      << ", Back Left: "                      << cmds.back_left_milli_newton
      << ", timestamp_secs: "                 << cmds.timestamp_secs
      << ", timestamp_nano_secs_correction: " << cmds.timestamp_nano_secs_correction;
    // clang-format on
    return o;
}

// TODO: put this in another file (.c and .h)
// TODO: better name for this class
class RobotInterface : public ThreadedObserver<World>,
                       public Subject<ConstPrimitiveVectorPtr>
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
     * @param robot_id The ID of the robot to control
     *
     */
    RobotInterface(std::string robot_state_ipc_queue_name,
                   std::string robot_wheel_commands_ipc_queue_name, unsigned int robot_id,
                   float max_wheel_force_centi_newtons)
        : robot_state_message_queue_name(robot_state_ipc_queue_name),
          robot_wheel_commands_message_queue_name(robot_wheel_commands_ipc_queue_name),
          robot_state_message_queue(boost::interprocess::open_or_create,
                                    robot_state_ipc_queue_name.c_str(), MAX_QUEUE_SIZE,
                                    sizeof(RobotStateMsgQueueEntry)),
          robot_wheel_commands_message_queue(boost::interprocess::open_or_create,
                                             robot_wheel_commands_ipc_queue_name.c_str(),
                                             MAX_QUEUE_SIZE,
                                             sizeof(RobotWheelCommandsMsgQueueEntry)),
          _robot_id(robot_id),
          _max_abs_wheel_force_centi_newtons(abs(max_wheel_force_centi_newtons)),
          _in_destructor(false)
    {
        receive_wheel_forces_thread =
            std::thread([this]() { return receiveWheelforcesLoop(); });
    }

    ~RobotInterface()
    {
        // Stop the thread that's constantly pulling robot wheel_commands
        _in_destructor = true;
        receive_wheel_forces_thread.join();


        // Close message queues
        boost::interprocess::message_queue::remove(
            robot_state_message_queue_name.c_str());
        boost::interprocess::message_queue::remove(
            robot_wheel_commands_message_queue_name.c_str());
    }

   private:
    static const int MAX_QUEUE_SIZE = 2;

    const int IPC_QUEUE_TIMEOUT_MS = 100;

    /**
     * This function will be called with new worlds when they are received
     * @param world
     */
    void onValueReceived(World world) override
    {
        auto friendly_team = world.friendlyTeam();
        auto robot_opt     = friendly_team.getRobotById(_robot_id);
        if (!robot_opt)
        {
            LOG(WARNING) << "Robot with id " << _robot_id << " not visible.";
            return;
        }
        auto robot = *robot_opt;

        // TODO: use actual robot position
        RobotStateMsgQueueEntry queue_elem{
            //.x_pos_mm       = std::round(robot.position().x() * 1000),
            .x_pos_mm = (int32_t)std::round(-1 * 1000),
            //.y_pos_mm       = std::round(robot.position().y() * 1000),
            .y_pos_mm = (int32_t)std::round(-9.5 * 1000),
            //.yaw_milli_rad  = (int32_t)std::round(robot.orientation().toRadians() *
            // 1000),
            .yaw_milli_rad  = (int32_t)std::round(M_PI / 2 * 1000),
            .x_vel_mm_per_s = (int32_t)std::round(robot.velocity().x() * 1000),
            .y_vel_mm_per_s = (int32_t)std::round(robot.velocity().y() * 1000),
            .angular_vel_milli_rad_per_s =
                (int32_t)std::round(robot.angularVelocity().toRadians() * 1000),
        };

        // Because of how the the MPC controller works, we use the system
        // time now instead of the timestamp on the received data.
        // TODO: redesign MPC stuff to properly use relative timestamps,
        //       or somehow offset this timestamp from the start time
        double curr_time_since_epoch_seconds =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count() *
            1e-9;
        queue_elem.setTimestampFromSecs(curr_time_since_epoch_seconds);

        LOG(INFO) << "Transmitting State: " << queue_elem << std::endl;

        const boost::posix_time::ptime t_timeout =
            boost::posix_time::microsec_clock::universal_time() +
            boost::posix_time::milliseconds(IPC_QUEUE_TIMEOUT_MS);
        bool send_succeeded = robot_state_message_queue.timed_send(
            &queue_elem, sizeof(queue_elem), 0, t_timeout);
        if (!send_succeeded)
        {
            LOG(WARNING) << "Failed to send robot state, queue probably full!";
        }
    }

    /**
     * An infinite loop that receives wheel forces and pushes them to the robot
     */
    void receiveWheelforcesLoop()
    {
        while (!_in_destructor)
        {
            // We use the version of receive with a timeout so that we can
            // regularly check if this class was destructed, and stop the loop
            // if so
            const boost::posix_time::ptime t_timeout =
                boost::posix_time::microsec_clock::universal_time() +
                boost::posix_time::milliseconds(IPC_QUEUE_TIMEOUT_MS);
            RobotWheelCommandsMsgQueueEntry wheel_commands;
            size_t received_size  = 0;
            unsigned int priority = 0;

            // TODO
            // For some reason boost will only allow a minimum max message
            // size of 100, even though the queue is configured to have a
            // max message size equal to the struct size.
            const bool data_available = robot_wheel_commands_message_queue.timed_receive(
                &wheel_commands, 100, received_size, priority, t_timeout);

            if (data_available && received_size == sizeof(wheel_commands))
            {
                std::cout << "Received Commands: (" << wheel_commands << ")" << std::endl;

                auto convert_wheel_force = [this](int32_t milli_newton) -> int16_t {
                    float force_centi_newtons = ((float)milli_newton) / 10.0;
                    float clamped_force_centi_newtons =
                        std::min(std::max(force_centi_newtons,
                                          -_max_abs_wheel_force_centi_newtons),
                                 _max_abs_wheel_force_centi_newtons);

                    return static_cast<int16_t>(clamped_force_centi_newtons);
                };

                // TODO: sanity checks to prevent aggressive wheel speeds

                // The direct wheels primitive forces are values in [-255, 255]
                // that are divided by 100 on the robots to get the force
                // to apply to each wheel, so we send centi-newtons to the
                // robot
                auto direct_wheels_primitive = std::make_unique<DirectWheelsPrimitive>(
                    _robot_id,
                    convert_wheel_force(wheel_commands.front_left_milli_newton),
                    convert_wheel_force(wheel_commands.back_left_milli_newton),
                    convert_wheel_force(wheel_commands.front_right_milli_newton),
                    convert_wheel_force(wheel_commands.back_right_milli_newton), 0);

                LOG(INFO) << "Sending Values Over Radio ( "
                          << direct_wheels_primitive->getWheel0Power() << ", "
                          << direct_wheels_primitive->getWheel1Power() << ", "
                          << direct_wheels_primitive->getWheel2Power() << ", "
                          << direct_wheels_primitive->getWheel3Power() << " )";

                std::vector<std::unique_ptr<Primitive>> primitives;
                primitives.emplace_back(std::move(direct_wheels_primitive));

                auto primitives_ptr =
                    std::make_shared<const std::vector<std::unique_ptr<Primitive>>>(
                        std::move(primitives));
                Subject<ConstPrimitiveVectorPtr>::sendValueToObservers(primitives_ptr);
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

    // The thread on which to run the infinite loop the receives the wheel
    // forces to run
    std::thread receive_wheel_forces_thread;

    // The id of the robot we're controlling
    unsigned int _robot_id;

    // The max wheel force to apply (per-wheel)
    float _max_abs_wheel_force_centi_newtons;
};

int main(int argc, char** argv)
{
    Util::Logger::LoggerSingleton::initializeLogger();

    auto world_observer =
        std::make_shared<RobotInterface>("robot_state", "robot_wheel_commands", 3, 10);

    std::shared_ptr<Backend> backend = std::make_unique<RadioBackend>();

    backend->Subject<World>::registerObserver(world_observer);
    world_observer->Subject<ConstPrimitiveVectorPtr>::registerObserver(backend);

    // This blocks forever without using the CPU
    std::promise<void>().get_future().wait();

    return 0;
}
