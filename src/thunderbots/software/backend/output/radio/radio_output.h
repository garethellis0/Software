#pragma once

#include <ros/ros.h>

#include <limits>

#include "ai/world/ball.h"
#include "ai/world/team.h"
#include "mrf/dongle.h"
#include "thunderbots_msgs/RobotStatus.h"

class RadioOutput
{
   public:
    /**
     * Creates a new MRFBackend.
     * Automatically connects to the dongle upon initialization.
     *
     * @param config MRF configuration to start dongle in
     * @param node_handle the ROS NodeHandle of the radio_communication node
     */
    explicit RadioOutput(unsigned int config, ros::NodeHandle& node_handle);

    ~RadioOutput();

    /**
     * Sends the given primitives to the backend to control the robots
     *
     * @param primitives the list of primitives to send
     */
    void sendPrimitives(const std::vector<std::unique_ptr<Primitive>>& primitives);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_robots a vector of tuples of {robot id, robot location,
     *                        robot orientation}
     * @param ball
     */
    void send_vision_packet(std::vector<std::tuple<uint8_t, Point, Angle>> friendly_robots,
                            Ball ball);

    /**
     * Sends a camera packet with the detected robots and ball.
     *
     * @param friendly_team
     * @param ball
     */
    void send_vision_packet(const Team& friendly_team,
                            Ball ball);

   private:
    MRFDongle dongle;

    // The Annunciator that sends messages from the dongle to AI
    Annunciator annunciator;
};