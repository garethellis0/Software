#include "network_input/filter/robot_filter.h"

RobotFilter::RobotFilter(unsigned int id) : robot_id(id), previous_states(5) {}

std::optional<FilteredRobotData> RobotFilter::getFilteredData(
        const std::vector<SSLRobotDetection> &new_robot_data)
{
    std::vector<SSLRobotDetection> new_data;
    Timestamp latest_robot_timestamp = Timestamp::fromSeconds(0);
    for(auto detection : new_robot_data) {
        if(detection.id == robot_id) {
            new_data.emplace_back(detection);
            if(detection.timestamp > latest_robot_timestamp) {
                latest_robot_timestamp = detection.timestamp;
            }
        }
    }

    SSLRobotDetection average_detection;
    average_detection.id = robot_id;
    average_detection.position = Point(0, 0);
    average_detection.orientation = Angle::zero();

    for(auto d : new_data) {
        average_detection.position = average_detection.position + d.position;
        average_detection.orientation = average_detection.orientation + d.orientation;
    }

    average_detection.position = average_detection.position.norm(average_detection.position.len() / new_data.size());
    average_detection.orientation = average_detection.orientation = average_detection.orientation / new_data.size();

    // average the new data
    FilteredRobotData new_filtered_data;
    new_filtered_data.id = average_detection.id;
    new_filtered_data.position = average_detection.position;
    new_filtered_data.velocity = Vector(0, 0);
    new_filtered_data.orientation = average_detection.orientation;
    new_filtered_data.angular_velocity = AngularVelocity::zero();
    new_filtered_data.timestamp = latest_robot_timestamp;

    if(!previous_states.empty()) {
        Duration time_diff = new_filtered_data.timestamp - previous_states.at(0).timestamp;

        auto velocity_vector = new_filtered_data.position - previous_states.at(0).position;
        auto velocity_length = velocity_vector.len() / time_diff.getSeconds();
        new_filtered_data.velocity = velocity_vector.norm(velocity_length);

        auto angular_velocity = (new_filtered_data.orientation - previous_states.at(0).orientation) / time_diff.getSeconds();
        new_filtered_data.angular_velocity = angular_velocity;
    }

    if(new_data.empty() && previous_states.empty()) {
        return std::nullopt;
    }
    if(!new_data.empty()) {
        previous_states.push_front(new_filtered_data);
    }

    FilteredRobotData average_state;
    average_state.id = robot_id;
    average_state.position = Point(0,0);
    average_state.orientation = Angle::zero();
    average_state.velocity = Vector(0, 0);
    average_state.angular_velocity = AngularVelocity::zero();
    average_state.timestamp = previous_states.at(0).timestamp;

    for(auto s : previous_states) {
        average_state.position = average_state.position + s.position;
        average_state.velocity = average_state.velocity + s.velocity;
        average_state.orientation = average_state.orientation + s.orientation;
        average_state.angular_velocity = average_state.angular_velocity + s.angular_velocity;
    }

    average_state.position = average_state.position.norm(average_state.position.len() / previous_states.size());
    average_state.velocity = average_state.velocity.norm(average_state.velocity.len() / previous_states.size());
    average_state.orientation = average_state.orientation = average_state.orientation / previous_states.size();
    average_state.angular_velocity = average_state.angular_velocity = average_state.angular_velocity / previous_states.size();

    return average_state;
}

unsigned int RobotFilter::getRobotId() const
{
    return robot_id;
}
