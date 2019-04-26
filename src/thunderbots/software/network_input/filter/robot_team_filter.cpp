#include "network_input/filter/robot_team_filter.h"

#include <algorithm>
#include <cmath>
#include <vector>


RobotTeamFilter::RobotTeamFilter() {}

Team RobotTeamFilter::getFilteredData(
    const Team &current_team_state,
    const std::vector<SSLRobotDetection> &new_robot_detections)
{
    for(auto d : new_robot_detections) {
        // Add filters for any robot we haven't seen before
        if(robot_filters.find(d.id) == robot_filters.end()) {
            robot_filters.insert({d.id, RobotFilter(d.id)});
        }
    }

    std::vector<FilteredRobotData> new_filtered_robot_data;
    for(auto it = robot_filters.begin(); it != robot_filters.end(); it++) {
        auto data = it->second.getFilteredData(new_robot_detections);
        if(data) {
            new_filtered_robot_data.emplace_back(*data);
        }
    }

    std::vector<Robot> new_robots;
    for(auto nfrd : new_filtered_robot_data) {
        new_robots.emplace_back(
                Robot(nfrd.id, nfrd.position, nfrd.velocity, nfrd.orientation, nfrd.angular_velocity, nfrd.timestamp));
    }

    Team new_team_state = current_team_state;
    new_team_state.updateRobots(new_robots);

    return new_team_state;
}
