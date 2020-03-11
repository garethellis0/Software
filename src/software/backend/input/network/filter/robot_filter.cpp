#include "software/backend/input/network/filter/robot_filter.h"

RobotFilter::RobotFilter(Robot current_robot_state, Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state),
      expiry_buffer_duration(expiry_buffer_duration),
      orientation_history(10)
{
}

RobotFilter::RobotFilter(RobotDetection current_robot_state,
                         Duration expiry_buffer_duration)
    : current_robot_state(current_robot_state.id, current_robot_state.position,
                          Vector(0, 0), current_robot_state.orientation,
                          AngularVelocity::zero(), current_robot_state.timestamp),
      expiry_buffer_duration(expiry_buffer_duration),
      // TODO: pass this size into the constructor?
      orientation_history(10)
{
}

std::optional<Robot> RobotFilter::getFilteredData(
    const std::vector<RobotDetection> &new_robot_data)
{
    int data_num               = 0;
    Timestamp latest_timestamp = Timestamp().fromSeconds(0);
    FilteredRobotData filtered_data;

    filtered_data.id               = this->getRobotId();
    filtered_data.position         = Point(0, 0);
    filtered_data.velocity         = Vector(0, 0);
    filtered_data.orientation      = Angle::fromRadians(0);
    filtered_data.angular_velocity = AngularVelocity::fromRadians(0);
    filtered_data.timestamp        = Timestamp().fromSeconds(0);

    for (const RobotDetection &robot_data : new_robot_data)
    {
        // add up all data points for this robot and then average it
        if (robot_data.id == this->getRobotId() &&
            robot_data.timestamp > this->current_robot_state.lastUpdateTimestamp())
        {
            filtered_data.position =
                filtered_data.position + robot_data.position.toVector();
            filtered_data.orientation =
                filtered_data.orientation + robot_data.orientation;

            orientation_history.push_front(std::pair<Angle, Timestamp>(
                robot_data.orientation, robot_data.timestamp));

            filtered_data.timestamp = filtered_data.timestamp.fromMilliseconds(
                filtered_data.timestamp.getMilliseconds() +
                robot_data.timestamp.getMilliseconds());
            data_num++;
        }

        // to get the latest timestamp of all data points in case there is no data for
        // this robot id
        if (latest_timestamp.getMilliseconds() < robot_data.timestamp.getMilliseconds())
        {
            latest_timestamp = robot_data.timestamp;
        }
    }

    if (data_num == 0)
    {
        // if there is no data the duration of expiry_buffer_duration after previously
        // recorded robot state, return null. Otherwise remain the same state
        if (latest_timestamp.getMilliseconds() >
            this->expiry_buffer_duration.getMilliseconds() +
                current_robot_state.lastUpdateTimestamp().getMilliseconds())
        {
            return std::nullopt;
        }
        else
        {
            return std::make_optional(current_robot_state);
        }
    }
    else
    {
        // update data by returning filtered robot data
        filtered_data.position    = Point(filtered_data.position.toVector() / data_num);
        filtered_data.orientation = filtered_data.orientation / data_num;

        filtered_data.timestamp = filtered_data.timestamp.fromMilliseconds(
            filtered_data.timestamp.getMilliseconds() / data_num);

        // velocity = position difference / time difference
        filtered_data.velocity =
            (filtered_data.position - current_robot_state.position()) /
            (filtered_data.timestamp.getSeconds() -
             current_robot_state.lastUpdateTimestamp().getSeconds());

        // angular_velocity = orientation difference / time difference
        // filtered_data.angular_velocity =
        //    (filtered_data.orientation - current_robot_state.orientation()) /
        //    (filtered_data.timestamp.getSeconds() -
        //     current_robot_state.lastUpdateTimestamp().getSeconds());

        // TODO: better naming
        double group_1_orientation_sum = 0;
        double group_2_orientation_sum = 0;
        double group_1_timestamp_sum   = 0;
        double group_2_timestamp_sum   = 0;
        size_t group_1_size            = std::round(orientation_history.size() / 2.0);
        size_t group_2_size            = orientation_history.size() - group_1_size;
        for (size_t i = 0; i < orientation_history.size(); i++)
        {
            if (i < group_1_size)
            {
                group_1_orientation_sum += orientation_history[i].first.toRadians();
                group_1_timestamp_sum += orientation_history[i].second.getSeconds();
            }
            else
            {
                group_2_orientation_sum += orientation_history[i].first.toRadians();
                group_2_timestamp_sum += orientation_history[i].second.getSeconds();
            }
        }

        double group_1_avg_orientation = group_1_orientation_sum / group_1_size;
        double group_1_avg_timestamp   = group_1_timestamp_sum / group_1_size;
        double group_2_avg_orientation = group_2_orientation_sum / group_2_size;
        double group_2_avg_timestamp   = group_2_timestamp_sum / group_2_size;

        filtered_data.angular_velocity =
            Angle::fromRadians((group_2_avg_orientation - group_1_avg_orientation) /
                               (group_2_avg_timestamp - group_1_avg_timestamp));

        // update current_robot_state
        this->current_robot_state =
            Robot(this->getRobotId(), filtered_data.position, filtered_data.velocity,
                  filtered_data.orientation, filtered_data.angular_velocity,
                  filtered_data.timestamp);

        return std::make_optional(this->current_robot_state);
    }
}

unsigned int RobotFilter::getRobotId() const
{
    return this->current_robot_state.id();
}
