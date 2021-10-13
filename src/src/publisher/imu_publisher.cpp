

#include "lidar_localization/publisher/imu_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization {
IMUPublisher::IMUPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_name, buff_size);
}


void IMUPublisher::Publish(IMUData& imu_data, double time) {
    ros::Time ros_time((float)time);
    PublishData(imu_data, ros_time);
}

void IMUPublisher::Publish(IMUData& imu_data) {
    ros::Time time = ros::Time::now();
    PublishData(imu_data, time);
}

void IMUPublisher::PublishData(IMUData& imu_data, ros::Time time) {
    sensor_msgs::Imu imu_output_;
    imu_output_.header.stamp = time;
    imu_output_.header.frame_id = frame_id_;
    imu_output_.angular_velocity.x = imu_data.angular_velocity.x;
    imu_output_.angular_velocity.y = imu_data.angular_velocity.y;
    imu_output_.angular_velocity.z = imu_data.angular_velocity.z;
    imu_output_.linear_acceleration.x = imu_data.linear_acceleration.x;
    imu_output_.linear_acceleration.y = imu_data.linear_acceleration.y;
    imu_output_.linear_acceleration.z = imu_data.linear_acceleration.z;
    imu_output_.orientation.w = imu_data.orientation.w;
    imu_output_.orientation.x = imu_data.orientation.x;
    imu_output_.orientation.y = imu_data.orientation.y;
    imu_output_.orientation.z = imu_data.orientation.z;

    publisher_.publish(imu_output_);
}

bool IMUPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization
