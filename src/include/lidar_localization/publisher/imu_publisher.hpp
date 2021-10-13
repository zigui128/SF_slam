
#ifndef LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "lidar_localization/sensor_data/imu_data.hpp"

namespace lidar_localization {
class IMUPublisher {
  public:
    IMUPublisher(ros::NodeHandle& nh,
                 std::string topic_name,
                 std::string frame_id,
                 size_t buff_size);
    IMUPublisher() = default;

    void Publish(IMUData& imu_data, double time);
    void Publish(IMUData& imu_data);

    bool HasSubscribers();

private:
   void PublishData(IMUData& imu_data, ros::Time time);

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif
