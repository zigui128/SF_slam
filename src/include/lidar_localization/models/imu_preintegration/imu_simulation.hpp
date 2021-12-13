#ifndef LIDAR_LOCALIZATION_MODELS_IMUSIMULATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_IMUSIMULATION_HPP_

#include <eigen3/Eigen/Core>

using namespace Eigen;
using namespace std;

namespace lidar_localization {
struct MotionData
{

    double timestamp;
    Eigen::Matrix3f Rwb;
    Eigen::Vector3f twb;
    Eigen::Vector3f imu_acc;
    Eigen::Vector3f imu_gyro;

    Eigen::Vector3f imu_gyro_bias;
    Eigen::Vector3f imu_acc_bias;

    Eigen::Vector3f imu_velocity;
};

// euler2Rotation:   body frame to interitail frame
Eigen::Matrix3f euler2Rotation( Eigen::Vector3f  eulerAngles);
Eigen::Matrix3f eulerRates2bodyRates(Eigen::Vector3f eulerAngles);


class IMU
{
public:

    MotionData MotionModel(double t);
    
};
}


#endif
